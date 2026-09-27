//! `xgc_rt_nx.h`, the NeighborExchange for C and C++ planners, is the Rust
//! `xgc_rt_abi::neighbor::NeighborExchange` contract: a seeded script of
//! offers (neighbors and strangers, past, current and future rounds, older,
//! duplicate and newer seqs) and snapshots runs through both, compiled as
//! C11 and as C++17, and every admission result, every neighbor's status,
//! stale count, round, age and plan bytes, and the encoded snapshot record
//! agree.

mod common;

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use xgc_rt_abi::neighbor::{NeighborExchange, NeighborStatus};

fn build(compiler: &str, flags: &[&str], out: &Path) -> PathBuf {
    let root = common::workspace_root();
    let status = Command::new(compiler)
        .args(flags)
        .args(["-O2", "-Wall", "-Wextra", "-Werror", "-I"])
        .arg(root.join("abi/include"))
        .arg(root.join("crates/xgc-rt-host/tests/nx/nx_script.c"))
        .arg("-o")
        .arg(out)
        .status()
        .unwrap();
    assert!(status.success(), "{compiler} failed");
    out.to_path_buf()
}

/// splitmix64
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn status_code(s: NeighborStatus) -> (u8, u64) {
    match s {
        NeighborStatus::Fresh => (0, 0),
        NeighborStatus::Stale(n) => (1, n),
        NeighborStatus::Missing => (2, 0),
    }
}

/// The script and the Rust side's expected output.
fn script(seed: u64) -> (String, Vec<String>) {
    let mut rng = Rng(seed);
    let neighbors: Vec<u16> = vec![1, 3, 4, 7];
    let s_max = rng.below(4);
    let mut nx = NeighborExchange::with_neighbors(neighbors.clone(), 0, 1, s_max);
    let mut input = format!("i {} {} {s_max}\n", neighbors.len(), neighbors.iter().map(|n| n.to_string()).collect::<Vec<_>>().join(" "));
    let mut expected = Vec::new();
    let mut k = 0u64;
    for _ in 0..600 {
        match rng.below(10) {
            0..=6 => {
                let origin = [1u16, 3, 4, 7, 2, 9][rng.below(6) as usize];
                let round = if rng.below(20) == 0 { k + 1 + rng.below(3) } else { k.saturating_sub(rng.below(6)) };
                let seq = 1 + rng.below(40);
                let t = rng.below(1_000_000) as i64;
                let len = rng.below(64) as usize;
                let data = vec![(seq & 0xff) as u8; len];
                input += &format!("o {k} {origin} {round} {seq} {t} {len}\n");
                expected.push(format!("o {}", i32::from(nx.offer(k, origin, round, seq, t, &data))));
            }
            7 => k += 1 + rng.below(2),
            8 => {
                let (sk, now) = (k.saturating_sub(rng.below(2)) + rng.below(2), 1_000_000 + rng.below(1_000) as i64);
                input += &format!("s {sk} {now}\n");
                for v in nx.snapshot(sk, now).neighbors {
                    let (status, stale) = status_code(v.status);
                    expected.push(format!(
                        "v {} {status} {stale} {} {} {} {}",
                        v.origin,
                        v.round.unwrap_or(u64::MAX),
                        v.age_ns.unwrap_or(0),
                        v.data.len(),
                        v.data.first().map_or(-1, |b| i32::from(*b))
                    ));
                }
            }
            _ => {
                let now = 2_000_000 + rng.below(1_000) as i64;
                input += &format!("e {k} {now}\n");
                let snap = nx.snapshot(k, now);
                let mut record = Vec::new();
                record.extend(k.to_le_bytes());
                record.extend((snap.neighbors.len() as u32).to_le_bytes());
                record.extend(0u32.to_le_bytes());
                for v in &snap.neighbors {
                    let (status, stale) = status_code(v.status);
                    record.extend(v.origin.to_le_bytes());
                    record.push(status);
                    record.extend([0u8; 5]);
                    record.extend(stale.to_le_bytes());
                    record.extend(v.round.unwrap_or(u64::MAX).to_le_bytes());
                    record.extend(v.age_ns.unwrap_or(0).to_le_bytes());
                }
                expected.push(format!("e {}", record.iter().map(|b| format!("{b:02x}")).collect::<String>()));
            }
        }
    }
    (input, expected)
}

fn run(program: &Path, input: &str) -> Vec<String> {
    let mut child = Command::new(program).stdin(Stdio::piped()).stdout(Stdio::piped()).spawn().unwrap();
    child.stdin.take().unwrap().write_all(input.as_bytes()).unwrap();
    let out = child.wait_with_output().unwrap();
    assert!(out.status.success(), "{} exited {:?}", program.display(), out.status);
    String::from_utf8(out.stdout).unwrap().lines().map(str::to_owned).collect()
}

#[test]
fn the_c_neighbor_exchange_is_the_rust_one() {
    let dir = common::scratch("nx-c");
    let c = build(&std::env::var("CC").unwrap_or_else(|_| "cc".into()), &["-std=c11", "-x", "c"], &dir.join("nx_c"));
    let cxx = build(&std::env::var("CXX").unwrap_or_else(|_| "c++".into()), &["-std=c++17", "-x", "c++"], &dir.join("nx_cxx"));
    let mut lines = 0;
    for seed in 0..40 {
        let (input, expected) = script(seed);
        for program in [&c, &cxx] {
            let got = run(program, &input);
            assert_eq!(got.len(), expected.len(), "seed {seed} {}: line count", program.display());
            for (i, (g, e)) in got.iter().zip(&expected).enumerate() {
                assert_eq!(g, e, "seed {seed} {}: output line {i}", program.display());
            }
        }
        lines += expected.len();
    }
    println!("xgc_rt_nx.h == NeighborExchange: 40 seeded scripts, {lines} results, C11 and C++17");
}
