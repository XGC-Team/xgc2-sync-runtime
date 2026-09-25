//! Offline merge of every node's records into per-stream metrics
//! (`audit-def/1`, docs/audit-definitions.md). Claims cite this merge, never
//! live estimates.
//!
//! A *stream* is `(channel, origin → receiver)` and exists from the
//! receiver's subscribe record on. Its **expected set** is every seq the
//! origin sent on that channel with `t_tx ≥ t_subscribed`. For each rx record
//! of the stream, in arrival (record) order:
//! 1. a seq the origin never sent is *phantom*, an integrity error;
//! 2. a seq already seen is a *duplicate*;
//! 3. a first arrival with `t_rx − t_tx > grace` is *late beyond grace* and
//!    counts as lost;
//! 4. otherwise it is *received*. It is *reordered* when its seq is below the
//!    highest seq already received on the stream (RFC 4737 §3.3), with extent
//!    `max_seen − seq`.
//!
//! Then `lost = |expected \ received|`, where received counts only expected
//! seqs.

use std::collections::{BTreeMap, HashMap};
use std::fs;
use std::path::Path;

use serde::Serialize;

use crate::record::{Kind, Record, RECORD_LEN};
use crate::recorder::NodeMeta;

pub const DEFINITIONS: &str = "audit-def/1";

#[derive(Debug, Clone, Copy)]
pub struct MergeOptions {
    pub grace_ns: i64,
    pub window_ns: i64,
}

impl Default for MergeOptions {
    fn default() -> Self {
        Self { grace_ns: 1_000_000_000, window_ns: 1_000_000_000 }
    }
}

#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct Dist {
    pub count: u64,
    pub min: i64,
    pub p50: i64,
    pub p90: i64,
    pub p99: i64,
    pub max: i64,
    pub mean: f64,
}

impl Dist {
    /// Nearest-rank percentiles. It sorts `v` in place.
    pub fn of(v: &mut [i64]) -> Self {
        if v.is_empty() {
            return Self::default();
        }
        v.sort_unstable();
        let rank = |p: f64| v[((p / 100.0 * v.len() as f64).ceil() as usize).clamp(1, v.len()) - 1];
        Self {
            count: v.len() as u64,
            min: v[0],
            p50: rank(50.0),
            p90: rank(90.0),
            p99: rank(99.0),
            max: v[v.len() - 1],
            mean: v.iter().map(|&x| x as f64).sum::<f64>() / v.len() as f64,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct StreamCounts {
    pub expected: u64,
    pub received: u64,
    pub lost: u64,
    pub loss_ratio: f64,
    pub late_beyond_grace: u64,
    pub duplicates: u64,
    pub reordered: u64,
    pub reorder_ratio: f64,
    pub reorder_extent_max: u64,
    pub payload_bytes_received: u64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct StreamReport {
    pub channel: String,
    pub origin: String,
    pub receiver: String,
    #[serde(flatten)]
    pub counts: StreamCounts,
    pub reorder_extent: Dist,
    /// One-way delay `t_rx − t_tx` of received samples, in ns.
    pub owd_ns: Dist,
    /// Per-sample clock-error bound (sender + receiver), in ns. An OWD figure
    /// is only as good as `owd_bound_ns.max`.
    pub owd_bound_ns: Dist,
    /// `t_consume − t_produce` for samples a module read.
    pub age_at_use_ns: Dist,
    /// Span of expected `t_tx`, extended by one mean send interval:
    /// `(t_last − t_first) · n / (n − 1)`. All rates divide by this.
    pub duration_s: f64,
    pub rate_hz: f64,
    /// Payload the origin sent on the stream (expected set), per second.
    pub offered_payload_mbps: f64,
    /// Payload received on the stream, per second.
    pub throughput_payload_mbps: f64,
    /// Payload plus the 64-byte envelope. The transport's own overhead is not
    /// included (see `ifstats` from Z2 on).
    pub throughput_envelope_mbps: f64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct WindowReport {
    pub channel: String,
    pub origin: String,
    pub receiver: String,
    /// Window index `floor((t_tx − t0) / window)`, where t0 is the run's
    /// first expected `t_tx`.
    pub window: u64,
    pub t_start_ns: i64,
    #[serde(flatten)]
    pub counts: StreamCounts,
    pub owd_p50_ns: i64,
    pub owd_p99_ns: i64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct NodeReport {
    pub node: String,
    pub records: u64,
    pub sent: u64,
    pub rejected_frames: u64,
    pub rx_queue_overflows: u64,
    pub inbox_overflows: u64,
    pub audit_queue_drops: u64,
    pub complete: bool,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Report {
    pub definitions: &'static str,
    pub session: String,
    pub roster: Vec<String>,
    pub channels: Vec<String>,
    pub grace_ns: i64,
    pub window_ns: i64,
    /// False when any evidence is incomplete. `invalid_reasons` says why.
    pub valid: bool,
    pub invalid_reasons: Vec<String>,
    pub nodes: Vec<NodeReport>,
    pub streams: Vec<StreamReport>,
    #[serde(skip)]
    pub windows: Vec<WindowReport>,
}

#[derive(Debug)]
pub struct MergeError(pub String);

impl std::fmt::Display for MergeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for MergeError {}

struct Tx {
    t_tx: i64,
    len: u32,
}

struct NodeData {
    meta: NodeMeta,
    records: Vec<Record>,
}

fn load_run(run_dir: &Path) -> Result<Vec<NodeData>, MergeError> {
    let err = |m: String| MergeError(m);
    let mut nodes = Vec::new();
    let entries = fs::read_dir(run_dir).map_err(|e| err(format!("{}: {e}", run_dir.display())))?;
    let mut dirs: Vec<_> = entries.filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.join("meta.json").is_file()).collect();
    dirs.sort();
    for dir in dirs {
        let meta: NodeMeta = serde_json::from_slice(&fs::read(dir.join("meta.json")).map_err(|e| err(e.to_string()))?)
            .map_err(|e| err(format!("{}: {e}", dir.display())))?;
        if meta.format != crate::record::FORMAT {
            return Err(err(format!("{}: unsupported format {}", dir.display(), meta.format)));
        }
        let bytes = fs::read(dir.join("records.bin")).map_err(|e| err(format!("{}: {e}", dir.display())))?;
        if bytes.len() % RECORD_LEN != 0 {
            return Err(err(format!("{}: records.bin is truncated", dir.display())));
        }
        let mut records = Vec::with_capacity(bytes.len() / RECORD_LEN);
        for chunk in bytes.chunks_exact(RECORD_LEN) {
            records.push(Record::decode(chunk).ok_or_else(|| err(format!("{}: bad record", dir.display())))?);
        }
        nodes.push(NodeData { meta, records });
    }
    if nodes.is_empty() {
        return Err(err(format!("{}: no node records", run_dir.display())));
    }
    Ok(nodes)
}

pub fn merge_run(run_dir: &Path, opts: MergeOptions) -> Result<Report, MergeError> {
    let nodes = load_run(run_dir)?;
    let first = &nodes[0].meta;
    let mut invalid = Vec::new();
    for n in &nodes {
        if n.meta.session != first.session || n.meta.roster != first.roster || n.meta.channels != first.channels {
            return Err(MergeError(format!("node {} belongs to a different session layout", n.meta.node)));
        }
        if n.meta.roster.get(n.meta.node_id as usize) != Some(&n.meta.node) {
            return Err(MergeError(format!("node {} id {} does not match the roster", n.meta.node, n.meta.node_id)));
        }
        if !n.meta.complete {
            invalid.push(format!("node {} did not finish its audit (meta.complete=false)", n.meta.node));
        }
        if n.meta.audit_queue_drops > 0 {
            invalid.push(format!("node {} dropped {} audit records", n.meta.node, n.meta.audit_queue_drops));
        }
        if (n.records.len() as u64) != n.meta.records_written && n.meta.complete {
            invalid.push(format!("node {} records.bin has {} records, meta says {}", n.meta.node, n.records.len(), n.meta.records_written));
        }
    }
    let roster = first.roster.clone();
    let channels = first.channels.clone();
    let name = |v: &Vec<String>, i: usize| v.get(i).cloned().unwrap_or_else(|| format!("#{i}"));

    // Sender truth: (origin, channel) → seq → tx.
    let mut sent: HashMap<(u16, u32), HashMap<u64, Tx>> = HashMap::new();
    let mut node_reports = Vec::new();
    for n in &nodes {
        let mut report = NodeReport {
            node: n.meta.node.clone(),
            records: n.records.len() as u64,
            sent: 0,
            rejected_frames: 0,
            rx_queue_overflows: 0,
            inbox_overflows: 0,
            audit_queue_drops: n.meta.audit_queue_drops,
            complete: n.meta.complete,
        };
        for r in &n.records {
            match r.kind {
                Kind::Tx => {
                    if r.origin != n.meta.node_id {
                        invalid.push(format!("node {} logged a tx for origin {}", n.meta.node, r.origin));
                        continue;
                    }
                    report.sent += 1;
                    let seqs = sent.entry((r.origin, r.channel)).or_default();
                    if seqs.insert(r.seq, Tx { t_tx: r.t_b, len: r.len }).is_some() {
                        invalid.push(format!("node {} reused seq {} on channel {}", n.meta.node, r.seq, name(&channels, r.channel as usize)));
                    }
                }
                Kind::Reject => report.rejected_frames += 1,
                Kind::Overflow if r.flags == xgc_rt_core::audit::OverflowSite::RxQueue as u8 => report.rx_queue_overflows += 1,
                Kind::Overflow => report.inbox_overflows += 1,
                _ => {}
            }
        }
        if report.rx_queue_overflows > 0 {
            invalid.push(format!("node {} overflowed its receive queue {} times (receive-side loss)", report.node, report.rx_queue_overflows));
        }
        node_reports.push(report);
    }

    let t0 = sent.values().flat_map(|m| m.values().map(|t| t.t_tx)).min().unwrap_or(0);
    let mut streams = Vec::new();
    let mut windows = Vec::new();

    for n in &nodes {
        let receiver = n.meta.node_id;
        // Streams of this receiver, from subscribe records.
        let mut subscribed: BTreeMap<(u32, u16), i64> = BTreeMap::new();
        for r in n.records.iter().filter(|r| r.kind == Kind::Subscribe) {
            subscribed.entry((r.channel, r.origin)).or_insert(r.t_b);
        }
        let mut arrivals: HashMap<(u32, u16), Vec<&Record>> = HashMap::new();
        let mut consumed: HashMap<(u32, u16), HashMap<u64, i64>> = HashMap::new();
        for r in &n.records {
            match r.kind {
                Kind::Rx => arrivals.entry((r.channel, r.origin)).or_default().push(r),
                Kind::Consume => {
                    consumed.entry((r.channel, r.origin)).or_default().entry(r.seq).or_insert(r.t_b - r.t_a);
                }
                _ => {}
            }
        }
        for (&(channel, origin), &t_sub) in &subscribed {
            let empty = HashMap::new();
            let tx = sent.get(&(origin, channel)).unwrap_or(&empty);
            let expected: HashMap<u64, &Tx> = tx.iter().filter(|(_, t)| t.t_tx >= t_sub).map(|(&s, t)| (s, t)).collect();
            let window_of = |t_tx: i64| ((t_tx - t0).max(0) / opts.window_ns) as u64;

            let mut total = StreamCounts { expected: expected.len() as u64, ..Default::default() };
            let mut per_window: BTreeMap<u64, (StreamCounts, Vec<i64>)> = BTreeMap::new();
            for t in expected.values() {
                per_window.entry(window_of(t.t_tx)).or_default().0.expected += 1;
            }

            let mut seen: HashMap<u64, ()> = HashMap::with_capacity(expected.len());
            let mut received_ok: HashMap<u64, ()> = HashMap::with_capacity(expected.len());
            let mut max_seen: Option<u64> = None;
            let (mut owd, mut bound, mut extents) = (Vec::new(), Vec::new(), Vec::new());
            let mut phantom = 0u64;
            for r in arrivals.get(&(channel, origin)).map(Vec::as_slice).unwrap_or(&[]) {
                let Some(t) = tx.get(&r.seq) else {
                    phantom += 1;
                    continue;
                };
                let w = window_of(t.t_tx);
                if seen.insert(r.seq, ()).is_some() {
                    total.duplicates += 1;
                    per_window.entry(w).or_default().0.duplicates += 1;
                    continue;
                }
                if !expected.contains_key(&r.seq) {
                    continue; // sent before the subscription; not part of the stream
                }
                let d = r.t_b - r.t_a;
                if d > opts.grace_ns {
                    total.late_beyond_grace += 1;
                    per_window.entry(w).or_default().0.late_beyond_grace += 1;
                    continue;
                }
                received_ok.insert(r.seq, ());
                total.received += 1;
                total.payload_bytes_received += r.len as u64;
                let win = per_window.entry(w).or_default();
                win.0.received += 1;
                win.0.payload_bytes_received += r.len as u64;
                win.1.push(d);
                owd.push(d);
                bound.push(r.bound_a as i64 + r.bound_b as i64);
                match max_seen {
                    Some(m) if r.seq < m => {
                        let extent = m - r.seq;
                        total.reordered += 1;
                        total.reorder_extent_max = total.reorder_extent_max.max(extent);
                        win.0.reordered += 1;
                        win.0.reorder_extent_max = win.0.reorder_extent_max.max(extent);
                        extents.push(extent as i64);
                    }
                    _ => max_seen = Some(r.seq),
                }
            }
            if phantom > 0 {
                invalid.push(format!(
                    "{} received {phantom} samples on {} from {} that were never logged as sent",
                    n.meta.node, name(&channels, channel as usize), name(&roster, origin as usize)
                ));
            }
            for (&seq, t) in &expected {
                if !received_ok.contains_key(&seq) {
                    total.lost += 1;
                    per_window.entry(window_of(t.t_tx)).or_default().0.lost += 1;
                }
            }
            finish_ratios(&mut total);

            let mut ages: Vec<i64> = consumed
                .get(&(channel, origin))
                .map(|m| m.iter().filter(|(s, _)| received_ok.contains_key(s)).map(|(_, &a)| a).collect())
                .unwrap_or_default();
            let (first, last) = expected.values().fold((i64::MAX, i64::MIN), |(a, b), t| (a.min(t.t_tx), b.max(t.t_tx)));
            // The t_tx span of n samples holds n − 1 send intervals. Extending
            // it by one mean interval makes n samples at period P report 1/P.
            let n = expected.len() as f64;
            let duration_s = if expected.len() > 1 { (last - first) as f64 / 1e9 * n / (n - 1.0) } else { 0.0 };
            let per_s = |x: f64| if duration_s > 0.0 { x / duration_s } else { 0.0 };
            let (chan_name, origin_name, receiver_name) =
                (name(&channels, channel as usize), name(&roster, origin as usize), name(&roster, receiver as usize));
            streams.push(StreamReport {
                channel: chan_name.clone(),
                origin: origin_name.clone(),
                receiver: receiver_name.clone(),
                reorder_extent: Dist::of(&mut extents),
                owd_ns: Dist::of(&mut owd),
                owd_bound_ns: Dist::of(&mut bound),
                age_at_use_ns: Dist::of(&mut ages),
                duration_s,
                rate_hz: per_s(total.received as f64),
                offered_payload_mbps: per_s(expected.values().map(|t| t.len as f64 * 8.0).sum::<f64>()) / 1e6,
                throughput_payload_mbps: per_s(total.payload_bytes_received as f64 * 8.0) / 1e6,
                throughput_envelope_mbps: per_s(
                    (total.payload_bytes_received + total.received * xgc_rt_core::envelope::HEADER_LEN as u64) as f64 * 8.0,
                ) / 1e6,
                counts: total,
            });
            for (w, (mut counts, mut d)) in per_window {
                finish_ratios(&mut counts);
                let dist = Dist::of(&mut d);
                windows.push(WindowReport {
                    channel: chan_name.clone(),
                    origin: origin_name.clone(),
                    receiver: receiver_name.clone(),
                    window: w,
                    t_start_ns: t0 + w as i64 * opts.window_ns,
                    counts,
                    owd_p50_ns: dist.p50,
                    owd_p99_ns: dist.p99,
                });
            }
        }
    }
    streams.sort_by(|a, b| (&a.channel, &a.origin, &a.receiver).cmp(&(&b.channel, &b.origin, &b.receiver)));
    invalid.sort();
    invalid.dedup();
    Ok(Report {
        definitions: DEFINITIONS,
        session: first.session.clone(),
        roster,
        channels,
        grace_ns: opts.grace_ns,
        window_ns: opts.window_ns,
        valid: invalid.is_empty(),
        invalid_reasons: invalid,
        nodes: node_reports,
        streams,
        windows,
    })
}

fn finish_ratios(c: &mut StreamCounts) {
    c.loss_ratio = if c.expected > 0 { c.lost as f64 / c.expected as f64 } else { 0.0 };
    c.reorder_ratio = if c.received > 0 { c.reordered as f64 / c.received as f64 } else { 0.0 };
}

/// Write `summary.json`, `streams.jsonl` (one line per stream and window)
/// and `summary.md` into `out_dir`.
pub fn write_report(report: &Report, out_dir: &Path) -> std::io::Result<()> {
    fs::create_dir_all(out_dir)?;
    fs::write(out_dir.join("summary.json"), serde_json::to_string_pretty(report).map_err(std::io::Error::other)? + "\n")?;
    let mut lines = String::new();
    for w in &report.windows {
        lines.push_str(&serde_json::to_string(w).map_err(std::io::Error::other)?);
        lines.push('\n');
    }
    fs::write(out_dir.join("streams.jsonl"), lines)?;
    fs::write(out_dir.join("summary.md"), markdown(report))
}

fn ms(ns: i64) -> String {
    format!("{:.3}", ns as f64 / 1e6)
}

pub fn markdown(r: &Report) -> String {
    let mut s = format!(
        "# Sync audit — session `{}`\n\nDefinitions `{}` · grace {} ms · window {} ms · **{}**\n\n",
        r.session,
        r.definitions,
        ms(r.grace_ns),
        ms(r.window_ns),
        if r.valid { "valid" } else { "INVALID" }
    );
    for reason in &r.invalid_reasons {
        s.push_str(&format!("- invalid: {reason}\n"));
    }
    s.push_str("\n| channel | origin → receiver | expected | lost | loss % | dup | reordered | OWD p50 / p99 / max ms | ± bound max ms | age p50 / p99 ms | rate Hz | payload Mbps |\n");
    s.push_str("|---|---|---:|---:|---:|---:|---:|---|---:|---|---:|---:|\n");
    for st in &r.streams {
        let c = &st.counts;
        s.push_str(&format!(
            "| {} | {} → {} | {} | {} | {:.3} | {} | {} | {} / {} / {} | {} | {} / {} | {:.1} | {:.3} |\n",
            st.channel,
            st.origin,
            st.receiver,
            c.expected,
            c.lost,
            c.loss_ratio * 100.0,
            c.duplicates,
            c.reordered,
            ms(st.owd_ns.p50),
            ms(st.owd_ns.p99),
            ms(st.owd_ns.max),
            ms(st.owd_bound_ns.max),
            ms(st.age_at_use_ns.p50),
            ms(st.age_at_use_ns.p99),
            st.rate_hz,
            st.throughput_payload_mbps,
        ));
    }
    s.push_str("\n| node | sent | rejected | rx-queue overflow | inbox overflow | audit drops | complete |\n|---|---:|---:|---:|---:|---:|---|\n");
    for n in &r.nodes {
        s.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} |\n",
            n.node, n.sent, n.rejected_frames, n.rx_queue_overflows, n.inbox_overflows, n.audit_queue_drops, n.complete
        ));
    }
    s
}
