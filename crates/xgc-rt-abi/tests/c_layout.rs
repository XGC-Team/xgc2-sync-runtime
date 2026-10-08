#![cfg(target_pointer_width = "64")]

#[test]
fn lp64_c_layout_preserves_the_previous_prefix() {
    use std::{fs, process::Command};
    let root = std::env::temp_dir().join(format!("xgc-abi-layout-{}", std::process::id()));
    fs::create_dir_all(&root).unwrap();
    let source = root.join("layout.c");
    fs::write(
        &source,
        r#"
#include <stddef.h>
#include <xgc_rt.h>
_Static_assert(XGC_RT_ABI_VERSION == 1 && XGC_RT_ABI_MINOR == 3, "ABI version");
_Static_assert(sizeof(xgc_host_api) == 88, "LP64 host table");
_Static_assert(offsetof(xgc_host_api, host) == 8, "old host context");
_Static_assert(offsetof(xgc_host_api, publish) == 16, "minor-0 prefix");
_Static_assert(offsetof(xgc_host_api, request_recover) == 56, "minor-0 end");
_Static_assert(offsetof(xgc_host_api, port_origins) == 64, "minor-1 prefix");
_Static_assert(offsetof(xgc_host_api, node_id) == 72, "minor-1 end");
_Static_assert(offsetof(xgc_host_api, rpc_runtime) == 80, "minor-3 extension");
int main(void) { return 0; }
"#,
    )
    .unwrap();
    let include = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../abi/include");
    let output = Command::new("cc")
        .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-fsyntax-only"])
        .arg("-I")
        .arg(include)
        .arg(&source)
        .output()
        .unwrap();
    let _ = fs::remove_dir_all(root);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
}
