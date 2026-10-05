//! The Rust mirror must match xgc_rt.h byte for byte on LP64 targets
//! (x86_64, aarch64). The C side of the same check is plugins/c-stub.
use std::mem::{offset_of, size_of};
use xgc_rt_abi::*;

#[test]
fn sample_view_layout_matches_header() {
    assert_eq!(size_of::<XgcSampleView>(), 56);
    assert_eq!(offset_of!(XgcSampleView, len), 4);
    assert_eq!(offset_of!(XgcSampleView, seq), 8);
    assert_eq!(offset_of!(XgcSampleView, t_rx), 40);
    assert_eq!(offset_of!(XgcSampleView, data), 48);
}

#[test]
fn step_ctx_and_descriptor_layout_match_header() {
    assert_eq!(size_of::<XgcStepCtx>(), 48);
    assert_eq!(offset_of!(XgcStepCtx, dirty_ports), 32);
    assert_eq!(offset_of!(XgcStepCtx, round_advanced), 40);
    assert_eq!(size_of::<XgcPortDecl>(), 32);
    assert_eq!(size_of::<XgcPluginDescriptor>(), 40);
    assert_eq!(offset_of!(XgcPluginDescriptor, vtbl), 32);
    assert_eq!(size_of::<XgcHostApi>(), 88);
    assert_eq!(offset_of!(XgcHostApi, node_id), 72);
    assert_eq!(offset_of!(XgcHostApi, acquire_clock_reader), 80);
    assert_eq!(size_of::<XgcClockReaderV1>(), 24);
    assert_eq!(offset_of!(XgcClockReaderV1, opaque), 0);
    assert_eq!(offset_of!(XgcClockReaderV1, now), 8);
    assert_eq!(offset_of!(XgcClockReaderV1, release), 16);
    assert_eq!(size_of::<XgcPluginVtbl>(), 7 * 8);
}
