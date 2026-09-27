//! Wire-boundary tests only. No controller, transport or plant is started.
#[path = "../src/controller_input.rs"]
mod controller_input;

use controller_input::ControllerAcceleration;
use numeric_vehicle::model::Segment;

fn payload(mask: u16) -> [u8; 104] {
    let mut bytes = [0u8; 104];
    bytes[..8].copy_from_slice(&42.0_f64.to_le_bytes());
    bytes[56..64].copy_from_slice(&1.0_f64.to_le_bytes());
    bytes[96..98].copy_from_slice(&mask.to_le_bytes());
    bytes[98] = 1;
    bytes
}

#[test]
fn smc_output_cannot_be_relabelled_as_a_planner_segment() {
    let bytes = payload(3135);
    assert!(ControllerAcceleration::decode(&bytes).is_ok());
    assert!(Segment::decode(&bytes).is_err());
}

#[test]
fn planner_segment_cannot_bypass_the_controller_input_boundary() {
    for mask in [0, 3072] {
        let bytes = payload(mask);
        assert!(Segment::decode(&bytes).is_ok());
        assert!(ControllerAcceleration::decode(&bytes).is_err());
    }
}
