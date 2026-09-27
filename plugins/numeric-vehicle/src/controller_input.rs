//! Unbound W24 input-boundary candidate; NOT a full HIL plant or a controller.
//!
//! This module is included by tests only, not by `lib.rs`. Root must approve
//! the loaded controller version, model semantics and timing before wiring it.
//! `decode` validates the observed Custom1 SMC wire shape only. It does not
//! establish source identity, freshness, command admission or a hold policy.
//!
//! Sources: abi/include/xgc_schemas_v1.h and the controller source references
//! in ../HIL_INTERFACE_REVIEW.md. Never route a planner PVA into this decoder
//! as a fallback, add gravity, rotate axes or run an extra tracking controller.

const WIRE_LEN: usize = 104;
const SMC_ACCELERATION_MASK: u16 = 3135;
const WORLD_ENU_FRAME: u8 = 1; // This repository's pre-MAVROS wire convention.

/// A decoded acceleration-only controller output, not an executable command.
/// Position, velocity and yaw are intentionally absent: the mask ignores them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerAcceleration {
    /// Controller publication time in Session seconds, NOT planner stage time.
    /// Kept in the wire unit; application/expiry times are a Root-owned decision.
    pub stamp_seconds: f64,
    /// World ENU acceleration, m/s^2. No gravity or feedback is added here.
    pub acceleration: [f64; 3],
}

impl ControllerAcceleration {
    pub fn decode(bytes: &[u8]) -> Result<Self, &'static str> {
        if bytes.len() != WIRE_LEN {
            return Err("controller PositionTarget must be 104 bytes");
        }
        let mask = u16::from_le_bytes([bytes[96], bytes[97]]);
        if mask != SMC_ACCELERATION_MASK {
            return Err("requires acceleration-only SMC mask 3135; no planner PVA fallback");
        }
        if bytes[98] != WORLD_ENU_FRAME {
            return Err("requires repository world ENU frame 1");
        }
        if bytes[99..].iter().any(|byte| *byte != 0) {
            return Err("nonzero PositionTarget reserved bytes");
        }
        let number = |offset: usize| {
            let mut value = [0u8; 8];
            value.copy_from_slice(&bytes[offset..offset + 8]);
            f64::from_le_bytes(value)
        };
        let stamp_seconds = number(0);
        if !stamp_seconds.is_finite() || stamp_seconds <= 0.0 {
            return Err("controller timestamp must be finite and positive");
        }
        let acceleration = [number(56), number(64), number(72)];
        if acceleration.iter().any(|value| !value.is_finite()) {
            return Err("nonfinite controller acceleration");
        }
        Ok(Self {
            stamp_seconds,
            acceleration,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> [u8; WIRE_LEN] {
        let mut bytes = [0u8; WIRE_LEN];
        for (offset, value) in [(0, 42.125_f64), (56, 1.25), (64, -2.5), (72, 3.75)] {
            bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
        bytes[96..98].copy_from_slice(&SMC_ACCELERATION_MASK.to_le_bytes());
        bytes[98] = WORLD_ENU_FRAME;
        bytes
    }

    #[test]
    fn preserves_wire_units_axes_signs_and_publication_time() {
        assert_eq!(
            ControllerAcceleration::decode(&sample()).unwrap(),
            ControllerAcceleration {
                stamp_seconds: 42.125,
                acceleration: [1.25, -2.5, 3.75],
            }
        );
    }

    #[test]
    fn ignores_disabled_position_velocity_and_yaw_even_when_nonfinite() {
        let expected = ControllerAcceleration::decode(&sample()).unwrap();
        let mut bytes = sample();
        for offset in [8, 16, 24, 32, 40, 48, 80, 88] {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1234.5] {
                bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
                assert_eq!(ControllerAcceleration::decode(&bytes).unwrap(), expected);
            }
        }
    }

    #[test]
    fn rejects_every_other_mask_including_pva_force_and_missing_acceleration() {
        let mut bytes = sample();
        for mask in 0..=u16::MAX {
            bytes[96..98].copy_from_slice(&mask.to_le_bytes());
            assert_eq!(ControllerAcceleration::decode(&bytes).is_ok(), mask == 3135);
        }
    }

    #[test]
    fn rejects_every_other_frame_without_implicit_rotation() {
        let mut bytes = sample();
        for frame in 0..=u8::MAX {
            bytes[98] = frame;
            assert_eq!(ControllerAcceleration::decode(&bytes).is_ok(), frame == 1);
        }
    }

    #[test]
    fn rejects_wrong_sizes_without_panicking() {
        let bytes = sample();
        for len in 0..WIRE_LEN {
            assert!(ControllerAcceleration::decode(&bytes[..len]).is_err());
        }
        let mut oversized = bytes.to_vec();
        oversized.push(0);
        assert!(ControllerAcceleration::decode(&oversized).is_err());
    }

    #[test]
    fn rejects_each_reserved_byte() {
        for offset in 99..WIRE_LEN {
            let mut bytes = sample();
            bytes[offset] = 1;
            assert!(ControllerAcceleration::decode(&bytes).is_err());
        }
    }

    #[test]
    fn rejects_invalid_timestamps() {
        for value in [0.0_f64, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut bytes = sample();
            bytes[..8].copy_from_slice(&value.to_le_bytes());
            assert!(ControllerAcceleration::decode(&bytes).is_err());
        }
    }

    #[test]
    fn rejects_nonfinite_acceleration_on_each_axis() {
        for offset in [56, 64, 72] {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut bytes = sample();
                bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
                assert!(ControllerAcceleration::decode(&bytes).is_err());
            }
        }
    }

    #[test]
    fn zero_command_is_not_changed_into_gravity_or_hover_feedback() {
        let mut bytes = sample();
        bytes[56..80].fill(0);
        assert_eq!(
            ControllerAcceleration::decode(&bytes).unwrap().acceleration,
            [0.0; 3]
        );
    }

    #[test]
    fn changing_an_enabled_axis_changes_only_that_decoded_axis() {
        let mut bytes = sample();
        bytes[64..72].copy_from_slice(&7.0_f64.to_le_bytes());
        let decoded = ControllerAcceleration::decode(&bytes).unwrap();
        assert_eq!(decoded.acceleration, [1.25, 7.0, 3.75]);
        assert_eq!(decoded.stamp_seconds, 42.125);
    }
}
