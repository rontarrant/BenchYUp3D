//! Coordinate convention for the planned Y-up migration.
//!
//! Legacy BenchUp uses right-handed Z-up coordinates. The new convention is
//! right-handed Y-up, with X unchanged and the old +Y mapped to new -Z.
//! This module provides pure conversions; it deliberately does not change
//! document serialization or runtime camera/tool behavior yet.

use glam::{DQuat, DVec3};

/// World-space vertical direction in the new coordinate system.
pub const UP: DVec3 = DVec3::Y;

/// Convert a legacy Z-up point or direction into right-handed Y-up space.
#[inline]
pub fn legacy_to_y_up(v: DVec3) -> DVec3 {
    DVec3::new(v.x, v.z, -v.y)
}

/// Convert a Y-up point or direction back to legacy Z-up space.
#[inline]
pub fn y_up_to_legacy(v: DVec3) -> DVec3 {
    DVec3::new(v.x, -v.z, v.y)
}

/// Convert an orientation by conjugating it with the basis change.
#[inline]
pub fn legacy_rotation_to_y_up(rotation: DQuat) -> DQuat {
    let basis = DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
    basis * rotation * basis.conjugate()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: DVec3, b: DVec3) {
        assert!((a - b).length() < 1e-10, "{a:?} != {b:?}");
    }

    #[test]
    fn maps_vertical_and_preserves_handedness() {
        close(legacy_to_y_up(DVec3::Z), DVec3::Y);
        close(legacy_to_y_up(DVec3::Y), -DVec3::Z);
        let x = legacy_to_y_up(DVec3::X);
        let y = legacy_to_y_up(DVec3::Y);
        let z = legacy_to_y_up(DVec3::Z);
        close(x.cross(y), z);
    }

    #[test]
    fn round_trip_coordinates() {
        let p = DVec3::new(2.5, -4.0, 7.25);
        close(y_up_to_legacy(legacy_to_y_up(p)), p);
        close(legacy_to_y_up(y_up_to_legacy(p)), p);
    }

    #[test]
    fn rotation_matches_converted_direction() {
        let r = DQuat::from_rotation_z(0.7) * DQuat::from_rotation_x(-0.3);
        let v = DVec3::new(1., 2., 3.).normalize();
        close(legacy_rotation_to_y_up(r) * legacy_to_y_up(v), legacy_to_y_up(r * v));
    }
}
