//! Number formatting for the Simulate screen: SI in, the app's unit set out.

use crate::units::{Quantity, UnitSystem};

/// Bearing as three digits, `068°`.
pub fn bearing(deg: f64) -> String {
    format!("{:03.0}°", deg.rem_euclid(360.0))
}

/// Distance and bearing from the pad, `104 m · 068°`.
pub fn landing(distance: f64, bearing_deg: f64, system: UnitSystem) -> String {
    format!("{} · {}", crate::units::with_unit_scaled(distance, Quantity::Altitude, system, 0), bearing(bearing_deg))
}

/// Turns per second of a roll rate of `rad_s` radians per second (the magnitude).
pub fn rev_per_s(rad_s: f64) -> f64 {
    rad_s.abs() / std::f64::consts::TAU
}

/// Turns per minute (rpm) of a roll rate of `rad_s` radians per second (the magnitude).
pub fn rpm(rad_s: f64) -> f64 {
    60.0 * rev_per_s(rad_s)
}

/// Which way a roll rate turns seen from behind the rocket: `CW` for a positive one (the way positive fin cant turns it), `CCW`
/// for a negative one.
pub fn spin_sense(rad_s: f64) -> &'static str {
    if rad_s < 0.0 {
        "CCW"
    } else {
        "CW"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn landing_reads_distance_then_three_digit_bearing() {
        assert_eq!(landing(104.2, 68.0, UnitSystem::MM_G_S), "104 m · 068°");
        assert_eq!(bearing(-10.0), "350°");
    }

    #[test]
    fn spin_reads_in_turns_per_second_and_per_minute_with_its_direction_seen_from_behind() {
        // 122.3 rad/s is 19.47 turns per second, 1168 rpm.
        assert!((rev_per_s(122.3) - 19.465).abs() < 1e-3 && (rpm(122.3) - 1167.9).abs() < 0.1);
        assert!((rpm(-122.3) - rpm(122.3)).abs() < 1e-12, "the magnitude");
        assert_eq!((spin_sense(2.0), spin_sense(-2.0), spin_sense(0.0)), ("CW", "CCW", "CW"));
    }
}
