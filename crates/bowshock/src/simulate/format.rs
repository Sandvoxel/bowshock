//! Number formatting for the Simulate screen: SI in, the app's unit set out.

use crate::units::{self, Quantity, Unit, UnitSystem};

/// `value_si` of `quantity` in `system` with `decimals` decimals: the number and its unit symbol.
///
/// `decimals` is what the caller wants in the quantity's first unit (metres for altitudes, m/s for speeds); a unit a power of
/// ten larger than that one (km, mi, kg for g) gets as many more decimals, so `0.616 km` does not collapse to `1 km`.
pub fn num(value_si: f64, quantity: Quantity, system: UnitSystem, decimals: usize) -> (String, &'static str) {
    let chosen = quantity.unit_for(system);
    let ratio = chosen.to_si / quantity.units()[0].to_si;
    let extra = if ratio >= 10.0 { ratio.log10().floor() as usize } else { 0 };
    let unit = Unit { decimals: decimals + extra, ..*chosen };
    (units::format_in(value_si, &unit), unit.symbol)
}

/// [`num`] with a space and the unit, `616 m`.
pub fn with_unit(value_si: f64, quantity: Quantity, system: UnitSystem, decimals: usize) -> String {
    let (n, u) = num(value_si, quantity, system, decimals);
    format!("{n} {u}")
}

/// A signed difference to the reference, with a true minus: `+274`, `−34`, `0`.
pub fn signed(diff: f64, decimals: usize) -> String {
    let scale = 10f64.powi(decimals as i32);
    let rounded = (diff * scale).round() / scale;
    if rounded == 0.0 {
        "0".to_string()
    } else if rounded > 0.0 {
        format!("+{rounded:.decimals$}")
    } else {
        format!("−{:.decimals$}", -rounded)
    }
}

/// Bearing as three digits, `068°`.
pub fn bearing(deg: f64) -> String {
    format!("{:03.0}°", deg.rem_euclid(360.0))
}

/// Distance and bearing from the pad, `104 m · 068°`.
pub fn landing(distance: f64, bearing_deg: f64, system: UnitSystem) -> String {
    format!("{} · {}", with_unit(distance, Quantity::Altitude, system, 0), bearing(bearing_deg))
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
    use crate::units::Family;

    #[test]
    fn differences_have_a_sign_and_a_true_minus() {
        assert_eq!(signed(274.0, 0), "+274");
        assert_eq!(signed(-34.0, 0), "−34");
        assert_eq!(signed(0.2, 0), "0");
        assert_eq!(signed(-0.04, 1), "0");
        assert_eq!(signed(1.25, 1), "+1.3");
    }

    #[test]
    fn quantities_follow_the_unit_set_per_family() {
        assert_eq!(with_unit(616.0, Quantity::Altitude, UnitSystem::MM_G_S, 0), "616 m");
        assert_eq!(with_unit(304.8, Quantity::Altitude, UnitSystem::FT_LB_S, 0), "1000 ft");
        // Kilometres and miles keep their fraction: the decimals asked for are those of the family's first unit (metres).
        let set = UnitSystem::MM_G_S.with(Family::Distance, "km").with(Family::Velocity, "mph");
        assert_eq!(with_unit(616.0, Quantity::Altitude, set, 0), "0.616 km");
        assert_eq!(with_unit(1609.344, Quantity::Altitude, UnitSystem::MM_G_S.with(Family::Distance, "mi"), 0), "1.000 mi");
        assert_eq!(with_unit(10.0, Quantity::Velocity, set, 1), "22.4 mph");
    }

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
