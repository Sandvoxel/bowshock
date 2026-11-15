//! Number formatting for the Simulate screen: SI in, the app's unit system out.

use crate::units::{self, Quantity, Unit, UnitSystem};

/// Whether `system` shows lengths and speeds in feet.
pub fn imperial(system: UnitSystem) -> bool {
    matches!(system, UnitSystem::InOzS | UnitSystem::FtLbS)
}

/// `value_si` of `quantity` in `system` with `decimals` decimals: the number and its unit symbol.
pub fn num(value_si: f64, quantity: Quantity, system: UnitSystem, decimals: usize) -> (String, &'static str) {
    let unit = Unit { decimals, ..*quantity.unit_for(system) };
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn differences_have_a_sign_and_a_true_minus() {
        assert_eq!(signed(274.0, 0), "+274");
        assert_eq!(signed(-34.0, 0), "−34");
        assert_eq!(signed(0.2, 0), "0");
        assert_eq!(signed(-0.04, 1), "0");
        assert_eq!(signed(1.25, 1), "+1.3");
    }

    #[test]
    fn plot_units_never_use_millimetres() {
        assert!(!imperial(UnitSystem::MmGS));
        assert!(imperial(UnitSystem::InOzS));
        assert_eq!(with_unit(616.0, Quantity::Altitude, UnitSystem::MmGS, 0), "616 m");
        assert_eq!(with_unit(304.8, Quantity::Altitude, UnitSystem::FtLbS, 0), "1000 ft");
    }

    #[test]
    fn landing_reads_distance_then_three_digit_bearing() {
        assert_eq!(landing(104.2, 68.0, UnitSystem::MmGS), "104 m · 068°");
        assert_eq!(bearing(-10.0), "350°");
    }
}
