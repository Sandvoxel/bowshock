//! Empirical base drag of a blunt base in free flight.

/// Base drag coefficient of a blunt base in free flight, referenced to the base area: 0.12 + 0.13 M^2 for M < 1 and
/// 0.25 / M for M >= 1 (S. Niskanen, "Development of an Open Source model rocket simulation software", M.Sc. thesis,
/// Helsinki University of Technology, 2009, OpenRocket technical documentation, base drag, eq. 3.94).
pub fn base_drag_coefficient(mach: f64) -> f64 {
    if mach < 1.0 {
        0.12 + 0.13 * mach * mach
    } else {
        0.25 / mach
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn correlation_values() {
        for (m, cd) in [(0.0, 0.12), (0.5, 0.1525), (1.0, 0.25), (2.0, 0.125)] {
            assert!((base_drag_coefficient(m) - cd).abs() < 1e-15, "M = {m}: {}", base_drag_coefficient(m));
        }
    }

    #[test]
    fn correlation_is_continuous_at_mach_one() {
        assert!((base_drag_coefficient(1.0 - 1e-12) - base_drag_coefficient(1.0)).abs() < 1e-11);
    }
}
