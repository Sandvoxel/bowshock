//! NASA TP-2005 (Covell, 1982): afterbody pressures with axisymmetric cold air jets at Mach 1.65 to 2.50 on a strut-mounted
//! ogive-cylinder, <https://ntrs.nasa.gov/api/citations/19820014341/downloads/19820014341.pdf>.

/// Body diameter [m] (Fig. 2).
pub const BODY_DIAMETER: f64 = 0.0635;
/// Model station of the base [m] (Figs. 1 and 2).
pub const BASE_STATION: f64 = 0.8255;
/// Nozzle exit diameter [m] (Fig. 2, all four nozzles).
pub const NOZZLE_EXIT_DIAMETER: f64 = 0.0508;
/// Free-stream stagnation temperature [K].
pub const T0_K: f64 = 325.0;
/// Free-stream Reynolds number per metre.
pub const RE_PER_M: f64 = 6.56e6;
/// Jet total temperature [K]; the report gives 300 to 311 K.
pub const JET_TOTAL_TEMPERATURE_K: f64 = 305.0;
/// Nose length [m]: not given in the report; fineness 3 is assumed (the Fig. 1 sketch suggests about 19 cm).
pub const NOSE_LENGTH: f64 = 0.1905;

/// Table II: `(M_inf, Me, pt_j/pb, pt_j/p_inf)` at the selected plume design conditions.
pub const TABLE_II: [(f64, f64, f64, f64); 11] = [
    (1.65, 1.69, 40.6, 63.4),
    (1.65, 1.97, 56.0, 91.0),
    (1.65, 2.33, 82.4, 137.2),
    (1.65, 2.53, 109.0, 181.4),
    (2.00, 1.69, 39.8, 53.4),
    (2.00, 1.97, 55.0, 84.2),
    (2.00, 2.33, 82.3, 135.1),
    (2.00, 2.53, 108.1, 177.4),
    (2.50, 1.69, 39.3, 37.4),
    (2.50, 1.97, 54.6, 61.7),
    (2.50, 2.53, 108.9, 145.4),
];

/// Measured base pressure ratio `pb/p_inf = (pt_j/p_inf) / (pt_j/pb)` of a [`TABLE_II`] row.
pub fn base_pressure_ratio(row: (f64, f64, f64, f64)) -> f64 {
    row.3 / row.2
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_pressure_ratio_matches_table() {
        assert!((base_pressure_ratio(TABLE_II[0]) - 1.562).abs() < 1e-3);
        assert!((base_pressure_ratio(TABLE_II[8]) - 0.952).abs() < 1e-3);
    }
}
