//! The 2D engine cases of the facade for the 3D wedge / sector GPU benchmarks and checks: `engine_case` is the 2D case of an analysis at one Mach number,
//! `wedge_case3` the single-cell wedge on the same base-wake grid (`Case3::wedge`).

use bowshock_core::engine::case::Case;
use bowshock_core::engine::case3::Case3;
use bowshock_core::{Analysis, Atmosphere, BodySpec, FlightConditions, Resolution, Wake};

/// The 2D engine case of the facade (`Analysis::engine_cases`) at `mach`: the Euler base-wake grid, or, with `rans`, the wall-resolved grid sized for
/// `mach` (ISA at 120 m as the app's table).
pub fn engine_case(body: &BodySpec, mach: f64, rans: bool, resolution: Resolution) -> Case {
    let mut a = Analysis::new(body).expect("body spec").resolution(resolution);
    a = if rans { a.viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 120.0 } }) } else { a.wake(Wake::Base) };
    a.engine_cases(&[mach]).expect("engine case").remove(0)
}

/// The base-grid parameters of [`engine_case`].
pub fn base_grid_params(body: &BodySpec, mach: f64, rans: bool, resolution: Resolution) -> bowshock_core::engine::body_grid::BaseGridParams {
    engine_case(body, mach, rans, resolution).grid_base
}

/// The wedge (`Case3::wedge`) of a 2D case: same Mach number, body and base-wake grid (cell counts, spacing), the Euler equations.
pub fn wedge_case3(case: &Case) -> Case3 {
    let mut c = Case3::wedge(case.mach, case.body.clone(), 0);
    c.grid.base = Some(case.grid_base.clone());
    c
}
