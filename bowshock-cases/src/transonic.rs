//! Transonic case runner: nose-cylinder body on the large-domain grid (`body_grid`), characteristic far-field BCs
//! (`bc`), steady-state iteration with optional warm start from the previous Mach number, and wall-pressure
//! post-processing (forebody pressure drag referenced to the base area).
//!
//! Boundary set-up: inflow plane and outer boundary `Bc::Characteristic`; outflow plane `Bc::Characteristic` (default)
//! or `Bc::PressureOutflow`; axis/body `Bc::SlipWall`. Lengths are in body diameters.

use bowshock::body_grid::{BaseBody, Body, Nose};
use bowshock::case::{Case, Wake};
use bowshock::numerics::Numerics;

/// Transonic validation case: nose + cylinder `cyl_length` long, sting wake (no base), explicit minmod numerics
/// ([`Numerics::transonic`]); domain 10 body lengths upstream and radially, 5 downstream.
pub fn transonic_case(mach: f64, level: u32, body: Body, cyl_length: f64) -> Case {
    Case::new(mach, BaseBody::cylinder(body, cyl_length), level, Wake::Sting, Numerics::transonic())
}

/// Published free-flight nose pressure-drag data, zero lift, fineness-ratio-3 noses on a cylinder.
///
/// Source: W. E. Stoney Jr., "Transonic Drag Measurements of Eight Body-Nose Shapes", NACA RM L53K17 (Langley, 1954),
/// NTRS 19930087953, Fig. 5 (p. 14): nose pressure-drag coefficient `CD` (reference area = maximum frontal area, i.e. the
/// base-area reference used here) vs Mach number. **The numbers are digitised by eye from a poor-quality scan of the plot
/// (they are not tabulated in the report): read-off uncertainty about +-0.01 in `CD` and +-0.01 in `M`** (`READ_UNCERTAINTY`).
/// The nose pressure drag is itself a derived quantity: free-flight total CD minus a tare (afterbody + fins + base,
/// 0.025 subsonic, about 0.065 supersonic, anchored at M = 1.2 on the Taylor-Maccoll cone drag; Fig. 4) minus the calculated
/// fully turbulent skin friction; the tare alone is uncertain by about +-0.01, the report states +-0.008 in total CD and +-0.005
/// in M. Free flight, Re = 8e6 .. 15e6 (body length), fully turbulent boundary layer (inferred by the author). Beyond
/// M ~ 1.225 the plotted curves are fairings to the Ames data (NACA Rep. 1386) and second-order theory, not new measurements.
/// Models: nose (l/d = 3), cylinder (l/d = 4), conical afterbody (l/d = 5, cut-off base), swept fins.
pub mod stoney {
    pub const READ_UNCERTAINTY: f64 = 0.01;
    /// Largest Mach number of the "present tests" (the curves beyond it are faired).
    pub const MEASURED_UP_TO: f64 = 1.225;
    /// Cone, `r = x tan(9.46 deg)` (`l/d = 3`).
    pub const CONE: &[(f64, f64)] =
        &[(0.80, 0.02), (0.90, 0.03), (0.95, 0.06), (1.00, 0.11), (1.05, 0.16), (1.10, 0.16), (1.15, 0.15), (1.20, 0.14), (1.25, 0.13)];
    /// Power series `r = x^(1/2)` (vertex parabola).
    pub const POWER_HALF: &[(f64, f64)] =
        &[(0.80, 0.00), (0.90, 0.00), (0.95, 0.01), (1.00, 0.05), (1.05, 0.06), (1.10, 0.06), (1.15, 0.07), (1.20, 0.08), (1.25, 0.07)];
    /// Von Karman (LD-Haack, `C = 0`), digitised above M = 0.95 only.
    pub const VON_KARMAN: &[(f64, f64)] = &[(0.95, 0.00), (1.00, 0.03), (1.05, 0.07), (1.10, 0.075), (1.15, 0.075), (1.20, 0.08), (1.24, 0.077)];
    /// LV-Haack (`C = 1/3`), digitised above M = 0.95 only.
    pub const LV_HAACK: &[(f64, f64)] = &[(0.95, 0.00), (1.00, 0.03), (1.05, 0.07), (1.10, 0.085), (1.15, 0.097), (1.17, 0.104)];
    /// Parabolic series `K = 1` (`r/R = 2 eta - eta^2`, tangent to the cylinder; closest of the eight shapes to a tangent ogive of
    /// the same fineness). Middle panel of Fig. 5, the hardest to read: uncertainty +-0.01 in CD, more where curves cross.
    pub const PARABOLA: &[(f64, f64)] = &[(0.95, 0.01), (0.975, 0.02), (1.00, 0.045), (1.05, 0.09), (1.10, 0.105), (1.15, 0.115), (1.20, 0.12)];

    /// Linear interpolation of a series; `None` outside its Mach range.
    pub fn interp(series: &[(f64, f64)], m: f64) -> Option<f64> {
        for w in series.windows(2) {
            if m >= w[0].0 - 1e-9 && m <= w[1].0 + 1e-9 {
                let t = (m - w[0].0) / (w[1].0 - w[0].0);
                return Some(w[0].1 + t * (w[1].1 - w[0].1));
            }
        }
        None
    }
}

/// Named test bodies (fineness-ratio-3 noses, `d = 1`) with the matching Stoney data series.
pub fn named_body(name: &str) -> Option<(Body, Option<&'static [(f64, f64)]>)> {
    let f = 3.0;
    Some(match name {
        "cone" => (Body { nose: Nose::cone_fineness(f) }, Some(stoney::CONE)),
        "karman" => (Body { nose: Nose::Haack { fineness: f, c: 0.0 } }, Some(stoney::VON_KARMAN)),
        "lvhaack" => (Body { nose: Nose::Haack { fineness: f, c: 1.0 / 3.0 } }, Some(stoney::LV_HAACK)),
        "parabola" => (Body { nose: Nose::Parabolic { fineness: f, k: 1.0 } }, Some(stoney::PARABOLA)),
        "sqrt" => (Body { nose: Nose::Power { fineness: f, n: 0.5 } }, Some(stoney::POWER_HALF)),
        "ogive" => (Body { nose: Nose::TangentOgive { fineness: f } }, None),
        _ => return None,
    })
}

/// Tangent-cone estimate of the forebody CD (base-area reference): `int Cp_TM(local slope) d(r^2/R^2)` over the nose,
/// `Cp_TM` the Taylor-Maccoll surface Cp of the cone whose half angle equals the local slope (linearly scaled with the slope
/// below 1 deg). NaN if any local slope is beyond the attached-shock limit (e.g. subsonic/transonic Mach, rounded tips).
/// For a cone body this is the exact Taylor-Maccoll cone drag `Cp_cone`.
pub fn tangent_cone_cd(body: &Body, mach: f64) -> f64 {
    let n = 400;
    let (ln, rb2) = (body.nose_length(), body.radius().powi(2));
    let floor = 1f64.to_radians();
    let mut cd = 0.0;
    let mut last: Option<(f64, f64)> = None; // (theta, cp) memo: a cone has the same slope everywhere
    for k in 0..n {
        let (x0, x1) = (ln * k as f64 / n as f64, ln * (k + 1) as f64 / n as f64);
        let theta = body.slope(0.5 * (x0 + x1)).atan();
        let cp = match last {
            Some((t, c)) if (t - theta).abs() < 1e-13 => c,
            _ => {
                let c = if theta >= floor {
                    bowshock_reference::taylor_maccoll::solve_cone(mach, theta).map_or(f64::NAN, |s| s.cp_surface)
                } else {
                    bowshock_reference::taylor_maccoll::solve_cone(mach, floor).map_or(f64::NAN, |s| s.cp_surface) * theta / floor
                };
                last = Some((theta, c));
                c
            }
        };
        if cp.is_nan() {
            return f64::NAN;
        }
        cd += cp * (body.r(x1).powi(2) - body.r(x0).powi(2)) / rb2;
    }
    cd
}

/// Published wind-tunnel surface pressures, zero angle of attack, 3-caliber tangent-ogive cylinder (the `ogive` body of
/// [`named_body`]).
///
/// Source: K. R. Czarnecki and W. J. Monta, "Pressure Distributions Due to Two-Dimensional Fabrication-Type Surface Roughness on an
/// Ogive Cylinder at Transonic Speeds", NASA TN D-3516 (Langley, 1966), NTRS 19660024157, Fig. 5 (report p. 27): the "basic smooth
/// model". Model (Fig. 1, Table II): tangent ogive nose 12.29 in = 3.00 d, ogive radius 37.89 in, d = 4.096 in, total length 50 in
/// (12.2 d), sting mounted (sting not shown in the figure). Langley 8-ft transonic pressure tunnel (slotted), fully turbulent
/// (No. 60 carborundum 0.75 in from the tip), Re per ft about 1e6 .. 6e6, no Reynolds-number effect on Cp reported, no wall corrections.
/// Empty-tunnel Mach non-uniformity about +-0.005 (subsonic) .. +-0.02 (highest M). **The Cp values are digitised from the plot
/// (not tabulated): +-0.01 for x >= 6 in, about +-0.02 for x < 5 in, about +-0.015 within 1 in of a shock; the last point (x = 49.62 in,
/// near the base/sting) +-0.02.** Slotted-wall shock reflections are stated by the authors: the reflected nose shock hits the body at
/// x ~ 30 in at M = 1.1 and x ~ 48 in at M = 1.2 (tunnel artefact, not present in free air).
pub mod czarnecki_monta {
    pub const D_IN: f64 = 4.096;
    pub const MACH: [f64; 5] = [0.7, 0.9, 1.0, 1.1, 1.2];
    /// Digitisation uncertainty in Cp (stations x >= 6 in); about twice that on the nose, x < 5 in.
    pub const READ_UNCERTAINTY: f64 = 0.01;
    /// Last station (in) not contaminated by the stated wall-reflected shock / base-sting region, per Mach number.
    pub fn valid_x_max_in(mach: f64) -> f64 {
        if (mach - 1.1).abs() < 1e-9 {
            28.8
        } else {
            44.1
        }
    }
    /// Station `x` [in] from the nose tip.
    pub const X_IN: [f64; 34] = [
        1.01, 1.95, 2.82, 3.81, 4.78, 5.78, 6.76, 7.76, 8.77, 9.79, 10.78, 11.80, 12.78, 13.77, 14.78, 15.78, 16.78, 17.75, 18.79, 19.78, 20.78, 22.75, 24.74,
        26.77, 28.76, 30.77, 32.76, 34.78, 36.79, 42.14, 44.10, 46.08, 48.08, 49.62,
    ];
    /// Cp at M = 0.7, 0.9, 1.0, 1.1, 1.2 for each station.
    pub const CP: [[f64; 5]; 34] = [
        [0.282, 0.312, 0.371, 0.372, 0.353],
        [0.186, 0.220, 0.283, 0.279, 0.285],
        [0.122, 0.140, 0.203, 0.233, 0.244],
        [0.057, 0.066, 0.131, 0.173, 0.193],
        [0.010, 0.009, 0.071, 0.122, 0.148],
        [-0.034, -0.045, 0.010, 0.063, 0.084],
        [-0.062, -0.086, -0.034, 0.021, 0.042],
        [-0.086, -0.120, -0.089, -0.026, 0.001],
        [-0.106, -0.152, -0.126, -0.065, -0.035],
        [-0.116, -0.167, -0.182, -0.112, -0.079],
        [-0.110, -0.149, -0.215, -0.146, -0.105],
        [-0.077, -0.098, -0.241, -0.172, -0.131],
        [-0.055, -0.066, -0.194, -0.137, -0.107],
        [-0.037, -0.047, -0.154, -0.108, -0.087],
        [-0.028, -0.035, -0.120, -0.089, -0.069],
        [-0.014, -0.024, -0.097, -0.070, -0.051],
        [-0.013, -0.021, -0.081, -0.059, -0.040],
        [-0.011, -0.017, -0.070, -0.053, -0.032],
        [-0.014, -0.023, -0.072, -0.053, -0.030],
        [-0.008, -0.017, -0.058, -0.047, -0.025],
        [-0.008, -0.015, -0.057, -0.043, -0.020],
        [-0.008, -0.015, -0.042, -0.037, -0.016],
        [-0.007, -0.014, 0.026, -0.024, -0.016],
        [-0.007, -0.013, 0.029, -0.018, -0.017],
        [-0.003, -0.010, 0.026, -0.010, -0.016],
        [-0.006, -0.012, 0.012, 0.059, -0.021],
        [-0.005, -0.013, 0.000, 0.061, -0.021],
        [0.000, -0.007, 0.001, 0.049, -0.015],
        [0.000, -0.007, -0.001, 0.032, -0.011],
        [0.005, -0.003, 0.001, 0.003, -0.011],
        [-0.001, -0.008, -0.007, -0.021, -0.017],
        [-0.010, -0.016, -0.015, -0.060, -0.013],
        [-0.022, -0.030, -0.025, -0.082, 0.040],
        [-0.058, -0.067, -0.079, -0.154, -0.046],
    ];

    /// Column of `CP` for Mach number `m` (exact match to the test Mach numbers).
    pub fn column(m: f64) -> Option<usize> {
        MACH.iter().position(|&x| (x - m).abs() < 1e-9)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock::run::run_cold;
    use bowshock_core::flux::Limiter;
    use bowshock_core::gas::{cons_to_prim, freestream, prim_to_cons};
    use bowshock_core::grid::Grid;
    use bowshock_core::solver::{Bc, Config, Solver};

    /// Smoothly curved, non-uniform grid with the lower boundary on the axis r=0.
    fn curved_grid(ni: usize, nj: usize) -> Grid {
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                let (s, t) = (i as f64 / ni as f64, j as f64 / nj as f64);
                let k = j * (ni + 1) + i;
                x[k] = 2.0 * s + 0.15 * (3.0 * t).sin() * (2.0 * s).cos() + 0.1 * t * t;
                r[k] = t * (1.0 + 0.3 * s) * (1.0 + 0.2 * t) + 0.05 * (4.0 * s).sin() * t * (1.0 - t);
            }
        }
        Grid::from_nodes(ni, nj, x, r)
    }

    /// Uniform flow with the characteristic BCs on inflow, outflow and outer boundary and a slip axis must stay uniform:
    /// no spurious waves from the BCs at M = 0.5 and 0.9 (also 0.98 and 1.2, other regimes).
    #[test]
    fn characteristic_farfield_preserves_freestream() {
        for mach in [0.5, 0.9, 0.98, 1.2] {
            for outflow in [Bc::Characteristic, Bc::PressureOutflow] {
                let fs = freestream(mach);
                let cfg = Config {
                    cfl: 0.5,
                    limiter: Limiter::Minmod,
                    freestream: fs,
                    bc: [Bc::Characteristic, outflow, Bc::SlipWall, Bc::Characteristic],
                    first_order_iters: 0,
                };
                let mut s = Solver::new(curved_grid(24, 16), cfg);
                for _ in 0..100 {
                    s.step();
                }
                let q0 = prim_to_cons(&fs);
                let mut worst: f64 = 0.0;
                for q in &s.u {
                    for k in 0..4 {
                        worst = worst.max((q[k] - q0[k]).abs() / q0[k].abs().max(1.0));
                    }
                }
                assert!(worst < 1e-12, "M={mach} {outflow:?}: drift {worst:e}");
            }
        }
    }

    /// An acoustic/entropy perturbation introduced in the interior must leave a subsonic domain through the
    /// characteristic BCs (the residual of the perturbation decays), while it is reflected / trapped less than with
    /// a fixed free stream. Here: Gaussian pressure pulse, M = 0.5, the perturbation energy must drop by > 90 %
    /// within a few domain crossing times.
    #[test]
    fn characteristic_farfield_lets_a_pressure_pulse_out() {
        let mach = 0.5;
        let fs = freestream(mach);
        let (ni, nj) = (60, 30);
        // uniform rectangle [0, 6] x [0, 3] with the axis at the bottom treated as slip wall
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                x[j * (ni + 1) + i] = 6.0 * i as f64 / ni as f64;
                r[j * (ni + 1) + i] = 3.0 * j as f64 / nj as f64;
            }
        }
        let grid = Grid::from_nodes(ni, nj, x, r);
        let run = |bc: [Bc; 4]| {
            let cfg = Config { cfl: 0.5, limiter: Limiter::Minmod, freestream: fs, bc, first_order_iters: 0 };
            let mut s = Solver::new(grid_clone(&grid), cfg);
            for j in 0..nj {
                for i in 0..ni {
                    let (xc, rc) = (s.grid.xc[j * ni + i], s.grid.rc[j * ni + i]);
                    let dp = 0.05 * fs[3] * (-((xc - 3.0).powi(2) + (rc - 1.0).powi(2)) / 0.2).exp();
                    s.u[j * ni + i] = prim_to_cons(&[fs[0] * (1.0 + dp / fs[3] / 1.4), fs[1], fs[2], fs[3] + dp]);
                }
            }
            let energy = |s: &Solver| -> f64 { s.u.iter().map(|q| (cons_to_prim(q)[3] - fs[3]).powi(2)).sum::<f64>().sqrt() };
            let e0 = energy(&s);
            // physical time is not tracked (local time stepping is used): iterate long enough for many crossings
            for _ in 0..1500 {
                s.step();
            }
            energy(&s) / e0
        };
        let char_bc = run([Bc::Characteristic, Bc::Characteristic, Bc::SlipWall, Bc::Characteristic]);
        assert!(char_bc < 0.1, "pulse remaining with characteristic BCs: {char_bc:e}");
    }

    fn grid_clone(g: &Grid) -> Grid {
        Grid::from_nodes(g.ni, g.nj, g.x.clone(), g.r.clone())
    }

    /// Smoke test: coarse grid, cone-cylinder, M = 0.9, 1500 iterations: finite, residual falls, nose CD plausible.
    #[test]
    fn transonic_smoke_coarse_m09() {
        let cone = Body { nose: Nose::cone_fineness(3.0) };
        let mut case = transonic_case(0.9, 0, cone, 6.0);
        // shrink to keep the test quick
        case.grid_sting.n_up = 16;
        case.grid_sting.n_nose = 32;
        case.grid_sting.n_seg[0] = 16;
        case.grid_sting.n_ext = 14;
        case.grid_sting.nj = 40;
        case.grid_sting.dx_tip = 0.04;
        case.grid_sting.dr_wall = 0.04;
        case.numerics.max_iter = 1500;
        case.numerics.first_order_iters = 150;
        let res = run_cold(&case);
        assert!(res.residual.iter().all(|r| r.is_finite()), "residual contains a non-finite value");
        assert!(res.min_rho > 0.0 && res.min_p > 0.0);
        assert!(res.drop_orders > 1.0, "residual dropped only {:.2} orders", res.drop_orders);
        // cone-cylinder: Stoney (RM L53K17, digitised) has 0.03 +- 0.01 at M = 0.9, the coarse 1500-iteration run must be within a factor ~3
        assert!(res.cd_nose.is_finite() && res.cd_nose > 0.0 && res.cd_nose < 0.12, "CD {}", res.cd_nose);
        // subsonic slender-body flow: the wall Cp on the nose is positive at the tip and negative (suction) near the shoulder
        assert!(res.wall.first().unwrap().cp > 0.0 && res.cp_min < 0.0);
    }
}
