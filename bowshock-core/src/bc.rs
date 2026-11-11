//! Characteristic (Riemann-invariant) far-field and static-pressure outflow ghost states.
//!
//! All functions take the primitive state of the first interior cell and the **outward** unit normal
//! `(nx, nr)` of the boundary face and return the ghost (boundary) state; both ghost layers are
//! filled with it. They are pure functions of their inputs so they port 1:1 to WGSL.
//!
//! # Characteristic far field (`characteristic_ghost`)
//!
//! With the outward normal `n`, `u_n = v.n`, `c` the sound speed and `g = gamma`, the one-dimensional
//! (normal to the boundary) Riemann invariants are
//!
//! ```text
//!   R_out = u_n,i + 2 c_i/(g-1)     carried out of the domain by the u_n + c wave  (from the interior state)
//!   R_in  = u_n,e - 2 c_e/(g-1)     carried into the domain by the u_n - c wave    (from the free stream)
//!   u_n,b = (R_out + R_in)/2,   c_b = (g-1)/4 (R_out - R_in)
//! ```
//!
//! The entropy `s = p/rho^g` and the tangential velocity are convected with the flow: taken from the interior
//! for outflow (`u_n,b >= 0`) and from the free stream for inflow. Then `rho_b = (c_b^2/(g s))^(1/(g-1))`,
//! `p_b = rho_b c_b^2/g`, `v_b = v_ref + (u_n,b - u_n,ref) n`.
//!
//! Which branch is used is decided per face by the **interior normal Mach number** `M_n = u_n,i/c_i`
//! ([`Regime`]):
//!
//! | `M_n` | regime | ghost |
//! |---|---|---|
//! | `<= -1` | supersonic inflow | free stream |
//! | `(-1, 0)` | subsonic inflow | Riemann (inflow/outflow split of entropy + tangential velocity by the sign of `u_n,b`) |
//! | `[0, 1)` | subsonic outflow | Riemann (same) |
//! | `>= 1` | supersonic outflow | interior (zero gradient) |
//!
//! If the interior state is the free stream, every branch returns the free stream (to round-off), so a uniform
//! flow produces no spurious waves.

use crate::gas::{sound_speed, Prim, GAMMA};

/// Flow regime of a far-field face, selected by the interior normal Mach number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Regime {
    SupersonicInflow,
    SubsonicInflow,
    SubsonicOutflow,
    SupersonicOutflow,
}

/// Normal Mach number `v.n / c` of `w` for the outward unit normal `(nx, nr)`.
#[inline]
pub fn normal_mach(w: &Prim, nx: f64, nr: f64) -> f64 {
    (w[1] * nx + w[2] * nr) / sound_speed(w)
}

/// Regime of a face from the interior state and the outward normal.
#[inline]
pub fn regime(interior: &Prim, nx: f64, nr: f64) -> Regime {
    let m = normal_mach(interior, nx, nr);
    if m <= -1.0 {
        Regime::SupersonicInflow
    } else if m < 0.0 {
        Regime::SubsonicInflow
    } else if m < 1.0 {
        Regime::SubsonicOutflow
    } else {
        Regime::SupersonicOutflow
    }
}

/// Ghost state of the characteristic far-field BC. `fs` is the free stream, `interior` the first interior cell,
/// `(nx, nr)` the outward unit normal. Returns the regime used as well.
pub fn characteristic_ghost(fs: &Prim, interior: &Prim, nx: f64, nr: f64) -> (Prim, Regime) {
    let reg = regime(interior, nx, nr);
    let g1 = GAMMA - 1.0;
    let ghost = match reg {
        Regime::SupersonicInflow => *fs,
        Regime::SupersonicOutflow => *interior,
        Regime::SubsonicInflow | Regime::SubsonicOutflow => {
            let (ci, ce) = (sound_speed(interior), sound_speed(fs));
            let uni = interior[1] * nx + interior[2] * nr;
            let une = fs[1] * nx + fs[2] * nr;
            let r_out = uni + 2.0 * ci / g1;
            let r_in = une - 2.0 * ce / g1;
            let un_b = 0.5 * (r_out + r_in);
            let c_b = (0.25 * g1 * (r_out - r_in)).max(1e-8);
            // reference state carrying entropy and tangential velocity
            let (rf, un_ref) = if un_b >= 0.0 { (interior, uni) } else { (fs, une) };
            let s = rf[3] / rf[0].powf(GAMMA);
            let rho = (c_b * c_b / (GAMMA * s)).powf(1.0 / g1);
            let p = rho * c_b * c_b / GAMMA;
            let dn = un_b - un_ref;
            [rho, rf[1] + dn * nx, rf[2] + dn * nr, p]
        }
    };
    (ghost, reg)
}

/// Fixed static-pressure outflow: subsonic faces (`M_n < 1`) take `rho, u, v` from the interior and
/// `p = p_back`; supersonic faces extrapolate.
pub fn pressure_outflow_ghost(p_back: f64, interior: &Prim, nx: f64, nr: f64) -> Prim {
    if normal_mach(interior, nx, nr) >= 1.0 {
        *interior
    } else {
        [interior[0], interior[1], interior[2], p_back]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gas::freestream;

    #[test]
    fn regime_selected_by_normal_mach() {
        let c = 1.0f64; // these states have rho = 1, p = 1/g, hence c = 1
        let mk = |u: f64, v: f64| [1.0, u, v, 1.0 / GAMMA];
        // normal pointing in +x
        assert_eq!(regime(&mk(-1.5 * c, 0.0), 1.0, 0.0), Regime::SupersonicInflow);
        assert_eq!(regime(&mk(-c, 0.0), 1.0, 0.0), Regime::SupersonicInflow);
        assert_eq!(regime(&mk(-0.5 * c, 0.0), 1.0, 0.0), Regime::SubsonicInflow);
        assert_eq!(regime(&mk(0.5 * c, 0.0), 1.0, 0.0), Regime::SubsonicOutflow);
        assert_eq!(regime(&mk(0.0, 0.0), 1.0, 0.0), Regime::SubsonicOutflow);
        assert_eq!(regime(&mk(1.0 * c, 0.0), 1.0, 0.0), Regime::SupersonicOutflow);
        assert_eq!(regime(&mk(2.0 * c, 0.0), 1.0, 0.0), Regime::SupersonicOutflow);
        // same state, normal -x: flips inflow <-> outflow
        assert_eq!(regime(&mk(0.5 * c, 0.0), -1.0, 0.0), Regime::SubsonicInflow);
        assert_eq!(regime(&mk(2.0 * c, 0.0), -1.0, 0.0), Regime::SupersonicInflow);
        // radial normal: axial flow is tangential -> subsonic whatever the Mach number
        assert_eq!(regime(&mk(3.0 * c, 0.0), 0.0, 1.0), Regime::SubsonicOutflow);
        // oblique normal: M_n = M cos(60 deg)
        let (nx, nr) = (0.5, 3f64.sqrt() / 2.0);
        assert_eq!(regime(&mk(1.9, 0.0), nx, nr), Regime::SubsonicOutflow);
        assert_eq!(regime(&mk(2.1, 0.0), nx, nr), Regime::SupersonicOutflow);
    }

    #[test]
    fn ghost_follows_regime() {
        let fs = freestream(0.9);
        let s = |w: &Prim| w[3] / w[0].powf(GAMMA);
        // supersonic inflow: free stream regardless of the interior; supersonic outflow: interior
        let int = [1.3, 2.0, 0.1, 1.2];
        let (g, r) = characteristic_ghost(&fs, &int, -1.0, 0.0);
        assert_eq!(r, Regime::SupersonicInflow);
        assert_eq!(g, fs);
        let (g, r) = characteristic_ghost(&fs, &int, 1.0, 0.0);
        assert_eq!(r, Regime::SupersonicOutflow);
        assert_eq!(g, int);
        // subsonic outflow (+x): entropy and tangential velocity from the interior, Riemann invariants reproduced
        let int = [1.1, 0.4, 0.2, 0.8];
        let (g, r) = characteristic_ghost(&fs, &int, 1.0, 0.0);
        assert_eq!(r, Regime::SubsonicOutflow);
        assert!((g[2] - int[2]).abs() < 1e-14, "tangential velocity must come from the interior");
        assert!((s(&g) - s(&int)).abs() < 1e-13, "entropy must come from the interior");
        let g1 = GAMMA - 1.0;
        let (ci, ce, cb) = (sound_speed(&int), sound_speed(&fs), sound_speed(&g));
        assert!((g[1] + 2.0 * cb / g1 - (int[1] + 2.0 * ci / g1)).abs() < 1e-12);
        assert!((g[1] - 2.0 * cb / g1 - (fs[1] - 2.0 * ce / g1)).abs() < 1e-12);
        // subsonic inflow (outward normal -x): entropy and tangential velocity from the free stream
        let (g, r) = characteristic_ghost(&fs, &int, -1.0, 0.0);
        assert_eq!(r, Regime::SubsonicInflow);
        assert!((g[2] - fs[2]).abs() < 1e-14, "tangential velocity must come from the free stream");
        assert!((s(&g) - s(&fs)).abs() < 1e-13, "entropy must come from the free stream");
    }

    #[test]
    fn freestream_is_a_fixed_point_for_any_normal() {
        for mach in [0.3, 0.5, 0.9, 0.98, 1.0, 1.1, 2.0] {
            let fs = freestream(mach);
            for k in 0..16 {
                let a = std::f64::consts::TAU * k as f64 / 16.0;
                let (g, _) = characteristic_ghost(&fs, &fs, a.cos(), a.sin());
                for q in 0..4 {
                    assert!((g[q] - fs[q]).abs() < 1e-13, "M={mach} a={a}: {g:?} vs {fs:?}");
                }
            }
        }
    }

    #[test]
    fn pressure_outflow_sets_static_pressure_only_when_subsonic() {
        let int = [1.0, 0.5, 0.1, 0.9];
        let g = pressure_outflow_ghost(0.7, &int, 1.0, 0.0);
        assert_eq!(g, [1.0, 0.5, 0.1, 0.7]);
        let int = [1.0, 1.5, 0.1, 0.9];
        assert_eq!(pressure_outflow_ghost(0.7, &int, 1.0, 0.0), int);
    }
}
