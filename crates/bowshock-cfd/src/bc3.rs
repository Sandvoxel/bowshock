//! 3D characteristic far-field and static-pressure outflow ghost states, ported from [`crate::bc`] with the outward
//! unit normal `n: [f64; 3]`. The Riemann-invariant logic acts on the normal velocity; the tangential velocity vector
//! is taken from the upwind side (free stream on inflow, interior on outflow). The 2D [`Regime`] is reused.

use crate::bc::Regime;
use crate::gas3::{sound_speed3, Prim3, GAMMA};

/// Normal Mach number `v.n / c` of `w` for the outward unit normal `n`.
#[inline]
pub fn normal_mach3(w: &Prim3, n: [f64; 3]) -> f64 {
    (w[1] * n[0] + w[2] * n[1] + w[3] * n[2]) / sound_speed3(w)
}

/// Regime of a face from the interior state and the outward normal.
#[inline]
pub fn regime3(interior: &Prim3, n: [f64; 3]) -> Regime {
    let m = normal_mach3(interior, n);
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
/// `n` the outward unit normal. Returns the regime used as well.
pub fn characteristic_ghost3(fs: &Prim3, interior: &Prim3, n: [f64; 3]) -> (Prim3, Regime) {
    let reg = regime3(interior, n);
    let g1 = GAMMA - 1.0;
    let ghost = match reg {
        Regime::SupersonicInflow => *fs,
        Regime::SupersonicOutflow => *interior,
        Regime::SubsonicInflow | Regime::SubsonicOutflow => {
            let (ci, ce) = (sound_speed3(interior), sound_speed3(fs));
            let uni = interior[1] * n[0] + interior[2] * n[1] + interior[3] * n[2];
            let une = fs[1] * n[0] + fs[2] * n[1] + fs[3] * n[2];
            let r_out = uni + 2.0 * ci / g1;
            let r_in = une - 2.0 * ce / g1;
            let un_b = 0.5 * (r_out + r_in);
            let c_b = (0.25 * g1 * (r_out - r_in)).max(1e-8);
            // reference state carrying entropy and tangential velocity
            let (rf, un_ref) = if un_b >= 0.0 { (interior, uni) } else { (fs, une) };
            let s = rf[4] / rf[0].powf(GAMMA);
            let rho = (c_b * c_b / (GAMMA * s)).powf(1.0 / g1);
            let p = rho * c_b * c_b / GAMMA;
            let dn = un_b - un_ref;
            [rho, rf[1] + dn * n[0], rf[2] + dn * n[1], rf[3] + dn * n[2], p]
        }
    };
    (ghost, reg)
}

/// Fixed static-pressure outflow: subsonic faces (`M_n < 1`) take `rho, u, v, w` from the interior and
/// `p = p_back`; supersonic faces extrapolate.
pub fn pressure_outflow_ghost3(p_back: f64, interior: &Prim3, n: [f64; 3]) -> Prim3 {
    if normal_mach3(interior, n) >= 1.0 {
        *interior
    } else {
        [interior[0], interior[1], interior[2], interior[3], p_back]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bc;
    use crate::flux3::tests::{random_rotation, rot, rot5, Rng};
    use crate::gas3::freestream3;

    fn unit(rng: &mut Rng) -> [f64; 3] {
        let n = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        [n[0] / l, n[1] / l, n[2] / l]
    }

    fn state(rng: &mut Rng, vmax: f64) -> Prim3 {
        [rng.range(0.5, 2.0), rng.range(-vmax, vmax), rng.range(-vmax, vmax), rng.range(-vmax, vmax), rng.range(0.5, 2.0)]
    }

    #[test]
    fn matches_2d_in_xz_plane() {
        let mut rng = Rng::new(5);
        let mut seen = [false; 4];
        for it in 0..4000 {
            let vmax = [0.3, 1.0, 3.0][it % 3];
            let t = rng.range(0.0, std::f64::consts::TAU);
            let (nx, nr) = (t.cos(), t.sin());
            let n = [nx, 0.0, nr];
            let mut w = state(&mut rng, vmax);
            w[2] = 0.0;
            let w2 = [w[0], w[1], w[3], w[4]];
            let fs3 = freestream3(rng.range(0.3, 3.0), rng.range(-0.3, 0.3));
            let fs2 = [fs3[0], fs3[1], fs3[3], fs3[4]];
            let (g3, r3) = characteristic_ghost3(&fs3, &w, n);
            let (g2, r2) = bc::characteristic_ghost(&fs2, &w2, nx, nr);
            assert_eq!(r3, r2);
            seen[r3 as usize] = true;
            assert_eq!(g3[2], 0.0);
            for (a, b) in [(g3[0], g2[0]), (g3[1], g2[1]), (g3[3], g2[2]), (g3[4], g2[3])] {
                assert!((a - b).abs() < 1e-14 * (1.0 + b.abs()), "{a} vs {b}");
            }
            let pb = rng.range(0.3, 1.5);
            let o3 = pressure_outflow_ghost3(pb, &w, n);
            let o2 = bc::pressure_outflow_ghost(pb, &w2, nx, nr);
            assert_eq!(o3[2], 0.0);
            for (a, b) in [(o3[0], o2[0]), (o3[1], o2[1]), (o3[3], o2[2]), (o3[4], o2[3])] {
                assert!((a - b).abs() < 1e-14);
            }
        }
        assert!(seen.iter().all(|&s| s), "every regime exercised: {seen:?}");
    }

    #[test]
    fn rotation_invariance() {
        let mut rng = Rng::new(9);
        for it in 0..4000 {
            let vmax = [0.3, 1.0, 3.0][it % 3];
            let r = random_rotation(&mut rng);
            let n = unit(&mut rng);
            let w = state(&mut rng, vmax);
            let fs = freestream3(rng.range(0.3, 3.0), rng.range(-0.3, 0.3));
            let (g, reg) = characteristic_ghost3(&fs, &w, n);
            let (gr, regr) = characteristic_ghost3(&rot5(&r, &fs), &rot5(&r, &w), rot(&r, n));
            assert_eq!(reg, regr);
            let want = rot5(&r, &g);
            for k in 0..5 {
                assert!((want[k] - gr[k]).abs() < 1e-12 * (1.0 + want[k].abs()));
            }
            let pb = rng.range(0.3, 1.5);
            let o = pressure_outflow_ghost3(pb, &w, n);
            let or = pressure_outflow_ghost3(pb, &rot5(&r, &w), rot(&r, n));
            let want = rot5(&r, &o);
            for k in 0..5 {
                assert!((want[k] - or[k]).abs() < 1e-12 * (1.0 + want[k].abs()));
            }
        }
    }
}
