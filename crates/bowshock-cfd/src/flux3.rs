//! 3D face kernels (Cartesian), ported one-to-one from [`crate::flux`]: MUSCL reconstruction, HLLC, Rusanov, the
//! shock-sensor blend and the low-Mach correction, all projected on a unit normal `n: [f64; 3]`.
//! The limiters, `limited_slope`, `shock_weight` and the sensor / low-Mach constants are shared with 2D.

use crate::flux::{limited_slope, shock_weight, Limiter, LOW_MACH_FLOOR};
use crate::gas3::{physical_flux3, Cons3, Prim3, GAMMA};

/// Cylindrical frame about the x axis at a face centroid `c`: `[sin, cos]` of its azimuth, `[c_y / r, c_z / r]` with
/// `r = hypot(c_y, c_z)`; `[0, 1]` on the axis.
#[inline]
pub fn face_rot(c: [f64; 3]) -> [f64; 2] {
    let r = c[1].hypot(c[2]);
    if r < 1e-300 {
        [0.0, 1.0]
    } else {
        [c[1] / r, c[2] / r]
    }
}

/// MUSCL reconstruction of the primitive variables at the face between cells `m1` (left) and `p0` (right);
/// `m2` and `p1` are the outer neighbours. The lateral velocity is limited in the face's cylindrical frame about the x
/// axis (`rot = face_rot(centroid)`: radial `v s + w co`, tangential `v co - w s`), so axisymmetric flow stays exactly
/// axisymmetric; the reconstructed states are transformed back. Falls back to first order for a side whose
/// reconstructed density or pressure is not positive.
#[inline]
pub fn reconstruct3(m2: &Prim3, m1: &Prim3, p0: &Prim3, p1: &Prim3, lim: Limiter, rot: [f64; 2]) -> (Prim3, Prim3) {
    let [s, co] = rot;
    let fwd = |w: &Prim3| [w[0], w[1], w[2] * s + w[3] * co, w[2] * co - w[3] * s, w[4]];
    let (a, b, c, d) = (fwd(m2), fwd(m1), fwd(p0), fwd(p1));
    let mut l = b;
    let mut r = c;
    for k in 0..5 {
        let phi_l = limited_slope(b[k] - a[k], c[k] - b[k], lim);
        let phi_r = limited_slope(c[k] - b[k], d[k] - c[k], lim);
        l[k] = b[k] + 0.5 * phi_l;
        r[k] = c[k] - 0.5 * phi_r;
    }
    if l[0] <= 0.0 || l[4] <= 0.0 {
        l = b;
    }
    if r[0] <= 0.0 || r[4] <= 0.0 {
        r = c;
    }
    let back = |w: &Prim3, orig: &Prim3, fallback: bool| {
        if fallback {
            *orig
        } else {
            [w[0], w[1], w[2] * s + w[3] * co, w[2] * co - w[3] * s, w[4]]
        }
    };
    (back(&l, m1, l == b), back(&r, p0, r == c))
}

/// HLLC flux for the 3D Euler equations projected on the unit normal `n` (see [`crate::flux::hllc`]). The contact
/// wave carries both tangential velocity components passively. Returns `F.n` per unit face area.
#[inline]
pub fn hllc3(wl: &Prim3, wr: &Prim3, n: [f64; 3]) -> Cons3 {
    let [rl, ul, vl, wl_z, pl] = *wl;
    let [rr, ur, vr, wr_z, pr] = *wr;
    let unl = ul * n[0] + vl * n[1] + wl_z * n[2];
    let unr = ur * n[0] + vr * n[1] + wr_z * n[2];
    let cl = (GAMMA * pl / rl).sqrt();
    let cr = (GAMMA * pr / rr).sqrt();
    let el = pl / (GAMMA - 1.0) + 0.5 * rl * (ul * ul + vl * vl + wl_z * wl_z);
    let er = pr / (GAMMA - 1.0) + 0.5 * rr * (ur * ur + vr * vr + wr_z * wr_z);

    // Roe averages
    let sql = rl.sqrt();
    let sqr = rr.sqrt();
    let inv = 1.0 / (sql + sqr);
    let uh = (sql * ul + sqr * ur) * inv;
    let vh = (sql * vl + sqr * vr) * inv;
    let wh = (sql * wl_z + sqr * wr_z) * inv;
    let hl = (el + pl) / rl;
    let hr = (er + pr) / rr;
    let hh = (sql * hl + sqr * hr) * inv;
    let ch2 = (GAMMA - 1.0) * (hh - 0.5 * (uh * uh + vh * vh + wh * wh));
    let ch = ch2.max(1e-300).sqrt();
    let unh = uh * n[0] + vh * n[1] + wh * n[2];

    let sl = (unl - cl).min(unh - ch);
    let sr = (unr + cr).max(unh + ch);
    if sl >= 0.0 {
        return physical_flux3(wl, n);
    }
    if sr <= 0.0 {
        return physical_flux3(wr, n);
    }

    let sm = (pr - pl + rl * unl * (sl - unl) - rr * unr * (sr - unr)) / (rl * (sl - unl) - rr * (sr - unr));

    if sm >= 0.0 {
        let f = physical_flux3(wl, n);
        let fac = rl * (sl - unl) / (sl - sm);
        let d = sm - unl;
        let star = [fac, fac * (ul + d * n[0]), fac * (vl + d * n[1]), fac * (wl_z + d * n[2]), fac * (el / rl + d * (sm + pl / (rl * (sl - unl))))];
        let q = [rl, rl * ul, rl * vl, rl * wl_z, el];
        let mut out = [0.0; 5];
        for k in 0..5 {
            out[k] = f[k] + sl * (star[k] - q[k]);
        }
        out
    } else {
        let f = physical_flux3(wr, n);
        let fac = rr * (sr - unr) / (sr - sm);
        let d = sm - unr;
        let star = [fac, fac * (ur + d * n[0]), fac * (vr + d * n[1]), fac * (wr_z + d * n[2]), fac * (er / rr + d * (sm + pr / (rr * (sr - unr))))];
        let q = [rr, rr * ur, rr * vr, rr * wr_z, er];
        let mut out = [0.0; 5];
        for k in 0..5 {
            out[k] = f[k] + sr * (star[k] - q[k]);
        }
        out
    }
}

/// Rusanov (local Lax-Friedrichs) flux projected on `n`: `0.5 (F_L + F_R) - 0.5 smax (U_R - U_L)` with
/// `smax = max(|un_L| + c_L, |un_R| + c_R)`.
#[inline]
pub fn rusanov3(wl: &Prim3, wr: &Prim3, n: [f64; 3]) -> Cons3 {
    let [rl, ul, vl, zl, pl] = *wl;
    let [rr, ur, vr, zr, pr] = *wr;
    let unl = ul * n[0] + vl * n[1] + zl * n[2];
    let unr = ur * n[0] + vr * n[1] + zr * n[2];
    let smax = (unl.abs() + (GAMMA * pl / rl).sqrt()).max(unr.abs() + (GAMMA * pr / rr).sqrt());
    let fl = physical_flux3(wl, n);
    let fr = physical_flux3(wr, n);
    let ql = [rl, rl * ul, rl * vl, rl * zl, pl / (GAMMA - 1.0) + 0.5 * rl * (ul * ul + vl * vl + zl * zl)];
    let qr = [rr, rr * ur, rr * vr, rr * zr, pr / (GAMMA - 1.0) + 0.5 * rr * (ur * ur + vr * vr + zr * zr)];
    let mut out = [0.0; 5];
    for k in 0..5 {
        out[k] = 0.5 * (fl[k] + fr[k]) - 0.5 * smax * (qr[k] - ql[k]);
    }
    out
}

/// Complete face kernel: MUSCL reconstruction + HLLC, blended towards Rusanov inside shocks
/// (see [`crate::flux::face_flux`]).
#[inline]
pub fn face_flux3(m2: &Prim3, m1: &Prim3, p0: &Prim3, p1: &Prim3, lim: Limiter, n: [f64; 3], rot: [f64; 2]) -> Cons3 {
    let (l, r) = reconstruct3(m2, m1, p0, p1, lim, rot);
    blend3(&l, &r, m1[4], p0[4], n)
}

#[inline]
fn blend3(l: &Prim3, r: &Prim3, pl: f64, pr: f64, n: [f64; 3]) -> Cons3 {
    let c = hllc3(l, r, n);
    let t = shock_weight(pl, pr);
    if t == 0.0 {
        return c;
    }
    let f = rusanov3(l, r, n);
    let mut out = c;
    for k in 0..5 {
        out[k] = (1.0 - t) * c[k] + t * f[k];
    }
    out
}

/// Low-Mach correction of the reconstructed face states: scales only the jump of the face-normal velocity by
/// `z = clamp(max(M_L, M_R), LOW_MACH_FLOOR, 1)` (see [`crate::flux::low_mach_states`]).
#[inline]
pub fn low_mach_states3(l: &Prim3, r: &Prim3, n: [f64; 3]) -> (Prim3, Prim3) {
    let unl = l[1] * n[0] + l[2] * n[1] + l[3] * n[2];
    let unr = r[1] * n[0] + r[2] * n[1] + r[3] * n[2];
    let ml = (l[1] * l[1] + l[2] * l[2] + l[3] * l[3]).sqrt() / (GAMMA * l[4] / l[0]).sqrt();
    let mr = (r[1] * r[1] + r[2] * r[2] + r[3] * r[3]).sqrt() / (GAMMA * r[4] / r[0]).sqrt();
    let z = ml.max(mr).clamp(LOW_MACH_FLOOR, 1.0);
    if z >= 1.0 {
        return (*l, *r);
    }
    let avg = 0.5 * (unl + unr);
    let dl = avg + z * (unl - avg) - unl;
    let dr = avg + z * (unr - avg) - unr;
    ([l[0], l[1] + dl * n[0], l[2] + dl * n[1], l[3] + dl * n[2], l[4]], [r[0], r[1] + dr * n[0], r[2] + dr * n[1], r[3] + dr * n[2], r[4]])
}

/// [`face_flux3`] with the optional low-Mach correction; `low_mach = false` is exactly [`face_flux3`].
#[inline]
pub fn face_flux_lm3(m2: &Prim3, m1: &Prim3, p0: &Prim3, p1: &Prim3, lim: Limiter, n: [f64; 3], rot: [f64; 2], low_mach: bool) -> Cons3 {
    if !low_mach {
        return face_flux3(m2, m1, p0, p1, lim, n, rot);
    }
    let (l, r) = reconstruct3(m2, m1, p0, p1, lim, rot);
    let (l, r) = low_mach_states3(&l, &r, n);
    blend3(&l, &r, m1[4], p0[4], n)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::flux;

    pub(crate) struct Rng(u64);
    impl Rng {
        pub(crate) fn new(seed: u64) -> Self {
            Rng(seed)
        }
        pub(crate) fn next(&mut self) -> f64 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
        }
        pub(crate) fn range(&mut self, a: f64, b: f64) -> f64 {
            a + (b - a) * self.next()
        }
    }

    pub(crate) type Rot = [[f64; 3]; 3];

    pub(crate) fn random_rotation(rng: &mut Rng) -> Rot {
        let mut a = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
        let l = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
        for x in &mut a {
            *x /= l;
        }
        let th = rng.range(0.0, std::f64::consts::TAU);
        let (s, c) = th.sin_cos();
        let k = [[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]];
        let mut r = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let kk: f64 = (0..3).map(|m| k[i][m] * k[m][j]).sum();
                r[i][j] = if i == j { 1.0 } else { 0.0 } + s * k[i][j] + (1.0 - c) * kk;
            }
        }
        r
    }

    pub(crate) fn rot(r: &Rot, v: [f64; 3]) -> [f64; 3] {
        [r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2], r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2], r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2]]
    }

    /// Rotate the velocity (indices 1..4) of a primitive or conservative vector.
    pub(crate) fn rot5(r: &Rot, q: &[f64; 5]) -> [f64; 5] {
        let v = rot(r, [q[1], q[2], q[3]]);
        [q[0], v[0], v[1], v[2], q[4]]
    }

    fn rand_state(rng: &mut Rng, planar: bool, vmax: f64) -> Prim3 {
        let v = if planar { 0.0 } else { rng.range(-vmax, vmax) };
        [rng.range(0.5, 2.0), rng.range(-vmax, vmax), v, rng.range(-vmax, vmax), rng.range(0.5, 2.0)]
    }

    fn to2(w: &Prim3) -> [f64; 4] {
        [w[0], w[1], w[3], w[4]]
    }

    fn close4(f3: &Cons3, f2: &[f64; 4]) {
        assert_eq!(f3[2], 0.0, "y-momentum flux must vanish");
        for (a, b) in [(f3[0], f2[0]), (f3[1], f2[1]), (f3[3], f2[2]), (f3[4], f2[3])] {
            assert!((a - b).abs() < 1e-14 * (1.0 + b.abs()), "{a} vs {b}");
        }
    }

    fn rand_normal_xz(rng: &mut Rng) -> (f64, f64) {
        let t = rng.range(0.0, std::f64::consts::TAU);
        (t.cos(), t.sin())
    }

    #[test]
    fn matches_2d_in_xz_plane() {
        let mut rng = Rng::new(7);
        for it in 0..2000 {
            let vmax = if it % 2 == 0 { 3.0 } else { 0.1 };
            let (nx, nr) = rand_normal_xz(&mut rng);
            let n = [nx, 0.0, nr];
            let s: Vec<Prim3> = (0..4).map(|_| rand_state(&mut rng, true, vmax)).collect();
            // second state close to the first so the shock sensor is exercised at both ends
            let mut s = s;
            if it % 3 == 0 {
                for k in 0..5 {
                    s[2][k] = s[1][k] * (1.0 + 0.01 * rng.range(-1.0, 1.0));
                }
                s[2][2] = 0.0;
            }
            let (a, b) = (&s[1], &s[2]);
            let z = [0.0, 1.0];
            close4(&hllc3(a, b, n), &flux::hllc(&to2(a), &to2(b), nx, nr));
            close4(&rusanov3(a, b, n), &flux::rusanov(&to2(a), &to2(b), nx, nr));
            let (l3, r3) = reconstruct3(&s[0], a, b, &s[3], Limiter::VanAlbada, z);
            let (l2, r2) = flux::reconstruct(&to2(&s[0]), &to2(a), &to2(b), &to2(&s[3]), Limiter::VanAlbada);
            for (x, y) in [(l3, l2), (r3, r2)] {
                assert_eq!(x[2], 0.0);
                for (p, q) in [(x[0], y[0]), (x[1], y[1]), (x[3], y[2]), (x[4], y[3])] {
                    assert!((p - q).abs() < 1e-14);
                }
            }
            let (m3, q3) = low_mach_states3(&l3, &r3, n);
            let (m2, q2) = flux::low_mach_states(&l2, &r2, nx, nr);
            for (x, y) in [(m3, m2), (q3, q2)] {
                assert_eq!(x[2], 0.0);
                for (p, q) in [(x[0], y[0]), (x[1], y[1]), (x[3], y[2]), (x[4], y[3])] {
                    assert!((p - q).abs() < 1e-14);
                }
            }
            for lim in [Limiter::FirstOrder, Limiter::Minmod, Limiter::VanAlbada] {
                close4(&face_flux3(&s[0], a, b, &s[3], lim, n, z), &flux::face_flux(&to2(&s[0]), &to2(a), &to2(b), &to2(&s[3]), lim, nx, nr));
                for lm in [false, true] {
                    close4(&face_flux_lm3(&s[0], a, b, &s[3], lim, n, z, lm), &flux::face_flux_lm(&to2(&s[0]), &to2(a), &to2(b), &to2(&s[3]), lim, nx, nr, lm));
                }
            }
        }
    }

    /// At a general azimuth the cylindrical-frame limiting equals 2D per-component limiting (radial = in-plane).
    #[test]
    fn matches_2d_at_general_azimuth() {
        let mut rng = Rng::new(17);
        for it in 0..2000 {
            let vmax = if it % 2 == 0 { 3.0 } else { 0.1 };
            let th = rng.range(0.0, std::f64::consts::TAU);
            let (st, ct) = th.sin_cos();
            let (nx, nr) = rand_normal_xz(&mut rng);
            let n = [nx, nr * st, nr * ct];
            let rad = rng.range(0.1, 2.0);
            let c = [rng.range(0.0, 1.0), rad * st, rad * ct];
            let rt = face_rot(c);
            let s: Vec<Prim3> = (0..4).map(|_| rand_state(&mut rng, true, vmax)).collect();
            // in-plane state (u, w_r) -> (u, w_r sin, w_r cos)
            let sr: Vec<Prim3> = s.iter().map(|w| [w[0], w[1], w[3] * st, w[3] * ct, w[4]]).collect();
            for lim in [Limiter::FirstOrder, Limiter::Minmod, Limiter::VanAlbada] {
                for lm in [false, true] {
                    let f3 = face_flux_lm3(&sr[0], &sr[1], &sr[2], &sr[3], lim, n, rt, lm);
                    let f2 = flux::face_flux_lm(&to2(&s[0]), &to2(&s[1]), &to2(&s[2]), &to2(&s[3]), lim, nx, nr, lm);
                    let want = [f2[0], f2[1], f2[2] * st, f2[2] * ct, f2[3]];
                    // roundoff of the frame rotation is amplified by the limiter/Riemann solver on these random states
                    let scale = 1.0 + want.iter().fold(0.0f64, |m, x| m.max(x.abs()));
                    for k in 0..5 {
                        assert!((f3[k] - want[k]).abs() < 1e-13 * scale, "{f3:?} vs {want:?}");
                    }
                }
            }
        }
    }

    fn assert_rotates(f: &Cons3, f_rot: &Cons3, r: &Rot) {
        let want = rot5(r, f);
        for k in 0..5 {
            assert!((want[k] - f_rot[k]).abs() < 1e-12 * (1.0 + want[k].abs()), "{want:?} vs {f_rot:?}");
        }
    }

    #[test]
    fn rotation_invariance() {
        let mut rng = Rng::new(11);
        for it in 0..1000 {
            let vmax = if it % 2 == 0 { 3.0 } else { 0.1 };
            let r = random_rotation(&mut rng);
            let mut n = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
            let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            n = [n[0] / l, n[1] / l, n[2] / l];
            let nr = rot(&r, n);
            let s: Vec<Prim3> = (0..4).map(|_| rand_state(&mut rng, false, vmax)).collect();
            let sr: Vec<Prim3> = s.iter().map(|w| rot5(&r, w)).collect();
            assert_rotates(&hllc3(&s[1], &s[2], n), &hllc3(&sr[1], &sr[2], nr), &r);
            assert_rotates(&rusanov3(&s[1], &s[2], n), &rusanov3(&sr[1], &sr[2], nr), &r);
        }
    }

    /// Rotation about the x axis rotates the face's cylindrical frame with it, so the full kernels (all limiters) are
    /// invariant when the centroid is rotated too.
    #[test]
    fn rotation_invariance_about_x() {
        let mut rng = Rng::new(13);
        for it in 0..1000 {
            let vmax = if it % 2 == 0 { 3.0 } else { 0.1 };
            let phi = rng.range(0.0, std::f64::consts::TAU);
            let (sp, cp) = phi.sin_cos();
            let r: Rot = [[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]];
            let n0 = {
                let n = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
                let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                [n[0] / l, n[1] / l, n[2] / l]
            };
            let nr = rot(&r, n0);
            let c0 = [rng.range(0.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
            let c1 = rot(&r, c0);
            let (f0, f1) = (face_rot(c0), face_rot(c1));
            let s: Vec<Prim3> = (0..4).map(|_| rand_state(&mut rng, false, vmax)).collect();
            let sr: Vec<Prim3> = s.iter().map(|w| rot5(&r, w)).collect();
            for lim in [Limiter::FirstOrder, Limiter::Minmod, Limiter::VanAlbada] {
                assert_rotates(&face_flux3(&s[0], &s[1], &s[2], &s[3], lim, n0, f0), &face_flux3(&sr[0], &sr[1], &sr[2], &sr[3], lim, nr, f1), &r);
                for lm in [false, true] {
                    assert_rotates(
                        &face_flux_lm3(&s[0], &s[1], &s[2], &s[3], lim, n0, f0, lm),
                        &face_flux_lm3(&sr[0], &sr[1], &sr[2], &sr[3], lim, nr, f1, lm),
                        &r,
                    );
                }
            }
        }
    }

    #[test]
    fn hllc_consistency() {
        let mut rng = Rng::new(3);
        for _ in 0..1000 {
            let w = rand_state(&mut rng, false, 3.0);
            let mut n = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
            let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            n = [n[0] / l, n[1] / l, n[2] / l];
            let a = hllc3(&w, &w, n);
            let b = physical_flux3(&w, n);
            for k in 0..5 {
                assert!((a[k] - b[k]).abs() < 1e-13 * (1.0 + b[k].abs()), "{a:?} vs {b:?}");
            }
        }
    }
}
