//! Large-domain, stretched body-fitted grid for ogive-cylinder / cone-cylinder bodies in subsonic and
//! transonic flow, where the far field must be many body lengths away (the existing `cone_grid` /
//! `ogive_grid` size the domain from the attached-shock angle and are not suitable).
//!
//! Lengths are in body diameters, `d = 1` (radius `R = 0.5`), the nose tip is at the origin. The grid is an
//! H-grid like the existing ones: vertical lines `x = x_i`, lower boundary = axis for `x < 0` and the body
//! for `x >= 0`, flat outer boundary `r = r_far`.
//!
//! x distribution (all segments share the tip/shoulder end-cell size `dx_tip`, so the cell size is continuous):
//!
//! 1. upstream `[-x_up, 0]`, `n_up` cells, geometric growth away from the tip (first cell `dx_tip`);
//! 2. nose `[0, L_n]`, `n_nose` cells, symmetric `tanh` clustering at the tip **and** the shoulder, the strength
//!    solved so that the end cells are `dx_tip` (uniform if `dx_tip >= L_n/n_nose`);
//! 3. cylinder `[L_n, L_n + cyl_length]`, `n_cyl` cells, geometric growth from the shoulder cell size;
//! 4. extension (sting) `[L_n + cyl_length, L_n + x_down]`, `n_ext` cells, geometric growth continuing from the last
//!    cylinder cell. The body stays a cylinder (sting) up to the outflow plane: there is no base, so base/wake
//!    effects cannot contaminate the forebody, the zero-lift pressure drag of the nose is the only drag source,
//!    and the outflow boundary is far away and coarse.
//!
//! r distribution: geometric from the wall (`dr_wall` first cell, growth solved so that `nj` cells reach `r_far`),
//! the same fractions in every column: `r = r_body(x) + s_j (r_far - r_body(x))`.

use crate::grid::Grid;
use crate::shock_expansion::Ogive;

/// Nose shape of the body (diameter `d = 1`, nose length `L_n`).
#[derive(Clone, Copy, Debug)]
pub enum Nose {
    /// Circular-arc tangent ogive of the given fineness `L_n/d`.
    TangentOgive { fineness: f64 },
    /// Cone of half angle `theta` [rad], nose length `R/tan(theta)`.
    Cone { half_angle: f64 },
    /// Power series `r/R = (x/L_n)^n` (`n = 1` cone, `n = 1/2` parabola with its vertex at the tip).
    Power { fineness: f64, n: f64 },
    /// Parabolic series `r/R = (2 eta - K eta^2)/(2 - K)`, `eta = x/L_n` (`K = 0` cone, `K = 1` tangent parabola).
    Parabolic { fineness: f64, k: f64 },
    /// Haack series `r/R = sqrt((theta - sin(2 theta)/2 + C sin^3 theta)/pi)`, `theta = acos(1 - 2 eta)`
    /// (`C = 0` Von Karman / LD-Haack, `C = 1/3` LV-Haack).
    Haack { fineness: f64, c: f64 },
}

impl Nose {
    /// Cone with fineness `L_n/d` (half angle `atan(1/(2 L_n/d))`).
    pub fn cone_fineness(fineness: f64) -> Nose {
        Nose::Cone { half_angle: (0.5 / fineness).atan() }
    }
}

/// Nose plus cylinder, `d = 1`.
#[derive(Clone, Copy, Debug)]
pub struct Body {
    pub nose: Nose,
}

impl Body {
    pub fn radius(&self) -> f64 {
        0.5
    }
    pub fn nose_length(&self) -> f64 {
        match self.nose {
            Nose::TangentOgive { fineness } | Nose::Power { fineness, .. } | Nose::Parabolic { fineness, .. } | Nose::Haack { fineness, .. } => {
                fineness
            }
            Nose::Cone { half_angle } => self.radius() / half_angle.tan(),
        }
    }
    /// Body radius at `x` (0 upstream of the tip, `R` on the cylinder).
    pub fn r(&self, x: f64) -> f64 {
        if x <= 0.0 {
            return 0.0;
        }
        let (ln, rr) = (self.nose_length(), self.radius());
        if x >= ln {
            return rr;
        }
        let eta = x / ln;
        match self.nose {
            Nose::TangentOgive { fineness } => Ogive { fineness }.r(x),
            Nose::Cone { half_angle } => x * half_angle.tan(),
            Nose::Power { n, .. } => rr * eta.powf(n),
            Nose::Parabolic { k, .. } => rr * (2.0 * eta - k * eta * eta) / (2.0 - k),
            Nose::Haack { c, .. } => {
                let th = (1.0 - 2.0 * eta).clamp(-1.0, 1.0).acos();
                rr * ((th - 0.5 * (2.0 * th).sin() + c * th.sin().powi(3)) / std::f64::consts::PI).max(0.0).sqrt()
            }
        }
    }
    /// Half angle of the tip [rad] (tangent cone; `pi/2` for tips with infinite slope).
    pub fn tip_half_angle(&self) -> f64 {
        self.slope(1e-9 * self.nose_length().max(1.0)).atan()
    }
    /// Surface slope `dr/dx` on the nose (0 on the cylinder): analytic for ogive/cone, central difference otherwise.
    pub fn slope(&self, x: f64) -> f64 {
        let ln = self.nose_length();
        if x >= ln {
            return 0.0;
        }
        match self.nose {
            Nose::TangentOgive { fineness } => Ogive { fineness }.slope(x),
            Nose::Cone { half_angle } => half_angle.tan(),
            _ => {
                let h = 1e-6 * ln;
                let (a, b) = ((x - h).max(0.0), (x + h).min(ln));
                (self.r(b) - self.r(a)) / (b - a)
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct LargeGridParams {
    pub body: Body,
    /// Cylinder length behind the shoulder inside the geometric-growth cylinder segment.
    pub cyl_length: f64,
    /// Distance of the inflow plane upstream of the tip.
    pub x_up: f64,
    /// Distance of the outflow plane behind the shoulder (`> cyl_length`); the sting continues to it.
    pub x_down: f64,
    /// Outer boundary radius.
    pub r_far: f64,
    pub n_up: usize,
    pub n_nose: usize,
    pub n_cyl: usize,
    pub n_ext: usize,
    pub nj: usize,
    /// Cell size at the nose tip and at the shoulder (x direction).
    pub dx_tip: f64,
    /// First cell height at the wall (and at the axis upstream of the tip).
    pub dr_wall: f64,
}

/// Node offsets `0 = s_0 < s_1 < ... < s_n = length` with spacing `first * q^k`, `q` solved so that they sum to `length`.
pub fn geometric_offsets(first: f64, length: f64, n: usize) -> Vec<f64> {
    assert!(n >= 1 && first > 0.0 && length > 0.0);
    let sum = |q: f64| {
        let (mut s, mut d) = (0.0, first);
        for _ in 0..n {
            s += d;
            d *= q;
        }
        s
    };
    let (mut lo, mut hi) = (0.2f64, 4.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if sum(mid) < length {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let q = 0.5 * (lo + hi);
    let mut v = Vec::with_capacity(n + 1);
    let (mut s, mut d) = (0.0, first);
    for _ in 0..n {
        v.push(s);
        s += d;
        d *= q;
    }
    v.push(length); // closes any residual of the bisection exactly
    v
}

/// Symmetric two-sided clustering of `n` cells on `[0, len]` with end-cell size `dx` (uniform if `dx >= len/n`).
pub fn two_sided_nodes(len: f64, n: usize, dx: f64) -> Vec<f64> {
    let node = |delta: f64, k: usize| {
        let t = k as f64 / n as f64;
        if delta < 1e-9 {
            len * t
        } else {
            0.5 * len * (1.0 + (delta * (t - 0.5)).tanh() / (0.5 * delta).tanh())
        }
    };
    if dx >= len / n as f64 {
        return (0..=n).map(|k| node(0.0, k)).collect();
    }
    let (mut lo, mut hi) = (0.0f64, 40.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if node(mid, 1) > dx {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let delta = 0.5 * (lo + hi);
    (0..=n).map(|k| node(delta, k)).collect()
}

/// Axial node positions (increasing) of the large-domain grid, `n_up + n_nose + n_cyl + n_ext + 1` entries.
pub fn axial_nodes(p: &LargeGridParams) -> Vec<f64> {
    assert!(p.x_up > 0.0 && p.x_down > p.cyl_length && p.cyl_length > 0.0);
    assert!(p.n_up >= 1 && p.n_nose >= 2 && p.n_cyl >= 1 && p.n_ext >= 1);
    let ln = p.body.nose_length();
    let nose = two_sided_nodes(ln, p.n_nose, p.dx_tip);
    let d_tip = nose[1] - nose[0];
    let d_sh = nose[p.n_nose] - nose[p.n_nose - 1];
    let mut xs = Vec::with_capacity(p.n_up + p.n_nose + p.n_cyl + p.n_ext + 1);
    let up = geometric_offsets(d_tip, p.x_up, p.n_up);
    for k in (1..=p.n_up).rev() {
        xs.push(-up[k]);
    }
    xs.extend(nose.iter().copied()); // x = 0 ..= L_n
    let cyl = geometric_offsets(d_sh, p.cyl_length, p.n_cyl);
    for k in 1..=p.n_cyl {
        xs.push(ln + cyl[k]);
    }
    let d_last = cyl[p.n_cyl] - cyl[p.n_cyl - 1];
    let ext = geometric_offsets(d_last, p.x_down - p.cyl_length, p.n_ext);
    for k in 1..=p.n_ext {
        xs.push(ln + p.cyl_length + ext[k]);
    }
    xs
}

/// Radial fractions `0 = s_0 < ... < s_nj = 1` (geometric, first cell `dr_wall / r_far`).
pub fn radial_fractions(dr_wall: f64, r_far: f64, nj: usize) -> Vec<f64> {
    geometric_offsets(dr_wall / r_far, 1.0, nj)
}

/// Build the large-domain grid.
pub fn large_domain_grid(p: &LargeGridParams) -> Grid {
    let xs = axial_nodes(p);
    let ni = xs.len() - 1;
    let nj = p.nj;
    let sj = radial_fractions(p.dr_wall, p.r_far, nj);
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = vec![0.0; (ni + 1) * (nj + 1)];
    for i in 0..=ni {
        let r_in = p.body.r(xs[i]);
        for j in 0..=nj {
            x[j * (ni + 1) + i] = xs[i];
            r[j * (ni + 1) + i] = r_in + sj[j] * (p.r_far - r_in);
        }
    }
    Grid::from_nodes(ni, nj, x, r)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> LargeGridParams {
        LargeGridParams {
            body: Body { nose: Nose::TangentOgive { fineness: 3.0 } },
            cyl_length: 7.0,
            x_up: 50.0,
            x_down: 50.0,
            r_far: 50.0,
            n_up: 30,
            n_nose: 40,
            n_cyl: 30,
            n_ext: 30,
            nj: 40,
            dx_tip: 0.02,
            dr_wall: 0.02,
        }
    }

    #[test]
    fn nodes_hit_tip_shoulder_and_extents_with_requested_end_cells() {
        let p = params();
        let xs = axial_nodes(&p);
        assert_eq!(xs.len(), 30 + 40 + 30 + 30 + 1);
        assert!(xs.windows(2).all(|w| w[1] > w[0]));
        assert!((xs[0] + 50.0).abs() < 1e-12);
        assert!(xs[30].abs() < 1e-12, "tip node");
        assert!((xs[70] - 3.0).abs() < 1e-12, "shoulder node");
        assert!((xs[100] - 10.0).abs() < 1e-12);
        assert!((xs[130] - 53.0).abs() < 1e-12, "outflow plane");
        assert!((xs[31] - xs[30] - 0.02).abs() < 1e-9, "tip cell");
        assert!((xs[70] - xs[69] - 0.02).abs() < 1e-9, "shoulder cell");
        assert!((xs[30] - xs[29] - 0.02).abs() < 1e-9, "upstream first cell");
        // growth ratios moderate everywhere (stretching smoothness)
        let worst = xs.windows(3).map(|w| ((w[2] - w[1]) / (w[1] - w[0])).max((w[1] - w[0]) / (w[2] - w[1]))).fold(1.0, f64::max);
        assert!(worst < 1.35, "max neighbouring cell-size ratio {worst}");
    }

    #[test]
    fn grid_is_valid_and_wall_and_far_field_are_where_expected() {
        for nose in [Nose::TangentOgive { fineness: 3.0 }, Nose::Cone { half_angle: 10f64.to_radians() }] {
            let p = LargeGridParams { body: Body { nose }, ..params() };
            let g = large_domain_grid(&p); // asserts positive cell areas
            let (ni, nj) = (g.ni, g.nj);
            for i in 0..=ni {
                let (x0, r0) = g.node(i, 0);
                assert!((r0 - p.body.r(x0)).abs() < 1e-12);
                let (_, rt) = g.node(i, nj);
                assert!((rt - p.r_far).abs() < 1e-12);
            }
            // first wall cell height equals dr_wall (far from the axis)
            let i = ni - 1;
            assert!((g.node(i, 1).1 - g.node(i, 0).1 - p.dr_wall * (p.r_far - 0.5) / p.r_far).abs() < 1e-9);
            // body radius reaches R on the cylinder
            assert!((p.body.r(5.0) - 0.5).abs() < 1e-14);
        }
    }

    #[test]
    fn nose_shapes_reach_the_shoulder_radius_monotonically() {
        let f = 3.0;
        for nose in [
            Nose::cone_fineness(f),
            Nose::TangentOgive { fineness: f },
            Nose::Power { fineness: f, n: 0.5 },
            Nose::Parabolic { fineness: f, k: 1.0 },
            Nose::Haack { fineness: f, c: 0.0 },
            Nose::Haack { fineness: f, c: 1.0 / 3.0 },
        ] {
            let b = Body { nose };
            assert!((b.nose_length() - f).abs() < 1e-12, "{nose:?}");
            assert!((b.r(f - 1e-12) - 0.5).abs() < 1e-6, "{nose:?} shoulder radius {}", b.r(f - 1e-12));
            let mut prev = 0.0;
            for k in 1..=300 {
                let r = b.r(f * k as f64 / 300.0);
                assert!(r > prev, "{nose:?} not monotone at {k}");
                prev = r;
            }
        }
        // fineness-3 cone half angle = atan(0.5/3) = 9.46 deg
        assert!((Body { nose: Nose::cone_fineness(3.0) }.tip_half_angle().to_degrees() - 9.4623).abs() < 1e-3);
        // parabolic K = 0 is the cone
        let (p, c) = (Body { nose: Nose::Parabolic { fineness: f, k: 0.0 } }, Body { nose: Nose::cone_fineness(f) });
        assert!((p.r(1.3) - c.r(1.3)).abs() < 1e-12);
    }

    #[test]
    fn cone_geometry() {
        let b = Body { nose: Nose::Cone { half_angle: 10f64.to_radians() } };
        assert!((b.r(b.nose_length()) - 0.5).abs() < 1e-14);
        assert!((b.nose_length() - 0.5 / 10f64.to_radians().tan()).abs() < 1e-14);
        assert!((b.slope(1.0) - 10f64.to_radians().tan()).abs() < 1e-14);
    }
}
