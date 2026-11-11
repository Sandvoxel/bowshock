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
//! 3. aft segments (tubes and cones, see [`BaseBody`]) from `L_n` on; the last segment of the sting grid, if it is a tube,
//!    has `n_seg` cells with geometric growth from the previous cell size, every other segment is clustered two-sided;
//! 4. extension (sting) `[L_n + aft_length, L_n + x_down]`, `n_ext` cells, geometric growth continuing from the last
//!    aft cell. The body stays a sting up to the outflow plane: there is no base, so base/wake
//!    effects cannot contaminate the forebody, the zero-lift pressure drag of the nose is the only drag source,
//!    and the outflow boundary is far away and coarse.
//!
//! r distribution: geometric from the wall (`dr_wall` first cell, growth solved so that `nj` cells reach `r_far`),
//! the same fractions in every column: `r = r_body(x) + s_j (r_far - r_body(x))`.

use bowshock_core::grid::Grid;
use bowshock_reference::shock_expansion::Ogive;

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
            Nose::TangentOgive { fineness } | Nose::Power { fineness, .. } | Nose::Parabolic { fineness, .. } | Nose::Haack { fineness, .. } => fineness,
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

/// Parameters of the large-domain (sting) grid.
#[derive(Clone, Debug)]
pub struct LargeGridParams {
    pub body: BaseBody,
    /// Distance of the inflow plane upstream of the tip.
    pub x_up: f64,
    /// Distance of the outflow plane behind the end of the nose (`> body.aft_length()`); the sting continues to it.
    pub x_down: f64,
    /// Outer boundary radius.
    pub r_far: f64,
    pub n_up: usize,
    pub n_nose: usize,
    /// Cells per aft segment (one entry per `body.segments`).
    pub n_seg: Vec<usize>,
    pub n_ext: usize,
    pub nj: usize,
    /// Cell size at the nose tip and at the segment ends (x direction).
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

/// Axial node positions (increasing) of the large-domain grid, `n_up + n_nose + sum(n_seg) + n_ext + 1` entries.
pub fn axial_nodes(p: &LargeGridParams) -> Vec<f64> {
    let bb = &p.body;
    let nseg = bb.segments.len();
    assert!(p.x_up > 0.0 && p.x_down > bb.aft_length() && nseg >= 1);
    assert!(p.n_seg.len() == nseg && bb.segments.iter().all(|s| s.length > 0.0));
    assert!(p.n_up >= 1 && p.n_nose >= 2 && p.n_seg.iter().all(|&n| n >= 1) && p.n_ext >= 1);
    let ln = bb.body.nose_length();
    let nose = two_sided_nodes(ln, p.n_nose, p.dx_tip);
    let d_tip = nose[1] - nose[0];
    let d_sh = nose[p.n_nose] - nose[p.n_nose - 1];
    let mut xs = Vec::with_capacity(p.n_up + p.n_nose + p.n_seg.iter().sum::<usize>() + p.n_ext + 1);
    let up = geometric_offsets(d_tip, p.x_up, p.n_up);
    for k in (1..=p.n_up).rev() {
        xs.push(-up[k]);
    }
    xs.extend(nose.iter().copied()); // x = 0 ..= L_n
    let mut d_prev = d_sh;
    for (k, seg) in bb.segments.iter().enumerate() {
        let n = p.n_seg[k];
        let x0 = bb.station(k).0;
        let offs = if k == nseg - 1 && seg.slope == 0.0 { geometric_offsets(d_prev, seg.length, n) } else { two_sided_nodes(seg.length, n, p.dx_tip) };
        for i in 1..=n {
            xs.push(x0 + offs[i]);
        }
        d_prev = offs[n] - offs[n - 1];
    }
    let x_base = bb.x_base();
    let ext = geometric_offsets(d_prev, p.x_down - bb.aft_length(), p.n_ext);
    for k in 1..=p.n_ext {
        xs.push(x_base + ext[k]);
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

/// One conical (or cylindrical) piece of the aft body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    pub length: f64,
    /// `dr/dx`; `0.0` is a tube.
    pub slope: f64,
}

/// Nose followed by a list of conical segments (tubes, flares, boattails) and a flat base, on a sting or free (wake behind the base).
///
/// The axial profile is `body.r(x)` up to the end of the nose `x = L_n`, then the segments in order (`r` linear in `x` on each),
/// then a flat annular base of outer radius [`BaseBody::base_radius`] and inner radius
/// `sting_radius` (`0` = no sting, the base is a full disc and a wake region reaches the axis).
#[derive(Clone, Debug)]
pub struct BaseBody {
    pub body: Body,
    pub segments: Vec<Segment>,
    pub sting_radius: f64,
}

impl BaseBody {
    /// Nose followed by one tube of the given length, no sting.
    pub fn cylinder(body: Body, length: f64) -> BaseBody {
        BaseBody { body, segments: vec![Segment { length, slope: 0.0 }], sting_radius: 0.0 }
    }
    /// Append a conical boattail of the given length and half-angle [rad].
    pub fn with_boattail(mut self, length: f64, half_angle: f64) -> BaseBody {
        self.segments.push(Segment { length, slope: -half_angle.tan() });
        self
    }
    /// Set the sting radius behind the base.
    pub fn with_sting(mut self, radius: f64) -> BaseBody {
        self.sting_radius = radius;
        self
    }
    /// `(x, r)` where segment `k` starts; `k == segments.len()` is the base plane.
    pub fn station(&self, k: usize) -> (f64, f64) {
        let (mut x, mut r) = (self.body.nose_length(), self.body.radius());
        for s in &self.segments[..k] {
            x += s.length;
            r += s.length * s.slope;
        }
        (x, r)
    }
    /// Total length of the aft segments.
    pub fn aft_length(&self) -> f64 {
        self.segments.iter().fold(0.0, |a, s| a + s.length)
    }
    /// Axial position of the flat base.
    pub fn x_base(&self) -> f64 {
        self.station(self.segments.len()).0
    }
    /// Outer radius of the base face.
    pub fn base_radius(&self) -> f64 {
        self.station(self.segments.len()).1
    }
    /// Index of the segment containing `x` (`x` between the end of the nose and the base).
    fn segment_at(&self, x: f64) -> Option<(usize, f64, f64)> {
        let (mut x0, mut r0) = (self.body.nose_length(), self.body.radius());
        for (k, s) in self.segments.iter().enumerate() {
            let x1 = x0 + s.length;
            if x < x1 {
                return Some((k, x0, r0));
            }
            x0 = x1;
            r0 += s.length * s.slope;
        }
        None
    }
    /// Body radius at `x` (0 upstream of the tip, base radius behind the base: the line the wall grid row follows).
    pub fn r(&self, x: f64) -> f64 {
        if x <= self.body.nose_length() {
            return self.body.r(x);
        }
        match self.segment_at(x) {
            Some((k, x0, r0)) => r0 + (x - x0) * self.segments[k].slope,
            None => self.base_radius(),
        }
    }
    /// Profile slope `dr/dx` at `x` (0 behind the base).
    pub fn slope(&self, x: f64) -> f64 {
        if x < self.body.nose_length() {
            return self.body.slope(x);
        }
        self.segment_at(x).map_or(0.0, |(k, ..)| self.segments[k].slope)
    }
}

#[derive(Clone, Debug)]
pub struct BaseGridParams {
    pub body: BaseBody,
    /// Inflow plane distance upstream of the tip, outflow plane distance behind the base, outer radius.
    pub x_up: f64,
    pub x_down: f64,
    pub r_far: f64,
    /// Cells upstream of the tip, along the nose, per aft segment (one entry per `body.segments`) and behind the base.
    pub n_up: usize,
    pub n_nose: usize,
    pub n_seg: Vec<usize>,
    pub n_wake: usize,
    /// Rows inside the sting (solid; 0 without sting, otherwise >= 2), rows between the sting (or axis) and the base
    /// radius behind the base (these rows are solid beside the body), rows above the body surface.
    pub n_sting: usize,
    pub n_gap: usize,
    pub nj_out: usize,
    /// End-cell size along x at the tip, shoulder, boattail ends and base, first cell height at walls / base corner.
    pub dx_tip: f64,
    pub dr_wall: f64,
}

/// Grid with a solid body block and base (see `solver::Block`): H-grid whose first `j_end = n_sting + n_gap` rows lie
/// inside the body for `x < x_base` (solid cells) and behind the base between the sting / axis and the base radius (wake
/// cells). Returns the grid and the block to pass to `Solver::set_block`.
pub fn base_domain_grid(p: &BaseGridParams) -> (Grid, bowshock_core::solver::Block) {
    let bb = &p.body;
    let ln = bb.body.nose_length();
    assert!(p.n_up >= 1 && p.n_nose >= 2 && p.n_wake >= 2 && p.n_gap >= 2);
    assert!(!bb.segments.is_empty() && p.n_seg.len() == bb.segments.len() && p.n_seg.iter().all(|&n| n >= 1));
    assert!(bb.segments.iter().all(|s| s.length > 0.0));
    assert!(p.n_sting == 0 || p.n_sting >= 2);
    assert!(bb.sting_radius < bb.base_radius(), "sting thicker than the base");
    let nose = two_sided_nodes(ln, p.n_nose, p.dx_tip);
    let d_tip = nose[1] - nose[0];
    let mut xs: Vec<f64> = Vec::new();
    let up = geometric_offsets(d_tip, p.x_up, p.n_up);
    for k in (1..=p.n_up).rev() {
        xs.push(-up[k]);
    }
    xs.extend(nose.iter().copied());
    for (k, seg) in bb.segments.iter().enumerate() {
        let seg_nodes = two_sided_nodes(seg.length, p.n_seg[k], p.dx_tip);
        let x0 = bb.station(k).0;
        for i in 1..=p.n_seg[k] {
            xs.push(x0 + seg_nodes[i]);
        }
    }
    let i_end = xs.len() - 1; // node index of the base plane is xs.len() - 1 at this point
    let d_base = xs[i_end] - xs[i_end - 1];
    let wake = geometric_offsets(d_base, p.x_down, p.n_wake);
    let x_base = bb.x_base();
    for k in 1..=p.n_wake {
        xs.push(x_base + wake[k]);
    }
    let ni = xs.len() - 1;
    let (j_sting, j_end) = (p.n_sting, p.n_sting + p.n_gap);
    let nj = j_end + p.nj_out;
    let (re, rs) = (bb.base_radius(), bb.sting_radius);
    let gap = two_sided_nodes(re - rs, p.n_gap, p.dr_wall);
    let sj = radial_fractions(p.dr_wall, p.r_far - re, p.nj_out);
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = vec![0.0; (ni + 1) * (nj + 1)];
    for i in 0..=ni {
        let rw = bb.r(xs[i]);
        for j in 0..=nj {
            let rad = if i >= i_end {
                if j <= j_sting {
                    rs - (j_sting - j) as f64 * p.dr_wall
                } else if j <= j_end {
                    rs + gap[j - j_sting]
                } else {
                    re + sj[j - j_end] * (p.r_far - re)
                }
            } else if j <= j_end {
                rw - (j_end - j) as f64 * p.dr_wall
            } else {
                rw + sj[j - j_end] * (p.r_far - rw)
            };
            x[j * (ni + 1) + i] = xs[i];
            r[j * (ni + 1) + i] = rad;
        }
    }
    (Grid::from_nodes(ni, nj, x, r), bowshock_core::solver::Block { i_end, j_end, j_sting })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> LargeGridParams {
        LargeGridParams {
            body: BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 7.0),
            x_up: 50.0,
            x_down: 50.0,
            r_far: 50.0,
            n_up: 30,
            n_nose: 40,
            n_seg: vec![30],
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
            let p = LargeGridParams { body: BaseBody::cylinder(Body { nose }, 7.0), ..params() };
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

    fn base_params(sting: f64, boat: f64) -> BaseGridParams {
        let mut body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 3.0).with_sting(sting);
        if boat > 0.0 {
            body = body.with_boattail(boat, 10f64.to_radians());
        }
        BaseGridParams {
            body,
            x_up: 20.0,
            x_down: 20.0,
            r_far: 20.0,
            n_up: 12,
            n_nose: 24,
            n_seg: if boat > 0.0 { vec![16, 8] } else { vec![16] },
            n_wake: 16,
            n_sting: if sting > 0.0 { 4 } else { 0 },
            n_gap: 8,
            nj_out: 20,
            dx_tip: 0.05,
            dr_wall: 0.05,
        }
    }

    #[test]
    fn base_grid_wall_lines_block_and_metrics_are_consistent() {
        for (sting, boat) in [(0.0, 0.0), (0.13, 1.0), (0.0, 1.0)] {
            let p = base_params(sting, boat);
            let (g, b) = base_domain_grid(&p); // asserts positive cell areas
            let bb = &p.body;
            assert_eq!(b.j_end, p.n_sting + p.n_gap);
            assert_eq!(b.j_sting, p.n_sting);
            assert_eq!(b.i_end, p.n_up + p.n_nose + p.n_seg.iter().sum::<usize>());
            // the base plane is at x_base, the wall row follows the body radius, the far field is at r_far
            assert!((g.node(b.i_end, 0).0 - bb.x_base()).abs() < 1e-12, "base plane");
            for i in 0..=b.i_end {
                let (x, r) = g.node(i, b.j_end);
                assert!((r - bb.r(x)).abs() < 1e-12, "wall radius at i={i}");
                assert!((g.node(i, g.nj).1 - p.r_far).abs() < 1e-12 || i > b.i_end);
            }
            // behind the base the row j_end sits at the base radius, the row j_sting at the sting radius (or the axis)
            for i in b.i_end..=g.ni {
                assert!((g.node(i, b.j_end).1 - bb.base_radius()).abs() < 1e-12);
                assert!((g.node(i, b.j_sting).1 - bb.sting_radius).abs() < 1e-12 || b.j_sting == 0);
                assert!(g.node(i, 0).1.abs() < 1e-12 || p.n_sting > 0);
            }
            // base face area (annulus between the sting and the base radius) equals pi (r_b^2 - r_s^2) in the r^2 weights
            let mut a = 0.0;
            for j in b.j_sting..b.j_end {
                let (n0, n1) = (g.node(b.i_end, j), g.node(b.i_end, j + 1));
                a += n1.1 * n1.1 - n0.1 * n0.1;
            }
            assert!((a - (bb.base_radius().powi(2) - bb.sting_radius.powi(2))).abs() < 1e-12);
            // geometric consistency: base face normal points downstream (+x) for the i-face at i_end
            assert!(g.iface[(b.j_end - 1) * (g.ni + 1) + b.i_end].nx > 0.99);
            // cell sizes grow smoothly through the base plane
            let xs: Vec<f64> = (0..=g.ni).map(|i| g.node(i, g.nj).0).collect();
            let worst = xs.windows(3).map(|w| ((w[2] - w[1]) / (w[1] - w[0])).max((w[1] - w[0]) / (w[2] - w[1]))).fold(1.0, f64::max);
            assert!(worst < 2.5, "neighbouring cell-size ratio {worst}");
        }
    }

    #[test]
    fn base_body_profile_is_continuous() {
        let b = base_params(0.0, 1.0).body;
        let xs = b.station(1).0;
        assert_eq!(xs, 6.0);
        assert!((b.r(xs - 1e-9) - 0.5).abs() < 1e-8);
        assert!((b.r(xs + 1e-9) - 0.5).abs() < 1e-8);
        assert!((b.r(b.x_base() - 1e-9) - b.base_radius()).abs() < 1e-8);
        assert!((b.base_radius() - (0.5 - 10f64.to_radians().tan())).abs() < 1e-14);
        assert_eq!(b.r(b.x_base() + 5.0), b.base_radius());
        assert_eq!(b.slope(b.x_base() + 5.0), 0.0);
        assert!((b.slope(xs + 0.5) + 10f64.to_radians().tan()).abs() < 1e-15);
    }

    fn four_segment_body() -> BaseBody {
        let mut b = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 2.0).with_boattail(1.0, 8f64.to_radians());
        b.segments[1].slope = 0.1; // flare
        b.segments.push(Segment { length: 1.5, slope: 0.0 });
        b.with_boattail(1.0, 8f64.to_radians())
    }

    #[test]
    fn multi_segment_grids_have_a_node_at_every_station_and_the_wall_follows_the_profile() {
        let body = four_segment_body();
        let nseg = body.segments.len();
        assert_eq!(nseg, 4);
        let stations: Vec<f64> = (0..=nseg).map(|k| body.station(k).0).collect();
        let has = |xs: &[f64], x: f64| xs.iter().any(|v| (v - x).abs() < 1e-12);
        // sting grid
        let lp = LargeGridParams {
            body: body.clone(),
            x_up: 30.0,
            x_down: 30.0,
            r_far: 30.0,
            n_up: 12,
            n_nose: 24,
            n_seg: vec![20, 10, 12, 10],
            n_ext: 12,
            nj: 24,
            dx_tip: 0.05,
            dr_wall: 0.05,
        };
        let g = large_domain_grid(&lp);
        let xs = axial_nodes(&lp);
        for (k, &x) in stations.iter().enumerate() {
            assert!(has(&xs, x), "sting grid: no node at station {k} (x = {x})");
        }
        assert!((xs[xs.len() - 1] - (body.body.nose_length() + 30.0)).abs() < 1e-12);
        for i in 0..=g.ni {
            let (x, r) = g.node(i, 0);
            assert!((r - body.r(x)).abs() < 1e-12, "sting grid wall at i={i}");
        }
        // base grid
        let bp = BaseGridParams {
            body: body.clone(),
            x_up: 30.0,
            x_down: 30.0,
            r_far: 30.0,
            n_up: 12,
            n_nose: 24,
            n_seg: vec![20, 10, 12, 10],
            n_wake: 16,
            n_sting: 0,
            n_gap: 8,
            nj_out: 20,
            dx_tip: 0.05,
            dr_wall: 0.05,
        };
        let (g, b) = base_domain_grid(&bp);
        let xs: Vec<f64> = (0..=g.ni).map(|i| g.node(i, g.nj).0).collect();
        for (k, &x) in stations.iter().enumerate() {
            assert!(has(&xs, x), "base grid: no node at station {k} (x = {x})");
        }
        for i in 0..=b.i_end {
            let (x, r) = g.node(i, b.j_end);
            assert!((r - body.r(x)).abs() < 1e-12, "base grid wall at i={i}");
        }
    }
}
