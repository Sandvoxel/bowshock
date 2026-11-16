//! 3D body-fitted grid of the half domain (`theta` in `[0, pi]`, pitch-plane symmetry) around a sting-mounted body with thick fins.
//!
//! The axisymmetric large-domain grid ([`super::body_grid`]) is revolved into `nk` azimuthal cells. Fins are one-cell-thick solid
//! layers in `k` (see `Grid3::solid`): the layer edges follow the fin thickness over the planform and relax to the nominal cell width
//! away from it, so the fin faces are flat planes through the body axis. Lengths are in nose diameters.
//!
//! * `Grid3` nodes are `(x, r sin(theta), r cos(theta))`: the pitch plane is `y = 0`, `theta = 0` points to `+z` (the lee side
//!   for `alpha > 0`) and `theta = pi` to `-z`.
//! * Fins sit at `theta_f = pi/n + 2 pi m / n`; those in `[0, pi]` are kept. For odd `n` the fin at `theta = pi` lies in the
//!   symmetry plane and is a half fin owning the last cell `k = nk - 1`.
//! * With fins, the root chord must lie on one tube segment. That segment is rebuilt as upstream piece, fin piece and downstream piece
//!   (`32 << level` cells over the root chord, clustered at both edges); the sweep and taper are applied as a piecewise-linear
//!   remap of the segment's `x` nodes per row (the cells outside the fin stretch or shrink instead of folding), blended away over
//!   four rows above the fin tip. The radial fractions are remapped so that a node row lies exactly on the fin tip.

use super::body_grid::{axial_nodes, radial_fractions, two_sided_nodes, LargeGridParams};
use crate::error::Error;
use crate::fins::normalised;
use crate::spec::FinProfile;
use bowshock_cfd::grid::geometric_offsets;
use bowshock_cfd::grid3::{Grid3, SolidBox};
use std::f64::consts::PI;

/// Trapezoidal fins in nose diameters.
#[derive(Clone, Debug)]
pub struct Fins3 {
    pub count: u32,
    pub root_chord: f64,
    pub tip_chord: f64,
    /// Radial extent from the body surface to the tip.
    pub span: f64,
    /// Axial distance root leading edge to tip leading edge, positive aft.
    pub sweep: f64,
    /// Maximum thickness.
    pub thickness: f64,
    /// Distance by which the root trailing edge lies ahead of the body base (0 = flush).
    pub te_to_base: f64,
    pub profile: FinProfile,
}

/// Grid parameters: the sting grid of the 2D case plus the fins and the azimuthal resolution.
#[derive(Clone, Debug)]
pub struct Grid3Params {
    pub sting: LargeGridParams,
    pub fins: Option<Fins3>,
    /// Azimuthal cells over `[0, pi]` (the plan default is `24 << level`, see [`Grid3Params::default_nk`]).
    pub nk: usize,
    /// Grid level: sets the cells over the fin root chord (`32 << level`) unless `fin_chord_cells` overrides it.
    pub level: u32,
    /// Cells over the fin root chord (`None`: `32 << level`; the ANF study found 16 too coarse for the fin loading at M >= 2).
    pub fin_chord_cells: Option<usize>,
    /// Radial cells from the body wall to the fin tip (`None`: whatever the sting grid's radial spacing puts there, with the tip
    /// row snapped to the nearest node). With `Some(n)` the radial fractions are regraded for the whole grid: `n` cells in
    /// `[R, R + span]` starting at the wall spacing (uniform when that is already finer than `span / n`), then a geometric fan of
    /// at least the sting grid's outer cell count with growth `<= 1.2` to the far boundary. `nj` grows accordingly.
    pub span_cells: Option<usize>,
}

impl Grid3Params {
    /// Default azimuthal cell count of a grid level.
    pub fn default_nk(level: u32) -> usize {
        24usize << level
    }
}

/// Half thickness of the fin section at the chordwise fraction `xi` (chord length `chord`, maximum thickness `t`), floored at
/// 25 % of `t/2` so that edges are blunted.
pub(crate) fn half_thickness(profile: &FinProfile, t: f64, chord: f64, xi: f64) -> f64 {
    let xi = xi.clamp(0.0, 1.0);
    let r = 0.5 * t;
    let h = match profile {
        FinProfile::Square => r,
        FinProfile::Rounded => {
            let s = (xi * chord).min((1.0 - xi) * chord);
            if s >= r {
                r
            } else {
                (r * r - (r - s) * (r - s)).max(0.0).sqrt()
            }
        }
        FinProfile::Airfoil => 5.0 * t * (0.2969 * xi.sqrt() - 0.1260 * xi - 0.3516 * xi * xi + 0.2843 * xi.powi(3) - 0.1015 * xi.powi(4)).max(0.0),
        FinProfile::DoubleWedge => t * xi.min(1.0 - xi),
        FinProfile::Custom { section } => {
            let z: Vec<[f64; 2]> = normalised(section).collect();
            let mut v = z[z.len() - 1][1];
            for w in z.windows(2) {
                if xi <= w[1][0] {
                    let d = w[1][0] - w[0][0];
                    let f = if d > 0.0 { ((xi - w[0][0]) / d).clamp(0.0, 1.0) } else { 1.0 };
                    v = w[0][1] + f * (w[1][1] - w[0][1]);
                    break;
                }
            }
            r * v
        }
    };
    h.max(0.25 * r)
}

/// Node-index layout of the fins on the axial/radial lines.
struct FinPlan {
    /// Node range `a..=b` of the rebuilt tube segment.
    i_seg0: usize,
    i_seg1: usize,
    i_le: usize,
    i_te: usize,
    j_tip: usize,
    x_le_root: f64,
    x_te_root: f64,
    r_t: f64,
    /// Radial fractions of the node rows, including the extra rows of `span_cells` (`nj + 1`).
    sj: Vec<f64>,
    /// Remapped radial fractions (`nj + 1`).
    s_remap: Vec<f64>,
}

/// Azimuthal layer structure: fin angles and the number of cells in each gap.
struct Layers {
    theta_f: Vec<f64>,
    half_last: bool,
    /// Cells of the gap before each layer, plus one trailing gap behind the last layer unless that is a half fin.
    gaps: Vec<usize>,
    /// First `k` of the cell owned by each layer.
    k_f: Vec<usize>,
}

fn layers(count: u32, nk: usize) -> Result<Layers, Error> {
    let n = count as f64;
    let mut theta_f = Vec::new();
    let mut half_last = false;
    for m in 0..count {
        let th = PI / n + 2.0 * PI * m as f64 / n;
        if th <= PI + 1e-9 {
            if (PI - th).abs() < 1e-9 {
                half_last = true;
                theta_f.push(PI);
            } else {
                theta_f.push(th);
            }
        }
    }
    let nl = theta_f.len();
    let nh = PI / (2.0 * nk as f64);
    let mut widths = Vec::new();
    let mut prev = 0.0;
    for (l, &th) in theta_f.iter().enumerate() {
        let half = half_last && l + 1 == nl;
        widths.push((if half { PI - nh } else { th - nh }) - prev);
        prev = if half { PI } else { th + nh };
    }
    if !half_last {
        widths.push(PI - prev);
    }
    let too_close = || Error::Grid("fins too close for nk cells".to_string());
    if nk < nl || widths.iter().any(|&w| w <= 0.0) || nk - nl < 2 * widths.len() {
        return Err(too_close());
    }
    let remaining = nk - nl;
    let total: f64 = widths.iter().sum();
    let q: Vec<f64> = widths.iter().map(|w| remaining as f64 * w / total).collect();
    let mut gaps: Vec<usize> = q.iter().map(|&x| (x.round() as usize).max(2)).collect();
    while gaps.iter().sum::<usize>() != remaining {
        if gaps.iter().sum::<usize>() > remaining {
            let i = (0..gaps.len()).filter(|&i| gaps[i] > 2).max_by(|&a, &b| (gaps[a] as f64 - q[a]).total_cmp(&(gaps[b] as f64 - q[b])));
            gaps[i.ok_or_else(too_close)?] -= 1;
        } else {
            let i = (0..gaps.len()).max_by(|&a, &b| (q[a] - gaps[a] as f64).total_cmp(&(q[b] - gaps[b] as f64))).unwrap();
            gaps[i] += 1;
        }
    }
    let mut k_f = Vec::new();
    let mut k = 0;
    for l in 0..nl {
        k += gaps[l];
        k_f.push(k);
        k += 1;
    }
    Ok(Layers { theta_f, half_last, gaps, k_f })
}

/// Azimuth of the nodes `k = 0..=nk` of one `(i, j)` column for the layer half-angle `alpha`.
fn fill_theta(theta: &mut [f64], lay: &Layers, alpha: f64) {
    let nk = theta.len() - 1;
    theta[0] = 0.0;
    let (mut k, mut prev) = (0usize, 0.0f64);
    let nl = lay.theta_f.len();
    for (l, &th) in lay.theta_f.iter().enumerate() {
        let half = lay.half_last && l + 1 == nl;
        let lo = if half { PI - alpha } else { th - alpha };
        let hi = if half { PI } else { th + alpha };
        let ng = lay.gaps[l];
        for m in 1..=ng {
            theta[k + m] = prev + (lo - prev) * m as f64 / ng as f64;
        }
        k += ng;
        theta[k + 1] = hi;
        k += 1;
        prev = hi;
    }
    if lay.gaps.len() > nl {
        let ng = lay.gaps[nl];
        for m in 1..=ng {
            theta[k + m] = prev + (PI - prev) * m as f64 / ng as f64;
        }
    }
    theta[nk] = PI;
}

fn smoothstep(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

/// Choose the first candidate `(node, distance, outer cell width)` that a geometric fan of `m` cells starting at `dx` with growth
/// `q <= 1.2` reaches with a last cell within 1.4 of the outer width; returns `(node, m, q)` (the best compromise if none does).
fn fan(dx: f64, cands: &[(usize, f64, f64)]) -> Option<(usize, usize, f64)> {
    let growth = |m: usize, d: f64| -> f64 {
        // dx (q^m - 1)/(q - 1) = d
        let (mut lo, mut hi) = (0.2f64, 3.0f64);
        for _ in 0..80 {
            let q = 0.5 * (lo + hi);
            let sum = if (q - 1.0).abs() < 1e-12 { dx * m as f64 } else { dx * (q.powi(m as i32) - 1.0) / (q - 1.0) };
            if sum < d {
                lo = q;
            } else {
                hi = q;
            }
        }
        0.5 * (lo + hi)
    };
    let mut best: Option<(f64, usize, usize, f64)> = None;
    for &(o, d, w) in cands {
        for m in 1..=200usize {
            let q = growth(m, d);
            let last = dx * q.powi(m as i32 - 1);
            let score = (q.max(1.0 / q) / 1.2).max((last / w).max(w / last) / 1.4);
            if score <= 1.0 {
                return Some((o, m, q));
            }
            if best.is_none_or(|b| score < b.0) {
                best = Some((score, o, m, q));
            }
        }
    }
    best.map(|b| (b.1, b.2, b.3))
}

/// Rebuild the tube segment that carries the fins in `xs` and compute the radial remap.
fn plan_fins(p: &Grid3Params, f: &Fins3, xs: &mut Vec<f64>) -> Result<FinPlan, Error> {
    let sp = &p.sting;
    let bad = |m: &str| Err(Error::InvalidInput(m.to_string()));
    let ok = |v: f64| v.is_finite();
    if !(f.count >= 1 && ok(f.root_chord) && f.root_chord > 0.0 && ok(f.tip_chord) && f.tip_chord >= 0.0 && ok(f.span) && f.span > 0.0)
        || !(ok(f.sweep) && ok(f.thickness) && f.thickness > 0.0 && ok(f.te_to_base) && f.te_to_base >= 0.0)
    {
        return bad("3D fins need count >= 1, positive root chord, span and thickness, tip chord >= 0 and te_to_base >= 0");
    }
    let bb = &sp.body;
    let x_te_root = bb.x_base() - f.te_to_base;
    let x_le_root = x_te_root - f.root_chord;
    let nseg = bb.segments.len();
    let unsupported = || Error::Unsupported("3D fins must sit on a tube".to_string());
    let k = (0..nseg)
        .find(|&k| {
            let x0 = bb.station(k).0;
            bb.segments[k].slope == 0.0 && x_le_root >= x0 - 1e-9 && x_te_root <= x0 + bb.segments[k].length + 1e-9
        })
        .ok_or_else(unsupported)?;
    let a = sp.n_up + sp.n_nose + sp.n_seg[..k].iter().sum::<usize>();
    let b_old = a + sp.n_seg[k];
    let (x_a, x_b) = (xs[a], xs[b_old]);
    let x_le_root = if x_le_root - x_a > 1e-9 { x_le_root } else { x_a };
    let x_te_root = if x_b - x_te_root > 1e-9 { x_te_root } else { x_b };
    let flush = x_te_root >= x_b;
    let nf = p.fin_chord_cells.unwrap_or(32usize << p.level);
    if nf < 2 {
        return Err(Error::InvalidInput("3D fins need at least 2 cells over the root chord".to_string()));
    }
    let mut fin_offs = two_sided_nodes(f.root_chord, nf, 0.3 * f.root_chord / nf as f64);
    fin_offs[nf] = x_te_root - x_le_root;
    let dx_f = fin_offs[1] - fin_offs[0];
    // Regrade: the fin piece replaces the original nodes over its chord. Geometric fans (growth <= 1.2) lead from its end cells
    // to the first original node where the spacing has caught up with the original one (ratio <= 1.5 on both sides).
    // Candidates: `(node, distance from the fin edge, width of the original cell beyond the node)`.
    let up: Vec<(usize, f64, f64)> = (a..b_old).rev().filter(|&o| x_le_root - xs[o] > 1e-9).map(|o| (o, x_le_root - xs[o], xs[o] - xs[o - 1])).collect();
    let o_max = if flush { (b_old + 60).min(xs.len() - 1) } else { b_old };
    let down: Vec<(usize, f64, f64)> = (a..=o_max)
        .filter(|&o| xs[o] - x_te_root > 1e-9)
        .map(|o| (o, xs[o] - x_te_root, if o + 1 < xs.len() { xs[o + 1] - xs[o] } else { xs[o] - xs[o - 1] }))
        .collect();
    let mut new = xs[..=a].to_vec();
    if let Some((o, m, q)) = fan(dx_f, &up) {
        {
            new = xs[..=o].to_vec();
            let sizes: Vec<f64> = (0..m).rev().map(|i| dx_f * q.powi(i as i32)).collect();
            let mut s = xs[o];
            for w in &sizes[..m - 1] {
                s += w;
                new.push(s);
            }
            new.push(x_le_root);
        }
    }
    let i_le = new.len() - 1;
    new.extend((1..=nf).map(|i| x_le_root + fin_offs[i]));
    let i_te = new.len() - 1;
    let (o_down, m_down) = match fan(dx_f, &down) {
        Some((o, m, q)) => {
            let mut s = 0.0;
            for i in 0..m - 1 {
                s += dx_f * q.powi(i as i32);
                new.push(x_te_root + s);
            }
            new.push(xs[o]);
            (o, m)
        }
        None => (b_old, 0),
    };
    new.extend_from_slice(&xs[o_down + 1..]);
    let i_seg1 = if flush { i_te } else { i_te + m_down + (b_old - o_down) };
    *xs = new;

    let r_t = bb.station(k).1;
    if sp.r_far.is_nan() || sp.r_far <= r_t + f.span || sp.nj < 6 {
        return Err(Error::Grid("the fin tip must lie inside the domain".to_string()));
    }
    let sj0 = radial_fractions(sp.dr_wall, sp.r_far, sp.nj);
    let target = f.span / (sp.r_far - r_t);
    let j_tip0 = (1..=sp.nj).min_by(|&a, &b| (sj0[a] - target).abs().total_cmp(&(sj0[b] - target).abs())).unwrap();
    let (sj, s_remap, j_tip) = match p.span_cells {
        None => {
            let s_remap: Vec<f64> = (0..=sp.nj)
                .map(|j| {
                    if j == j_tip0 {
                        target
                    } else if j < j_tip0 {
                        sj0[j] * target / sj0[j_tip0]
                    } else {
                        target + (sj0[j] - sj0[j_tip0]) * (1.0 - target) / (1.0 - sj0[j_tip0])
                    }
                })
                .collect();
            (sj0, s_remap, j_tip0)
        }
        Some(n) => {
            if n < 2 {
                return Err(Error::InvalidInput("span_cells >= 2 is required".to_string()));
            }
            let first = sj0[1];
            let inner = if first * n as f64 >= target {
                (0..=n).map(|j| target * j as f64 / n as f64).collect::<Vec<_>>()
            } else {
                geometric_offsets(first, target, n)
            };
            let c_last = inner[n] - inner[n - 1];
            let outer = 1.0 - target;
            // growth q of `m` cells c_last q, c_last q^2, .. summing to `outer`
            let growth = |m: usize| -> f64 {
                let (mut lo, mut hi) = (0.2f64, 4.0f64);
                for _ in 0..200 {
                    let q = 0.5 * (lo + hi);
                    let (mut s, mut d) = (0.0, c_last);
                    for _ in 0..m {
                        d *= q;
                        s += d;
                    }
                    if s < outer {
                        lo = q;
                    } else {
                        hi = q;
                    }
                }
                0.5 * (lo + hi)
            };
            let mut m = (sp.nj - j_tip0).max(4);
            while growth(m) > 1.2 {
                m += 1;
                if m > 2000 {
                    return Err(Error::Grid("span_cells: no outer fan with growth <= 1.2".to_string()));
                }
            }
            let q = growth(m);
            let mut s = inner;
            let (mut acc, mut d) = (target, c_last);
            for _ in 0..m {
                d *= q;
                acc += d;
                s.push(acc);
            }
            *s.last_mut().unwrap() = 1.0;
            (s.clone(), s, n)
        }
    };
    let nj = s_remap.len() - 1;
    if j_tip + 4 > nj {
        return Err(Error::Grid("the fin tip is too close to the outer boundary".to_string()));
    }
    // the swept tip trailing edge must stay inside the rebuilt segment (flush: not behind the base)
    let x_te_tip = x_le_root + f.sweep + f.tip_chord;
    let room = if i_te == i_seg1 { 1e-9 } else { 0.5 * (x_b - x_te_root) };
    if x_te_tip > x_te_root + room || x_le_root + f.sweep < x_a - 1e-9 {
        return Err(Error::Unsupported("3D fin tip trailing edge lies behind the root trailing edge room".to_string()));
    }
    Ok(FinPlan { i_seg0: a, i_seg1, i_le, i_te, j_tip, x_le_root, x_te_root, r_t, sj, s_remap })
}

/// Build the 3D grid. With fins the grid has `sting.nj`, `nk` cells in `j`, `k` and the axial count of `axial_nodes` changed by
/// the rebuilt fin segment.
pub fn body_grid3(p: &Grid3Params) -> Result<Grid3, Error> {
    let sp = &p.sting;
    let nk = p.nk;
    if nk < 2 {
        return Err(Error::Grid("nk >= 2 is required".to_string()));
    }
    if !(sp.nj >= 1 && sp.dr_wall > 0.0 && sp.r_far > 0.0) {
        return Err(Error::Grid("nj >= 1, dr_wall > 0 and r_far > 0 are required".to_string()));
    }
    let mut xs = axial_nodes(sp)?;
    let plan = match &p.fins {
        Some(f) => Some((plan_fins(p, f, &mut xs)?, f)),
        None => None,
    };
    let lay = match &p.fins {
        Some(f) => Some(layers(f.count, nk)?),
        None => None,
    };
    let ni = xs.len() - 1;
    let sj = match &plan {
        Some((pl, _)) => pl.sj.clone(),
        None => radial_fractions(sp.dr_wall, sp.r_far, sp.nj),
    };
    let nj = sj.len() - 1;
    let n2 = (ni + 1) * (nj + 1);

    // (x, r) of every (i, j) node
    let mut xn = vec![0.0; n2];
    let mut rn = vec![0.0; n2];
    let r_in: Vec<f64> = xs.iter().map(|&x| sp.body.r(x)).collect();
    for j in 0..=nj {
        for i in 0..=ni {
            xn[j * (ni + 1) + i] = xs[i];
            rn[j * (ni + 1) + i] = r_in[i] + sj[j] * (sp.r_far - r_in[i]);
        }
    }
    // per-row planform (frac along the span, LE position, chord) and the half-angle of the fin layer at each column
    let mut alpha = Vec::new();
    if let Some((pl, f)) = &plan {
        let wcol = |i: usize| -> f64 {
            let d = if i < pl.i_le { pl.i_le - i } else { i.saturating_sub(pl.i_te) };
            if d <= 4 {
                1.0
            } else {
                (1.0 - (d as f64 - 4.0) / 4.0).max(0.0)
            }
        };
        for j in 0..=nj {
            for i in 0..=ni {
                let s = sj[j] + wcol(i) * (pl.s_remap[j] - sj[j]);
                rn[j * (ni + 1) + i] = r_in[i] + s * (sp.r_far - r_in[i]);
            }
        }
        let row = |j: usize| -> (f64, f64, f64) {
            let r = pl.r_t + pl.s_remap[j.min(pl.j_tip)] * (sp.r_far - pl.r_t);
            let frac = ((r - pl.r_t) / f.span).clamp(0.0, 1.0);
            (frac, pl.x_le_root + f.sweep * frac, f.root_chord + (f.tip_chord - f.root_chord) * frac)
        };
        // sweep and taper: piecewise-linear remap of the segment nodes
        let nodes0: Vec<f64> = xs.clone();
        for j in 0..=(pl.j_tip + 3).min(nj) {
            let vj = if j <= pl.j_tip { 1.0 } else { 1.0 - (j - pl.j_tip) as f64 / 4.0 };
            let (_, x_le, chord) = row(j);
            let mut knots = vec![(pl.i_seg0, nodes0[pl.i_seg0], nodes0[pl.i_seg0])];
            if pl.i_le == pl.i_seg0 {
                knots.clear();
            }
            knots.push((pl.i_le, pl.x_le_root, x_le));
            knots.push((pl.i_te, pl.x_te_root, x_le + chord));
            if pl.i_te != pl.i_seg1 {
                knots.push((pl.i_seg1, nodes0[pl.i_seg1], nodes0[pl.i_seg1]));
            }
            for w in knots.windows(2) {
                for i in w[0].0..=w[1].0 {
                    let old = nodes0[i];
                    let d = w[1].1 - w[0].1;
                    let t = if d.abs() > 0.0 { (old - w[0].1) / d } else { 1.0 };
                    let new = w[0].2 + t * (w[1].2 - w[0].2);
                    xn[j * (ni + 1) + i] = old + vj * (new - old);
                }
            }
        }
        // layer half-angle at every (i, j) node
        alpha = vec![0.0; n2];
        let t = f.thickness;
        let h_at = |i: usize, j: usize| -> f64 {
            let (_, _, chord) = row(j);
            let xi = (nodes0[i] - pl.x_le_root) / f.root_chord;
            half_thickness(&f.profile, t, chord, xi)
        };
        for j in 0..=nj {
            for i in 0..=ni {
                let r = rn[j * (ni + 1) + i];
                let di = if i < pl.i_le { pl.i_le - i } else { i.saturating_sub(pl.i_te) };
                let dj = j.saturating_sub(pl.j_tip);
                let d = di.max(dj);
                let (ic, jc) = (i.clamp(pl.i_le, pl.i_te), j.min(pl.j_tip));
                let h_f = h_at(ic, jc);
                let nominal = 0.5 * r * PI / nk as f64;
                let w = if d == 0 { h_f } else { h_f + (nominal - h_f) * smoothstep(d as f64 / 4.0) };
                alpha[j * (ni + 1) + i] = if r < 1e-12 { (PI / (2.0 * nk as f64)).asin() } else { (w / r).min(0.99).asin() };
            }
        }
    }

    let nn = (ni + 1) * (nj + 1) * (nk + 1);
    let (mut x, mut y, mut z) = (vec![0.0; nn], vec![0.0; nn], vec![0.0; nn]);
    let mut theta = vec![0.0; nk + 1];
    for j in 0..=nj {
        for i in 0..=ni {
            let c = j * (ni + 1) + i;
            match &lay {
                Some(l) => fill_theta(&mut theta, l, alpha[c]),
                None => {
                    for (k, t) in theta.iter_mut().enumerate() {
                        *t = PI * k as f64 / nk as f64;
                    }
                }
            }
            let (xc, r) = (xn[c], rn[c]);
            for k in 0..=nk {
                let id = (k * (nj + 1) + j) * (ni + 1) + i;
                x[id] = xc;
                y[id] = r * theta[k].sin();
                z[id] = r * theta[k].cos();
            }
        }
    }
    let mut g = Grid3::try_from_nodes(ni, nj, nk, x, y, z).map_err(Error::Grid)?;
    g.axis_cells = sp.n_up;
    if let (Some((pl, _)), Some(l)) = (&plan, &lay) {
        g.solid = l.k_f.iter().map(|&kf| SolidBox { i: pl.i_le..pl.i_te, j: 0..pl.j_tip, k: kf..kf + 1 }).collect();
        for k in 0..nk {
            for j in 0..nj {
                for i in 0..ni {
                    if !g.is_solid(i, j, k) && g.vol[g.cell(i, j, k)] <= 0.0 {
                        return Err(Error::Grid(format!("non-positive fluid cell volume at ({i},{j},{k})")));
                    }
                }
            }
        }
    } else {
        for k in 0..nk {
            for j in 0..nj {
                for i in 0..ni {
                    if g.vol[g.cell(i, j, k)] <= 0.0 && i >= sp.n_up {
                        return Err(Error::Grid(format!("non-positive fluid cell volume at ({i},{j},{k})")));
                    }
                }
            }
        }
    }
    Ok(g)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::body_grid::{BaseBody, Body, Nose};
    use crate::engine::case::{Case, Wake};
    use crate::engine::numerics::Numerics;
    use bowshock_cfd::grid::Grid;

    const R: f64 = 0.5;

    fn anf_sting(level: u32) -> LargeGridParams {
        let nose = Nose::Cone { half_angle: 10f64.to_radians() };
        let body = Body { nose };
        let ln = body.nose_length();
        let bb = BaseBody::cylinder(body, 10.0 - ln);
        Case::new(2.0, bb, level, Wake::Sting, Numerics::auto(2.0, level, Wake::Sting)).grid_sting
    }

    fn anf_fins(profile: FinProfile) -> Fins3 {
        Fins3 { count: 4, root_chord: 1.0, tip_chord: 1.0, span: 1.0, sweep: 0.0, thickness: 0.08, te_to_base: 0.0, profile }
    }

    fn params(level: u32, fins: Option<Fins3>) -> Grid3Params {
        Grid3Params { sting: anf_sting(level), fins, nk: Grid3Params::default_nk(level), level, fin_chord_cells: None, span_cells: None }
    }

    /// Distance of node `(i, j, k)` from the fin mid-plane at angle `th`.
    fn dist(g: &Grid3, i: usize, j: usize, k: usize, th: f64) -> f64 {
        let id = (k * (g.nj + 1) + j) * (g.ni + 1) + i;
        g.y[id] * th.cos() - g.z[id] * th.sin()
    }

    fn node(g: &Grid3, i: usize, j: usize, k: usize) -> [f64; 3] {
        let id = (k * (g.nj + 1) + j) * (g.ni + 1) + i;
        [g.x[id], g.y[id], g.z[id]]
    }

    #[test]
    fn planform_area_of_a_swept_tapered_fin_set() {
        let f = Fins3 { count: 4, root_chord: 1.0, tip_chord: 0.6, span: 1.0, sweep: 0.3, thickness: 0.08, te_to_base: 0.0, profile: FinProfile::Square };
        let g = body_grid3(&params(0, Some(f))).unwrap();
        assert_eq!(g.solid.len(), 2);
        for b in &g.solid {
            let k = b.k.start;
            let a: f64 = (b.j.clone()).flat_map(|j| b.i.clone().map(move |i| (i, j))).map(|(i, j)| g.kface[(k * g.nj + j) * g.ni + i].area).sum();
            assert!((a / 0.8 - 1.0).abs() < 0.01, "planform area {a} vs 0.8");
        }
    }

    #[test]
    fn fin_faces_follow_the_profile() {
        for profile in [FinProfile::Rounded, FinProfile::Airfoil, FinProfile::DoubleWedge, FinProfile::Square] {
            let f = Fins3 { count: 4, root_chord: 1.0, tip_chord: 0.6, span: 1.0, sweep: 0.3, thickness: 0.08, te_to_base: 0.0, profile: profile.clone() };
            let g = body_grid3(&params(0, Some(f.clone()))).unwrap();
            for (m, b) in g.solid.iter().enumerate() {
                let th = PI / 4.0 + PI / 2.0 * m as f64;
                for j in 0..=b.j.end {
                    for i in b.i.start..=b.i.end {
                        let p = node(&g, i, j, b.k.start);
                        let r = (p[1] * p[1] + p[2] * p[2]).sqrt();
                        let frac = (r - R) / f.span;
                        let x_le = f.sweep * frac;
                        let x_le0 = g.x[b.i.start]; // wall row (k = 0, j = 0) at the root leading edge
                        let chord = f.root_chord + (f.tip_chord - f.root_chord) * frac;
                        let xi = (p[0] - (x_le0 + x_le)) / chord;
                        let h = half_thickness(&profile, f.thickness, chord, xi);
                        let (lo, hi) = (dist(&g, i, j, b.k.start, th), dist(&g, i, j, b.k.end, th));
                        assert!((lo + h).abs() < 1e-12 && (hi - h).abs() < 1e-12, "{profile:?} ({i},{j}): {lo} {hi} vs {h}");
                    }
                }
            }
        }
    }

    #[test]
    fn odd_count_gives_a_half_fin_in_the_symmetry_plane() {
        let mut f = anf_fins(FinProfile::Square);
        f.count = 3;
        let p = params(0, Some(f));
        let g = body_grid3(&p).unwrap();
        assert_eq!(g.solid.len(), 2);
        assert_eq!(g.solid[1].k, p.nk - 1..p.nk);
        let b = &g.solid[1];
        let (i, j) = (b.i.start + 3, 2);
        assert!(node(&g, i, j, p.nk)[1].abs() < 1e-12);
        let id = |k: usize| (k * (g.nj + 1) + j) * (g.ni + 1) + i;
        assert!(g.y[id(p.nk - 1)] > 0.03);
        // the full fin is not a half fin
        assert!(g.solid[0].k.end < p.nk - 1);
    }

    #[test]
    fn body_alone_volume_matches_the_revolved_2d_grid() {
        let mut p = params(0, None);
        p.nk = 48;
        let g = body_grid3(&p).unwrap();
        let g2: Grid = crate::engine::body_grid::large_domain_grid(&p.sting).unwrap();
        let v2: f64 = g2.vol.iter().sum();
        let v3: f64 = g.vol.iter().sum();
        let flat = p.nk as f64 * (PI / p.nk as f64).sin() * v2;
        assert!((v3 / flat - 1.0).abs() < 1e-10, "{v3} vs {flat}");
        assert!((v3 / (PI * v2) - 1.0).abs() < 1e-3, "{v3} vs {}", PI * v2);
        assert_eq!(g.axis_cells, p.sting.n_up);
        assert!(g.solid.is_empty());
    }

    #[test]
    fn anf_like_grids_have_positive_fluid_volumes() {
        for level in [0, 1] {
            let p = params(level, Some(anf_fins(FinProfile::Square)));
            let g = body_grid3(&p).unwrap();
            assert_eq!(g.solid.len(), 2);
            eprintln!("level {level}: {} x {} x {} cells", g.ni, g.nj, g.nk);
        }
    }

    #[test]
    fn k_stretching_next_to_the_fin_layers() {
        for nk in [24usize, 32] {
            let mut p = params(0, Some(anf_fins(FinProfile::Square)));
            p.nk = nk;
            let g = body_grid3(&p).unwrap();
            let width = |i: usize, j: usize, k: usize| {
                let (a, b) = (node(&g, i, j, k), node(&g, i, j, k + 1));
                ((a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
            };
            let (mut all, mut foot) = (1.0f64, 1.0f64);
            for b in &g.solid {
                let kf = b.k.start;
                for j in 1..g.nj {
                    for i in 0..g.ni {
                        for (k0, k1) in [(kf - 1, kf), (kf + 1, kf)] {
                            if k0 + 1 > g.nk - 1 || width(i, j, k1) < 1e-12 || width(i, j, k0) < 1e-12 {
                                continue;
                            }
                            let q = (width(i, j, k0) / width(i, j, k1)).max(width(i, j, k1) / width(i, j, k0));
                            all = all.max(q);
                            if b.i.contains(&i) && j < b.j.end {
                                foot = foot.max(q);
                            }
                        }
                    }
                }
            }
            eprintln!("nk = {nk}: max k stretching ratio next to fin layers: {all:.3} (footprint cells: {foot:.3})");
        }
    }

    /// Axial node positions along the wall row of a 3D grid.
    fn wall_x(g: &Grid3) -> Vec<f64> {
        (0..=g.ni).map(|i| g.x[i]).collect()
    }

    /// `(max ratio, index of the larger cell's left node)` of adjacent cell widths.
    fn max_ratio(xs: &[f64], from: usize) -> (f64, usize) {
        let mut m = (1.0, 0);
        for i in from..xs.len() - 2 {
            let (a, b) = (xs[i + 1] - xs[i], xs[i + 2] - xs[i + 1]);
            let q = (a / b).max(b / a);
            if q > m.0 {
                m = (q, i);
            }
        }
        m
    }

    #[test]
    fn axial_spacing_is_smooth_around_the_fins() {
        for level in [0, 1] {
            let p = params(level, Some(anf_fins(FinProfile::Square)));
            let g2 = crate::engine::body_grid::large_domain_grid(&p.sting).unwrap();
            let x2: Vec<f64> = (0..=g2.ni).map(|i| g2.x[i]).collect();
            let g = body_grid3(&p).unwrap();
            let xs = wall_x(&g);
            let (le, te) = (g.solid[0].i.start, g.solid[0].i.end);
            let (q2, i2) = max_ratio(&x2, 0);
            let (q2_aft, _) = max_ratio(&x2, p.sting.n_up);
            let (q, iq) = max_ratio(&xs, 0);
            let (q_aft, iqa) = max_ratio(&xs, p.sting.n_up);
            let ratio = |i: usize| ((xs[i + 1] - xs[i]) / (xs[i] - xs[i - 1])).max((xs[i] - xs[i - 1]) / (xs[i + 1] - xs[i]));
            let (q_fin, _) = max_ratio(&xs[le..=te], 0);
            eprintln!(
                "level {level}: ni 2D {} 3D {}; 2D max {q2:.3} at {i2} (from tip {q2_aft:.3}); 3D max {q:.3} at {iq} (from tip {q_aft:.3} at {iqa}); junctions LE {:.3} TE {:.3}; inside fin piece {q_fin:.3}",
                g2.ni, g.ni, ratio(le), ratio(te)
            );
            assert!(q <= 1.5 && q_aft <= 1.5, "level {level}: ratio {q} at {iq}, from the tip {q_aft} at {iqa}");
            let _ = q2_aft;
        }
    }

    #[test]
    fn refinement_knobs_change_only_what_they_name() {
        let base = params(0, Some(anf_fins(FinProfile::Square)));
        let g0 = body_grid3(&base).unwrap();
        let j0 = g0.solid[0].j.end;
        let mut p = base.clone();
        p.span_cells = Some(2 * j0);
        let g = body_grid3(&p).unwrap();
        assert_eq!(g.solid[0].j.end, 2 * j0);
        assert_eq!(g.ni, g0.ni);
        assert!(g.nj >= g0.nj + j0 - 2, "nj {} vs {}", g.nj, g0.nj);
        let tip = |g: &Grid3| {
            let id = g.solid[0].j.end * (g.ni + 1) + g.solid[0].i.start;
            (g.y[id].powi(2) + g.z[id].powi(2)).sqrt()
        };
        assert!((tip(&g) - 1.5).abs() < 1e-9 && (tip(&g0) - 1.5).abs() < 1e-9);
        let mut p = base.clone();
        p.fin_chord_cells = Some(16);
        assert_eq!(body_grid3(&p).unwrap().solid[0].i.len(), 16);
        assert_eq!(g0.solid[0].i.len(), 32);
        let mut p = base;
        p.nk = 48;
        assert_eq!(body_grid3(&p).unwrap().nk, 48);
        for (label, nk, fc, sc) in [
            ("base", 24, None, None),
            ("nk32", 32, None, None),
            ("nk48", 48, None, None),
            ("span2x", 24, None, Some(2 * j0)),
            ("chord16", 24, Some(16), None),
            ("all", 48, None, Some(2 * j0)),
        ] {
            let mut p = params(0, Some(anf_fins(FinProfile::Square)));
            (p.nk, p.fin_chord_cells, p.span_cells) = (nk, fc, sc);
            let g = body_grid3(&p).unwrap();
            eprintln!("{label}: ni {} nj {} nk {} cells {} span cells {}", g.ni, g.nj, g.nk, g.ni * g.nj * g.nk, g.solid[0].j.end);
        }
    }
}
