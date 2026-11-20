//! 3D body-fitted grids around a body with thick fins: the axisymmetric meridian grid of the 2D case ([`super::body_grid`]) revolved into `nk`
//! azimuthal cells, in one of three domains ([`Domain3`]): the half domain (`theta` in `[0, pi]`, pitch-plane symmetry, for incidence), the sector
//! `[0, pi/n]` of a body with `n` identical fins (a fin mid-plane to mid-gap, symmetry planes at both ends) and the single-cell wedge of an
//! axisymmetric flow. Two meridian grids: the sting grid (`large_domain_grid`: the body is the `j = 0` wall, no base) and the base-wake grid
//! (`base_domain_grid`: the body interior is a solid block, the base its rear face, the wake reaches the axis behind it).
//!
//! Fins are solid layers in `k` ([`Grid3Params::fin_cells_across`] cells across the thickness, one by default; see `Grid3::part`): the layer edges follow the fin thickness over the planform and relax to
//! the nominal width (that many cells of the neighbouring spacing) away from it, so the fin faces are flat planes through the body axis; the nodes between the two faces are spread uniformly over the local thickness.
//!
//! The nodes of a `k` line are scaled out to `s_k r` ([`bowshock_cfd::grid3::polygon_scale`]) so that every polygonal ring has the cross-section area of the
//! circular one (the inscribed polygon lacks `dtheta^2 / 6` of it); the fin faces are put where their scaled offset is the fin half-thickness. With a
//! [`FinStack`] the cells of every gap grow from the fin sides (first cell from a target `y+`, 1.25 inside the boundary layer, 1.35 beyond) and the azimuthal
//! count follows from it. Sharp fin edges keep a needle (2 % of the half thickness at really sharp nodes; 10 % at blunt ends), tapered over four cells ahead of
//! the leading edge, behind the trailing edge and above the tip. Per-part roughness is carried on the grid ([`Grid3Params::roughness`]).
//!
//! * `Grid3` nodes are `(x, r sin(theta), r cos(theta))`: the pitch plane is `y = 0`, `theta = 0` points to `+z` (the lee side
//!   for `alpha > 0`) and `theta = pi` to `-z`.
//! * Half domain: fins sit at `theta_f = phi0 + 2 pi m / n` with `phi0 = pi/n` ([`FinPhase::Between`]: no fin in the pitch plane) or `0`
//!   ([`FinPhase::InPlane`]); those in `[0, pi]` are kept. A fin on `theta = 0` or `pi` lies in a symmetry plane and is a half fin owning the
//!   first / last cell.
//! * Sector: one half fin on the `theta = 0` plane (cells `k = 0..`: half the cells of a fin, at least one), `nk` minus those gap cells up to the mid-gap plane `theta = pi/n`.
//! * With fins, the root chord must lie on one tube segment. That segment is rebuilt as upstream piece, fin piece and downstream piece
//!   (`32 << level` cells over the root chord, clustered at both edges); the sweep and taper are applied as a piecewise-linear
//!   remap of the segment's `x` nodes per row (the cells outside the fin stretch or shrink instead of folding), blended away over
//!   four rows above the fin tip. The radial fractions are remapped so that a node row lies exactly on the fin tip. Fins are built on the sting
//!   grid only for now.

use super::body_grid::{
    axial_nodes, base_axial_nodes, base_domain_grid, base_rows_below, radial_fractions, two_sided_nodes, BaseBody, BaseGridParams, LargeGridParams,
};
use crate::engine::planform::Planform;
use crate::error::Error;
use crate::fins::normalised;
use crate::spec::FinProfile;
use bowshock_cfd::grid::geometric_offsets;
use bowshock_cfd::grid3::{polygon_scale_where, revolve, AxisRun, Grid3, Part, PartRoughness, SolidBox, Symmetry};
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
    /// The outline when it is not the trapezoid of `root_chord`, `tip_chord`, `span` and `sweep` ([`Fins3::with_planform`] keeps the four numbers consistent with it).
    pub planform: Option<Planform>,
}

/// The width of the zone that absorbs the move of the leading edge along the rows of a swept or tapered fin, in units of the largest move: the cells in it are
/// stretched (or shrunk) by at most `1 + 1 / ZONE_REACH`.
const ZONE_REACH: f64 = 3.0;

/// How much longer, in units of the largest move of the leading edge, the fade of the tip's radial bending ahead of the edge is when the zone is widened.
const RAMP_REACH: f64 = 0.3;

/// Cells over the radius of a round fin edge: the first chord cell is at most this fraction of it.
const LE_RADIUS_CELLS: f64 = 4.0;

/// The slope of the node rows beside a fin edge, bent to meet the tip's cylinder, as a fraction of the slope of the edge the rows are sheared along: the fade of the bend is
/// as long as keeps them below it (`ramp_need` in `body_grid3_inner`).
const RAMP_SLOPE: f64 = 0.5;

/// The smallest angle between the edges of the cells beside a trailing edge that overhangs the root's, where the rows bent to meet the tip's cylinder and the sheared column lean the one
/// way (`overhang_need` in `body_grid3_inner`, DESIGN3D item 73), and the least row slope the fade is allowed to ask for to keep it.
const OVERHANG_EDGE_ANGLE_DEG: f64 = 25.0;
const OVERHANG_MIN_SLOPE: f64 = 0.25;

/// A fin whose zone is not widened still fades the move of its edges over more than four rows above the tip when four rows lift less than this many moves above it: the
/// shear of four rows is then steeper than 59 degrees from the radial (Byrum, Cancer Descending, Ion Drive: 10, 5 and 4.8 degrees between the cell edges above the tip; the
/// example rocket's rows are 0.77 and 1.29 moves high and keep their four).
const FADE_TRIGGER: f64 = 0.6;

/// A zone is widened when the move of an edge is more than this fraction of the room the segment has for it.
const ZONE_TRIGGER: f64 = 0.5;

/// The height over the tip row, in units of the largest move of the edges, over which the move fades away above the fin (at least four rows).
const FADE_REACH: f64 = 1.5;

/// The chord at which a pointed or elliptical planform is cut at the tip, as a fraction of the root chord.
pub const TIP_CHORD_FLOOR: f64 = 0.04;

impl Fins3 {
    /// These fins with the outline `pf` (an ellipse, a freeform polygon, ...): the four trapezoid numbers become those of the outline (root chord, tip chord, span,
    /// sweep of the tip leading edge) so that everything that reads them stays consistent; the mesher follows the outline.
    pub fn with_planform(mut self, pf: Planform) -> Fins3 {
        self.root_chord = pf.root_chord();
        self.tip_chord = pf.tip_chord();
        self.span = pf.span();
        self.sweep = pf.sweep();
        self.planform = Some(pf);
        self
    }
}

/// Where the fins sit relative to the pitch plane of a half domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FinPhase {
    /// Fins at `pi/n + 2 pi m/n`: the pitch plane lies between two fins (the "x" orientation of four fins).
    Between,
    /// Fins at `2 pi m/n`: a fin lies in the pitch plane at `theta = 0` (the "+" orientation of four fins).
    InPlane,
}

/// The solved part of the body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Domain3 {
    /// `theta` in `[0, pi]`, the pitch plane a symmetry plane: any incidence in the pitch plane.
    Half { phase: FinPhase },
    /// `theta` in `[0, pi/n]` of a body with `n` fins (`fins` required), zero incidence.
    Sector,
    /// A single cell of angle `dtheta` of an axisymmetric flow (no fins), zero incidence; `nk` is ignored.
    Wedge { dtheta: f64 },
}

/// Default angle of a wedge domain, 2 degrees. The nodes of a revolved grid are area-preserving (`bowshock_cfd::grid3::polygon_scale`); against 2 degrees the
/// drag of the 2D-parity body (Mach 0.9 / Mach 2) changes by 0.000 % / 0.003 % at 1 degree, 0.004 % / 0.035 % at 6 and 0.018 % / 0.153 % at 12 (the
/// inscribed polygon alone gave -0.76 % and -1.17 % at 12 degrees).
pub const WEDGE_ANGLE: f64 = 2.0 * PI / 180.0;

/// Azimuthal spacing next to the fin sides of a wall-resolved grid: the cells of every gap grow from the fin face, `growth_bl` inside the boundary layer
/// (`delta`) and `growth_out` beyond it, until two stacks meet in mid-gap (a gap between two fins) or the mid-gap plane is reached (a gap next to a symmetry
/// plane). The heights are the lateral distances at the fin root radius; the nodes of a `k` line are on one azimuth for every radius, so the cells are
/// `r / r_root` times higher at radius `r`. The number of azimuthal cells follows from the stacks (`Grid3Params::nk` is not used).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FinStack {
    /// Height of the first cell next to a fin side \[d\].
    pub y1: f64,
    /// Thickness of the turbulent boundary layer on the fin \[d\]: the growth switches from `growth_bl` to `growth_out` there.
    pub delta: f64,
    pub growth_bl: f64,
    pub growth_out: f64,
}

impl FinStack {
    /// The stack for `y+ = y_plus` of the first cell on a flat plate of the fin's root chord `chord` \[d\] at the free-stream state of `re_c = rho c d / mu`
    /// and the largest Mach number `mach` of the grid: the friction of `Case::viscous` (`cf = 0.027 Re_x^(-1/7)`), `delta = 0.37 chord Re_x^(-1/5)`, growth
    /// 1.25 in the layer and 1.35 outside.
    pub fn from_flow(re_c: f64, mach: f64, chord: f64, y_plus: f64) -> FinStack {
        let re_d = re_c * mach;
        let re_x = re_d * chord;
        let cf = 0.027 * re_x.powf(-1.0 / 7.0);
        FinStack { y1: y_plus / (re_d * (0.5 * cf).sqrt()), delta: 0.37 * chord * re_x.powf(-0.2), growth_bl: 1.25, growth_out: 1.35 }
    }
}

/// Grid parameters: the meridian grid of the 2D case (sting grid, or base-wake grid when `base` is set) plus the fins, the domain and the
/// azimuthal resolution.
#[derive(Clone, Debug)]
pub struct Grid3Params {
    pub sting: LargeGridParams,
    /// The base-wake meridian grid: used instead of `sting` when set (the product grid; fins stand on it as on the sting grid).
    pub base: Option<BaseGridParams>,
    pub fins: Option<Fins3>,
    /// Azimuthal cells over the domain (the plan default is `24 << level` over `[0, pi]`, see [`Grid3Params::default_nk`]); ignored with a `fin_stack`.
    pub nk: usize,
    /// Grid level: sets the cells over the fin root chord (`32 << level`) unless `fin_chord_cells` overrides it.
    pub level: u32,
    /// Cells over the fin root chord (`None`: `32 << level`; the ANF study found 16 too coarse for the fin loading at M >= 2).
    pub fin_chord_cells: Option<usize>,
    /// Cells across the thickness of a fin (>= 1; 1 is the one-cell layer of the first grids, bit for bit). They are in `k`, for the solid fin and for the fluid columns
    /// ahead of the leading edge and behind the trailing edge (the base region of a blunt trailing edge has that many cells across); a half fin on a symmetry plane
    /// (a sector, an in-plane fin of a half domain) owns half of them, rounded down, at least one. With a `fin_stack` they are added to the stack's cells; without, they
    /// come out of the `nk` of the domain.
    pub fin_cells_across: usize,
    /// Radial cells from the body wall to the fin tip (`None`: whatever the sting grid's radial spacing puts there, with the tip
    /// row snapped to the nearest node). With `Some(n)` the radial fractions are regraded for the whole grid: `n` cells in
    /// `[R, R + span]` starting at the wall spacing (uniform when that is already finer than `span / n`), then a geometric fan of
    /// at least the sting grid's outer cell count with growth `<= 1.2` to the far boundary. `nj` grows accordingly.
    pub span_cells: Option<usize>,
    pub domain: Domain3,
    /// Wall-resolved azimuthal spacing next to the fin sides (`None`: uniform cells in every gap, `nk` of them over the domain).
    pub fin_stack: Option<FinStack>,
    /// Roughness of the body and the fins, carried on the grid for the wall treatment ([`Grid3::wall_ks`]).
    pub roughness: PartRoughness,
    /// Rounded fins keep the first chord cell of the root row down the span (`FinPlan::row_offsets`, DESIGN3D item 66); off by default: it makes the supersonic Rabia points
    /// of the Coarse tier solve (Mach 1.3 / 2.3 in 1 200 iterations) and its Mach 0.8 point fail.
    pub fin_le_rows: bool,
}

impl Grid3Params {
    /// Default azimuthal cell count of a grid level.
    pub fn default_nk(level: u32) -> usize {
        24usize << level
    }

    /// Half domain on the sting grid, fins between the pitch plane and each other (the layout of the incidence solves).
    pub fn half(sting: LargeGridParams, fins: Option<Fins3>, nk: usize, level: u32) -> Grid3Params {
        Grid3Params {
            sting,
            base: None,
            fins,
            nk,
            level,
            fin_chord_cells: None,
            span_cells: None,
            domain: Domain3::Half { phase: FinPhase::Between },
            fin_stack: None,
            fin_cells_across: 1,
            roughness: PartRoughness::default(),
            fin_le_rows: false,
        }
    }
}

impl Grid3Params {
    /// The grid one sequencing level down (DESIGN3D S9): every axial and radial cell count halved (semi-coarsening in `i` and `j`; the azimuth, with its fin stack, is
    /// kept), so that the cells of this grid are the 2 x 2 children (in `i`, `j`) of the cells of the result: the section ends (nose, base plane, segment ends, body
    /// surface, base radius) and the fin leading / trailing edges and tip row fall on even indices. `Err` names a count that is odd (a diameter step is one cell and
    /// cannot be halved; a nozzle's rows are not halved).
    pub fn coarsened(&self) -> Result<Grid3Params, String> {
        let half = |what: &str, n: usize| -> Result<usize, String> {
            if n >= 2 && n.is_multiple_of(2) {
                Ok(n / 2)
            } else {
                Err(format!("{what} = {n} cannot be halved"))
            }
        };
        let mut c = self.clone();
        let segs = |what: &str, n_seg: &[usize]| -> Result<Vec<usize>, String> { n_seg.iter().map(|&n| half(what, n)).collect() };
        c.sting.n_up = half("sting n_up", self.sting.n_up)?;
        c.sting.n_nose = half("sting n_nose", self.sting.n_nose)?;
        c.sting.n_seg = segs("sting n_seg", &self.sting.n_seg)?;
        c.sting.n_ext = half("sting n_ext", self.sting.n_ext)?;
        c.sting.nj = half("sting nj", self.sting.nj)?;
        if let Some(b) = &self.base {
            if b.nozzle.is_some() {
                return Err("a motor's nozzle rows are not halved".to_string());
            }
            let cb = c.base.as_mut().expect("base");
            cb.n_up = half("base n_up", b.n_up)?;
            cb.n_nose = half("base n_nose", b.n_nose)?;
            cb.n_seg = segs("base n_seg", &b.n_seg)?;
            cb.n_wake = half("base n_wake", b.n_wake)?;
            cb.n_gap = half("base n_gap", b.n_gap)?;
            cb.nj_out = half("base nj_out", b.nj_out)?;
            if b.n_sting > 0 {
                cb.n_sting = half("base n_sting", b.n_sting)?;
            }
        }
        if self.fins.is_some() {
            c.fin_chord_cells = Some(half("cells over the fin root chord", self.fin_chord_cells.unwrap_or(32usize << self.level))?);
            if let Some(n) = self.span_cells {
                c.span_cells = Some(half("span_cells", n)?);
            }
        }
        Ok(c)
    }
}

/// Half thickness of the fin section at the chordwise fraction `xi` (chord length `chord`, maximum thickness `t`). At an end that is really sharp (the section's
/// thickness there is below 5 % of `t/2`: a wedge, an airfoil, the circle of a rounded edge) it is floored at 2 % of `t/2`, a needle the nodes follow (a zero-area
/// edge face, the polygon of the clustered nodes for a round edge); at a blunt end (a square section, a blunt trailing edge, a leading edge of radius `0.1 t/2`) [`BLUNT_EDGE_FLOOR`] of `t/2` (it was 25 %: a blunt edge below that, the 0.004 cal leading edge of the Basic Finner against 0.04 cal of half thickness, was inflated 2.5 times).
/// Smallest half thickness of a blunt fin edge as a fraction of `t/2`.
const BLUNT_EDGE_FLOOR: f64 = 0.1;

pub(crate) fn half_thickness(profile: &FinProfile, t: f64, chord: f64, xi: f64) -> f64 {
    let r = 0.5 * t;
    let raw = |xi: f64| -> f64 {
        let xi = xi.clamp(0.0, 1.0);
        match profile {
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
            FinProfile::Hexagonal { wedge_fraction } => r * (xi / wedge_fraction).min((1.0 - xi) / wedge_fraction).min(1.0),
            FinProfile::Biconvex => r * 4.0 * xi * (1.0 - xi),
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
        }
    };
    let xi = xi.clamp(0.0, 1.0);
    let sharp_end = raw(if xi < 0.5 { 0.0 } else { 1.0 }) < 0.05 * r;
    raw(xi).max(if sharp_end { 0.02 * r } else { BLUNT_EDGE_FLOOR * r })
}

/// Node-index layout of the fins on the axial/radial lines.
struct FinPlan {
    /// Node range `a..=b` of the rebuilt segments under the fin root (one tube, or several segments).
    i_seg0: usize,
    i_seg1: usize,
    /// The node where the nose ends: the farthest the sweep zone ([`ZONE_REACH`]) reaches upstream.
    i_floor: usize,
    i_le: usize,
    i_te: usize,
    j_tip: usize,
    x_le_root: f64,
    x_te_root: f64,
    /// The largest body radius under the root (the reference of the stack and of the span), and the tip radius `r_t + span`: the tip stands on one cylinder
    /// whatever the body radius under the root does.
    r_t: f64,
    r_tip: f64,
    /// The span the grid meshes: the planform's, cut where the chord of a pointed or elliptical outline falls to [`TIP_CHORD_FLOOR`] of the root chord.
    span: f64,
    /// The outline (`None`: the trapezoid of the four numbers of `Fins3`).
    planform: Option<Planform>,
    /// Radial fractions of the node rows, including the extra rows of `span_cells` (`nj + 1`).
    sj: Vec<f64>,
    /// The chord nodes of the root row (offsets from the root leading edge, `nf + 1`), the first cell the clustering aimed at and whether the rows above the root keep it
    /// (a rounded section: [`FinPlan::row_offsets`]).
    fin_offs: Vec<f64>,
    edge: f64,
    round_rows: bool,
}

impl FinPlan {
    /// Radial fraction (of the way from the body surface to the outer boundary) of the node row `jj` above the surface in a column whose tip row sits at the
    /// fraction `target` (`(r_tip - r_body) / (r_far - r_body)`): the rows below the tip scale with it, the rows above it fan to the boundary.
    fn remap(&self, jj: usize, target: f64) -> f64 {
        let tip = self.sj[self.j_tip];
        if jj <= self.j_tip {
            self.sj[jj] * target / tip
        } else {
            target + (self.sj[jj] - tip) * (1.0 - target) / (1.0 - tip)
        }
    }

    /// Fraction of the span (from the body surface to the tip) of the node row `jj`.
    fn span_fraction(&self, jj: usize) -> f64 {
        self.sj[jj.min(self.j_tip)] / self.sj[self.j_tip]
    }

    /// Chord nodes (offsets from the leading edge of the row) of a row whose chord is `chord`. The rows of a tapered fin used to be the root row scaled with the chord, so
    /// the first cell of a round edge of constant radius shrank with it (the Rabia fixture: a fourth of the radius at the root, a sixteenth at 76 % span, a cell 100 to 450
    /// times shorter than the row is high, and the collapse that started there); a rounded section keeps the root's first cell down the span until the chord itself gives
    /// `0.3 chord / nf` (item 66). The root row and any row of the root's chord are the root's nodes.
    fn row_offsets(&self, chord: f64) -> Vec<f64> {
        let nf = self.fin_offs.len() - 1;
        let c_root = self.fin_offs[nf];
        if !self.round_rows || chord <= 1e-12 || (chord - c_root).abs() <= 1e-12 * c_root {
            return self.fin_offs.iter().map(|o| o * chord / c_root).collect();
        }
        let edge = (0.3 * chord / nf as f64).min(self.edge);
        let fresh = two_sided_nodes(chord, nf, edge);
        // the nodes of the root row that sit on a body station stay where they were, scaled
        let root = two_sided_nodes(c_root, nf, self.edge);
        fresh.iter().zip(self.fin_offs.iter().zip(&root)).map(|(f, (o, r))| f + (o - r) * chord / c_root).collect()
    }
}

/// Azimuthal layer structure: fin angles, the number of cells in each gap and where the nodes of a gap lie.
struct Layers {
    theta_f: Vec<f64>,
    /// The first / last layer is a half fin on the `theta = 0` / end plane: it owns only its own half thickness and has no gap before / after it.
    half_first: bool,
    half_last: bool,
    /// Cells of the gap before each layer (0 before a half first layer) and one trailing gap behind the last (0 behind a half last layer).
    gaps: Vec<usize>,
    /// Node positions of each gap as fractions of its width, `gaps[s] + 1` of them from 0 to 1 (empty for an absent gap).
    gap_frac: Vec<Vec<f64>>,
    /// First `k` of the cells owned by each layer.
    k_f: Vec<usize>,
    /// Cells owned by each layer: [`Grid3Params::fin_cells_across`], half of them (at least one) for a half fin on a symmetry plane.
    n_f: Vec<usize>,
    /// Azimuth width of the cells next to a layer: what a layer cell relaxes to away from the fin.
    cell_ang: f64,
}

/// Fin azimuths of a half domain `[0, pi]` with `count` fins at `phase`, and whether the first / last is a half fin on the `theta = 0` / `pi` plane.
fn fin_angles(count: u32, phase: FinPhase) -> (Vec<f64>, bool, bool) {
    let n = f64::from(count);
    let phi0 = match phase {
        FinPhase::Between => PI / n,
        FinPhase::InPlane => 0.0,
    };
    let mut theta_f = Vec::new();
    for m in 0..count {
        let th = phi0 + 2.0 * PI * f64::from(m) / n;
        if th <= PI + 1e-9 {
            theta_f.push(if th < 1e-9 {
                0.0
            } else if (PI - th).abs() < 1e-9 {
                PI
            } else {
                th
            });
        }
    }
    let (first, last) = (theta_f[0], theta_f[theta_f.len() - 1]);
    (theta_f, first == 0.0, last == PI)
}

/// Cells owned by each of `nl` layers for `n` cells across a fin: half of them (at least one) for the half fins on the symmetry planes.
fn layer_counts(n: usize, nl: usize, half_first: bool, half_last: bool) -> Vec<usize> {
    (0..nl).map(|l| if (half_first && l == 0) || (half_last && l + 1 == nl) { (n / 2).max(1) } else { n }).collect()
}

/// First `k` of the cells owned by each layer (`n_f` cells), for the cells of every gap.
fn layer_cells(gaps: &[usize], n_f: &[usize]) -> Vec<usize> {
    let mut k = 0;
    (0..gaps.len() - 1)
        .map(|l| {
            k += gaps[l];
            let first = k;
            k += n_f[l];
            first
        })
        .collect()
}

fn uniform_fractions(gaps: &[usize]) -> Vec<Vec<f64>> {
    gaps.iter().map(|&n| if n == 0 { Vec::new() } else { (0..=n).map(|m| m as f64 / n as f64).collect() }).collect()
}

/// Layers of a half domain `[0, pi]` with `count` fins at `phase` of `n` cells across, `nk` cells in all, uniform in every gap.
fn layers(count: u32, nk: usize, phase: FinPhase, n: usize) -> Result<Layers, Error> {
    let (theta_f, half_first, half_last) = fin_angles(count, phase);
    let nl = theta_f.len();
    let n_f = layer_counts(n, nl, half_first, half_last);
    let owned: usize = n_f.iter().sum();
    let nh = n as f64 * PI / (2.0 * nk as f64);
    // gap `s` (before layer `s`, `s = nl`: behind the last) between the end of the previous layer and the start of the next
    let present: Vec<bool> = (0..=nl).map(|s| !(s == 0 && half_first) && !(s == nl && half_last)).collect();
    let mut widths = vec![0.0; nl + 1];
    let mut prev = 0.0;
    for (l, &th) in theta_f.iter().enumerate() {
        let (hf, hl) = (half_first && l == 0, half_last && l + 1 == nl);
        let lo = if hl {
            PI - nh
        } else if hf {
            0.0
        } else {
            th - nh
        };
        widths[l] = if hf { 0.0 } else { lo - prev };
        prev = if hl {
            PI
        } else if hf {
            nh
        } else {
            th + nh
        };
    }
    widths[nl] = if half_last { 0.0 } else { PI - prev };
    let slots = present.iter().filter(|p| **p).count();
    let too_close = || Error::Grid("fins too close for nk cells".to_string());
    if nk < owned || (0..=nl).any(|s| present[s] && widths[s] <= 0.0) || nk - owned < 2 * slots {
        return Err(too_close());
    }
    let remaining = nk - owned;
    let total: f64 = widths.iter().sum();
    let q: Vec<f64> = widths.iter().map(|w| remaining as f64 * w / total).collect();
    let mut gaps: Vec<usize> = (0..=nl).map(|s| if present[s] { (q[s].round() as usize).max(2) } else { 0 }).collect();
    while gaps.iter().sum::<usize>() != remaining {
        if gaps.iter().sum::<usize>() > remaining {
            let i = (0..=nl).filter(|&i| present[i] && gaps[i] > 2).max_by(|&a, &b| (gaps[a] as f64 - q[a]).total_cmp(&(gaps[b] as f64 - q[b])));
            gaps[i.ok_or_else(too_close)?] -= 1;
        } else {
            let i = (0..=nl).filter(|&i| present[i]).max_by(|&a, &b| (q[a] - gaps[a] as f64).total_cmp(&(q[b] - gaps[b] as f64))).unwrap();
            gaps[i] += 1;
        }
    }
    let k_f = layer_cells(&gaps, &n_f);
    let gap_frac = uniform_fractions(&gaps);
    Ok(Layers { theta_f, half_first, half_last, gaps, gap_frac, k_f, n_f, cell_ang: PI / nk as f64 })
}

/// Layers of a sector `[0, big]`, `big = pi/n`: a half fin on the `theta = 0` plane (`n` cells across a whole fin, half of them here) and the other cells of the `nk`
/// as the gap to the mid-gap plane.
fn sector_layers(nk: usize, big: f64, n: usize) -> Result<Layers, Error> {
    let c = (n / 2).max(1);
    if nk < c + 2 {
        return Err(Error::Grid(format!("a sector needs nk >= {}", c + 2)));
    }
    let gaps = vec![0, nk - c];
    Ok(Layers {
        theta_f: vec![0.0],
        half_first: true,
        half_last: false,
        gap_frac: uniform_fractions(&gaps),
        gaps,
        k_f: vec![0],
        n_f: vec![c],
        cell_ang: big / nk as f64,
    })
}

/// Cell heights of a stack that starts with `h1` at a wall and grows by `growth_bl` until it is `delta` high and by `growth_out` beyond, over a gap of height
/// `total`: the fewest cells that reach `total` at those growth rates, with the growth increments scaled down together (a common factor `lam <= 1`) so that
/// the stack closes the gap exactly. The first cell stays `h1` and no growth ratio exceeds the design's.
fn stack_sizes(total: f64, st: &FinStack, h1: f64, delta: f64) -> Vec<f64> {
    let sizes_at = |n: usize, lam: f64| -> Vec<f64> {
        let (qb, qo) = (1.0 + lam * (st.growth_bl - 1.0), 1.0 + lam * (st.growth_out - 1.0));
        let (mut sum, mut h) = (0.0, h1);
        (0..n)
            .map(|_| {
                let s = h;
                sum += s;
                h *= if sum < delta { qb } else { qo };
                s
            })
            .collect()
    };
    let mut n = 1;
    while sizes_at(n, 1.0).iter().sum::<f64>() < total && n < 1000 {
        n += 1;
    }
    if n == 1 {
        return vec![total];
    }
    if n as f64 * h1 >= total {
        return vec![total / n as f64; n];
    }
    let (mut lo, mut hi) = (0.0, 1.0);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if sizes_at(n, mid).iter().sum::<f64>() < total {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let mut sizes = sizes_at(n, hi);
    let scale = total / sizes.iter().sum::<f64>();
    sizes.iter_mut().for_each(|s| *s *= scale);
    sizes
}

/// Node positions (fractions of `width`) of a gap with a fin wall at its low end (`wall_lo`), at its high end (`wall_hi`) or at both (the stacks meet in the
/// middle); `y1` and `delta` are the first cell and the boundary layer in azimuth.
fn stack_fractions(width: f64, wall_lo: bool, wall_hi: bool, st: &FinStack, y1: f64, delta: f64) -> Vec<f64> {
    let mut sizes = match (wall_lo, wall_hi) {
        (true, true) => {
            let half = stack_sizes(0.5 * width, st, y1, delta);
            half.iter().chain(half.iter().rev()).copied().collect()
        }
        (true, false) => stack_sizes(width, st, y1, delta),
        (false, true) => {
            let mut s = stack_sizes(width, st, y1, delta);
            s.reverse();
            s
        }
        (false, false) => vec![0.5 * width; 2],
    };
    if sizes.len() < 2 {
        sizes = vec![0.5 * sizes[0]; 2];
    }
    let mut acc = 0.0;
    let mut frac = vec![0.0];
    for s in &sizes {
        acc += s;
        frac.push(acc / width);
    }
    *frac.last_mut().expect("nodes") = 1.0;
    frac
}

/// Layers whose gaps are stacked from the fin sides: `fins` is `Some((count, phase))` for a half domain, `None` for a sector `[0, big]` with its half fin on
/// `theta = 0`. `r_root` and `h_max` are the fin's root radius and its largest half-thickness (the nominal angle of the fin side).
fn stacked_layers(fins: Option<(u32, FinPhase)>, st: &FinStack, r_root: f64, h_max: f64, big: f64, n: usize) -> Result<Layers, Error> {
    let (theta_f, half_first, half_last) = match fins {
        Some((count, phase)) => fin_angles(count, phase),
        None => (vec![0.0], true, false),
    };
    let nl = theta_f.len();
    let alpha_ref = (h_max / r_root).min(0.99).asin();
    let (y1, delta) = (st.y1 / r_root, st.delta / r_root);
    let mut gaps = vec![0; nl + 1];
    let mut gap_frac = vec![Vec::new(); nl + 1];
    for s in 0..=nl {
        if (s == 0 && half_first) || (s == nl && half_last) {
            continue;
        }
        // the gap between the previous fin side (or the plane at 0) and the next one (or the end plane)
        let lo = if s == 0 { 0.0 } else { theta_f[s - 1] + alpha_ref };
        let hi = if s == nl { big } else { theta_f[s] - alpha_ref };
        if hi - lo <= 2.0 * y1 {
            return Err(Error::Grid("fins too close for the wall-resolved stack".to_string()));
        }
        let frac = stack_fractions(hi - lo, s > 0, s < nl, st, y1, delta);
        gaps[s] = frac.len() - 1;
        gap_frac[s] = frac;
    }
    let n_f = layer_counts(n, nl, half_first, half_last);
    let k_f = layer_cells(&gaps, &n_f);
    Ok(Layers { theta_f, half_first, half_last, gaps, gap_frac, k_f, n_f, cell_ang: y1 })
}

/// Azimuth of the nodes `k = 0..=nk` of one `(i, j)` column for the layer half-angle `alpha` in the domain `[0, big]`.
fn fill_theta(theta: &mut [f64], lay: &Layers, alpha: f64, big: f64) {
    let nk = theta.len() - 1;
    theta[0] = 0.0;
    let (mut k, mut prev) = (0usize, 0.0f64);
    let nl = lay.theta_f.len();
    for (l, &th) in lay.theta_f.iter().enumerate() {
        let (hf, hl) = (lay.half_first && l == 0, lay.half_last && l + 1 == nl);
        let lo = if hl {
            big - alpha
        } else if hf {
            0.0
        } else {
            th - alpha
        };
        let hi = if hl {
            big
        } else if hf {
            alpha
        } else {
            th + alpha
        };
        let ng = lay.gaps[l];
        for m in 1..=ng {
            theta[k + m] = prev + (lo - prev) * lay.gap_frac[l][m];
        }
        k += ng;
        // the nodes across the layer: uniform between its two faces
        let c = lay.n_f[l];
        for m in 1..c {
            theta[k + m] = lo + (hi - lo) * m as f64 / c as f64;
        }
        theta[k + c] = hi;
        k += c;
        prev = hi;
    }
    let ng = lay.gaps[nl];
    for m in 1..=ng {
        theta[k + m] = prev + (big - prev) * lay.gap_frac[nl][m];
    }
    theta[nk] = big;
}

/// Columns over which the layer behind a flush fin trailing edge tapers from the fin's thickness (at the base plane) to the nominal cell width. The layer cell behind the trailing
/// edge is as wide as the fin (0.074 d across on the ANF Lift-tier grid) and its two neighbours are the thin first cells of the fin stack (7e-4 d): over 4 columns (the value of the
/// other fin edges) its side faces tilt by up to 23 degrees against the axis (the cosine between a face normal and the line to the lower neighbour is 0.918 in `k`, 0.934 in `i`)
/// and its volume falls by up to a factor 10 per column. The viscous Lift-tier half domain with fins is then non-finite in iteration 150 to 190 at `cfl_max` 20 (Mach 2) and
/// 50 (Mach 0.8); the first cells to go lie right behind the trailing edge at the body radius (`DESIGN3D.md`, "The viscous half domain at incidence"). Over 8 columns the cosines
/// are 0.968 and 0.986 and the Mach 2 runs hold at `cfl_max` 20 and 30 (4 columns: 0.918 / 0.934, non-finite; 2: 0.83 / 0.85, earlier).
const TE_TAPER_COLUMNS: f64 = 8.0;

fn smoothstep(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * (3.0 - 2.0 * u)
}

/// Choose the first candidate `(node, distance, outer cell width)` that a geometric fan of `m` cells starting at `dx` with growth
/// `q <= 1.2` reaches with a last cell within `join` of the outer width (1.4 ahead of the fin, 1.25 behind a flush trailing edge on the base-wake grid, where the fan is the whole transition); returns `(node, m, q)` (the best compromise if none does).
fn fan(dx: f64, cands: &[(usize, f64, f64)], join: f64) -> Option<(usize, usize, f64)> {
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
            let score = (q.max(1.0 / q) / 1.2).max((last / w).max(w / last) / join);
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

/// What the fins sit on: the body of the meridian grid, the cell counts that find its segments among the axial nodes, and the radial fractions of the rows above
/// the body surface (`sj0`, `0` at the surface, `1` at the outer boundary, first cell `dr_wall` of the grid).
struct Host<'a> {
    body: &'a BaseBody,
    n_up: usize,
    n_nose: usize,
    n_seg: &'a [usize],
    r_far: f64,
    sj0: Vec<f64>,
    /// The nodes behind the base are the wake grid's and stay as the finless grids have them (flush fins do not regrade the wake: the inviscid base drag
    /// depends on the first wake cells, by 10 % for a cell a third as long). On the sting grid the body goes on behind the base and the fan may use it.
    keep_wake: bool,
}

/// Rebuild the tube segment that carries the fins in `xs` and compute the radial remap.
fn plan_fins(p: &Grid3Params, f: &Fins3, xs: &mut Vec<f64>, host: &Host) -> Result<FinPlan, Error> {
    let (nj0, r_far) = (host.sj0.len() - 1, host.r_far);
    let bad = |m: &str| Err(Error::InvalidInput(m.to_string()));
    let ok = |v: f64| v.is_finite();
    let custom = f.planform.is_some();
    if let Some(pf) = &f.planform {
        if (pf.root_chord() - f.root_chord).abs() > 1e-9 * (1.0 + f.root_chord) {
            return bad("Fins3.root_chord disagrees with the planform: build the fins with Fins3::with_planform");
        }
    }
    if !(f.count >= 1 && ok(f.root_chord) && f.root_chord > 0.0 && (custom || (ok(f.tip_chord) && f.tip_chord >= 0.0 && ok(f.span) && f.span > 0.0)))
        || !(ok(f.sweep) && ok(f.thickness) && f.thickness > 0.0 && ok(f.te_to_base) && f.te_to_base >= 0.0)
    {
        return bad("3D fins need count >= 1, positive root chord, span and thickness, tip chord >= 0 and te_to_base >= 0");
    }
    let bb = host.body;
    let x_te_root = bb.x_base() - f.te_to_base;
    let x_le_root = x_te_root - f.root_chord;
    let nseg = bb.segments.len();
    let seg_end = |k: usize| bb.station(k).0 + bb.segments[k].length;
    // the aft segments under the root: the one that holds the leading edge to the one that holds the trailing edge (tubes, boattails, flares, any number)
    let k0 = (0..nseg).find(|&k| x_le_root >= bb.station(k).0 - 1e-9 && x_le_root < seg_end(k) - 1e-9).ok_or_else(|| {
        Error::Unsupported(format!(
            "the fin root leading edge (x = {x_le_root:.4} d) lies ahead of the aft body (the nose ends at {:.4} d) or behind its last segment: fins stand on the aft segments",
            bb.body.nose_length()
        ))
    })?;
    let k1 = (k0..nseg).find(|&k| x_te_root > bb.station(k).0 + 1e-9 && x_te_root <= seg_end(k) + 1e-9).ok_or_else(|| {
        Error::Unsupported(format!("the fin root trailing edge (x = {x_te_root:.4} d) lies behind the body base or ahead of its leading edge"))
    })?;
    if let Some(k) = (k0..=k1).find(|&k| bb.segments[k].is_step()) {
        return Err(Error::Unsupported(format!(
            "the fin root crosses a diameter step (segment {k}, x = {:.4} to {:.4} d): fins stand on tubes, boattails and flares",
            bb.station(k).0,
            seg_end(k)
        )));
    }
    let a = host.n_up + host.n_nose + host.n_seg[..k0].iter().sum::<usize>();
    let b_old = host.n_up + host.n_nose + host.n_seg[..=k1].iter().sum::<usize>();
    let (x_a, x_b) = (xs[a], xs[b_old]);
    let x_le_root = if x_le_root - x_a > 1e-9 { x_le_root } else { x_a };
    let x_te_root = if x_b - x_te_root > 1e-9 { x_te_root } else { x_b };
    let flush = x_te_root >= x_b;
    // the body radius under the root: piecewise linear, its largest value is at an end or a station between the segments; the tip stands `span` above it
    let r_ref = (k0 + 1..=k1).map(|k| bb.station(k).1).chain([bb.r(x_le_root), bb.r(x_te_root)]).fold(0.0, f64::max);
    let nf = p.fin_chord_cells.unwrap_or(32usize << p.level);
    if nf < 2 {
        return Err(Error::InvalidInput("3D fins need at least 2 cells over the root chord".to_string()));
    }
    // minimum chord spacing 0.3 c / nf at the edges: 0.2 resolves the blunt leading edge like 48 cells (ANF face Cp 0.78 -> 1.02) at +1 % cells, 0.1 and below diverge on the
    // example rocket (DESIGN3D item 44)
    let mut edge = 0.3 * f.root_chord / nf as f64;
    if matches!(f.profile, FinProfile::Rounded) {
        // a round edge of radius `t / 2` stands in the grid as a polygon of a few cells: with the first cell as wide as the radius its corners turn the flow by 60 degrees and the
        // swept thick fin of the Rabia fixture (radius 0.03 d, cell 0.031 d) grew a vacuum (p 1e-5, Mach 900) at its leading edge that no CFL cured; a first cell of a fourth of
        // the radius (not below a tenth of the chord's cell: that and below diverged on the example rocket) settles at Mach 0.8 in 770 iterations (Draft)
        edge = edge.min((0.5 * f.thickness / LE_RADIUS_CELLS).max(0.1 * f.root_chord / nf as f64));
    }
    let mut fin_offs = two_sided_nodes(f.root_chord, nf, edge);
    fin_offs[nf] = x_te_root - x_le_root;
    // the stations between the segments under the root stay nodes (the body parts are split there): the nearest chord node moves onto each
    for k in k0 + 1..=k1 {
        let off = bb.station(k).0 - x_le_root;
        if off > 1e-9 && off < fin_offs[nf] - 1e-9 {
            let m = (1..nf).min_by(|&p, &q| (fin_offs[p] - off).abs().total_cmp(&(fin_offs[q] - off).abs())).unwrap();
            fin_offs[m] = off;
        }
    }
    if fin_offs.windows(2).any(|w| w[1] - w[0] <= 1e-9) {
        return Err(Error::Unsupported(
            "two segment stations lie within one chord cell of each other under the fin root: use more cells over the root chord".to_string(),
        ));
    }
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
    if let Some((o, m, q)) = fan(dx_f, &up, 1.4) {
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
    let (o_down, m_down) = match fan(dx_f, &down, if flush && host.keep_wake { 1.25 } else { 1.4 }) {
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

    let r_t = r_ref;
    // the grid cannot take a chord that goes to zero: a pointed or elliptical planform is cut where its chord has fallen to a few per cent of the root's
    let span = match &f.planform {
        Some(pf) => pf.height_with_chord_at_least(TIP_CHORD_FLOOR * f.root_chord),
        None => f.span,
    };
    if r_far.is_nan() || r_far <= r_t + span || nj0 < 6 {
        return Err(Error::Grid("the fin tip must lie inside the domain".to_string()));
    }
    let sj0 = host.sj0.clone();
    let target = span / (r_far - r_t);
    let j_tip0 = (1..=nj0).min_by(|&a, &b| (sj0[a] - target).abs().total_cmp(&(sj0[b] - target).abs())).unwrap();
    let (sj, s_remap, j_tip) = match p.span_cells {
        None => {
            let s_remap: Vec<f64> = (0..=nj0)
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
            let mut m = (nj0 - j_tip0).max(4);
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
    // the outline must stay inside the rebuilt segment: not ahead of its start, not behind the root trailing edge room (flush: not behind the base)
    let room = if i_te == i_seg1 { 1e-9 } else { 0.5 * (x_b - x_te_root) };
    let (x_front, x_back) = match &f.planform {
        Some(pf) => pf.x_extent_up_to(span),
        None => (f.sweep.min(0.0), (f.sweep + f.tip_chord).max(f.root_chord)),
    };
    let x_floor = xs[host.n_up + host.n_nose];
    if x_le_root + x_front < x_floor + ZONE_REACH.recip() * (x_le_root - x_floor) {
        return Err(Error::Unsupported(format!(
            "the fin outline reaches {:.4} d ahead of its root leading edge, too close to the end of the nose ({:.4} d ahead of the root)",
            -x_front,
            x_le_root - x_floor
        )));
    }
    if x_le_root + x_back > x_te_root + room {
        return Err(Error::Unsupported(format!(
            "the fin outline reaches {:.4} d behind its root trailing edge{}: a fin may not overhang the base",
            x_le_root + x_back - x_te_root,
            if i_te == i_seg1 { " (flush with the base)" } else { "" }
        )));
    }
    Ok(FinPlan {
        i_seg0: a,
        i_seg1,
        i_floor: host.n_up + host.n_nose,
        i_le,
        i_te,
        j_tip,
        x_le_root,
        x_te_root,
        r_t,
        r_tip: r_ref + span,
        span,
        planform: f.planform.clone(),
        sj,
        fin_offs,
        edge,
        round_rows: matches!(f.profile, FinProfile::Rounded) && p.fin_le_rows,
    })
}

/// Azimuth span and symmetry of the domain of `p`.
fn domain_span(p: &Grid3Params) -> Result<(f64, Symmetry), Error> {
    Ok(match (p.domain, &p.fins) {
        (Domain3::Half { .. }, _) => (PI, Symmetry::Half),
        (Domain3::Sector, Some(f)) if f.count >= 1 => (PI / f64::from(f.count), Symmetry::Sector { n: f.count }),
        (Domain3::Sector, _) => return Err(Error::InvalidInput("a sector needs fins".to_string())),
        (Domain3::Wedge { dtheta }, None) if dtheta > 0.0 && dtheta < PI => (dtheta, Symmetry::Wedge),
        (Domain3::Wedge { .. }, _) => return Err(Error::InvalidInput("a wedge needs 0 < dtheta < pi and no fins".to_string())),
    })
}

/// Build the 3D grid. With fins the grid has `nk` cells in `k` and the axial count of the meridian grid changed by the rebuilt fin segment; the rows are
/// those of the sting grid (`sting.nj`) or the base-wake grid (`n_sting + n_gap + nj_out`), regraded so that a node row lies on the fin tip.
///
/// On the base-wake grid the fins stand on the rows above the body surface (`j >= j_off = n_sting + n_gap`, the rows below are the solid body block and the wake
/// gap). A fin whose trailing edge is flush with the base (`te_to_base = 0`) ends on the base plane: its trailing-edge face (`+i`, rows `j_off..j_off + j_tip`)
/// is coplanar with the base face (rows below `j_off`, the body block's `+i` face) and joins it along the root of the trailing edge, so the closed body is one
/// plane `x = x_base` there. Behind the plane the layer continues as fluid cells that taper over four columns to the nominal width; below the base radius
/// (the wake rows under the surface) the layer keeps the angle it has on the surface row, so it is a thin sector that narrows to the axis, never a jump.
pub fn body_grid3(p: &Grid3Params) -> Result<Grid3, Error> {
    let first = body_grid3_inner(p);
    let Err(Error::Grid(m)) = &first else { return first };
    if !(m.starts_with("negative cell volume") || m.starts_with("non-positive fluid cell volume")) || p.fins.is_none() || p.fin_stack.is_some() || p.nk < 2 {
        return first;
    }
    // uniform azimuthal cells beside a fin that fold: name the cause and the smallest `nk` that does not (a negative volume is never the reason the caller sees)
    let (span, _) = domain_span(p)?;
    let cell = span.to_degrees() / p.nk as f64;
    let f = p.fins.as_ref().expect("fins");
    let valid = (p.nk + 1..=(3 * p.nk).max(p.nk + 8)).find(|&nk| body_grid3_inner(&Grid3Params { nk, ..p.clone() }).is_ok());
    Err(Error::Grid(format!(
        "the {} azimuthal cells of the domain are {cell:.1} degrees wide, too wide for the fins ({:.3} d thick, {:.2} d exposed span): a cell beside a fin folds ({m}): the gap cells on the two sides of the fin differ in width (the counts are rounded), so the polygon scale puts its two faces at different radii and shears the thin layer{}",
        p.nk,
        f.thickness,
        f.span,
        valid.map_or(", and no nk up to three times as large builds either: check the fins".to_string(), |n| format!("; use nk >= {n} ({:.1} degree cells)", span.to_degrees() / n as f64))
    )))
}

/// [`body_grid3`] without the explanation of a fold.
fn body_grid3_inner(p: &Grid3Params) -> Result<Grid3, Error> {
    if let (Some(bp), None) = (&p.base, &p.fins) {
        return base_grid3(p, bp);
    }
    let (span, sym) = domain_span(p)?;
    let wedge = sym == Symmetry::Wedge;
    let nk_req = if wedge { 1 } else { p.nk };
    if nk_req < 2 && !wedge && p.fin_stack.is_none() {
        return Err(Error::Grid("nk >= 2 is required".to_string()));
    }
    let n_fin = p.fin_cells_across;
    if n_fin == 0 && p.fins.is_some() {
        return Err(Error::InvalidInput("fin_cells_across >= 1 is required".to_string()));
    }
    // the meridian grid the fins stand on: its axial nodes, the host data of `plan_fins`, the first row above the body surface
    let (mut xs, host, j_off) = match &p.base {
        None => {
            let sp = &p.sting;
            if !(sp.nj >= 1 && sp.dr_wall > 0.0 && sp.r_far > 0.0) {
                return Err(Error::Grid("nj >= 1, dr_wall > 0 and r_far > 0 are required".to_string()));
            }
            let host = Host {
                body: &sp.body,
                n_up: sp.n_up,
                n_nose: sp.n_nose,
                n_seg: &sp.n_seg,
                r_far: sp.r_far,
                sj0: radial_fractions(sp.dr_wall, sp.r_far, sp.nj),
                keep_wake: false,
            };
            (axial_nodes(sp)?, host, 0)
        }
        Some(bp) => {
            let host = Host {
                body: &bp.body,
                n_up: bp.n_up,
                n_nose: bp.n_nose,
                n_seg: &bp.n_seg,
                r_far: bp.r_far,
                sj0: radial_fractions(bp.dr_wall, bp.r_far - bp.body.base_radius(), bp.nj_out),
                keep_wake: true,
            };
            (base_axial_nodes(bp)?.0, host, bp.n_sting + bp.n_gap)
        }
    };
    let r_far = host.r_far;
    let plan = match &p.fins {
        Some(f) => Some((plan_fins(p, f, &mut xs, &host)?, f)),
        None => None,
    };
    let root = plan.as_ref().map(|(pl, f)| (pl.r_t, 0.5 * f.thickness));
    let lay = match (p.domain, &p.fins, &p.fin_stack, root) {
        (Domain3::Half { phase }, Some(f), None, _) => Some(layers(f.count, nk_req, phase, n_fin)?),
        (Domain3::Sector, Some(_), None, _) => Some(sector_layers(nk_req, span, n_fin)?),
        (Domain3::Half { phase }, Some(f), Some(st), Some((r, h))) => Some(stacked_layers(Some((f.count, phase)), st, r, h, span, n_fin)?),
        (Domain3::Sector, Some(_), Some(st), Some((r, h))) => Some(stacked_layers(None, st, r, h, span, n_fin)?),
        _ => None,
    };
    // with a stack the cells of the gaps, not `nk`, set the azimuthal count
    let nk = match (&lay, &p.fin_stack) {
        (Some(l), Some(_)) => l.n_f.iter().sum::<usize>() + l.gaps.iter().sum::<usize>(),
        _ => nk_req,
    };
    let ni = xs.len() - 1;
    // the base plane among the nodes (the rebuilt fin segment keeps it)
    let i_end = p.base.as_ref().map(|bp| {
        let xb = bp.body.x_base();
        xs.partition_point(|&v| v < xb - 1e-9 * (1.0 + xb.abs()))
    });
    let sj = match &plan {
        Some((pl, _)) => pl.sj.clone(),
        None => host.sj0.clone(),
    };
    let nj = j_off + sj.len() - 1;
    let n2 = (ni + 1) * (nj + 1);
    let below = match (&p.base, i_end) {
        (Some(bp), Some(ie)) => base_rows_below(bp, &xs, ie)?,
        _ => Vec::new(),
    };

    // (x, r) of every (i, j) node
    let mut xn = vec![0.0; n2];
    let mut rn = vec![0.0; n2];
    let r_in: Vec<f64> = xs.iter().map(|&x| host.body.r(x)).collect();
    for j in 0..=nj {
        for i in 0..=ni {
            xn[j * (ni + 1) + i] = xs[i];
            rn[j * (ni + 1) + i] = if j < j_off { below[j * (ni + 1) + i] } else { r_in[i] + sj[j - j_off] * (r_far - r_in[i]) };
        }
    }
    // per-row planform (frac along the span, LE position, chord) and the half-angle of the fin layer at each column
    let mut alpha = Vec::new();
    // half-thickness of the fin layer at every node (`NaN` on the axis and below the body surface)
    let mut half_w = Vec::new();
    if let Some((pl, f)) = &plan {
        // planform of the row `jj` above the surface
        let row = |jj: usize| -> (f64, f64, f64) {
            let frac = pl.span_fraction(jj);
            match &pl.planform {
                Some(pf) => {
                    let h = frac * pl.span;
                    (frac, pl.x_le_root + pf.x_le(h), pf.chord(h))
                }
                None => (frac, pl.x_le_root + f.sweep * frac, f.root_chord + (f.tip_chord - f.root_chord) * frac),
            }
        };
        let mut up = 0.0f64;
        let row_offs: Vec<Vec<f64>> = if pl.round_rows { (0..=pl.j_tip).map(|jj| pl.row_offsets(row(jj).2)).collect() } else { Vec::new() };
        for jj in 0..=pl.j_tip {
            up = up.max((row(jj).1 - pl.x_le_root).abs());
        }
        // the zone is only widened where the segment's own would stretch its cells by more than `ZONE_TRIGGER` (the move over the room it has, or over the width of the one cell
        // that absorbs it when it has none): a fin with room keeps the grid it always had. Only the leading edge's: behind the trailing edge the one cell takes the move of the
        // taper as it always did (the example rocket's trailing edge moves 0.13 d over a 0.015 d cell, and its table is as it was)
        let room_up = if pl.i_seg0 < pl.i_le { pl.x_le_root - xs[pl.i_seg0] } else { xs[pl.i_le] - xs[pl.i_le.saturating_sub(1)] };
        let widened = up > ZONE_TRIGGER * room_up;
        // the move fades away over the rows above the tip: four, or as many as lift the row `FADE_REACH` times the move above the tip (a row a move
        // higher is a shear of 45 degrees; a fin with room ahead of its edge, Byrum's, sheared its rows above the tip within four cells, 10 degrees between the cell edges), and the rows of the columns next to the fin, bent to meet its tip, fold under a steeper one: the cells at x = 30.2 d of the Rabia grid
        // did at 75 degrees)
        let height = |jj: usize| (pl.sj[jj] - pl.sj[pl.j_tip]) * (r_far - pl.r_t);
        let nfade = if widened || (height((pl.j_tip + 4).min(nj - j_off)) < FADE_TRIGGER * up) {
            (4..=nj - j_off - pl.j_tip).find(|&n| height(pl.j_tip + n) >= FADE_REACH * up).unwrap_or(nj - j_off - pl.j_tip)
        } else {
            4
        };
        let vj_of = |jj: usize| if jj <= pl.j_tip { 1.0 } else { 1.0 - (jj - pl.j_tip) as f64 / nfade as f64 };
        // the tip row stands on one cylinder `r_tip` over the fin: the fraction of its way from the body surface under each column to the boundary
        let target_ref = pl.span / (r_far - pl.r_t);
        let tcol: Vec<f64> = r_in.iter().map(|&r| ((pl.r_tip - r) / (r_far - r)).max(0.2 * target_ref)).collect();
        // How long the fade of the rows' radial bending beside an edge must be: the bend of row `jj` (`bend`) is a slope along the row, the slope of the edge the rows are
        // sheared along is `dr / shift` (row height over the move of the edge from one row to the next), and a cell folds where the first reaches the second (the cells of the
        // swept Cancer Descending fin at 63 % span, 60 degrees: 0.45 against 0.53, negative volume). Fading over `bend * shift / dr / RAMP_SLOPE` keeps the slope of the rows at
        // `RAMP_SLOPE` of the edge's. `x_edge(jj)` is where the edge of row `jj` ends up (the leading edge, or the trailing edge) and `i_ref` the column the bend is that of.
        let ramp_need = |i_ref: usize, x_edge: &dyn Fn(usize) -> f64| -> f64 {
            let scale = r_far - r_in[i_ref];
            let mut need = 0.0f64;
            for jj in 0..(pl.j_tip + nfade).min(nj - j_off) {
                let bend = (pl.remap(jj, tcol[i_ref]) - sj[jj]).abs() * scale;
                let dr = (pl.remap(jj + 1, tcol[i_ref]) - pl.remap(jj, tcol[i_ref])) * scale;
                let shift = (x_edge(jj + 1) - x_edge(jj)).abs();
                if dr > 0.0 {
                    need = need.max(bend * shift / dr / RAMP_SLOPE);
                }
            }
            need
        };
        // The same for a trailing edge that overhangs the root's (DESIGN3D item 73). A tip trailing edge aft of the root's leans aft below the tip and forward above it (the move
        // fades over the rows above), and the rows beside the edge, bent to meet the tip's cylinder, rise (below the tip, squeezed toward the body) or fall (above it) aft to the
        // nominal radius: the edges of the cell between such a row and the column are then nearly antiparallel however `RAMP_SLOPE` limits the slope (5.6 degrees at the first row
        // above the tip of the Cancer Descending fin, 0.42 d of overhang: the fade of 4 + 4 columns, 0.04 d long, drops the row by 0.11 d). A row of slope `S` beside a column that leans
        // by `L = shift / dr` makes the angle `asin((1 - S L) / sqrt((1 + S^2)(1 + L^2)))`; the fade is long enough that it stays above `OVERHANG_EDGE_ANGLE_DEG`. Only the cells
        // whose row and column lean the one way count, and only on a fin whose trailing edge is aft of the root's somewhere along the span.
        let overhang_need = |i_ref: usize, x_edge: &dyn Fn(usize) -> f64| -> f64 {
            // only a trailing edge that is aft of the root's somewhere along the span: the grids of every other fin (the example rocket, the gate rockets) are as they were
            if !(0..=pl.j_tip).any(|jj| x_edge(jj) > pl.x_te_root + 1e-9) {
                return 0.0;
            }
            let scale = r_far - r_in[i_ref];
            let sin_min = OVERHANG_EDGE_ANGLE_DEG.to_radians().sin();
            let bend = |jj: usize| (pl.remap(jj, tcol[i_ref]) - sj[jj]) * scale;
            let mut need = 0.0f64;
            for jj in 0..(pl.j_tip + nfade).min(nj - j_off) {
                let dr = (pl.remap(jj + 1, tcol[i_ref]) - pl.remap(jj, tcol[i_ref])) * scale;
                let shift = x_edge(jj + 1) - x_edge(jj);
                if dr <= 0.0 || shift == 0.0 {
                    continue;
                }
                let lean = shift.abs() / dr;
                // the largest row slope that keeps the angle: (1 - S L) / sqrt((1 + S^2)(1 + L^2)) falls with S
                let angle_ok = |s: f64| (1.0 - s * lean) / ((1.0 + s * s) * (1.0 + lean * lean)).sqrt() >= sin_min;
                let s_max = if lean >= 1.0 || !angle_ok(0.0) {
                    OVERHANG_MIN_SLOPE
                } else {
                    let (mut lo, mut hi) = (0.0f64, 1.0 / lean);
                    for _ in 0..60 {
                        let m = 0.5 * (lo + hi);
                        if angle_ok(m) {
                            lo = m;
                        } else {
                            hi = m;
                        }
                    }
                    lo.max(OVERHANG_MIN_SLOPE)
                };
                // the columns behind the edge are the segment's nodes between it and the base, remapped linearly: a row whose edge moved aft by `m` has them `1 - m / room` as far apart
                let squeeze = if pl.i_te == pl.i_seg1 { 1.0 } else { ((xs[pl.i_seg1] - x_edge(jj)) / (xs[pl.i_seg1] - pl.x_te_root)).max(0.05) };
                for b in [bend(jj), bend(jj + 1)] {
                    // `b` rises the row going aft when it is negative (the fin side squeezes the rows toward the body), falls when positive; the column leans aft when `shift` is
                    // positive: antiparallel edges are a falling row beside a forward lean, or a rising row beside an aft lean
                    if b * shift < 0.0 {
                        need = need.max(b.abs() / (s_max * squeeze));
                    }
                }
            }
            need
        };
        let x_le_eff = |jj: usize| pl.x_le_root + vj_of(jj) * (row(jj).1 - pl.x_le_root);
        let x_te_eff = |jj: usize| pl.x_te_root + vj_of(jj) * (row(jj).1 + row(jj).2 - pl.x_te_root);
        let (need_le, need_te) = (ramp_need(pl.i_le, &x_le_eff), ramp_need(pl.i_te, &x_te_eff).max(overhang_need(pl.i_te, &x_te_eff)));
        // columns beside the edges: the rows follow the tip's cylinder at full weight over the 4 columns next to the edge and fade away over the next 4; when the sweep zone is
        // widened ([`ZONE_REACH`]) the sweep shears the rows of those columns by up to the move of the edge, and a fade over cells that short bends them (a slope of 0.44 against
        // the shear's 0.42 at Mach 2.3 of the Rabia grid) into a fold: the fade is `RAMP_REACH` times the move longer. Where those columns do not give the length `ramp_need`
        // asks for, the fade is that long in x instead (the grids that had enough keep their columns bit for bit)
        let wcol = |i: usize| -> f64 {
            let columns = |d: usize| if d <= 4 { 1.0 } else { (1.0 - (d as f64 - 4.0) / 4.0).max(0.0) };
            if i < pl.i_le {
                let dx = |n: usize| xs[pl.i_le] - xs[pl.i_le.saturating_sub(n)];
                let x8 = if widened { dx(8).max(dx(4) + RAMP_REACH * up) } else { dx(8) };
                if !widened && need_le <= x8 - dx(4) {
                    return columns(pl.i_le - i);
                }
                let (x4, x8) = (dx(4), x8.max(dx(4) + need_le));
                return if xs[pl.i_le] - xs[i] <= x4 { 1.0 } else { (1.0 - (xs[pl.i_le] - xs[i] - x4) / (x8 - x4)).max(0.0) };
            }
            let ex = |n: usize| xs[(pl.i_te + n).min(ni)] - xs[pl.i_te];
            if i > pl.i_te && need_te > ex(8) - ex(4) {
                let (x4, x8) = (ex(4), ex(4) + need_te);
                return if xs[i] - xs[pl.i_te] <= x4 { 1.0 } else { (1.0 - (xs[i] - xs[pl.i_te] - x4) / (x8 - x4)).max(0.0) };
            }
            columns(i.saturating_sub(pl.i_te))
        };
        let exposed = (pl.i_le..=pl.i_te).map(|i| pl.r_tip - r_in[i]);
        // the tip stands `span` above the largest root radius, so the exposed span is at least `span` there
        let (h_min, h_max) = (exposed.clone().fold(f64::INFINITY, f64::min), exposed.fold(0.0, f64::max));
        if h_max > 3.0 * h_min {
            return Err(Error::Unsupported(format!(
                "the exposed span of the fin varies from {h_min:.3} d to {h_max:.3} d along its root (a steep boattail or flare under the fin): the grid rows cannot follow a fin whose exposed span changes by more than a factor 3"
            )));
        }
        for j in j_off..=nj {
            for i in 0..=ni {
                let s = sj[j - j_off] + wcol(i) * (pl.remap(j - j_off, tcol[i]) - sj[j - j_off]);
                rn[j * (ni + 1) + i] = r_in[i] + s * (r_far - r_in[i]);
            }
        }
        // sweep and taper: piecewise-linear remap of the segment nodes. The nodes ahead of the leading edge absorb what the edge moves by (the leading edge of the tip row is
        // `sweep` aft of the root's): over a zone [`ZONE_REACH`] times the largest move wide, so that no cell takes it alone. The zone starts at the segment's start node and,
        // when the segment has less room than that, goes on past it to the end of the nose: a fin that starts at the start of its segment (a fin can) stretched the one cell
        // ahead of it by the whole sweep (2.6 d for 3.1 d of sweep, 2 400 times the volume of the cell behind it)
        let nodes0: Vec<f64> = xs.clone();
        let i_up =
            if widened { (pl.i_floor..=pl.i_seg0).rev().find(|&i| pl.x_le_root - nodes0[i] >= ZONE_REACH * up).unwrap_or(pl.i_floor) } else { pl.i_seg0 };
        for jj in 0..(pl.j_tip + nfade).min(nj - j_off + 1) {
            let j = j_off + jj;
            let vj = vj_of(jj);
            let (_, x_le, chord) = row(jj);
            let mut knots = Vec::new();
            if i_up < pl.i_le {
                knots.push((i_up, nodes0[i_up], nodes0[i_up]));
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
            if pl.round_rows && chord > 1e-12 {
                // a rounded section keeps the first chord cell of the root down the span ([`FinPlan::row_offsets`])
                for i in pl.i_le..=pl.i_te {
                    let old = nodes0[i];
                    xn[j * (ni + 1) + i] = old + vj * (x_le + row_offs[jj.min(pl.j_tip)][i - pl.i_le] - old);
                }
            }
        }
        // nominal angular width of a layer cell away from the fin: the cells next to it (one uniform cell of the domain, the first cell of a stack)
        let cell_ang = lay.as_ref().map_or(span / nk as f64, |l| l.cell_ang);
        // layer half-angle at every (i, j) node
        alpha = vec![0.0; n2];
        half_w = vec![f64::NAN; n2];
        let t = f.thickness;
        let h_at = |i: usize, jj: usize| -> f64 {
            let (_, _, chord) = row(jj);
            let xi = if pl.round_rows && chord > 1e-12 {
                row_offs[jj.min(pl.j_tip)][i.clamp(pl.i_le, pl.i_te) - pl.i_le] / chord
            } else {
                (nodes0[i] - pl.x_le_root) / f.root_chord
            };
            half_thickness(&f.profile, t, chord, xi)
        };
        for j in j_off..=nj {
            let jj = j - j_off;
            for i in 0..=ni {
                let r = rn[j * (ni + 1) + i];
                let di = if i < pl.i_le { pl.i_le - i } else { i.saturating_sub(pl.i_te) };
                let dj = jj.saturating_sub(pl.j_tip);
                let u_i = if i > pl.i_te { di as f64 / TE_TAPER_COLUMNS } else { di as f64 / 4.0 };
                let u = u_i.max(dj as f64 / 4.0);
                let (ic, jc) = (i.clamp(pl.i_le, pl.i_te), jj.min(pl.j_tip));
                let h_f = h_at(ic, jc);
                // a layer of `n_fin` cells across is as wide as that many neighbouring cells away from the fin (a half fin: half of it)
                let nominal = 0.5 * r * cell_ang * n_fin as f64;
                let w = if u == 0.0 { h_f } else { h_f + (nominal - h_f) * smoothstep(u) };
                alpha[j * (ni + 1) + i] = if r < 1e-12 { (0.5 * cell_ang).asin() } else { (w / r).min(0.99).asin() };
                if r >= 1e-12 {
                    half_w[j * (ni + 1) + i] = w;
                }
            }
        }
    }

    let nn = (ni + 1) * (nj + 1) * (nk + 1);
    let (mut x, mut y, mut z) = (vec![0.0; nn], vec![0.0; nn], vec![0.0; nn]);
    let mut theta = vec![0.0; nk + 1];
    // nodes across the fin layers (not the symmetry planes) with the azimuth of their layer and their place between the faces as a fraction of the half-thickness
    // from the mid-plane (-1 and +1 at the two faces, uniform between): the node's lateral offset from the fin mid-plane, `s_k r sin(theta_k - theta_f)`, is
    // that fraction of the fin half-thickness
    let mut sides: Vec<(usize, f64, f64)> = Vec::new();
    if let Some(l) = &lay {
        let nl = l.theta_f.len();
        for m in 0..nl {
            let (hf, hl) = (l.half_first && m == 0, l.half_last && m + 1 == nl);
            let c = l.n_f[m];
            let (f_lo, f_hi) = (if hf { 0.0 } else { -1.0 }, if hl { 0.0 } else { 1.0 });
            for q in 0..=c {
                if (q == 0 && hf) || (q == c && hl) {
                    continue;
                }
                let frac = if q == 0 {
                    f_lo
                } else if q == c {
                    f_hi
                } else {
                    f_lo + (f_hi - f_lo) * q as f64 / c as f64
                };
                sides.push((l.k_f[m] + q, l.theta_f[m], frac));
            }
        }
    }
    // cells of the fin layers, solid inside the planform: they do not set the scale of the lines at their sides
    let mut layer_cell = vec![false; nk];
    if let Some(l) = &lay {
        for (&kf, &c) in l.k_f.iter().zip(&l.n_f) {
            layer_cell[kf..kf + c].fill(true);
        }
    }
    for j in 0..=nj {
        for i in 0..=ni {
            let c = j * (ni + 1) + i;
            // the rows below the body surface (base-wake grid: the solid body block, and behind the base plane the wake rows down to the axis) take the node
            // angles and the scales of the surface row `j_off` at the column: the node lines are planes through the axis there, so no cell is sheared where the
            // layer is wide on the surface (a stack cell folded otherwise, and so did the cells of a wall-resolved grid, whose height is smaller than the
            // lateral shift of the side nodes by the polygon scale), and the layer tapering behind the trailing edge is a thin sector down to the axis, not a
            // width that jumps at the base radius
            let cr = if plan.is_some() && j < j_off { j_off * (ni + 1) + i } else { c };
            // the rows of the body block upstream of the nose tip lie below the axis: clamped onto it as in `revolve`, their cells are dead
            let (xc, r, r_ref) = (xn[c], rn[c].max(0.0), rn[cr].max(0.0));
            let in_plan = plan.as_ref().is_some_and(|(pl, _)| (pl.i_le..=pl.i_te).contains(&i) && cr >= j_off * (ni + 1) && cr / (ni + 1) - j_off <= pl.j_tip);
            let scale_of = |theta: &[f64]| polygon_scale_where(theta, |k| !(in_plan && layer_cell[k]));
            match &lay {
                Some(l) => {
                    fill_theta(&mut theta, l, alpha[cr], span);
                    if !half_w[cr].is_nan() {
                        // the nodes of a line are scaled by `polygon_scale` (the rings keep the circle's area), which moves the fin faces out by that factor:
                        // put every side node at the angle whose scaled offset is the half-thickness (the scale depends on the neighbouring angles: a few passes)
                        for _ in 0..8 {
                            let s = scale_of(&theta);
                            for &(k, th, frac) in &sides {
                                theta[k] = th + frac.signum() * (frac.abs() * half_w[cr] / (r_ref * s[k])).min(0.99).asin();
                            }
                        }
                    }
                }
                None => {
                    for (k, t) in theta.iter_mut().enumerate() {
                        *t = span * k as f64 / nk as f64;
                    }
                }
            }
            let scale = scale_of(&theta);
            for k in 0..=nk {
                let id = (k * (nj + 1) + j) * (ni + 1) + i;
                x[id] = xc;
                y[id] = r * scale[k] * theta[k].sin();
                z[id] = r * scale[k] * theta[k].cos();
            }
        }
    }
    let mut g = Grid3::try_from_nodes(ni, nj, nk, x, y, z).map_err(Error::Grid)?;
    g.sym = sym;
    g.theta_span = span;
    g.roughness = p.roughness;
    let col = |x: f64| xs.partition_point(|&v| v < x - 1e-9 * (1.0 + x.abs())).min(ni);
    let bb = host.body;
    match (&p.base, i_end) {
        (Some(bp), Some(ie)) => {
            // the body is a solid block under row `j_off` from the nose tip to the base plane (its cells upstream of the tip are dead, see `revolve`), split by
            // segment for the force classes, with the sting block behind the base; the axis rows are the line `j_off` upstream and the rows `j = 0` in the wake
            let n_up = bp.n_up;
            g.axis.push(AxisRun { j: j_off, i: 0..n_up });
            if bp.n_sting == 0 {
                g.axis.push(AxisRun { j: 0, i: ie..ni });
            }
            let (rows, ks) = (0..j_off, 0..nk);
            g.mark_solid(SolidBox { i: 0..n_up, j: rows.clone(), k: ks.clone() }, None);
            let mut i0 = n_up;
            for s in 0..=bb.segments.len() {
                let x_end = if s == 0 { bb.body.nose_length() } else { bb.station(s).0 };
                let i1 = col(x_end).max(i0);
                g.mark_solid(SolidBox { i: i0..i1, j: rows.clone(), k: ks.clone() }, Some(Part::Body(s as u16)));
                i0 = i1;
            }
            if i0 != ie {
                return Err(Error::Grid(format!("the body block ends at {i0} but the base plane is node {ie}")));
            }
            if bp.n_sting > 0 {
                g.mark_solid(SolidBox { i: ie..ni, j: 0..bp.n_sting, k: ks }, Some(Part::Sting));
            }
            mark_jet(&mut g, bp, ie, nk);
        }
        _ => {
            g.axis = vec![AxisRun { j: 0, i: 0..host.n_up }];
            // the body is the `j = 0` wall: its runs by body segment (the nose, each aft segment, then the sting extension)
            let mut i0 = host.n_up;
            for s in 0..=bb.segments.len() {
                let x_end = if s == 0 { bb.body.nose_length() } else { bb.station(s).0 };
                let i1 = col(x_end).max(i0);
                g.wall_j0.push((i0..i1, Part::Body(s as u16)));
                i0 = i1;
            }
            g.wall_j0.push((i0..ni, Part::Sting));
        }
    }
    if let (Some((pl, _)), Some(l)) = (&plan, &lay) {
        for (&kf, &c) in l.k_f.iter().zip(&l.n_f) {
            g.mark_solid(SolidBox { i: pl.i_le..pl.i_te, j: j_off..j_off + pl.j_tip, k: kf..kf + c }, Some(Part::Fin(0)));
        }
    }
    for k in 0..nk {
        for j in 0..nj {
            for i in 0..ni {
                if !g.is_solid(i, j, k) && g.vol[g.cell(i, j, k)] <= 0.0 && (p.base.is_some() || i >= host.n_up) {
                    return Err(Error::Grid(format!("non-positive fluid cell volume at ({i},{j},{k})")));
                }
            }
        }
    }
    Ok(g)
}

/// A single-cell wedge of the angle `dtheta` on a 2D grid whose `j = 0` boundary is the body wall behind `n_axis` columns that lie on the axis (the validation
/// cases' grids: the cone and the ogive, `bowshock_cfd::grid::cone_grid` and the ogive grid): area-preserving nodes, the axis run, the wall a body part.
pub fn wedge_of_meridian(g2: &bowshock_cfd::grid::Grid, dtheta: f64, n_axis: usize) -> Result<Grid3, Error> {
    let mut g = revolve(g2, 1, &[0.0, dtheta]).map_err(Error::Grid)?;
    g.axis = vec![AxisRun { j: 0, i: 0..n_axis }];
    g.sym = Symmetry::Wedge;
    g.theta_span = dtheta;
    g.wall_j0.push((n_axis..g.ni, Part::Body(0)));
    Ok(g)
}

/// Motor-on grids: the nozzle rows of the 2D base grid (`NozzleRows`, rows `0..n_jet` behind the base, none of them a wall) as a face patch on the base. The two
/// cell columns of the body block just ahead of the base plane (`i_end - 2..i_end`, the ghost cells the 2D solver fills with the jet state) in those rows, every
/// `k`, become `Part::Jet`; the `+i` faces between them and the fluid behind the base are the exit (`Wall::Jet`), the rest of the base stays `Wall::Base`.
fn mark_jet(g: &mut Grid3, bp: &BaseGridParams, i_end: usize, nk: usize) {
    if let Some(nz) = &bp.nozzle {
        g.mark_solid(SolidBox { i: i_end - 2..i_end, j: 0..nz.n_jet, k: 0..nk }, Some(Part::Jet));
    }
}

/// Build a finless grid on the base-wake meridian grid `bp` (`base_domain_grid`): the body block (the cells inside the body, from the nose tip to the
/// base plane) is solid, split by body segment for the force classes, its cells upstream of the nose tip are dead (their nodes lie on or behind the axis,
/// see `revolve`), a sting behind the base is its own part. Axis runs: the line `j = j_end` upstream of the nose and, without a sting, the rows
/// `j = 0` behind the base.
fn base_grid3(p: &Grid3Params, bp: &BaseGridParams) -> Result<Grid3, Error> {
    let (span, sym) = domain_span(p)?;
    let nk = if sym == Symmetry::Wedge { 1 } else { p.nk };
    if nk < 2 && sym != Symmetry::Wedge {
        return Err(Error::Grid("nk >= 2 is required".to_string()));
    }
    let (g2, blk) = base_domain_grid(bp)?;
    let theta: Vec<f64> = (0..=nk).map(|k| span * k as f64 / nk as f64).collect();
    let mut g = revolve(&g2, nk, &theta).map_err(Error::Grid)?;
    let (ni, n_up) = (g.ni, bp.n_up);
    g.axis.push(AxisRun { j: blk.j_end, i: 0..n_up });
    if blk.j_sting == 0 {
        g.axis.push(AxisRun { j: 0, i: blk.i_end..ni });
    }
    g.sym = sym;
    g.theta_span = span;
    g.roughness = p.roughness;
    let (rows, ks) = (0..blk.j_end, 0..nk);
    g.mark_solid(SolidBox { i: 0..n_up, j: rows.clone(), k: ks.clone() }, None);
    let mut i0 = n_up;
    for (s, n) in std::iter::once(bp.n_nose).chain(bp.n_seg.iter().copied()).enumerate() {
        g.mark_solid(SolidBox { i: i0..i0 + n, j: rows.clone(), k: ks.clone() }, Some(Part::Body(s as u16)));
        i0 += n;
    }
    if i0 != blk.i_end {
        return Err(Error::Grid(format!("the body block ends at {} but its segments at {i0}", blk.i_end)));
    }
    if blk.j_sting > 0 {
        g.mark_solid(SolidBox { i: blk.i_end..ni, j: 0..blk.j_sting, k: ks }, Some(Part::Sting));
    }
    mark_jet(&mut g, bp, blk.i_end, nk);
    for k in 0..nk {
        for j in 0..g.nj {
            for i in 0..ni {
                if !g.is_solid(i, j, k) && g.vol[g.cell(i, j, k)] <= 0.0 {
                    return Err(Error::Grid(format!("non-positive fluid cell volume at ({i},{j},{k})")));
                }
            }
        }
    }
    Ok(g)
}

/// The azimuthal stack on the + side of the first fin of a grid, measured at the root row (`j = 0`) of the mid-chord column: for the wall-resolved checks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StackReport {
    /// Height of the first cell next to the fin side, normal to the fin \[d\].
    pub y1: f64,
    /// Largest ratio of one cell's height to the previous one's, moving away from the fin.
    pub growth_max: f64,
    /// Cells from the fin to the next fin or symmetry plane.
    pub cells: usize,
}

/// [`StackReport`] of the first fin layer of `g` (at its root row, the body surface); `None` without fins.
pub fn fin_stack_report(g: &Grid3) -> Option<StackReport> {
    let b = g.solid.iter().find(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Fin(_))))?;
    let (i, j) = ((b.i.start + b.i.end) / 2, b.j.start);
    let node = |k: usize| {
        let id = (k * (g.nj + 1) + j) * (g.ni + 1) + i;
        [g.y[id], g.z[id]]
    };
    // the fin's mid-plane azimuth: a half fin on the `theta = 0` plane or the end plane, else the middle of its two sides
    let th = if b.k.start == 0 {
        0.0
    } else if b.k.end == g.nk {
        let q = node(g.nk);
        q[0].atan2(q[1])
    } else {
        let (a, c) = (node(b.k.start), node(b.k.end));
        0.5 * (a[0].atan2(a[1]) + c[0].atan2(c[1]))
    };
    // distance of node `k` from the mid-plane, so the height of cell `k` normal to the fin is the difference of its two nodes' distances
    let off = |k: usize| {
        let q = node(k);
        q[0] * th.cos() - q[1] * th.sin()
    };
    let height = |k: usize| (off(k + 1) - off(k)).abs();
    let ks: Vec<usize> = (b.k.end..g.nk).take_while(|&k| !g.is_solid(i, j, k)).collect();
    let first = *ks.first()?;
    let growth_max = ks.windows(2).map(|w| height(w[1]) / height(w[0])).fold(1.0, f64::max);
    Some(StackReport { y1: height(first), growth_max, cells: ks.len() })
}

#[doc(hidden)]
fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn len3(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Diagnostic: prints the quality numbers of the fluid cells of `g`: volume, edge lengths, aspect ratio, the angles between the mean edge directions, the volume ratio to the neighbours.
pub fn print_quality(label: &str, g: &Grid3) {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let node = |i: usize, j: usize, k: usize| {
        let n = (k * (nj + 1) + j) * (ni + 1) + i;
        [g.x[n], g.y[n], g.z[n]]
    };
    let at = |i: usize, j: usize, k: usize| {
        let c = g.xc[g.cell(i, j, k)];
        format!("({i},{j},{k}) x {:.4} r {:.4} th {:.1}", c[0], c[1].hypot(c[2]), c[2].atan2(c[1]).to_degrees())
    };
    // (value, i, j, k)
    let mut vol_min = (f64::INFINITY, 0, 0, 0);
    let mut aspect = (0.0f64, 0, 0, 0);
    let mut skew = (90.0f64, 0, 0, 0);
    let mut skew_pair = (0usize, 0usize);
    let mut ratio = [(0.0f64, 0, 0, 0); 3];
    let mut fluid = 0usize;
    let mut aspect_hist = [0usize; 5];
    let mut vol_sum = 0.0;
    let dims = |i: usize, j: usize, k: usize| {
        let m = |a: [f64; 3], b: [f64; 3]| len3(sub3(a, b));
        [m(node(i + 1, j, k), node(i, j, k)), m(node(i, j + 1, k), node(i, j, k)), m(node(i, j, k + 1), node(i, j, k))]
    };
    for k in 0..nk {
        for j in 0..nj {
            for i in 0..ni {
                let c = g.cell(i, j, k);
                if g.part[c] != bowshock_cfd::grid3::PART_FLUID {
                    continue;
                }
                fluid += 1;
                let v = g.vol[c];
                vol_sum += v;
                if v < vol_min.0 {
                    vol_min = (v, i, j, k);
                }
                let mut e = [[0.0; 3]; 3];
                for a in 0..2 {
                    for b in 0..2 {
                        let d = [
                            sub3(node(i + 1, j + a, k + b), node(i, j + a, k + b)),
                            sub3(node(i + b, j + 1, k + a), node(i + b, j, k + a)),
                            sub3(node(i + a, j + b, k + 1), node(i + a, j + b, k)),
                        ];
                        for (m, dm) in d.iter().enumerate() {
                            for q in 0..3 {
                                e[m][q] += 0.25 * dm[q];
                            }
                        }
                    }
                }
                let l = [len3(e[0]), len3(e[1]), len3(e[2])];
                // the cells on the axis are degenerate by construction: the aspect and the ratios are of the others
                if g.is_axis(i, j) {
                    continue;
                }
                let asp = l.iter().cloned().fold(0.0, f64::max) / l.iter().cloned().fold(f64::INFINITY, f64::min);
                if asp > aspect.0 {
                    aspect = (asp, i, j, k);
                }
                let bin = if asp < 10.0 {
                    0
                } else if asp < 100.0 {
                    1
                } else if asp < 1000.0 {
                    2
                } else if asp < 10000.0 {
                    3
                } else {
                    4
                };
                aspect_hist[bin] += 1;
                for (m, n) in [(0, 1), (0, 2), (1, 2)] {
                    let cs = (e[m][0] * e[n][0] + e[m][1] * e[n][1] + e[m][2] * e[n][2]) / (l[m] * l[n]);
                    // angle between the two edge directions as lines: 90 is orthogonal
                    let ang = cs.abs().min(1.0).acos().to_degrees();
                    if ang < skew.0 {
                        skew = (ang, i, j, k);
                        skew_pair = (m, n);
                    }
                }
                for (d, (di, dj, dk)) in [(1, 0, 0), (0, 1, 0), (0, 0, 1)].into_iter().enumerate() {
                    let (i2, j2, k2) = (i + di, j + dj, k + dk);
                    if i2 < ni && j2 < nj && k2 < nk && g.part[g.cell(i2, j2, k2)] == bowshock_cfd::grid3::PART_FLUID {
                        let (a, b) = (g.vol[c], g.vol[g.cell(i2, j2, k2)]);
                        let r = (a / b).max(b / a);
                        if r.is_finite() && r > ratio[d].0 {
                            ratio[d] = (r, i, j, k);
                        }
                    }
                }
            }
        }
    }
    println!(
        "GRID {label}: {ni} x {nj} x {nk} = {} cells, {fluid} fluid, fluid volume {vol_sum:.4e} (aspect, angles and ratios without the axis cells)",
        ni * nj * nk
    );
    println!("GRID {label}: min volume {:.3e} at {}", vol_min.0, at(vol_min.1, vol_min.2, vol_min.3));
    println!(
        "GRID {label}: max aspect {:.1} at {}; cells with aspect <10 / <100 / <1e3 / <1e4 / more: {aspect_hist:?}",
        aspect.0,
        at(aspect.1, aspect.2, aspect.3)
    );
    println!(
        "GRID {label}: min angle between cell edges {:.2} deg at {} (between the {} and {} edges)",
        skew.0,
        at(skew.1, skew.2, skew.3),
        ["i", "j", "k"][skew_pair.0],
        ["i", "j", "k"][skew_pair.1]
    );
    for (d, n) in ["i", "j", "k"].iter().enumerate() {
        let (r, i, j, k) = ratio[d];
        if r == 0.0 {
            continue;
        }
        let nb = [(1, 0, 0), (0, 1, 0), (0, 0, 1)][d];
        println!(
            "GRID {label}: max volume ratio of neighbours in {n} {:.2} at {} (edges {:?} against {:?})",
            ratio[d].0,
            at(i, j, k),
            dims(i, j, k),
            dims(i + nb.0, j + nb.1, k + nb.2)
        );
    }
}

/// Diagnostic: the cells beside the leading edge of the fin walls, row by row (`j`): the first three chord cells' axial lengths on the fin face line, the radial row height,
/// how far the edge moves aft from one row to the next (the sweep), the angle between the i and j edges in the meridian plane and the cell's aspect.
pub fn print_le_quality(label: &str, g: &Grid3) {
    let node = |i: usize, j: usize, k: usize| {
        let n = (k * (g.nj + 1) + j) * (g.ni + 1) + i;
        [g.x[n], g.y[n], g.z[n]]
    };
    let is_fin = |i: usize, j: usize, k: usize| matches!(g.part_of(i, j, k), Some(Part::Fin(_)));
    // a fin layer cell and its first chord column
    let Some((_, _, k0)) = (0..g.nk).flat_map(|k| (0..g.nj).flat_map(move |j| (0..g.ni).map(move |i| (i, j, k)))).find(|&(i, j, k)| is_fin(i, j, k)) else {
        return;
    };
    let k_side = if k0 > 0 { k0 - 1 } else { k0 };
    for j in 0..g.nj {
        let Some(i_le) = (0..g.ni).find(|&i| is_fin(i, j, k0)) else { continue };
        let r = |i: usize, j: usize| node(i, j, k_side)[1].hypot(node(i, j, k_side)[2]);
        let dx = |i: usize| node(i + 1, j, k_side)[0] - node(i, j, k_side)[0];
        let dr = r(i_le, j + 1) - r(i_le, j);
        let shift = node(i_le, j + 1, k_side)[0] - node(i_le, j, k_side)[0];
        let ang = dr.atan2(shift.abs()).to_degrees();
        let along = dr.hypot(shift);
        println!(
            "LE {label}: j {j} i_le {i_le} x {:.4} r {:.4} cells dx {:.5} {:.5} {:.5} (ahead {:.5}) dr {dr:.5} shift {shift:.5} edge angle {ang:.1} deg aspect {:.1}",
            node(i_le, j, k_side)[0],
            r(i_le, j),
            dx(i_le),
            dx(i_le + 1),
            dx(i_le + 2),
            dx(i_le - 1),
            along / dx(i_le).abs().max(1e-300)
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::body_grid::{BaseBody, Body, Nose, Segment};
    use crate::engine::case::Case;
    use crate::engine::meshing::Wake;
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
        Fins3 { count: 4, root_chord: 1.0, tip_chord: 1.0, span: 1.0, sweep: 0.0, thickness: 0.08, te_to_base: 0.0, profile, planform: None }
    }

    fn params(level: u32, fins: Option<Fins3>) -> Grid3Params {
        Grid3Params::half(anf_sting(level), fins, Grid3Params::default_nk(level), level)
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
        let f = Fins3 {
            count: 4,
            root_chord: 1.0,
            tip_chord: 0.6,
            span: 1.0,
            sweep: 0.3,
            thickness: 0.08,
            te_to_base: 0.0,
            profile: FinProfile::Square,
            planform: None,
        };
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
            let f = Fins3 {
                count: 4,
                root_chord: 1.0,
                tip_chord: 0.6,
                span: 1.0,
                sweep: 0.3,
                thickness: 0.08,
                te_to_base: 0.0,
                profile: profile.clone(),
                planform: None,
            };
            let g = body_grid3(&params(0, Some(f.clone()))).unwrap();
            for (m, b) in g.solid.iter().enumerate() {
                let th = PI / 4.0 + PI / 2.0 * m as f64;
                for j in 0..=b.j.end {
                    for i in b.i.start..=b.i.end {
                        let p = node(&g, i, j, b.k.start);
                        // the nodes of a `k` line are scaled out by `polygon_scale`: the nominal radius is the node's over that scale
                        let theta: Vec<f64> = (0..=g.nk).map(|k| node(&g, i, j, k)).map(|q| q[1].atan2(q[2])).collect();
                        let r = (p[1] * p[1] + p[2] * p[2]).sqrt() / polygon_scale_where(&theta, |k| !g.solid.iter().any(|s| s.k.contains(&k)))[b.k.start];
                        let frac = (r - R) / f.span;
                        let x_le = f.sweep * frac;
                        let x_le0 = g.x[b.i.start]; // wall row (k = 0, j = 0) at the root leading edge
                        let chord = f.root_chord + (f.tip_chord - f.root_chord) * frac;
                        let xi = (p[0] - (x_le0 + x_le)) / chord;
                        let h = half_thickness(&profile, f.thickness, chord, xi);
                        let (lo, hi) = (dist(&g, i, j, b.k.start, th), dist(&g, i, j, b.k.end, th));
                        assert!((lo + h).abs() < 1e-9 && (hi - h).abs() < 1e-9, "{profile:?} ({i},{j}): {lo} {hi} vs {h}");
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
        // area-preserving rings: the volume is the revolved 2D grid's
        assert!((v3 / (PI * v2) - 1.0).abs() < 1e-10, "{v3} vs {}", PI * v2);
        assert_eq!(g.axis, vec![AxisRun { j: 0, i: 0..p.sting.n_up }]);
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

    /// A fin that starts where its segment starts (the fin can of a rocket) and is swept: the sweep is spread over the cells ahead of the leading edge instead of stretching the one cell next to the edge, which was 2 400 times the volume of the fin cell behind it for the delta fin of the
    /// Rabia fixture (DESIGN3D item 57).
    #[test]
    fn the_sweep_of_a_fin_at_the_start_of_its_segment_is_spread_over_many_cells() {
        let mut p = params(
            0,
            Some(Fins3 {
                count: 4,
                root_chord: 3.0,
                tip_chord: 0.0,
                span: 1.0,
                sweep: 2.4,
                thickness: 0.08,
                te_to_base: 0.0,
                profile: FinProfile::Rounded,
                planform: None,
            }),
        );
        let nose = Nose::Cone { half_angle: 10f64.to_radians() };
        p.sting.body =
            BaseBody { body: Body { nose }, segments: vec![Segment { length: 6.0, slope: 0.0 }, Segment { length: 3.0, slope: 0.0 }], sting_radius: 0.0 };
        p.sting.n_seg = vec![24, 16];
        let g = body_grid3(&p).unwrap();
        let j_tip = g.solid[0].j.end;
        let first = p.sting.n_up + p.sting.n_nose;
        let mut worst = (1.0f64, 0, 0);
        // the rows up to half the span: near a pointed tip the fin piece itself shrinks to nothing
        for j in 1..j_tip / 2 {
            let xs: Vec<f64> = (0..=g.ni).map(|i| g.x[j * (g.ni + 1) + i]).collect();
            // up to the trailing edge: behind it the one cell takes the move of the taper, as it always did
            let (q, i) = max_ratio(&xs[..=g.solid[0].i.end], first);
            if q > worst.0 {
                worst = (q, i, j);
            }
        }
        eprintln!("sweep at a segment start: widest neighbour ratio {:.2} at i {} row {}", worst.0, worst.1, worst.2);
        // the cell ahead of the edge is the segment's own, 3 times the finer edge cell of the round section (more in the rows of a tapering chord): it was 2 400
        assert!(worst.0 < 6.0, "neighbouring cell widths differ by {:.1} at i {} of row {}", worst.0, worst.1, worst.2);
    }

    /// The swept rounded fin of the Cancer Descending fixture (60 degrees, tip chord 0.29 of the root's, the tip trailing edge behind the root's, 0.0625 d thick; DESIGN3D item
    /// 66) built with a fold at 63 % span: the rows bent to meet the tip's cylinder faded over 4 columns, a slope that reached the sweep shear's. It builds, and the first chord
    /// cell of the round edge keeps its size down the span.
    #[test]
    fn a_swept_tapered_rounded_fin_does_not_fold_and_keeps_its_edge_cell() {
        for level in [0u32, 1] {
            let mut p = params(
                level,
                Some(Fins3 {
                    count: 3,
                    root_chord: 3.167,
                    tip_chord: 0.917,
                    span: 1.5,
                    sweep: 2.667,
                    thickness: 0.0625,
                    te_to_base: 0.84,
                    profile: FinProfile::Rounded,
                    planform: None,
                }),
            );
            let nose = Nose::Cone { half_angle: 10f64.to_radians() };
            p.sting.body = BaseBody { body: Body { nose }, segments: vec![Segment { length: 14.0, slope: 0.0 }], sting_radius: 0.0 };
            p.sting.n_seg = vec![56 << level];
            p.fin_le_rows = true;
            let g = body_grid3(&p).expect("the grid builds without a negative volume");
            let (le_root, j0, j_tip) = (g.solid[0].i.start, g.solid[0].j.start, g.solid[0].j.len());
            let k_side = if g.solid[0].k.start > 0 { g.solid[0].k.start - 1 } else { g.solid[0].k.end };
            let node = |i: usize, j: usize| g.x[(k_side * (g.nj + 1) + j) * (g.ni + 1) + i];
            // the first chord cell of the row at 60 % of the span against the root's
            let j60 = j0 + 3 * j_tip / 5;
            let cell = |j: usize| node(le_root + 1, j) - node(le_root, j);
            eprintln!("level {level}: first chord cell {:.5} at the root, {:.5} at row {j60} of {j_tip}", cell(j0), cell(j60));
            assert!(cell(j60) > 0.7 * cell(j0), "the edge cell shrank from {:.5} to {:.5}", cell(j0), cell(j60));
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
        // the nodes of a `k` line sit at `polygon_scale` times their nominal radius (`1 + dtheta^2 / 12`, 0.14 % for the 24 cells of 7.5 degrees)
        assert!((tip(&g) / 1.5 - 1.0).abs() < 5e-3 && (tip(&g0) / 1.5 - 1.0).abs() < 5e-3 && (tip(&g) - tip(&g0)).abs() < 1e-9);
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

    fn flow() -> crate::engine::case3::WallFlow {
        // the example rocket at Mach 2: Re/m 9.84e6, d = 0.0572 m
        crate::engine::case3::WallFlow { re_c: 9.84e6 * 0.0572 / 2.0, mach_max: 2.0, ks_body: 2e-5, ks_fin: 1e-4, sutherland: 0.368 }
    }

    #[test]
    fn a_stack_starts_at_the_wall_and_grows_at_the_design_rates() {
        let st = FinStack::from_flow(flow().re_c, 2.0, 1.5, 15.0);
        // the design's numbers for the example fin: first cell 6.1e-4 d, boundary layer 0.036 d
        assert!((st.y1 / 6.1e-4 - 1.0).abs() < 0.1 && (st.delta / 0.036 - 1.0).abs() < 0.1, "{st:?}");
        let (y1, delta) = (st.y1 / 0.5, st.delta / 0.5);
        for (lo, hi) in [(true, false), (false, true), (true, true)] {
            let f = stack_fractions(0.7, lo, hi, &st, y1, delta);
            assert!(f[0] == 0.0 && (f[f.len() - 1] - 1.0).abs() < 1e-15 && f.windows(2).all(|w| w[1] > w[0]));
            let sizes: Vec<f64> = f.windows(2).map(|w| 0.7 * (w[1] - w[0])).collect();
            // the wall end: the first cell, `y1`; away from it the cells grow by at most 1.35 (and by 1.25 inside the layer)
            let from_wall: Vec<f64> = if lo { sizes.clone() } else { sizes.iter().rev().copied().collect() };
            assert!((from_wall[0] / y1 - 1.0).abs() < 1e-9, "first cell {} vs {y1}", from_wall[0]);
            let half = if lo && hi { from_wall.len() / 2 } else { from_wall.len() };
            for w in from_wall[..half].windows(2) {
                assert!(w[1] / w[0] <= 1.35 + 1e-9 && w[1] / w[0] >= 1.0 - 1e-9, "growth {}", w[1] / w[0]);
            }
            let inside = from_wall
                .iter()
                .scan(0.0, |s, h| {
                    *s += h;
                    Some(*s)
                })
                .take_while(|&s| s < delta)
                .count();
            assert!(inside >= 8 && from_wall[..inside.min(half)].windows(2).all(|w| w[1] / w[0] <= 1.25 + 1e-9), "{inside} cells in the layer");
        }
    }

    #[test]
    fn the_fin_stack_sets_the_azimuthal_count_and_keeps_the_fin_thickness() {
        let fl = flow();
        for (sector, phase) in [(true, FinPhase::Between), (false, FinPhase::Between), (false, FinPhase::InPlane)] {
            let mut p = params(0, Some(anf_fins(FinProfile::Square)));
            p.fin_stack = Some(FinStack::from_flow(fl.re_c, fl.mach_max, 1.0, 15.0));
            p.domain = if sector { Domain3::Sector } else { Domain3::Half { phase } };
            let g = body_grid3(&p).unwrap();
            let rep = fin_stack_report(&g).unwrap();
            eprintln!("sector {sector} {phase:?}: nk {} {rep:?}", g.nk);
            // the first cell is the design's, measured on the grid (the nodes sit at 1 + dtheta^2/12 of their radius)
            let y1 = p.fin_stack.unwrap().y1;
            assert!((rep.y1 / y1 - 1.0).abs() < 0.02, "first cell {} vs {y1}", rep.y1);
            assert!(rep.growth_max <= 1.35 + 1e-9, "growth {}", rep.growth_max);
            assert!(g.nk >= 12 && g.nk != p.nk, "nk {} is set by the stack, not by `nk` ({})", g.nk, p.nk);
            // the fin face is at the half-thickness 0.04 from the mid-plane, exactly (the scale of the nodes is taken out)
            let b = &g.solid[0];
            let th = if sector || phase == FinPhase::InPlane { 0.0 } else { PI / 4.0 };
            for (i, j) in [(b.i.start + 3, 0), (b.i.end - 2, b.j.end - 1)] {
                let hi = dist(&g, i, j, b.k.end, th);
                assert!((hi - 0.04).abs() < 1e-9, "face at {hi}");
            }
        }
    }

    #[test]
    fn the_two_sided_gap_between_two_fins_is_symmetric() {
        let fl = flow();
        let mut p = params(0, Some(anf_fins(FinProfile::Square)));
        p.fin_stack = Some(FinStack::from_flow(fl.re_c, fl.mach_max, 1.0, 15.0));
        let g = body_grid3(&p).unwrap();
        // two fins at 45 and 135 degrees: the gap between them (90 degrees) has its stacks meeting at 90
        let (b0, b1) = (&g.solid[0], &g.solid[1]);
        let i = (b0.i.start + b0.i.end) / 2;
        let theta = |k: usize| {
            let id = (k * (g.nj + 1)) * (g.ni + 1) + i;
            g.y[id].atan2(g.z[id])
        };
        let (k0, k1) = (b0.k.end, b1.k.start);
        assert_eq!((k1 - k0) % 2, 0);
        let mid = theta(k0 + (k1 - k0) / 2);
        assert!((mid - PI / 2.0).abs() < 2e-3, "stacks meet at {mid}");
        for m in 0..(k1 - k0) / 2 {
            let (a, c) = (theta(k0 + m + 1) - theta(k0 + m), theta(k1 - m) - theta(k1 - m - 1));
            assert!((a / c - 1.0).abs() < 0.05, "cell {m}: {a} vs {c}");
        }
    }

    /// Grid quality of a wall-resolved sector: everywhere in the flow the azimuthal cells of a gap grow by at most the design's ratio (the same fractions of
    /// the gap at every `(i, j)`), the fin layer cell against its first neighbour is the one place that jumps (the fin is one cell thick: the report says how
    /// much), no cell is folded or flat, and the radial and axial spacings the fins add stay as smooth as without the stack.
    #[test]
    fn the_wall_resolved_sector_is_smooth_in_the_gaps_and_reports_the_fin_jump() {
        let fl = flow();
        let mut p = params(0, Some(anf_fins(FinProfile::DoubleWedge)));
        p.domain = Domain3::Sector;
        p.fin_stack = Some(FinStack::from_flow(fl.re_c, fl.mach_max, 1.0, 15.0));
        let g = body_grid3(&p).unwrap();
        let width = |i: usize, j: usize, k: usize| {
            let (id0, id1) = (((k * (g.nj + 1)) + j) * (g.ni + 1) + i, (((k + 1) * (g.nj + 1)) + j) * (g.ni + 1) + i);
            (g.y[id0] - g.y[id1]).hypot(g.z[id0] - g.z[id1])
        };
        let layer = &g.solid[0].k;
        let (mut gap_growth, mut jump) = (1.0f64, 1.0f64);
        for j in 1..g.nj {
            for i in p.sting.n_up..g.ni {
                for k in 0..g.nk - 1 {
                    let (w0, w1) = (width(i, j, k), width(i, j, k + 1));
                    if w0 < 1e-12 || w1 < 1e-12 {
                        continue;
                    }
                    let q = (w1 / w0).max(w0 / w1);
                    if layer.contains(&k) || layer.contains(&(k + 1)) {
                        jump = jump.max(q);
                    } else {
                        gap_growth = gap_growth.max(q);
                    }
                }
            }
        }
        eprintln!("sector stack: gap growth {gap_growth:.3}, layer cell against its neighbour at most {jump:.1}");
        assert!(gap_growth <= 1.45, "gap growth {gap_growth}");
        // the layer is as wide as the fin (0.08 d) next to a first cell of 6e-4 d at the root: the jump is by design, bounded by their ratio
        assert!(jump < 0.08 * 1.5 / (0.9 * p.fin_stack.unwrap().y1), "layer jump {jump}");
    }

    #[test]
    fn sharp_edges_keep_their_needle_and_blunt_ones_their_floor() {
        let (t, chord) = (0.08, 1.0);
        let r = 0.5 * t;
        // a double wedge: both ends sharp, the floor 2 % of the half thickness, no flat face
        assert!((half_thickness(&FinProfile::DoubleWedge, t, chord, 0.0) - 0.02 * r).abs() < 1e-15);
        assert!((half_thickness(&FinProfile::DoubleWedge, t, chord, 1.0) - 0.02 * r).abs() < 1e-15);
        // the rounded edge: the node at the nose is a point, the next one on the circle
        assert!((half_thickness(&FinProfile::Rounded, t, chord, 0.0) - 0.02 * r).abs() < 1e-15);
        let s = 0.01;
        assert!((half_thickness(&FinProfile::Rounded, t, chord, s / chord) - (r * r - (r - s) * (r - s)).sqrt()).abs() < 1e-15);
        // the ANF section: sharp leading edge, blunt trailing edge of the full thickness
        let anf = FinProfile::Custom { section: vec![[0.0, 0.0], [0.1, 1.0], [1.0, 1.0]] };
        assert!((half_thickness(&anf, t, chord, 0.0) - 0.02 * r).abs() < 1e-15);
        assert!((half_thickness(&anf, t, chord, 1.0) - r).abs() < 1e-15);
        // a square section and a blunt end are not touched
        assert!((half_thickness(&FinProfile::Square, t, chord, 0.0) - r).abs() < 1e-15);
        let blunt = FinProfile::Custom { section: vec![[0.0, 0.1], [0.5, 1.0], [1.0, 0.1]] };
        assert!((half_thickness(&blunt, t, chord, 0.0) - BLUNT_EDGE_FLOOR * r).abs() < 1e-15, "{}", half_thickness(&blunt, t, chord, 0.0));
        // the floor of a blunt edge went from 25 % to 10 % of t/2 (DESIGN3D item 38): no built-in profile and no section with a sharp or a full-thickness end changes (the
        // old rule re-implemented here); only a section whose end is blunt but thinner than t/4 does (the Finner's 0.004 cal leading edge)
        let old = |profile: &FinProfile, xi: f64| {
            let raw = |x: f64| half_thickness(profile, t, chord, x.clamp(0.0, 1.0));
            let sharp_end = raw(if xi < 0.5 { 0.0 } else { 1.0 }) < 0.05 * r;
            half_thickness(profile, t, chord, xi).max(if sharp_end { 0.0 } else { 0.25 * r })
        };
        let hexagon = FinProfile::Hexagonal { wedge_fraction: 0.25 };
        for profile in [FinProfile::Square, FinProfile::Rounded, FinProfile::Airfoil, FinProfile::DoubleWedge, hexagon, FinProfile::Biconvex, anf.clone()] {
            for n in 0..=200 {
                let xi = n as f64 / 200.0;
                assert_eq!(half_thickness(&profile, t, chord, xi), old(&profile, xi), "{profile:?} at {xi}");
            }
        }
        // on the grid the leading-edge face of a double wedge is a sliver: its area against a square fin's
        let area = |profile: FinProfile| {
            let g = body_grid3(&params(0, Some(anf_fins(profile)))).unwrap();
            let b = &g.solid[0];
            (b.j.start..b.j.end).map(|j| g.iface[(b.k.start * g.nj + j) * (g.ni + 1) + b.i.start].area).sum::<f64>()
        };
        let (wedge, square) = (area(FinProfile::DoubleWedge), area(FinProfile::Square));
        assert!(wedge < 0.1 * square, "double wedge LE face {wedge} vs square {square}");
    }

    #[test]
    fn the_roughness_of_the_parts_reaches_the_cells_that_touch_them() {
        let mut p = params(0, Some(anf_fins(FinProfile::Square)));
        p.roughness = PartRoughness { body: 1e-4, fin: 5e-4 };
        let g = body_grid3(&p).unwrap();
        let ks = g.wall_ks();
        let b = &g.solid[0];
        let i = (b.i.start + b.i.end) / 2;
        // beside a fin side, away from the body: the fin's; on the body wall under no fin: the body's; at the root next to the fin: the larger
        assert_eq!(ks[g.cell(i, b.j.end / 2 + 1, b.k.end)], 5e-4);
        assert_eq!(ks[g.cell(p.sting.n_up + 2, 0, 1)], 1e-4);
        assert_eq!(ks[g.cell(i, 0, b.k.end)], 5e-4);
        // far from every wall, on the axis rows and in solid cells: 0
        assert_eq!(ks[g.cell(i, g.nj - 1, 3)], 0.0);
        assert_eq!(ks[g.cell(2, 0, 3)], 0.0, "axis row");
        assert_eq!(ks[g.cell(i, 2, b.k.start)], 0.0, "solid cell");
    }

    #[test]
    fn the_wall_resolved_tiers_size_the_grid_from_the_flow() {
        use crate::engine::case3::{Case3, Tier};
        let fl = flow();
        let body = Case3::anf_body();
        let euler = Case3::sector(2.0, body.clone(), Case3::anf_fins(), Tier::Euler, 0);
        let coarse = Case3::sector(2.0, body.clone(), Case3::anf_fins(), Tier::Coarse(fl), 0);
        let fine = Case3::sector(2.0, body.clone(), Case3::anf_fins(), Tier::Fine(fl), 0);
        assert!(euler.grid.fin_stack.is_none() && euler.grid.roughness == PartRoughness::default());
        let (sc, sf) = (coarse.grid.fin_stack.unwrap(), fine.grid.fin_stack.unwrap());
        // fin y+ 15 against 1: 15 times the first cell; the body wall spacing is the 2D case's for y+ 1 on both
        assert!((sc.y1 / sf.y1 - 15.0).abs() < 1e-9);
        let viscous = Case::viscous(2.0, body.clone(), 0, Wake::Sting, fl.re_c, 0.5, fl.mach_max);
        assert_eq!(coarse.grid.sting.dr_wall, viscous.grid_sting.dr_wall);
        assert_eq!(coarse.grid.sting.nj, viscous.grid_sting.nj);
        assert_eq!(coarse.grid.roughness, PartRoughness { body: 2e-5, fin: 1e-4 });
        let (gc, gf) = (body_grid3(&coarse.grid).unwrap(), body_grid3(&fine.grid).unwrap());
        eprintln!("sector N=4: Euler nk {}, coarse nk {}, fine nk {}; coarse {} x {} x {}", euler.grid.nk, gc.nk, gf.nk, gc.ni, gc.nj, gc.nk);
        // the design's estimate for the example fin: nk 18 (y+ 15) and 30 (y+ 1)
        assert!(gf.nk > gc.nk && (12..=30).contains(&gc.nk) && gf.nk >= 20, "nk {} {}", gc.nk, gf.nk);
        // the lift tier keeps uniform cells on the half domain
        let lift = Case3::base_half(2.0, 0.0, body, None, Tier::Lift(fl), 0);
        assert_eq!(lift.grid.nk, 32);
        assert!(lift.grid.base.as_ref().unwrap().dr_wall > 30.0 * viscous.grid_base.dr_wall);
    }

    /// ANF sector on the base-wake grid (level 0) with `nk` cells and the fins' trailing edge `te_to_base` ahead of the base.
    fn base_fin_grid(profile: FinProfile, te_to_base: f64, nk: usize) -> (Grid3Params, Grid3) {
        use crate::engine::case3::{Case3, Tier};
        let mut f = anf_fins(profile);
        f.te_to_base = te_to_base;
        let mut p = Case3::sector(2.0, Case3::anf_body(), f, Tier::Euler, 0).grid;
        p.nk = nk;
        let g = body_grid3(&p).unwrap();
        (p, g)
    }

    /// `(base plane node, first row above the body)` of a base-wake grid with fins.
    fn base_plane(g: &Grid3) -> (usize, usize) {
        let body: Vec<&SolidBox> = g.solid.iter().filter(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Body(_)))).collect();
        (body.iter().map(|b| b.i.end).max().unwrap(), body.iter().map(|b| b.j.end).max().unwrap())
    }

    #[test]
    fn the_axial_nodes_grow_smoothly_through_both_fin_edges_on_the_base_grid() {
        for nf in [32usize, 48, 64] {
            let mut f = anf_fins(FinProfile::Square);
            f.te_to_base = 0.0;
            let mut p = crate::engine::case3::Case3::sector(2.0, crate::engine::case3::Case3::anf_body(), f, crate::engine::case3::Tier::Euler, 0).grid;
            p.nk = 6;
            p.fin_chord_cells = Some(nf);
            let g = body_grid3(&p).unwrap();
            let (i_end, j_off) = base_plane(&g);
            let fin = g.solid.iter().find(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Fin(0)))).unwrap();
            let x = |i: usize| g.x[(fin.k.start * (g.nj + 1) + j_off) * (g.ni + 1) + i];
            let worst = (fin.i.start.saturating_sub(15)..i_end + 20).map(|i| (x(i + 1) - x(i)) / (x(i) - x(i - 1))).map(|r| r.max(1.0 / r)).fold(0.0, f64::max);
            let all: Vec<String> = (fin.i.start.saturating_sub(15)..i_end + 20).map(|i| format!("{i}:{:.2}", (x(i + 1) - x(i)) / (x(i) - x(i - 1)))).collect();
            assert!(worst <= 1.25, "nf {nf} i_end {i_end}: largest neighbour ratio {worst}; {}", all.join(" "));
        }
    }

    #[test]
    fn flush_fins_on_the_base_grid_end_on_the_base_plane_and_taper_behind_it() {
        let (p, g) = base_fin_grid(FinProfile::Square, 0.0, 6);
        let (i_end, j_off) = base_plane(&g);
        let fin = g.solid.iter().find(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Fin(0)))).unwrap();
        assert_eq!((fin.i.end, fin.j.start, fin.k.len()), (i_end, j_off, 1), "the fin ends on the base plane and stands on the body surface row");
        // the trailing-edge face of the fin and the base face are one plane `x = x_base`, normal +x
        let x_base = p.base.as_ref().unwrap().body.x_base();
        let mut faces = 0;
        for j in (0..j_off).chain(fin.j.clone()) {
            let f = &g.iface[(fin.k.start * g.nj + j) * (g.ni + 1) + i_end];
            if f.area > 1e-14 {
                assert!((f.c[0] - x_base).abs() < 1e-12 && (f.n[0] - 1.0).abs() < 1e-12, "row {j}: x {} normal {:?}", f.c[0], f.n);
                faces += 1;
            }
        }
        assert!(faces >= j_off - 2 + fin.j.len(), "{faces} faces on the plane");
        // behind the plane: the layer is the fin's angle on every wake row below the surface (a sector to the axis) and tapers over the next columns
        let theta = |i: usize, j: usize, k: usize| {
            let id = (k * (g.nj + 1) + j) * (g.ni + 1) + i;
            g.y[id].atan2(g.z[id])
        };
        let width = |i: usize, j: usize| theta(i, j, fin.k.end) - theta(i, j, fin.k.start);
        let mut last = f64::INFINITY;
        for i in i_end..i_end + 6 {
            let w = width(i, j_off);
            assert!(w > 0.0 && w <= last + 1e-12, "column {i}: width {w} after {last}");
            last = w;
            for j in 1..j_off {
                // the surface row's side nodes are placed at the scaled offset (`1 + d^2/12`, here 0.2 %), the rows below at the plain angle
                assert!((width(i, j) / w - 1.0).abs() < 5e-3, "column {i} row {j}: {} vs {w}", width(i, j));
            }
        }
        // the layer at the base plane is the fin's: its full angle
        assert!(width(i_end, j_off) > 0.07, "{}", width(i_end, j_off));
    }

    #[test]
    fn fins_ahead_of_the_boattail_and_half_domains_build_on_the_base_grid() {
        use crate::engine::case3::{Case3, Tier};
        // a tube of 6 d then a 1 d boattail: the fins' trailing edge 1.2 d ahead of the base is on the tube
        let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0).with_boattail(1.0, 5f64.to_radians());
        let mut f = anf_fins(FinProfile::DoubleWedge);
        f.te_to_base = 1.2;
        f.span = 0.6;
        let g = body_grid3(&Case3::sector(2.0, body.clone(), f.clone(), Tier::Euler, 0).grid).unwrap();
        let (i_end, j_off) = base_plane(&g);
        let fin = g.solid.iter().find(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Fin(0)))).unwrap();
        assert!(fin.i.end < i_end && fin.j.start == j_off, "fin {:?} base plane {i_end}", fin);
        for (count, nk) in [(4, 24), (3, 24), (2, 16)] {
            let mut f = anf_fins(FinProfile::Square);
            f.count = count;
            let g = body_grid3(&Case3::base_half(2.0, 0.0, Case3::anf_body(), Some(f), Tier::Euler, 0).grid).unwrap();
            let fins = g.solid.iter().filter(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Fin(0)))).count();
            assert_eq!(fins, (count as usize).div_ceil(2), "N {count}: fin layers in the half domain");
            let _ = nk;
        }
    }

    #[test]
    fn the_fin_stack_grid_on_the_base_grid_is_as_smooth_as_on_the_sting_grid() {
        use crate::engine::case3::{Case3, Tier};
        let fl = flow();
        let sting = {
            let mut p = params(0, Some(anf_fins(FinProfile::DoubleWedge)));
            p.domain = Domain3::Sector;
            p.fin_stack = Some(FinStack::from_flow(fl.re_c, fl.mach_max, 1.0, 15.0));
            body_grid3(&p).unwrap()
        };
        let mut f = anf_fins(FinProfile::DoubleWedge);
        f.count = 4;
        let case = Case3::sector(2.0, Case3::anf_body(), f, Tier::Coarse(fl), 0);
        let g = body_grid3(&case.grid).unwrap();
        let (a, b) = (fin_stack_report(&sting).unwrap(), fin_stack_report(&g).unwrap());
        eprintln!("stack on the sting grid {a:?} (nk {}), on the base grid {b:?} (nk {})", sting.nk, g.nk);
        assert!((b.y1 / a.y1 - 1.0).abs() < 0.01 && b.growth_max <= 1.35 + 1e-9 && b.cells == a.cells, "{a:?} {b:?}");
        assert_eq!(g.nk, sting.nk);
    }

    /// FNV-1a of the node coordinates, the part flags and the solid boxes of a grid.
    fn fingerprint(g: &Grid3) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut eat = |v: u64| {
            for b in v.to_le_bytes() {
                h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
            }
        };
        [g.ni, g.nj, g.nk].iter().for_each(|&n| eat(n as u64));
        g.x.iter().chain(&g.y).chain(&g.z).for_each(|v| eat(v.to_bits()));
        g.part.iter().for_each(|&p| eat(p as u64));
        g.solid.iter().for_each(|b| [b.i.start, b.i.end, b.j.start, b.j.end, b.k.start, b.k.end].iter().for_each(|&n| eat(n as u64)));
        h
    }

    /// The grids of the default `fin_cells_across = 1` (uniform and stacked, half domain and sector, half fins in the planes, the base-wake grid) as the first
    /// (one-cell-layer) mesher built them: the fingerprints were taken before the parameter existed.
    #[test]
    fn one_cell_across_reproduces_the_first_grids_exactly() {
        use crate::engine::case3::{Case3, Tier};
        let fl = flow();
        let stack = FinStack::from_flow(fl.re_c, fl.mach_max, 1.0, 15.0);
        let mut three = anf_fins(FinProfile::Square);
        three.count = 3;
        let mut sector = params(0, Some(anf_fins(FinProfile::DoubleWedge)));
        sector.domain = Domain3::Sector;
        sector.nk = 20;
        let mut stacked_sector = sector.clone();
        stacked_sector.fin_stack = Some(stack);
        let mut stacked_half = params(0, Some(anf_fins(FinProfile::Airfoil)));
        stacked_half.fin_stack = Some(stack);
        let mut stacked_in_plane = params(0, Some(anf_fins(FinProfile::Square)));
        stacked_in_plane.fin_stack = Some(stack);
        stacked_in_plane.domain = Domain3::Half { phase: FinPhase::InPlane };
        // the fingerprint is the one-cell grid of the first mesher: its 32 chord cells (the Coarse tier's default is 64 since DESIGN3D item 47) are pinned
        let mut coarse = Case3::sector(2.0, Case3::anf_body(), Case3::anf_fins(), Tier::Coarse(fl), 0).grid;
        coarse.fin_chord_cells = Some(32);
        let all = [params(0, Some(anf_fins(FinProfile::Square))), params(0, Some(three)), sector, stacked_sector, stacked_half, stacked_in_plane, coarse];
        let got: Vec<u64> = all.iter().map(|p| fingerprint(&body_grid3(p).unwrap())).collect();
        eprintln!("fingerprints {got:x?}");
        assert!(all.iter().all(|p| p.fin_cells_across == 1));
        let expected: [u64; 7] = [
            9370751059839986959,
            14583859758804283294,
            6453443399358374581,
            17893710889803027280,
            1387836882980492884,
            5755310565696465849,
            11535300992460737863,
        ];
        assert_eq!(got, expected);
    }

    /// The number of solid cells of the fin layers.
    fn fin_cells(g: &Grid3) -> usize {
        let mut n = 0;
        for k in 0..g.nk {
            for j in 0..g.nj {
                for i in 0..g.ni {
                    n += usize::from(matches!(g.part_of(i, j, k), Some(Part::Fin(_))));
                }
            }
        }
        n
    }

    /// `n` cells across a fin, on the uniform half domain: every box is `n` cells thick, its faces are at the profile's half-thickness and the nodes between are
    /// uniform over it (the fin of the first test, swept and tapered, every profile).
    #[test]
    fn n_cells_across_follow_the_profile_with_uniform_nodes() {
        for n in [2usize, 4] {
            for profile in [FinProfile::Airfoil, FinProfile::DoubleWedge, FinProfile::Square] {
                let f = Fins3 {
                    count: 4,
                    root_chord: 1.0,
                    tip_chord: 0.6,
                    span: 1.0,
                    sweep: 0.3,
                    thickness: 0.08,
                    te_to_base: 0.0,
                    profile: profile.clone(),
                    planform: None,
                };
                let mut p = params(0, Some(f.clone()));
                p.fin_cells_across = n;
                let g = body_grid3(&p).unwrap();
                assert_eq!(g.nk, p.nk);
                assert_eq!(g.solid.len(), 2);
                for (m, b) in g.solid.iter().enumerate() {
                    assert_eq!(b.k.len(), n);
                    let th = PI / 4.0 + PI / 2.0 * m as f64;
                    for j in 0..=b.j.end {
                        for i in b.i.start..=b.i.end {
                            let p0 = node(&g, i, j, b.k.start);
                            let theta: Vec<f64> = (0..=g.nk).map(|k| node(&g, i, j, k)).map(|q| q[1].atan2(q[2])).collect();
                            let r =
                                (p0[1] * p0[1] + p0[2] * p0[2]).sqrt() / polygon_scale_where(&theta, |k| !g.solid.iter().any(|s| s.k.contains(&k)))[b.k.start];
                            let frac = (r - R) / f.span;
                            let chord = f.root_chord + (f.tip_chord - f.root_chord) * frac;
                            let xi = (p0[0] - (g.x[b.i.start] + f.sweep * frac)) / chord;
                            let h = half_thickness(&profile, f.thickness, chord, xi);
                            for q in 0..=n {
                                let want = h * (-1.0 + 2.0 * q as f64 / n as f64);
                                let d = dist(&g, i, j, b.k.start + q, th);
                                assert!((d - want).abs() < 1e-8, "n {n} {profile:?} ({i},{j}) node {q}: {d} vs {want}");
                            }
                        }
                    }
                }
            }
        }
    }

    /// `n` = 2 and 4 on the wall-resolved sector, the half domain and the in-plane half domain: positive volumes, the fin faces at the half-thickness, the nodes
    /// between them uniform, `n` times today's solid cells (a half fin in a plane: half of them, at least one), the stack unchanged, and the azimuthal count the
    /// stack's plus the extra layer cells.
    #[test]
    fn n_cells_across_in_a_sector_and_a_half_domain() {
        let fl = flow();
        let st = FinStack::from_flow(fl.re_c, fl.mach_max, 1.0, 15.0);
        for (label, domain) in
            [("sector", Domain3::Sector), ("half", Domain3::Half { phase: FinPhase::Between }), ("in plane", Domain3::Half { phase: FinPhase::InPlane })]
        {
            let build = |n: usize| {
                let mut p = params(0, Some(anf_fins(FinProfile::Square)));
                p.domain = domain;
                p.fin_stack = Some(st);
                p.fin_cells_across = n;
                body_grid3(&p).unwrap()
            };
            let n_up = anf_sting(0).n_up;
            let g1 = build(1);
            let per_layer = fin_cells(&g1) / g1.solid.iter().filter(|b| matches!(g1.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Fin(_)))).count();
            let rep1 = fin_stack_report(&g1).unwrap();
            for n in [2usize, 4] {
                let g = build(n);
                let layers: Vec<&SolidBox> = g.solid.iter().filter(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Fin(_)))).collect();
                // a half fin on a symmetry plane (k = 0, or the last cell of the half domain at pi) owns half the cells
                let half = |b: &SolidBox| b.k.start == 0 || b.k.end == g.nk;
                let cells: usize = layers.iter().map(|b| b.k.len()).sum();
                for b in &layers {
                    assert_eq!(b.k.len(), if half(b) { n / 2 } else { n }, "{label} n {n}: {b:?}");
                }
                assert_eq!(fin_cells(&g), per_layer * cells, "{label} n {n}: solid cells");
                let (c1, c): (usize, usize) = (g1.solid.iter().map(|b| b.k.len()).sum(), cells);
                assert_eq!(g.nk, g1.nk + c - c1, "{label} n {n}: the extra layer cells are extra k cells");
                if label == "sector" {
                    assert_eq!(fin_cells(&g), per_layer * (n / 2).max(1));
                }
                if label == "half" {
                    assert_eq!(fin_cells(&g), n * fin_cells(&g1), "half domain: n times today's solid cells");
                }
                for k in 0..g.nk {
                    for j in 0..g.nj {
                        for i in 0..g.ni {
                            if !g.is_solid(i, j, k) && i >= n_up {
                                assert!(g.vol[g.cell(i, j, k)] > 0.0, "{label} n {n}: volume at ({i},{j},{k})");
                            }
                        }
                    }
                }
                // faces at the half-thickness 0.04 and uniform nodes between (a half fin: from its mid-plane), mid-chord, root row and a row near the tip
                for b in &layers {
                    let th = if b.k.start == 0 {
                        0.0
                    } else if b.k.end == g.nk {
                        PI
                    } else if matches!(domain, Domain3::Half { .. }) {
                        let node = |k: usize| node(&g, (b.i.start + b.i.end) / 2, 0, k);
                        0.5 * (node(b.k.start)[1].atan2(node(b.k.start)[2]) + node(b.k.end)[1].atan2(node(b.k.end)[2]))
                    } else {
                        unreachable!()
                    };
                    let (lo, hi) = (if half(b) && b.k.start == 0 { 0.0 } else { -0.04 }, if half(b) && b.k.end == g.nk { 0.0 } else { 0.04 });
                    for (i, j) in [((b.i.start + b.i.end) / 2, 0), (b.i.end - 2, b.j.end - 1)] {
                        for q in 0..=b.k.len() {
                            let want = lo + (hi - lo) * q as f64 / b.k.len() as f64;
                            let d = dist(&g, i, j, b.k.start + q, th);
                            assert!((d - want).abs() < 1e-8, "{label} n {n} ({i},{j}) node {q}: {d} vs {want}");
                        }
                    }
                }
                let rep = fin_stack_report(&g).unwrap();
                assert!(
                    (rep.y1 / rep1.y1 - 1.0).abs() < 0.02 && rep.growth_max <= 1.35 + 1e-9 && rep.cells == rep1.cells,
                    "{label} n {n}: {rep:?} vs {rep1:?}"
                );
            }
        }
    }
}
