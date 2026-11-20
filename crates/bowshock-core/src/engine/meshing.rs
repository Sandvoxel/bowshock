//! The grid parameters of a case at a grid level, shared by every engine: the sting and base-wake meridian grids of a body ([`sting_grid`], [`base_grid`]), the first cell height of a
//! wall-resolved run ([`wall_spacing`], [`resolve_walls`]), the nozzle rows of a motor ([`motor_rows`], [`Jet`]).

use crate::engine::body_grid::{BaseBody, BaseGridParams, LargeGridParams, NozzleRows};

/// Treatment of the region behind the body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Wake {
    /// The cylinder continues as a sting to the outflow plane (`large_domain_grid`), no base.
    Sting,
    /// Finite body with a flat base and a wake region (`base_domain_grid`).
    Base,
}

/// Motor jet at the nozzle exit, internal units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Jet {
    /// Exit radius.
    pub r_exit: f64,
    /// Primitive state at the exit (supersonic, axial).
    pub state: [f64; 4],
    /// `[k, omega]` of the jet.
    pub turb: [f64; 2],
}

/// Smallest even integer `>= x` (`x >= 0`).
fn even_ceil(x: f64) -> usize {
    let n = x.ceil() as usize;
    n + n % 2
}

/// Number of rows of a wall-clustered layer of height `h` with first cell height `y1` and growth ratio 1.2 (even).
/// Splits the gap behind the base of `grid` into the nozzle exit (rows clustered at the lip, `n_jet` rows) and the annulus around it (`n_ann` rows, at least 4),
/// `n_gap = n_jet + n_ann`, the first cell height `dr_wall` of the grid: the base grid of [`crate::engine::case::Case::with_motor`] and of [`crate::engine::case3::Case3::with_motor`].
pub(crate) fn motor_rows(grid: &mut BaseGridParams, body: &BaseBody, jet: &Jet) {
    let y1 = grid.dr_wall;
    let re = body.base_radius();
    let n_jet = layer_count(jet.r_exit, y1);
    let n_ann = (2 * layer_count(0.5 * (re - jet.r_exit), y1)).max(4);
    grid.n_gap = n_jet + n_ann;
    grid.nozzle = Some(NozzleRows { r_exit: jet.r_exit, n_jet });
}

fn layer_count(h: f64, y1: f64) -> usize {
    even_ceil((1.0 + 0.2 * h / y1).ln() / 1.2f64.ln())
}

/// First cell height at the body wall \[d\] for `y_plus` at the highest Mach number `mach_max` of a sweep: the flat-plate friction over the body length,
/// `cf = 0.027 (re_c M L)^(-1/7)`, `y1 = y+ / (re_c M sqrt(cf/2))` (`re_c = rho c d / mu` of the free stream).
pub(crate) fn wall_spacing(body: &BaseBody, re_c: f64, mach_max: f64, y_plus: f64) -> f64 {
    let cf = 0.027 * (re_c * mach_max * body.x_base()).powf(-1.0 / 7.0);
    y_plus / (re_c * mach_max * (0.5 * cf).sqrt())
}

/// Wall-resolve both meridian grids of a case for the first cell height `y1`: radial cell counts for a growth ratio of 1.2, the gap behind the base clustered at
/// the base corner only; a body on a sting has two walls bounding the gap (the base corner above, the sting below, a wall on the whole length behind the base), so its rows are
/// clustered at both ends (`2 layer_count(half the gap)` rows; with the one-sided rows the sting wall cells were as large as the gap's growth made them, `y+` 155-340 on the sting of the
/// Fox and Arcas tunnel models, DESIGN3D item 65).
pub(crate) fn resolve_walls(grid_base: &mut BaseGridParams, grid_sting: &mut LargeGridParams, body: &BaseBody, y1: f64) {
    let (re, rs) = (body.base_radius(), body.sting_radius);
    grid_base.dr_wall = y1;
    grid_base.nj_out = layer_count(grid_base.r_far - re, y1);
    let dr_gap = grid_base.dr_gap.unwrap_or(y1);
    if rs > 0.0 {
        grid_base.n_gap = 2 * layer_count(0.5 * (re - rs), dr_gap);
        grid_base.gap_one_sided = false;
    } else {
        grid_base.n_gap = layer_count(re - rs, dr_gap);
        grid_base.gap_one_sided = true;
    }
    grid_sting.dr_wall = y1;
    grid_sting.nj = layer_count(grid_sting.r_far, y1);
}

/// Longest mean axial cell of an aft segment (tube or cone) of the grids of [`Case::new`] at level 0, in nose diameters; level `l`
/// divides it by `2^l`. A segment has 40 cells (tube), 36 (last tube of a sting grid) or 24 (cone) at level 0 however long it is, so a
/// tube of 40 d, the longest body the facade first accepted, had a mean of 1 d per cell (its largest cell, in the middle of the clustered
/// distribution, 3 d). A longer segment takes more cells to stay within that mean, and so within that largest cell; a shorter one
/// keeps its count.
const MAX_AXIAL_CELL: f64 = 1.0;

/// Cells of an aft segment of `length` at grid level `log2(f)`: `per` (the count of the segment kind, already scaled with `f`), or more
/// so that no cell is longer than [`MAX_AXIAL_CELL`]` / f`, and never finer than the end-cell size `dx_tip`; even, at least 4.
fn segment_cells(per: usize, length: f64, f: usize, dx_tip: f64) -> usize {
    let coarsest = even_ceil(length * f as f64 / MAX_AXIAL_CELL);
    per.max(coarsest).min(even_ceil(length / dx_tip)).max(4)
}

/// Sting-wake grid parameters of [`Case::new`] for `body` at grid `level` (domain `10 L` upstream and radially, `5 L` downstream).
pub(crate) fn sting_grid(body: &BaseBody, level: u32) -> LargeGridParams {
    let f = 1usize << level;
    let dx_tip = 0.03 / f as f64;
    let l_body = body.x_base();
    let nseg = body.segments.len();
    let n_seg_sting: Vec<usize> = body
        .segments
        .iter()
        .enumerate()
        .map(|(k, s)| {
            if s.is_step() {
                return 1;
            }
            segment_cells(
                if s.slope != 0.0 {
                    24 * f
                } else if k + 1 == nseg {
                    36 * f
                } else {
                    40 * f
                },
                s.length,
                f,
                dx_tip,
            )
        })
        .collect();
    LargeGridParams {
        body: body.clone(),
        x_up: 10.0 * l_body,
        x_down: 5.0 * l_body,
        r_far: 10.0 * l_body,
        n_up: 30 * f,
        n_nose: 48 * f,
        n_seg: n_seg_sting,
        n_ext: 30 * f,
        nj: 70 * f,
        dx_tip,
        dr_wall: 0.03 / f as f64,
    }
}

/// Base-wake grid parameters of [`Case::new`] for `body` at grid level `level` (domain `10 L` upstream and radially, `5 L` downstream).
pub(crate) fn base_grid(body: &BaseBody, level: u32) -> BaseGridParams {
    let f = 1usize << level;
    let dx_tip = 0.03 / f as f64;
    let l_body = body.x_base();
    let n_sting = if body.sting_radius > 0.0 { 4 * f } else { 0 };
    // cells per segment: tubes are finer than cones, never finer than the end-cell size `dx_tip` allows, never coarser than
    // `MAX_AXIAL_CELL / f` (see `segment_cells`)
    let n_seg: Vec<usize> =
        body.segments.iter().map(|s| if s.is_step() { 1 } else { segment_cells(if s.slope == 0.0 { 40 * f } else { 24 * f }, s.length, f, dx_tip) }).collect();
    BaseGridParams {
        body: body.clone(),
        x_up: 10.0 * l_body,
        x_down: 5.0 * l_body,
        r_far: 10.0 * l_body,
        n_up: 28 * f,
        n_nose: 48 * f,
        n_seg,
        n_wake: 40 * f,
        wake_near: None,
        n_sting,
        n_gap: 12 * f,
        nj_out: 64 * f,
        dx_tip,
        dr_wall: 0.03 / f as f64,
        gap_one_sided: false,
        dr_gap: None,
        nozzle: None,
    }
}
/// Cell counts of the Draft tier relative to the wall-resolved tiers: axial cells (upstream, nose, segments, wake) times this, even, at least 4.
pub const DRAFT_AXIAL: f64 = 0.6;
/// Domain of the Draft tier in body lengths: inflow plane and outer radius, outflow plane (the full tiers: 10 and 5). The grid does not depend on the Mach number
/// (a table shares it), so this is the size a subsonic far field still tolerates with the characteristic boundary conditions.
pub const DRAFT_DOMAIN: (f64, f64) = (5.0, 3.0);
/// First cell height of the gap rows behind the base of the Draft tier (the wall-resolved tiers: the wall spacing, 1e-5 d and less).
pub const DRAFT_DR_GAP: f64 = 0.01;
/// End-cell size at the nose tip and the segment ends of the Draft tier.
pub const DRAFT_DX_TIP: f64 = 0.05;

/// The coarse axial grid and the smaller domain of the Draft tier for `body` ([`crate::engine::case3::Tier::Draft`]); the radial layers follow from the wall spacing
/// ([`resolve_walls`], call it after this: the layer counts depend on the outer radius).
pub(crate) fn draft_coarsen(base: &mut BaseGridParams, sting: &mut LargeGridParams, body: &BaseBody) {
    let cut = |n: usize| even_ceil(n as f64 * DRAFT_AXIAL).max(4);
    let l = body.x_base();
    base.n_up = cut(base.n_up);
    base.n_nose = cut(base.n_nose);
    base.n_seg.iter_mut().zip(&body.segments).for_each(|(n, s)| *n = if s.is_step() { 1 } else { cut(*n) });
    base.n_wake = cut(base.n_wake);
    base.dx_tip = DRAFT_DX_TIP;
    base.dr_gap = Some(DRAFT_DR_GAP);
    (base.x_up, base.r_far, base.x_down) = (DRAFT_DOMAIN.0 * l, DRAFT_DOMAIN.0 * l, DRAFT_DOMAIN.1 * l);
    sting.n_up = cut(sting.n_up);
    sting.n_nose = cut(sting.n_nose);
    sting.n_seg.iter_mut().zip(&body.segments).for_each(|(n, s)| *n = if s.is_step() { 1 } else { cut(*n) });
    sting.n_ext = cut(sting.n_ext);
    sting.dx_tip = DRAFT_DX_TIP;
    (sting.x_up, sting.r_far, sting.x_down) = (DRAFT_DOMAIN.0 * l, DRAFT_DOMAIN.0 * l, DRAFT_DOMAIN.1 * l);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A segment that the facade accepted before the cell count followed the length keeps its count at every level; a longer one grows
    /// linearly with the length and never has a cell above `1 / f`.
    #[test]
    fn segment_cells_keep_the_old_counts_and_grow_past_the_old_limit() {
        for level in 0..3u32 {
            let f = 1usize << level;
            let dx_tip = 0.03 / f as f64;
            for per in [24 * f, 40 * f] {
                for k in 1..=4 * per as i32 {
                    let length = f64::from(k) * 0.25 / f as f64;
                    let old = per.min(even_ceil(length / dx_tip)).max(4);
                    let new = segment_cells(per, length, f, dx_tip);
                    if length * f as f64 <= per as f64 {
                        assert_eq!(new, old, "level {level}, per {per}, length {length}");
                    } else {
                        assert!(new > old && length / new as f64 <= 1.0 / f as f64 + 1e-12, "level {level}, per {per}, length {length}: {new}");
                    }
                }
            }
        }
        // the 46 d rocket's 25.7 d tube keeps its 40 cells, an 80 d one (59.7 d tube) gets 60
        assert_eq!(segment_cells(40, 25.7, 1, 0.03), 40);
        assert_eq!(segment_cells(40, 59.7, 1, 0.03), 60);
        assert_eq!(segment_cells(80, 59.7, 2, 0.015), 120);
    }
}
