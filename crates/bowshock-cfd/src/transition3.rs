//! Prescribed (fixed-location) transition of the 3D SST model: an opt-in laminar run on the walls upstream of a given station, turbulent downstream.
//!
//! **The mechanism** is a fixed intermittency `gamma` per cell, `0` (laminar) or `1` (turbulent), the field the Langtry-Menter transition model would compute, here prescribed
//! (the `gamma` of [`crate::turbulence3::TurbIn::gamma`]; an empty field is `1` everywhere: the fully turbulent solver, unchanged bit for bit). Where `gamma = 0`:
//!
//! * the eddy viscosity is zero (`mu_t = gamma mu_t,SST` after every update, also in the initial state). Since the production of both equations is `P = mu_t S^2` (and the
//!   source of `omega` is `gamma_SST rho P / mu_t`), **this one factor is the suppression of the production of `k` and `omega`**: nothing is produced, `k` and `omega` are
//!   convected, diffused and destroyed as in the free stream. This is what a "laminar zone" does in commercial codes (ANSYS Fluent: turbulent viscosity and production
//!   of the turbulence equations zero in the zone) and what the `gamma P_k` factor of Menter's `gamma-Re_theta` model does with `gamma = 0` (its destruction factor
//!   `min(max(gamma, 0.1), 1)` is not used: it only matters for the free-stream decay of `k`, which is slow here (`omega_inf = 1e-6 Re`));
//! * a wall cell takes the **laminar wall state** (the `y+ -> 0` limit of [`crate::turbulence3::wall_state`], the state of a plain resolved wall: `mu_t,w = 0`, `k_w = 0`,
//!   Menter's `omega_w`), whatever its `y+` (the law of the wall would put the turbulent shear on a wall cell above `y+ ~ 5`: the fin stacks of the Coarse grid have `y+` 10-17
//!   in the turbulent boundary layer, but a laminar layer of the same Reynolds number is `5-10` times thicker and its first cell resolves it), whatever its roughness
//!   (a laminar layer does not feel `ks` below the trip; the free-flight model's roughness is what trips it), and its `S^2` comes from the velocity gradient, not from
//!   the law of the wall ([`crate::turbulence3::TurbIn::gamma`]).
//!
//! **The field** ([`gamma_field`]) is the property of the nearest wall of the cell (the exact nearest wall triangle of [`crate::walldist3`]): a cell whose nearest wall face lies
//! on a body segment or the sting upstream of the station `x_tr` (nose diameters from the nose tip) is laminar; one whose nearest face lies on a fin is laminar upstream of
//! the fraction `fin_frac` of the local fin chord (`x_le(j) + fin_frac (x_te(j) - x_le(j))`, `x_le`, `x_te` the extreme `x` of the fin cells of the row `j` of the face's
//! solid cell: a swept fin has a swept transition line). Cells of other parts and cells that touch no wall (solid, no wall face) are turbulent. Free-stream cells take the
//! property of the wall they are nearest to, which is harmless: the production is zero there anyway.
//!
//! **Why not the alternatives.** A full `gamma-Re_theta` model needs two more transport equations, a calibrated free-stream turbulence intensity and pressure-gradient
//! correlations that are not validated for supersonic slender bodies (Tu of a free-flight range is not known to a factor of 10); the transition location of a rocket in free flight
//! is a roughness and Reynolds-number question (see DESIGN3D item 50), so the location is the parameter. A `mu_t` trip of the Baldwin-Lomax kind (`mu_t` zeroed, production left on) hides
//! a developed `k` in the laminar layer that appears at once at the trip; with the production off the turbulence grows from the free-stream value over a distance of
//! the order of the trip location itself, like a physical transition length (the plate test of the module measures it).

use crate::grid3::{Grid3, Part};
use crate::walldist3::{wall_nearest_face3, WallFace};
use std::collections::HashMap;

/// Prescribed transition locations (see the module documentation).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transition {
    /// Station of the transition on the body (and the sting), nose diameters from the nose tip: laminar upstream, turbulent downstream. `0`: turbulent from the tip.
    pub x_tr: f64,
    /// Fraction of the local chord of a fin that is laminar from the leading edge: `0` turbulent from the leading edge, `1` laminar everywhere.
    pub fin_frac: f64,
}

/// Is the wall face `f` laminar? `part` of the solid cell behind it, `rows` the `x` extent `(x_le, x_te)` of the fin cells by `(fin set, j)`.
fn laminar(grid: &Grid3, f: &WallFace, tr: Transition, rows: &HashMap<(u16, usize), (f64, f64)>) -> bool {
    let (ni, nj) = (grid.ni, grid.nj);
    let [i, j, _] = f.idx;
    let n = [ni, nj, grid.nk][f.dir];
    // the solid cell of the face: behind it (index - 1) or at it; a `j = 0` boundary wall has none (its part is the run's)
    let cell = |lo: bool| {
        let mut c = f.idx;
        if lo {
            c[f.dir] -= 1;
        }
        c
    };
    let solid_cell = if f.idx[f.dir] == 0 || f.idx[f.dir] >= n {
        None
    } else {
        [cell(true), cell(false)].into_iter().find(|c| grid.is_wall(c[0], c[1], c[2]))
    };
    let part = match solid_cell {
        Some(c) => grid.part_of(c[0], c[1], c[2]),
        None if f.dir == 1 && j == 0 => grid.wall_j0.iter().find(|(r, _)| r.contains(&i)).map(|(_, p)| *p),
        None => None,
    };
    let xf = f.corners.iter().map(|p| p[0]).sum::<f64>() / 4.0;
    match part {
        Some(Part::Body(_)) | Some(Part::Sting) => xf < tr.x_tr,
        Some(Part::Fin(s)) => {
            if tr.fin_frac >= 1.0 {
                return true;
            }
            let row = solid_cell.map_or(j, |c| c[1]);
            match rows.get(&(s, row)) {
                Some(&(x_le, x_te)) => xf < x_le + tr.fin_frac * (x_te - x_le),
                None => false,
            }
        }
        _ => false,
    }
}

/// The `x` extent of the fin cells by `(fin set, j)`.
fn fin_rows(grid: &Grid3) -> HashMap<(u16, usize), (f64, f64)> {
    let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
    let mut rows: HashMap<(u16, usize), (f64, f64)> = HashMap::new();
    for k in 0..nk {
        for j in 0..nj {
            for i in 0..ni {
                if let Some(Part::Fin(s)) = grid.part_of(i, j, k) {
                    let e = rows.entry((s, j)).or_insert((f64::INFINITY, f64::NEG_INFINITY));
                    for (di, dj, dk) in [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (1, 0, 1), (0, 1, 1), (1, 1, 1)] {
                        let x = grid.x[((k + dk) * (nj + 1) + j + dj) * (ni + 1) + i + di];
                        e.0 = e.0.min(x);
                        e.1 = e.1.max(x);
                    }
                }
            }
        }
    }
    rows
}

/// The prescribed intermittency of every cell (`ni nj nk`, index `Grid3::cell`): `0` laminar, `1` turbulent (see the module documentation), from the wall `faces`
/// ([`crate::walldist3::wall_faces`]) of the grid. Empty when no cell is laminar (`x_tr <= 0` and `fin_frac <= 0`, or no wall face upstream): the solver treats an empty field
/// as fully turbulent.
pub fn gamma_field(grid: &Grid3, solid: &[bool], faces: &[WallFace], tr: Transition) -> Vec<f64> {
    assert!(tr.x_tr >= 0.0 && (0.0..=1.0).contains(&tr.fin_frac), "transition: x_tr >= 0 and 0 <= fin_frac <= 1");
    if tr.x_tr <= 0.0 && tr.fin_frac <= 0.0 {
        return Vec::new();
    }
    let rows = fin_rows(grid);
    let lam: Vec<bool> = faces.iter().map(|f| laminar(grid, f, tr, &rows)).collect();
    let near = wall_nearest_face3(grid, solid, faces);
    let gamma: Vec<f64> = near.iter().map(|&f| if f != u32::MAX && lam[f as usize] { 0.0 } else { 1.0 }).collect();
    if gamma.iter().all(|&g| g == 1.0) {
        Vec::new()
    } else {
        gamma
    }
}
