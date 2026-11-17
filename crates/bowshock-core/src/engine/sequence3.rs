//! Grid sequencing (DESIGN3D S9): the state of a coarse grid ([`crate::engine::grid3::Grid3Params::coarsened`]) as the start of the fine one, and back. The fine cell
//! `(i, j, k)` is a child of the coarse cell `(i / 2, j / 2, k)`: the grids are the same domain with every axial and radial count halved (semi-coarsening), the
//! azimuth is shared. [`prolong`] copies the parent's state to its four children (the first-order start the fine run corrects in its first-order iterations),
//! [`restrict`] is the volume-weighted mean of the children.

use bowshock_cfd::gas3::Cons3;
use bowshock_cfd::grid3::{Grid3, PART_FLUID};

/// Checks that `fine` is the 2 x 2 refinement of `coarse` in `(i, j)` with the same `nk`.
fn check_pair(coarse: &Grid3, fine: &Grid3) -> Result<(), String> {
    if fine.ni != 2 * coarse.ni || fine.nj != 2 * coarse.nj || fine.nk != coarse.nk {
        return Err(format!("the fine grid {} x {} x {} is not the refinement of {} x {} x {}", fine.ni, fine.nj, fine.nk, coarse.ni, coarse.nj, coarse.nk));
    }
    Ok(())
}

/// The state `uc` of `coarse` on `fine`: every fluid cell takes the state of its parent; the cells that are not fluid keep `fill`. `Err` when the grids do not pair,
/// or when a fluid child has a parent that is not fluid (the solid pattern is not aligned: the number of such cells is in the message).
pub fn prolong(coarse: &Grid3, uc: &[Cons3], fine: &Grid3, fill: Cons3) -> Result<Vec<Cons3>, String> {
    check_pair(coarse, fine)?;
    let mut u = vec![fill; fine.ni * fine.nj * fine.nk];
    let mut orphans = 0usize;
    for k in 0..fine.nk {
        for j in 0..fine.nj {
            for i in 0..fine.ni {
                let c = fine.cell(i, j, k);
                if fine.part[c] != PART_FLUID {
                    continue;
                }
                let p = coarse.cell(i / 2, j / 2, k);
                if coarse.part[p] != PART_FLUID {
                    orphans += 1;
                    continue;
                }
                u[c] = uc[p];
            }
        }
    }
    if orphans > 0 {
        return Err(format!("{orphans} fluid cells of the fine grid have a parent that is not fluid"));
    }
    Ok(u)
}

/// The state `u` of `fine` on `coarse`: the volume-weighted mean of the fluid children of every fluid coarse cell; the other coarse cells keep `fill`.
pub fn restrict(fine: &Grid3, u: &[Cons3], coarse: &Grid3, fill: Cons3) -> Result<Vec<Cons3>, String> {
    check_pair(coarse, fine)?;
    let mut uc = vec![fill; coarse.ni * coarse.nj * coarse.nk];
    for k in 0..coarse.nk {
        for j in 0..coarse.nj {
            for i in 0..coarse.ni {
                let p = coarse.cell(i, j, k);
                if coarse.part[p] != PART_FLUID {
                    continue;
                }
                let (mut sum, mut vol) = ([0.0; 5], 0.0);
                for (di, dj) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                    let c = fine.cell(2 * i + di, 2 * j + dj, k);
                    if fine.part[c] != PART_FLUID {
                        continue;
                    }
                    for (s, q) in sum.iter_mut().zip(u[c]) {
                        *s += q * fine.vol[c];
                    }
                    vol += fine.vol[c];
                }
                if vol > 0.0 {
                    uc[p] = sum.map(|s| s / vol);
                }
            }
        }
    }
    Ok(uc)
}
