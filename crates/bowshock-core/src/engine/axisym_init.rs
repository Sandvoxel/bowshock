//! Initial field of a 3D sector (or any 3D grid of the same meridian) from an axisymmetric single-cell wedge solution: every fluid cell of the destination takes the state of the
//! nearest fluid cell of the wedge in the meridian plane `(x, r)`, its velocity rotated to the azimuth of the destination cell. Nearest-cell (piecewise constant) transfer: on
//! the same wall-resolved meridian it is the identity per meridian cell.
//!
//! The grid axes are `(x, y, z) = (x, r sin(theta), r cos(theta))` ([`crate::engine::grid3`]), so the radial unit vector is `(0, sin theta, cos theta)` and `theta = atan2(y, z)`.

use bowshock_cfd::gas3::Cons3;
use bowshock_cfd::grid3::{Grid3, PART_FLUID};

/// The transferred field and how well the meridians matched.
pub struct AxisymInit {
    /// Conserved state of every destination cell (the free stream where the destination cell is not fluid).
    pub u: Vec<Cons3>,
    /// `[k, omega]` of every destination cell (`turb_fs` where it is not fluid).
    pub turb: Vec<[f64; 2]>,
    /// Largest `|x|` and `|r|` distances between a destination fluid cell centre and the source centre it took.
    pub max_dx: f64,
    pub max_dr: f64,
    /// Destination fluid cells whose nearest source cell was not fluid after the descent (found by a search over all source fluid cells).
    pub fallbacks: usize,
}

#[inline]
fn meridian(c: [f64; 3]) -> (f64, f64) {
    (c[0], c[1].hypot(c[2]))
}

/// Transfers the axisymmetric solution `(src_u, src_turb)` on `src` (a wedge: `nk = 1`) to the fluid cells of `dst`; the other cells keep `fs` and `turb_fs`.
pub fn axisym_to_sector(src: &Grid3, src_u: &[Cons3], src_turb: &[[f64; 2]], dst: &Grid3, fs: Cons3, turb_fs: [f64; 2]) -> AxisymInit {
    assert_eq!(src.nk, 1, "the source must be a single-cell wedge");
    let (sni, snj) = (src.ni, src.nj);
    let sc: Vec<(f64, f64)> = src.xc.iter().map(|&c| meridian(c)).collect();
    // `[rho, rho u_x, rho u_r, E]` of a wedge cell, the radial momentum from its own azimuth
    let src_rad: Vec<[f64; 4]> = (0..src.xc.len())
        .map(|c| {
            let th = src.xc[c][1].atan2(src.xc[c][2]);
            let q = src_u[c];
            [q[0], q[1], q[2] * th.sin() + q[3] * th.cos(), q[4]]
        })
        .collect();
    let fluid: Vec<usize> = (0..sc.len()).filter(|&c| src.part[c] == PART_FLUID).collect();
    assert!(!fluid.is_empty(), "the source has no fluid cell");
    let d2 = |c: usize, p: (f64, f64)| (sc[c].0 - p.0).powi(2) + (sc[c].1 - p.1).powi(2);

    let n = dst.xc.len();
    let (nni, nnj) = (dst.ni, dst.nj);
    let mut u = vec![fs; n];
    let mut turb = vec![turb_fs; n];
    let threads = std::thread::available_parallelism().map_or(1, |t| t.get());
    let chunk = n.div_ceil(threads).max(1);
    let stats: Vec<(f64, f64, usize)> = std::thread::scope(|s| {
        let handles: Vec<_> = u
            .chunks_mut(chunk)
            .zip(turb.chunks_mut(chunk))
            .enumerate()
            .map(|(t, (uc, tc))| {
                let (sc, src_rad, fluid, d2) = (&sc, &src_rad, &fluid, &d2);
                s.spawn(move || {
                    let (mut max_dx, mut max_dr, mut fallbacks) = (0.0f64, 0.0f64, 0usize);
                    for (o, (uo, to)) in uc.iter_mut().zip(tc.iter_mut()).enumerate() {
                        let c = t * chunk + o;
                        if dst.part[c] != PART_FLUID {
                            continue;
                        }
                        let (i, j) = (c % nni, (c / nni) % nnj);
                        let p = meridian(dst.xc[c]);
                        // greedy descent on the structured source meridian from the cell of the same relative index
                        let mut cur = (j * snj / nnj).min(snj - 1) * sni + (i * sni / nni).min(sni - 1);
                        loop {
                            let (ci, cj) = (cur % sni, cur / sni);
                            let mut best = (d2(cur, p), cur);
                            for dj in -1i64..=1 {
                                for di in -1i64..=1 {
                                    let (a, b) = (ci as i64 + di, cj as i64 + dj);
                                    if (di, dj) == (0, 0) || a < 0 || b < 0 || a >= sni as i64 || b >= snj as i64 {
                                        continue;
                                    }
                                    let nb = b as usize * sni + a as usize;
                                    let d = d2(nb, p);
                                    if d < best.0 {
                                        best = (d, nb);
                                    }
                                }
                            }
                            if best.1 == cur {
                                break;
                            }
                            cur = best.1;
                        }
                        if src.part[cur] != PART_FLUID {
                            fallbacks += 1;
                            cur = *fluid.iter().min_by(|&&a, &&b| d2(a, p).total_cmp(&d2(b, p))).unwrap();
                        }
                        max_dx = max_dx.max((sc[cur].0 - p.0).abs());
                        max_dr = max_dr.max((sc[cur].1 - p.1).abs());
                        let th = dst.xc[c][1].atan2(dst.xc[c][2]);
                        let [rho, mx, mr, e] = src_rad[cur];
                        *uo = [rho, mx, mr * th.sin(), mr * th.cos(), e];
                        *to = src_turb[cur];
                    }
                    (max_dx, max_dr, fallbacks)
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().expect("axisym_to_sector worker")).collect()
    });
    let (mut max_dx, mut max_dr, mut fallbacks) = (0.0f64, 0.0f64, 0usize);
    for (a, b, f) in stats {
        (max_dx, max_dr, fallbacks) = (max_dx.max(a), max_dr.max(b), fallbacks + f);
    }
    AxisymInit { u, turb, max_dx, max_dr, fallbacks }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::case3::Case3;

    /// The velocity of a transferred cell is the radial velocity of the wedge cell it took, along the radial direction of the destination cell: on the half domain
    /// (`theta` in `[0, pi]`, the pitch plane at `0` and `pi`, both signs of `z`) and on the sector, with the axial momentum and the scalars carried unchanged.
    #[test]
    fn the_radial_velocity_is_rotated_to_the_azimuth_of_every_destination_cell() {
        let wedge = Case3::wedge(2.0, Case3::anf_body(), 0).build_solver_cached().unwrap().grid;
        assert_eq!(wedge.nk, 1);
        let (mr, mx) = (0.5, 0.3);
        let src_u: Vec<Cons3> = wedge
            .xc
            .iter()
            .map(|c| {
                let th = c[1].atan2(c[2]);
                [1.0 + 0.01 * c[0], mx, mr * th.sin(), mr * th.cos(), 2.0]
            })
            .collect();
        let src_turb: Vec<[f64; 2]> = wedge.xc.iter().map(|c| [c[0], 7.0]).collect();
        let fs = [9.0; 5];
        let mut cases = vec![Case3::tiny_anf(2.0, 0.0)];
        let mut sector = Case3::tiny_anf(2.0, 0.0);
        sector.grid.domain = crate::engine::grid3::Domain3::Sector;
        cases.push(sector);
        for case in cases {
            let dst = case.build_solver_cached().unwrap().grid;
            let m = axisym_to_sector(&wedge, &src_u, &src_turb, &dst, fs, [1.0, 1.0]);
            let (mut fluid, mut lee, mut windward) = (0, 0, 0);
            for c in 0..dst.xc.len() {
                if dst.part[c] != PART_FLUID {
                    assert_eq!((m.u[c], m.turb[c]), (fs, [1.0, 1.0]), "a solid cell keeps the free stream");
                    continue;
                }
                fluid += 1;
                let th = dst.xc[c][1].atan2(dst.xc[c][2]);
                lee += usize::from(dst.xc[c][2] > 0.0);
                windward += usize::from(dst.xc[c][2] < 0.0);
                let [_, ux, uy, uz, e] = m.u[c];
                let radial = uy * th.sin() + uz * th.cos();
                let azimuthal = uy * th.cos() - uz * th.sin();
                assert!((radial - mr).abs() < 1e-12 && azimuthal.abs() < 1e-12 && (ux - mx).abs() < 1e-12 && e == 2.0, "cell {c}: {:?}", m.u[c]);
                assert_eq!(m.turb[c][1], 7.0);
            }
            assert!(fluid > 0 && lee > 0);
            // the half domain reaches the windward side, the sector (`[0, pi/4]` of four fins) does not
            assert_eq!(windward > 0, matches!(dst.sym, bowshock_cfd::grid3::Symmetry::Half), "windward cells {windward}");
        }
    }
}
