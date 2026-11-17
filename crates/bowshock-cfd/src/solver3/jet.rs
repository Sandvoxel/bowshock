//! The motor-jet patch of [`Solver3`] (DESIGN3D section L, item 20; the 3D counterpart of the 2D `Nozzle` of `solver.rs`, `implicit.rs`, `viscous.rs::Walls::jet_iface` and
//! `turbulence.rs`). The mesher marks the two cell columns of the body block just ahead of the base plane in the rows of the nozzle `Part::Jet`; their `+i` faces against the
//! fluid are the exit. What the solver does with them:
//!
//! * a `Part::Jet` cell is solid (never updated) but **no wall**: [`Solver3::set_jet`] clears its `wall` flag, so no reflection or no-slip ghost is built at its faces, and sets
//!   its conserved state to the jet state; the padded array `w` then holds the jet state in both columns (the two stencil layers);
//! * inviscid flux of an exit face: the stencil behind it is the jet state, `(jet, jet, w[i], w[i + 1])` at the exit, and the second face of the fluid has `(jet, w[i_end], ..)`
//!   from the padded array (`Family::jet_at` in `solver3.rs`, the 2D `(m2, m1)`);
//! * implicit (`implicit3::assemble3`, `diag3`): in the `i` factor the cell behind the exit has the fixed jet state as its lower neighbour: no sub-diagonal block and no
//!   boundary Jacobian (2D `implicit.rs`: "fixed ghost: no dependence on the interior"); the diffusive and convective dissipation read the jet state;
//! * viscous: the exit face is no no-slip wall (`SideKind::Jet`): its ghost is the jet state, placed at the mirror image of the centre of the fluid cell, with the viscosity
//!   `mu(T_jet)`, the eddy viscosity `rho_jet k / omega` (SST) and the gradient of the fluid cell, which [`Solver3::jet_viscous_fix`] stores in the jet cell every iteration
//!   (the kernels read the jet cell as the ghost's cell); the jet faces are not in the wall-distance set and give the cell behind them no wall state;
//! * turbulence: the `k`, `omega` ghost of the exit is the jet's (`turb[jet cell]`, re-stored every iteration: the update zeroes the entries of solid cells).
//!
//! `Solver3::set_jet` after `set_viscous_with` or before: both orders work (the viscous data are stored in every residual evaluation).

use super::Solver3;
use crate::gas3::{prim_to_cons3, Prim3, GAMMA};
use crate::solver::Turbulence;
use crate::viscous3::is_jet_cell;

/// The state of the jet at the exit plane in solver units: `[rho, u, v, w, p]` (axial) and the `[k, omega]` of the SST run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JetState3 {
    pub state: Prim3,
    pub turb: [f64; 2],
}

/// The jet cells of a solver and what they hold.
pub(crate) struct JetPatch {
    pub state: JetState3,
    /// Per cell: a `Part::Jet` cell.
    pub mask: Vec<bool>,
    /// `(jet cell, the fluid cell behind it if any)` of every jet cell: the fluid cell gives the jet cell its gradient.
    pub cells: Vec<(usize, Option<usize>)>,
}

impl Solver3 {
    /// Give the `Part::Jet` cells of the grid the jet state (`Some`) or make them walls again (`None`). A grid without `Part::Jet` cells is not changed.
    pub fn set_jet(&mut self, jet: Option<JetState3>) {
        if let Some(old) = self.jet.take() {
            for &(c, _) in &old.cells {
                self.wall[c] = true;
            }
        }
        let Some(state) = jet else { return };
        let (ni, nj) = (self.grid.ni, self.grid.nj);
        let q = prim_to_cons3(&state.state);
        let mut mask = vec![false; self.u.len()];
        let mut cells = Vec::new();
        for c in 0..self.u.len() {
            if !is_jet_cell(&self.grid, c) {
                continue;
            }
            mask[c] = true;
            self.wall[c] = false;
            self.u[c] = q;
            // the fluid cell behind the exit (`+i`), if the jet cell has one
            let behind = (c % ni + 1 < ni && !self.solid[c + 1]).then_some(c + 1);
            cells.push((c, behind));
        }
        let _ = nj;
        self.jet = Some(Box::new(JetPatch { state, mask, cells }));
        self.jet_viscous_fix();
    }

    /// The jet of the solver, `None` for none.
    pub fn jet(&self) -> Option<JetState3> {
        self.jet.as_ref().map(|j| j.state)
    }

    /// The viscous data the kernels read in a jet cell, as the ghost of its exit face: `mu(T_jet)`, the eddy viscosity `rho k / omega` (SST) and the scalars of the jet, and the
    /// gradient of the fluid cell behind it. Call after `viscous3::cell_pass` (which zeroes the solid cells) in every residual evaluation, and after the turbulence update.
    pub(crate) fn jet_viscous_fix(&mut self) {
        let (Some(j), Some(r)) = (self.jet.as_deref(), self.rans.as_deref_mut()) else { return };
        let [rho, _, _, _, p] = j.state.state;
        let sst = r.visc.turbulence == Turbulence::Sst;
        let mu = r.visc.mu(GAMMA * p / rho);
        let mu_t = if sst { rho * j.state.turb[0] / j.state.turb[1] } else { 0.0 };
        for &(c, behind) in &j.cells {
            r.mu[c] = mu;
            r.mu_t[c] = mu_t;
            if let Some(f) = behind {
                r.grad[c] = r.grad[f];
            }
            if sst {
                r.turb[c] = j.state.turb;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gas3::freestream3;
    use crate::grid3::{Part, SolidBox};
    use crate::implicit::{ImplicitParams, LineSolve3};
    use crate::limiter::Limiter;
    use crate::solver::Viscous;
    use crate::solver3::{Config3, Scheme3};
    use crate::viscous3::tests::box_grid;
    use crate::viscous3::{face_cells, SideKind};
    use crate::walldist3::wall_faces;

    /// A box of `8 x 4 x 4` cells whose first two columns (`i < 2`, every row and plane) are the jet cells, uniform axial flow at Mach 0.5 everywhere, the jet state equal to it.
    fn box_with_jet(scheme: Scheme3, viscous: bool) -> Solver3 {
        let mut g = box_grid([8, 4, 4], [0.1, 0.1, 0.1]);
        g.mark_solid(SolidBox { i: 0..2, j: 0..4, k: 0..4 }, Some(Part::Jet));
        let fs = freestream3(0.5, 0.0);
        let cfg = Config3 { freestream: fs, cfl: 0.5, limiter: Limiter::VanAlbada, first_order_iters: 0, low_mach: true, scheme };
        let mut s = Solver3::new(g, cfg);
        if viscous {
            s.set_viscous(Some(Viscous { re_c: 500.0, sutherland: 0.368, turbulence: crate::solver::Turbulence::Sst, wall_i_start: 0, roughness: 0.0 }));
        }
        s.set_jet(Some(JetState3 { state: fs, turb: [1e-6, 1.0] }));
        s
    }

    /// The exit faces are an inflow of the jet state, not a wall: with the jet state equal to the uniform flow behind it the residual vanishes in every fluid cell; the same grid
    /// with the `Part::Jet` cells left as walls (`set_jet(None)`) has a residual at the exit.
    #[test]
    fn the_exit_faces_carry_the_jet_state_not_a_wall() {
        let mut s = box_with_jet(Scheme3::Explicit, false);
        s.eval_residual();
        let worst = |s: &Solver3| (0..s.u.len()).filter(|&c| !s.solid[c]).flat_map(|c| s.res[c]).fold(0.0f64, |m, r| m.max(r.abs()));
        assert!(worst(&s) < 1e-12, "residual with the jet {}", worst(&s));
        s.set_jet(None);
        s.eval_residual();
        assert!(worst(&s) > 1e-3, "a wall at the exit must leave a residual: {}", worst(&s));
    }

    /// The jet cells are solid cells that are no walls, hold the jet state in `w` (both columns), and the exit face is the `Jet` side of `face_cells` and no no-slip wall.
    #[test]
    fn jet_cells_are_no_walls_and_hold_the_jet_state() {
        let mut s = box_with_jet(Scheme3::Explicit, true);
        let jet = s.jet().expect("jet");
        s.eval_residual();
        let g = &s.grid;
        for j in 0..4 {
            for k in 0..4 {
                for i in 0..2 {
                    let c = g.cell(i, j, k);
                    assert!(s.solid[c] && !s.wall[c], "jet cell ({i}, {j}, {k})");
                    assert_eq!(s.prim(i, j, k), jet.state);
                }
                assert!(!s.wall[g.cell(2, j, k)] && !s.solid[g.cell(2, j, k)]);
                let fc = face_cells(g, &s.solid, 0, 2, j, k).expect("exit face");
                assert_eq!(fc.kind, [SideKind::Jet, SideKind::Cell]);
                assert!(!fc.noslip());
            }
        }
        // no wall face for the distance field, no wall traction: the exit is no wall
        assert!(wall_faces(g, &s.solid).is_empty());
        assert!(s.wall_tractions().is_empty());
        // the ghost's viscosity, eddy viscosity and gradient: mu(T_jet), rho k / omega, the fluid cell's gradient
        let r = s.rans().unwrap();
        let c = g.cell(1, 2, 2);
        assert_eq!(r.mu[c], r.visc.mu(crate::gas3::GAMMA * jet.state[4] / jet.state[0]));
        assert_eq!(r.mu_t[c], jet.state[0] * 1e-6 / 1.0);
        assert_eq!(r.turb[c], [1e-6, 1.0]);
        assert_eq!(r.grad[c], r.grad[g.cell(2, 2, 2)]);
    }

    /// Both line solves keep the uniform flow through the jet exit (laminar and SST): the fixed-ghost blocks make no non-finite or non-zero update.
    #[test]
    fn implicit_steps_keep_the_uniform_flow_through_the_exit() {
        for line_solve in [LineSolve3::Block, LineSolve3::DiagonalCorrected] {
            for viscous in [false, true] {
                let p = ImplicitParams { cfl_max: 5.0, growth: 1.5, line_solve, ..ImplicitParams::VISCOUS };
                let mut s = box_with_jet(Scheme3::Implicit(p), viscous);
                let q0 = s.u[s.grid.cell(5, 2, 2)];
                for _ in 0..6 {
                    s.step();
                }
                let dev = s.u.iter().enumerate().filter(|(c, _)| !s.solid[*c]).flat_map(|(_, q)| (0..5).map(|k| (q[k] - q0[k]).abs())).fold(0.0f64, f64::max);
                assert!(dev < 1e-9, "{line_solve:?} viscous {viscous}: deviation {dev}");
            }
        }
    }
}
