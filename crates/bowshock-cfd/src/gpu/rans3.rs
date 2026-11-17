// The viscous / SST run of the 3D GPU solver (stage S5b): the integration of `viscous3.rs` and `turbulence3.rs` into the step, mirroring the CPU
// `solver3/rans.rs` (read its header: the per-iteration order). `include!`d twice by `gpu/mod.rs` after `turbulence3.rs`.
//
//  1. cons -> prim into the padded record w, ghost fill (`assign_ghosts3`; the i = ni static-pressure outflow of a viscous run)
//  2. viscous_cells3 (grad, mu), the wall state (SST), viscous_fluxes3 (Fv.n of every face)
//  3. the inviscid face fluxes, the solid side of a wall the NO-SLIP ghost (rho, -u, -v, -w, p) also in the MUSCL stencil (`wall_ghost3`)
//  4. res = -(1/V) sum S (F - Fv);  the time step adds the diffusive 2 lv A^2 / V of each direction
//  5. the implicit scheme (`implicit3.rs`: 2 lam_f I on every face dissipation, the no-slip wall Jacobian), then the turbulence update (SST)


/// The SST model of a viscous run (the constants live in `turbulence3.rs`; the wall distance and the other host data of the update are in `TurbGeo`).
pub(crate) struct TurbRun3 {
    re_c: f64,
}

impl TurbRun3 {
    /// The free-stream `[k, omega]` and `mu_t` of every cell (`turbulence3::init`).
    fn init_host(&self, solid: &[bool], rho: f64) -> (Vec<[f64; 2]>, Vec<f64>) {
        crate::turbulence3::init(&crate::turbulence3::Sst { re_c: self.re_c }, solid, |_| rho)
    }
}

/// What the viscous passes of one residual evaluation produced (read by the implicit scheme and the turbulence update).
struct VisAux {
    cell: VCell3,
    /// The viscous face fluxes `Fv.n` (momentum and energy) of the three families.
    fv: [Vec4; 3],
    /// The eddy viscosity of the evaluation (zeros for a laminar run).
    mu_t: A,
}

/// One residual evaluation: the residual, the time step (if asked), the primitives and the inviscid face fluxes `[fi, fj, fk]` (the mass flow of the
/// turbulence update is their component 0) and the viscous data.
struct Resid3 {
    res: Rec3,
    dt: Option<A>,
    p: Rec3,
    w: Rec3,
    f: [Rec3; 3],
    vis: Option<VisAux>,
}

impl LazySolver3 {
    /// The solid side of a wall seen from the fluid state `w` with outward normal `n`: the no-slip ghost `(rho, -u, -v, -w, p)` in a viscous run, the mirror image
    /// otherwise (`Solver3::wall_ghost`).
    fn wall_ghost3(&self, w: &Rec3, n: &Vec3) -> Rec3 {
        if self.visc.is_some() {
            [w[0].clone(), -&w[1], -&w[2], -&w[3], w[4].clone()]
        } else {
            reflect3(w, n)
        }
    }

    /// [`LazySolver3::wall_ghost3`] on the f32 left-hand side (also applied to the perturbation of the defect operator: the ghost map is linear).
    fn wall_ghost_f3(&self, w: &V5, n: &N3) -> V5 {
        if self.visc.is_some() {
            [w[0].clone(), -&w[1], -&w[2], -&w[3], w[4].clone()]
        } else {
            reflect_f3(w, n)
        }
    }

    /// `dU_ghost / dU` of the solid side of a wall face with unit normal `n`: the no-slip `diag(1, -1, -1, -1, 1)` of a viscous run
    /// (`Solver3::wall_jacobian`), the reflection otherwise; `one` the array of ones of the face shape.
    fn wall_jac_f3(&self, n: &N3, one: &F) -> Blk5 {
        if self.visc.is_some() {
            let mut m = Blk5::zero();
            for q in 0..5 {
                m.0[6 * q] = if q == 0 || q == 4 { E::O } else { E::A(-one) };
            }
            m
        } else {
            reflection_f3(n)
        }
    }

    /// The turbulence part of the next loop state: the SST update of `k`, `omega`, `mu_t` from the residual evaluation `last` and the time step `dt` (the CPU
    /// `Solver3::turbulence_update`); passed on unchanged without SST.
    fn turb_step3(&self, s: &State3, _dt: &A, _last: &Resid3) -> [A; 3] {
        turb_pass(s)
    }

    /// `[k, omega]` and `mu_t` of every cell on the host (`None` without SST).
    pub fn turb_f64(&self) -> Result<Option<(Vec<[f64; 2]>, Vec<f64>)>> {
        if self.turb.is_none() {
            return Ok(None);
        }
        let s = self.state.as_ref().ok_or_else(no_state)?;
        let (k, om, mt) = (to_f64(&s[7])?, to_f64(&s[8])?, to_f64(&s[9])?);
        Ok(Some(((0..k.len()).map(|q| [k[q], om[q]]).collect(), mt)))
    }

    /// Does the run carry the SST model?
    fn sst3(&self) -> bool {
        self.turb.is_some()
    }

    /// The eddy viscosity of the current loop state (`None` without SST).
    fn state_mu_t(&self) -> Option<A> {
        self.turb.as_ref()?;
        self.state.as_ref().map(|s| s[9].clone())
    }

    /// Switches the viscous terms (and with `Turbulence::Sst` the SST model) on from a host solver that has them (`Solver3::set_viscous`): the geometry of the
    /// face passes (`viscous3::face_dirs`, the symmetry sides derived from its flags), the outflow back pressure and, for SST, the free-stream turbulence state.
    pub fn set_viscous_from_host(&mut self, s: &Solver3) -> Result<()> {
        let Some(r) = s.rans() else {
            self.visc = None;
            self.turb = None;
            self.p_out = None;
            return Ok(());
        };
        let g = &s.grid;
        // a side of the domain box is a symmetry plane when its boundary faces carry the flag
        let sym_side = |dir: usize, hi: bool| -> bool {
            let shape = crate::viscous3::face_shape(g, dir);
            let fam = r.dirs.family(dir);
            let at = [0, shape[dir] - 1][hi as usize];
            (0..shape[0] * shape[1] * shape[2]).any(|q| {
                let idx = [q % shape[0], (q / shape[0]) % shape[1], q / (shape[0] * shape[1])];
                idx[dir] == at && fam[q].sym
            })
        };
        let sym: crate::viscous3::SymSides = [sym_side(0, false), sym_side(0, true), sym_side(1, false), sym_side(1, true), sym_side(2, false), sym_side(2, true)];
        self.visc = Some(VGeo3::new(&self.ctx, g, &self.solid_host, &sym, r.visc)?);
        self.p_out = r.p_outflow;
        self.turb = (r.visc.turbulence == Turbulence::Sst).then_some(TurbRun3 { re_c: r.visc.re_c });
        Ok(())
    }

    /// One residual evaluation (steps 1 to 4 of the module comment) of the conserved state `u`; `mu_t` the eddy viscosity (`None`: zero).
    fn residual3(&self, u: &Rec3, mu_t: Option<&A>, lim: Limiter, compute_dt: bool) -> Resid3 {
        let (p, w) = self.padded3(u);
        let f = self.face_fluxes3(&w, lim);
        let Some(vg) = &self.visc else {
            let (res, dt) = self.cell_residual3(&p, &f, compute_dt, None);
            return Resid3 { res, dt, p, w, f, vis: None };
        };
        let cell = self.viscous_cells3(vg, &p, &w);
        let mu_t = mu_t.cloned().unwrap_or_else(|| &cell.mu * c(0.0));
        let fv = self.viscous_fluxes3(vg, &w, &cell, &mu_t, None);
        let total: [Rec3; 3] = std::array::from_fn(|d| Self::total_flux3(&f[d], &fv[d]));
        let diff = compute_dt.then(|| self.diffusion_rates3(&cell, &mu_t, &p[0]));
        let (res, dt) = self.cell_residual3(&p, &total, compute_dt, diff.as_ref());
        Resid3 { res, dt, p, w, f, vis: Some(VisAux { cell, fv, mu_t }) }
    }
}
