// The viscous / SST run of the 3D GPU solver (stage S5b): the integration of `viscous3.rs` and `turbulence3.rs` into the step, mirroring the CPU
// `solver3/rans.rs` (read its header: the per-iteration order). `include!`d twice by `gpu/mod.rs` after `turbulence3.rs`.
//
//  1. cons -> prim into the padded record w, ghost fill (`assign_ghosts3`; the i = ni static-pressure outflow of a viscous run)
//  2. viscous_cells3 (grad, mu), the wall state (SST), viscous_fluxes3 (Fv.n of every face)
//  3. the inviscid face fluxes, the solid side of a wall the NO-SLIP ghost (rho, -u, -v, -w, p) also in the MUSCL stencil (`wall_ghost3`)
//  4. res = -(1/V) sum S (F - Fv);  the time step adds the diffusive 2 lv A^2 / V of each direction
//  5. the implicit scheme (`implicit3.rs`: 2 lam_f I on every face dissipation, the no-slip wall Jacobian), then the turbulence update (SST)


/// The SST model of a viscous run: the model, the host-built geometry of its kernels, the wall distance (uploaded once) and the roughness per cell.
pub(crate) struct TurbRun3 {
    re_c: f64,
    model: Sst3,
    tg: TGeo3,
    dwall: A,
    ks: Option<A>,
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
    /// The wall state `[k_w, omega_w, mu_t,w]` of the wall cells (SST).
    wall: Option<[A; 3]>,
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
    fn turb_step3(&self, s: &State3, dt: &A, last: &Resid3) -> [A; 3] {
        let (Some(t), Some(vg), Some(vis)) = (&self.turb, &self.visc, &last.vis) else { return turb_pass(s) };
        let wall = vis.wall.clone().expect("an SST evaluation computes the wall state");
        let dev = TDev3 {
            rho: last.p[0].clone(),
            cell: VCell3 { grad: vis.cell.grad.clone(), mu: vis.cell.mu.clone() },
            mu_t: vis.mu_t.clone(),
            wall,
            dwall: t.dwall.clone(),
            mass: std::array::from_fn(|d| last.f[d][0].clone()),
            dt: dt.clone(),
            turb: [s[7].clone(), s[8].clone()],
            speed: self.wall_speed_grid3(&t.tg, &last.p).expect("the wall speed of the wall cells"),
            ks: t.ks.clone(),
            law_strain: true,
            law: None,
        };
        let u = self.turbulence_update3(&t.tg, vg, &t.model, &dev);
        let [k, om] = u.turb;
        let (k, om, mu_t) = self.jet_fix_turb3(k, om, u.mu_t);
        [k, om, mu_t]
    }

    /// The viscous data the exit faces of a motor jet read in the jet cell (`Solver3::jet_viscous_fix`): `mu(T_jet)` is the cell pass's own (the jet cell holds the jet state);
    /// the velocity / temperature gradient of the jet cell is that of the fluid cell behind the exit (`+i`).
    fn jet_fix_cell3(&self, cell: VCell3) -> VCell3 {
        let Some(j) = &self.jet else { return cell };
        let ni = self.ni;
        let grad: Grad12 = std::array::from_fn(|q| {
            let ahead = pad_edge3(&cell.grad[q], 0, 0, 1).slice(0, 1..ni + 1);
            select(&j.behind, ahead, &cell.grad[q])
        });
        VCell3 { grad, mu: cell.mu }
    }

    /// The scalars of the jet cells after a turbulence update (it zeroes the solid cells): `k`, `omega` of the jet and `mu_t = rho_jet k / omega`.
    fn jet_fix_turb3(&self, k: A, om: A, mu_t: A) -> (A, A, A) {
        let Some(j) = &self.jet else { return (k, om, mu_t) };
        let [rho, _, _, _, _] = j.state.state;
        let [kj, oj] = j.state.turb;
        (select(&j.mask, c(kj), &k), select(&j.mask, c(oj), &om), select(&j.mask, c(rho * kj / oj), &mu_t))
    }

    /// Sets the jet of the run to `jet` (the jet cells must exist: the solver was built from a host with a jet): the jet cells of the loop state take its state, `k`, `omega`, `mu_t`.
    pub fn set_jet(&mut self, jet: crate::solver3::JetState3) -> Result<()> {
        self.compiled = None;
        let Some(j) = &mut self.jet else {
            return Err(luma::LumaError::Unsupported("the solver was built without a motor jet (build it from a host with Solver3::set_jet)".into()));
        };
        j.state = jet;
        let (mask, [rho, kj, oj]) = (j.mask.clone(), [jet.state[0], jet.turb[0], jet.turb[1]]);
        let q = crate::gas3::prim_to_cons3(&jet.state);
        if let Some(s) = &mut self.state {
            for (k, qk) in q.iter().enumerate() {
                s[k] = select(&mask, c(*qk), &s[k]);
            }
            if self.turb.is_some() {
                s[7] = select(&mask, c(kj), &s[7]);
                s[8] = select(&mask, c(oj), &s[8]);
                s[9] = select(&mask, c(rho * kj / oj), &s[9]);
            }
        }
        Ok(())
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

    /// Are the viscous terms on?
    pub fn has_viscous(&self) -> bool {
        self.visc.is_some()
    }

    /// The eddy viscosity of the current loop state (`None` without SST).
    fn state_mu_t(&self) -> Option<A> {
        self.turb.as_ref()?;
        self.state.as_ref().map(|s| s[9].clone())
    }

    /// Switches the viscous terms (and with `Turbulence::Sst` the SST model) on from a host solver that has them (`Solver3::set_viscous`): the geometry of the
    /// face passes (`viscous3::face_dirs`, the symmetry sides derived from its flags), the outflow back pressure and, for SST, the free-stream turbulence state.
    pub fn set_viscous_from_host(&mut self, s: &Solver3) -> Result<()> {
        self.compiled = None;
        let Some(r) = s.rans() else {
            self.visc = None;
            self.turb = None;
            self.p_out = None;
            self.wall_j0.clear();
            self.wall_j0_cols = None;
            self.swirl = None;
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
        let vg = VGeo3::new(&self.ctx, g, &self.solid_host, &sym, r.visc)?;
        self.p_out = r.p_outflow;
        self.turb = if r.visc.turbulence == Turbulence::Sst {
            let shape = [g.ni, g.nj, g.nk];
            Some(TurbRun3 {
                re_c: r.visc.re_c,
                model: Sst3 { re_c: r.visc.re_c },
                tg: TGeo3::new(&self.ctx, g, &self.solid_host, &sym, &r.bc, &vg, r.law_i_walls)?,
                dwall: up(&self.ctx, &r.dwall.iter().map(|d| d.min(1e10)).collect::<Vec<_>>(), shape)?,
                ks: if r.ks.is_empty() { None } else { Some(up(&self.ctx, &r.ks, shape)?) },
            })
        } else {
            None
        };
        self.wall_j0 = g.wall_j0.iter().map(|(r, _)| r.clone()).collect();
        self.wall_j0_cols = {
            let cols: Vec<bool> = (0..g.nk).flat_map(|_| (0..g.ni).map(|i| self.wall_j0.iter().any(|r| r.contains(&i)) && !g.is_axis(i, 0))).collect();
            if cols.iter().any(|&b| b) {
                Some(Array::from_slice_on(&self.ctx, &cols, [g.ni, 1, g.nk])?)
            } else {
                None
            }
        };
        self.swirl = s.is_wedge().then(|| (0.5 * g.theta_span).sin_cos());
        self.visc = Some(vg);
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
        let cell = self.jet_fix_cell3(self.viscous_cells3(vg, &p, &w));
        let mu_t = mu_t.cloned().unwrap_or_else(|| &cell.mu * c(0.0));
        // SST: the wall state of the cells with a no-slip face (the automatic treatment), its eddy viscosity `mu_t,w` the face value of the wall faces
        let wall = self.turb.as_ref().map(|t| self.wall_state_pass3(&t.tg, &p, &cell.mu, &t.dwall, t.ks.as_ref()).expect("the wall state arrays of the grid"));
        let fv = self.viscous_fluxes3(vg, &w, &cell, &mu_t, wall.as_ref().map(|w| &w[2]));
        let total: [Rec3; 3] = std::array::from_fn(|d| Self::total_flux3(&f[d], &fv[d]));
        let diff = compute_dt.then(|| self.diffusion_rates3(&cell, &mu_t, &p[0]));
        let (mut res, dt) = self.cell_residual3(&p, &total, compute_dt, diff.as_ref());
        // a viscous wedge: the azimuthal momentum of the residual is projected out (the mirror ghost of the symmetry planes damps a swirl at an explicit rate no CFL
        // survives): `t = res_v cos - res_w sin`, `res_v -= t cos`, `res_w += t sin` with `th = theta_span / 2` (`Solver3::add_viscous_residual`)
        if let Some((sn, cs)) = self.swirl {
            let t = &res[2] * c(cs) - &res[3] * c(sn);
            res[2] = &res[2] - &t * c(cs);
            res[3] = &res[3] + &t * c(sn);
        }
        Resid3 { res, dt, p, w, f, vis: Some(VisAux { cell, fv, mu_t, wall }) }
    }

    /// The viscous face fluxes `Fv.n` (mass component 0) of the three families at the current state, on the host (the eddy viscosity and wall state of the SST run included):
    /// the input of `viscous3::wall_tractions` (skin friction, `plate3::station`).
    pub fn viscous_flux_f64(&self) -> Result<[Vec<[f64; 5]>; 3]> {
        let r = self.residual_at_state3(false)?;
        let vis = r.vis.ok_or_else(|| luma::LumaError::Unsupported("viscous terms are not set".into()))?;
        let rows = |f: &Vec4| -> Result<Vec<[f64; 5]>> {
            let cols: Vec<Vec<f64>> = f.iter().map(to_f64).collect::<Result<_>>()?;
            Ok((0..cols[0].len()).map(|q| [0.0, cols[0][q], cols[1][q], cols[2][q], cols[3][q]]).collect())
        };
        Ok([rows(&vis.fv[0])?, rows(&vis.fv[1])?, rows(&vis.fv[2])?])
    }

    /// The viscous wall force by class `[Fx, Fz, Mz]` of the current state, reduced on the device (needs [`LazySolver3::set_traction_faces`]).
    pub fn viscous_wall_forces(&self) -> Result<Vec<[f64; 3]>> {
        let vis = self.residual_at_state3(false)?.vis.ok_or_else(|| luma::LumaError::Unsupported("viscous terms are not set".into()))?;
        self.wall_tractions3(&vis.fv)
    }

    /// One residual evaluation of the current loop state (its eddy viscosity included).
    fn residual_at_state3(&self, compute_dt: bool) -> Result<Resid3> {
        let u = unpack3(self.state.as_ref().ok_or_else(no_state)?);
        let mu_t = self.state_mu_t();
        Ok(self.residual3(&u, mu_t.as_ref(), self.limiter3(self.iter), compute_dt))
    }
}
