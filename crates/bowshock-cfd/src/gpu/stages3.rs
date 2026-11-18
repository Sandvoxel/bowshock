// GPU time per stage of one implicit (`LineSolve3::Block`) step of the 3D solver: `LazySolver3::profile_stages3`. `include!`d twice by `gpu/mod.rs` after `rans3.rs`.
//
// The step is one lazy graph that the loop replays; the profile of a step (`profile_step`) lists its kernels by the expression they fuse, which says little about the
// physics. Here the stages of `implicit_step3` are evaluated one after the other, each under `Context::profile` (a warm run first, then the best of `reps`), so every
// launch belongs to a named stage. The sum of the stages is more than the real step: a stage boundary is a materialisation the fused step does not always have
// (`Staging::Fluxes`: the inviscid fluxes, the primitives and `P` are cached in the real step too; the f32 factor blocks are lazy and live inside the line solves, so
// "block assembly" is the face matrices only and the rest of the assembly is inside the solve rows). Intended for the benchmark, not for the loop.

/// An array to evaluate at the end of a stage: of the precision `S` or an f32 left-hand-side array.
enum Ev {
    S(A),
    L(F),
}

fn evs_s(a: &[A]) -> Vec<Ev> {
    a.iter().cloned().map(Ev::S).collect()
}

fn evs_l(a: &[F]) -> Vec<Ev> {
    a.iter().cloned().map(Ev::L).collect()
}

fn eval_evs(v: &[Ev]) -> Result<()> {
    let s: Vec<&A> = v.iter().filter_map(|e| if let Ev::S(a) = e { Some(a) } else { None }).collect();
    let l: Vec<&F> = v.iter().filter_map(|e| if let Ev::L(a) = e { Some(a) } else { None }).collect();
    if !s.is_empty() {
        luma::eval_all(&s)?;
    }
    if !l.is_empty() {
        luma::eval_all(&l)?;
    }
    Ok(())
}

/// Runs `$body` (an expression of type `(Vec<Ev>, T)` that may use `?`) as a stage: once to compile, then `$reps` times under the profiler (the fastest kept, pushed to
/// `$rows` under `$name`); gives the value of the last run (its arrays are evaluated).
macro_rules! stage {
    ($self:expr, $rows:expr, $reps:expr, $name:expr, $body:expr) => {{
        let run = || -> Result<_> {
            let (ev, val) = $body;
            eval_evs(&ev)?;
            $self.ctx.sync()?;
            Ok(val)
        };
        run()?;
        let mut best: Option<luma::Profile> = None;
        let mut val = None;
        for _ in 0..$reps {
            let (r, profile) = $self.ctx.profile(|| run())?;
            val = Some(r?);
            if best.as_ref().is_none_or(|b| profile.total() < b.total()) {
                best = Some(profile);
            }
        }
        $rows.push(($name.to_string(), best.expect("at least one repetition")));
        val.expect("at least one repetition")
    }};
}

impl LazySolver3 {
    /// The stages of one implicit block step from the current state and their GPU profiles (best of `reps`): inviscid fluxes, the viscous cell pass, the wall state, the viscous
    /// face fluxes, the residual, the face lambdas, the f32 primitives / `P` / time step, the face matrices of every factor, every line solve of every sweep, every defect
    /// residual, the update, and the SST update (law of the wall, gradients, blends, face data, cell terms, solves). The state is not changed.
    pub fn profile_stages3(&mut self, reps: usize) -> Result<Vec<(String, luma::Profile)>> {
        let Scheme3::Implicit(p) = self.cfg.scheme else {
            return Err(luma::LumaError::Unsupported("profile_stages3 profiles the implicit scheme".into()));
        };
        if p.line_solve != crate::implicit_params::LineSolve3::Block {
            return Err(luma::LumaError::Unsupported("profile_stages3 profiles the block line solve".into()));
        }
        let reps = reps.max(1);
        let lim = self.limiter3(self.iter);
        let s = self.state.clone().ok_or_else(no_state)?;
        self.ctx.sync()?;
        let mut rows: Vec<(String, luma::Profile)> = Vec::new();
        let u = unpack3(&s);
        let mu_t_state = self.turb.as_ref().map(|_| s[9].clone());

        // 1. inviscid
        let (prim, w, f) = stage!(self, rows, reps, "inviscid: primitives, ghosts, face fluxes", {
            let (prim, w) = self.padded3(&u);
            let f = self.face_fluxes3(&w, lim);
            let mut ev = evs_s(&prim);
            ev.extend(evs_s(&w));
            for d in 0..3 {
                ev.extend(evs_s(&f[d]));
            }
            (ev, (prim, w, f))
        });

        // 2. viscous
        let mut vis: Option<(VCell3, A, Option<[A; 3]>)> = None;
        let res: Rec3 = if let Some(vg) = &self.visc {
            let cell = stage!(self, rows, reps, "viscous: cell pass (gradients, mu)", {
                let cell = self.jet_fix_cell3(self.viscous_cells3(vg, &prim, &w));
                let mut ev = evs_s(&cell.grad);
                ev.push(Ev::S(cell.mu.clone()));
                (ev, cell)
            });
            let mu_t = mu_t_state.clone().unwrap_or_else(|| &cell.mu * c(0.0));
            let wall = match &self.turb {
                Some(t) => Some(stage!(self, rows, reps, "wall state (automatic wall treatment)", {
                    let wl = self.wall_state_pass3(&t.tg, &prim, &cell.mu, &t.dwall, t.ks.as_ref())?;
                    (evs_s(&wl), wl)
                })),
                None => None,
            };
            let fv = stage!(self, rows, reps, "viscous: face fluxes", {
                let fv = self.viscous_fluxes3(vg, &w, &cell, &mu_t, wall.as_ref().map(|w| &w[2]));
                let ev: Vec<Ev> = fv.iter().flat_map(|d| evs_s(d)).collect();
                (ev, fv)
            });
            let res = stage!(self, rows, reps, "residual: total flux, cell residual, swirl, norm", {
                let total: [Rec3; 3] = std::array::from_fn(|d| Self::total_flux3(&f[d], &fv[d]));
                let (mut res, _) = self.cell_residual3(&prim, &total, false, None);
                if let Some((sn, cs)) = self.swirl {
                    let t = &res[2] * c(cs) - &res[3] * c(sn);
                    res[2] = &res[2] - &t * c(cs);
                    res[3] = &res[3] + &t * c(sn);
                }
                let mut ev = evs_s(&res);
                ev.push(Ev::S(self.residual_norm3(&res[0])));
                (ev, res)
            });
            vis = Some((cell, mu_t, wall));
            res
        } else {
            stage!(self, rows, reps, "residual: cell residual, norm", {
                let (res, _) = self.cell_residual3(&prim, &f, false, None);
                let mut ev = evs_s(&res);
                ev.push(Ev::S(self.residual_norm3(&res[0])));
                (ev, res)
            })
        };

        // 3. left-hand side
        let (diff, lam): (Option<[F; 3]>, Option<[F; 3]>) = match (&self.visc, &vis) {
            (Some(vg), Some((cell, mu_t, wall))) => stage!(self, rows, reps, "face lambdas + diffusion rates", {
                let d = self.diffusion_rates3(cell, mu_t, &prim[0]);
                let lam = self.lambda_pass3(vg, &w, cell, mu_t, wall.as_ref().map(|w| &w[2]));
                let d: [F; 3] = std::array::from_fn(|k| lhs(&d[k]).cache());
                let lam: [F; 3] = std::array::from_fn(|k| lhs(&lam[k]).cache());
                let mut ev = evs_l(&d);
                ev.extend(evs_l(&lam));
                (ev, (Some(d), Some(lam)))
            }),
            _ => (None, None),
        };
        let (wf, pc, dt) = stage!(self, rows, reps, "lhs: f32 primitives, P matrices, pseudo time step", {
            let wf: V5 = Self::lhs_rec3(&w).cache();
            let wk: V5 = v5(|k| wf[k].slice(0, 2..self.ni + 2).slice(1, 2..self.nj + 2).slice(2, 2..self.nk + 2));
            let pc: Option<Pc3> = LmGpu3::of(&p, &self.cfg.freestream).map(|lm| Pc3 { p: self.p_cells3(&wk, &lm), lm });
            let cfl = lhs(&s[6]);
            let dt = self.pseudo_dt3(&wk, &cfl, p.dt_mode, diff.as_ref(), pc.as_ref().map(|c| &c.lm)).cache();
            let mut ev = evs_l(&wf);
            ev.push(Ev::L(dt.clone()));
            if let Some(pc) = &pc {
                ev.extend(evs_l(&pc.p));
            }
            (ev, (wf, pc, dt))
        });
        let wk: V5 = v5(|k| wf[k].slice(0, 2..self.ni + 2).slice(1, 2..self.nj + 2).slice(2, 2..self.nk + 2));
        let nd = self.n_dirs3();
        let fac: Vec<Factor3> = stage!(self, rows, reps, "block assembly: face matrices of every factor (A + D, dissipation)", {
            let fac: Vec<Factor3> = (0..nd).map(|dir| self.factor3(&wf, &wk, &dt, &p, dir, lam.as_ref(), pc.as_ref())).collect();
            let mut ev = Vec::new();
            for f in &fac {
                ev.extend(evs_l(&f.fl));
                ev.extend(evs_l(&f.fr));
            }
            (ev, fac)
        });
        let b: V5 = v5(|k| &lhs(&res[k]) * &dt);
        let pm = pc.as_ref().map(|c| &c.p);
        let apply_p = |x: &V5| -> V5 { pm.map_or_else(|| x.clone(), |m| mv5(m, x)) };

        // 4. the sweeps and the defect corrections (`implicit_step3`, Block)
        let sweep = |this: &LazySolver3, rows: &mut Vec<(String, luma::Profile)>, label: &str, rhs: &V5| -> Result<(V5, V5, V5)> {
            let x = stage!(this, rows, reps, format!("line solve i ({label})"), {
                let x = this.solve_lines3(&fac[0], 0, rhs);
                (evs_l(&x), x)
            });
            let y = stage!(this, rows, reps, format!("line solve j ({label})"), {
                let y = this.solve_lines3(&fac[1], 1, &apply_p(&x));
                (evs_l(&y), y)
            });
            let du = if nd == 3 {
                stage!(this, rows, reps, format!("line solve k ({label})"), {
                    let du = this.solve_lines3(&fac[2], 2, &apply_p(&y));
                    (evs_l(&du), du)
                })
            } else {
                y.clone()
            };
            Ok((x, y, du))
        };
        let (x, y, mut du) = sweep(self, &mut rows, "sweep 1", &b)?;
        let mut passes = 1;
        if p.lines && p.sub_iters >= 2 {
            let r: V5 = stage!(self, rows, reps, "defect 1: N_i (X - dU) + N_j (Y - dU)", {
                let ni_ = self.n_apply3(&fac[0], 0, &v5(|k| &x[k] - &du[k]), &p);
                let r: V5 = if nd == 3 {
                    let nj_ = self.n_apply3(&fac[1], 1, &v5(|k| &y[k] - &du[k]), &p);
                    v5(|k| &ni_[k] + &nj_[k])
                } else {
                    ni_
                };
                let r = r.cache();
                (evs_l(&r), r)
            });
            let (_, _, corr) = sweep(self, &mut rows, "sweep 2", &r)?;
            du = stage!(self, rows, reps, "du += correction", {
                let d: V5 = v5(|k| &du[k] + &corr[k]);
                let d: V5 = d.cache();
                (evs_l(&d), d)
            });
            passes = 2;
        }
        for pass in passes..p.sub_iters.max(1) {
            let r: V5 = stage!(self, rows, reps, format!("defect {pass}: b - (sum A_d - (nd - 1) P) dU"), {
                let a: Vec<V5> = (0..nd).map(|dir| self.apply_factor3(&fac[dir], dir, &du)).collect();
                let pdu: Option<V5> = pm.map(|m| mv5(m, &du).cache());
                let r: V5 = v5(|k| {
                    let pk = pdu.as_ref().map_or_else(|| du[k].clone(), |p| p[k].clone());
                    &b[k] - (a.iter().fold(&pk * -((nd - 1) as f32), |acc, x| acc + &x[k]))
                });
                let r = r.cache();
                (evs_l(&r), r)
            });
            let (_, _, corr) = sweep(self, &mut rows, &format!("sweep {}", pass + 1), &r)?;
            du = stage!(self, rows, reps, format!("du += correction {}", pass + 1), {
                let d: V5 = v5(|k| &du[k] + &corr[k]);
                let d: V5 = d.cache();
                (evs_l(&d), d)
            });
        }

        // 5. update
        stage!(self, rows, reps, "update: positivity damping, state", {
            let alpha = self.damping3(&Self::lhs_rec3(&u), &du);
            let moved = match &self.geo.fluid {
                Some(fluid) => alpha.gt(0.0f32) & fluid,
                None => alpha.gt(0.0f32),
            };
            let alpha = from_lhs(&alpha);
            let un: Rec3 = rec3(|k| select(&moved, &u[k] + &alpha * from_lhs(&du[k]), &u[k]));
            (evs_s(&un), un)
        });

        // 6. the SST update
        if let (Some(t), Some(vg), Some((cell, mu_t, wall))) = (&self.turb, &self.visc, &vis) {
            let wall = wall.clone().expect("an SST evaluation computes the wall state");
            let dev = TDev3 {
                rho: prim[0].clone(),
                pres: prim[4].clone(),
                cell: VCell3 { grad: cell.grad.clone(), mu: cell.mu.clone() },
                mu_t: mu_t.clone(),
                wall,
                dwall: t.dwall.clone(),
                mass: std::array::from_fn(|d| f[d][0].clone()),
                dt: from_lhs(&dt),
                turb: [s[7].clone(), s[8].clone()],
                speed: self.wall_speed_grid3(&t.tg, &prim)?,
                ks: t.ks.clone(),
                law_strain: true,
                law: None,
            };
            let dev = stage!(self, rows, reps, "turbulence: law of the wall", {
                let dev = self.with_law3(&t.tg, &dev);
                let ev = match &dev.law {
                    Some(l) => vec![Ev::S(l.smooth.clone()), Ev::S(l.s2.clone()), Ev::S(l.yp.clone()), Ev::S(l.u_tau.clone()), Ev::S(l.nu.clone())],
                    None => Vec::new(),
                };
                (ev, dev)
            });
            let grad = stage!(self, rows, reps, "turbulence: scalar gradients", {
                let g = self.scalar_gradients3(&t.tg, vg, &t.model, &dev);
                let ev: Vec<Ev> = g.iter().flat_map(|e| evs_s(e)).collect();
                (ev, g)
            });
            let blend = stage!(self, rows, reps, "turbulence: blends (F1, X)", {
                let b = self.blends3(&t.tg, &t.model, &dev, &grad);
                (vec![Ev::S(b.f1.clone()), Ev::S(b.x.clone())], b)
            });
            let tf = stage!(self, rows, reps, "turbulence: face data (convection, diffusion)", {
                let tf = self.face_data3(&t.tg, vg, &t.model, &dev, &grad, &blend);
                let mut ev = Vec::new();
                for f in &tf {
                    ev.push(Ev::S(f.m.clone()));
                    for e in 0..2 {
                        ev.extend([Ev::S(f.g[e].clone()), Ev::S(f.flux[e].clone()), Ev::S(f.l[e].clone()), Ev::S(f.r[e].clone())]);
                    }
                }
                (ev, tf)
            });
            let ct = stage!(self, rows, reps, "turbulence: cell terms (residual, line coefficients)", {
                let ct = self.cell_terms3(&t.tg, &t.model, &dev, &grad, &blend, &tf);
                let mut ev = Vec::new();
                for e in 0..2 {
                    ev.push(Ev::S(ct.r[e].clone()));
                    ev.push(Ev::S(ct.d0[e].clone()));
                    for d in 0..3 {
                        ev.extend([Ev::S(ct.lo[d][e].clone()), Ev::S(ct.dd[d][e].clone()), Ev::S(ct.up[d][e].clone())]);
                    }
                }
                (ev, ct)
            });
            stage!(self, rows, reps, "turbulence: line solves (k, omega) and update", {
                let up = self.solve_and_update3(&t.tg, &t.model, &dev, &grad, &ct);
                let mut ev = evs_s(&up.turb);
                ev.push(Ev::S(up.mu_t.clone()));
                ev.extend(evs_s(&up.rnorm));
                (ev, ())
            });
        }
        Ok(rows)
    }
}
