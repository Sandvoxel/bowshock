// The Menter SST k-omega model (2003) in array ops. This file is `include!`d by `lib.rs` after `viscous.rs` into the same per-precision
// module. It mirrors `bowshock_core::turbulence` (`scalar_gradients`, `blends`, `face_pass`, `cell_terms`, `update`) kernel for kernel:
//
//     F1  = tanh(a^4),  a = min( max( sqrt(k) / (beta* omega d), 500 nu / (d^2 omega) ), 4 rho sigma_w2 k / (CD d^2) )
//     F2  = tanh(b^2),  b = max( 2 sqrt(k) / (beta* omega d), 500 nu / (d^2 omega) )
//     mu_t = min( rho a1 k / max(a1 omega, S F2), 1e5 / re_c ),            P = min( mu_t S^2, 10 beta* rho k omega )
//     rho dk/dt     = (-conv_k + diff_k) / V + P - beta* rho omega k
//     rho domega/dt = (-conv_w + diff_w) / V + gamma rho P / max(mu_t, 1e-30) - beta rho omega^2 + (1 - F1) X
//
// (the constants and the boundary states of the two scalars are documented there). The two scalars are advanced once per iteration, after the
// mean-flow update, from the quantities of the start of the iteration: a loosely coupled pseudo-time step with the line-implicit solves
// `(diag0 + dd_i) x + lo_i x_- + up_i x_+ = V R` (i lines) and `(1 + dd_j / diag0) dphi + (lo_j / diag0) dphi_- + (up_j / diag0) dphi_+ = x`
// (j lines), each scalar with Luma's scalar tridiagonal solver in f32 (the CPU solves both with a 4x4 block solver whose blocks are diagonal:
// the same systems). `tanh` of the blending functions is evaluated in f32 in both precisions.
//
// `Stage::mass` is `F[0]` of the face fluxes, `dt` the time step of the iteration (the pseudo time step of the implicit scheme, in the
// element type).

use bowshock_core::turbulence::{A1, BETA1, BETA2, BETA_STAR, GAMMA1, GAMMA2, SIGMA_K1, SIGMA_K2, SIGMA_W1, SIGMA_W2};

/// `S^2 = 2 (u_x^2 + v_r^2 + (v/r)^2) + (u_r + v_x)^2 - 2/3 div^2` of a cell from its gradient `g = (u_x, u_r, v_x, v_r, ..)`, radial
/// velocity `v` and radius `r`.
fn strain2(g: &[A], v: &A, r: &A) -> A {
    let vr = v / r;
    let div = &g[0] + &g[3] + &vr;
    let shear = &g[1] + &g[2];
    c(2.0) * (&g[0] * &g[0] + &g[3] * &g[3] + &vr * &vr) + &shear * &shear - c(2.0 / 3.0) * &div * &div
}

/// `(sqrt(k) / (beta* omega d), 500 nu / (d^2 omega))`, the pieces of the arguments of the blending functions.
fn blend_args(nu: &A, k: &A, om: &A, d: &A) -> (A, A) {
    (k.sqrt() / (c(BETA_STAR) * om * d), c(500.0) * nu / (d * d * om))
}

/// `F2 = tanh(b^2)`, `b = max(2 sqrt(k) / (beta* omega d), 500 nu / (d^2 omega))`.
fn f2_of(nu: &A, k: &A, om: &A, d: &A) -> A {
    let (t1, t2) = blend_args(nu, k, om, d);
    let b = (c(2.0) * t1).maximum(t2);
    tanh(&(&b * &b))
}

/// Eddy viscosity `min(rho a1 k / max(a1 omega, S F2), 1e5 / re_c)`.
fn eddy_viscosity(rho: &A, k: &A, om: &A, s: &A, f2: &A, re_c: f64) -> A {
    (rho * c(A1) * k / (c(A1) * om).maximum(s * f2)).minimum(c(1e5 / re_c))
}

/// Does the boundary condition give the free-stream values of `k`, `omega` to a ghost?
fn bc_inf(bc: Bc) -> bool {
    matches!(bc, Bc::Freestream | Bc::Characteristic | Bc::PressureOutflow)
}

impl LazySolver {
    /// Green-Gauss gradient `g = (1/A) sum_faces s phi_f n len`, `phi_f = (phi_c + phi_neighbour) / 2`, of the `M` fields `pc` of every cell
    /// from the neighbour values `nb` across the right, left, top and bottom face; the `2 M` components are ordered `(x, r)` per field.
    fn green_gauss<const M: usize>(&self, pc: &[A; M], nb: &[[A; M]; 4]) -> Vec<A> {
        let (ni, nj) = (self.ni, self.nj);
        let g = &self.geo;
        let inv_a = &self.visc.as_ref().expect("viscous run").inv_a;
        let faces = [
            (g.inx.slice(0, 1..ni + 1), g.inr.slice(0, 1..ni + 1), g.ilen.slice(0, 1..ni + 1), true),
            (g.inx.slice(0, 0..ni), g.inr.slice(0, 0..ni), g.ilen.slice(0, 0..ni), false),
            (g.jnx.slice(1, 1..nj + 1), g.jnr.slice(1, 1..nj + 1), g.jlen.slice(1, 1..nj + 1), true),
            (g.jnx.slice(1, 0..nj), g.jnr.slice(1, 0..nj), g.jlen.slice(1, 0..nj), false),
        ];
        let mut out = Vec::with_capacity(2 * M);
        for m in 0..M {
            for comp in 0..2 {
                let mut acc: Option<A> = None;
                for (f, (nx, nr, len, plus)) in faces.iter().enumerate() {
                    let pf = c(0.5) * (&pc[m] + &nb[f][m]);
                    let n = if comp == 0 { nx } else { nr };
                    let term = &pf * n * len;
                    acc = Some(match (acc, plus) {
                        (None, true) => term,
                        (None, false) => -term,
                        (Some(a), true) => a + term,
                        (Some(a), false) => a - term,
                    });
                }
                out.push(acc.expect("four faces") * inv_a);
            }
        }
        out
    }

    /// `[k, omega]` on the two sides of every face of direction `dir`: the cells (`cell_sides`), the free-stream values `(k_inf, omega_inf)` in the
    /// ghost of a free-stream, characteristic or pressure-outflow boundary, and on a no-slip wall the ghost value `wall` (`[-k, 2 omega_w - omega]`
    /// of the cell, `omega_w = 60 nu / (beta1 d^2)`).
    fn kw_sides(&self, t: &[A; 2], wall: &[A; 2], dir: usize) -> ([A; 2], [A; 2]) {
        let (ni, nj) = (self.ni, self.nj);
        let vg = self.visc.as_ref().expect("viscous run");
        let inf = [K_INF, omega_inf(vg.v.re_c)];
        let bc = self.cfg.bc;
        let (mut l, mut r) = self.cell_sides(t, dir);
        for e in 0..2 {
            if dir == 0 {
                if bc_inf(bc[0]) {
                    l[e].slice_mut(0, 0..1).assign(c(inf[e])).expect("free-stream side");
                }
                if bc_inf(bc[1]) {
                    r[e].slice_mut(0, ni..ni + 1).assign(c(inf[e])).expect("free-stream side");
                }
            } else {
                if bc_inf(bc[2]) {
                    l[e].slice_mut(1, 0..1).assign(c(inf[e])).expect("free-stream side");
                }
                if bc_inf(bc[3]) {
                    r[e].slice_mut(1, nj..nj + 1).assign(c(inf[e])).expect("free-stream side");
                }
            }
        }
        // no-slip faces (`Walls`): the ghost below / behind the face is the wall value of the cell above / in front of it
        let mut walls: Vec<(std::ops::Range<usize>, std::ops::Range<usize>)> = vec![];
        let wis = vg.v.wall_i_start;
        match (self.block, dir) {
            (Some(b), 0) => walls.push((b.i_end..b.i_end + 1, b.j_sting..b.j_end)),
            (Some(b), _) => {
                walls.push((wis.min(b.i_end)..b.i_end, b.j_end..b.j_end + 1));
                if b.j_sting > 0 {
                    walls.push((b.i_end..ni, b.j_sting..b.j_sting + 1));
                }
            }
            (None, 1) if bc[2] == Bc::SlipWall => walls.push((wis.min(ni)..ni, 0..1)),
            (None, _) => {}
        }
        for (ri, rj) in walls.into_iter().filter(|(ri, rj)| !ri.is_empty() && !rj.is_empty()) {
            for e in 0..2 {
                let src = wall[e].slice(0, ri.clone()).slice(1, rj.clone());
                l[e].slice_mut(0, ri.clone()).slice(1, rj.clone()).assign(src).expect("no-slip side assign");
            }
        }
        (l, r)
    }

    /// The SST update of the iteration of the state `s` (`turbulence::update`): `[k, omega, mu_t, rn]` of the next loop state, from the
    /// evaluation `st` of the start of the iteration and the time step `dt`; `rn = sqrt(sum (R_k)^2 / N)` with `R_k = rho dk/dt`.
    fn sst_update(&self, s: &State, st: &Stage, dt: &A) -> [A; 4] {
        let vg = self.visc.as_ref().expect("viscous run");
        let vf = st.vf.as_ref().expect("viscous quantities");
        let v = vg.v;
        let (ni, nj) = (self.ni, self.nj);
        let g = &self.geo;
        let d = vg.dwall.as_ref().expect("wall distance");
        let (k, om) = (&s[6], &s[7]);
        let rho = &st.p[0];
        let (mu, mu_t) = (&vf.cell[6], &vf.cell[7]);
        let nu = mu / rho;
        let inf = [K_INF, omega_inf(v.re_c)];

        // kernel 1: Green-Gauss gradient (k_x, k_r, omega_x, omega_r) of every cell from the face values of `k`, `omega`
        let wall = [-k, c(2.0) * (c(60.0) * &nu / (c(BETA1) * d * d)) - om];
        let t = [k.clone(), om.clone()];
        let (kw_i, kw_j) = (self.kw_sides(&t, &wall, 0), self.kw_sides(&t, &wall, 1));
        let nb = [
            [kw_i.1[0].slice(0, 1..ni + 1), kw_i.1[1].slice(0, 1..ni + 1)],
            [kw_i.0[0].slice(0, 0..ni), kw_i.0[1].slice(0, 0..ni)],
            [kw_j.1[0].slice(1, 1..nj + 1), kw_j.1[1].slice(1, 1..nj + 1)],
            [kw_j.0[0].slice(1, 0..nj), kw_j.0[1].slice(1, 0..nj)],
        ];
        let tg = self.green_gauss(&t, &nb);

        // kernel 2: F1 and the cross-diffusion X of every cell
        let x = c(2.0) * rho * c(SIGMA_W2) * (&tg[0] * &tg[2] + &tg[1] * &tg[3]) / om;
        let cd = x.maximum(c(1e-20));
        let (t1, t2) = blend_args(&nu, k, om, d);
        let t3 = c(4.0) * rho * c(SIGMA_W2) * k / (&cd * d * d);
        // (`a` is bounded by `1e3` only to keep `a^4` finite in f32: `tanh` is 1 there)
        let a = t1.maximum(t2).minimum(t3).minimum(c(1e3));
        let a2 = &a * &a;
        let f1 = tanh(&(&a2 * &a2));
        let [tg0, tg1, tg2, tg3, f1, x] = staged(self.staging, [tg[0].clone(), tg[1].clone(), tg[2].clone(), tg[3].clone(), f1, x]);
        let cells = [tg0, tg1, tg2, tg3, f1.clone(), mu.clone(), mu_t.clone()];

        // kernel 3: scalar face data (mass flow, conductances, diffusive fluxes, the two sides) of every face
        let faces: [[A; 9]; 2] = std::array::from_fn(|dir| {
            let dg = &vg.dir[dir];
            let (nx, nr, w) = if dir == 0 { (&g.inx, &g.inr, &g.iw) } else { (&g.jnx, &g.jnr, &g.jw) };
            let (cl, cr) = self.cell_sides(&cells, dir);
            let (kw_l, kw_r) = if dir == 0 { (&kw_i.0, &kw_i.1) } else { (&kw_j.0, &kw_j.1) };
            let mu_f = c(0.5) * (&cl[5] + &cr[5]);
            let mut_f = select(&dg.noslip, c(0.0), c(0.5) * (&cl[6] + &cr[6]));
            let mut out: Vec<A> = vec![&st.mass[dir] * w];
            let (mut cond, mut flux) = (vec![], vec![]);
            for e in 0..2 {
                let (s1, s2) = if e == 0 { (SIGMA_K1, SIGMA_K2) } else { (SIGMA_W1, SIGMA_W2) };
                let sigma = c(0.5) * (&cl[4] * c(s1) + (c(1.0) - &cl[4]) * c(s2) + &cr[4] * c(s1) + (c(1.0) - &cr[4]) * c(s2));
                let coef = &mu_f + sigma * &mut_f;
                let (gbx, gbr) = (c(0.5) * (&cl[2 * e] + &cr[2 * e]), c(0.5) * (&cl[2 * e + 1] + &cr[2 * e + 1]));
                let corr = (&kw_r[e] - &kw_l[e]) * &dg.inv_d - (&gbx * &dg.ex + &gbr * &dg.er);
                let gn = (&gbx + &corr * &dg.ex) * nx + (&gbr + &corr * &dg.er) * nr;
                flux.push(&coef * &gn * w);
                cond.push(&coef * w * &dg.inv_d);
            }
            out.extend(cond);
            out.extend(flux);
            out.extend([kw_l[0].clone(), kw_l[1].clone(), kw_r[0].clone(), kw_r[1].clone()]);
            let arr: [A; 9] = out.try_into().unwrap_or_else(|_| unreachable!("nine face arrays"));
            staged(self.staging, arr)
        });

        // kernel 4: residual `R = rho dphi/dt` and the line coefficients of every cell
        let phi = [k.clone(), om.clone()];
        let s2 = strain2(&vf.cell[0..6], &st.p[2], &vg.rc);
        let prod = (mu_t * &s2).minimum(c(10.0 * BETA_STAR) * rho * k * om);
        let beta = &f1 * c(BETA1) + (c(1.0) - &f1) * c(BETA2);
        let gamma = &f1 * c(GAMMA1) + (c(1.0) - &f1) * c(GAMMA2);
        let sources = [&prod - c(BETA_STAR) * rho * om * k, &gamma * rho * &prod / mu_t.maximum(c(1e-30)) - &beta * rho * om * om + (c(1.0) - &f1) * &x];
        let dest = [c(BETA_STAR) * om, c(2.0) * &beta * om];
        let dt = match &self.lb {
            Some(lb) => select(&lb.solid, c(1.0), dt),
            None => dt.clone(),
        };
        let rho_vol = rho * &vg.vol;
        let mut rate: [Option<A>; 2] = [None, None];
        // `[direction][scalar]`
        let (mut lo, mut up, mut dd): ([[Option<A>; 2]; 2], [[Option<A>; 2]; 2], [[Option<A>; 2]; 2]) = Default::default();
        for dir in 0..2 {
            let n = self.line_len(dir);
            let f = &faces[dir];
            let a_lo = f[0].slice(dir, 0..n).maximum(c(0.0));
            let a_up = (-f[0].slice(dir, 1..n + 1)).maximum(c(0.0));
            for e in 0..2 {
                let (g_lo, g_up) = (f[1 + e].slice(dir, 0..n), f[1 + e].slice(dir, 1..n + 1));
                // -conv + diff of this direction
                let term = &a_lo * (f[5 + e].slice(dir, 0..n) - &phi[e]) + &a_up * (f[7 + e].slice(dir, 1..n + 1) - &phi[e]) + f[3 + e].slice(dir, 1..n + 1)
                    - f[3 + e].slice(dir, 0..n);
                rate[e] = Some(match rate[e].take() {
                    None => term,
                    Some(r) => r + term,
                });
                lo[dir][e] = Some(-(&a_lo + &g_lo));
                up[dir][e] = Some(-(&a_up + &g_up));
                dd[dir][e] = Some(&a_lo + &a_up + &g_lo + &g_up);
            }
        }
        let r: [A; 2] = std::array::from_fn(|e| rate[e].take().expect("both directions") * &g.inv_v + &sources[e]);
        let d0: [A; 2] = std::array::from_fn(|e| &rho_vol / &dt + &rho_vol * &dest[e]);

        // the two line solves, per scalar, in f32 (the left-hand side): i lines, then j lines
        let solve = |dir: usize, lo: &F, diag: &F, up: &F, rhs: &F| -> F {
            match &self.lb {
                Some(lb) => luma::solve_tridiagonal_from(dir, lo, diag, up, rhs, &lb.start[dir]),
                None => luma::solve_tridiagonal(dir, lo, diag, up, rhs),
            }
        };
        let ends = |dir: usize| match &self.lb {
            Some(lb) => (&lb.first[dir], &self.lg.dir[dir].last),
            None => (&self.lg.dir[dir].first, &self.lg.dir[dir].last),
        };
        let dphi: [A; 2] = std::array::from_fn(|e| {
            let (first, last) = ends(0);
            let rhs = lhs(&(&vg.vol * &r[e]));
            let x = solve(
                0,
                &select(first, 0.0f32, lhs(lo[0][e].as_ref().expect("coefficient"))),
                &lhs(&(&d0[e] + dd[0][e].as_ref().expect("coefficient"))),
                &select(last, 0.0f32, lhs(up[0][e].as_ref().expect("coefficient"))),
                &rhs,
            );
            let (first, last) = ends(1);
            let dphi = solve(
                1,
                &select(first, 0.0f32, lhs(&(lo[1][e].as_ref().expect("coefficient") / &d0[e]))),
                &lhs(&(c(1.0) + dd[1][e].as_ref().expect("coefficient") / &d0[e])),
                &select(last, 0.0f32, lhs(&(up[1][e].as_ref().expect("coefficient") / &d0[e]))),
                &x,
            );
            from_lhs(&dphi)
        });

        // clipped update and eddy viscosity
        let k_new = (k + &dphi[0]).maximum(c(0.1) * k).maximum(c(1e-3 * inf[0]));
        let om_new = (om + &dphi[1]).maximum(c(0.1) * om).maximum(c(1e-3 * inf[1]));
        let strain = s2.maximum(c(0.0)).sqrt();
        let mu_t_new = eddy_viscosity(rho, &k_new, &om_new, &strain, &f2_of(&nu, &k_new, &om_new, d), v.re_c);
        let rk2 = &r[0] * &r[0];
        let (mu_t_new, sum) = match &self.lb {
            Some(lb) => (select(&lb.solid, c(0.0), &mu_t_new), rk2.sum_where(&lb.fluid)),
            None => (mu_t_new, rk2.sum()),
        };
        let rn = (sum / c(self.n_active as f64)).sqrt();
        [k_new, om_new, mu_t_new, rn]
    }

    /// The turbulence slots of the next loop state: the SST update of the iteration (`dt` the time step of the iteration), else the slots
    /// carried over.
    fn turb_next(&self, s: &State, st: &Stage, dt: &A) -> [A; 4] {
        if self.sst() {
            self.sst_update(s, st, dt)
        } else {
            std::array::from_fn(|k| &s[6 + k] * c(1.0))
        }
    }
}
