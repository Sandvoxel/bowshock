//! GPU (f32, WGSL via Luma) vs CPU (f64, bowshock-cfd) on the same inputs.

use bowshock_gpu::transonic::GpuTransonic;
use bowshock_gpu::{eval_faces, eval_faces_ext, ogive, GpuSolver, Mode, Precision};
use bowshock_cfd::cone::{build_solver, ConeCase};
use bowshock_cfd::ogive::OgiveCase;
use bowshock_cfd::flux::{face_flux, hllc, reconstruct, Limiter};
use bowshock_cfd::gas::{cons_to_prim, freestream, prim_to_cons, Prim};
use bowshock_cfd::grid::Grid;
use bowshock_cfd::solver::{Bc, Config, Solver};
use bowshock_cfd::taylor_maccoll::solve_cone;
use bowshock_cfd::transonic::{self, TransonicCase};

/// xorshift64* uniform in [0, 1).
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
}

fn random_prim(r: &mut Rng) -> [f32; 4] {
    [
        r.range(0.1, 3.0) as f32,
        r.range(-4.0, 4.0) as f32,
        r.range(-4.0, 4.0) as f32,
        r.range(0.1, 4.0) as f32,
    ]
}

fn to64(w: &[f32; 4]) -> Prim {
    [w[0] as f64, w[1] as f64, w[2] as f64, w[3] as f64]
}

/// Random face samples (primitive stencil `m2, m1, p0, p1` + unit normal).
fn samples(n: usize) -> (Vec<[[f32; 4]; 4]>, Vec<[f32; 2]>) {
    let mut rng = Rng(0x9E3779B97F4A7C15);
    let mut states = Vec::new();
    let mut normals = Vec::new();
    for s in 0..n {
        // two thirds of the samples: smooth data (neighbours are perturbations of 0.3% resp. 3%, so
        // the limiters are active and the shock sensor is off resp. in its smoothstep range); the
        // rest: unrelated states (strong discontinuities, Rusanov).
        let base = random_prim(&mut rng);
        let st: [[f32; 4]; 4] = if s % 3 != 2 {
            let amp = if s % 3 == 0 { 0.003f32 } else { 0.03 };
            let mut st = [base; 4];
            let mut cur = base;
            for k in 0..4 {
                for c in 0..4 {
                    cur[c] *= 1.0 + amp * (rng.next() as f32 - 0.4);
                }
                st[k] = cur;
            }
            st
        } else {
            [random_prim(&mut rng), random_prim(&mut rng), random_prim(&mut rng), random_prim(&mut rng)]
        };
        states.push(st);
        let a = rng.range(0.0, std::f64::consts::TAU);
        normals.push([a.cos() as f32, a.sin() as f32]);
    }
    (states, normals)
}

/// Face-kernel comparison on random states: reconstruction, HLLC and the complete face flux (HLLC/Rusanov
/// blend with the pressure-jump sensor) for first order and both limiters. Inputs are f32-representable, so the CPU evaluates the same numbers in f64.
#[test]
fn hllc_and_reconstruction_match_cpu_on_random_states() {
    let (states, normals) = samples(20_000);
    let n = states.len();
    for lim in [Limiter::FirstOrder, Limiter::Minmod, Limiter::VanAlbada] {
        let gpu = eval_faces(&states, &normals, lim).expect("GPU face kernel");
        let (mut worst_flux, mut worst_state, mut worst_face) = (0.0f64, 0.0f64, 0.0f64);
        let (mut mismatched_fallback, mut blended) = (0usize, 0usize);
        for s in 0..n {
            let [m2, m1, p0, p1] = states[s].map(|w| to64(&w));
            let (l, r) = reconstruct(&m2, &m1, &p0, &p1, lim);
            let (nx, nr) = (normals[s][0] as f64, normals[s][1] as f64);
            let f = hllc(&l, &r, nx, nr);
            let ff = face_flux(&m2, &m1, &p0, &p1, lim, nx, nr);
            let [gl, gr, gf, gff] = gpu[s];
            // reconstructed states: relative to the state magnitude
            let (gl, gr) = (to64(&gl), to64(&gr));
            let mut st_err = 0.0f64;
            for c in 0..4 {
                st_err = st_err.max((gl[c] - l[c]).abs() / (1.0 + l[c].abs()));
                st_err = st_err.max((gr[c] - r[c]).abs() / (1.0 + r[c].abs()));
            }
            // the positivity fallback is a hard switch; a sample sitting on it may legitimately differ
            if st_err > 1e-3 {
                mismatched_fallback += 1;
                continue;
            }
            worst_state = worst_state.max(st_err);
            // fluxes: relative to the largest flux component (floor 1)
            let scale = f.iter().chain(&hllc(&m1, &p0, nx, nr)).fold(1.0f64, |a, b| a.max(b.abs()));
            let (gf, gff) = (to64(&gf), to64(&gff));
            for c in 0..4 {
                worst_flux = worst_flux.max((gf[c] - f[c]).abs() / scale);
                worst_face = worst_face.max((gff[c] - ff[c]).abs() / scale);
            }
            if bowshock_cfd::flux::shock_weight(m1[3], p0[3]) > 0.0 {
                blended += 1;
            }
        }
        println!(
            "{lim:?}: max state err {worst_state:.2e}, max HLLC flux err {worst_flux:.2e}, max face_flux err {worst_face:.2e} \
             ({blended} of {n} samples with a Rusanov share), hard-switch mismatches {mismatched_fallback}"
        );
        assert!(mismatched_fallback <= 5, "{lim:?}: too many reconstruction mismatches");
        assert!(worst_state < 2e-6, "{lim:?}: reconstruction error {worst_state:e}");
        assert!(worst_flux < 1e-4, "{lim:?}: HLLC flux error {worst_flux:e}");
        assert!(worst_face < 1e-4, "{lim:?}: face flux error {worst_face:e}");
    }
}

/// The double-float face kernels (`Precision::Extended`) on the same samples: reconstruction, HLLC and the complete
/// face flux agree with the f64 `bowshock-cfd` functions to ~1e-9 (relative to the largest flux component, floor 1) where
/// the f32 kernels reach ~1e-5.
#[test]
fn extended_face_kernels_match_cpu_to_1e9() {
    let (states, normals) = samples(20_000);
    let n = states.len();
    for lim in [Limiter::FirstOrder, Limiter::Minmod, Limiter::VanAlbada] {
        let gpu = eval_faces_ext(&states, &normals, lim).expect("GPU face kernel");
        let (mut worst_flux, mut worst_state, mut worst_face, mut skipped) = (0.0f64, 0.0f64, 0.0f64, 0usize);
        for s in 0..n {
            let [m2, m1, p0, p1] = states[s].map(|w| to64(&w));
            let (l, r) = reconstruct(&m2, &m1, &p0, &p1, lim);
            let (nx, nr) = (normals[s][0] as f64, normals[s][1] as f64);
            let f = hllc(&l, &r, nx, nr);
            let ff = face_flux(&m2, &m1, &p0, &p1, lim, nx, nr);
            let [gl, gr, gf, gff] = gpu[s];
            let mut st_err = 0.0f64;
            for c in 0..4 {
                st_err = st_err.max((gl[c] - l[c]).abs() / (1.0 + l[c].abs()));
                st_err = st_err.max((gr[c] - r[c]).abs() / (1.0 + r[c].abs()));
            }
            // hard switches (positivity fallback, limiter / wave-speed selection) can legitimately differ for a
            // sample sitting exactly on them
            if st_err > 1e-3 {
                skipped += 1;
                continue;
            }
            worst_state = worst_state.max(st_err);
            let scale = f.iter().chain(&hllc(&m1, &p0, nx, nr)).fold(1.0f64, |a, b| a.max(b.abs()));
            for c in 0..4 {
                worst_flux = worst_flux.max((gf[c] - f[c]).abs() / scale);
                worst_face = worst_face.max((gff[c] - ff[c]).abs() / scale);
            }
        }
        println!("df {lim:?}: max state err {worst_state:.2e}, max HLLC flux err {worst_flux:.2e}, max face_flux err {worst_face:.2e}, skipped {skipped}");
        assert!(skipped <= 5, "{lim:?}: too many hard-switch mismatches");
        assert!(worst_state < 1e-9, "{lim:?}: reconstruction error {worst_state:e}");
        assert!(worst_flux < 1e-9, "{lim:?}: HLLC flux error {worst_flux:e}");
        assert!(worst_face < 1e-9, "{lim:?}: face flux error {worst_face:e}");
    }
}

/// A few iterations on a small cone grid: conserved state GPU vs CPU, both limiters and both
/// kernel formulations; MUSCL active from iteration 5 on (the first 5 iterations are first order,
/// which exercises the per-launch limiter uniform).
#[test]
fn state_after_20_iterations_matches_cpu() {
    let ctx = luma::default_context().unwrap();
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    for limiter in [Limiter::Minmod, Limiter::VanAlbada] {
        let mut case = ConeCase::new(2.0, 15.0, 40, 20);
        case.limiter = limiter;
        case.first_order_iters = 5;
        let mut cpu = build_solver(&case, tm.beta);
        let n = 20;
        let res_cpu: Vec<f64> = (0..n).map(|_| cpu.step()).collect();
        for (mode, prec) in [Mode::Direct, Mode::Guarded]
            .into_iter()
            .flat_map(|m| [Precision::F32, Precision::Compensated, Precision::Extended].into_iter().map(move |p| (m, p)))
        {
            let mut gpu = GpuSolver::new_with(&ctx, &build_solver(&case, tm.beta).grid, &cpu.cfg, mode, prec).expect("GPU solver");
            let res_gpu = gpu.run(n).expect("GPU run");

            let ug = gpu.state_f64().unwrap();
            let (mut worst, mut worst_p) = (0.0f64, 0.0f64);
            for (a, b) in ug.iter().zip(&cpu.u) {
                for k in 0..4 {
                    worst = worst.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()));
                }
                let (pa, pb) = (cons_to_prim(&a.map(|x| x as f64)), cons_to_prim(b));
                worst_p = worst_p.max((pa[3] - pb[3]).abs() / pb[3]);
            }
            let mut worst_res = 0.0f64;
            for (a, b) in res_gpu.iter().zip(&res_cpu) {
                worst_res = worst_res.max((a - b).abs() / b);
            }
            // Luma's norm() reduction of the last stage-1 density residual vs the history kernels
            let luma_norm = gpu.luma_residual().unwrap();
            let hist_last = *res_gpu.last().unwrap();
            println!(
                "{mode:?} {prec:?} {limiter:?}: max |dU|/(1+|U|) = {worst:.2e}, max rel dp = {worst_p:.2e}, max rel residual diff = {worst_res:.2e}, \
                 luma norm {luma_norm:.6e} vs history {hist_last:.6e}"
            );
            assert!(worst < 1e-5, "{mode:?} {prec:?} {limiter:?}: state differs by {worst:e}");
            if prec == Precision::Extended {
                // only the f32 geometry (normals, face weights, volumes) separates the extended solver from the f64 CPU
                assert!(worst < 3e-7 && worst_res < 2e-7, "{mode:?} {limiter:?}: extended precision state {worst:e}, residual {worst_res:e}");
            }
            assert!(worst_p < 2e-5, "{mode:?} {prec:?} {limiter:?}: pressure differs by {worst_p:e}");
            assert!(worst_res < 1e-4, "{mode:?} {prec:?} {limiter:?}: residual history differs by {worst_res:e}");
            assert!(((luma_norm - hist_last) / hist_last).abs() < 1e-5, "Luma norm vs history kernels");
        }
    }
}

/// 400x200: more than 128 workgroups in the fused residual kernel (partial-sum path of the
/// history reduction), 3 iterations; residual history and state vs CPU.
#[test]
fn large_grid_residual_history_matches_cpu() {
    let ctx = luma::default_context().unwrap();
    let tm = solve_cone(2.0, 10f64.to_radians()).unwrap();
    let mut case = ConeCase::new(2.0, 10.0, 400, 200);
    case.first_order_iters = 0;
    let mut cpu = build_solver(&case, tm.beta);
    let res_cpu: Vec<f64> = (0..3).map(|_| cpu.step()).collect();
    for mode in [Mode::Direct, Mode::Guarded] {
        let mut gpu = GpuSolver::new_on(&ctx, &cpu.grid, &cpu.cfg, mode).unwrap();
        let res_gpu = gpu.run(3).unwrap();
        let worst_res = res_gpu.iter().zip(&res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        let worst = gpu.state().unwrap().iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| {
            (0..4).fold(m, |m, k| m.max((a[k] as f64 - b[k]).abs() / (1.0 + b[k].abs())))
        });
        println!("{mode:?} 400x200: residual diff {worst_res:.2e}, state diff {worst:.2e}");
        assert!(worst_res < 1e-4 && worst < 1e-5, "{mode:?}: residual diff {worst_res:e}, state diff {worst:e}");
    }
}

/// Regression test of the f32 residual floor: a small cone grid (60x30, 15 deg, minmod) is converged by the GPU solver
/// in all three precisions. The original f32 solver and the compensated state stall at 4 - 5 orders; the extended
/// precision reaches the CPU's 6-orders criterion (and keeps dropping), and its converged field agrees with the f64
/// CPU solution.
#[test]
fn extended_precision_reaches_six_orders_on_a_small_grid() {
    let ctx = luma::default_context().unwrap();
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let case = ConeCase::new(2.0, 15.0, 60, 30);
    let host = build_solver(&case, tm.beta);
    let mut orders = Vec::new();
    for prec in [Precision::F32, Precision::Compensated, Precision::Extended] {
        let mut gpu = GpuSolver::new_with(&ctx, &host.grid, &host.cfg, Mode::Guarded, prec).unwrap();
        let mut res: Vec<f64> = Vec::new();
        let mut peak = 0.0f64;
        while res.len() < 4000 {
            for r in gpu.run(50).unwrap() {
                if res.len() >= 20 {
                    peak = peak.max(r);
                }
                res.push(r);
            }
            if prec == Precision::Extended && (peak / res[res.len() - 1]).log10() >= 6.0 {
                break;
            }
        }
        let best = res.iter().skip(20).cloned().fold(f64::INFINITY, f64::min);
        println!("{prec:?}: {} iterations, peak {peak:.2e}, min {best:.2e}, {:.2} orders", res.len(), (peak / best).log10());
        orders.push((peak / best).log10());
        if prec == Precision::Extended {
            // converged field vs the f64 CPU solution (same number of iterations are not comparable; compare the converged state)
            let mut cpu = build_solver(&case, tm.beta);
            for _ in 0..res.len() {
                cpu.step();
            }
            let ug = gpu.state_f64().unwrap();
            let worst = ug.iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| (0..4).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
            println!("Extended: max |dU|/(1+|U|) vs CPU after the same number of iterations = {worst:.2e}");
            assert!(worst < 1e-5, "extended precision state differs from the CPU by {worst:e}");
        }
    }
    assert!(orders[0] < 5.5, "f32 solver unexpectedly converged {} orders", orders[0]);
    assert!(orders[1] < 5.5, "compensated solver unexpectedly converged {} orders", orders[1]);
    assert!(orders[2] >= 6.0, "extended solver only reached {} orders", orders[2]);
}

/// Ogive-cylinder (M = 1.98, coarse grid 114x50: free-stream inflow, extrapolation outflow, slip wall/axis, free-stream outer
/// boundary; nose tip node at x = 0, nose/cylinder junction node): state after 20 iterations (5 first order) vs CPU for both
/// formulations and every precision, then the wall Cp and forebody CD after 500 iterations vs CPU.
#[test]
fn ogive_state_and_wall_cp_match_cpu() {
    let ctx = luma::default_context().unwrap();
    let mut case = OgiveCase::level(1.98, 0);
    case.first_order_iters = 5;
    let mut cpu = case.build_solver();
    let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
    for (mode, prec) in [Mode::Direct, Mode::Guarded]
        .into_iter()
        .flat_map(|m| [Precision::F32, Precision::Extended].into_iter().map(move |p| (m, p)))
    {
        let mut gpu = GpuSolver::new_with(&ctx, &case.build_solver().grid, &cpu.cfg, mode, prec).expect("GPU solver");
        let res_gpu = gpu.run(20).expect("GPU run");
        let ug = gpu.state_f64().unwrap();
        let worst = ug.iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| (0..4).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
        let worst_res = res_gpu.iter().zip(&res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        println!("ogive {mode:?} {prec:?}: max |dU|/(1+|U|) = {worst:.2e}, max rel residual diff = {worst_res:.2e}");
        assert!(worst < 1e-5 && worst_res < 1e-4, "{mode:?} {prec:?}: state {worst:e}, residual {worst_res:e}");
        if prec == Precision::Extended {
            assert!(worst < 3e-7 && worst_res < 2e-7, "{mode:?}: extended precision state {worst:e}, residual {worst_res:e}");
        }
    }

    // 500 iterations (MUSCL from iteration 5): wall Cp and forebody CD
    let n = 500;
    let mut cpu = case.build_solver();
    for _ in 0..n {
        cpu.step();
    }
    let post_cpu = ogive::postprocess(&case, &cpu.grid, &cpu.cfg.freestream, &cpu.u);
    let mut gpu = GpuSolver::new_with(&ctx, &cpu.grid, &cpu.cfg, Mode::Guarded, Precision::Extended).unwrap();
    for _ in 0..n / 50 {
        gpu.run(50).unwrap();
    }
    let post_gpu = ogive::postprocess(&case, &cpu.grid, &cpu.cfg.freestream, &gpu.state_f64().unwrap());
    let max_dcp = post_gpu.wall.iter().zip(&post_cpu.wall).fold(0.0f64, |m, (a, b)| m.max((a.cp - b.cp).abs()));
    let cd_rel = (post_gpu.cd_nose - post_cpu.cd_nose) / post_cpu.cd_nose;
    println!("ogive after {n} iterations: CD gpu {:.6} cpu {:.6} (rel {cd_rel:+.2e}), max |dCp| over the wall {max_dcp:.2e}", post_gpu.cd_nose, post_cpu.cd_nose);
    assert!(cd_rel.abs() < 1e-5, "forebody CD differs by {cd_rel:e}");
    assert!(max_dcp < 1e-5, "wall Cp differs by {max_dcp:e}");
}

/// Rectangular axisymmetric box `x in [0, 4]`, `r in [0, 2]` (axis at `j = 0`, slightly stretched in both directions so that
/// the geometry is not uniform) with the characteristic far field on the three outer boundaries (`outflow` on `imax`).
fn far_field_box(mach: f64, outflow: Bc) -> Solver {
    let (ni, nj) = (40usize, 20usize);
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = x.clone();
    for j in 0..=nj {
        for i in 0..=ni {
            x[j * (ni + 1) + i] = 4.0 * (i as f64 / ni as f64).powf(1.15);
            r[j * (ni + 1) + i] = 2.0 * (j as f64 / nj as f64).powf(1.1);
        }
    }
    let cfg = Config {
        cfl: 0.5,
        limiter: Limiter::Minmod,
        freestream: freestream(mach),
        bc: [Bc::Characteristic, outflow, Bc::SlipWall, Bc::Characteristic],
        first_order_iters: 5,
    };
    Solver::new(Grid::from_nodes(ni, nj, x, r), cfg)
}

/// Smooth density / pressure / transverse-velocity perturbation of the free stream, strong enough to put the far-field faces
/// in the subsonic inflow, subsonic outflow and (at M = 1.3) supersonic branches of the characteristic BC.
fn perturbed_state(s: &Solver) -> Vec<[f64; 4]> {
    let fs = s.cfg.freestream;
    (0..s.grid.ni * s.grid.nj)
        .map(|c| {
            let (xc, rc) = (s.grid.xc[c], s.grid.rc[c]);
            let g = (-((xc - 3.0).powi(2) + (rc - 1.4).powi(2)) / 0.8).exp() + 0.5 * (-((xc - 0.6).powi(2) + (rc - 1.6).powi(2)) / 0.5).exp();
            prim_to_cons(&[fs[0] * (1.0 + 0.25 * g), fs[1] + 0.2 * g, 0.3 * g * (0.5 * xc).sin(), fs[3] * (1.0 + 0.4 * g)])
        })
        .collect()
}

/// Characteristic far field at M = 0.9 (all far-field faces subsonic) and 1.3: the uniform free stream is preserved to round-off
/// by the CPU and to the f32 geometry rounding by the GPU (both formulations, every precision): no spurious waves from the BC.
#[test]
fn characteristic_far_field_preserves_the_free_stream() {
    let ctx = luma::default_context().unwrap();
    for mach in [0.9, 1.3] {
        let mut cpu = far_field_box(mach, Bc::Characteristic);
        let fs = prim_to_cons(&cpu.cfg.freestream);
        let dev = |u: &[[f64; 4]]| u.iter().fold(0.0f64, |m, q| (0..4).fold(m, |m, k| m.max((q[k] - fs[k]).abs() / (1.0 + fs[k].abs()))));
        for _ in 0..30 {
            cpu.step();
        }
        let dev_cpu = dev(&cpu.u);
        println!("M = {mach}: CPU max deviation from the free stream after 30 iterations: {dev_cpu:.2e}");
        assert!(dev_cpu < 1e-12, "CPU: free stream not preserved at M = {mach}: {dev_cpu:e}");
        for (mode, prec) in [Mode::Direct, Mode::Guarded]
            .into_iter()
            .flat_map(|m| [Precision::F32, Precision::Extended].into_iter().map(move |p| (m, p)))
        {
            let mut gpu = GpuSolver::new_with(&ctx, &cpu.grid, &cpu.cfg, mode, prec).unwrap();
            gpu.run(30).unwrap();
            let d = dev(&gpu.state_f64().unwrap());
            println!("M = {mach} {mode:?} {prec:?}: max deviation from the free stream after 30 iterations: {d:.2e}");
            // f32 rounding of the face normals / areas / volumes bounds the closure of a uniform flow (~1e-7); no growth in 30 iterations
            assert!(d < 1e-5 && (prec == Precision::F32 || d < 2e-6), "{mode:?} {prec:?} M = {mach}: free stream deviates by {d:e}");
        }
    }
}

/// Characteristic far field and static-pressure outflow on a perturbed state: GPU vs CPU after 20 iterations (5 first order), both
/// formulations and precisions, at M = 0.9 (subsonic inflow / outflow Riemann branches, outflow with the characteristic BC and with
/// fixed pressure) and M = 1.3 (supersonic inflow = free stream and supersonic outflow = extrapolation on the same boundaries).
#[test]
fn characteristic_far_field_matches_cpu_on_a_perturbed_state() {
    let ctx = luma::default_context().unwrap();
    for (mach, outflow) in [(0.9, Bc::Characteristic), (0.9, Bc::PressureOutflow), (1.3, Bc::Characteristic), (1.3, Bc::PressureOutflow)] {
        let mut cpu = far_field_box(mach, outflow);
        let u0 = perturbed_state(&cpu);
        cpu.u = u0.clone();
        let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
        for (mode, prec) in [Mode::Direct, Mode::Guarded]
            .into_iter()
            .flat_map(|m| [Precision::F32, Precision::Extended].into_iter().map(move |p| (m, p)))
        {
            let mut gpu = GpuSolver::new_with(&ctx, &cpu.grid, &cpu.cfg, mode, prec).unwrap();
            gpu.set_state(&u0).unwrap();
            let res_gpu = gpu.run(20).unwrap();
            let ug = gpu.state_f64().unwrap();
            let worst = ug.iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| (0..4).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
            let worst_res = res_gpu.iter().zip(&res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
            println!("M = {mach} {outflow:?} {mode:?} {prec:?}: max |dU|/(1+|U|) = {worst:.2e}, max rel residual diff = {worst_res:.2e}");
            assert!(worst < 1e-5 && worst_res < 1e-4, "M = {mach} {outflow:?} {mode:?} {prec:?}: state {worst:e}, residual {worst_res:e}");
            if prec == Precision::Extended {
                assert!(worst < 3e-7 && worst_res < 2e-7, "M = {mach} {outflow:?} {mode:?}: extended state {worst:e}, residual {worst_res:e}");
            }
        }
    }
}

/// Transonic case (ogive-cylinder in the large domain, characteristic far field, coarse level 0: 144x70) at M = 0.9: state after
/// 20 iterations (5 first order) vs CPU for both formulations and the f32 / extended precisions.
#[test]
fn transonic_state_after_20_iterations_matches_cpu() {
    let ctx = luma::default_context().unwrap();
    let mut case = TransonicCase::level(0.9, 0);
    case.first_order_iters = 5;
    let mut cpu = case.build_solver();
    let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
    for (mode, prec) in [Mode::Direct, Mode::Guarded]
        .into_iter()
        .flat_map(|m| [Precision::F32, Precision::Extended].into_iter().map(move |p| (m, p)))
    {
        let mut gpu = GpuSolver::new_with(&ctx, &cpu.grid, &cpu.cfg, mode, prec).unwrap();
        let res_gpu = gpu.run(20).unwrap();
        let ug = gpu.state_f64().unwrap();
        let worst = ug.iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| (0..4).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
        let worst_res = res_gpu.iter().zip(&res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        println!("transonic M = 0.9 {mode:?} {prec:?}: max |dU|/(1+|U|) = {worst:.2e}, max rel residual diff = {worst_res:.2e}");
        assert!(worst < 1e-5 && worst_res < 1e-4, "{mode:?} {prec:?}: state {worst:e}, residual {worst_res:e}");
        if prec == Precision::Extended {
            // the large-domain grid has cells of very different size: f32 geometry rounding enters the residual a little more than on the cone grids
            assert!(worst < 3e-7 && worst_res < 5e-7, "{mode:?}: extended state {worst:e}, residual {worst_res:e}");
        }
    }
}

/// Short cold-start run at M = 0.9 followed by a warm-started run at M = 0.95 (`run_on` of both solvers with the same
/// arguments, 200 iterations each): residual history, forebody CD and wall Cp vs CPU.
#[test]
fn transonic_run_and_warm_start_match_cpu() {
    let mut case = TransonicCase::level(0.9, 0);
    case.max_iter = 200;
    case.first_order_iters = 20;
    let mut s = case.build_solver();
    let cpu0 = transonic::run_on(&case, &mut s, false, None);
    let mut gt = GpuTransonic::new(&case, Mode::Guarded, Precision::Extended).unwrap();
    let gpu0 = gt.run_on(&case, false, None).unwrap();
    let check = |tag: &str, cpu: &transonic::TransonicResult, gpu: &bowshock_gpu::transonic::GpuTransonicRun| {
        let g = &gpu.result;
        assert_eq!((g.iters, gpu.iters_exec), (cpu.iters, cpu.iters), "{tag}: iteration counts");
        let res = g.residual.iter().zip(&cpu.residual).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        let dcp = g.wall.iter().zip(&cpu.wall).fold(0.0f64, |m, (a, b)| m.max((a.cp - b.cp).abs()));
        let cd_rel = (g.cd_nose - cpu.cd_nose) / cpu.cd_nose;
        let dcd_hist = g.cd_history.iter().zip(&cpu.cd_history).fold(0.0f64, |m, (a, b)| m.max((a.1 - b.1).abs()));
        println!("{tag}: CD gpu {:.6} cpu {:.6} (rel {cd_rel:+.2e}), max |dCp| {dcp:.2e}, max rel residual diff {res:.2e}, max |dCD history| {dcd_hist:.2e}, farfield dp {:.2e} / {:.2e}", g.cd_nose, cpu.cd_nose, g.farfield_dp, cpu.farfield_dp);
        assert!(res < 1e-4 && dcp < 1e-5 && cd_rel.abs() < 1e-5 && dcd_hist < 1e-6, "{tag}: GPU differs from CPU");
    };
    check("M = 0.9 cold start", &cpu0, &gpu0);
    case.mach = 0.95;
    let cpu1 = transonic::run_on(&case, &mut s, true, Some(cpu0.residual_peak));
    let gpu1 = gt.run_on(&case, true, Some(gpu0.result.residual_peak)).unwrap();
    check("M = 0.95 warm start", &cpu1, &gpu1);
}
