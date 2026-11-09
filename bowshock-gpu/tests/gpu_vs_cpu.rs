//! GPU (f32, WGSL via Luma) vs CPU (f64, bowshock-cfd) on the same inputs.

use bowshock_gpu::{eval_faces, eval_faces_ext, GpuSolver, Mode, Precision};
use bowshock_cfd::cone::{build_solver, ConeCase};
use bowshock_cfd::flux::{face_flux, hllc, reconstruct, Limiter};
use bowshock_cfd::gas::{cons_to_prim, Prim};
use bowshock_cfd::taylor_maccoll::solve_cone;

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
