//! Viscous terms and the SST model are not ported to the array-op GPU solver yet: `GpuSolver` refuses a viscous host solver. The comparisons
//! of the previous hand-written kernels (laminar plate G1, SST plate G2, the G3 body) are kept, ignored, for the port.

use bowshock::atmosphere::Atmosphere;
use bowshock::body_grid::BaseBody;
use bowshock::case::{Case, Wake};
use bowshock::gpu::GpuRunner;
use bowshock::run::run_cold;
use bowshock::spec::{AftSpec, BodySpec, NoseShape, NoseSpec};
use bowshock_core::flux::Limiter;
use bowshock_core::gas::freestream;
use bowshock_core::grid::{plate_grid, PlateGridParams};
use bowshock_core::implicit::ImplicitParams;
use bowshock_core::solver::{Bc, Config, Solver, Turbulence, Viscous};
use bowshock_core::turbulence::K_INF;
use bowshock_gpu::{GpuError, GpuSolver, Precision};

/// The flat plates of the CPU gates (annulus formulation, Mach 0.2, `Scheme::Implicit(ImplicitParams::VISCOUS)`, van Albada, low-Mach
/// correction), with only 5 first-order iterations: G1 (laminar, `re_c = 1e6`) or G2 (SST, `re_c = 2.5e7`).
fn plate_solver(sst: bool) -> Solver {
    let (grid, re_c, turbulence, wall_i_start) = if sst {
        (
            plate_grid(&PlateGridParams { r0: 20.0, x_up: 0.33, length: 2.0, height: 1.0, n_up: 24, n_plate: 112, nj: 72, dx_le: 0.002, dr_wall: 4e-6 }),
            2.5e7,
            Turbulence::Sst,
            24,
        )
    } else {
        (
            plate_grid(&PlateGridParams { r0: 20.0, x_up: 0.5, length: 1.0, height: 0.6, n_up: 20, n_plate: 80, nj: 64, dx_le: 0.005, dr_wall: 2e-4 }),
            1e6,
            Turbulence::Laminar,
            20,
        )
    };
    let cfg = Config {
        cfl: 0.5,
        limiter: Limiter::VanAlbada,
        freestream: freestream(0.2),
        bc: [Bc::Characteristic, Bc::PressureOutflow, Bc::SlipWall, Bc::Characteristic],
        first_order_iters: 5,
    };
    let mut s = Solver::new(grid, cfg);
    s.low_mach = true;
    s.set_scheme(bowshock_core::solver::Scheme::Implicit(ImplicitParams::VISCOUS));
    s.set_viscous(Some(Viscous { re_c, sutherland: 0.368, turbulence, wall_i_start }));
    s
}

/// Maximum and mean of `|U_gpu - U_cpu| / (1 + |U_cpu|)` over the cells and the four components.
fn state_diff(gpu: &[[f64; 4]], cpu: &[[f64; 4]]) -> (f64, f64) {
    let d: Vec<f64> = gpu.iter().zip(cpu).flat_map(|(a, b)| (0..4).map(move |k| (a[k] - b[k]).abs() / (1.0 + b[k].abs()))).collect();
    (d.iter().fold(0.0f64, |m, &x| m.max(x)), d.iter().sum::<f64>() / d.len() as f64)
}

/// Laminar flat plate (G1): the viscous terms (cell gradients, viscous face fluxes, hoop stress, diffusive pseudo-time step), the no-slip wall
/// ghost with its Jacobian and the diffusive spectral radius of the implicit operator, after 30 implicit iterations from the free stream (5
/// first order): the GPU state in both precisions that matter vs the CPU.
#[test]
#[ignore = "viscous GPU port pending"]
fn viscous_laminar_plate_state_matches_cpu() {
    let ctx = luma::default_context().unwrap();
    let mut cpu = plate_solver(false);
    let res_cpu: Vec<f64> = (0..30).map(|_| cpu.step()).collect();
    for prec in [Precision::F32, Precision::Extended] {
        let mut gpu = GpuSolver::from_host(&ctx, &cpu, prec).unwrap();
        let res_gpu = gpu.run(30).unwrap();
        let (max, mean) = state_diff(&gpu.state_f64().unwrap(), &cpu.u);
        // (the first residual is round-off of the uniform start on both sides: compared from the second iteration on)
        let worst_res = res_gpu.iter().zip(&res_cpu).skip(1).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        println!(
            "laminar plate {prec:?}: max |dU|/(1+|U|) = {max:.2e}, mean {mean:.2e}, max rel residual diff {worst_res:.2e}, residual {:.3e} -> {:.3e}",
            res_cpu[1], res_cpu[29]
        );
        assert!(max <= 1e-3 && res_gpu.iter().all(|r| r.is_finite()), "laminar plate {prec:?}: state difference {max:e}");
    }
}

/// SST flat plate (G2 grid), 50 iterations from the free stream: the mean state difference vs CPU and the `k`, `omega` fields on the cells where the
/// turbulence has grown (`k > 10 k_inf`).
#[test]
#[ignore = "viscous GPU port pending"]
fn viscous_sst_plate_state_matches_cpu() {
    let ctx = luma::default_context().unwrap();
    let mut cpu = plate_solver(true);
    let res_cpu: Vec<f64> = (0..50).map(|_| cpu.step()).collect();
    for prec in [Precision::F32, Precision::Extended] {
        let mut gpu = GpuSolver::from_host(&ctx, &cpu, prec).unwrap();
        let res_gpu = gpu.run(50).unwrap();
        let (max, mean) = state_diff(&gpu.state_f64().unwrap(), &cpu.u);
        let ts = gpu.turb_state().unwrap();
        let (mut dk, mut dw, mut n) = (0.0f64, 0.0f64, 0);
        for (t, c) in ts.iter().zip(&cpu.turb) {
            if c[0] > 10.0 * K_INF {
                dk = dk.max((t[0] as f64 - c[0]).abs() / c[0]);
                dw = dw.max((t[1] as f64 - c[1]).abs() / c[1]);
                n += 1;
            }
        }
        let worst_res = res_gpu.iter().zip(&res_cpu).skip(1).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        println!(
            "SST plate {prec:?}: state max {max:.2e} mean {mean:.2e}; k, omega on {n} cells with k > 10 k_inf: max rel diff {dk:.2e}, {dw:.2e}; max rel residual diff {worst_res:.2e}, residual {:.3e} -> {:.3e}",
            res_cpu[1], res_cpu[49]
        );
        assert!(n > 100, "the turbulence has not grown: {n} cells with k > 10 k_inf");
        assert!(mean <= 1e-3 && dk <= 1e-2 && dw <= 1e-2, "SST plate {prec:?}: mean state {mean:e}, k {dk:e}, omega {dw:e}");
    }
}

/// G3 body (tangent ogive 0.30 x 0.10 m + 0.60 m tube, ISA 0 m, Mach 0.5, coarse grid, flat base and wake, SST) iterated by the CPU to its own stop and
/// by the GPU for the same number of iterations: total drag within 1 %. (The CPU run takes about half a minute.)
#[test]
#[ignore = "viscous GPU port pending"]
fn viscous_body_total_drag_matches_cpu() {
    let (d, mach) = (0.10, 0.5);
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: d }, aft: vec![AftSpec::Tube { length: 0.60 }] };
    let (body, _): (BaseBody, _) = spec.to_internal().unwrap();
    let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
    let re_c = air.density * air.sound_speed * d / air.viscosity;
    let mut case = Case::viscous(mach, body, 0, Wake::Base, re_c, 110.4 / air.temperature_k, mach);
    let cpu = run_cold(&case);
    println!("CPU: {} after {} iterations ({:.1} orders)", cpu.stop_reason, cpu.iters, cpu.drop_orders);
    case.numerics.min_iter = cpu.iters;
    case.numerics.max_iter = cpu.iters;
    let mut gs = GpuRunner::new(&case, Precision::Extended).unwrap();
    let gpu = gs.run_on(&case, false, None).unwrap().result;
    let rel = (gpu.cd_total / cpu.cd_total - 1.0).abs();
    println!(
        "total CD gpu {:.5} cpu {:.5} ({:+.3} %); cd_friction gpu {:.5} cpu {:.5}; cd_base gpu {:.5} cpu {:.5}; cp_base gpu {:.4} cpu {:.4}; y+ max gpu {:.3} cpu {:.3}",
        gpu.cd_total,
        cpu.cd_total,
        100.0 * (gpu.cd_total / cpu.cd_total - 1.0),
        gpu.cd_friction,
        cpu.cd_friction,
        gpu.cd_base,
        cpu.cd_base,
        gpu.cp_base,
        cpu.cp_base,
        gpu.y_plus_max,
        cpu.y_plus_max
    );
    assert!(rel < 0.01, "total CD differs by {:.2} %", 100.0 * rel);
}

/// The GPU solver refuses viscous terms with a clear error instead of running the Euler equations.
#[test]
fn viscous_host_solvers_are_refused() {
    let ctx = luma::default_context().unwrap();
    let cpu = plate_solver(false);
    for prec in [Precision::F32, Precision::Extended] {
        assert!(matches!(GpuSolver::from_host(&ctx, &cpu, prec), Err(GpuError::Unsupported(_))));
    }
    let mut gpu = GpuSolver::new(&ctx, &cpu.grid, &cpu.cfg, Precision::F32).unwrap();
    assert!(matches!(gpu.set_viscous(cpu.viscous()), Err(GpuError::Unsupported(_))));
    assert!(gpu.set_viscous(None).is_ok() && gpu.viscous().is_none());
}
