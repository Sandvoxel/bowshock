use bowshock_core::{AftSpec, Analysis, Backend, BackendUsed, BodySpec, Condition, Error, FinProfile, FinSpec, NoseShape, NoseSpec, Quality, Resolution};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Instant;

const D: f64 = 0.04;

/// ANF-like rocket: 10 d long, cone nose (10 degree half-angle), tube, four square fins of 1 d chord and span, flush with the base.
fn analysis() -> Analysis {
    let nose_len = 0.5 * D / 10f64.to_radians().tan();
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: D }, aft: vec![AftSpec::Tube { length: 10.0 * D - nose_len }] };
    let fins = FinSpec {
        count: 4,
        root_chord: D,
        tip_chord: D,
        span: D,
        sweep: 0.0,
        thickness: 0.08 * D,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Square,
    };
    Analysis::legacy(&spec)
        .unwrap()
        .fins(fins)
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .case3_hook(|c| {
            let s = &mut c.grid.sting;
            s.n_up = 10;
            s.n_nose = 16;
            s.n_seg = vec![12];
            s.n_ext = 10;
            s.nj = 24;
            c.grid.nk = 8;
        })
        .max_iterations(3000)
}

/// Mach numbers by angles of attack as conditions.
fn grid(machs: &[f64], alphas: &[f64]) -> Vec<Condition> {
    machs.iter().flat_map(|&m| alphas.iter().map(move |&a| Condition::new(m, a))).collect()
}

#[test]
fn run_gives_physical_points_in_the_order_asked_for() {
    let t = Instant::now();
    let polar = analysis().run(&grid(&[2.0], &[6.0, 2.0])).unwrap();
    println!("wall {:.1} s", t.elapsed().as_secs_f64());
    assert_eq!(polar.backend, BackendUsed::Cpu);
    assert!(polar.fallback.is_none());
    assert_eq!(polar.points.len(), 2);
    for p in &polar.points {
        let f = p.forces.expect("a point at an angle of attack has forces");
        println!("M {} alpha {:.1} deg: CN {:.4} CA {:.4} x_cp {:.4} m {:?} iters {}", p.mach, p.alpha_deg, f.cn, f.ca, f.x_cp, p.quality, p.detail.iterations);
        assert!(matches!(p.quality, Quality::Converged | Quality::Stationary));
        assert!(f.x_cp > 0.0 && f.x_cp < 10.0 * D, "x_cp {}", f.x_cp);
        assert_eq!(p.drag.forebody_pressure, f.ca, "the pressure axial force is the forebody pressure drag");
    }
    // the order asked for (6 degrees first), each point at its own angle
    let (b, a) = (&polar.points[0], &polar.points[1]);
    assert_eq!((a.alpha_deg, b.alpha_deg), (2.0, 6.0));
    assert!(b.forces.unwrap().cn > a.forces.unwrap().cn && a.forces.unwrap().cn > 0.0);
}

#[test]
fn run_rejects_bad_input() {
    // a Mach number outside [0.3, 3] at an angle of attack, an angle outside (0, 15], a NaN of either
    for (m, a) in [(3.5, 2.0), (0.2, 2.0), (2.0, -2.0), (2.0, 16.0), (f64::NAN, 2.0), (2.0, f64::NAN), (2.0, f64::INFINITY)] {
        assert!(matches!(analysis().run(&[Condition::new(m, a)]), Err(Error::InvalidInput(_))), "{m} {a}");
    }
    assert!(matches!(analysis().run::<Condition>(&[]), Err(Error::InvalidInput(_))));
    // zero incidence is the drag solver's: its Mach range is [0.1, 3]
    assert!(matches!(analysis().run(&[Condition::new(7.0, 0.0)]), Err(Error::InvalidInput(_))));
    // a bad condition after a good one of the other kind is found before anything runs
    let t = Instant::now();
    assert!(matches!(analysis().run(&[Condition::new(0.9, 0.0), Condition::new(2.0, 16.0)]), Err(Error::InvalidInput(_))));
    assert!(t.elapsed().as_secs_f64() < 2.0, "rejected up front, not after the drag sweep");
}

#[test]
fn gpu_without_feature_errors_and_auto_falls_back() {
    if cfg!(feature = "gpu") {
        return; // the GPU path is covered by the ignored `gpu_matches_cpu_tiny`
    }
    assert!(analysis().backend(Backend::Gpu).run(&[Condition::new(2.0, 2.0)]).is_err());
    let p = analysis().backend(Backend::Auto).max_iterations(100).run(&[Condition::new(2.0, 2.0)]).unwrap();
    assert_eq!(p.backend, BackendUsed::Cpu);
    assert!(p.fallback.is_some());
}

/// GPU (extended precision) against the CPU on the tiny case. Run through gpuq:
/// `cargo test --release -p bowshock-core --test angle_of_attack -- --ignored --nocapture`.
#[test]
#[ignore = "needs a GPU: run through tools/gpuq/gpuq"]
fn gpu_matches_cpu_tiny() {
    let t = Instant::now();
    let cpu = analysis().run(&grid(&[2.0], &[2.0, 6.0])).unwrap();
    let t_cpu = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let gpu = analysis().backend(Backend::Gpu).run(&grid(&[2.0], &[2.0, 6.0])).unwrap();
    let t_gpu = t.elapsed().as_secs_f64();
    println!("wall: CPU {t_cpu:.1} s, GPU {t_gpu:.1} s");
    assert_eq!(gpu.backend, BackendUsed::Gpu);
    assert!(gpu.fallback.is_none());
    assert_eq!(gpu.points.len(), 2);
    for (c, g) in cpu.points.iter().zip(&gpu.points) {
        let (cf, gf) = (c.forces.unwrap(), g.forces.unwrap());
        println!(
            "alpha {:.0}: CPU CN {:.5} x_cp {:.5} d {:?} {} it | GPU CN {:.5} x_cp {:.5} d {:?} {} it",
            c.alpha_deg,
            cf.cn,
            cf.x_cp / D,
            c.quality,
            c.detail.iterations,
            gf.cn,
            gf.x_cp / D,
            g.quality,
            g.detail.iterations
        );
        assert!(matches!(g.quality, Quality::Converged | Quality::Stationary));
        assert!((gf.cn - cf.cn).abs() <= 0.005 * cf.cn.abs(), "CN {} vs {}", gf.cn, cf.cn);
        assert!((gf.x_cp - cf.x_cp).abs() <= 0.01 * D, "x_cp {} vs {}", gf.x_cp / D, cf.x_cp / D);
    }
}

/// The same rocket on the real 3D grid (level 0: the 353 000 cells of the example table), no test hook.
fn analysis_full() -> Analysis {
    let nose_len = 0.5 * D / 10f64.to_radians().tan();
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: D }, aft: vec![AftSpec::Tube { length: 10.0 * D - nose_len }] };
    let fins = FinSpec {
        count: 4,
        root_chord: D,
        tip_chord: D,
        span: D,
        sweep: 0.0,
        thickness: 0.08 * D,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Square,
    };
    Analysis::legacy(&spec).unwrap().fins(fins).backend(Backend::Gpu).resolution(Resolution::Coarse)
}

/// What the flow field of a running 3D solve costs: the same two conditions on the GPU with and without `on_field3(100)` (five times the rate of
/// the app's 500) and `keep_fields`, alternating, four times each; the first round (kernels, context) is not counted. The solve does not change (same iterations, same CN); the wall time is printed (`SNAPSHOT_COST`).
/// `cargo test --release -p bowshock-core --features gpu --test angle_of_attack -- --ignored --nocapture gpu_snapshots`, through `tools/gpuq/gpuq`.
#[test]
#[ignore = "needs a GPU: run through tools/gpuq/gpuq"]
fn gpu_snapshots_and_kept_fields_cost_nothing_measurable() {
    let conditions = [Condition::new(2.0, 2.0), Condition::new(2.0, 6.0)];
    let mut times = [Vec::new(), Vec::new()];
    let mut seen = 0usize;
    for round in 0..5 {
        for (with, name) in [(false, "plain"), (true, "snapshots + kept fields")] {
            let count = Arc::new(AtomicUsize::new(0));
            let c = count.clone();
            let analysis = if with {
                analysis_full().keep_fields(true).on_field3(100, move |_| {
                    c.fetch_add(1, Ordering::SeqCst);
                })
            } else {
                analysis_full()
            };
            let t = Instant::now();
            let polar = analysis.run(&conditions).unwrap();
            let secs = t.elapsed().as_secs_f64();
            let iters: usize = polar.points.iter().map(|p| p.detail.iterations).sum();
            println!(
                "round {round} {name}: {secs:.2} s, {iters} iterations, {} snapshots, fields kept {}",
                count.load(Ordering::SeqCst),
                polar.points.iter().all(|p| p.field3.is_some())
            );
            if round > 0 {
                times[usize::from(with)].push(secs);
            }
            if with {
                seen = count.load(Ordering::SeqCst);
            }
        }
    }
    let mean = |v: &Vec<f64>| v.iter().sum::<f64>() / v.len() as f64;
    let (plain, with) = (mean(&times[0]), mean(&times[1]));
    println!("SNAPSHOT_COST {{\"plain_s\":{plain:.3},\"with_snapshots_s\":{with:.3},\"ratio\":{:.4},\"snapshots_per_solve\":{seen},\"plain_runs\":{:?},\"with_runs\":{:?}}}", with / plain, times[0], times[1]);
    assert!(seen >= 2, "snapshots arrived");
    assert!(with < plain * 1.10, "snapshots and kept fields cost {:.1} %", 100.0 * (with / plain - 1.0));
}
