use bowshock_core::{AftSpec, Analysis, Backend, BackendUsed, BodySpec, Error, FinProfile, FinSpec, NoseShape, NoseSpec, Quality, Resolution};
use std::time::Instant;

const D: f64 = 0.04;

/// ANF-like rocket: 10 d long, cone nose (10 degree half-angle), tube, four square fins of 1 d chord and span, flush with the base.
fn analysis() -> Analysis {
    let nose_len = 0.5 * D / 10f64.to_radians().tan();
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: D }, aft: vec![AftSpec::Tube { length: 10.0 * D - nose_len }] };
    let fins = FinSpec { count: 4, root_chord: D, tip_chord: D, span: D, sweep: 0.0, thickness: 0.08 * D, roughness: 0.0, root_te_to_base: 0.0, profile: FinProfile::Square };
    Analysis::new(&spec)
        .unwrap()
        .fins(fins)
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .lift_case_hook(|c| {
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

#[test]
fn run_lift_gives_ordered_physical_points() {
    let t = Instant::now();
    let polar = analysis().run_lift(&[2.0], &[6.0, 2.0]).unwrap();
    println!("wall {:.1} s", t.elapsed().as_secs_f64());
    assert_eq!(polar.backend, BackendUsed::Cpu);
    assert!(polar.fallback.is_none());
    assert_eq!(polar.points.len(), 2);
    for p in &polar.points {
        println!("M {} alpha {:.1} deg: CN {:.4} CA {:.4} x_cp {:.4} m {:?} iters {}", p.mach, p.alpha.to_degrees(), p.cn, p.ca_pressure, p.x_cp, p.quality, p.iterations);
        assert!(matches!(p.quality, Quality::Converged | Quality::Stationary));
        assert!(p.x_cp > 0.0 && p.x_cp < 10.0 * D, "x_cp {}", p.x_cp);
    }
    let (a, b) = (&polar.points[0], &polar.points[1]);
    assert!((a.alpha - 2f64.to_radians()).abs() < 1e-15 && (b.alpha - 6f64.to_radians()).abs() < 1e-15);
    assert!(b.cn > a.cn && a.cn > 0.0);
}

#[test]
fn run_lift_rejects_bad_input() {
    for (m, a) in [(3.5, 2.0), (0.2, 2.0), (2.0, 0.0), (2.0, -2.0), (2.0, 16.0), (f64::NAN, 2.0)] {
        assert!(matches!(analysis().run_lift(&[m], &[a]), Err(Error::InvalidInput(_))), "{m} {a}");
    }
    assert!(matches!(analysis().run_lift(&[], &[2.0]), Err(Error::InvalidInput(_))));
}

#[test]
fn gpu_without_feature_errors_and_auto_falls_back() {
    if cfg!(feature = "gpu") {
        return; // the GPU path is covered by the ignored `gpu_matches_cpu_tiny`
    }
    assert!(analysis().backend(Backend::Gpu).run_lift(&[2.0], &[2.0]).is_err());
    let p = analysis().backend(Backend::Auto).max_iterations(100).run_lift(&[2.0], &[2.0]).unwrap();
    assert_eq!(p.backend, BackendUsed::Cpu);
    assert!(p.fallback.is_some());
}

/// GPU (extended precision) against the CPU on the tiny case. Run through gpuq:
/// `cargo test --release -p bowshock-core --test run_lift -- --ignored --nocapture`.
#[test]
#[ignore = "needs a GPU: run through tools/gpuq/gpuq"]
fn gpu_matches_cpu_tiny() {
    let t = Instant::now();
    let cpu = analysis().run_lift(&[2.0], &[2.0, 6.0]).unwrap();
    let t_cpu = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let gpu = analysis().backend(Backend::Gpu).run_lift(&[2.0], &[2.0, 6.0]).unwrap();
    let t_gpu = t.elapsed().as_secs_f64();
    println!("wall: CPU {t_cpu:.1} s, GPU {t_gpu:.1} s");
    assert_eq!(gpu.backend, BackendUsed::Gpu);
    assert!(gpu.fallback.is_none());
    assert_eq!(gpu.points.len(), 2);
    for (c, g) in cpu.points.iter().zip(&gpu.points) {
        println!(
            "alpha {:.0}: CPU CN {:.5} x_cp {:.5} d {:?} {} it | GPU CN {:.5} x_cp {:.5} d {:?} {} it",
            c.alpha.to_degrees(), c.cn, c.x_cp / D, c.quality, c.iterations, g.cn, g.x_cp / D, g.quality, g.iterations
        );
        assert!(matches!(g.quality, Quality::Converged | Quality::Stationary));
        assert!((g.cn - c.cn).abs() <= 0.005 * c.cn.abs(), "CN {} vs {}", g.cn, c.cn);
        assert!((g.x_cp - c.x_cp).abs() <= 0.01 * D, "x_cp {} vs {}", g.x_cp / D, c.x_cp / D);
    }
}
