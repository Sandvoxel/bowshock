use bowshock::analysis::{Analysis, Backend, Quality, Resolution};
use bowshock::spec::{AftSpec, BodySpec, NoseShape, NoseSpec};

fn nose(shape: NoseShape, length: f64, diameter: f64) -> NoseSpec {
    NoseSpec { shape, length, diameter }
}

/// Tangent ogive 0.30 x 0.10, tube 0.60, transition 0.10 to diameter 0.08.
fn body_a() -> BodySpec {
    BodySpec {
        nose: nose(NoseShape::TangentOgive, 0.30, 0.10),
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    }
}

/// Cone 0.25 x 0.08, tube 0.30, transition 0.08 to diameter 0.10, tube 0.40.
fn body_b() -> BodySpec {
    BodySpec {
        nose: nose(NoseShape::Cone, 0.25, 0.08),
        aft: vec![AftSpec::Tube { length: 0.30 }, AftSpec::Transition { length: 0.08, aft_diameter: 0.10 }, AftSpec::Tube { length: 0.40 }],
    }
}

/// Haack c = 0 0.40 x 0.10, tube 0.80, transition 0.12 to diameter 0.07.
fn body_c() -> BodySpec {
    BodySpec {
        nose: nose(NoseShape::Haack { c: 0.0 }, 0.40, 0.10),
        aft: vec![AftSpec::Tube { length: 0.80 }, AftSpec::Transition { length: 0.12, aft_diameter: 0.07 }],
    }
}

#[test]
fn small_matrix_never_fails() {
    let machs = [0.5, 0.95, 1.6];
    for (name, spec) in [("A", body_a()), ("B", body_b())] {
        let polar = Analysis::new(&spec)
            .unwrap()
            .backend(Backend::Cpu)
            .resolution(Resolution::Coarse)
            .max_iterations(1500)
            .run(&machs)
            .unwrap_or_else(|e| panic!("body {name}: {e}"));
        assert_eq!(polar.points.len(), machs.len());
        for (k, p) in polar.points.iter().enumerate() {
            assert_eq!(p.mach, machs[k], "body {name}: order");
            assert_ne!(p.quality, Quality::Failed, "body {name} M{}: {:?}", p.mach, p.detail.stop_reason);
            assert!(p.drag.total.is_finite() && p.drag.total > 0.0, "body {name} M{}: total {}", p.mach, p.drag.total);
        }
    }
}

#[test]
#[ignore]
fn full_matrix() {
    let machs = [0.3, 0.6, 0.8, 0.9, 0.95, 1.05, 1.2, 1.5, 2.0, 2.5];
    let mut failed = 0;
    let mut good = 0;
    for (name, spec) in [("A", body_a()), ("B", body_b()), ("C", body_c())] {
        let polar =
            Analysis::new(&spec).unwrap().backend(Backend::Auto).resolution(Resolution::Coarse).run(&machs).unwrap_or_else(|e| panic!("body {name}: {e}"));
        println!("body {name}: backend {:?}, fallback {:?}", polar.backend, polar.fallback);
        for p in &polar.points {
            println!(
                "{name} M{:<5} {:<10?} {:<26} iters {:>6} fore {:>9.5} base {:>9.5} total {:>9.5} {:?}",
                p.mach,
                p.quality,
                p.detail.stop_reason.to_string(),
                p.detail.iters,
                p.drag.forebody_pressure,
                p.drag.base,
                p.drag.total,
                p.warnings
            );
            match p.quality {
                Quality::Failed => failed += 1,
                Quality::Converged | Quality::Stationary => good += 1,
                Quality::Stalled => {}
            }
        }
    }
    assert_eq!(failed, 0, "failed points");
    assert!(good >= 24, "only {good} of 30 points converged or stationary");
}
