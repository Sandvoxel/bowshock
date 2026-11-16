//! The progress callback: what it reports, that it does not change a run, and that its `Break` ends one.

use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::progress::SAMPLE_INTERVAL;
use bowshock_core::{AftSpec, Analysis, Backend, BodySpec, Error, FinProfile, FinSpec, NoseShape, NoseSpec, Progress, Resolution};
use std::ops::ControlFlow;
use std::sync::{Arc, Mutex};

fn drag_analysis() -> Analysis {
    let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0);
    Analysis::from_internal(body).backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(300)
}

/// A sink that keeps every report and breaks once `stop_at` reports are in (never when `None`).
fn recorder(stop_at: Option<usize>) -> (Arc<Mutex<Vec<Progress>>>, impl Fn(&Progress) -> ControlFlow<()> + Send + Sync + 'static) {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let log = seen.clone();
    (seen, move |p: &Progress| {
        let mut log = log.lock().unwrap();
        log.push(*p);
        if stop_at.is_some_and(|n| log.len() >= n) {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    })
}

#[test]
fn a_drag_run_reports_every_sample_interval_and_the_result_is_unchanged() {
    let (seen, sink) = recorder(None);
    let reported = drag_analysis().on_progress(sink).run(&[0.9]).unwrap();
    let silent = drag_analysis().run(&[0.9]).unwrap();
    // reporting reads numbers the solver has; it changes nothing
    assert_eq!(reported.points[0].drag, silent.points[0].drag);
    assert_eq!(reported.points[0].detail.iterations, silent.points[0].detail.iterations);

    let seen = seen.lock().unwrap();
    let iterations = silent.points[0].detail.iterations;
    assert_eq!(seen.len(), iterations / SAMPLE_INTERVAL, "one report per {SAMPLE_INTERVAL} iterations, none for a final partial chunk");
    for (k, p) in seen.iter().enumerate() {
        assert_eq!(p.iteration, (k + 1) * SAMPLE_INTERVAL);
        assert_eq!(p.mach, 0.9);
        assert_eq!(p.alpha, 0.0);
        assert_eq!(p.max_iterations, 300);
        assert!(p.residual_drop_orders.is_finite() && p.residual_drop_orders >= 0.0);
    }
    // the Euler sting body has no drag tolerance
    assert!(seen.iter().all(|p| p.tolerance.is_none()));
}

#[test]
fn a_break_ends_the_analysis_with_cancelled_after_that_report() {
    let (seen, sink) = recorder(Some(2));
    let result = drag_analysis().on_progress(sink).run(&[0.9, 1.5]);
    assert!(matches!(result, Err(Error::Cancelled)), "{result:?}");
    let seen = seen.lock().unwrap();
    assert_eq!(seen.len(), 2, "no iteration runs after the report that said stop, and the second Mach number never starts");
    assert!(seen.iter().all(|p| p.mach == 0.9));
}

fn lift_analysis() -> Analysis {
    let d = 0.04;
    let nose_len = 0.5 * d / 10f64.to_radians().tan();
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: d }, aft: vec![AftSpec::Tube { length: 10.0 * d - nose_len }] };
    let fins = FinSpec {
        count: 4,
        root_chord: d,
        tip_chord: d,
        span: d,
        sweep: 0.0,
        thickness: 0.08 * d,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Square,
    };
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
        .max_iterations(200)
}

#[test]
fn a_3d_run_reports_its_angle_and_a_break_stops_the_sweep() {
    let (seen, sink) = recorder(Some(3));
    let result = lift_analysis().on_progress(sink).run_lift(&[2.0], &[2.0, 6.0]);
    assert!(matches!(result, Err(Error::Cancelled)), "{result:?}");
    let seen = seen.lock().unwrap();
    assert_eq!(seen.len(), 3);
    assert_eq!(seen.iter().map(|p| p.iteration).collect::<Vec<_>>(), [50, 100, 150]);
    for p in seen.iter() {
        assert!((p.alpha - 2f64.to_radians()).abs() < 1e-15);
        assert_eq!(p.mach, 2.0);
        assert_eq!(p.max_iterations, 200);
        assert_eq!(p.tolerance, Some(0.002));
        assert!(p.monitor.is_finite(), "CN is sampled at every report");
    }
}

#[test]
fn the_gpu_name_is_a_name_or_nothing() {
    if let Some(name) = bowshock_core::gpu_name() {
        assert!(!name.trim().is_empty());
        println!("GPU: {name}");
    }
}
