//! The flow field of a point: kept only when asked for, on the grid the drag was computed on, and handed out while a run is in flight.

use bowshock_core::field::GAMMA;
use bowshock_core::{AftSpec, Analysis, Backend, BodySpec, Field, NoseShape, NoseSpec, Resolution, Wake};
use std::sync::{Arc, Mutex};

const DIAMETER: f64 = 0.10;
const NOSE: f64 = 0.30;
const TUBE: f64 = 0.60;

fn body() -> BodySpec {
    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: NOSE, diameter: DIAMETER }, aft: vec![AftSpec::Tube { length: TUBE }] }
}

/// A short Euler run on the coarsest grid: far from converged, but a real solver state.
fn analysis() -> Analysis {
    Analysis::legacy(&body()).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(300)
}

#[test]
fn a_kept_field_is_on_the_grid_of_the_point_at_the_free_stream_far_away_and_gives_the_wall_cp() {
    let mach = 2.0;
    let polar = analysis().keep_fields(true).run(&[mach]).unwrap();
    let point = &polar.points[0];
    let f = point.field.as_ref().expect("keep_fields keeps the field");

    // the grid of the point: the sting grid of an Euler analysis, nothing blocked
    let (ni, nj) = analysis().engine_cases(&[mach]).unwrap()[0].cells();
    assert_eq!((f.ni, f.nj), (ni, nj));
    assert_eq!(f.wake, Wake::Sting);
    assert_eq!(f.x.len(), (ni + 1) * (nj + 1));
    assert_eq!(f.r.len(), f.x.len());
    for cells in [f.rho.len(), f.u.len(), f.v.len(), f.p.len(), f.solid.len()] {
        assert_eq!(cells, ni * nj);
    }
    assert!(f.solid.iter().all(|s| !s));
    assert_eq!(f.iteration, point.detail.iterations);
    assert_eq!(f.mach, mach);
    assert_eq!(f.freestream, [1.0, mach, 0.0, 1.0 / GAMMA]);

    // metres from the nose tip: the inflow plane is upstream of the tip, the wall row passes through the shoulder
    let surface = f.surface();
    assert!(surface[0][0] < 0.0 && surface[0][1] == 0.0, "the line starts on the axis at the inflow plane: {:?}", surface[0]);
    assert!(surface.iter().any(|p| (f64::from(p[0]) - NOSE).abs() < 1e-6 && (f64::from(p[1]) - 0.5 * DIAMETER).abs() < 1e-6), "the shoulder is a node");

    // the far field (the inflow column and the outer row) is the free stream
    let machs = f.mach_numbers();
    let cps = f.pressure_coefficients();
    for c in (0..nj).map(|j| j * ni).chain((0..ni).map(|i| (nj - 1) * ni + i)) {
        assert!((f64::from(machs[c]) - mach).abs() < 1e-3 * mach, "cell {c}: Mach {}", machs[c]);
        assert!(cps[c].abs() < 1e-3, "cell {c}: Cp {}", cps[c]);
    }

    // the wall Cp of the point is the pressure of the two cells at the wall, extrapolated to the face: the field gives it again
    assert!(point.detail.wall.len() > 40);
    let wall = f.wall();
    let mut worst: f64 = 0.0;
    for w in &point.detail.wall {
        let mine = wall.iter().find(|m| (m.x - w.x).abs() < 1e-6).unwrap_or_else(|| panic!("no wall face at x = {}", w.x));
        assert!((mine.r - w.r).abs() < 1e-6);
        worst = worst.max((mine.cp - w.cp).abs());
        // the first cell alone is already close
        let i = (0..ni).find(|&i| (0.5 * f64::from(f.node(i, 0).0 + f.node(i + 1, 0).0) - w.x).abs() < 1e-6).unwrap();
        assert!((f64::from(cps[i]) - w.cp).abs() < 0.1, "x = {}: cell Cp {} against wall Cp {}", w.x, cps[i], w.cp);
    }
    assert!(worst < 2e-3, "extrapolated Cp of the field against the reported wall Cp: {worst}");
}

#[test]
fn no_field_is_kept_unless_asked_for_and_keeping_one_changes_nothing() {
    let plain = analysis().run(&[0.8]).unwrap();
    assert!(plain.points[0].field.is_none());
    let kept = analysis().keep_fields(true).run(&[0.8]).unwrap();
    assert!(kept.points[0].field.is_some());
    assert_eq!(kept.points[0].drag, plain.points[0].drag);
    // (the details hold NaN, which is not equal to itself: compare their printed form)
    assert_eq!(format!("{:?}", kept.points[0].detail), format!("{:?}", plain.points[0].detail));
    // the spec carries the choice, so a rebuilt analysis (a remote worker) keeps fields too
    assert!(analysis().keep_fields(true).spec().unwrap().keep_fields);
    assert!(!analysis().spec().unwrap().keep_fields);
}

#[test]
fn the_cells_inside_the_body_of_a_base_wake_grid_are_blocked() {
    let polar = analysis().wake(Wake::Base).max_iterations(100).keep_fields(true).run(&[2.0]).unwrap();
    let f = polar.points[0].field.as_ref().unwrap();
    assert_eq!(f.wake, Wake::Base);
    let (ni, nj) = (f.ni, f.nj);
    let blocked = f.solid.iter().filter(|s| **s).count();
    assert!(blocked > 0 && blocked < ni * nj);
    let (machs, grad) = (f.mach_numbers(), f.density_gradient());
    let length = NOSE + TUBE;
    let mut wake_cells = 0;
    for j in 0..nj {
        for i in 0..ni {
            let c = j * ni + i;
            let (x, r) = f.centre(i, j);
            let (x, r) = (f64::from(x), f64::from(r));
            if f.solid[c] {
                // inside the body (or, ahead of the tip, squeezed below the axis): not flow, flagged and without derived values
                assert!(x < length && r < 0.5 * DIAMETER, "blocked cell at ({x}, {r})");
                assert!(machs[c].is_nan() && grad[c].is_nan());
                assert_eq!([f.rho[c], f.u[c], f.v[c]], [1.0, 2.0, 0.0], "a blocked cell holds the free stream");
            } else {
                assert!(r > 0.0, "a flow cell below the axis at ({x}, {r})");
                assert!(machs[c].is_finite() && grad[c].is_finite(), "cell ({i}, {j})");
                // beside the tube every flow cell is outside the wall
                assert!(!(x > NOSE && x < length) || r > 0.5 * DIAMETER, "a flow cell inside the tube at ({x}, {r})");
                wake_cells += usize::from(x > length && r < 0.5 * DIAMETER);
            }
        }
    }
    assert!(wake_cells > 0, "the rows below the base radius carry the wake behind the base");
    // the lower edge of the flow: up the nose, along the tube, down the flat base to the axis
    let surface = f.surface();
    let at_base: Vec<f64> = surface.iter().filter(|p| (f64::from(p[0]) - length).abs() < 1e-6).map(|p| f64::from(p[1])).collect();
    assert!(at_base.len() >= 2, "the base is a step: {at_base:?}");
    assert!((at_base[0] - 0.5 * DIAMETER).abs() < 1e-6 && at_base[at_base.len() - 1].abs() < 1e-6, "{at_base:?}");
}

#[test]
fn snapshots_of_the_run_in_flight_arrive_at_the_interval_and_do_not_change_the_result() {
    let seen: Arc<Mutex<Vec<Field>>> = Arc::default();
    let log = seen.clone();
    let watched = analysis().on_field(100, move |f| log.lock().unwrap().push(f)).run(&[2.0]).unwrap();
    let silent = analysis().run(&[2.0]).unwrap();
    assert_eq!(watched.points[0].drag, silent.points[0].drag);
    assert_eq!(format!("{:?}", watched.points[0].detail), format!("{:?}", silent.points[0].detail), "the details hold NaN: compared as printed");
    assert!(watched.points[0].field.is_none(), "snapshots do not keep the final field");

    let seen = seen.lock().unwrap();
    let iterations = silent.points[0].detail.iterations;
    assert_eq!(iterations, 300, "the short run ends at its cap");
    assert_eq!(seen.iter().map(|f| f.iteration).collect::<Vec<_>>(), vec![100, 200, 300]);
    let (ni, nj) = analysis().engine_cases(&[2.0]).unwrap()[0].cells();
    assert!(seen.iter().all(|f| (f.ni, f.nj) == (ni, nj) && f.mach == 2.0));
    // the flow develops: the state at the nose moves between snapshots
    let nose = (0..ni).find(|&i| f64::from(seen[0].centre(i, 0).0) > 0.5 * NOSE).unwrap();
    assert_ne!(seen[0].p[nose], seen[2].p[nose]);

    // the grid with the free stream on it, without a solve
    let start = analysis().initial_field(2.0).unwrap();
    assert_eq!((start.ni, start.nj, start.iteration), (ni, nj, 0));
    assert_eq!(start.x, seen[0].x);
    assert!(start.mach_numbers().iter().all(|m| (m - 2.0).abs() < 1e-6));
}
