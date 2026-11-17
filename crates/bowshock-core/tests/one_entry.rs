//! `Analysis::run`, the one solve entry: conditions at zero incidence go to the drag solver and those at an angle of attack to the 3D
//! solver. The numbers it gives are those of the two entries it replaced (`run` over Mach numbers and `run_lift`), bit for bit on the CPU:
//! the constants below were taken from those two functions at the commit before they were merged (the drag numbers are the same at the
//! commit before and after the 3D topology of `cfd-3d` S1, 65c1d78, whose force integration moved the 3D numbers by a few last bits; the
//! angle-of-attack constants are S1's: `Analysis::run_lift` of 65c1d78).

use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::{
    AftSpec, Analysis, Atmosphere, Backend, BodySpec, Condition, FinProfile, FinSpec, FlightConditions, NoseShape, NoseSpec, Quality, Resolution,
};
use std::sync::{Arc, Mutex};

const D: f64 = 0.04;

/// The ANF-like rocket on the tiny 3D grid (20 k cells): 10 d long, cone nose, tube, four square fins, CPU.
fn finned_3d() -> Analysis {
    with_3d_grid(true)
}

/// The same without the fins: the drag solver takes fins only with flight conditions.
fn bare_3d() -> Analysis {
    with_3d_grid(false)
}

fn with_3d_grid(fins: bool) -> Analysis {
    let nose_len = 0.5 * D / 10f64.to_radians().tan();
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: D }, aft: vec![AftSpec::Tube { length: 10.0 * D - nose_len }] };
    let fins_spec = FinSpec {
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
    let analysis = Analysis::new(&spec).unwrap();
    let analysis = if fins { analysis.fins(fins_spec) } else { analysis };
    analysis.backend(Backend::Cpu).resolution(Resolution::Coarse).case3_hook(|c| {
        let s = &mut c.grid.sting;
        s.n_up = 10;
        s.n_nose = 16;
        s.n_seg = vec![12];
        s.n_ext = 10;
        s.nj = 24;
        c.grid.nk = 8;
    })
}

fn bits(x: f64) -> u64 {
    x.to_bits()
}

/// Equal including the NaNs a point at an angle of attack holds (`NaN != NaN` makes `==` of two equal polars false).
fn same<T: std::fmt::Debug>(a: &T, b: &T) -> bool {
    format!("{a:?}") == format!("{b:?}")
}

/// The point without its residual drop and its field (kept fields are compared on their own): the 3D solver's residual is a parallel sum whose last bits depend on how the threads were
/// scheduled, so the orders of drop agree to many digits but not to the bit, while the state, the forces and the iteration count do.
fn settled(p: &bowshock_core::PolarPoint) -> bowshock_core::PolarPoint {
    let mut p = p.clone();
    assert!(p.detail.residual_drop_orders.is_finite());
    p.detail.residual_drop_orders = 0.0;
    p.field3 = None;
    p
}

fn same_points(a: &bowshock_core::Polar, b: &bowshock_core::Polar) -> bool {
    a.points.len() == b.points.len()
        && a.points.iter().zip(&b.points).all(|(x, y)| {
            let close = (x.detail.residual_drop_orders - y.detail.residual_drop_orders).abs() <= 1e-9 * x.detail.residual_drop_orders.abs().max(1.0);
            close && same(&settled(x), &settled(y))
        })
}

#[test]
fn a_euler_drag_sweep_gives_the_numbers_of_the_old_entry_to_the_last_bit() {
    let a = Analysis::from_internal(BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0))
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .max_iterations(300);
    let p = a.run(&[0.9, 0.5]).unwrap();
    // (Mach, total, forebody pressure, base): the points come back in the order asked for
    let want = [(0.9, 0x3fd2c5aa7e63f472u64, 0x3fb1696735c5e2fau64, 0x3fccd6a161e4f766u64), (0.5, 0x3fc3288fe1495d5c, 0xbf6723b5c2238a4c, 0x3fc3851eb851eb85)];
    for (pt, (mach, total, fore, base)) in p.points.iter().zip(want) {
        assert_eq!((pt.mach, pt.alpha_deg, pt.forces.is_none()), (mach, 0.0, true));
        assert_eq!((bits(pt.drag.total), bits(pt.drag.forebody_pressure), bits(pt.drag.base)), (total, fore, base), "Mach {mach}");
        assert_eq!((pt.detail.iterations, pt.quality), (300, Quality::Stalled));
    }
    // an explicit condition at zero incidence is the same run
    let q = a.run(&[Condition::new(0.9, 0.0), Condition::new(0.5, 0.0)]).unwrap();
    assert!(same(&p, &q));
}

#[test]
fn a_viscous_finned_drag_sweep_gives_the_numbers_of_the_old_entry_to_the_last_bit() {
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.1 }, aft: vec![AftSpec::Tube { length: 0.6 }] };
    let fins = FinSpec {
        count: 4,
        root_chord: 0.12,
        tip_chord: 0.06,
        span: 0.08,
        sweep: 0.06,
        thickness: 0.003,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Rounded,
    };
    let p = Analysis::new(&spec)
        .unwrap()
        .fins(fins)
        .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 120.0 } })
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .max_iterations(60)
        .run(&[1.5, 0.6])
        .unwrap();
    // total, forebody, friction, base, fin friction, fin pressure
    let want = [
        (1.5, [0x3ff187f3eee11dd8u64, 0x3fc4aa734fd25473, 0x3fdc8a6fdab47eb2, 0x3fd80c6630d48751, 0x3f97b2266d865690, 0x3fb6e27684e786ed]),
        (0.6, [0x3ffb1f25f27ee5ef, 0x3fc614ae86bc4a07, 0x3fee5d42aa226f95, 0x3fe084e8fa0b2dad, 0x3fa005d63150d6cf, 0x3f9ad2e78181d5bb]),
    ];
    for (pt, (mach, w)) in p.points.iter().zip(want) {
        let d = &pt.drag;
        let got = [d.total, d.forebody_pressure, d.friction.unwrap(), d.base, d.fins.unwrap().friction, d.fins.unwrap().pressure].map(bits);
        assert_eq!((pt.mach, got), (mach, w));
        assert_eq!((pt.detail.iterations, pt.quality), (60, Quality::Stalled));
    }
}

#[test]
fn conditions_at_an_angle_of_attack_give_the_numbers_of_the_old_lift_entry_to_the_last_bit() {
    // asked for 6 degrees first: solved 2 degrees cold, then 6 degrees warm; the points come back in the order asked for
    let p = finned_3d().max_iterations(3000).run(&[Condition::new(2.0, 6.0), Condition::new(2.0, 2.0)]).unwrap();
    // (angle, CN, CA of the pressure, x_cp [m], iterations)
    let want = [
        (6.0, 0x3ff394c225ec3ddeu64, 0x3ff01b9bec4d022eu64, 0x3fd17e5353844eafu64, 318),
        (2.0, 0x3fd85d6ebf7557ce, 0x3ff05c45835ed819, 0x3fd21cc7180c0df5, 291),
    ];
    for (pt, (alpha, cn, ca, x_cp, iters)) in p.points.iter().zip(want) {
        let f = pt.forces.expect("forces at an angle of attack");
        assert_eq!((pt.mach, pt.alpha_deg), (2.0, alpha));
        assert_eq!((bits(f.cn), bits(f.ca), bits(f.x_cp)), (cn, ca, x_cp), "alpha {alpha}");
        assert_eq!((pt.detail.iterations, pt.quality), (iters, Quality::Converged));
        // what a point at an angle of attack holds besides its forces (see PolarPoint)
        assert_eq!(bits(pt.drag.forebody_pressure), bits(f.ca));
        assert!(pt.drag.friction.is_none() && pt.drag.fins.is_none() && pt.drag.base.is_nan() && pt.drag.total.is_nan() && !pt.drag.base_modelled);
        assert!(pt.detail.cd_nose.is_nan() && pt.detail.cp_base.is_nan() && pt.detail.y_plus_max.is_nan() && pt.detail.cd_base_solver.is_nan());
        assert!(pt.detail.cd_aft.is_empty() && pt.detail.wall.is_empty() && pt.warnings.is_empty() && pt.cd_uncertainty.is_none());
        assert_eq!((pt.detail.cd_friction, pt.detail.farfield_dp), (0.0, 0.0));
        // the moment about the nose tip: a normal force behind it pitches the nose down
        assert!((f.cm + f.cn * f.x_cp / D).abs() < 1e-12 * f.cn.abs() && f.cm < 0.0, "cm {} cn {} x_cp {}", f.cm, f.cn, f.x_cp);
    }
    // the same conditions in the other order are the same run
    let q = finned_3d().max_iterations(3000).run(&[Condition::new(2.0, 2.0), Condition::new(2.0, 6.0)]).unwrap();
    for (a, b) in [(&q.points[1], &p.points[0]), (&q.points[0], &p.points[1])] {
        assert!(same(&settled(a), &settled(b)), "{a:?} vs {b:?}");
    }
}

#[test]
fn conditions_of_both_kinds_come_back_each_in_its_place() {
    let p = bare_3d().max_iterations(200).run(&[Condition::new(2.0, 6.0), Condition::from(0.9), Condition::new(2.0, 2.0)]).unwrap();
    let got: Vec<(f64, f64, bool)> = p.points.iter().map(|pt| (pt.mach, pt.alpha_deg, pt.forces.is_some())).collect();
    assert_eq!(got, [(2.0, 6.0, true), (0.9, 0.0, false), (2.0, 2.0, true)]);
    assert!(p.points[1].drag.total.is_finite() && p.points[1].drag.base_modelled);
    assert_eq!(p.backend, bowshock_core::BackendUsed::Cpu);
}

#[test]
fn the_final_field_of_each_3d_point_is_kept_on_request_and_changes_nothing() {
    let conditions = [Condition::new(2.0, 2.0), Condition::new(2.0, 6.0)];
    let plain = finned_3d().max_iterations(250).run(&conditions).unwrap();
    assert!(plain.points.iter().all(|p| p.field3.is_none() && p.field.is_none()));
    let kept = finned_3d().max_iterations(250).keep_fields(true).run(&conditions).unwrap();
    for (a, b) in plain.points.iter().zip(&kept.points) {
        assert_eq!(a.forces, b.forces);
        assert!(same(&settled(a), &settled(b)));
        let f = b.field3.as_ref().expect("the field of the point");
        assert_eq!((f.mach, f.iteration), (2.0, b.detail.iterations));
        assert!((f.alpha_deg - b.alpha_deg).abs() < 1e-12, "{} vs {}", f.alpha_deg, b.alpha_deg);
        assert!(f.nk > 1 && f.cells() > 10_000 && !f.walls.is_empty() && !f.fin_theta.is_empty());
        // metres: the body is 10 d long
        let x_max = f.nodes.iter().map(|n| n[0]).fold(0.0f32, f32::max);
        assert!(f64::from(x_max) > 10.0 * D, "{x_max}");
    }
    // one grid for both angles, two flows
    let (a, b) = (kept.points[0].field3.as_ref().unwrap(), kept.points[1].field3.as_ref().unwrap());
    assert_eq!(a.mesh_id, b.mesh_id);
    assert!(a.prim != b.prim);
}

#[test]
fn snapshots_of_a_3d_run_come_every_interval_and_change_nothing() {
    let conditions = [Condition::new(2.0, 4.0)];
    let plain = finned_3d().max_iterations(250).run(&conditions).unwrap();
    let seen = Arc::new(Mutex::new(Vec::new()));
    let log = seen.clone();
    let snapped = finned_3d().max_iterations(250).on_field3(100, move |f| log.lock().unwrap().push(f)).run(&conditions).unwrap();
    assert!(same_points(&plain, &snapped), "a snapshot does not change the iteration or the result");
    let seen = seen.lock().unwrap();
    assert_eq!(seen.iter().map(|f| f.iteration).collect::<Vec<_>>(), [100, 200], "every 100 iterations, none at the end");
    for f in seen.iter() {
        assert_eq!(f.mach, 2.0);
        assert!((f.alpha_deg - 4.0).abs() < 1e-12);
    }
    assert_eq!(seen[0].mesh_id, seen[1].mesh_id);
    assert!(seen[0].prim != seen[1].prim, "the flow develops between snapshots");
    // an interval that is not a multiple of the sampling interval is rounded up
    let rounded = Arc::new(Mutex::new(Vec::new()));
    let log = rounded.clone();
    finned_3d().max_iterations(120).on_field3(60, move |f| log.lock().unwrap().push(f.iteration)).run(&conditions).unwrap();
    assert_eq!(*rounded.lock().unwrap(), [100]);
}

#[test]
fn a_float_is_a_condition_at_zero_incidence() {
    assert_eq!(Condition::from(0.7), Condition::new(0.7, 0.0));
    let c: Condition = 2.5.into();
    assert_eq!((c.mach, c.alpha_deg), (2.5, 0.0));
}
