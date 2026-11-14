use bowshock::analysis::{Analysis, Backend, Quality, Resolution, Warning};
use bowshock::body_grid::{BaseBody, Body, Nose};
use bowshock::error::Error;
use bowshock::spec::{AftSpec, BodySpec, NoseShape, NoseSpec};
use bowshock_reference::base_drag::base_drag_coefficient;

fn body() -> BaseBody {
    BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0)
}

fn analysis(body: BaseBody) -> Analysis {
    Analysis::from_internal(body).backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(300)
}

#[test]
fn points_come_back_in_input_order() {
    let polar = analysis(body()).run(&[0.9, 0.5]).unwrap();
    let machs: Vec<f64> = polar.points.iter().map(|p| p.mach).collect();
    assert_eq!(machs, [0.9, 0.5]);
    assert_eq!(polar.points[0].detail.mach, 0.9);
    assert_eq!(polar.points[1].detail.mach, 0.5);
}

#[test]
fn invalid_mach_lists_are_rejected() {
    for machs in [&[][..], &[7.0][..], &[f64::NAN][..]] {
        assert!(matches!(analysis(body()).run(machs), Err(Error::InvalidInput(_))), "{machs:?}");
    }
}

fn spec() -> BodySpec {
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.1 },
        aft: vec![AftSpec::Tube { length: 0.6 }, AftSpec::Transition { length: 0.1, aft_diameter: 0.08 }],
    }
}

#[test]
fn invalid_specs_are_rejected_with_the_field_named() {
    type Edit = fn(&mut BodySpec);
    let cases: [(&str, Edit); 15] = [
        ("nose.length", |s| s.nose.length = f64::NAN),
        ("nose.length", |s| s.nose.length = -0.3),
        ("nose.diameter", |s| s.nose.diameter = 0.0),
        ("nose.length", |s| s.nose.length = 1.5),  // fineness 15
        ("nose.length", |s| s.nose.length = 0.04), // fineness 0.4
        ("nose.shape", |s| {
            s.nose.shape = NoseShape::Cone;
            s.nose.length = 0.08; // half-angle 32 deg
        }),
        ("nose.shape.n", |s| s.nose.shape = NoseShape::Power { n: 0.2 }),
        ("nose.shape.k", |s| s.nose.shape = NoseShape::Parabolic { k: 1.5 }),
        ("nose.shape.c", |s| s.nose.shape = NoseShape::Haack { c: 0.5 }),
        ("aft", |s| s.aft.clear()),
        ("aft[0].length", |s| s.aft[0] = AftSpec::Tube { length: 0.0 }),
        ("aft[1].aft_diameter", |s| s.aft[1] = AftSpec::Transition { length: 0.1, aft_diameter: f64::INFINITY }),
        ("aft[1].aft_diameter", |s| s.aft[1] = AftSpec::Transition { length: 0.1, aft_diameter: -0.08 }),
        ("aft[1]", |s| s.aft[1] = AftSpec::Transition { length: 0.01, aft_diameter: 0.04 }), // 71 deg
        ("aft", |s| s.aft[0] = AftSpec::Tube { length: 5.0 }),                               // 53 diameters
    ];
    for (field, edit) in cases {
        let mut s = spec();
        edit(&mut s);
        match Analysis::new(&s) {
            Err(Error::InvalidInput(msg)) => assert!(msg.starts_with(field), "expected `{field}`, got `{msg}`"),
            other => panic!("expected InvalidInput for {field}, got {:?}", other.map(|_| ())),
        }
    }
    assert!(Analysis::new(&spec()).is_ok());
}

#[test]
fn drag_is_referenced_to_the_largest_cross_section() {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.24, diameter: 0.08 },
        aft: vec![AftSpec::Tube { length: 0.2 }, AftSpec::Transition { length: 0.1, aft_diameter: 0.10 }],
    };
    assert_eq!(spec.reference_diameter(), 0.10);
    let (internal, _) = spec.to_internal().unwrap();
    let mach = 0.6;
    let polar = Analysis::new(&spec).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(100).run(&[mach]).unwrap();
    let p = &polar.points[0];
    let base = base_drag_coefficient(mach) * (internal.base_radius() / internal.body.radius()).powi(2);
    let internal_total = p.detail.cd_aft.iter().fold(p.detail.cd_nose, |a, c| a + c) + base;
    let expected = internal_total * 0.64;
    assert!((p.drag.total - expected).abs() <= 1e-12 * expected.abs(), "{} vs {expected}", p.drag.total);
}

fn cone_body() -> BaseBody {
    BaseBody::cylinder(Body { nose: Nose::Cone { half_angle: (0.5f64 / 3.0).atan() } }, 6.0)
}

#[test]
fn iteration_cap_gives_stalled() {
    let polar = analysis(body()).max_iterations(60).run(&[0.9]).unwrap();
    assert_eq!(polar.points[0].quality, Quality::Stalled);
}

#[test]
fn sharp_corner_warning_only_for_the_cone() {
    let cone = analysis(cone_body()).max_iterations(30).run(&[0.9]).unwrap();
    assert!(cone.points[0].warnings.contains(&Warning::SharpCornerTransonic), "{:?}", cone.points[0].warnings);
    let ogive = analysis(body()).max_iterations(30).run(&[0.9]).unwrap();
    assert!(!ogive.points[0].warnings.contains(&Warning::SharpCornerTransonic), "{:?}", ogive.points[0].warnings);
}

#[test]
fn uncertainty_is_reported_only_when_asked() {
    let off = analysis(body()).run(&[0.6]).unwrap();
    assert_eq!(off.points[0].cd_uncertainty, None);
    let on = analysis(body()).max_iterations(200).estimate_uncertainty(true).run(&[0.6]).unwrap();
    let u = on.points[0].cd_uncertainty.expect("both levels finite");
    assert!(u.is_finite(), "{u}");
}
