use bowshock::analysis::{Analysis, Backend, Resolution};
use bowshock::body_grid::{BaseBody, Body, Nose};
use bowshock::error::Error;

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
