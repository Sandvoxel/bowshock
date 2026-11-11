use bowshock::analysis::{Analysis, Backend, Resolution};
use bowshock::body_grid::{BaseBody, Body, Nose};
use bowshock::case::Wake;
use bowshock::error::Error;

fn body(boat_length: f64) -> BaseBody {
    BaseBody {
        body: Body { nose: Nose::TangentOgive { fineness: 3.0 } },
        cyl_length: 6.0,
        boat_length,
        boat_angle: if boat_length > 0.0 { 7f64.to_radians() } else { 0.0 },
        sting_radius: 0.0,
    }
}

fn analysis(body: BaseBody) -> Analysis {
    Analysis::from_internal(body).backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(300)
}

#[test]
fn points_come_back_in_input_order() {
    let polar = analysis(body(0.0)).run(&[0.9, 0.5]).unwrap();
    let machs: Vec<f64> = polar.points.iter().map(|p| p.mach).collect();
    assert_eq!(machs, [0.9, 0.5]);
    assert_eq!(polar.points[0].detail.mach, 0.9);
    assert_eq!(polar.points[1].detail.mach, 0.5);
}

#[test]
fn invalid_mach_lists_are_rejected() {
    for machs in [&[][..], &[7.0][..], &[f64::NAN][..]] {
        assert!(matches!(analysis(body(0.0)).run(machs), Err(Error::InvalidInput(_))), "{machs:?}");
    }
}

#[test]
fn boattail_with_sting_is_unsupported() {
    let r = analysis(body(1.0)).wake(Wake::Sting).run(&[0.5]);
    assert!(matches!(r, Err(Error::Unsupported(_))));
}
