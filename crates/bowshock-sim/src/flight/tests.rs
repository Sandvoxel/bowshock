use super::*;
use crate::aero::BODY_CROSSFLOW_CD;
use crate::attitude::from_axis;
use crate::staging::Hits;
use crate::vehicle::*;
use crate::G0;

/// A 38 mm rocket with four fins and a 0.23 kg motor holding 0.105 kg of propellant.
fn vehicle() -> Vehicle {
    let r = 0.038;
    Vehicle {
        reference_diameter: 2.0 * r,
        body: vec![
            BodySegment { station: 0.0, length: 0.30, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::TangentOgive), roughness: 30e-6 },
            BodySegment { station: 0.30, length: 0.80, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 30e-6 },
        ],
        tube_fins: vec![],
        fin_sets: vec![FinSet {
            count: 4,
            station: 0.93,
            body_radius: r,
            root_chord: 0.17,
            tip_chord: 0.07,
            span: 0.10,
            sweep: 0.09,
            thickness: 0.003,
            cross_section: FinCrossSection::Rounded,
            cant: 0.0,
            roughness: 30e-6,
            planform: FinPlanform::Trapezoidal,
            angles: Vec::new(),
            parent: 0,
            bodies: 1,
            mid_chord_cos: None,
            root_extent: None,
        }],
        protuberances: vec![],
        stages: vec![StageMass {
            dry: MassProps { mass: 1.07, cg: 0.62, i_long: 0.1, i_rot: 0.0008 },
            fore_station: 0.0,
            aft_station: 1.10,
            motor: Some(StageMotor {
                curve: ThrustCurve {
                    points: vec![(0.0, 0.0), (0.05, 150.0), (1.0, 120.0), (1.2, 0.0)],
                    propellant_mass: 0.105,
                    total_mass: 0.230,
                    length: 0.205,
                    diameter: 0.038,
                },
                mount_aft_station: 1.10,
                ignition: Timing::LAUNCH,
                ejection_delay: None,
                tilt_lockout: None,
            }),
            separation: Timing::BURNOUT,
        }],
        labels: Default::default(),
    }
}

fn with_ctx<R>(v: Vehicle, f: impl FnOnce(&Ctx<'_>) -> R) -> R {
    let input = SimInput::new(v, Launch::default(), vec![]);
    let aeros = build_aeros(&input);
    let n = input.vehicle.stages.len();
    let ctx = Ctx {
        input: &input,
        launch: &input.launch,
        tw: Tweaks::NOMINAL,
        aeros: &aeros,
        atm: Atmosphere::new(0.0, 288.15),
        sched: Timeline::resolve(&input.vehicle, &Hits::none(n)),
        wind: input.launch.wind_profile(),
        axis: [0.0, 0.0, 1.0],
        a_ref: input.vehicle.reference_area(),
        g_surface: G0,
        omega: None,
        controlled: vec![Vec::new()],
        deflection: Vec::new(),
        control_cant: 0.0,
    };
    f(&ctx)
}

#[test]
fn inertia_is_the_parallel_axis_sum_and_the_propellant_part_falls_as_it_burns() {
    with_ctx(vehicle(), |ctx| {
        let st = &ctx.input.vehicle.stages[0];
        let (dry, mo) = (st.dry, st.motor.as_ref().unwrap());
        let (r, l, xm) = (0.019, 0.205, mo.cg_station());
        let (casing, prop) = (0.230 - 0.105, 0.105);
        // Before ignition: the full motor is a dead weight.
        let p0 = ctx.mass_props(-5.0, 0);
        let m = dry.mass + 0.230;
        let cg = (dry.mass * dry.cg + 0.230 * xm) / m;
        assert!((p0.mass - m).abs() < 1e-12 && (p0.cg - cg).abs() < 1e-12 && p0.thrust == 0.0 && p0.jet == 0.0);
        let motor_long = casing * (0.5 * r * r + l * l / 12.0) + prop * (0.25 * r * r + l * l / 12.0);
        let hand = dry.i_long + dry.mass * (dry.cg - cg).powi(2) + motor_long + 0.230 * (xm - cg).powi(2);
        assert!((p0.i_long - hand).abs() < 1e-9, "{} vs {hand}", p0.i_long);
        assert!((p0.i_rot - (dry.i_rot + casing * r * r + 0.5 * prop * r * r)).abs() < 1e-12);
        // After burnout: only the casing's roll inertia is left, the propellant's half m r^2 is gone.
        let p1 = ctx.mass_props(10.0, 0);
        assert!((p0.i_rot - p1.i_rot - 0.5 * prop * r * r).abs() < 1e-12, "{} {}", p0.i_rot, p1.i_rot);
        assert!((p0.mass - p1.mass - prop).abs() < 1e-12 && p1.cg < p0.cg && p1.thrust == 0.0);
        // During the burn the jet damping is mdot (x_exit - x_cg)^2 with mdot = thrust x propellant / impulse.
        let (tau, c) = (0.5, &mo.curve);
        let pm = ctx.mass_props(tau, 0);
        let mdot = c.propellant_mass * c.thrust(tau) / c.total_impulse();
        assert!((pm.jet - mdot * (mo.mount_aft_station - pm.cg).powi(2)).abs() < 1e-12, "{} {}", pm.jet, mdot);
        assert!(pm.thrust > 100.0 && pm.i_long < p0.i_long + 1e-3);
    });
}

#[test]
fn a_vehicle_without_inertia_data_gets_a_rod_and_a_cylinder() {
    let mut v = vehicle();
    v.stages[0].dry = MassProps { mass: 1.0, cg: 0.55, i_long: 0.0, i_rot: 0.0 };
    v.stages[0].motor = None;
    with_ctx(v, |ctx| {
        let p = ctx.mass_props(0.0, 0);
        assert!((p.i_long - 1.0 * 1.1 * 1.1 / 12.0).abs() < 1e-12, "{}", p.i_long);
        assert!((p.i_rot - 0.5 * 1.0 * 0.038 * 0.038).abs() < 1e-12);
    });
}

/// State of the vehicle flying straight up at `v` m/s with body rate `w`.
fn flying(v: f64, w: Vec3) -> State {
    State { pos: [0.0, 0.0, 100.0], vel: [0.0, 0.0, v], q: from_axis([0.0, 0.0, 1.0]), w }
}

#[test]
fn pitch_rate_is_damped_by_the_strip_sum_of_the_lifting_components() {
    with_ctx(vehicle(), |ctx| {
        let (v, wy) = (60.0, 0.4);
        let d = ctx.eval(100.0, &flying(v, [0.0, wy, 0.0]), [0.0; 3], 0, Mode::Rigid, &[]);
        let p = ctx.mass_props(100.0, 0);
        // Coasting after burnout, no angle of attack: the only moment is the damping.
        assert_eq!(d.thrust, 0.0);
        let rho = d.air.density;
        let aero = &ctx.aeros[0];
        let st = aero.stability(v / d.air.speed_of_sound, 0.0);
        let strip: f64 = st.components.iter().map(|(_, cna, x)| cna * (x - p.cg).powi(2)).sum();
        let linear = 0.5 * rho * v * ctx.a_ref * strip;
        let crossflow = 0.5 * rho * BODY_CROSSFLOW_CD * aero.crossflow_damping(p.cg) * wy;
        let moment = d.ang_acc[1] * p.i_long;
        assert!((moment + (linear + crossflow) * wy).abs() < 1e-9 * moment.abs(), "{moment} vs {}", -(linear + crossflow) * wy);
        // It opposes the rate, in either sense, and does not couple into the other axes.
        let flipped = ctx.eval(100.0, &flying(v, [0.0, -wy, 0.0]), [0.0; 3], 0, Mode::Rigid, &[]);
        assert!((flipped.ang_acc[1] + d.ang_acc[1]).abs() < 1e-12 && d.ang_acc[1] < 0.0);
        assert!(d.ang_acc[0].abs() < 1e-12 && d.ang_acc[2].abs() < 1e-12);
        // The damping grows with the air speed (a viscous-like term plus the quadratic crossflow).
        let fast = ctx.eval(100.0, &flying(2.0 * v, [0.0, wy, 0.0]), [0.0; 3], 0, Mode::Rigid, &[]);
        assert!(fast.ang_acc[1] < 1.9 * d.ang_acc[1]);
    });
}

#[test]
fn a_stable_rocket_is_pushed_back_towards_the_relative_wind_and_an_unstable_one_away() {
    // Moving up through a 5 m/s crosswind blowing towards -x: the air velocity relative to the rocket is (+5, 0, 50), so the
    // nose (+z) should turn towards +x if the rocket is stable.
    for (cg, stable) in [(0.62, true), (0.98, false)] {
        let mut v = vehicle();
        v.stages[0].dry.cg = cg;
        with_ctx(v, |ctx| {
            let st = State { pos: [0.0, 0.0, 100.0], vel: [0.0, 0.0, 50.0], q: from_axis([0.0, 0.0, 1.0]), w: [0.0; 3] };
            let d = ctx.eval(100.0, &st, [-5.0, 0.0, 0.0], 0, Mode::Rigid, &[]);
            // The body frame of `from_axis` is x_b = +z, y_b = +y, z_b = -x: turning the nose towards +x is a rotation about +y_b.
            let toward_wind = d.ang_acc[1];
            assert_eq!(toward_wind > 0.0, stable, "cg {cg}: {:?}", d.ang_acc);
            // Both feel the same side force towards -x (the normal force opposes the lateral motion).
            assert!(d.acc[0] < 0.0 && d.alpha > 0.05);
        });
    }
}

#[test]
fn the_step_limit_tracks_the_pitch_frequency() {
    with_ctx(vehicle(), |ctx| {
        let d = ctx.eval(100.0, &flying(100.0, [0.0; 3]), [0.0; 3], 0, Mode::Rigid, &[]);
        let p = ctx.mass_props(100.0, 0);
        let nf = ctx.aeros[0].normal_force(d.mach, 0.0);
        let q = 0.5 * d.air.density * 100.0 * 100.0;
        let wn = (q * ctx.a_ref * nf.cn_alpha * (nf.cp - p.cg) / p.i_long).sqrt();
        assert!(d.stiff >= wn - 1e-9 && d.stiff < 4.0 * wn, "{} vs {wn}", d.stiff);
        // The recorded pitch frequency is that formula for a stable rocket, and zero for an unstable one.
        assert!((d.pitch_freq - wn).abs() < 1e-9 * wn, "{} vs {wn}", d.pitch_freq);
        let mut tail_heavy = vehicle();
        tail_heavy.stages[0].dry.cg = 0.98;
        with_ctx(tail_heavy, |ctx| {
            let d = ctx.eval(100.0, &flying(100.0, [0.0; 3]), [0.0; 3], 0, Mode::Rigid, &[]);
            assert_eq!(d.pitch_freq, 0.0);
        });
        // wn^2 = q A CNalpha (x_cp - x_cg) / I: tens of rad/s for this rocket at 100 m/s.
        assert!((5.0..100.0).contains(&wn), "{wn}");
    });
}

#[test]
fn positive_cant_turns_the_body_right_handed_about_the_nose_clockwise_seen_from_behind() {
    // Fin roll angle 0 is +Y and increases towards +X seen from behind (clockwise). A positive cant swings each fin's chord
    // towards -tangent, so the air streaming along +Z (nose to tail) pushes the fin towards +tangent, the way its angle
    // increases: a moment about the nose direction in the right-handed sense (thumb to the nose, fingers clockwise seen from
    // behind). The body angular acceleration about its x axis (along the nose) is therefore positive.
    let cant_eval = |cant: f64, w: f64| {
        let mut v = vehicle();
        v.fin_sets[0].cant = cant;
        with_ctx(v, |ctx| ctx.eval(100.0, &flying(100.0, [w, 0.0, 0.0]), [0.0; 3], 0, Mode::Rigid, &[]))
    };
    assert_eq!(cant_eval(0.0, 0.0).ang_acc[0], 0.0, "no cant, no roll moment");
    let (plus, minus) = (cant_eval(0.02, 0.0), cant_eval(-0.02, 0.0));
    assert!(plus.ang_acc[0] > 0.0 && minus.ang_acc[0] < 0.0, "{} {}", plus.ang_acc[0], minus.ang_acc[0]);
    assert!((plus.ang_acc[0] + minus.ang_acc[0]).abs() < 1e-9 * plus.ang_acc[0], "odd in the cant");
    // Already spinning the way the cant turns it, the fins' own roll damping brakes the spin: the acceleration falls.
    let spun = cant_eval(0.02, 20.0);
    assert!(spun.ang_acc[0] < plus.ang_acc[0], "damped: {} vs {}", spun.ang_acc[0], plus.ang_acc[0]);
}
