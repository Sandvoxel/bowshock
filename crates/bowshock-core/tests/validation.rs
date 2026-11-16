//! Total-drag validation against NASA wind-tunnel data: the Arcas sounding rocket (TN D-4013/D-4014, fins on and off) and the
//! base pressure with a cold air jet (TP-2005). Run with `--ignored --nocapture`.

use bowshock_core::{
    AftSpec, Analysis, Atmosphere, Backend, BodySpec, FinProfile, FinSpec, FlightConditions, MotorSpec, NoseShape, NoseSpec, Quality, Resolution,
};
use bowshock_reference::{arcas, tp2005};
use std::io::Write;
use std::time::Instant;

const R_AIR: f64 = 287.052_87;

/// Tunnel flow at `mach` from the stagnation temperature and the Reynolds number per metre.
fn tunnel(mach: f64, t0: f64, re_per_m: f64) -> FlightConditions {
    let t = t0 / (1.0 + 0.2 * mach * mach);
    let a = (1.4 * R_AIR * t).sqrt();
    let mu = 1.716e-5 * (t / 273.15).powf(1.5) * (273.15 + 110.4) / (t + 110.4);
    let rho = re_per_m * mu / (mach * a);
    FlightConditions { atmosphere: Atmosphere::Explicit { temperature_k: t, pressure_pa: rho * R_AIR * t } }
}

fn flush() {
    std::io::stdout().flush().ok();
}

fn arcas_body() -> BodySpec {
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: arcas::NOSE_LENGTH, diameter: arcas::BODY_DIAMETER },
        aft: vec![AftSpec::Tube { length: arcas::TUBE_LENGTH }, AftSpec::Transition { length: arcas::BOATTAIL_LENGTH, aft_diameter: arcas::BASE_DIAMETER }],
    }
}

fn arcas_fins() -> FinSpec {
    FinSpec {
        count: 4,
        root_chord: arcas::FIN_ROOT_CHORD,
        tip_chord: arcas::FIN_TIP_CHORD,
        span: arcas::FIN_SPAN,
        sweep: arcas::FIN_SWEEP,
        thickness: arcas::FIN_THICKNESS,
        roughness: 0.0,
        profile: FinProfile::DoubleWedge,
    }
}

#[test]
#[ignore = "validation: GPU sweep, minutes"]
fn arcas_axial_force() {
    let body = arcas_body();
    // (config, M, data, T0, Re/m, fins, gated, tolerance rule)
    let mut cases = Vec::new();
    for &(m, off, on) in &arcas::TRANSONIC {
        let gated = [0.60, 0.80, 0.90, 1.20].contains(&m);
        cases.push(("D4013 off", m, off, arcas::TRANSONIC_T0_K, arcas::TRANSONIC_RE_PER_M, false, gated));
        cases.push(("D4013 on ", m, on, arcas::TRANSONIC_T0_K, arcas::TRANSONIC_RE_PER_M, true, gated));
    }
    for &(m, off, on) in &arcas::SUPERSONIC {
        cases.push(("D4014 off", m, off, arcas::SUPERSONIC_T0_K, arcas::SUPERSONIC_RE_PER_M, false, false));
        cases.push(("D4014 on ", m, on, arcas::SUPERSONIC_T0_K, arcas::SUPERSONIC_RE_PER_M, true, true));
    }
    println!(
        "{:<10} {:>5} {:>7} {:>9} {:>8} {:>8} {:>10} {:>5} {:<20} {:>6}  gate",
        "config", "M", "data", "bowshock", "delta", "d/data", "quality", "iters", "stop", "sec"
    );
    let mut failures = Vec::new();
    let mut backends = Vec::new();
    let t_all = Instant::now();
    for (config, m, data, t0, re, fins, gated) in cases {
        let mut a = Analysis::new(&body).unwrap().backend(Backend::Auto).resolution(Resolution::Medium).viscous(tunnel(m, t0, re));
        if fins {
            a = a.fins(arcas_fins());
        }
        let t = Instant::now();
        let polar = a.run(&[m]).unwrap();
        let p = &polar.points[0];
        let value = p.drag.forebody_pressure + p.drag.friction.unwrap_or(f64::NAN) + p.drag.fins.map_or(0.0, |f| f.friction + f.pressure);
        let delta = value - data;
        let tol = if fins { 0.15 * data } else { (0.12 * data).max(0.03) };
        let ok = delta.abs() <= tol && p.quality != Quality::Failed;
        println!(
            "{config:<10} {m:>5.2} {data:>7.3} {value:>9.4} {delta:>+8.4} {:>+7.1}% {:>10} {:>5} {:<20} {:>6.0}  {}",
            100.0 * delta / data,
            format!("{:?}", p.quality),
            p.detail.iterations,
            p.detail.stop_reason.to_string(),
            t.elapsed().as_secs_f64(),
            if !gated {
                "-"
            } else if ok {
                "pass"
            } else {
                "FAIL"
            }
        );
        flush();
        if gated && !ok {
            failures.push(format!("{config} M {m}: data {data}, bowshock {value:.4}, delta {delta:+.4}, tol {tol:.4}, quality {:?}", p.quality));
        }
        let b = format!("{:?} fallback {:?}", polar.backend, polar.fallback);
        if !backends.contains(&b) {
            backends.push(b);
        }
    }
    println!("backends: {backends:?}  wall {:.0} s", t_all.elapsed().as_secs_f64());
    flush();
    assert!(failures.is_empty(), "gate failures:\n{}", failures.join("\n"));
}

#[test]
#[ignore = "validation: GPU sweep, minutes"]
fn tp2005_base_pressure_with_jet() {
    let body = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: tp2005::NOSE_LENGTH, diameter: tp2005::BODY_DIAMETER },
        aft: vec![AftSpec::Tube { length: tp2005::BASE_STATION - tp2005::NOSE_LENGTH }],
    };
    println!("{:>5} {:>5} {:>9} {:>9} {:>8} {:>10} {:>5} {:<20} {:>6}  gate", "Minf", "Me", "data", "bowshock", "delta", "quality", "iters", "stop", "sec");
    let mut failures = Vec::new();
    let mut results: Vec<(f64, f64, f64)> = Vec::new();
    let mut backends = Vec::new();
    let t_all = Instant::now();
    for row in tp2005::TABLE_II {
        let (minf, me, _, ptj_pinf) = row;
        let flight = tunnel(minf, tp2005::T0_K, tp2005::RE_PER_M);
        let p_inf = flight.atmosphere.state().unwrap().pressure_pa;
        let motor = MotorSpec {
            exit_diameter: tp2005::NOZZLE_EXIT_DIAMETER,
            exit_mach: me,
            exit_pressure_pa: ptj_pinf / (1.0 + 0.2 * me * me).powf(3.5) * p_inf,
            total_temperature_k: tp2005::JET_TOTAL_TEMPERATURE_K,
        };
        let t = Instant::now();
        let polar = Analysis::new(&body).unwrap().backend(Backend::Auto).resolution(Resolution::Medium).viscous(flight).motor(motor).run(&[minf]).unwrap();
        let p = &polar.points[0];
        let data = tp2005::base_pressure_ratio(row);
        let value = 1.0 + 0.7 * minf * minf * p.detail.cp_base;
        let delta = value - data;
        let ok = delta.abs() <= 0.15 && p.quality != Quality::Failed;
        println!(
            "{minf:>5.2} {me:>5.2} {data:>9.3} {value:>9.3} {delta:>+8.3} {:>10} {:>5} {:<20} {:>6.0}  {}",
            format!("{:?}", p.quality),
            p.detail.iterations,
            p.detail.stop_reason.to_string(),
            t.elapsed().as_secs_f64(),
            if ok { "pass" } else { "FAIL" }
        );
        flush();
        if !ok {
            failures.push(format!("M {minf} Me {me}: data {data:.3}, bowshock {value:.3}, delta {delta:+.3}, quality {:?}", p.quality));
        }
        results.push((minf, me, value));
        let b = format!("{:?} fallback {:?}", polar.backend, polar.fallback);
        if !backends.contains(&b) {
            backends.push(b);
        }
    }
    println!("backends: {backends:?}  wall {:.0} s", t_all.elapsed().as_secs_f64());
    for minf in [1.65, 2.0, 2.5] {
        let at = |me: f64| results.iter().find(|r| r.0 == minf && r.1 == me).map(|r| r.2).unwrap();
        if at(1.97) <= at(1.69) {
            failures.push(format!("M {minf}: pb/p_inf does not increase from Me 1.69 ({:.3}) to 1.97 ({:.3})", at(1.69), at(1.97)));
        }
    }
    flush();
    assert!(failures.is_empty(), "gate failures:\n{}", failures.join("\n"));
}
