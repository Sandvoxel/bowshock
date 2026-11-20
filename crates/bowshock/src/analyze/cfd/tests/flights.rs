//! Flight-data validation (DESIGN3D item 56): public rockets with full geometry, a motor and a measured apogee, flown through the app's own flight simulator on the drag
//! of the 3D engine, of the legacy engine and of the build-up (no table).
//!
//! The rockets are `assets/validation/flights/*.json` (inches and pounds as the RASAero II files give them; sources and licence notes in the `README.md` there). The
//! stored drag tables are `assets/validation/flights/tables/<id>.<engine>.json`: the `CfdTable` the app's `solve` made for the rocket, with the whole app path (the plan,
//! the full Coarse solve with its angle-of-attack points, the build-up for what the solver does not model).
//!
//! * `flight_fixtures_build_and_fly` (CPU, every run) checks that every rocket builds a vehicle, a plan and a flight.
//! * `flight_validation` (ignored) prints the table. `FLIGHTVAL_SOLVE=1` solves the tables first (GPU; the engine is `BOWSHOCK_ENGINE`: `3d`, or unset for the legacy
//!   engine) for the rockets of `FLIGHTVAL_CASES` (comma separated ids, default all) and stores them:
//!   `tools/gpuq/gpuq submit --host any --name flightval -- env BOWSHOCK_ENGINE=3d FLIGHTVAL_SOLVE=1 FLIGHTVAL_OUT=out cargo nextest run --release -p bowshock flight_validation --run-ignored only --no-capture`.
//!   Without it the stored tables are flown (seconds, CPU). A queue job sets `FLIGHTVAL_OUT=out` and copies the tables from `~/.cache/bowshock-gpuq/done/<id>/out`.

use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Instant;

use bowshock_model::{CfdGrid, CfdQuality, CfdTable, Rocket};
use bowshock_motors::{Motor, MotorDb};
use bowshock_sim::{
    simulate, vs_mach, Atmosphere, BodyKind, BodySegment, DragTable, FinCrossSection, FinPlanform, FinSet, Labels, Launch, MassProps, NoseShape as SimNose,
    Protuberance, SimInput, StageMass, StageMotor, ThrustCurve, Timing, Vehicle,
};
use serde::Deserialize;

use super::*;
use crate::units::UnitSystem;

const IN: f64 = 0.0254;
const FT: f64 = 0.3048;
const LB: f64 = 0.453_592_37;
const MPH: f64 = 0.447_04;

#[derive(Deserialize)]
struct Fixture {
    id: String,
    name: String,
    nose: NoseF,
    body: Vec<SectionF>,
    fins: FinsF,
    roughness_in: f64,
    rail_guide: Option<RailF>,
    launch_weight_lb: f64,
    cg_in: f64,
    motor: MotorF,
    site: SiteF,
    measured: MeasuredF,
    rasaero: RasaeroF,
    #[serde(default)]
    notes: String,
    source: String,
}
#[derive(Deserialize)]
struct NoseF {
    shape: String,
    length: f64,
    diameter: f64,
    blunt_radius: f64,
}
#[derive(Deserialize)]
struct SectionF {
    length: f64,
    fore_diameter: f64,
    aft_diameter: f64,
}
#[derive(Deserialize)]
struct FinsF {
    count: u32,
    root_chord: f64,
    span: f64,
    sweep: f64,
    tip_chord: f64,
    thickness: f64,
    section: String,
    wedge_fraction: f64,
    te_to_base: f64,
}
#[derive(Deserialize)]
struct RailF {
    diameter: f64,
    height: f64,
}
#[derive(Deserialize)]
struct MotorF {
    designation: String,
    manufacturer: String,
    /// A RASP file beside the fixture for a motor the bundled catalogue does not hold.
    file: Option<String>,
}
#[derive(Deserialize)]
struct SiteF {
    altitude_ft: f64,
    temperature_f: f64,
    rod_angle_deg: f64,
    rod_length_ft: f64,
    wind_mph: f64,
}
#[derive(Deserialize)]
struct MeasuredF {
    apogee_ft: f64,
    data: String,
}
#[derive(Deserialize)]
struct RasaeroF {
    table_ft: f64,
    file_ft: f64,
}

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/validation/flights")
}

fn fixtures() -> Vec<Fixture> {
    let mut paths: Vec<_> =
        std::fs::read_dir(dir()).unwrap().filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.extension().is_some_and(|x| x == "json")).collect();
    paths.sort();
    let mut all: Vec<Fixture> =
        paths.iter().map(|p| serde_json::from_slice(&std::fs::read(p).unwrap()).unwrap_or_else(|e| panic!("{}: {e}", p.display()))).collect();
    // probes: `FLIGHTVAL_TIP_IN=0.5` gives every fin that tip chord, `FLIGHTVAL_BLUNT_IN=0` every nose that tip radius (in)
    if let Some(tip) = std::env::var("FLIGHTVAL_TIP_IN").ok().and_then(|v| v.parse().ok()) {
        all.iter_mut().for_each(|f| f.fins.tip_chord = tip);
    }
    if let Some(blunt) = std::env::var("FLIGHTVAL_BLUNT_IN").ok().and_then(|v| v.parse().ok()) {
        all.iter_mut().for_each(|f| f.nose.blunt_radius = blunt);
    }
    // `FLIGHTVAL_SHOULDER=0` merges the body sections into one tube of the first section's diameter (no fin can, no conical shoulder); `FLIGHTVAL_FIN_THICK_IN`,
    // `FLIGHTVAL_SPAN_IN`, `FLIGHTVAL_SWEEP_IN` set the fin's thickness, span and sweep
    if std::env::var("FLIGHTVAL_SHOULDER").is_ok_and(|v| v == "0") {
        for f in &mut all {
            let (d, l) = (f.body[0].fore_diameter, f.body.iter().map(|s| s.length).sum());
            f.body = vec![SectionF { length: l, fore_diameter: d, aft_diameter: d }];
        }
    }
    let probe = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<f64>().ok());
    for f in &mut all {
        f.fins.thickness = probe("FLIGHTVAL_FIN_THICK_IN").unwrap_or(f.fins.thickness);
        f.fins.span = probe("FLIGHTVAL_SPAN_IN").unwrap_or(f.fins.span);
        f.fins.sweep = probe("FLIGHTVAL_SWEEP_IN").unwrap_or(f.fins.sweep);
        if let Ok(s) = std::env::var("FLIGHTVAL_SECTION") {
            f.fins.section = s;
        }
    }
    match std::env::var("FLIGHTVAL_CASES") {
        Ok(list) => {
            let ids: Vec<&str> = list.split(',').map(str::trim).collect();
            all.into_iter().filter(|f| ids.contains(&f.id.as_str())).collect()
        }
        Err(_) => all,
    }
}

fn find_motor<'a>(db: &'a MotorDb, m: &MotorF) -> Result<&'a Motor, String> {
    let want = m.manufacturer.to_lowercase();
    db.motors
        .iter()
        .find(|x| x.common_name.to_lowercase().starts_with(&m.designation.to_lowercase()) && x.manufacturer.to_lowercase().contains(&want))
        .ok_or_else(|| format!("no motor {} {} in the catalogue", m.manufacturer, m.designation))
}

/// The vehicle of a rocket: the sections in order, one fin set, the motor of the catalogue at the base, the dry mass and balance from the launch weight and CG.
fn vehicle(f: &Fixture, db: &MotorDb) -> Result<Vehicle, String> {
    let rough = f.roughness_in * IN;
    let mut body = vec![BodySegment {
        station: 0.0,
        length: f.nose.length * IN,
        fore_radius: 0.0,
        aft_radius: 0.5 * f.nose.diameter * IN,
        kind: BodyKind::Nose(match f.nose.shape.as_str() {
            "tangent-ogive" => SimNose::TangentOgive,
            "conical" => SimNose::Conical,
            s => return Err(format!("nose shape {s}")),
        }),
        tip_radius: f.nose.blunt_radius * IN,
        roughness: rough,
    }];
    let mut x = f.nose.length * IN;
    for s in &f.body {
        let (fore, aft) = (0.5 * s.fore_diameter * IN, 0.5 * s.aft_diameter * IN);
        let kind = if (fore - aft).abs() < 1e-9 { BodyKind::Tube } else { BodyKind::Transition };
        body.push(BodySegment { station: x, length: s.length * IN, fore_radius: fore, aft_radius: aft, kind, tip_radius: 0.0, roughness: rough });
        x += s.length * IN;
    }
    let length = x;
    let radius_at = |station: f64| body.iter().find(|s| station < s.aft_station() + 1e-9).map_or(0.0, |s| s.radius_at(station - s.station));
    let fin = &f.fins;
    let te = length - fin.te_to_base * IN;
    let le = te - fin.root_chord * IN;
    let fin_set = FinSet {
        count: fin.count,
        station: le,
        body_radius: radius_at(0.5 * (le + te)),
        root_chord: fin.root_chord * IN,
        tip_chord: fin.tip_chord * IN,
        span: fin.span * IN,
        sweep: fin.sweep * IN,
        thickness: fin.thickness * IN,
        cross_section: match fin.section.as_str() {
            "rounded" => FinCrossSection::Rounded,
            "square" => FinCrossSection::Square,
            "hexagonal" => FinCrossSection::Hexagonal(fin.wedge_fraction.clamp(0.01, 0.5)),
            s => return Err(format!("fin section {s}")),
        },
        cant: 0.0,
        roughness: rough,
        planform: FinPlanform::Trapezoidal,
        mid_chord_cos: None,
        root_extent: None,
        angles: Vec::new(),
        parent: 0,
        bodies: 1,
    };
    let own;
    let motor = match &f.motor.file {
        Some(file) => {
            let mut one = MotorDb::default();
            let text = std::fs::read_to_string(dir().join(file)).map_err(|e| format!("{file}: {e}"))?;
            one.add_eng_str(&text).map_err(|e| format!("{file}: {e:?}"))?;
            own = one;
            own.motors.first().ok_or_else(|| format!("{file}: no motor"))?
        }
        None => find_motor(db, &f.motor)?,
    };
    let mut points = motor.points.clone();
    if points.first().is_some_and(|p| p.0 > 0.0) {
        points.insert(0, (0.0, 0.0));
    }
    let curve = ThrustCurve { points, propellant_mass: motor.propellant_mass, total_mass: motor.total_mass, length: motor.length, diameter: motor.diameter };
    let (launch_mass, cg) = (f.launch_weight_lb * LB, f.cg_in * IN);
    let dry = launch_mass - motor.total_mass;
    if dry <= 0.0 {
        return Err(format!("the launch weight {:.2} kg is below the motor's {:.2} kg", launch_mass, motor.total_mass));
    }
    let motor_cg = length - 0.5 * motor.length;
    let dry_cg = (launch_mass * cg - motor.total_mass * motor_cg) / dry;
    let reference_diameter = body.iter().map(|s| 2.0 * s.fore_radius.max(s.aft_radius)).fold(0.0, f64::max);
    let protuberances = f
        .rail_guide
        .iter()
        .flat_map(|r| [0.55, 0.85].map(|s| Protuberance::RailButton { station: s * length, height: r.height * IN, diameter: r.diameter * IN }))
        .collect();
    Ok(Vehicle {
        reference_diameter,
        body,
        fin_sets: vec![fin_set],
        tube_fins: Vec::new(),
        protuberances,
        stages: vec![StageMass {
            dry: MassProps { mass: dry, cg: dry_cg, i_long: 0.0, i_rot: 0.0 },
            fore_station: 0.0,
            aft_station: length,
            motor: Some(StageMotor { curve, mount_aft_station: length, ignition: Timing::LAUNCH, ejection_delay: None, tilt_lockout: None }),
            separation: Timing::BURNOUT,
        }],
        labels: Labels::default(),
    })
}

fn launch(f: &Fixture) -> Launch {
    Launch {
        rail_length: f.site.rod_length_ft * FT,
        angle_from_vertical: f.site.rod_angle_deg.to_radians(),
        site_altitude: f.site.altitude_ft * FT,
        ground_temperature: (f.site.temperature_f - 32.0) / 1.8 + 273.15,
        wind_speed: f.site.wind_mph * MPH,
        ..Launch::default()
    }
}

/// The flight (apogee above ground in feet, peak Mach) on `table`, or on the build-up when there is none.
fn fly(f: &Fixture, v: &Vehicle, table: Option<&CfdTable>) -> (f64, f64) {
    let mut input = SimInput::new(v.clone(), launch(f), Vec::new());
    if let Some(t) = table {
        input.drag_tables = vec![Some(DragTable::from_cfd(t))];
    }
    let s = simulate(&input).summary;
    (s.apogee / FT, s.max_mach)
}

fn table_path(f: &Fixture, engine: &str) -> PathBuf {
    dir().join("tables").join(format!("{}.{engine}.json", f.id))
}

/// Where a solved table is written: `$FLIGHTVAL_OUT` (a GPU queue job writes `out/`, which `gpuq` copies back) or the fixture directory.
fn write_table(name: &str, table: &CfdTable) {
    let dir = std::env::var("FLIGHTVAL_OUT").map_or_else(|_| dir().join("tables"), PathBuf::from);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(name), serde_json::to_vec_pretty(table).unwrap()).unwrap();
}

fn engine_name() -> &'static str {
    if solver::Engine::from_env() == solver::Engine::Solver3 {
        "3d"
    } else {
        "legacy"
    }
}

/// The Mach nodes and stability nodes of a solve: the app's, or `FLIGHTVAL_MACHS=0.8,2.0` (drag nodes only, no angle points) for a quick look.
fn job_nodes() -> (Vec<f64>, Vec<f64>) {
    match std::env::var("FLIGHTVAL_MACHS") {
        Ok(list) => (list.split(',').map(|m| m.trim().parse().unwrap()).collect(), Vec::new()),
        Err(_) => (nodes((0.1, 3.0)).unwrap(), lift_nodes((0.1, 3.0)).unwrap()),
    }
}

/// The solve's progress for the log: one line per finished Mach number (and angle) with its time, iterations and how it ended.
fn log_points(label: String) -> impl Fn(Event) + Send + Sync + 'static {
    move |e| {
        if let Event::Point(p) = e {
            println!("{label} POINT at {:.0} s: M {:.2} alpha {:?} {} iterations, {:?}", p.at, p.mach, p.alpha_deg, p.iterations, p.outcome);
        }
    }
}

fn plan_of(v: &Vehicle) -> Result<Plan, String> {
    plan(&Rocket::default(), v, UnitSystem::default())
}

/// The plan of the fixture `id` (with the probes of the environment applied) and its launch altitude in metres, for the diagnostics of `diag56`.
pub(super) fn plan_and_altitude(id: &str) -> (Plan, f64) {
    let f = fixtures().into_iter().find(|f| f.id == id).unwrap_or_else(|| panic!("no fixture {id}"));
    let v = vehicle(&f, &MotorDb::bundled()).unwrap();
    (plan_of(&v).unwrap(), f.site.altitude_ft * FT)
}

#[test]
fn flight_fixtures_build_and_fly() {
    let db = MotorDb::bundled();
    let atm = Atmosphere::standard();
    for f in fixtures() {
        let v = vehicle(&f, &db).unwrap_or_else(|e| panic!("{}: {e}", f.id));
        let p = plan_of(&v).unwrap_or_else(|e| panic!("{}: {e}", f.id));
        assert!(p.fins.is_some(), "{}: the solver takes the fins: {:?}", f.id, p.notes);
        // the build-up flight lands within a third of the measured apogee: the fixture is the rocket it says
        let (apogee, mach) = fly(&f, &v, None);
        assert!((apogee / f.measured.apogee_ft - 1.0).abs() < 0.35, "{}: build-up apogee {apogee:.0} ft against {} ft measured", f.id, f.measured.apogee_ft);
        assert!(mach > 0.3, "{}: Mach {mach}", f.id);
        let _ = vs_mach(&v, 0.0, (0.5, 0.5), 1, 0.0, &atm, 0.0);
        let _ = (&f.name, &f.source, &f.notes, &f.measured.data, f.rasaero.file_ft, f.rasaero.table_ft);
    }
}

/// The table of the flight-data validation: solve (with `FLIGHTVAL_SOLVE=1`), fly and print. See the module docs.
#[test]
#[ignore = "GPU with FLIGHTVAL_SOLVE=1: solves the full Coarse table of every rocket; otherwise flies the stored tables"]
fn flight_validation() {
    let db = MotorDb::bundled();
    let engine = engine_name();
    let solve_now = std::env::var("FLIGHTVAL_SOLVE").is_ok();
    for f in fixtures() {
        let v = vehicle(&f, &db).unwrap_or_else(|e| panic!("{}: {e}", f.id));
        if solve_now {
            let p = plan_of(&v).unwrap_or_else(|e| panic!("{}: {e}", f.id));
            println!("{}: {engine} plan notes {:?}", f.id, p.notes);
            let job = Job {
                plan: p,
                altitude: f.site.altitude_ft * FT,
                nodes: job_nodes().0,
                stage: 1,
                grid: CfdGrid::Coarse,
                compute: Default::default(),
                cpu_only: false,
                stability: job_nodes().1,
            };
            let t0 = Instant::now();
            let table = solve(&job, &Arc::new(AtomicBool::new(false)), log_points(f.id.clone())).unwrap_or_else(|e| panic!("{}: solve failed: {e}", f.id));
            println!(
                "{}: {engine} solved {} points + {} angle points on {} in {:.1} s wall ({:.1} s app); notes {:?}",
                f.id,
                table.points.len(),
                table.lift.as_ref().map_or(0, |l| l.points.len()),
                table.backend,
                t0.elapsed().as_secs_f64(),
                table.solve_seconds,
                table.notes
            );
            write_table(&format!("{}.{engine}.json", f.id), &table);
        }
    }
    print_table(&fixtures(), &db);
}

fn load_table(f: &Fixture, engine: &str) -> Option<CfdTable> {
    serde_json::from_slice(&std::fs::read(table_path(f, engine)).ok()?).ok()
}

fn print_table(fixtures: &[Fixture], db: &MotorDb) {
    println!("\n| rocket | motor | measured ft | data | peak Mach | 3D ft | 3D % | legacy ft | legacy % | build-up ft | build-up % | RASAero ft | RASAero % |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    let mut errs: [Vec<f64>; 4] = Default::default();
    for f in fixtures {
        let v = vehicle(f, db).unwrap();
        let m = f.measured.apogee_ft;
        let err = |a: f64| 100.0 * (a - m) / m;
        let (build, mach) = fly(f, &v, None);
        let cell = |engine: &str| match load_table(f, engine) {
            Some(t) => {
                let (a, _) = fly(f, &v, Some(&t));
                (format!("{a:.0}"), format!("{:+.1}", err(a)), Some(err(a)))
            }
            None => ("-".into(), "-".into(), None),
        };
        let (d3, e3, x3) = cell("3d");
        let (dl, el, xl) = cell("legacy");
        for (k, x) in [x3, xl, Some(err(build)), Some(err(f.rasaero.table_ft))].into_iter().enumerate() {
            errs[k].extend(x);
        }
        println!(
            "| {} | {} {} | {:.0} | {} | {:.2} | {d3} | {e3} | {dl} | {el} | {build:.0} | {:+.1} | {:.0} | {:+.1} |",
            f.name,
            f.motor.manufacturer,
            f.motor.designation,
            m,
            f.measured.data,
            mach,
            err(build),
            f.rasaero.table_ft,
            err(f.rasaero.table_ft)
        );
    }
    for (name, e) in ["3D engine", "legacy engine", "build-up", "RASAero II"].iter().zip(&errs) {
        if e.is_empty() {
            continue;
        }
        let n = e.len() as f64;
        println!(
            "{name}: {} rockets, mean error {:+.1} %, mean |error| {:.1} %, within 10 %: {}/{}, largest |error| {:.1} %",
            e.len(),
            e.iter().sum::<f64>() / n,
            e.iter().map(|x| x.abs()).sum::<f64>() / n,
            e.iter().filter(|x| x.abs() <= 10.0).count(),
            e.len(),
            e.iter().fold(0.0f64, |a, x| a.max(x.abs()))
        );
    }
}

/// A rocket with two real diameter steps solved on the GPU by the 3D engine (DESIGN3D item 56, `BOWSHOCK_ENGINE=3d`): the Rabia short-fin-can rocket with a 3.0 in nose behind
/// which the 3.126 in airframe starts (a shoulder step) and with its 3.19 in fin can stepping out abruptly 6 in (2 calibres) ahead of the fin root instead of the conical shoulder
/// 1 in ahead of it. Prints the table and the point qualities; every point must have settled or converged, none failed.
/// `tools/gpuq/gpuq submit --host any --name stepped -- env BOWSHOCK_ENGINE=3d cargo nextest run --release -p bowshock stepped_rocket_solves_and_settles --run-ignored only --no-capture`.
#[test]
#[ignore = "BOWSHOCK_ENGINE=3d: solves a stepped rocket's table and angles on the GPU"]
fn stepped_rocket_solves_and_settles() {
    assert_eq!(solver::Engine::from_env(), solver::Engine::Solver3, "run with BOWSHOCK_ENGINE=3d");
    let f = fixtures().into_iter().find(|f| f.id == "rabia-short-fin-can").expect("the Rabia fixture");
    let mut stepped = f;
    stepped.nose.diameter = 3.0;
    stepped.body =
        vec![SectionF { length: 66.25, fore_diameter: 3.126, aft_diameter: 3.126 }, SectionF { length: 17.75, fore_diameter: 3.19, aft_diameter: 3.19 }];
    let v = vehicle(&stepped, &MotorDb::bundled()).unwrap();
    let p = plan_of(&v).expect("the 3D engine takes steps");
    let steps = p.body.aft.iter().filter(|a| matches!(a, solver::AftSpec::Step { .. })).count();
    assert_eq!(steps, 2, "{:?}", p.body.aft);
    assert!(p.fins.is_some(), "the fins stand behind both steps: {:?}", p.notes);
    println!("plan: {:?} notes {:?}", p.body.aft, p.notes);
    let job = Job {
        plan: p,
        altitude: stepped.site.altitude_ft * FT,
        nodes: job_nodes().0,
        stage: 1,
        grid: CfdGrid::Coarse,
        compute: Default::default(),
        cpu_only: false,
        stability: job_nodes().1,
    };
    let t0 = Instant::now();
    let table = solve(&job, &Arc::new(AtomicBool::new(false)), log_points("stepped".into())).expect("the stepped rocket solves");
    println!(
        "stepped: {} points + {} angle points on {} in {:.1} s wall; notes {:?}",
        table.points.len(),
        table.lift.as_ref().map_or(0, |l| l.points.len()),
        table.backend,
        t0.elapsed().as_secs_f64(),
        table.notes
    );
    for p in &table.points {
        println!(
            "DRAG M {:.2} cd {:.5} friction {:.5} pressure {:.5} base {:.5} fins {:.5} other {:.5} {:?}",
            p.mach, p.cd, p.friction, p.pressure, p.base, p.fins, p.other, p.quality
        );
    }
    for p in table.lift.iter().flat_map(|l| &l.points) {
        println!("LIFT M {:.2} alpha {:.0} CN {:.4} x_cp {:.4} {:?}", p.mach, p.alpha_deg, p.cn, p.x_cp, p.quality);
    }
    let bad: Vec<_> =
        table.points.iter().filter(|p| !matches!(p.quality, CfdQuality::Converged | CfdQuality::Stationary)).map(|p| (p.mach, p.quality)).collect();
    assert!(bad.is_empty(), "points that did not settle: {bad:?}");
    let lift_bad: Vec<_> = table
        .lift
        .iter()
        .flat_map(|l| &l.points)
        .filter(|p| !matches!(p.quality, CfdQuality::Converged | CfdQuality::Stationary))
        .map(|p| (p.mach, p.alpha_deg, p.quality))
        .collect();
    assert!(lift_bad.is_empty(), "angle points that did not settle: {lift_bad:?}");
    write_table("stepped-rabia.3d.json", &table);
}
