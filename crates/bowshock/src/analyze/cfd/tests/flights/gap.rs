//! Apogee-gap diagnosis of the flight validation (DESIGN3D item 74; ignored tests, CPU, seconds, printed as tables, nothing asserts; `FLIGHTVAL_CASES` picks rockets):
//! `apogee_gap` (baseline against RASAero II's own flight, rail guide, motor and masses, exhaust, wind and site), `apogee_gap_finish` (surface finish by friction scaling),
//! `apogee_gap_baro` (the barometric altimeter's reading), `apogee_gap_parts` (the zero-lift drag parts against the build-up), `apogee_gap_mass` (mass conventions),
//! `apogee_gap_nodes` (where on the Mach axis the drag matters). `RASAERO_MOTORS=<dir>` (files `<id>.eng` taken by hand from the RASAero II motor file) adds the flight on RASAero's
//! thrust curve of the same designation. The drag factors of boost and coast that reproduce RASAero II's flight (item 74) were fitted with a six-line local hook in the flight's
//! drag (a factor while the motor burns and while it coasts), not kept.

use bowshock_sim::EventKind;

use super::*;

struct Run {
    apogee_ft: f64,
    mach: f64,
    vmax_fps: f64,
    t_apogee: f64,
    t_burn: f64,
    h_burn_ft: f64,
    v_burn_fps: f64,
}

fn run_of(f: &Fixture, v: &Vehicle, table: Option<&CfdTable>, tweak: &dyn Fn(&mut Launch)) -> Run {
    let mut input = SimInput::new(v.clone(), launch(f), Vec::new());
    tweak(&mut input.launch);
    if let Some(t) = table {
        input.drag_tables = vec![Some(DragTable::from_cfd(t, None))];
    }
    let r = simulate(&input);
    let burn = r.trajectory.events.iter().find(|e| matches!(e.kind, EventKind::Burnout));
    let (t_burn, h_burn) = burn.map_or((0.0, 0.0), |e| (e.t, e.altitude));
    // the speed at burnout: the sample nearest after it
    let v_burn = r.trajectory.samples.iter().find(|s| s.t >= t_burn).map_or(0.0, |s| s.speed);
    let s = r.summary;
    Run {
        apogee_ft: s.apogee / FT,
        mach: s.max_mach,
        vmax_fps: s.max_velocity / FT,
        t_apogee: s.time_to_apogee,
        t_burn,
        h_burn_ft: h_burn / FT,
        v_burn_fps: v_burn / FT,
    }
}

fn apogee(f: &Fixture, v: &Vehicle, t: Option<&CfdTable>) -> f64 {
    run_of(f, v, t, &|_| {}).apogee_ft
}

fn pct(a: f64, m: f64) -> String {
    format!("{:+.1}", 100.0 * (a - m) / m)
}

fn with_rail(f: &Fixture, rail: Option<(f64, f64)>) -> Fixture {
    let mut g = Fixture { rail_guide: rail.map(|(d, h)| RailF { diameter: d, height: h }), ..clone_fixture(f) };
    g.id = f.id.clone();
    g
}

fn clone_fixture(f: &Fixture) -> Fixture {
    Fixture {
        id: f.id.clone(),
        name: f.name.clone(),
        nose: NoseF { shape: f.nose.shape.clone(), length: f.nose.length, diameter: f.nose.diameter, blunt_radius: f.nose.blunt_radius },
        body: f.body.iter().map(|s| SectionF { length: s.length, fore_diameter: s.fore_diameter, aft_diameter: s.aft_diameter }).collect(),
        fins: FinsF {
            count: f.fins.count,
            root_chord: f.fins.root_chord,
            span: f.fins.span,
            sweep: f.fins.sweep,
            tip_chord: f.fins.tip_chord,
            thickness: f.fins.thickness,
            section: f.fins.section.clone(),
            wedge_fraction: f.fins.wedge_fraction,
            te_to_base: f.fins.te_to_base,
        },
        roughness_in: f.roughness_in,
        rail_guide: f.rail_guide.as_ref().map(|r| RailF { diameter: r.diameter, height: r.height }),
        launch_weight_lb: f.launch_weight_lb,
        cg_in: f.cg_in,
        motor: MotorF { designation: f.motor.designation.clone(), manufacturer: f.motor.manufacturer.clone(), file: f.motor.file.clone() },
        site: SiteF {
            altitude_ft: f.site.altitude_ft,
            temperature_f: f.site.temperature_f,
            rod_angle_deg: f.site.rod_angle_deg,
            rod_length_ft: f.site.rod_length_ft,
            wind_mph: f.site.wind_mph,
        },
        measured: MeasuredF { apogee_ft: f.measured.apogee_ft, data: f.measured.data.clone() },
        rasaero: RasaeroF { table_ft: f.rasaero.table_ft, file_ft: f.rasaero.file_ft },
        notes: f.notes.clone(),
        source: f.source.clone(),
    }
}

/// RASAero II's own predictions of the example files: (time to apogee s, maximum velocity ft/s), read from the `Simulation` block of each example by hand.
fn rasaero_flight(id: &str) -> (f64, f64) {
    match id {
        "byrum" => (15.990, 834.70),
        "cancer-descending" => (20.350, 648.69),
        "dont-debate-this" => (55.487, 3469.3),
        "ion-drive" => (21.900, 926.79),
        "rabia-short-fin-can" => (23.900, 994.43),
        "torrent" => (27.131, 1404.3),
        _ => (0.0, 0.0),
    }
}

#[test]
#[ignore = "diagnosis: prints tables from the stored tables, CPU"]
fn apogee_gap() {
    let db = MotorDb::bundled();
    let fx = fixtures();
    let none = |_: &mut Launch| {};
    println!("\n## baseline (3D table): ours against RASAero II's own flight");
    println!("| rocket | measured | 3D apogee | peak M | vmax fps | RASAero vmax | t apogee | RASAero t apogee | t burn | h burn ft | v burn fps | RASAero file ft | file err % |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    let mut rows = Vec::new();
    for f in &fx {
        let v = vehicle(f, &db).unwrap();
        let t3 = load_table(f, &db, "3d").unwrap();
        let r = run_of(f, &v, Some(&t3), &none);
        let (rt, rv) = rasaero_flight(&f.id);
        println!(
            "| {} | {} | {:.0} ({}) | {:.2} | {:.0} | {:.0} | {:.2} | {:.2} | {:.2} | {:.0} | {:.0} | {:.0} | {} |",
            f.id,
            f.measured.apogee_ft,
            r.apogee_ft,
            pct(r.apogee_ft, f.measured.apogee_ft),
            r.mach,
            r.vmax_fps,
            rv,
            r.t_apogee,
            rt,
            r.t_burn,
            r.h_burn_ft,
            r.v_burn_fps,
            f.rasaero.file_ft,
            pct(f.rasaero.file_ft, f.measured.apogee_ft)
        );
        rows.push((f, v, t3));
    }

    println!("\n## (c) rail guide: the fixture's, the example file's (RASAero reads it from the body tube or fin can), none; the table's `other` column rebuilt for each");
    println!("| rocket | fixture rail | file rail d x h | as stored | table other rebuilt, fixture rail | with file rail | none |");
    println!("|---|---|---|---|---|---|---|");
    let file_rail = |id: &str| match id {
        "byrum" => Some((0.375, 0.3438)),
        "cancer-descending" => Some((0.5, 0.5)),
        "ion-drive" => Some((0.375, 0.3438)),
        "rabia-short-fin-can" => Some((0.375, 0.5)),
        "torrent" => Some((0.375, 0.5)),
        _ => None,
    };
    for (f, v, t3) in &rows {
        let stored = apogee(f, v, Some(t3));
        let rebuilt = |rail: Option<(f64, f64)>| {
            let g = with_rail(f, rail);
            let gv = vehicle(&g, &db).unwrap();
            let t = refresh_other(&g, &gv, t3);
            apogee(&g, &gv, Some(&t))
        };
        let fixture_rail = f.rail_guide.as_ref().map(|r| (r.diameter, r.height));
        println!(
            "| {} | {:?} | {:?} | {:.0} | {:.0} | {:.0} | {:.0} |",
            f.id,
            fixture_rail,
            file_rail(&f.id),
            stored,
            rebuilt(fixture_rail),
            rebuilt(file_rail(&f.id)),
            rebuilt(None)
        );
    }

    println!("\n## (d) masses and the motor: ours against RASAero's motor file of the same designation");
    println!("| rocket | launch kg | motor | impulse Ns | burn s | prop kg | total kg | dry kg | burnout kg | RASAero motor | impulse Ns | burn s | prop kg | total kg | apogee on RASAero motor | d ft |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    let dir = std::env::var("RASAERO_MOTORS").ok();
    for (f, v, t3) in &rows {
        let st = &v.stages[0];
        let m = st.motor.as_ref().unwrap();
        let imp = |c: &ThrustCurve| c.total_impulse();
        let burn = |c: &ThrustCurve| c.points.last().map_or(0.0, |p| p.0);
        let launch_kg = f.launch_weight_lb * LB;
        let base = apogee(f, v, Some(t3));
        let (rname, ri, rb, rp, rt, ra) = match &dir {
            Some(d) => {
                let g = Fixture { motor: MotorF { file: Some(format!("{d}/{}.eng", f.id)), ..clone_fixture(f).motor }, ..clone_fixture(f) };
                let gv = vehicle(&g, &db).unwrap();
                let rm = gv.stages[0].motor.as_ref().unwrap();
                let a = apogee(&g, &gv, Some(t3));
                (g.motor.file.clone().unwrap_or_default(), imp(&rm.curve), burn(&rm.curve), rm.curve.propellant_mass, rm.curve.total_mass, a)
            }
            None => (String::new(), 0.0, 0.0, 0.0, 0.0, base),
        };
        println!(
            "| {} | {:.3} | {} {} | {:.0} | {:.2} | {:.3} | {:.3} | {:.3} | {:.3} | {} | {:.0} | {:.2} | {:.3} | {:.3} | {:.0} | {:+.0} |",
            f.id,
            launch_kg,
            f.motor.manufacturer,
            f.motor.designation,
            imp(&m.curve),
            burn(&m.curve),
            m.curve.propellant_mass,
            m.curve.total_mass,
            st.dry.mass,
            st.dry.mass + m.curve.total_mass - m.curve.propellant_mass,
            rname.rsplit('/').next().unwrap_or(""),
            ri,
            rb,
            rp,
            rt,
            ra,
            ra - base
        );
    }

    println!(
        "\n## (d2) launch weight: apogee change of +-5 % of the launch weight (same motor) and of the dry mass being the launch weight (motor not subtracted)"
    );
    println!("| rocket | -5 % | +5 % | motor not in the weight (dry = launch weight) |");
    println!("|---|---|---|---|");
    for (f, v, t3) in &rows {
        let a0 = apogee(f, v, Some(t3));
        let w = |k: f64| {
            let g = Fixture { launch_weight_lb: f.launch_weight_lb * k, ..clone_fixture(f) };
            let gv = vehicle(&g, &db).unwrap();
            apogee(&g, &gv, Some(t3)) - a0
        };
        let heavy = {
            let m = v.stages[0].motor.as_ref().unwrap();
            let g = Fixture { launch_weight_lb: f.launch_weight_lb + m.curve.total_mass / LB, ..clone_fixture(f) };
            let gv = vehicle(&g, &db).unwrap();
            apogee(&g, &gv, Some(t3)) - a0
        };
        println!("| {} | {:+.0} | {:+.0} | {:+.0} |", f.id, w(0.95), w(1.05), heavy);
    }

    println!("\n## (e) the exhaust: the base area covered while the motor burns is the nozzle exit the flight estimates from the thrust (0.5-0.8 of the case) / none (the stored tables are the power-off base)");
    println!("| rocket | case d in | estimated exit d in | RASAero II's exit d in | body d in | exit estimated | no correction |");
    println!("|---|---|---|---|---|---|---|");
    for (f, v, t3) in &rows {
        let a0 = apogee(f, v, Some(t3));
        let mut g = v.clone();
        g.stages[0].motor.as_mut().unwrap().curve.diameter = 0.0;
        let curve = &v.stages[0].motor.as_ref().unwrap().curve;
        println!(
            "| {} | {:.2} | {:.2} | {:.3} | {:.2} | {:.0} | {:.0} |",
            f.id,
            curve.diameter / IN,
            curve.nozzle_exit_diameter() / IN,
            nozzle_in(&f.id),
            f.body.iter().map(|s| s.fore_diameter.max(s.aft_diameter)).fold(0.0, f64::max),
            a0,
            apogee(f, &g, Some(t3))
        );
    }

    println!("\n## (f) wind, rod, site: the fixture's, then wind 0, rod vertical, ISA temperature, site at sea level");
    println!("| rocket | stored | wind 0 | rod 0 deg | temp ISA | site MSL 0 (table as is) | rod 10 ft -> 20 ft |");
    println!("|---|---|---|---|---|---|---|");
    for (f, v, t3) in &rows {
        let f_run = |tweak: &dyn Fn(&mut Launch)| run_of(f, v, Some(t3), tweak).apogee_ft;
        let isa = bowshock_sim::Atmosphere::standard().at_msl(f.site.altitude_ft * FT).temperature;
        println!(
            "| {} | {:.0} | {:+.0} | {:+.0} | {:+.0} | {:+.0} | {:+.0} |",
            f.id,
            f_run(&none),
            f_run(&|l| l.wind_speed = 0.0) - f_run(&none),
            f_run(&|l| l.angle_from_vertical = 0.0) - f_run(&none),
            f_run(&|l| l.ground_temperature = isa) - f_run(&none),
            f_run(&|l| l.site_altitude = 0.0) - f_run(&none),
            f_run(&|l| l.rail_length *= 2.0) - f_run(&none)
        );
    }
}

/// RASAero II's nozzle exit diameters (in) of the example files (`SustainerNozzle`), 0 where the file gives none.
fn nozzle_in(id: &str) -> f64 {
    match id {
        "dont-debate-this" => 2.488,
        "rabia-short-fin-can" => 1.187,
        "torrent" => 1.625,
        _ => 0.0,
    }
}

/// RASAero II's example-file flight: (apogee ft, time to apogee s, maximum velocity ft/s), the `Simulation` block of each example.
fn rasaero_triple(f: &Fixture) -> (f64, f64, f64) {
    let (t, v) = rasaero_flight(&f.id);
    (f.rasaero.file_ft, t, v)
}

/// The table with its friction column scaled by the build-up's friction of a surface of `roughness` (m) over that of a polished one, at the table's own Reynolds numbers: what
/// the CFD would give with the finish, to first order (the rough-wall solve itself needs the GPU).
fn rough_table(f: &Fixture, db: &MotorDb, table: &CfdTable, roughness: f64) -> (Vehicle, CfdTable) {
    let smooth = Fixture { roughness_in: 0.0, ..clone_fixture(f) };
    let rough = Fixture { roughness_in: roughness / IN, ..clone_fixture(f) };
    let (vs, vr) = (vehicle(&smooth, db).unwrap(), vehicle(&rough, db).unwrap());
    let atm = bowshock_sim::Atmosphere::standard();
    let mut t = table.clone();
    for p in &mut t.points {
        let fs = vs_mach(&vs, 0.0, (p.mach, p.mach), 1, 0.0, &atm, table.altitude)[0].drag.friction;
        let fr = vs_mach(&vr, 0.0, (p.mach, p.mach), 1, 0.0, &atm, table.altitude)[0].drag.friction;
        p.friction *= fr / fs;
        p.cd = p.friction + p.pressure + p.base + p.fins + p.other;
    }
    (vr, t)
}

#[test]
#[ignore = "diagnosis: prints tables from the stored tables, CPU"]
fn apogee_gap_finish() {
    let db = MotorDb::bundled();
    println!("\n## (h) surface finish: apogee error % against the measured of the stored (polished, DDT 6 um) 3D table with its friction column scaled to the finish (the build-up's ratio at the table's Re), geometric and read by a barometric altimeter; and of the build-up");
    println!("| rocket | fixture finish um | 3D as stored | 6 um | 25 um | 60 um (the app's default finish) | baro: as stored | 6 um | 25 um | 60 um | build-up as fixture | build-up 25 um | build-up 60 um | friction x at M0.6, 60 um |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for f in fixtures() {
        let v = vehicle(&f, &db).unwrap();
        let t3 = load_table(&f, &db, "3d").unwrap();
        let m = f.measured.apogee_ft;
        let base3 = apogee(&f, &v, Some(&t3));
        let baro = |a: f64| indicated_above_pad(&f, a * FT) / FT;
        let mut row = format!("| {} | {:.1} | {}", f.id, f.roughness_in * IN * 1e6, pct(base3, m));
        let mut baro_cells = format!(" | {}", pct(baro(base3), m));
        for um in [6.0, 25.0, 60.0] {
            let (vr, tr) = rough_table(&f, &db, &t3, um * 1e-6);
            let a = apogee(&f, &vr, Some(&tr));
            row += &format!(" | {}", pct(a, m));
            baro_cells += &format!(" | {}", pct(baro(a), m));
        }
        row += &baro_cells;
        row += &format!(" | {}", pct(apogee(&f, &v, None), m));
        for um in [25.0, 60.0] {
            let g = Fixture { roughness_in: um * 1e-6 / IN, ..clone_fixture(&f) };
            let gv = vehicle(&g, &db).unwrap();
            row += &format!(" | {}", pct(apogee(&g, &gv, None), m));
        }
        let (_, t60) = rough_table(&f, &db, &t3, 60e-6);
        let at = |t: &CfdTable| crate::analyze::cfd::interp::at(t, 0.6).friction;
        row += &format!(" | {:.2} |", at(&t60) / at(&t3));
        println!("{row}");
    }
}

#[test]
#[ignore = "diagnosis: the barometric altimeter's reading of the sim's apogee"]
fn apogee_gap_baro() {
    let db = MotorDb::bundled();
    println!("\n## (i) the barometric altimeter reads pressure altitude: the 3D apogee (geometric AGL) as the altimeter would read it in the site's atmosphere, against the measured");
    println!("| rocket | site F | site ft | geometric ft | indicated ft | indicated / geometric | geometric err % | indicated err % | RASAero err % |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for f in fixtures() {
        let v = vehicle(&f, &db).unwrap();
        let t3 = load_table(&f, &db, "3d").unwrap();
        let a = apogee(&f, &v, Some(&t3));
        let ind = indicated_above_pad(&f, a * FT) / FT;
        println!(
            "| {} | {:.0} | {:.0} | {a:.0} | {ind:.0} | {:.3} | {} | {} | {} |",
            f.id,
            f.site.temperature_f,
            f.site.altitude_ft,
            ind / a,
            pct(a, f.measured.apogee_ft),
            pct(ind, f.measured.apogee_ft),
            pct(f.rasaero.table_ft, f.measured.apogee_ft)
        );
    }
}

#[test]
#[ignore = "diagnosis: the drag parts of the tables against the build-up at the flight's Mach numbers"]
fn apogee_gap_parts() {
    let db = MotorDb::bundled();
    let atm = bowshock_sim::Atmosphere::standard();
    println!("\n## (j) zero-lift drag parts at M 0.3 / 0.6 / 0.8: 3D table (interpolated), legacy table, build-up (polished, as the fixture), all at the table's Reynolds numbers");
    println!("| rocket | M | source | friction | pressure | base | fins | other | total |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for f in fixtures() {
        let v = vehicle(&f, &db).unwrap();
        for m in [0.3, 0.6, 0.8] {
            for engine in ["3d", "legacy"] {
                let t = load_table(&f, &db, engine).unwrap();
                let p = crate::analyze::cfd::interp::at(&t, m);
                println!(
                    "| {} | {m} | {engine} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} |",
                    f.id, p.friction, p.pressure, p.base, p.fins, p.other, p.cd
                );
            }
            let d = vs_mach(&v, 0.0, (m, m), 1, 0.0, &atm, f.site.altitude_ft * FT)[0].drag;
            println!(
                "| {} | {m} | build-up | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} |",
                f.id, d.friction, d.pressure, d.base, d.fins, d.protuberances, d.total
            );
        }
    }
}

#[test]
#[ignore = "diagnosis: mass conventions against RASAero II's own flight"]
fn apogee_gap_mass() {
    let db = MotorDb::bundled();
    println!("\n## mass conventions: apogee ft / vmax ft/s / time to apogee s of the 3D table flown against RASAero II's file flight");
    println!("| rocket | variant | apogee | vmax | t apogee |");
    println!("|---|---|---|---|---|");
    for f in fixtures() {
        let v = vehicle(&f, &db).unwrap();
        let t3 = load_table(&f, &db, "3d").unwrap();
        let (ra, rt, rv) = rasaero_triple(&f);
        let show = |name: &str, g: &Fixture, gv: &Vehicle| {
            let r = run_of(g, gv, Some(&t3), &|_| {});
            println!("| {} | {name} | {:.0} | {:.0} | {:.2} |", f.id, r.apogee_ft, r.vmax_fps, r.t_apogee);
        };
        println!("| {} | RASAero II | {ra:.0} | {rv:.0} | {rt:.2} |", f.id);
        show("as flown", &f, &v);
        // no mass loss: the propellant stays in the rocket
        let mut c = v.clone();
        c.stages[0].motor.as_mut().unwrap().curve.propellant_mass = 0.0;
        show("mass constant at the launch weight", &f, &c);
        // the launch weight does not hold the motor
        let m = v.stages[0].motor.as_ref().unwrap().curve.total_mass;
        let g = Fixture { launch_weight_lb: f.launch_weight_lb + m / LB, ..clone_fixture(&f) };
        let gv = vehicle(&g, &db).unwrap();
        show("launch weight excludes the motor", &g, &gv);
        // the launch weight holds the motor but not the propellant
        let mut c = v.clone();
        let p = c.stages[0].motor.as_ref().unwrap().curve.propellant_mass;
        c.stages[0].dry.mass += p;
        show("launch weight is burnout weight + propellant counted twice", &f, &c);
    }
}

#[test]
#[ignore = "diagnosis: where on the Mach axis the 3D table's drag matters for the apogee"]
fn apogee_gap_nodes() {
    let db = MotorDb::bundled();
    println!("\n## (k) apogee change (ft) of +10 % on the drag of one node of the 3D table (its neighbours interpolate to it), and the share of the whole-table +10 % change");
    println!("| rocket | whole table | node: Mach -> change |");
    println!("|---|---|---|");
    for f in fixtures() {
        let v = vehicle(&f, &db).unwrap();
        let t3 = load_table(&f, &db, "3d").unwrap();
        let a0 = apogee(&f, &v, Some(&t3));
        let scaled = |only: Option<usize>| {
            let mut t = t3.clone();
            for (i, p) in t.points.iter_mut().enumerate().filter(|(i, _)| only.is_none_or(|o| o == *i)) {
                let _ = i;
                p.friction *= 1.1;
                p.pressure *= 1.1;
                p.base *= 1.1;
                p.fins *= 1.1;
                p.other *= 1.1;
                p.cd *= 1.1;
            }
            apogee(&f, &v, Some(&t)) - a0
        };
        let whole = scaled(None);
        let nodes: Vec<String> = (0..t3.points.len()).map(|i| format!("{:.1}: {:+.0}", t3.points[i].mach, scaled(Some(i)))).collect();
        println!("| {} | {whole:+.0} | {} |", f.id, nodes.join(", "));
    }
}
