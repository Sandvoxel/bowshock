//! Compares Bowshock with OpenRocket on the same rockets.
//!
//! ```text
//! tools/compare-openrocket/run.sh            # exports the references with the harness, then runs this
//! cargo run --release -p bowshock-sim --example compare_openrocket -- <harness-dir> <label>=<file.ork> ...
//! ```
//!
//! For each `.ork`: import it ([`bowshock_model::ork`]), read OpenRocket's reference output for it (`<harness-dir>/<stem>.json`,
//! written by `tools/openrocket-harness`), fly the same configurations with the thrust curves OpenRocket used
//! (`<harness-dir>/motors/*.eng`), and print Markdown tables: static aerodynamics against Mach at alpha = 0 and 2 degrees, mass
//! and CG, and the no-wind flights. Differences are Bowshock relative to OpenRocket.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use bowshock_model::ork;
use bowshock_motors::MotorDb;
use bowshock_sim::atmosphere::Atmosphere;
use bowshock_sim::from_model::{balance, sim_input, vehicle, Phase};
use bowshock_sim::{simulate, Aero, Launch, Turbulence};
use serde_json::Value;

const MACHS: [f64; 9] = [0.1, 0.3, 0.5, 0.8, 0.9, 1.0, 1.2, 1.5, 2.0];

fn f(v: &Value, key: &str) -> f64 {
    v.get(key).and_then(Value::as_f64).unwrap_or(f64::NAN)
}

fn pct(bs: f64, or: f64) -> f64 {
    if or.abs() < 1e-12 {
        f64::NAN
    } else {
        100.0 * (bs - or) / or
    }
}

fn cell(x: f64, digits: usize) -> String {
    if x.is_nan() {
        "–".to_string()
    } else {
        format!("{x:.digits$}")
    }
}

fn signed(x: f64) -> String {
    if x.is_nan() {
        "–".to_string()
    } else {
        format!("{x:+.1}%")
    }
}

/// The motors of one configuration: the curves OpenRocket flew it with (its exported `.eng` files, which win over a same-named
/// motor of another configuration), then the bundled set.
fn motor_db(dir: &Path, config: &Value) -> MotorDb {
    let mut db = MotorDb::default();
    for m in config["motors"].as_array().into_iter().flatten() {
        if let Some(file) = m["eng_file"].as_str() {
            if let Err(e) = db.load_eng_file(dir.join(file)) {
                eprintln!("warning: {file}: {e}");
            }
        }
    }
    db.motors.extend(MotorDb::bundled().motors);
    db
}

fn main() {
    let mut args = std::env::args().skip(1);
    let dir = PathBuf::from(args.next().expect("usage: compare_openrocket <harness-dir> <label>=<file.ork>..."));
    let mut summary: Vec<(String, Vec<String>)> = Vec::new();
    for spec in args {
        let (label, path) = spec.split_once('=').unwrap_or((spec.as_str(), spec.as_str()));
        let stem = Path::new(path).file_stem().and_then(|s| s.to_str()).unwrap_or(path).to_string();
        let reference: Value = match std::fs::read_to_string(dir.join(format!("{stem}.json")))
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
        {
            Ok(v) => v,
            Err(e) => {
                eprintln!("{label}: no reference output ({e})");
                continue;
            }
        };
        let imp = match ork::import_file(path) {
            Ok(i) => i,
            Err(e) => {
                eprintln!("{label}: {e}");
                continue;
            }
        };
        let rows = compare(label, &reference, &imp, &dir);
        summary.push((label.to_string(), rows));
    }
    println!("\n## Summary\n");
    println!("| rocket | worst CP Δ (cal) | worst CNα Δ | worst Cd Δ (M≤2) | apogee Δ | max v Δ |");
    println!("|---|---|---|---|---|---|");
    for (label, rows) in summary {
        println!("| {label} | {} |", rows.join(" | "));
    }
}

#[derive(Default)]
struct Worst {
    cp_cal: f64,
    cna: f64,
    cd: f64,
    apogee: f64,
    vmax: f64,
}

fn upd(slot: &mut f64, v: f64) {
    if v.is_finite() && v.abs() > slot.abs() {
        *slot = v;
    }
}

fn compare(label: &str, reference: &Value, imp: &ork::OrkImport, dir: &Path) -> Vec<String> {
    let configs_json = reference["configs"].as_array().cloned().unwrap_or_default();
    let db_of = |short: &str| -> MotorDb { motor_db(dir, configs_json.iter().find(|c| c["id"] == short).unwrap_or(&Value::Null)) };
    println!("\n# {label}\n");
    println!(
        "OpenRocket {} — {} stage(s); import warnings: {}.\n",
        reference["openrocket_version"].as_str().unwrap_or("?"),
        reference["stage_count"],
        imp.warnings.len()
    );
    for w in &imp.warnings {
        println!("- import: {w}");
    }
    let mut rocket = imp.rocket.clone();
    let mut worst = Worst::default();

    // Map OpenRocket's short configuration keys to the model's configurations.
    let model_config = |short: &str| -> Option<u64> {
        let i = imp.config_ids.iter().position(|id| id.starts_with(short))?;
        rocket_config_ids(&imp.rocket).get(i).copied()
    };
    let configs = reference["configs"].as_array().cloned().unwrap_or_default();
    let sims = reference["simulations"].as_array().cloned().unwrap_or_default();
    // Configurations that a simulation flies, in first-simulation order; static tables for the first of them.
    let mut flown: Vec<String> = Vec::new();
    for s in &sims {
        let c = s["config"].as_str().unwrap_or("").to_string();
        if !flown.contains(&c) {
            flown.push(c);
        }
    }

    // ---- mass and CG, all configurations ----
    println!("\n## Mass and CG at liftoff\n");
    println!("| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|");
    for c in &configs {
        let short = c["id"].as_str().unwrap_or("");
        let Some(cid) = model_config(short) else { continue };
        rocket.select_configuration(cid);
        let db = db_of(short);
        let (Ok(on), Ok(off)) = (balance(&rocket, &db, Phase::Full, 0.3, 0.0, true), balance(&rocket, &db, Phase::Full, 0.3, 0.0, false)) else {
            println!("| {} | (motor missing) | | | | | | | | | |", c["name"].as_str().unwrap_or(short));
            continue;
        };
        let d = on.reference_diameter;
        println!(
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |",
            c["name"].as_str().unwrap_or(short),
            cell(f(c, "launch_mass_kg"), 3),
            cell(on.mass, 3),
            signed(pct(on.mass, f(c, "launch_mass_kg"))),
            cell(f(c, "launch_cg_x"), 4),
            cell(on.cg, 4),
            cell((on.cg - f(c, "launch_cg_x")) / d, 3),
            cell(f(c, "burnout_mass_kg"), 3),
            cell(off.mass, 3),
            cell(f(c, "burnout_cg_x"), 4),
            cell(off.cg, 4),
        );
    }

    // ---- static aero ----
    let first = flown.first().cloned().or_else(|| configs.first().and_then(|c| c["id"].as_str().map(str::to_string)));
    if let Some(short) = first {
        if let (Some(cfg), Some(cid)) = (configs.iter().find(|c| c["id"] == short.as_str()), model_config(&short)) {
            rocket.select_configuration(cid);
            let db = db_of(&short);
            match vehicle(&rocket, &db, Phase::Full) {
                Ok(v) => {
                    let aero = Aero::new(&v);
                    let air = Atmosphere::standard().at_msl(0.0);
                    let d = aero.reference_diameter();
                    let table: BTreeMap<(u32, u32), &Value> = cfg["aero"]
                        .as_array()
                        .map(|a| a.iter().map(|r| (((f(r, "mach") * 1000.0).round() as u32, (f(r, "alpha_deg") * 10.0).round() as u32), r)).collect())
                        .unwrap_or_default();
                    for alpha_deg in [0.0, 2.0] {
                        println!("\n## Static aerodynamics, α = {alpha_deg}° (configuration {}, ISA sea level)\n", cfg["name"].as_str().unwrap_or(&short));
                        println!("| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |");
                        println!("|---|---|---|---|---|---|---|---|---|---|---|---|");
                        let alpha = alpha_deg * std::f64::consts::PI / 180.0;
                        // Worst-case over the whole sweep.
                        for (&(m1000, a10), r) in &table {
                            if a10 != (alpha_deg * 10.0) as u32 {
                                continue;
                            }
                            let mach = f64::from(m1000) / 1000.0;
                            let re = mach * air.speed_of_sound / air.kinematic_viscosity();
                            let st = aero.stability(mach, alpha);
                            let dr = aero.drag_at_alpha(mach, re, false, alpha);
                            // OpenRocket's total CD does not change with alpha; the flight simulation uses its axial value.
                            let or_cd = if alpha_deg > 0.0 { f(r, "cd_axial") } else { f(r, "cd") };
                            upd(&mut worst.cp_cal, (st.cp - f(r, "cp_x")) / d);
                            upd(&mut worst.cna, pct(st.cn_alpha, f(r, "cna")));
                            upd(&mut worst.cd, pct(dr.total, or_cd));
                            if MACHS.iter().any(|m| (m - mach).abs() < 1e-9) {
                                println!(
                                    "| {mach:.2} | {} | {} | {:+.3} | {} | {} | {} | {} | {} | {} | {}/{}/{} | {}/{}/{} |",
                                    cell(f(r, "cp_x"), 4),
                                    cell(st.cp, 4),
                                    (st.cp - f(r, "cp_x")) / d,
                                    cell(f(r, "cna"), 3),
                                    cell(st.cn_alpha, 3),
                                    signed(pct(st.cn_alpha, f(r, "cna"))),
                                    cell(or_cd, 4),
                                    cell(dr.total, 4),
                                    signed(pct(dr.total, or_cd)),
                                    cell(f(r, "cd_friction"), 3),
                                    cell(f(r, "cd_pressure"), 3),
                                    cell(f(r, "cd_base"), 3),
                                    cell(dr.friction + dr.fins + dr.protuberances, 3),
                                    cell(dr.pressure, 3),
                                    cell(dr.base, 3),
                                );
                            }
                        }
                    }
                    // Drag by category at alpha = 0: OpenRocket's per-component figures grouped like Bowshock's breakdown.
                    if let Some(rows) = cfg["aero_components"].as_array() {
                        println!("\n## Drag by category, α = 0° (OR | BS)\n");
                        println!("| Mach | body friction | body pressure | base | fins | lugs/buttons | total |");
                        println!("|---|---|---|---|---|---|---|");
                        let mut machs: Vec<f64> = rows.iter().map(|r| f(r, "mach")).collect();
                        machs.dedup();
                        for mach in machs {
                            let (mut fr, mut pr, mut ba, mut fi, mut lu) = (0.0, 0.0, 0.0, 0.0, 0.0);
                            for r in rows.iter().filter(|r| (f(r, "mach") - mach).abs() < 1e-9) {
                                let t = r["type"].as_str().unwrap_or("");
                                if t.contains("Fin") {
                                    fi += f(r, "cd");
                                } else if t == "Launch Lug" || t == "Rail Button" {
                                    lu += f(r, "cd");
                                } else if t == "Rocket" || t == "Stage" {
                                    continue;
                                } else {
                                    fr += f(r, "cd_friction");
                                    pr += f(r, "cd_pressure");
                                    ba += f(r, "cd_base");
                                }
                            }
                            let re = mach * air.speed_of_sound / air.kinematic_viscosity();
                            let b = aero.drag_at_alpha(mach, re, false, 0.0);
                            println!(
                                "| {mach:.2} | {:.4} \\| {:.4} | {:.4} \\| {:.4} | {:.4} \\| {:.4} | {:.4} \\| {:.4} | {:.4} \\| {:.4} | {:.4} \\| {:.4} |",
                                fr,
                                b.friction,
                                pr,
                                b.pressure,
                                ba,
                                b.base,
                                fi,
                                b.fins,
                                lu,
                                b.protuberances,
                                fr + pr + ba + fi + lu,
                                b.total
                            );
                        }
                    }
                }
                Err(e) => println!("\n(static aerodynamics skipped: {e})"),
            }
        }
    }

    // ---- flights ----
    println!("\n## Flights (no wind, ISA)\n");
    println!("| simulation | metric | OpenRocket | Bowshock | Δ |");
    println!("|---|---|---|---|---|");
    for s in sims.iter().filter(|s| s["variant"] == "calm") {
        let Some(cid) = model_config(s["config"].as_str().unwrap_or("")) else { continue };
        rocket.select_configuration(cid);
        let c = &s["conditions"];
        let altitude = f(c, "altitude_m");
        let launch = Launch {
            rail_length: f(c, "rod_length_m"),
            angle_from_vertical: f(c, "rod_angle_deg").to_radians(),
            direction: 0.0,
            site_altitude: altitude,
            ground_temperature: Atmosphere::standard().at_msl(altitude).temperature,
            wind_speed: 0.0,
            wind_from: 0.0,
            turbulence: Turbulence::Off,
            seed: 1,
        };
        let name = s["name"].as_str().unwrap_or("?");
        let db = db_of(s["config"].as_str().unwrap_or(""));
        let input = match sim_input(&rocket, &db, Phase::Full, launch) {
            Ok(i) => i,
            Err(e) => {
                println!("| {name} | (skipped: {e}) | | | |");
                continue;
            }
        };
        let result = simulate(&input);
        // Bowshock takes the motor-exhaust area out of the base drag while a motor burns; OpenRocket does not. The same flight
        // without that credit (motor diameter 0) shows how much of the difference it is.
        let mut no_credit = input.clone();
        for stage in &mut no_credit.vehicle.stages {
            if let Some(m) = &mut stage.motor {
                m.curve.diameter = 0.0;
            }
        }
        let plain = simulate(&no_credit).summary;
        let sm = &result.summary;
        let r = &s["results"];
        // OpenRocket's own 50 ms step overshoots the rail end; its rail-exit speed is taken from the 2 ms run.
        let fine = sims.iter().find(|x| x["variant"] == "calm_fine" && x["name"] == s["name"] && x["config"] == s["config"]).map_or(r, |x| &x["results"]);
        let rows: [(&str, f64, f64); 10] = [
            ("apogee (m)", f(r, "apogee_m"), sm.apogee),
            ("apogee (m), BS without the exhaust base-drag credit", f(r, "apogee_m"), plain.apogee),
            ("max velocity (m/s)", f(r, "max_velocity_ms"), sm.max_velocity),
            ("max velocity (m/s), BS without the credit", f(r, "max_velocity_ms"), plain.max_velocity),
            ("max Mach", f(r, "max_mach"), sm.max_mach),
            ("time to apogee (s)", f(r, "time_to_apogee_s"), sm.time_to_apogee),
            ("rail-exit velocity (m/s, OR at 2 ms step)", f(fine, "rail_exit_velocity_ms"), sm.rail_exit_velocity),
            ("stability off rail (cal)", f(r, "stability_off_rail_cal"), sm.stability_off_rail_cal),
            ("ground-hit velocity (m/s)", f(r, "ground_hit_velocity_ms"), sm.ground_hit_velocity),
            ("flight time (s)", f(r, "flight_time_s"), sm.flight_time),
        ];
        for (metric, or, bs) in rows {
            println!("| {name} | {metric} | {} | {} | {} |", cell(or, 3), cell(bs, 3), signed(pct(bs, or)));
        }
        upd(&mut worst.apogee, pct(sm.apogee, f(r, "apogee_m")));
        upd(&mut worst.vmax, pct(sm.max_velocity, f(r, "max_velocity_ms")));
    }
    vec![format!("{:+.3}", worst.cp_cal), signed(worst.cna), signed(worst.cd), signed(worst.apogee), signed(worst.vmax)]
}

fn rocket_config_ids(r: &bowshock_model::Rocket) -> Vec<u64> {
    r.configurations().iter().map(|c| c.id).collect()
}
