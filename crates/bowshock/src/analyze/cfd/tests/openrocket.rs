use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Instant;

use bowshock_model::CfdGrid;
use bowshock_sim::{vs_mach, Atmosphere};

use super::*;
use crate::units::UnitSystem;

/// Linear interpolation in Mach of one OpenRocket drag column at alpha 0 (clamped at the ends).
fn or_at(rows: &[(f64, serde_json::Value)], mach: f64, key: &str) -> f64 {
    let f = |r: &(f64, serde_json::Value)| r.1[key].as_f64().unwrap_or(f64::NAN);
    let i = rows.partition_point(|r| r.0 < mach);
    if i == 0 {
        return f(&rows[0]);
    }
    if i == rows.len() {
        return f(&rows[i - 1]);
    }
    let (a, b) = (&rows[i - 1], &rows[i]);
    let t = (mach - a.0) / (b.0 - a.0);
    f(a) + t * (f(b) - f(a))
}

/// Compares the CFD drag table, the Extended method and OpenRocket's own static drag on OpenRocket's example rockets.
///
/// For every `*.ork` in `$OR_ROCKETS` (default `/tmp/bowshock-openrocket/rockets`) with a `<stem>.json` in `$OR_REF`
/// (default `/tmp/bowshock-openrocket/ref`, written by `tools/openrocket-harness`) it imports the file, builds the model and
/// the vehicle for the configuration OpenRocket's first simulation flies, solves the coarse CFD table at sea level and prints
/// a Markdown table of the parts against Mach at alpha 0. OpenRocket is rescaled to the app's reference area when they differ.
/// Rockets the solver cannot take are named and skipped.
///
/// `cargo test --release -p bowshock -- --ignored cfd_against_openrocket --nocapture`
#[test]
#[ignore = "solves a CFD table per rocket on the GPU"]
fn cfd_against_openrocket() {
    use serde_json::Value;
    let ref_dir = std::path::PathBuf::from(std::env::var("OR_REF").unwrap_or_else(|_| "/tmp/bowshock-openrocket/ref".into()));
    let rockets = std::path::PathBuf::from(std::env::var("OR_ROCKETS").unwrap_or_else(|_| "/tmp/bowshock-openrocket/rockets".into()));
    let mut files: Vec<_> =
        std::fs::read_dir(&rockets).unwrap().filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.extension().is_some_and(|x| x == "ork")).collect();
    files.sort();
    let atm = Atmosphere::standard();
    for path in files {
        let stem = path.file_stem().unwrap().to_string_lossy().to_string();
        let Ok(text) = std::fs::read_to_string(ref_dir.join(format!("{stem}.json"))) else { continue };
        let reference: Value = serde_json::from_str(&text).unwrap();
        println!("\n# {stem}\n");
        let imp = match bowshock_model::ork::import_file(&path) {
            Ok(i) => i,
            Err(e) => {
                println!("SKIPPED: import failed: {e}");
                continue;
            }
        };
        let mut rocket = imp.rocket.clone();
        let configs = reference["configs"].as_array().cloned().unwrap_or_default();
        let first = reference["simulations"].as_array().and_then(|s| s.first()).and_then(|s| s["config"].as_str()).map(str::to_string);
        let cfg = first.as_deref().and_then(|id| configs.iter().find(|c| c["id"] == id)).or(configs.first()).expect("a configuration").clone();
        let short = cfg["id"].as_str().unwrap_or("").to_string();
        let ids: Vec<u64> = rocket.configurations().iter().map(|c| c.id).collect();
        if let Some(i) = imp.config_ids.iter().position(|id| id.starts_with(&short)) {
            rocket.select_configuration(ids[i]);
        }
        let mut db = MotorDb::default();
        for m in cfg["motors"].as_array().into_iter().flatten() {
            if let Some(file) = m["eng_file"].as_str() {
                let _ = db.load_eng_file(ref_dir.join(file));
            }
        }
        db.motors.extend(MotorDb::bundled().motors);
        let vehicle = match from_model::vehicle(&rocket, &db, Phase::Full) {
            Ok(v) => v,
            Err(e) => {
                println!("SKIPPED: vehicle: {e:?}");
                continue;
            }
        };
        let plan = match plan(&rocket, &vehicle, UnitSystem::default()) {
            Ok(p) => p,
            Err(why) => {
                println!("SKIPPED: {why}");
                continue;
            }
        };
        let app_area = std::f64::consts::PI * vehicle.reference_diameter.powi(2) / 4.0;
        let or_area = cfg["reference_area"].as_f64().unwrap_or(app_area);
        let scale = or_area / app_area;
        println!(
            "OpenRocket {} configuration {} ({}); notes {:?}",
            reference["openrocket_version"].as_str().unwrap_or("?"),
            short,
            cfg["name"].as_str().unwrap_or(""),
            plan.notes
        );
        if (scale - 1.0).abs() > 1e-3 {
            println!("Reference area differs: OpenRocket {or_area:.6} m2, app {app_area:.6} m2; OpenRocket Cd rescaled by {scale:.4} to the app's.");
        } else {
            println!("Reference area equal: {app_area:.6} m2.");
        }
        let mut finishes: Vec<String> = rocket.parts().iter().map(|p| format!("{:?} ({:.0} um)", p.finish, p.finish.roughness() * 1e6)).collect();
        finishes.dedup();
        println!(
            "Model body finishes: {}; OpenRocket perfect_finish = {}; CFD walls use the same finishes.\n",
            finishes.join(", "),
            reference["perfect_finish"]
        );
        let mut rows: Vec<(f64, Value)> = cfg["aero"]
            .as_array()
            .cloned()
            .unwrap_or_default()
            .into_iter()
            .filter(|r| r["alpha_deg"].as_f64() == Some(0.0))
            .map(|r| (r["mach"].as_f64().unwrap(), r))
            .collect();
        rows.sort_by(|a, b| a.0.total_cmp(&b.0));
        let nodes = nodes((0.1, 3.0)).unwrap();
        let job = Job { plan, altitude: 0.0, nodes, stage: 1, grid: CfdGrid::Coarse, compute: Default::default(), cpu_only: false };
        let t0 = Instant::now();
        let table = match solve(&job, &Arc::new(AtomicBool::new(false)), |_| ()) {
            Ok(t) => t,
            Err(e) => {
                println!("SKIPPED: solve failed: {e}");
                continue;
            }
        };
        println!("Solved {} points in {:.0} s.\n", table.points.len(), t0.elapsed().as_secs_f64());
        let cg = cfg["launch_cg_x"].as_f64().unwrap_or(0.0);
        println!("| Mach | OR Cd | OR fric / press / base | Ext Cd | Ext fric / press / base / fins / prot | CFD Cd | CFD fric / press / base / fins / other | CFD vs OR % | Ext vs OR % |");
        println!("|---|---|---|---|---|---|---|---|---|");
        for p in &table.points {
            let g = |k: &str| scale * or_at(&rows, p.mach, k);
            let o = g("cd");
            let e = &vs_mach(&vehicle, cg, (p.mach, p.mach), 1, 0.0, &atm, 0.0)[0].drag;
            println!(
                "| {:.2} | {:.4} | {:.4} / {:.4} / {:.4} | {:.4} | {:.4} / {:.4} / {:.4} / {:.4} / {:.4} | {:.4} | {:.4} / {:.4} / {:.4} / {:.4} / {:.4} | {:+.1} | {:+.1} |",
                p.mach,
                o,
                g("cd_friction"),
                g("cd_pressure"),
                g("cd_base"),
                e.total,
                e.friction,
                e.pressure,
                e.base,
                e.fins,
                e.protuberances,
                p.cd,
                p.friction,
                p.pressure,
                p.base,
                p.fins,
                p.other,
                100.0 * (p.cd / o - 1.0),
                100.0 * (e.total / o - 1.0),
            );
        }
    }
}
