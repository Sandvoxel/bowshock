//! Tangent-ogive-cylinder validation driver.
//!
//! * M = 1.98: fineness-ratio-3 tangent ogive + cylinder vs the Perkins & Jorgensen wind-tunnel pressures
//!   (NACA RM A54H23, Table I(a)), tangent-cone and shock-expansion theory, coarse/medium/fine grids.
//! * `--sweep`: forebody pressure drag vs Mach (1.5, 3.0, 4.0 in addition) against theory only (no data).
//!
//! Usage: `cargo run --release --bin ogive [-- --quick] [--sweep] [--limiter minmod|vanalbada]`
//! `--quick` runs only the coarse and medium grids. `--stretch s` clusters nose cells towards the tip
//! (`x = L (k/n)^s`; output then goes to `out/ogive_stretch<s>/`). Output: `out/ogive/`.

use bowshock_core::flux::Limiter;
use bowshock_cases::ogive::{perkins_jorgensen as pj, run_ogive, theory_for, wall_cp_at, OgiveCase, OgiveResult};
use bowshock_reference::shock_expansion::{Ogive, SurfaceTheory};
use std::fmt::Write as _;
use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

const GRID_NAMES: [&str; 3] = ["coarse", "medium", "fine"];

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let quick = args.iter().any(|a| a == "--quick");
    let sweep = args.iter().any(|a| a == "--sweep");
    let limiter = match args.iter().position(|a| a == "--limiter").and_then(|i| args.get(i + 1)).map(String::as_str) {
        Some("vanalbada") => Limiter::VanAlbada,
        _ => Limiter::Minmod,
    };
    let stretch: f64 = args.iter().position(|a| a == "--stretch").and_then(|i| args.get(i + 1)).and_then(|v| v.parse().ok()).unwrap_or(1.0);
    let dir_name = if stretch == 1.0 { "ogive".to_string() } else { format!("ogive_stretch{stretch}") };
    let out_dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("out").join(dir_name);
    fs::create_dir_all(&out_dir).expect("create out dir");
    let n_levels: u32 = if quick { 2 } else { 3 };

    let body = Ogive { fineness: pj::FINENESS };
    let mut md = String::new();
    let mut drag_csv = String::from("mach,source,grid,cd_forebody,note\n");
    println!("bowshock-core ogive validation: tangent ogive L/d = {}, limiter = {limiter:?}, CFL = 0.5", pj::FINENESS);
    println!("threads (rayon): {}", rayon::current_num_threads());

    // ---------------- M = 1.98 against the wind-tunnel data ----------------
    let mach = pj::MACH;
    let th = SurfaceTheory::new(mach, body);
    let tip = th.tip.expect("attached tip cone");
    println!(
        "\n=== M = {mach}: tip half angle {:.3} deg, Taylor-Maccoll tip cone beta = {:.3} deg, Cp = {:.5}",
        body.tip_half_angle().to_degrees(),
        tip.beta.to_degrees(),
        tip.cp_surface
    );

    // Experimental forebody drag with explicit bounds for the unmeasured ends of the nose.
    let (x0, x1) = (pj::X_OVER_D[0], pj::X_OVER_D[1]);
    let tip_extrap = pj::CP[0] + (pj::CP[0] - pj::CP[1]) / (x1 - x0) * x0; // linear extrapolation to x = 0
    let cd_nom = pj::forebody_cd(&body, tip_extrap, pj::CP[7]);
    let tip_lo = pj::CP[0].min(tip_extrap).min(tip.cp_surface);
    let tip_hi = pj::CP[0].max(tip_extrap).max(tip.cp_surface);
    let cd_lo = pj::forebody_cd(&body, tip_lo, pj::CP[7]) - pj::CP_UNCERTAINTY;
    let cd_hi = pj::forebody_cd(&body, tip_hi, 0.0) + pj::CP_UNCERTAINTY;
    println!(
        "experiment: CD_forebody = {cd_nom:.4} (tip Cp {tip_extrap:.3} linear extrapolation, tail Cp {:.3}); bounds [{cd_lo:.4}, {cd_hi:.4}] (tip Cp {tip_lo:.3}..{tip_hi:.3}, tail Cp {:.3}..0, +-{} Cp)",
        pj::CP[7], pj::CP[7], pj::CP_UNCERTAINTY
    );
    let cd_se = th.forebody_cd(SurfaceTheory::cp_shock_expansion, 4000);
    let cd_tc = th.forebody_cd(SurfaceTheory::cp_tangent_cone, 400);
    println!("theory: CD_forebody shock-expansion = {cd_se:.4}, tangent-cone = {cd_tc:.4}");
    drag_csv += &format!("{mach},experiment (Perkins-Jorgensen Table I(a)),,{cd_nom:.5},tip Cp {tip_extrap:.3} linear extrapolation; tail Cp {:.3}; bounds [{cd_lo:.5} {cd_hi:.5}]\n", pj::CP[7]);
    drag_csv += &format!("{mach},shock-expansion,,{cd_se:.5},\n{mach},tangent-cone,,{cd_tc:.5},\n");

    let mut results: Vec<(String, usize, usize, OgiveResult, f64)> = Vec::new();
    for lv in 0..n_levels {
        let mut case = OgiveCase::level(mach, lv);
        case.limiter = limiter;
        case.nose_stretch = stretch;
        let (ni, nj) = case.cells();
        let t0 = Instant::now();
        let res = run_ogive(&case);
        let dt = t0.elapsed().as_secs_f64();
        let name = GRID_NAMES[lv as usize];
        write_case_files(&out_dir, mach, name, &res, &th);
        report_run(name, ni, nj, &res, dt);
        drag_csv += &format!(
            "{mach},cfd,{name} {ni}x{nj},{:.5},first-cell Cp variant {:.5}; {} in {} iters; {:.1} orders; shock margin {:.3}\n",
            res.cd_nose, res.cd_nose_cell, res.stop_reason, res.iters, res.drop_orders, res.shock_margin
        );
        results.push((name.to_string(), ni, nj, res, dt));
    }

    // Cp comparison at the orifice stations
    let mut f = fs::File::create(out_dir.join("cp_compare_M1.98.csv")).unwrap();
    let mut header = String::from("x_over_d,cp_experiment,cp_shock_expansion,cp_tangent_cone");
    for r in &results {
        write!(header, ",cp_cfd_{}", r.0).unwrap();
    }
    writeln!(f, "{header}").unwrap();
    for (k, &x) in pj::X_OVER_D.iter().enumerate() {
        let tcone = if x <= body.nose_length() { th.cp_tangent_cone(x) } else { f64::NAN };
        let mut line = format!("{x},{},{:.5},{:.5}", pj::CP[k], th.cp_shock_expansion(x), tcone);
        for r in &results {
            write!(line, ",{:.5}", wall_cp_at(&r.3.wall, x)).unwrap();
        }
        writeln!(f, "{line}").unwrap();
    }

    // error statistics: all 15 stations and the 8 nose stations
    let stat = |model: &dyn Fn(f64) -> f64, n: usize| -> (f64, f64) {
        let e: Vec<f64> = (0..n).map(|k| model(pj::X_OVER_D[k]) - pj::CP[k]).collect();
        let rms = (e.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
        let mx = e.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        (rms, mx)
    };
    writeln!(md, "### M = {mach}: Cp at the 15 orifice stations (Perkins & Jorgensen Table I(a))\n").unwrap();
    let mut hdr = String::from("| x/d | Cp exp | shock-exp. | tangent-cone |");
    let mut sep = String::from("|---|---|---|---|");
    for r in &results {
        write!(hdr, " CFD {} {}x{} |", r.0, r.1, r.2).unwrap();
        sep += "---|";
    }
    writeln!(md, "{hdr}\n{sep}").unwrap();
    for (k, &x) in pj::X_OVER_D.iter().enumerate() {
        let tcone = if x <= body.nose_length() { format!("{:.3}", th.cp_tangent_cone(x)) } else { "-".into() };
        let mut line = format!("| {x:.2} | {:.3} | {:.3} | {tcone} |", pj::CP[k], th.cp_shock_expansion(x));
        for r in &results {
            write!(line, " {:.3} |", wall_cp_at(&r.3.wall, x)).unwrap();
        }
        writeln!(md, "{line}").unwrap();
    }
    writeln!(md, "\nCp error vs experiment (model - experiment); nose = first 8 stations (x/d <= 2.95), all = 15 stations:\n").unwrap();
    writeln!(md, "| model | nose RMS | nose max | all RMS | all max |\n|---|---|---|---|---|").unwrap();
    let mut models: Vec<(String, Box<dyn Fn(f64) -> f64>)> = vec![("shock-expansion".into(), Box::new(|x| th.cp_shock_expansion(x)))];
    models.push(("tangent-cone (nose only)".into(), Box::new(|x| th.cp_tangent_cone(x))));
    for r in &results {
        let wall = r.3.wall.clone();
        models.push((format!("CFD {} {}x{}", r.0, r.1, r.2), Box::new(move |x| wall_cp_at(&wall, x))));
    }
    for (name, m) in &models {
        let (nr, nm) = stat(m.as_ref(), 8);
        let all = if name.starts_with("tangent") { "-".to_string() } else { let (r, mx) = stat(m.as_ref(), 15); format!("{r:.4} | {mx:.4}") };
        let all = if all == "-" { "- | -".into() } else { all };
        writeln!(md, "| {name} | {nr:.4} | {nm:.4} | {all} |").unwrap();
    }

    writeln!(md, "\n### M = {mach}: forebody pressure-drag coefficient (base-area reference, nose 0 <= x <= 3d)\n").unwrap();
    writeln!(md, "| source | CD_forebody | vs experiment nominal |\n|---|---|---|").unwrap();
    writeln!(md, "| experiment (Table I(a), tip extrapolated) | {cd_nom:.4} (bounds {cd_lo:.4} .. {cd_hi:.4}) | - |").unwrap();
    let rel = |c: f64| format!("{:+.1} %", 100.0 * (c - cd_nom) / cd_nom);
    writeln!(md, "| shock-expansion | {cd_se:.4} | {} |", rel(cd_se)).unwrap();
    writeln!(md, "| tangent-cone | {cd_tc:.4} | {} |", rel(cd_tc)).unwrap();
    for r in &results {
        writeln!(md, "| CFD {} {}x{} | {:.4} (first-cell Cp: {:.4}) | {} |", r.0, r.1, r.2, r.3.cd_nose, r.3.cd_nose_cell, rel(r.3.cd_nose)).unwrap();
    }

    // ---------------- Mach sweep against theory ----------------
    if sweep {
        writeln!(md, "\n### Forebody pressure drag vs Mach (fineness-ratio-3 tangent ogive): CFD vs theory (no experimental data)\n").unwrap();
        writeln!(md, "| M | tip cone beta [deg] | shock-expansion | tangent-cone | CFD medium | CFD fine | fine vs tangent-cone |\n|---|---|---|---|---|---|---|").unwrap();
        for &m in &[1.5f64, 3.0, 4.0] {
            let mut cds = Vec::new();
            let mut thm_ = None;
            for lv in 1..n_levels {
                let mut case = OgiveCase::level(m, lv);
                case.limiter = limiter;
                case.nose_stretch = stretch;
                let th = theory_for(&case);
                let (ni, nj) = case.cells();
                let t0 = Instant::now();
                let res = run_ogive(&case);
                let name = GRID_NAMES[lv as usize];
                write_case_files(&out_dir, m, name, &res, &th);
                report_run(name, ni, nj, &res, t0.elapsed().as_secs_f64());
                drag_csv += &format!(
                    "{m},cfd,{name} {ni}x{nj},{:.5},{} in {} iters; {:.1} orders; shock margin {:.3}\n",
                    res.cd_nose, res.stop_reason, res.iters, res.drop_orders, res.shock_margin
                );
                cds.push(res.cd_nose);
                thm_ = Some(th);
            }
            let th = thm_.unwrap();
            let (se, tc) = (th.forebody_cd(SurfaceTheory::cp_shock_expansion, 4000), th.forebody_cd(SurfaceTheory::cp_tangent_cone, 400));
            drag_csv += &format!("{m},shock-expansion,,{se:.5},\n{m},tangent-cone,,{tc:.5},\n");
            let fine = cds.get(1).copied();
            writeln!(
                md,
                "| {m} | {:.2} | {se:.4} | {tc:.4} | {:.4} | {} | {} |",
                th.tip.map_or(f64::NAN, |t| t.beta.to_degrees()),
                cds[0],
                fine.map_or("-".into(), |c| format!("{c:.4}")),
                fine.map_or("-".into(), |c| format!("{:+.1} %", 100.0 * (c - tc) / tc))
            )
            .unwrap();
        }
    }

    fs::write(out_dir.join("drag_summary.csv"), drag_csv).unwrap();
    fs::write(out_dir.join("tables.md"), &md).unwrap();
    let mut f = fs::File::create(out_dir.join("exp_cp_M1.98.csv")).unwrap();
    writeln!(f, "x_over_d,cp_experiment").unwrap();
    for (x, c) in pj::X_OVER_D.iter().zip(pj::CP.iter()) {
        writeln!(f, "{x},{c}").unwrap();
    }
    println!("\n{md}");
    println!("CSV/tables written to {}", out_dir.canonicalize().unwrap_or(out_dir).display());
}

fn report_run(name: &str, ni: usize, nj: usize, res: &OgiveResult, dt: f64) {
    println!(
        "  {name:6} {ni}x{nj}: {} iters ({dt:.1}s), stop: {}, residual drop {:.1} orders, CD_nose = {:.5} (first cell {:.5}), shock margin {:.3}, min rho {:.3} min p {:.3}",
        res.iters, res.stop_reason, res.drop_orders, res.cd_nose, res.cd_nose_cell, res.shock_margin, res.min_rho, res.min_p
    );
}

fn write_case_files(out_dir: &std::path::Path, mach: f64, grid: &str, res: &OgiveResult, th: &SurfaceTheory) {
    let tag = format!("M{mach}_{grid}");
    let mut f = fs::File::create(out_dir.join(format!("cp_{tag}.csv"))).unwrap();
    writeln!(f, "x_over_d,r_over_d,cp_wall_extrapolated,cp_first_cell,cp_shock_expansion,cp_tangent_cone").unwrap();
    for w in &res.wall {
        let tc = if w.x <= th.body.nose_length() { th.cp_tangent_cone(w.x) } else { f64::NAN };
        writeln!(f, "{:.8},{:.8},{:.8},{:.8},{:.8},{:.8}", w.x, w.r, w.cp, w.cp_cell, th.cp_shock_expansion(w.x), tc).unwrap();
    }
    let mut f = fs::File::create(out_dir.join(format!("residual_{tag}.csv"))).unwrap();
    writeln!(f, "iter,density_residual_l2").unwrap();
    for (i, r) in res.residual.iter().enumerate() {
        writeln!(f, "{},{:e}", i + 1, r).unwrap();
    }
}
