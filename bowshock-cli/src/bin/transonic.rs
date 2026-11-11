//! Transonic Mach sweep of a nose-cylinder body (default: cone-cylinder, fineness-ratio-3 nose) on the large-domain grid
//! with characteristic far-field BCs; forebody pressure drag vs Mach compared with the Stoney free-flight data
//! (NACA RM L53K17, digitised from Fig. 5) and Taylor-Maccoll / tangent-cone theory at the supersonic end.
//!
//! Usage: `cargo run --release --bin transonic -- [--body cone|karman|lvhaack|sqrt|parabola|ogive] [--level 0|1|2] [--quick]
//!   [--mach 0.8,0.9,...] [--warm] [--domain <body lengths>] [--max-iter N] [--drop <orders>] [--cd-tol <abs CD>]
//!   [--outflow pressure] [--limiter minmod|vanalbada] [--scheme explicit|implicit] [--lowmach on|off] [--tag name]`
//! `--scheme implicit` = line-implicit convergence acceleration (default limiter then van Albada; explicit SSP-RK3 + minmod stays the default).
//! The ogive body is also compared with the NASA TN D-3516 surface-pressure data (M = 0.7, 0.9, 1.0, 1.1, 1.2).
//!
//! `--quick` = grid level 0. `--warm` = start every Mach number from the previous converged field (ascending order).
//! Output: `out/transonic/` (`sweep_<tag>.csv`, `compare_<tag>.csv`, `cp_<tag>_M<m>.csv`, `residual_<tag>_M<m>.csv`, `tables_<tag>.md`).

use bowshock_core::flux::Limiter;
use bowshock_core::implicit::ImplicitParams;
use bowshock_core::solver::{Bc, Scheme};
use bowshock_cases::ogive::wall_cp_at;
use bowshock_cases::transonic::{named_body, run_on, stoney, tangent_cone_cd, TransonicCase, TransonicResult};
use std::fmt::Write as _;
use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

const DEFAULT_MACH: [f64; 12] = [0.7, 0.8, 0.85, 0.9, 0.95, 0.98, 1.0, 1.02, 1.05, 1.1, 1.2, 1.5];

fn arg<'a>(args: &'a [String], key: &str) -> Option<&'a str> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1)).map(String::as_str)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let quick = args.iter().any(|a| a == "--quick");
    let warm = args.iter().any(|a| a == "--warm");
    let level: u32 = arg(&args, "--level").and_then(|v| v.parse().ok()).unwrap_or(if quick { 0 } else { 1 });
    let body_name = arg(&args, "--body").unwrap_or("cone");
    let (body, data) = named_body(body_name).unwrap_or_else(|| panic!("unknown body '{body_name}' (cone|karman|lvhaack|sqrt|parabola|ogive)"));
    let machs: Vec<f64> = match arg(&args, "--mach") {
        Some(l) => l.split(',').map(|v| v.parse().expect("Mach list")).collect(),
        None => DEFAULT_MACH.to_vec(),
    };
    let implicit = arg(&args, "--scheme") == Some("implicit");
    let limiter = match arg(&args, "--limiter") {
        Some("vanalbada") => Limiter::VanAlbada,
        Some("minmod") => Limiter::Minmod,
        _ if implicit => Limiter::VanAlbada, // minmod stalls ~4.5 orders with the implicit scheme (README, "Subsonic")
        _ => Limiter::Minmod,
    };
    let low_mach = arg(&args, "--lowmach") == Some("on");
    let tag = arg(&args, "--tag").map(String::from).unwrap_or_else(|| format!("{body_name}_L{level}{}{}", if implicit { "_imp" } else { "" }, if warm { "_warm" } else { "" }));
    let out_dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("out").join("transonic");
    fs::create_dir_all(&out_dir).expect("create out dir");

    let make_case = |m: f64| {
        let cyl = 6.0;
        let mut c = TransonicCase::with_body(m, level, body, cyl);
        c.limiter = limiter;
        c.low_mach = low_mach;
        if implicit {
            c.scheme = Scheme::Implicit(ImplicitParams::DEFAULT);
            c.first_order_iters = 100;
        }
        if let Some(v) = arg(&args, "--max-iter").and_then(|v| v.parse().ok()) {
            c.max_iter = v;
        }
        if let Some(v) = arg(&args, "--drop").and_then(|v| v.parse().ok()) {
            c.drop_orders = v;
        }
        if let Some(v) = arg(&args, "--cd-tol").and_then(|v| v.parse().ok()) {
            c.cd_tol = v; // 0 disables the "CD stationary" stop
        }
        if arg(&args, "--outflow") == Some("pressure") {
            c.outflow = Bc::PressureOutflow;
        }
        if let Some(d) = arg(&args, "--domain").and_then(|v| v.parse::<f64>().ok()) {
            let lb = body.nose_length() + cyl;
            c.grid.x_up = d * lb;
            c.grid.r_far = d * lb;
        }
        c
    };

    let c0 = make_case(machs[0]);
    let (ni, nj) = c0.cells();
    let lb = body.nose_length() + c0.grid.cyl_length;
    println!(
        "bowshock-core transonic: body {body_name} (nose l/d {:.3}, tip half angle {:.2} deg), grid L{level} {ni}x{nj}, upstream {:.0} d / radial {:.0} d / downstream {:.0} d (body length {:.1} d), limiter {limiter:?}, scheme {}, low-Mach {low_mach}, outflow {:?}, {}",
        body.nose_length(),
        body.tip_half_angle().to_degrees(),
        c0.grid.x_up,
        c0.grid.r_far,
        c0.grid.x_down,
        lb,
        if implicit { "line-implicit" } else { "explicit SSP-RK3" },
        c0.outflow,
        if warm { "warm-start sweep" } else { "cold start for every Mach" }
    );
    println!("threads (rayon): {}", rayon::current_num_threads());

    let mut results: Vec<(TransonicResult, f64)> = Vec::new();
    let mut solver = c0.build_solver();
    let mut r_ref: Option<f64> = None;
    for (k, &m) in machs.iter().enumerate() {
        let case = make_case(m);
        if !warm || k == 0 {
            solver = case.build_solver();
        }
        let t0 = Instant::now();
        let res = run_on(&case, &mut solver, warm && k > 0, if warm { r_ref } else { None });
        let dt = t0.elapsed().as_secs_f64();
        if warm && k == 0 {
            r_ref = Some(res.residual_peak);
        }
        println!(
            "M = {m:<5} {:>6} iters ({dt:6.1} s) stop: {:<22} drop {:.1} orders (peak {:.2e} -> {:.2e}), CD_nose = {:.5} (first cell {:.5}, last-window range {:.1e}), Cp_min {:.3} @ x={:.2}, M_wall,max {:.2}, far-field |dp/p| {:.1e}, min rho {:.3} min p {:.3}",
            res.iters,
            res.stop_reason,
            res.drop_orders,
            res.residual_peak,
            res.residual.last().copied().unwrap_or(f64::NAN),
            res.cd_nose,
            res.cd_nose_cell,
            res.cd_range,
            res.cp_min,
            res.x_cp_min,
            res.m_wall_max,
            res.farfield_dp,
            res.min_rho,
            res.min_p
        );
        write_case_files(&out_dir, &tag, &res);
        results.push((res, dt));
    }

    // ---- summary tables ----
    let mut sweep = String::from("mach,iters,seconds,stop_reason,drop_orders,residual_peak,residual_final,cd_nose,cd_nose_first_cell,cd_range_last_window,cp_min,x_cp_min,m_wall_max,farfield_dp_over_p\n");
    let mut cmp = String::from("mach,cd_cfd,cd_cfd_first_cell,cd_stoney_digitised,stoney_read_uncertainty,stoney_status,cfd_minus_stoney,cd_tangent_cone_or_taylor_maccoll\n");
    let mut md = String::new();
    writeln!(md, "### {body_name}, grid L{level} {ni}x{nj}, {}\n", if warm { "warm-start sweep" } else { "cold starts" }).unwrap();
    writeln!(md, "| M | iters | time [s] | stop | residual drop [orders] | CD_nose CFD | Stoney (digitised) | CFD - Stoney | tangent-cone / T-M | Cp_min (x/d) | M_wall,max |").unwrap();
    writeln!(md, "|---|---|---|---|---|---|---|---|---|---|---|").unwrap();
    for (r, dt) in &results {
        writeln!(
            sweep,
            "{},{},{:.1},{},{:.2},{:e},{:e},{:.6},{:.6},{:.6},{:.5},{:.3},{:.4},{:e}",
            r.mach,
            r.iters,
            dt,
            r.stop_reason,
            r.drop_orders,
            r.residual_peak,
            r.residual.last().copied().unwrap_or(f64::NAN),
            r.cd_nose,
            r.cd_nose_cell,
            r.cd_range,
            r.cp_min,
            r.x_cp_min,
            r.m_wall_max,
            r.farfield_dp
        )
        .unwrap();
        let st = data.and_then(|d| stoney::interp(d, r.mach));
        let status = match st {
            Some(_) if r.mach <= stoney::MEASURED_UP_TO => "measured (digitised)",
            Some(_) => "faired beyond M=1.225 (not measurement)",
            None => "",
        };
        let tc = tangent_cone_cd(&body, r.mach);
        writeln!(
            cmp,
            "{},{:.5},{:.5},{},{},{},{},{}",
            r.mach,
            r.cd_nose,
            r.cd_nose_cell,
            st.map_or(String::new(), |v| format!("{v:.4}")),
            st.map_or(String::new(), |_| format!("{}", stoney::READ_UNCERTAINTY)),
            status,
            st.map_or(String::new(), |v| format!("{:+.4}", r.cd_nose - v)),
            if tc.is_nan() { String::new() } else { format!("{tc:.5}") }
        )
        .unwrap();
        writeln!(
            md,
            "| {} | {} | {:.0} | {} | {:.1} | {:.4} | {} | {} | {} | {:.3} ({:.2}) | {:.2} |",
            r.mach,
            r.iters,
            dt,
            r.stop_reason,
            r.drop_orders,
            r.cd_nose,
            st.map_or("-".into(), |v| format!("{v:.3} ± {}{}", stoney::READ_UNCERTAINTY, if r.mach > stoney::MEASURED_UP_TO { " (faired)" } else { "" })),
            st.map_or("-".into(), |v| format!("{:+.3}", r.cd_nose - v)),
            if tc.is_nan() { "-".into() } else { format!("{tc:.4}") },
            r.cp_min,
            r.x_cp_min,
            r.m_wall_max
        )
        .unwrap();
    }

    // ---- surface Cp vs Czarnecki & Monta (NASA TN D-3516) for the 3-caliber tangent ogive cylinder ----
    if body_name == "ogive" {
        use bowshock_cases::transonic::czarnecki_monta as cm;
        writeln!(md, "\n### Surface Cp vs NASA TN D-3516 Fig. 5 (digitised, ±{} Cp; ±0.02 on the nose x/d < 1.2)\n", cm::READ_UNCERTAINTY).unwrap();
        writeln!(md, "| M | stations used | RMS nose (x/d<=3) | max nose | RMS valid | max valid | shoulder peak Cp data | shoulder peak Cp CFD |").unwrap();
        writeln!(md, "|---|---|---|---|---|---|---|---|").unwrap();
        for (r, _) in &results {
            let Some(col) = cm::column(r.mach) else { continue };
            let mut f = fs::File::create(out_dir.join(format!("cp_compare_{tag}_M{}.csv", r.mach))).unwrap();
            writeln!(f, "x_in,x_over_d,cp_data_digitised,cp_cfd,cfd_minus_data,within_valid_region").unwrap();
            let xmax = cm::valid_x_max_in(r.mach);
            let (mut sn, mut mn, mut nn) = (0.0f64, 0.0f64, 0usize);
            let (mut sv, mut mv, mut nv) = (0.0f64, 0.0f64, 0usize);
            let mut peak_data = f64::INFINITY;
            for (k, &x_in) in cm::X_IN.iter().enumerate() {
                let xd = x_in / cm::D_IN;
                let cp_d = cm::CP[k][col];
                let cp_c = wall_cp_at(&r.wall, xd);
                let e = cp_c - cp_d;
                let valid = x_in <= xmax;
                writeln!(f, "{x_in},{xd:.4},{cp_d:.4},{cp_c:.5},{e:+.5},{valid}").unwrap();
                if x_in >= 8.0 && x_in <= 13.0 {
                    peak_data = peak_data.min(cp_d);
                }
                if xd <= body.nose_length() {
                    sn += e * e;
                    mn = mn.max(e.abs());
                    nn += 1;
                }
                if valid {
                    sv += e * e;
                    mv = mv.max(e.abs());
                    nv += 1;
                }
            }
            writeln!(
                md,
                "| {} | {} / {} | {:.4} | {:.4} | {:.4} | {:.4} | {:.3} | {:.3} |",
                r.mach,
                nv,
                cm::X_IN.len(),
                (sn / nn as f64).sqrt(),
                mn,
                (sv / nv as f64).sqrt(),
                mv,
                peak_data,
                r.cp_min
            )
            .unwrap();
        }
    }
    fs::write(out_dir.join(format!("sweep_{tag}.csv")), sweep).unwrap();
    fs::write(out_dir.join(format!("compare_{tag}.csv")), cmp).unwrap();
    fs::write(out_dir.join(format!("tables_{tag}.md")), &md).unwrap();
    println!("\n{md}");
    println!("CSV/tables written to {}", out_dir.canonicalize().unwrap_or(out_dir).display());
}

fn write_case_files(out_dir: &std::path::Path, tag: &str, res: &TransonicResult) {
    let name = format!("{tag}_M{}", res.mach);
    let mut f = fs::File::create(out_dir.join(format!("cp_{name}.csv"))).unwrap();
    writeln!(f, "x_over_d,r_over_d,cp_wall_extrapolated,cp_first_cell").unwrap();
    for w in &res.wall {
        writeln!(f, "{:.8},{:.8},{:.8},{:.8}", w.x, w.r, w.cp, w.cp_cell).unwrap();
    }
    let mut f = fs::File::create(out_dir.join(format!("residual_{name}.csv"))).unwrap();
    writeln!(f, "iter,density_residual_l2,cd_nose").unwrap();
    let cd: std::collections::HashMap<usize, f64> = res.cd_history.iter().copied().collect();
    for (i, r) in res.residual.iter().enumerate() {
        match cd.get(&(i + 1)) {
            Some(c) => writeln!(f, "{},{:e},{:.6}", i + 1, r, c).unwrap(),
            None => writeln!(f, "{},{:e},", i + 1, r).unwrap(),
        }
    }
}
