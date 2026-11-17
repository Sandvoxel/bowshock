//! Transonic Mach sweep of a nose-cylinder body (default: cone-cylinder, fineness-ratio-3 nose) on the sting grid with characteristic far-field BCs, run on the
//! single-cell wedge of the 3D engine; forebody pressure drag vs Mach compared with the Stoney free-flight data (NACA RM L53K17, digitised from Fig. 5) and
//! Taylor-Maccoll / tangent-cone theory at the supersonic end.
//!
//! Usage: `cargo run --release --bin transonic -- [--body cone|karman|lvhaack|sqrt|parabola|ogive] [--level 0|1|2] [--quick]
//!   [--mach 0.8,0.9,...] [--warm] [--domain <body lengths>] [--max-iter N] [--limiter minmod|vanalbada] [--lowmach on|off] [--tag name]`
//! The numerics are the 3D engine's Euler numerics of the Mach regime (`Numerics3::euler_wedge`: line-implicit, van Albada with the low-Mach correction below Mach 0.85,
//! minmod above). The ogive body is also compared with the NASA TN D-3516 surface-pressure data (M = 0.7, 0.9, 1.0, 1.1, 1.2).
//!
//! `--quick` = grid level 0. `--warm` = start every Mach number from the previous converged field (ascending order).
//! Output: `out/transonic/` (`sweep_<tag>.csv`, `compare_<tag>.csv`, `cp_<tag>_M<m>.csv`, `tables_<tag>.md`).

use bowshock_cases::transonic::{named_body, stoney, tangent_cone_cd, transonic_wedge_case};
use bowshock_cases::wedge::{run_wedge_point, WedgeResult};
use bowshock_cfd::limiter::Limiter;
use bowshock_core::engine::result::wall_cp_at;
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
    let limiter = match arg(&args, "--limiter") {
        Some("vanalbada") => Some(Limiter::VanAlbada),
        Some("minmod") => Some(Limiter::Minmod),
        _ => None,
    };
    let low_mach = arg(&args, "--lowmach").map(|v| v == "on");
    let max_iter: Option<usize> = arg(&args, "--max-iter").and_then(|v| v.parse().ok());
    let tag = arg(&args, "--tag").map(String::from).unwrap_or_else(|| format!("{body_name}_L{level}{}", if warm { "_warm" } else { "" }));
    let out_dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").join("out").join("transonic");
    fs::create_dir_all(&out_dir).expect("create out dir");

    let make_case = |m: f64| {
        let mut c = transonic_wedge_case(m, level, body, 6.0);
        if let Some(l) = limiter {
            c.numerics.limiter = l;
        }
        if let Some(lm) = low_mach {
            c.numerics.low_mach = lm;
        }
        if let Some(n) = max_iter {
            c.numerics.max_iter = n;
        }
        if let Some(d) = arg(&args, "--domain").and_then(|v| v.parse::<f64>().ok()) {
            let lb = body.nose_length() + 6.0;
            c.grid.sting.x_up = d * lb;
            c.grid.sting.r_far = d * lb;
        }
        c
    };

    let c0 = make_case(machs[0]);
    let g0 = c0.build_solver().expect("wedge grid").grid.clone();
    println!(
        "bowshock-cfd transonic on the 3D wedge: body {body_name} (nose l/d {:.3}, tip half angle {:.2} deg), grid L{level} {}x{}, body length {:.1} d, {}",
        body.nose_length(),
        body.tip_half_angle().to_degrees(),
        g0.ni,
        g0.nj,
        c0.body.x_base(),
        if warm { "warm-start sweep" } else { "cold start for every Mach" }
    );
    println!("threads (rayon): {}", rayon::current_num_threads());

    let mut results: Vec<(WedgeResult, f64)> = Vec::new();
    let mut solver = None;
    for (k, &m) in machs.iter().enumerate() {
        let case = make_case(m);
        let t0 = Instant::now();
        if !warm || k == 0 {
            solver = Some(case.build_solver().expect("wedge grid"));
        }
        let res = run_wedge_point(&case, solver.as_mut().expect("solver"), warm && k > 0);
        let dt = t0.elapsed().as_secs_f64();
        println!(
            "M = {m:<5} {:>6} iters ({dt:6.1} s) stop: {:<22} drop {:.1} orders, CD_nose = {:.5}, Cp_min {:.3} @ x={:.2}, M_max {:.2}, far-field |dp/p| {:.1e}, min rho {:.3} min p {:.3}",
            res.iters, res.stop_reason, res.drop_orders, res.cd_nose, res.cp_min, res.x_cp_min, res.m_max, res.farfield_dp, res.min_rho, res.min_p
        );
        let mut f = fs::File::create(out_dir.join(format!("cp_{tag}_M{m}.csv"))).unwrap();
        writeln!(f, "x_over_d,r_over_d,cp_wall_extrapolated").unwrap();
        for w in &res.wall {
            writeln!(f, "{:.8},{:.8},{:.8}", w.x, w.r, w.cp).unwrap();
        }
        results.push((res, dt));
    }

    // ---- summary tables ----
    let mut sweep = String::from("mach,iters,seconds,stop_reason,drop_orders,cd_nose,cp_min,x_cp_min,m_max,farfield_dp_over_p\n");
    let mut cmp = String::from("mach,cd_cfd,cd_stoney_digitised,stoney_read_uncertainty,stoney_status,cfd_minus_stoney,cd_tangent_cone_or_taylor_maccoll\n");
    let mut md = String::new();
    writeln!(md, "### {body_name}, grid L{level} {}x{}, {}\n", g0.ni, g0.nj, if warm { "warm-start sweep" } else { "cold starts" }).unwrap();
    writeln!(md, "| M | iters | time [s] | stop | residual drop [orders] | CD_nose CFD | Stoney (digitised) | CFD - Stoney | tangent-cone / T-M | Cp_min (x/d) | M_max |").unwrap();
    writeln!(md, "|---|---|---|---|---|---|---|---|---|---|---|").unwrap();
    for (r, dt) in &results {
        writeln!(
            sweep,
            "{},{},{:.1},{},{:.2},{:.6},{:.5},{:.3},{:.4},{:e}",
            r.mach, r.iters, dt, r.stop_reason, r.drop_orders, r.cd_nose, r.cp_min, r.x_cp_min, r.m_max, r.farfield_dp
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
            "{},{:.5},{},{},{},{},{}",
            r.mach,
            r.cd_nose,
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
            r.m_max
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
                let cp_c = wall_cp_at(&r.wall, xd).unwrap_or(f64::NAN);
                let e = cp_c - cp_d;
                let valid = x_in <= xmax;
                writeln!(f, "{x_in},{xd:.4},{cp_d:.4},{cp_c:.5},{e:+.5},{valid}").unwrap();
                if (8.0..=13.0).contains(&x_in) {
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
