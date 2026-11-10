//! Subsonic Mach sweep (M = 0.1 ... 0.8) of a nose-cylinder body with and without a flat blunt base, plus validation against
//! published data (NASA TM X-2439) and incompressible / Goethert-rule axisymmetric potential-flow theory.
//!
//! Usage: `cargo run --release --bin subsonic -- [--body ogive|cone|karman|lvhaack|sqrt|parabola] [--level 0|1|2] [--quick]
//!   [--mach 0.1,0.2,...] [--base on|off|both] [--warm] [--lowmach on|off] [--limiter vanalbada|minmod] [--scheme implicit|explicit]
//!   [--max-iter N] [--drop <orders>] [--cyl <diameters>] [--domain <body lengths>] [--tag name]`
//!
//! Modes: default = Mach sweep -> `out/subsonic/sweep_<tag>.csv`, `tables_<tag>.md`, `cp_<tag>_M<m>_<base|nobase>.csv`,
//! `residual_<tag>_M<m>_<base|nobase>.csv`; `--validate` = TM X-2439 configurations 4 (ogive) and 2 (cone) at M = 0.404, 0.608, 0.831
//! -> `fox_*.csv`, `tables_validate.md`; `--lowmach-study` = low-Mach correction on/off at M = 0.1 ... 0.5 against potential theory
//! -> `lowmach_<tag>.csv`, `tables_lowmach.md`.

use bowshock_cases::body_grid::{Body, Nose};
use bowshock_core::implicit::ImplicitParams;
use bowshock_core::flux::Limiter;
use bowshock_reference::potential::{nose_cylinder_body, solve, solve_compressible, PotentialSolution};
use bowshock_core::solver::Scheme;
use bowshock_cases::subsonic::{fox_case, fox_tm_x2439 as fox, run_on, run_subsonic, wall_cp_at, SubsonicCase, SubsonicResult};
use bowshock_cases::transonic::named_body;
use std::fmt::Write as _;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

fn arg<'a>(args: &'a [String], key: &str) -> Option<&'a str> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1)).map(String::as_str)
}

struct Opts {
    level: u32,
    limiter: Limiter,
    low_mach: bool,
    scheme: Scheme,
    max_iter: Option<usize>,
    drop: Option<f64>,
    domain: Option<f64>,
}

impl Opts {
    fn parse(args: &[String]) -> Opts {
        let quick = args.iter().any(|a| a == "--quick");
        let explicit = arg(args, "--scheme") == Some("explicit");
        Opts {
            level: arg(args, "--level").and_then(|v| v.parse().ok()).unwrap_or(if quick { 0 } else { 1 }),
            limiter: match arg(args, "--limiter") {
                Some("minmod") => Limiter::Minmod,
                Some("vanalbada") => Limiter::VanAlbada,
                _ if explicit => Limiter::Minmod,
                _ => Limiter::VanAlbada,
            },
            low_mach: arg(args, "--lowmach") != Some("off"),
            scheme: if explicit { Scheme::Explicit } else { Scheme::Implicit(ImplicitParams::DEFAULT) },
            max_iter: arg(args, "--max-iter").and_then(|v| v.parse().ok()),
            drop: arg(args, "--drop").and_then(|v| v.parse().ok()),
            domain: arg(args, "--domain").and_then(|v| v.parse().ok()),
        }
    }
    fn apply(&self, mut c: SubsonicCase) -> SubsonicCase {
        c.limiter = self.limiter;
        c.low_mach = self.low_mach;
        c.scheme = self.scheme;
        if let Some(n) = self.max_iter {
            c.max_iter = n;
        } else if self.scheme == Scheme::Explicit {
            c.max_iter = 60_000;
            c.first_order_iters = 300;
        }
        if let Some(d) = self.drop {
            c.drop_orders = d;
        }
        if let Some(d) = self.domain {
            c = c.with_domain(d, 0.5 * d);
        }
        c
    }
}

/// Goethert-rule / incompressible potential solutions of the closed body (nose + 20 d cylinder + mirrored tail).
struct Potential {
    inc: PotentialSolution,
    comp: PotentialSolution,
}

fn potential(body: &Body, mach: f64) -> Option<Potential> {
    if matches!(body.nose, Nose::Cone { .. } | Nose::Power { .. }) {
        return None; // corner / blunt-tip noses: the axial-source representation is not accurate there
    }
    let m = nose_cylinder_body(&|x| body.r(x), body.nose_length(), 20.0, body.radius(), 300, 250);
    Some(Potential { inc: solve(&m, 400, 0.0005), comp: solve_compressible(&m, mach, 400, 0.0005) })
}

/// RMS of `cp - reference` over the wall faces `0.3 < x < 9` (nose + cylinder) for the incompressible and the Goethert reference.
fn rms_vs_potential(r: &SubsonicResult, p: &Potential) -> (f64, f64) {
    let (mut a, mut b, mut n) = (0.0, 0.0, 0usize);
    for w in &r.wall {
        if w.x > 0.3 && w.x < 9.0 {
            if let (Some(i), Some(c)) = (p.inc.cp_at(w.x), p.comp.cp_at(w.x)) {
                a += (w.cp - i).powi(2);
                b += (w.cp - c).powi(2);
                n += 1;
            }
        }
    }
    ((a / n.max(1) as f64).sqrt(), (b / n.max(1) as f64).sqrt())
}

fn write_case_files(dir: &Path, name: &str, r: &SubsonicResult, pot: Option<&Potential>) {
    let mut f = fs::File::create(dir.join(format!("cp_{name}.csv"))).unwrap();
    writeln!(f, "x_over_d,r_over_d,cp_wall_extrapolated,cp_first_cell,cp_goethert_potential,cp_incompressible_potential").unwrap();
    for w in &r.wall {
        let (g, i) = pot.map_or((f64::NAN, f64::NAN), |p| (p.comp.cp_at(w.x).unwrap_or(f64::NAN), p.inc.cp_at(w.x).unwrap_or(f64::NAN)));
        writeln!(f, "{:.8},{:.8},{:.8},{:.8},{:.6},{:.6}", w.x, w.r, w.cp, w.cp_cell, g, i).unwrap();
    }
    if !r.base_wall.is_empty() {
        let mut f = fs::File::create(dir.join(format!("base_{name}.csv"))).unwrap();
        writeln!(f, "r0,r1,cp_wall_extrapolated,cp_first_cell").unwrap();
        for b in &r.base_wall {
            writeln!(f, "{:.6},{:.6},{:.6},{:.6}", b.r0, b.r1, b.cp, b.cp_cell).unwrap();
        }
    }
    let mut f = fs::File::create(dir.join(format!("residual_{name}.csv"))).unwrap();
    writeln!(f, "iter,density_residual_l2").unwrap();
    for (i, x) in r.residual.iter().enumerate() {
        writeln!(f, "{},{:e}", i + 1, x).unwrap();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let out_dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("out").join("subsonic");
    fs::create_dir_all(&out_dir).expect("create out dir");
    let opts = Opts::parse(&args);
    println!("bowshock-cfd subsonic: grid level {}, limiter {:?}, low-Mach {}, scheme {}", opts.level, opts.limiter, opts.low_mach, if matches!(opts.scheme, Scheme::Explicit) { "explicit SSP-RK3" } else { "line-implicit" });
    println!("threads (rayon): {}", rayon::current_num_threads());
    if args.iter().any(|a| a == "--validate") {
        validate(&out_dir, &opts);
    } else if args.iter().any(|a| a == "--lowmach-study") {
        lowmach_study(&out_dir, &opts, &args);
    } else {
        sweep(&out_dir, &opts, &args);
    }
}

fn sweep(out_dir: &Path, opts: &Opts, args: &[String]) {
    let body_name = arg(args, "--body").unwrap_or("ogive");
    let (body, _) = named_body(body_name).unwrap_or_else(|| panic!("unknown body '{body_name}'"));
    let machs: Vec<f64> = match arg(args, "--mach") {
        Some(l) => l.split(',').map(|v| v.parse().expect("Mach list")).collect(),
        None => vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
    };
    let bases: Vec<bool> = match arg(args, "--base") {
        Some("on") => vec![true],
        Some("off") => vec![false],
        _ => vec![false, true],
    };
    let warm = args.iter().any(|a| a == "--warm");
    let cyl: f64 = arg(args, "--cyl").and_then(|v| v.parse().ok()).unwrap_or(6.0);
    let tag = arg(args, "--tag").map(String::from).unwrap_or_else(|| format!("{body_name}_L{}{}", opts.level, if warm { "_warm" } else { "" }));
    let make = |m: f64, base: bool| {
        let mut bb = SubsonicCase::new(m, body.nose, opts.level, base);
        bb.body.cyl_length = cyl;
        bb.grid_base.body.cyl_length = cyl;
        bb.grid_sting.cyl_length = cyl;
        opts.apply(bb)
    };
    let c0 = make(machs[0], true);
    println!("body {body_name} (nose l/d {:.2}), cylinder {cyl} d, {} sweep", body.nose_length(), if warm { "warm-start" } else { "cold-start" });
    let mut sweep_csv = String::from("mach,base,iters,seconds,stop_reason,drop_orders,cd_nose,cd_cylinder,cd_boattail,cd_base,cd_total,cp_base_mean,cp_base_range,cp_base_last,cp_min,x_cp_min,m_max,rms_vs_incompressible,rms_vs_goethert,farfield_dp_over_p\n");
    let mut md = String::new();
    writeln!(md, "### {body_name}, grid level {}, {} cells (no base) / {} cells (base), {}\n", opts.level, { let (a, b) = make(machs[0], false).cells(); a * b }, { let (a, b) = c0.cells(); a * b }, if warm { "warm starts" } else { "cold starts" }).unwrap();
    writeln!(md, "| M | base | iters | time [s] | residual drop | CD nose | CD base | CD total | CD base corr. | CD total corr. | Cp_base (mean, range) | Cp_min | RMS vs Goethert | RMS vs incompressible |").unwrap();
    writeln!(md, "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|").unwrap();
    for &base in &bases {
        let mut solver = None;
        let mut r_ref = None;
        for (k, &m) in machs.iter().enumerate() {
            let case = make(m, base);
            let t0 = Instant::now();
            let res = if warm && k > 0 {
                run_on(&case, solver.as_mut().unwrap(), true, r_ref)
            } else {
                let mut s = case.build_solver();
                let r = run_on(&case, &mut s, false, None);
                if warm {
                    r_ref = Some(r.residual_peak);
                    solver = Some(s);
                }
                r
            };
            let dt = t0.elapsed().as_secs_f64();
            let pot = potential(&body, m);
            let (ri, rg) = pot.as_ref().map_or((f64::NAN, f64::NAN), |p| rms_vs_potential(&res, p));
            let name = format!("{tag}_M{m}_{}", if base { "base" } else { "nobase" });
            write_case_files(out_dir, &name, &res, pot.as_ref());
            println!(
                "M={m:<4} base={:<5} {:>6} iters ({dt:6.1} s) {:<22} drop {:.1}: CD nose {:+.4} cyl {:+.4} boat {:+.4} base {:+.4} total {:+.4}; Cp_b {:+.4} (window range {:.1e}); Cp_min {:+.3} @ {:.2}; M_max {:.2}; vs Goethert {:.4} (incomp. {:.4}); far-field |dp/p| {:.1e}",
                base, res.iters, res.stop_reason, res.drop_orders, res.cd_nose, res.cd_cyl, res.cd_boat, res.cd_base, res.cd_total, res.cp_base_mean, res.cp_base_range, res.cp_min, res.x_cp_min, res.m_max, rg, ri, res.farfield_dp
            );
            writeln!(sweep_csv, "{m},{base},{},{dt:.1},{},{:.2},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.5},{:.3},{:.3},{:.5},{:.5},{:e}", res.iters, res.stop_reason, res.drop_orders, res.cd_nose, res.cd_cyl, res.cd_boat, res.cd_base, res.cd_total, res.cp_base_mean, res.cp_base_range, res.cp_base, res.cp_min, res.x_cp_min, res.m_max, ri, rg, res.farfield_dp).unwrap();
            writeln!(md, "| {m} | {} | {} | {dt:.0} | {:.1} | {:+.4} | {:+.4} | {:+.4} | {:+.4} | {:+.4} | {} | {:.3} | {:.4} | {:.4} |", if base { "on" } else { "off" }, res.iters, res.drop_orders, res.cd_nose, res.cd_base, res.cd_total, res.cd_base_model, res.cd_total_model, if base { format!("{:+.3} ({:.0e})", res.cp_base_mean, res.cp_base_range) } else { "-".into() }, res.cp_min, rg, ri).unwrap();
        }
    }
    fs::write(out_dir.join(format!("sweep_{tag}.csv")), sweep_csv).unwrap();
    fs::write(out_dir.join(format!("tables_{tag}.md")), &md).unwrap();
    println!("\n{md}\nCSV/tables written to {}", out_dir.display());
}

fn validate(out_dir: &Path, opts: &Opts) {
    let mut md = String::new();
    writeln!(md, "### NASA TM X-2439 (Fox 1971), sting-mounted nose-cylinder-boattail bodies, grid level {}\n", opts.level).unwrap();
    writeln!(md, "| config | M | iters | time [s] | drop | RMS fore+cyl | max fore+cyl | RMS boattail | CFD base Cp | data base Cp | CD fore | CD boat | CD base | CD total | gage unc. |").unwrap();
    writeln!(md, "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|").unwrap();
    for cone in [false, true] {
        for (k, &m) in fox::MACH.iter().enumerate() {
            let case = opts.apply(fox_case(cone, m, opts.level));
            let t0 = Instant::now();
            let r = run_subsonic(&case);
            let dt = t0.elapsed().as_secs_f64();
            let (xs, cps): (&[f64], &[f64]) = match (cone, k) {
                (false, 0) => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M04_MEAN),
                (false, 1) => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M06_MEAN),
                (false, _) => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M08_MEAN),
                (true, 0) => (&fox::CONE_CYL_X_D, &fox::CONE_CYL_CP_M04_MEAN),
                (true, 1) => (&fox::CONE_CYL_X_D, &fox::CONE_CYL_CP_M06_MEAN),
                (true, _) => (&fox::CONE_CYL_X_D, &fox::CONE_CYL_CP_M08_MEAN),
            };
            let cfg = if cone { "2 (cone)" } else { "4 (ogive)" };
            let x_fore = if cone { 4.2 } else { 3.4 }; // boattail shoulder influence starts upstream of the shoulder
            let name = format!("fox_{}_M{m}", if cone { "cone" } else { "ogive" });
            let mut f = fs::File::create(out_dir.join(format!("{name}.csv"))).unwrap();
            writeln!(f, "x_over_d,cp_data_mean,cp_cfd,cfd_minus_data,region").unwrap();
            let (mut s1, mut m1, mut n1, mut s2, mut n2) = (0.0f64, 0.0f64, 0usize, 0.0f64, 0usize);
            for (x, d) in xs.iter().zip(cps.iter()) {
                if let Some(c) = wall_cp_at(&r.wall, *x) {
                    let e = c - d;
                    let fore = *x <= x_fore;
                    // the boattail-shoulder suction peak is a corner singularity: excluded from the "fore+cyl" statistics (x <= x_fore)
                    writeln!(f, "{x},{d:.4},{c:.5},{e:+.5},{}", if fore { "fore+cylinder" } else { "boattail" }).unwrap();
                    if fore {
                        s1 += e * e;
                        m1 = m1.max(e.abs());
                        n1 += 1;
                    } else {
                        s2 += e * e;
                        n2 += 1;
                    }
                }
            }
            let data_base = if cone { fox::BASE_CP_CONE[k] } else { fox::BASE_CP_OGIVE[k] };
            println!(
                "config {cfg} M={m}: {} iters ({dt:.1} s) {} drop {:.1}; Cp RMS (x <= {x_fore}) {:.4} max {:.4} ({} stations), boattail RMS {:.4}; base Cp CFD {:+.4} vs data {:+.4}; CD fore {:+.4} boat {:+.4} base {:+.4} total {:+.4}; gage unc. {:.4}",
                r.iters, r.stop_reason, r.drop_orders, (s1 / n1 as f64).sqrt(), m1, n1, (s2 / n2.max(1) as f64).sqrt(), r.cp_base_mean, data_base, r.cd_nose, r.cd_boat, r.cd_base, r.cd_total, fox::CP_UNCERTAINTY[k]
            );
            writeln!(md, "| {cfg} | {m} | {} | {dt:.0} | {:.1} | {:.4} | {:.4} | {:.4} | {:+.4} | {:+.4} | {:+.4} | {:+.4} | {:+.4} | {:+.4} | {:.4} |", r.iters, r.drop_orders, (s1 / n1 as f64).sqrt(), m1, (s2 / n2.max(1) as f64).sqrt(), r.cp_base_mean, data_base, r.cd_nose + r.cd_cyl, r.cd_boat, r.cd_base, r.cd_total, fox::CP_UNCERTAINTY[k]).unwrap();
            write_case_files(out_dir, &name, &r, None);
        }
    }
    fs::write(out_dir.join("tables_validate.md"), &md).unwrap();
    println!("\n{md}");
}

fn lowmach_study(out_dir: &Path, opts: &Opts, args: &[String]) {
    let machs: Vec<f64> = match arg(args, "--mach") {
        Some(l) => l.split(',').map(|v| v.parse().expect("Mach list")).collect(),
        None => vec![0.1, 0.2, 0.3, 0.5],
    };
    let (body, _) = named_body("ogive").unwrap();
    let stations = [0.5, 1.5, 2.5, 3.5, 6.0];
    let mut csv = String::from("mach,low_mach,iters,seconds,drop_orders,rms_vs_incompressible,rms_vs_goethert,cp_0.5,cp_1.5,cp_2.5,cp_3.5,cp_6.0,cp_min\n");
    let mut md = String::new();
    writeln!(md, "### Low-Mach correction, tangent ogive (L/d = 3) + cylinder, no base, grid level {}\n", opts.level).unwrap();
    writeln!(md, "| M | low-Mach | iters | time [s] | RMS Cp vs incompressible | RMS Cp vs Goethert(M) | Cp(0.5) | Cp(1.5) | Cp(2.5) | Cp(3.5) | Cp(6) |").unwrap();
    writeln!(md, "|---|---|---|---|---|---|---|---|---|---|---|").unwrap();
    let p0 = potential(&body, 0.0).unwrap();
    writeln!(md, "| 0 | potential (incompressible) | - | - | 0 | - | {} |", stations.iter().map(|&x| format!("{:.4}", p0.inc.cp_at(x).unwrap())).collect::<Vec<_>>().join(" | ")).unwrap();
    for &m in &machs {
        let pot = potential(&body, m).unwrap();
        for lm in [false, true] {
            let mut case = opts.apply(SubsonicCase::new(m, body.nose, opts.level, false));
            case.low_mach = lm;
            let t0 = Instant::now();
            let r = run_subsonic(&case);
            let dt = t0.elapsed().as_secs_f64();
            let (ri, rg) = rms_vs_potential(&r, &pot);
            let cps: Vec<f64> = stations.iter().map(|&x| wall_cp_at(&r.wall, x).unwrap()).collect();
            println!("M={m} low-Mach {lm}: {} iters ({dt:.1} s) drop {:.1}; RMS vs incompressible {ri:.4}, vs Goethert {rg:.4}; Cp@stations {:?}", r.iters, r.drop_orders, cps.iter().map(|c| format!("{c:.4}")).collect::<Vec<_>>());
            writeln!(csv, "{m},{lm},{},{dt:.1},{:.2},{ri:.5},{rg:.5},{},{:.5}", r.iters, r.drop_orders, cps.iter().map(|c| format!("{c:.5}")).collect::<Vec<_>>().join(","), r.cp_min).unwrap();
            writeln!(md, "| {m} | {} | {} | {dt:.0} | {ri:.4} | {rg:.4} | {} |", if lm { "on" } else { "off" }, r.iters, cps.iter().map(|c| format!("{c:.4}")).collect::<Vec<_>>().join(" | ")).unwrap();
        }
        writeln!(md, "| {m} | Goethert potential | - | - | - | 0 | {} |", stations.iter().map(|&x| format!("{:.4}", pot.comp.cp_at(x).unwrap())).collect::<Vec<_>>().join(" | ")).unwrap();
    }
    fs::write(out_dir.join(format!("lowmach_ogive_L{}.csv", opts.level)), csv).unwrap();
    fs::write(out_dir.join("tables_lowmach.md"), &md).unwrap();
    println!("\n{md}");
}
