//! Boattail pressure drag diagnosis (DESIGN3D item 64): the Fox TM X-2439 boattail (configuration 4: tangent ogive 2 d, cylinder 2 d, 10 degree boattail of 1 d, sting 0.13 d) on the Euler wedge,
//! the wall `Cp` along the boattail against the tabulated data, the total-pressure and entropy loss of the first cell row along the wall, and the variants of the numerics and the grid that could
//! explain the over-predicted drag.
//!
//! `BT_MACHS` (comma list, default `0.404,0.608,0.831`), `BT_VARIANTS` (comma list, default `base`), tokens joined by `+`:
//! - `lvl<N>`: grid level (default 0); `ref<NN>`: `with_refinement(NN / 10)`;
//! - `fo`: first order throughout; `nolm`: the low-Mach flux correction off; `lm`: on; `pc`: low-Mach preconditioning of the implicit left-hand side on; `minmod`, `venk`: limiter;
//! - `sens<lo>_<hi>`: CPU shock-sensor thresholds (relative pressure jump of the Rusanov blend; `sens0_0` is not allowed: `sensoff` = thresholds 1e9);
//! - `sting<R>`: the sting radius (default 0.13); `cflmax<N>`, `cfl0_<x>`, `sub<N>`, `nobc`, `nolines`: implicit parameters;
//! - `closed`: the boattail continues as a sting of the base radius to the outflow (`Wake::Sting`, no base, no wake: the pressure drag of nose and boattail must vanish, d'Alembert);
//! - `round`: the shoulder rounded by four short cones (2.5, 5, 7.5 degrees over 0.2 d) before the 10 degrees; `nosting`: full-disc base; `cyl<L>`: cylinder length (default 2);
//! - `iters<N>`: cap of the iterations.
//!
//! Prints `BT <variant> M<mach> ...` lines. CPU unless `BOWSHOCK_WEDGE_BACKEND=gpu`.
#[path = "fin_gates/common.rs"]
mod common;

use bowshock_cases::subsonic::fox_tm_x2439 as fox;
use bowshock_cfd::gas3::{cons_to_prim3, GAMMA};
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose, Segment};
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::run3::{wall_samples, Wall};
use common::{Session, Solved};

fn host(s: &Session) -> &bowshock_cfd::solver3::Solver3 {
    match s {
        Session::Cpu(sol) => sol,
        #[cfg(feature = "gpu")]
        Session::Gpu(r) => &r.host,
    }
}

fn body_of(parts: &[&str]) -> BaseBody {
    let nose = Nose::TangentOgive { fineness: 2.0 };
    let cyl = parts.iter().find_map(|p| p.strip_prefix("cyl")).map_or(2.0, |v| v.parse::<f64>().expect("cylN"));
    let mut b = BaseBody::cylinder(Body { nose }, cyl);
    if parts.contains(&"round") {
        // four cones of 0.05 d (x) at 2.5, 5, 7.5 and 10 degrees, then the 10 degree boattail for the rest, ending at the radius of the sharp one (the last cone is longer by the radius difference)
        let sharp_drop = 10f64.to_radians().tan();
        let fil = [2.5f64, 5.0, 7.5];
        let lf = 0.05;
        let mut drop = 0.0;
        for a in fil {
            b.segments.push(Segment { length: lf, slope: -a.to_radians().tan() });
            drop += lf * a.to_radians().tan();
        }
        let full = 1.0 * sharp_drop;
        // the rest at 10 degrees covers `full - drop` of radius, i.e. a length (full - drop) / tan 10
        b.segments.push(Segment { length: (full - drop) / sharp_drop, slope: -sharp_drop });
        // total length 3 * 0.05 + 1 - drop / tan10 (slightly less than 1): fine, the base moves up by the same amount
    } else {
        b = b.with_boattail(1.0, 10f64.to_radians());
    }
    if !parts.contains(&"nosting") && !parts.contains(&"closed") {
        let r = parts.iter().find_map(|p| p.strip_prefix("sting")).map_or(fox::STING_RADIUS, |v| v.parse::<f64>().expect("stingR"));
        b = b.with_sting(r);
    }
    b
}

fn case_of(m: f64, variant: &str) -> Case3 {
    let parts: Vec<&str> = variant.split('+').collect();
    let level = parts.iter().find_map(|p| p.strip_prefix("lvl")).map_or(0, |v| v.parse::<u32>().expect("lvlN"));
    let mut c = if parts.contains(&"closed") { bowshock_cases::wedge::sting_case(m, body_of(&parts), level) } else { Case3::wedge(m, body_of(&parts), level) };
    for p in &parts {
        if let Some(v) = p.strip_prefix("ref") {
            c = c.with_refinement(v.parse::<f64>().expect("refNN") / 10.0);
        }
        if let Some(v) = p.strip_prefix("iters") {
            c.numerics.max_iter = v.parse().expect("itersN");
        }
        if let Some(v) = p.strip_prefix("sens") {
            let (lo, hi) = if v == "off" { (1e9, 2e9) } else { v.split_once('_').map(|(a, b)| (a.parse().unwrap(), b.parse().unwrap())).expect("sens<lo>_<hi>") };
            bowshock_cfd::limiter::set_diag_sensor(lo, hi);
        }
    }
    if parts.contains(&"fo") {
        c.numerics.limiter = bowshock_cfd::limiter::Limiter::FirstOrder;
    }
    if parts.contains(&"minmod") {
        c.numerics.limiter = bowshock_cfd::limiter::Limiter::Minmod;
    }
    if parts.contains(&"venk") {
        c.numerics.limiter = bowshock_cfd::limiter::Limiter::Venkatakrishnan;
    }
    if parts.contains(&"nolm") {
        c.numerics.low_mach = false;
    }
    if parts.contains(&"lm") {
        c.numerics.low_mach = true;
    }
    for p in &parts {
        if let Some(v) = p.strip_prefix("cflmax") {
            c.numerics.implicit.cfl_max = v.parse().expect("cflmaxN");
        }
        if let Some(v) = p.strip_prefix("cfl0_") {
            c.numerics.implicit.cfl0 = v.parse().expect("cfl0_X");
        }
        if let Some(v) = p.strip_prefix("sub") {
            c.numerics.implicit.sub_iters = v.parse().expect("subN");
        }
    }
    if parts.contains(&"nobc") {
        c.numerics.implicit.implicit_bc = false;
    }
    if parts.contains(&"nolines") {
        c.numerics.implicit.lines = false;
    }
    if parts.contains(&"pc") {
        c.numerics.implicit.low_mach_precond = true;
    }
    c
}

/// Data `Cp` at `x` (linear between orifices).
fn data_cp(k: usize, x: f64) -> Option<f64> {
    let (xs, cps) = match k {
        0 => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M04_MEAN),
        1 => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M06_MEAN),
        _ => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M08_MEAN),
    };
    (0..xs.len() - 1).find(|&i| x >= xs[i] && x <= xs[i + 1]).map(|i| cps[i] + (cps[i + 1] - cps[i]) * (x - xs[i]) / (xs[i + 1] - xs[i]))
}

#[test]
#[ignore = "diagnosis: Fox boattail on the Euler wedge"]
fn bt1_fox_boattail() {
    let machs: Vec<f64> = std::env::var("BT_MACHS").map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or_else(|_| fox::MACH.to_vec());
    let variants: Vec<String> = std::env::var("BT_VARIANTS").unwrap_or_else(|_| "base".into()).split(',').map(|s| s.trim().to_string()).collect();
    for variant in &variants {
        for &m in &machs {
            let k = fox::MACH.iter().position(|x| (x - m).abs() < 1e-9).expect("a Fox Mach number");
            let case = case_of(m, variant);
            let tag = format!("BT {variant} M{m:.3}");
            let t0 = std::time::Instant::now();
            let mut s = Session::new(&case).expect("session");
            let sol: Solved = s.run(&case, false).expect("run");
            let p = &sol.parts;
            let h = host(&s);
            let g = &h.grid;
            let x_bt = case.body.station(case.body.segments.len() - sol_nseg(&case)).0;
            let aft_sum: f64 = p.aft.iter().sum();
            println!(
                "{tag} grid {}x{} iters {} stop {:?} drop {:.2} t {:.0}s | cd_nose {:+.4} cd_aft_sum {:+.4} (segments {:?}) cd_base {:+.4} cp_base {:+.4}",
                g.ni,
                g.nj,
                sol.point.iterations,
                sol.point.stop_reason,
                sol.point.drop_orders,
                t0.elapsed().as_secs_f64(),
                p.nose,
                aft_sum,
                p.aft.iter().map(|a| (a * 1e4).round() / 1e4).collect::<Vec<_>>(),
                p.base,
                sol.point.cp_base
            );
            let p_inf = h.cfg.freestream[4];
            let q = 0.5 * m * m;
            let mut cps: Vec<(f64, f64, f64)> = wall_samples(g, &h.u)
                .into_iter()
                .filter(|w| matches!(w.class, Wall::Body(_)))
                .map(|w| (w.face.c[0], (w.pw - p_inf) / q, (w.p0 - p_inf) / q))
                .collect();
            cps.sort_by(|a, b| a.0.total_cmp(&b.0));
            // Cp at the data orifices (linear interpolation of the wall samples)
            let at = |x: f64| -> f64 {
                let i = cps.partition_point(|c| c.0 < x).clamp(1, cps.len() - 1);
                let (a, b) = (cps[i - 1], cps[i]);
                a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0)
            };
            let xs = if k == 0 || true { fox::OGIVE_CYL_X_D } else { fox::OGIVE_CYL_X_D };
            let mut sq = 0.0;
            let mut n = 0;
            for &x in xs.iter().filter(|&&x| x > 3.0) {
                if let Some(d) = data_cp(k, x) {
                    println!("{tag} cp x {x:.3} data {d:+.4} solver {:+.4} diff {:+.4}", at(x), at(x) - d);
                    sq += (at(x) - d).powi(2);
                    n += 1;
                }
            }
            println!("{tag} rms Cp x>3 {:.4} (n {n})", (sq / n as f64).sqrt());
            // first fluid row above the body: total pressure and entropy loss per column
            let pt = |w: &[f64; 5]| {
                let c2 = GAMMA * w[4] / w[0];
                let m2 = (w[1] * w[1] + w[2] * w[2] + w[3] * w[3]) / c2;
                w[4] * (1.0 + 0.5 * (GAMMA - 1.0) * m2).powf(GAMMA / (GAMMA - 1.0))
            };
            let fs = h.cfg.freestream;
            let (pt_inf, s_inf) = (pt(&fs), fs[4] / fs[0].powf(GAMMA));
            let step = (g.ni / 60).max(1);
            for i in (0..g.ni).step_by(step) {
                let x = g.xc[g.cell(i, 0, 0)][0];
                if x < 0.2 || x > 5.0 {
                    continue;
                }
                // the first fluid cell above a solid one
                let Some(j) = (1..g.nj).find(|&j| g.is_solid(i, j - 1, 0) && !g.is_solid(i, j, 0)) else { continue };
                let w1 = cons_to_prim3(&h.u[g.cell(i, j, 0)]);
                let w4 = cons_to_prim3(&h.u[g.cell(i, (j + 4).min(g.nj - 1), 0)]);
                println!(
                    "{tag} loss x {x:.3} pt1 {:+.5} s1 {:+.5} | pt+4 {:+.5} s+4 {:+.5} | y {:.4}",
                    1.0 - pt(&w1) / pt_inf,
                    w1[4] / w1[0].powf(GAMMA) / s_inf - 1.0,
                    1.0 - pt(&w4) / pt_inf,
                    w4[4] / w4[0].powf(GAMMA) / s_inf - 1.0,
                    g.xc[g.cell(i, j, 0)][1].hypot(g.xc[g.cell(i, j, 0)][2]) - case.body.r(x)
                );
            }
            let _ = x_bt;
        }
    }
}

/// Segments of the aft body that are the boattail (not the cylinder): all but the first.
fn sol_nseg(case: &Case3) -> usize {
    case.body.segments.len() - 1
}

/// GPU (f32) against the CPU solver iteration by iteration on the same case (`BT_MACHS` first entry, `BT_VARIANTS` first entry, `BT_STEPS` iterations, checkpoints every `BT_EVERY`):
/// residuals, the first non-finite cell and the largest density difference, with the cell's place.
#[cfg(feature = "gpu")]
#[test]
#[ignore = "diagnosis: GPU against CPU on the Euler wedge"]
fn bt2_gpu_vs_cpu() {
    let m: f64 = std::env::var("BT_MACHS").ok().and_then(|v| v.split(',').next().map(|x| x.parse().unwrap())).unwrap_or(0.608);
    let variants: Vec<String> = std::env::var("BT_VARIANTS").unwrap_or_else(|_| "base".into()).split(',').map(|s| s.trim().to_string()).collect();
    for variant in variants {
        bt2_one(m, &variant);
    }
}

#[cfg(feature = "gpu")]
fn bt2_one(m: f64, variant: &str) {
    use bowshock_cfd::gpu::Precision;
    let steps: usize = std::env::var("BT_STEPS").ok().and_then(|v| v.parse().ok()).unwrap_or(140);
    let every: usize = std::env::var("BT_EVERY").ok().and_then(|v| v.parse().ok()).unwrap_or(10);
    let case = case_of(m, variant);
    let prec = if std::env::var("BT_PREC").as_deref() == Ok("ext") { Precision::Extended } else { Precision::F32 };
    let mut r = bowshock_core::engine::gpu3::GpuRunner3::new(&case, prec).expect("gpu runner");
    let mut cpu = case.build_solver_cached().expect("cpu solver");
    let tag = format!("BT2 {variant} M{m:.3}");
    let g = cpu.grid.clone();
    println!("{tag} grid {}x{} first_order {} cfl_max {} scheme {:?}", g.ni, g.nj, cpu.cfg.first_order_iters, cpu.cfl_cap(), r.gpu.config().scheme);
    let mut it = 0;
    while it < steps {
        let rg = r.gpu.run(every).expect("gpu run");
        let rc: Vec<f64> = (0..every).map(|_| cpu.step()).collect();
        it += every;
        let ug = r.gpu.state_f64().expect("state");
        let mut bad = 0;
        let mut first_bad = None;
        let (mut dmax, mut dloc) = (0.0f64, (0, 0));
        for j in 0..g.nj {
            for i in 0..g.ni {
                let c = g.cell(i, j, 0);
                if g.is_solid(i, j, 0) {
                    continue;
                }
                let (a, b) = (ug[c], cpu.u[c]);
                if !a.iter().all(|x| x.is_finite()) {
                    bad += 1;
                    first_bad.get_or_insert((i, j));
                    continue;
                }
                let d = (a[0] - b[0]).abs() / b[0];
                if d > dmax {
                    dmax = d;
                    dloc = (i, j);
                }
            }
        }
        // the five cells with the largest difference in any component (scaled by the free-stream state)
        let scale = [1.0, 0.6, 0.3, 0.3, 0.3, 2.0];
        let mut diffs: Vec<(f64, usize, usize, usize)> = Vec::new();
        for j in 0..g.nj {
            for i in 0..g.ni {
                let c = g.cell(i, j, 0);
                if g.is_solid(i, j, 0) || !ug[c].iter().all(|x| x.is_finite()) {
                    continue;
                }
                for k in 0..5 {
                    diffs.push(((ug[c][k] - cpu.u[c][k]).abs() / scale[k], i, j, k));
                }
            }
        }
        diffs.sort_by(|a, b| b.0.total_cmp(&a.0));
        let top: Vec<String> = diffs.iter().take(5).map(|(d, i, j, k)| format!("{d:.1e}@({i},{j}).{k}")).collect();
        println!("{tag}   top diffs {}", top.join(" "));
        let place = |(i, j): (usize, usize)| {
            let c = g.cell(i, j, 0);
            format!("(i {i}, j {j}; x {:.3} r {:.3})", g.xc[c][0], g.xc[c][1].hypot(g.xc[c][2]))
        };
        println!(
            "{tag} it {it} res gpu {:.3e} cpu {:.3e} | nonfinite {bad} first {} | max drho/rho {:.3e} at {}",
            rg.last().copied().unwrap_or(f64::NAN),
            rc.last().copied().unwrap_or(f64::NAN),
            first_bad.map_or("-".to_string(), place),
            dmax,
            place(dloc)
        );
        // the outflow corner: the last column, the first fluid row above the sting (or the axis)
        let jf = (0..g.nj).find(|&j| !g.is_solid(g.ni - 1, j, 0)).unwrap_or(0);
        let cc = g.cell(g.ni - 1, jf, 0);
        let f = |u: &[f64; 5]| format!("[{:.4} {:.4} {:.4} {:.4}]", u[0], u[1], u[2], u[4]);
        println!("{tag}   corner (i {}, j {jf}) gpu {} cpu {}", g.ni - 1, f(&ug[cc]), f(&cpu.u[cc]));
        if bad > 0 || dmax > 1e3 {
            break;
        }
    }
}

/// The legacy 2D axisymmetric engine (`bowshock_cases::subsonic::fox_case`, the same grids and numerics without the wedge) on the Fox body: the independent-code comparison of `bt1`.
/// `BT_MACHS`, `BT_LEVELS` (comma list, default 0).
#[test]
#[ignore = "diagnosis: Fox boattail on the legacy 2D engine"]
fn bt3_fox_legacy_2d() {
    let machs: Vec<f64> = std::env::var("BT_MACHS").map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or_else(|_| fox::MACH.to_vec());
    let levels: Vec<u32> = std::env::var("BT_LEVELS").map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or_else(|_| vec![0]);
    for level in levels {
        for &m in &machs {
            let k = fox::MACH.iter().position(|x| (x - m).abs() < 1e-9).expect("a Fox Mach number");
            let case = bowshock_cases::subsonic::fox_case(false, m, level);
            let t0 = std::time::Instant::now();
            let r = bowshock_core::engine::run::run_cold(&case);
            let tag = format!("BT 2D lvl{level} M{m:.3}");
            println!(
                "{tag} iters {} stop {:?} drop {:.2} t {:.0}s | cd_nose {:+.4} cd_aft {:?} cd_base {:+.4} cp_base {:+.4}",
                r.iters,
                r.stop_reason,
                r.drop_orders,
                t0.elapsed().as_secs_f64(),
                r.cd_nose,
                r.cd_aft.iter().map(|a| (a * 1e4).round() / 1e4).collect::<Vec<_>>(),
                r.cd_base,
                r.cp_base
            );
            let mut cps: Vec<(f64, f64)> = r.wall.iter().map(|w| (w.x, w.cp)).collect();
            cps.sort_by(|a, b| a.0.total_cmp(&b.0));
            let at = |x: f64| -> f64 {
                let i = cps.partition_point(|c| c.0 < x).clamp(1, cps.len() - 1);
                let (a, b) = (cps[i - 1], cps[i]);
                a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0)
            };
            let mut sq = 0.0;
            let mut n = 0;
            for &x in fox::OGIVE_CYL_X_D.iter().filter(|&&x| x > 3.0) {
                if let Some(d) = data_cp(k, x) {
                    println!("{tag} cp x {x:.3} data {d:+.4} solver {:+.4} diff {:+.4}", at(x), at(x) - d);
                    sq += (at(x) - d).powi(2);
                    n += 1;
                }
            }
            println!("{tag} rms Cp x>3 {:.4} (n {n})", (sq / n as f64).sqrt());
        }
    }
}
