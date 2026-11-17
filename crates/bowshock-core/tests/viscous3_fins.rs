//! Solver-level checks of the 3D viscous (RANS) CPU solver on finned grids: does the SST solver run stably at the CFL of the body cases on the Coarse-tier sector and on
//! a half domain, with fin walls in the k lines, junction and tip cells, fin leading edges and the axis ghost of an odd sector count. The gates themselves are
//! `tests/fin_lift_gates.rs`. All `#[ignore]`: minutes of CPU each.
//!
//! Environment knobs: `MACH`, `MACH_MAX` (grid sizing, 2), `RE_C` (1e6), `N` (fin count, 4), `LEVEL`, `ITERS`, `EVERY`, `CFL` (cfl_max; default 50 below Mach 1, 20 above, as the
//! 2D `Numerics::viscous`), `GROWTH` (CFL growth per iteration, 1.02), `LINES` (`block` | `diag`), `PRECOND` (`0` | `1`), `NK` (azimuthal cells of the sector), `TIER` (`coarse` |
//! `fine`), `BODY` (`anf` | `short`), `TUBE`, `CHORD_CELLS`, `SPAN_CELLS` (the fin grid resolution), `ALPHA` (degrees, half domain), `LAW_I` (`1`: the automatic wall treatment on
//! the i walls too), `THREADS`.

use bowshock_cfd::gas3::cons_to_prim3;
use bowshock_cfd::grid3::{Grid3, Part};
use bowshock_cfd::implicit::{DtMode, ImplicitParams, LineSolve3};
use bowshock_cfd::solver::{Turbulence, Viscous};
use bowshock_cfd::solver3::{RansOpts3, Scheme3, Solver3};
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::grid3::{body_grid3, Fins3};
use bowshock_core::engine::run3;
use bowshock_core::FinProfile;
use std::time::Instant;

fn env<T: std::str::FromStr>(key: &str, default: T) -> T {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// The G-F1 fins: thin double wedge (t/c 2 %), chord 0.5 d, span 0.5 d, `n` of them.
fn gf1_fins(n: u32) -> Fins3 {
    let (chord, tc) = (0.5, 0.02);
    Fins3 {
        count: n,
        root_chord: chord,
        tip_chord: chord,
        span: 0.5,
        sweep: 0.0,
        thickness: tc * chord,
        te_to_base: 0.0,
        profile: FinProfile::DoubleWedge,
        planform: None,
    }
}

fn flow() -> WallFlow {
    WallFlow {
        re_c: env("RE_C", 1.0e6),
        mach_max: env("MACH_MAX", 2.0),
        ks_body: 0.0,
        ks_fin: 0.0,
        sutherland: bowshock_core::engine::case3::SUTHERLAND_SEA_LEVEL,
    }
}

fn tier() -> Tier {
    match std::env::var("TIER").as_deref() {
        Ok("fine") => Tier::Fine(flow()),
        _ => Tier::Coarse(flow()),
    }
}

/// The body of the experiments: the ANF (`BODY=anf`, default: 10 d long) or a short cone-cylinder of the same nose (`BODY=short`, `TUBE` d of tube, default 1.5).
fn body() -> BaseBody {
    match std::env::var("BODY").as_deref() {
        Ok("short") => BaseBody::cylinder(Body { nose: Nose::Cone { half_angle: 10f64.to_radians() } }, env("TUBE", 1.5)),
        _ => Case3::anf_body(),
    }
}

fn resolution(mut c: Case3) -> Case3 {
    if let Some(n) = std::env::var("CHORD_CELLS").ok().and_then(|v| v.parse().ok()) {
        c.grid.fin_chord_cells = Some(n);
    }
    if let Some(n) = std::env::var("SPAN_CELLS").ok().and_then(|v| v.parse().ok()) {
        c.grid.span_cells = Some(n);
    }
    c
}

/// The finned sector of the G-F1 case.
fn sector_case(mach: f64, n: u32, level: u32) -> Case3 {
    let mut c = Case3::sector(mach, body(), gf1_fins(n), tier(), level);
    if let Some(nk) = std::env::var("NK").ok().and_then(|v| v.parse().ok()) {
        c.grid.nk = nk;
    }
    resolution(c)
}

/// The finned half domain of the same body at incidence `ALPHA` degrees.
fn half_case(mach: f64, n: u32, level: u32) -> Case3 {
    resolution(Case3::base_half(mach, env("ALPHA", 0.0f64).to_radians(), body(), Some(gf1_fins(n)), tier(), level))
}

fn describe(label: &str, g: &Grid3) {
    let n = g.ni * g.nj * g.nk;
    let dead = g.part.iter().filter(|&&p| p == bowshock_cfd::grid3::PART_DEAD).count();
    let solid = g.part.iter().filter(|&&p| p != 0).count();
    println!(
        "{label}: {} x {} x {} = {n} cells, {dead} dead, {solid} solid, sym {:?}, theta_span {:.4}, axis runs {}, wall_j0 {:?}",
        g.ni,
        g.nj,
        g.nk,
        g.sym,
        g.theta_span,
        g.axis.len(),
        g.wall_j0
    );
    let vmin = g.vol.iter().zip(&g.part).filter(|(_, &p)| p == 0).map(|(v, _)| *v).fold(f64::INFINITY, f64::min);
    println!("  parts {:?}  roughness {:?}  min fluid cell volume {vmin:.3e}", g.parts, g.roughness);
}

/// The viscous solver on the grid of `case` with the numerics of the 2D wall-resolved cases (`Numerics::viscous`: van Albada + the low-Mach correction and
/// preconditioning below Mach 1, minmod above, `cfl_max` 50 / 20, `DtMode::Sum`, 3 sub-iterations, 200 first-order iterations, characteristic outflow).
fn build(case: &Case3) -> Solver3 {
    let grid = body_grid3(&case.grid).expect("grid");
    let mut s = case.solver_on(grid);
    let subsonic = case.mach < 1.0;
    let p = ImplicitParams {
        dt_mode: DtMode::Sum,
        cfl_max: env("CFL", if subsonic { 50.0 } else { 20.0 }),
        growth: env("GROWTH", 1.02),
        sub_iters: env("SUB", 3),
        low_mach_precond: env("PRECOND", u8::from(subsonic)) != 0,
        line_solve: if std::env::var("LINES").as_deref() == Ok("diag") { LineSolve3::DiagonalCorrected } else { LineSolve3::Block },
        ..ImplicitParams::DEFAULT
    };
    s.cfg.scheme = Scheme3::Implicit(p);
    s.cfg.limiter = if subsonic { bowshock_cfd::flux::Limiter::VanAlbada } else { bowshock_cfd::flux::Limiter::Minmod };
    s.cfg.low_mach = subsonic;
    s.cfg.first_order_iters = 200;
    let re_c = flow().re_c;
    let v = Viscous { re_c, sutherland: 110.4 / 288.15, turbulence: Turbulence::Sst, wall_i_start: 0, roughness: 0.0 };
    let ks = s.grid.wall_ks();
    let opts = RansOpts3 {
        pressure_outflow: false,
        ks: if ks.iter().any(|&k| k > 0.0) { ks } else { Vec::new() },
        law_i_walls: env("LAW_I", 0u8) != 0,
        ..RansOpts3::default()
    };
    s.set_viscous_with(Some(v), opts);
    s.restart_ramp();
    s
}

/// Friction of the fin walls (and of the body), the viscous traction `tau_x` over the area of the faces against a `Part::Fin(_)` / body part, scaled to the whole body
/// (`2 pi / theta_span` of the domain) as a coefficient on `pi/4` of the nose diameter; `y+` of the first cell (min, mean, max) of the fin side faces.
#[derive(Debug, Default, Clone, Copy)]
struct Friction {
    fin: f64,
    body: f64,
    yp_min: f64,
    yp_mean: f64,
    yp_max: f64,
}

fn friction(s: &Solver3, mach: f64) -> Friction {
    let g = &s.grid;
    let fs = s.cfg.freestream;
    let q = 0.5 * fs[0] * fs[1] * fs[1] * 0.0 + 0.5 * mach * mach;
    let norm = (2.0 * std::f64::consts::PI / g.theta_span) / (q * std::f64::consts::FRAC_PI_4);
    let rans = s.rans().expect("viscous");
    let (mut fin, mut body, mut n, mut mean, mut area) = (0.0, 0.0, 0usize, 0.0, 0.0);
    let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
    for t in s.wall_tractions() {
        let [i, j, k] = t.idx;
        // the two cells of the face: the solid side tells the part
        let (cl, ch) = match t.dir {
            0 => (g.cell(i - 1, j, k), g.cell(i, j, k)),
            1 if j > 0 => (g.cell(i, j - 1, k), g.cell(i, j, k)),
            2 if k > 0 => (g.cell(i, j, k - 1), g.cell(i, j, k)),
            _ => continue,
        };
        let (solid, fluid) = if g.part[cl] != 0 { (cl, ch) } else { (ch, cl) };
        match g.part_of(solid % g.ni, (solid / g.ni) % g.nj, solid / (g.ni * g.nj)) {
            Some(Part::Fin(_)) => {
                fin += t.tau[0] * t.area;
                // fin sides: the face normal is azimuthal
                if t.dir == 2 {
                    let f = &g.kface[(k * g.nj + j) * g.ni + i];
                    let x = g.xc[fluid];
                    let d = ((x[0] - f.c[0]) * t.n[0] + (x[1] - f.c[1]) * t.n[1] + (x[2] - f.c[2]) * t.n[2]).abs();
                    let tau = (t.tau[0] * t.tau[0] + t.tau[1] * t.tau[1] + t.tau[2] * t.tau[2]).sqrt();
                    let yp = d * (s.u[fluid][0] * tau).sqrt() / rans.mu[fluid];
                    lo = lo.min(yp);
                    hi = hi.max(yp);
                    mean += yp * t.area;
                    area += t.area;
                    n += 1;
                }
            }
            Some(_) => body += t.tau[0] * t.area,
            None => {}
        }
    }
    let _ = n;
    Friction { fin: norm * fin, body: norm * body, yp_min: lo, yp_mean: mean / area.max(1e-300), yp_max: hi }
}

/// The turbulence quantities at the fin leading edge: over the fin span above the body's boundary layer (`r` 0.7 to 0.95) and the first three azimuthal cells next to the fin plane, the largest `k / k_inf`
/// and `mu_t / mu` in the cells within 0.05 d ahead of the first fin cell, within 0.05 d behind it (the leading edge) and 0.35 to 0.45 d behind it (a developed layer):
/// an SST stagnation over-production shows as the first two being much larger than the third.
fn le_report(s: &Solver3) {
    let g = &s.grid;
    let rans = s.rans().expect("viscous");
    let ijk = |c: usize| (c % g.ni, (c / g.ni) % g.nj, c / (g.ni * g.nj));
    let is_fin = |c: usize| {
        let (i, j, k) = ijk(c);
        matches!(g.part_of(i, j, k), Some(Part::Fin(_)))
    };
    let x_le = (0..g.part.len()).filter(|&c| is_fin(c)).map(|c| g.xc[c][0]).fold(f64::INFINITY, f64::min);
    let k_inf = bowshock_cfd::turbulence3::TurbModel::inf(&rans.model)[0];
    let mut zones = [(0.0f64, 0.0f64); 3];
    for c in 0..g.part.len() {
        let (_, _, k) = ijk(c);
        let r = g.xc[c][1].hypot(g.xc[c][2]);
        if g.part[c] != 0 || k > 2 || !(0.7..=0.95).contains(&r) {
            continue;
        }
        let dx = g.xc[c][0] - x_le;
        let zone = if (-0.05..0.0).contains(&dx) {
            0
        } else if (0.0..0.05).contains(&dx) {
            1
        } else if (0.35..0.45).contains(&dx) {
            2
        } else {
            continue;
        };
        zones[zone].0 = zones[zone].0.max(rans.turb[c][0] / k_inf);
        zones[zone].1 = zones[zone].1.max(rans.mu_t[c] / rans.mu[c]);
    }
    println!(
        "    fin LE (x {x_le:.3}): k/k_inf, mu_t/mu max: ahead {:.1} {:.1}  at the edge {:.1} {:.1}  developed {:.1} {:.1}",
        zones[0].0, zones[0].1, zones[1].0, zones[1].1, zones[2].0, zones[2].1
    );
}

/// `iters` iterations, a line every `every`: residual, drag parts (pressure of nose, aft, base, fins), friction of fins and body, `y+` of the fin sides, the largest
/// `mu_t / mu` and the minimum density / pressure. Stops at a non-finite residual.
fn run(label: &str, case: &Case3, iters: usize, every: usize) -> Solver3 {
    let t_setup = Instant::now();
    let mut s = build(case);
    println!("  {label}: solver and viscous set-up (wall distance of every cell, face directions): {:.1} s", t_setup.elapsed().as_secs_f64());
    describe(label, &s.grid);
    let t0 = Instant::now();
    let mut hist: Vec<f64> = Vec::new();
    for it in 0..iters {
        let r = s.step();
        hist.push(r);
        if !r.is_finite() {
            println!("  {label}: non-finite residual at iteration {}", it + 1);
            break;
        }
        if (it + 1) % every == 0 || it + 1 == iters {
            let p = run3::drag_parts(case, &s.grid, &s.u, s.cfg.freestream[4]);
            let f = friction(&s, case.mach);
            let rans = s.rans().unwrap();
            let mut mu_t_max = 0.0f64;
            let (mut rho_min, mut p_min) = (f64::INFINITY, f64::INFINITY);
            for c in 0..s.u.len() {
                if s.grid.part[c] != 0 {
                    continue;
                }
                mu_t_max = mu_t_max.max(rans.mu_t[c] / rans.mu[c]);
                let w = cons_to_prim3(&s.u[c]);
                rho_min = rho_min.min(w[0]);
                p_min = p_min.min(w[4]);
            }
            println!(
                "  {label} it {:5}  r {:.3e}  CD p(nose {:.4} aft {:.4} base {:.4} fins {:.4}) fric(fin {:.5} body {:.4})  fin y+ {:.1}/{:.1}/{:.1}  mut/mu max {mu_t_max:.0}  rho min {rho_min:.3} p min {p_min:.3}  {:.1} s ({:.0} ms/it)",
                it + 1,
                r,
                p.nose,
                p.aft.iter().sum::<f64>(),
                p.base,
                p.fin_sides + p.fin_edges + p.fin_tips,
                f.fin,
                f.body,
                f.yp_min,
                f.yp_mean,
                f.yp_max,
                t0.elapsed().as_secs_f64(),
                1e3 * t0.elapsed().as_secs_f64() / (it + 1) as f64
            );
            le_report(&s);
        }
    }
    residual_map(&s);
    if std::env::var_os("PROBE").is_some() {
        axis_probe(&mut s);
    }
    s
}

/// The axis cell of the largest `|d rho / dt|`: the state of its row of `k` cells across the sector, their residuals, and the update of the next step (a limit cycle of the
/// cell shows as an update as large as the swirl it has).
fn axis_probe(s: &mut Solver3) {
    let g = &s.grid;
    let Some((c0, _)) = (0..s.res.len())
        .filter(|&c| g.part[c] == 0 && g.is_axis(c % g.ni, (c / g.ni) % g.nj))
        .map(|c| (c, s.res[c][0].abs()))
        .max_by(|a, b| a.1.total_cmp(&b.1))
    else {
        println!("  no axis cell");
        return;
    };
    let (i, j) = (c0 % g.ni, (c0 / g.ni) % g.nj);
    println!("  axis probe at i {i} j {j}: x {:.4}", g.xc[c0][0]);
    let (nk, ni, nj) = (g.nk, g.ni, g.nj);
    let row = |k: usize| (k * nj + j) * ni + i;
    for k in 0..nk {
        let c = row(k);
        let w = s.prim(i, j, k);
        println!(
            "    k {k:2}: r {:.1e} vol {:.1e} dt {:.1e}  w [{:.5} {:.5} {:.2e} {:.2e} {:.5}]  res [{:.1e} {:.1e} {:.1e} {:.1e} {:.1e}]",
            s.grid.xc[c][1].hypot(s.grid.xc[c][2]),
            s.grid.vol[c],
            s.dt[c],
            w[0],
            w[1],
            w[2],
            w[3],
            w[4],
            s.res[c][0],
            s.res[c][1],
            s.res[c][2],
            s.res[c][3],
            s.res[c][4]
        );
    }
    let before = s.u.clone();
    s.step();
    for k in 0..nk {
        let c = row(k);
        let d: Vec<String> = (0..5).map(|m| format!("{:.1e}", s.u[c][m] - before[c][m])).collect();
        println!("    k {k:2}: du [{}]  u [{:.5} {:.5} {:.2e} {:.2e} {:.5}]", d.join(" "), s.u[c][0], s.u[c][1], s.u[c][2], s.u[c][3], s.u[c][4]);
    }
}

/// The cells of the largest `|d rho / dt|` of the last residual evaluation: where the residual that does not fall sits.
fn residual_map(s: &Solver3) {
    let g = &s.grid;
    let mut v: Vec<(f64, usize)> = (0..s.res.len()).filter(|&c| g.part[c] == 0).map(|c| (s.res[c][0].abs(), c)).collect();
    v.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (r, c) in v.iter().take(8) {
        let (i, j, k) = (c % g.ni, (c / g.ni) % g.nj, c / (g.ni * g.nj));
        let x = g.xc[*c];
        println!(
            "    |d rho/dt| {r:.3e} at ({i}, {j}, {k})  x {:.3} r {:.4} theta {:.3}  vol {:.2e}  mu_t/mu {:.1}",
            x[0],
            x[1].hypot(x[2]),
            x[1].atan2(x[2]),
            g.vol[*c],
            s.rans().unwrap().mu_t[*c] / s.rans().unwrap().mu[*c]
        );
    }
}

#[test]
#[ignore]
fn fin_grids_report() {
    let level: u32 = env("LEVEL", 0);
    let n: u32 = env("N", 4);
    let mach: f64 = env("MACH", 0.5);
    for (label, case) in [("sector", sector_case(mach, n, level)), ("half", half_case(mach, n, level))] {
        match body_grid3(&case.grid) {
            Ok(g) => describe(label, &g),
            Err(e) => println!("{label}: {e}"),
        }
    }
}

/// The finned sector (N = `N`, 4) on the Coarse-tier fin stack, Mach `MACH`, `ITERS` iterations at the CFL of the body cases.
#[test]
#[ignore]
fn fin_sector_runs() {
    let case = sector_case(env("MACH", 0.5), env("N", 4), env("LEVEL", 0));
    run("sector", &case, env("ITERS", 400), env("EVERY", 50));
}

/// The finned half domain at incidence `ALPHA`.
#[test]
#[ignore]
fn fin_half_runs() {
    let case = half_case(env("MACH", 0.5), env("N", 4), env("LEVEL", 0));
    run("half", &case, env("ITERS", 300), env("EVERY", 25));
}

// ---------------------------------------------------------------------------------------------------------------------------------
// the finned half domain at incidence, the release path of the lift tier

/// The half domain of the G-L1 ANF case (`anf_half` of `tests/fin_lift_gates.rs`): ANF body and fins, the tier `TIER` (`lift` default here), `ALPHA` degrees, `MACH`,
/// `PHASE` (`between` | `inplane`), grid sized for `MACH_MAX` (3.147) at the sea-level `Re_c` of the 0.03 m ANF (6.99e5), the viscous engine as `Case3::with_tier` wires it.
fn anf_incidence_case(mach: f64, alpha_deg: f64) -> Case3 {
    use bowshock_core::engine::grid3::{Domain3, FinPhase};
    let flow = WallFlow {
        re_c: env("RE_C", 1.225 * 340.294 * 0.03 / 1.7894e-5),
        mach_max: env("MACH_MAX", 3.147),
        ks_body: 0.0,
        ks_fin: 0.0,
        sutherland: bowshock_core::engine::case3::SUTHERLAND_SEA_LEVEL,
    };
    let tier = match std::env::var("TIER").as_deref() {
        Ok("coarse") => Tier::Coarse(flow),
        Ok("fine") => Tier::Fine(flow),
        _ => Tier::Lift(flow),
    };
    let mut c = Case3::base_half(mach, alpha_deg.to_radians(), body(), Some(Case3::anf_fins()), tier, env("LEVEL", 0));
    let phase = if std::env::var("PHASE").as_deref() == Ok("inplane") { FinPhase::InPlane } else { FinPhase::Between };
    c.grid.domain = Domain3::Half { phase };
    resolution(c)
}

/// A run as `run3::run3_on` does it (the same stop rule, the same samples) on the solver `Case3::build_solver` gives (the viscous tier, its numerics: the engine's, not
/// the 2D ones of `build`), recording CA, CN and x_cp every `MEAN_EVERY` iterations: the verdict of the mean-stationary rule and the oscillation of every force over the
/// last `WINDOW` iterations.
#[test]
#[ignore]
fn fin_half_incidence_series() {
    use bowshock_core::engine::run3::{lift_forces, StopRule3, MEAN_EVERY, SAMPLE_EVERY};
    let (mach, alpha): (f64, f64) = (env("MACH", 0.8), env("ALPHA", 2.0));
    let mut case = anf_incidence_case(mach, alpha);
    if let Some(n) = std::env::var("MAX_ITER").ok().and_then(|v| v.parse().ok()) {
        case.numerics.max_iter = n;
    }
    if let Some(c) = std::env::var("CFL_MAX").ok().and_then(|v| v.parse().ok()) {
        case.numerics.implicit.cfl_max = c;
    }
    let mut s = case.build_solver().expect("solver");
    describe("half", &s.grid);
    println!(
        "  tier {:?}, scheme {:?}, limiter {:?}, viscous {}, max_iter {}, stop rule window {} rel_tol {}",
        case.tier,
        s.cfg.scheme,
        s.cfg.limiter,
        s.viscous().is_some(),
        case.numerics.max_iter,
        case.numerics.window,
        case.numerics.rel_tol
    );
    s.cfg.freestream = case.freestream();
    s.restart_ramp();
    let mut rule = StopRule3::new(&case, false);
    let t0 = Instant::now();
    let mut series: Vec<(usize, f64, f64, f64)> = Vec::new();
    let mut it = 0;
    let mut hist = Vec::new();
    let diag_from: usize = env("DIAG_FROM", 100);
    let mut prev: Vec<bowshock_cfd::gas3::Cons3> = Vec::new();
    while it < case.numerics.max_iter {
        if it >= diag_from {
            prev.clone_from(&s.u);
        }
        let r = s.step();
        let monitor = ((it + 1) % SAMPLE_EVERY == 0).then(|| {
            let f = lift_forces(&case, &s);
            if alpha.abs() < 1e-12 {
                f.ca
            } else {
                f.cn
            }
        });
        it += 1;
        if rule.wants_forces(it, MEAN_EVERY) {
            let f = lift_forces(&case, &s);
            rule.observe(it, &f);
            series.push((it, f.ca, f.cn, f.x_cp));
        }
        hist.push(r);
        if let Some(why) = rule.check(it - 1, r, monitor) {
            println!("  stop rule at iteration {it}: {why:?}");
            if !prev.is_empty() && matches!(why, bowshock_core::StopReason::NonFinite) {
                println!("  state before the last step:");
                blowup_report(&s, &prev);
                println!("  state after it:");
                blowup_report(&s, &s.u);
            }
            break;
        }
        if it % env("EVERY", 250usize) == 0 {
            let f = lift_forces(&case, &s);
            println!(
                "  it {it:5}  r {r:.3e}  CA {:.5}  CN {:.5}  x_cp {:.4}  {:.0} s ({:.0} ms/it)",
                f.ca,
                f.cn,
                f.x_cp,
                t0.elapsed().as_secs_f64(),
                1e3 * t0.elapsed().as_secs_f64() / it as f64
            );
        }
        if !r.is_finite() {
            println!("  non-finite residual at iteration {it}");
            break;
        }
    }
    let result = rule.finish(&case, lift_forces(&case, &s), it);
    println!(
        "  result: {:?} after {} iterations: CN {:.5} CA {:.5} x_cp {:.4} y+ max (n/a) window {:?}",
        result.stop_reason, result.iterations, result.cn, result.ca, result.x_cp, result.window
    );
    // the oscillation of each force over the last windows of 500 / 1000 / 2000 iterations: mean and half range / |mean|
    for w in [500usize, 1000, 2000] {
        let tail: Vec<_> = series.iter().filter(|(i, ..)| *i + w > it).collect();
        if tail.len() < 10 {
            continue;
        }
        let stat = |f: &dyn Fn(&(usize, f64, f64, f64)) -> f64| {
            let v: Vec<f64> = tail.iter().map(|t| f(t)).collect();
            let mean = v.iter().sum::<f64>() / v.len() as f64;
            let (lo, hi) = v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &x| (l.min(x), h.max(x)));
            (mean, 0.5 * (hi - lo), 0.5 * (hi - lo) / mean.abs().max(1e-300))
        };
        let (ca, cn, xc) = (stat(&|t| t.1), stat(&|t| t.2), stat(&|t| t.3));
        println!(
            "  last {w} its: CA {:.5} +- {:.5} ({:.2} %)  CN {:.5} +- {:.5} ({:.2} %)  x_cp {:.4} +- {:.4} ({:.2} %)",
            ca.0,
            ca.1,
            100.0 * ca.2,
            cn.0,
            cn.1,
            100.0 * cn.2,
            xc.0,
            xc.1,
            100.0 * xc.2
        );
    }
}

/// Where a state is about to blow up: the cells of lowest pressure, lowest density and highest speed (and any non-finite one), with their place in the grid.
fn blowup_report(s: &Solver3, u: &[bowshock_cfd::gas3::Cons3]) {
    let g = &s.grid;
    let rows: Vec<(usize, [f64; 5])> = (0..u.len()).filter(|&c| g.part[c] == 0).map(|c| (c, cons_to_prim3(&u[c]))).collect();
    let bad = rows.iter().filter(|(_, w)| !w.iter().all(|x| x.is_finite())).count();
    println!("    non-finite fluid cells: {bad} of {}", rows.len());
    let place = |c: usize| {
        let (i, j, k) = (c % g.ni, (c / g.ni) % g.nj, c / (g.ni * g.nj));
        let x = g.xc[c];
        // the fin / body part touching the cell (a solid neighbour)
        let mut near = String::new();
        for (di, dj, dk) in [(-1i64, 0i64, 0i64), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)] {
            let (a, b, d) = (i as i64 + di, j as i64 + dj, k as i64 + dk);
            if a >= 0 && b >= 0 && d >= 0 && (a as usize) < g.ni && (b as usize) < g.nj && (d as usize) < g.nk {
                if let Some(p) = g.part_of(a as usize, b as usize, d as usize) {
                    near.push_str(&format!("{p:?} "));
                }
            }
        }
        format!(
            "({i}, {j}, {k}) x {:.3} r {:.4} theta {:.3} vol {:.2e} axis {} next to [{near}]",
            x[0],
            x[1].hypot(x[2]),
            x[1].atan2(x[2]),
            g.vol[c],
            g.is_axis(i, j)
        )
    };
    let mut by_p: Vec<&(usize, [f64; 5])> = rows.iter().filter(|(_, w)| w[4].is_finite()).collect();
    by_p.sort_by(|a, b| a.1[4].total_cmp(&b.1[4]));
    for (c, w) in by_p.iter().take(4) {
        println!("    low p {:.3e} rho {:.3e} |u| {:.2} at {}", w[4], w[0], (w[1] * w[1] + w[2] * w[2] + w[3] * w[3]).sqrt(), place(*c));
    }
    let mut by_v: Vec<&(usize, [f64; 5])> = rows.iter().filter(|(_, w)| w.iter().all(|x| x.is_finite())).collect();
    by_v.sort_by(|a, b| (b.1[1] * b.1[1] + b.1[2] * b.1[2] + b.1[3] * b.1[3]).total_cmp(&(a.1[1] * a.1[1] + a.1[2] * a.1[2] + a.1[3] * a.1[3])));
    for (c, w) in by_v.iter().take(3) {
        println!("    fast |u| {:.2} rho {:.3e} p {:.3e} at {}", (w[1] * w[1] + w[2] * w[2] + w[3] * w[3]).sqrt(), w[0], w[4], place(*c));
    }
    if let Some((c, _)) = rows.iter().find(|(_, w)| !w.iter().all(|x| x.is_finite())) {
        println!("    first non-finite cell at {}", place(*c));
    }
}
