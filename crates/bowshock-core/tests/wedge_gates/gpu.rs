//! The GPU half of `wedge_runner()` (stage S2, `gpu3d`): the 3D solver on the GPU through `GpuRunner3` / `GpuSolver3`, the Euler equations only (viscous and
//! SST are stage S5: the runner reports them as errors, the gate fails rather than passes).
//!
//! Selected by `BOWSHOCK_WEDGE_BACKEND=gpu`; every GPU solver run goes through `tools/gpuq`:
//!
//! ```text
//! tools/gpuq/gpuq submit --host local --name wedge-gates -- env BOWSHOCK_WEDGE_BACKEND=gpu \
//!     cargo test --release -p bowshock-core --test wedge_gates -- --ignored --nocapture --test-threads=1 gate_w1 gate_w4
//! ```
//!
//! * `facade_case`: the wedge (`Case3::wedge` on the 2D case's base-wake grid) to its stop rule, drag split by wall class on the device
//!   (`GpuRunner3::drag_parts`); a wake-less case is the same on the sting grid.
//! * `cone`, `ogive_forebody_cd`: the 2D grids of `bowshock-cases` revolved into the one-cell wedge, the iteration and stop rule of those cases.
//! * `case2d`: the smoke cases of `bowshock-cases` (transonic / subsonic) on the wedge.
//! * `gpu_vs_cpu`: tiny wedge / sector, 30 iterations from the free stream, GPU against the f64 CPU solver (`k` and `omega` do not exist in Euler: 0).

use super::*;
use bowshock_cases::cone::{build_solver as cone_solver, ConeCase};
use bowshock_cases::ogive::{ogive_grid, OgiveCase};
use bowshock_cfd::gas3::{cons_to_prim3, freestream3, Cons3};
use bowshock_cfd::gpu::{GpuSolver3, Precision};
use bowshock_cfd::grid::Grid;
use bowshock_cfd::grid3::{revolve, AxisRun, Grid3, Part, Symmetry};
use bowshock_cfd::implicit_params::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};
use bowshock_core::engine::case3::{Case3, Numerics3, Tier};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::{body_grid3, Fins3, WEDGE_ANGLE};
use bowshock_core::engine::run3::{wall_forces, wall_samples, Wall};
use bowshock_core::{Analysis, Wake};
use bowshock_gpu_cases::wedge3::wedge_case3;
use serde_json::json;
use std::time::Instant;

/// `precision`: `None` runs a case in the precision of the fixture's 2D case, `Some` forces one.
pub struct GpuWedge {
    pub precision: Option<Precision>,
}

impl GpuWedge {
    pub fn new() -> GpuWedge {
        GpuWedge { precision: None }
    }
}

fn err<E: std::fmt::Display>(e: E) -> String {
    e.to_string()
}

/// The 2D engine case of a fixture facade case.
fn engine_case_of(c: &FacadeCase) -> Result<bowshock_core::engine::case::Case, String> {
    let mut a = Analysis::new(&c.body).map_err(err)?.resolution(c.resolution);
    a = if c.rans {
        // the wall-resolved grid of the facade's viscous analysis (ISA 120 m unless the fixture says otherwise), the wall roughness of the fixture
        let a = a.viscous(bowshock_core::FlightConditions { atmosphere: c.atmosphere.unwrap_or(bowshock_core::Atmosphere::Isa { altitude_m: 120.0 }) });
        let a = if c.roughness_m > 0.0 { a.roughness(c.roughness_m) } else { a };
        // the motor of the fixture case (TP-2005): the nozzle rows of the 2D base grid and the jet state (`Case3::wedge_of` carries both)
        match c.motor {
            Some(m) => a.motor(m),
            None => a,
        }
    } else {
        a.wake(if c.wake_base { Wake::Base } else { Wake::Sting })
    };
    Ok(a.engine_cases(&[c.mach]).map_err(err)?.remove(0))
}

/// The GPU precision of a facade case: single precision whatever the fixture's (the 2D table ran in df32): the product's `GpuRunner3` runs `Precision::F32`
/// (`analysis.rs`), and on the RANS wedge it is within 0.001 % CD of df32 (G-V3 0.213277 both, 5 digits) at 0.55 of the step time.
fn precision_of(_c: &FacadeCase) -> Precision {
    Precision::F32
}

/// The one-cell wedge of a 2D meridian grid `g2` (the cone and ogive grids of `bowshock-cases`: slip wall on `j = 0` from the tip, the axis upstream of
/// it): the body is the `j = 0` wall, split into the segments `Part::Body(s)` ending at the stations `ends` (the last one is open).
fn wedge_of(g2: &Grid, ends: &[f64]) -> Result<Grid3, String> {
    let mut g = revolve(g2, 1, &[0.0, WEDGE_ANGLE])?;
    g.sym = Symmetry::Wedge;
    g.theta_span = WEDGE_ANGLE;
    let n_up = (0..g2.ni).take_while(|&i| g2.x[i + 1] <= 1e-12).count();
    g.axis = vec![AxisRun { j: 0, i: 0..n_up }];
    let mut i0 = n_up;
    for (s, &x_end) in ends.iter().enumerate() {
        let i1 = (i0..g2.ni).find(|&i| g2.x[i + 1] > x_end + 1e-9).unwrap_or(g2.ni);
        g.wall_j0.push((i0..i1, Part::Body(s as u16)));
        i0 = i1;
    }
    Ok(g)
}

/// A GPU run of `grid` to the residual-drop rule of the cases of `bowshock-cases` (`drop` orders below the peak residual from iteration 20; the plateau rule
/// after 8000 iterations), the Euler equations at `mach`, the implicit scheme of the facade; returns the host solver, the final state, iterations, the stop
/// reason and the orders dropped.
struct GridRun {
    host: Solver3,
    u: Vec<Cons3>,
    iterations: usize,
    stop: &'static str,
    drop_orders: f64,
}

fn run_grid(grid: Grid3, mach: f64, first_order_iters: usize, max_iter: usize, drop: f64, prec: Precision) -> Result<GridRun, String> {
    let n = Numerics3::auto(mach, 0);
    let cfg = Config3 {
        freestream: freestream3(mach, 0.0),
        cfl: 0.5,
        limiter: n.limiter,
        first_order_iters,
        low_mach: n.low_mach,
        scheme: Scheme3::Implicit(ImplicitParams { line_solve: LineSolve3::Block, ..n.implicit_for(true) }),
    };
    let host = Solver3::new(grid, cfg);
    let ctx = luma::default_context().map_err(err)?;
    if ctx.is_cpu() {
        return Err("no GPU: Luma opened a software (CPU) Vulkan device".into());
    }
    let mut gpu = GpuSolver3::from_host(&ctx, &host, prec).map_err(err)?;
    let (mut residual, mut rmax, mut stop) = (Vec::<f64>::new(), 0.0f64, "MaxIterations");
    'run: while residual.len() < max_iter {
        let len = 50.min(max_iter - residual.len());
        for r in gpu.run(len).map_err(err)? {
            let it = residual.len();
            residual.push(r);
            if !r.is_finite() {
                stop = "NonFinite";
                break 'run;
            }
            // the residual of the first-order start-up does not count: the peak is taken from the second-order phase, whose drop is what the cases ask for
            let from = first_order_iters.max(20);
            if it >= from {
                rmax = rmax.max(r);
            }
            if it > from && rmax / r >= 10f64.powf(drop) {
                stop = "ResidualDrop";
                break 'run;
            }
            if it >= 8000 && it % 1000 == 0 {
                let m = |a: usize, b: usize| residual[a..b].iter().map(|x| x.log10()).sum::<f64>() / (b - a) as f64;
                if m(it - 2000, it) - m(it - 4000, it - 2000) > -0.05 {
                    stop = "ResidualPlateau";
                    break 'run;
                }
            }
        }
    }
    let u = gpu.state_f64().map_err(err)?;
    let last = residual.last().copied().unwrap_or(rmax);
    Ok(GridRun { host, u, iterations: residual.len(), stop, drop_orders: if rmax > 0.0 { (rmax / last).log10() } else { 0.0 } })
}

fn p_of(q: &Cons3) -> f64 {
    cons_to_prim3(q)[4]
}

impl GpuWedge {
    /// A RANS case of the facade on the single-cell wedge of its wall-resolved 2D grid, on the GPU: `Case3::wedge_of` carries the 2D case's viscous parameters and numerics
    /// (block lines), `GpuRunner3` runs SST with the shear of the no-slip faces in its device-side class forces and in the monitor.
    fn rans_case(&self, c: &FacadeCase, case2: &bowshock_core::engine::case::Case) -> Result<Numbers, String> {
        let case = Case3::wedge_of(case2, 0);
        let mut r = GpuRunner3::new(&case, self.precision.unwrap_or_else(|| precision_of(c))).map_err(err)?;
        let t = Instant::now();
        let p = r.run_on(&case, false).map_err(err)?;
        let wall_s = t.elapsed().as_secs_f64();
        let d = r.drag_parts(&case).map_err(err)?;
        // the facade reports on the largest cross-section (`BaseBody::nose_to_largest_area`), the solver on the nose-base area
        let q = case.body.nose_to_largest_area();
        println!("INFO RANS wedge {} on the GPU: {} iterations, {wall_s:.1} s, stop {:?}, {:.2} orders", c.id, p.iterations, p.stop_reason, p.drop_orders);
        Ok(Numbers {
            cd_total: q * d.total,
            cd_nose: q * d.nose,
            cd_aft: d.aft.iter().map(|x| q * x).collect(),
            cd_friction: Some(q * d.friction.total),
            cd_base: Some(q * (d.base + d.friction.base)),
            cp_base: p.cp_base.is_finite().then_some(p.cp_base),
            y_plus_max: p.y_plus_max.is_finite().then_some(p.y_plus_max),
            iterations: p.iterations,
            stop_reason: format!("{:?}", p.stop_reason),
        })
    }
}

impl WedgeRunner for GpuWedge {
    fn facade_case(&self, c: &FacadeCase) -> Result<Numbers, String> {
        let case2 = engine_case_of(c)?;
        if c.rans {
            return self.rans_case(c, &case2);
        }
        let mut case = if c.wake_base {
            wedge_case3(&case2)
        } else {
            let mut k = Case3::wedge(case2.mach, case2.body.clone(), 0);
            k.grid.base = None;
            k.grid.sting = case2.grid_sting.clone();
            k
        };
        // `DiagonalCorrected` (the GPU default of `Numerics3::implicit_for`) diverges on the wedge, CPU f64 too (cfl_scan --cpu): the block factorisation it is
        // verified against is the one that converges
        case.numerics.line_solve = Some(LineSolve3::Block);
        let mut r = GpuRunner3::new(&case, self.precision.unwrap_or_else(|| precision_of(c))).map_err(err)?;
        let p = r.run_on(&case, false).map_err(err)?;
        let d = r.drag_parts(&case).map_err(err)?;
        // the facade reports on the area of the largest diameter of the body, the solver (and `drag_parts`) on the nose-base area
        let s = (c.body.nose.diameter / c.body.reference_diameter()).powi(2);
        Ok(Numbers {
            cd_total: s * d.total,
            cd_nose: s * d.nose,
            cd_aft: d.aft.iter().map(|x| s * x).collect(),
            cd_friction: None,
            cd_base: c.wake_base.then_some(s * d.base),
            cp_base: None,
            y_plus_max: None,
            iterations: p.iterations,
            stop_reason: format!("{:?}", p.stop_reason),
        })
    }

    fn cone(&self, mach: f64, theta_deg: f64, ni: usize, nj: usize) -> Result<ConeNumbers, String> {
        let tm = bowshock_reference::taylor_maccoll::solve_cone(mach, theta_deg.to_radians()).ok_or("Taylor-Maccoll has no attached solution")?;
        let case = ConeCase::new(mach, theta_deg, ni, nj);
        let s2 = cone_solver(&case, tm.beta);
        let g = wedge_of(&s2.grid, &[f64::INFINITY])?;
        let run = run_grid(g, mach, case.first_order_iters, case.max_iter, case.drop_orders, Precision::Extended)?;
        let g = &run.host.grid;
        let (p_inf, q) = (run.host.cfg.freestream[4], 0.5 * mach * mach);
        let length = s2.grid.x[ni];
        // wall Cp from the extrapolated wall pressure, averaged over the window of the cone case
        let cps: Vec<f64> = wall_samples(g, &run.u)
            .iter()
            .filter(|w| matches!(w.class, Wall::Body(_)) && w.face.c[0] / length >= case.window.0 && w.face.c[0] / length <= case.window.1)
            .map(|w| (w.pw - p_inf) / q)
            .collect();
        let cp = cps.iter().sum::<f64>() / cps.len() as f64;
        // shock angle: per column the steepest pressure gradient in `j` (parabolic refinement), a line fitted over the window
        let (ni, nj) = (g.ni, g.nj);
        let rc = |c: usize| g.xc[c][1].hypot(g.xc[c][2]);
        let mut pts = Vec::new();
        for i in 0..ni {
            let xr = g.xc[i][0] / length;
            if xr < case.window.0 || xr > case.window.1 {
                continue;
            }
            let gr: Vec<f64> = (0..nj - 1)
                .map(|j| {
                    let (ca, cb) = (j * ni + i, (j + 1) * ni + i);
                    let d = (g.xc[cb][0] - g.xc[ca][0]).hypot(rc(cb) - rc(ca));
                    (p_of(&run.u[ca]) - p_of(&run.u[cb])).abs() / d
                })
                .collect();
            let k = (0..gr.len()).max_by(|&a, &b| gr[a].total_cmp(&gr[b])).unwrap();
            let mut frac = k as f64;
            if k > 0 && k + 1 < gr.len() {
                let (a, b, c) = (gr[k - 1], gr[k], gr[k + 1]);
                let den = a - 2.0 * b + c;
                if den < 0.0 {
                    frac += (0.5 * (a - c) / den).clamp(-0.5, 0.5);
                }
            }
            let pos = |f: &dyn Fn(usize) -> f64, fr: f64| {
                let t = fr.clamp(0.0, (nj - 2) as f64);
                let j0 = t.floor() as usize;
                let a = 0.5 * (f(j0 * ni + i) + f((j0 + 1) * ni + i));
                let j1 = (j0 + 1).min(nj - 2);
                let b = 0.5 * (f(j1 * ni + i) + f((j1 + 1) * ni + i));
                a + (t - j0 as f64) * (b - a)
            };
            pts.push((pos(&|c| g.xc[c][0], frac), pos(&rc, frac)));
        }
        let n = pts.len() as f64;
        let (sx, sr) = (pts.iter().map(|p| p.0).sum::<f64>(), pts.iter().map(|p| p.1).sum::<f64>());
        let sxx = pts.iter().map(|p| p.0 * p.0).sum::<f64>();
        let sxr = pts.iter().map(|p| p.0 * p.1).sum::<f64>();
        let beta = ((n * sxr - sx * sr) / (n * sxx - sx * sx)).atan().to_degrees();
        println!("INFO cone M {mach} {theta_deg} deg {ni}x{nj}: {} iterations, {}, {:.2} orders", run.iterations, run.stop, run.drop_orders);
        Ok(ConeNumbers { cp, beta_deg: beta })
    }

    fn ogive_forebody_cd(&self, mach: f64, level: u32) -> Result<f64, String> {
        let case = OgiveCase::level(mach, level);
        let g2 = ogive_grid(&case.grid_params());
        let g = wedge_of(&g2, &[case.body.nose_length()])?;
        let run = run_grid(g, mach, case.first_order_iters, case.max_iter, case.drop_orders, Precision::Extended)?;
        let g = &run.host.grid;
        let w = wall_forces(g, &run.u, run.host.cfg.freestream[4], f64::INFINITY);
        let q = 0.5 * mach * mach;
        println!("INFO ogive M {mach} level {level}: {} iterations, {}, {:.2} orders", run.iterations, run.stop, run.drop_orders);
        Ok(w.get(Wall::Body(0))[0] * (2.0 * std::f64::consts::PI / g.theta_span) / (q * std::f64::consts::FRAC_PI_4))
    }

    fn case2d(&self, c: &Case2d) -> Result<Value, String> {
        let i = &c.inputs;
        let mach = num(&i["mach"]);
        let level = i["level"].as_u64().unwrap_or(0) as u32;
        let cyl = num(&i["cyl_length"]);
        let case2 = match s(&i["builder"]).as_str() {
            "transonic_case" => {
                let body = bowshock_core::engine::body_grid::Body {
                    nose: bowshock_core::engine::body_grid::Nose::Cone { half_angle: (0.5 / num(&i["nose"]["cone_fineness"])).atan() },
                };
                bowshock_cases::transonic::transonic_case(mach, level, body, cyl)
            }
            "subsonic_case" => {
                let nose = bowshock_core::engine::body_grid::Nose::TangentOgive { fineness: num(&i["nose"]["tangent_ogive_fineness"]) };
                bowshock_cases::subsonic::subsonic_case(mach, nose, level, i["base"].as_bool().unwrap_or(false))
            }
            other => return Err(format!("case2d builder `{other}` is not known to the GPU runner")),
        };
        let base = case2.wake == Wake::Base;
        let mut case = if base {
            wedge_case3(&case2)
        } else {
            let mut k = Case3::wedge(case2.mach, case2.body.clone(), level);
            k.grid.base = None;
            k.grid.sting = case2.grid_sting.clone();
            k
        };
        case.numerics.first_order_iters = i["first_order_iters"].as_u64().unwrap_or(0) as usize;
        case.numerics.max_iter = i["max_iter"].as_u64().unwrap_or(0) as usize;
        case.numerics.line_solve = Some(LineSolve3::Block);
        let mut r = GpuRunner3::new(&case, Precision::F32).map_err(err)?;
        let p = r.run_on(&case, false).map_err(err)?;
        let d = r.drag_parts(&case).map_err(err)?;
        let u = r.gpu.state_f64().map_err(err)?;
        let g = &r.host.grid;
        let (p_inf, q) = (r.host.cfg.freestream[4], 0.5 * case.mach * case.mach);
        let (mut min_rho, mut min_p) = (f64::INFINITY, f64::INFINITY);
        for qc in &u {
            let w = cons_to_prim3(qc);
            min_rho = min_rho.min(w[0]);
            min_p = min_p.min(w[4]);
        }
        let samples = wall_samples(g, &u);
        let body: Vec<_> = samples.iter().filter(|w| matches!(w.class, Wall::Body(_))).collect();
        let cp = |pw: f64| (pw - p_inf) / q;
        let tip = body.iter().min_by(|a, b| a.face.c[0].total_cmp(&b.face.c[0])).map(|w| cp(w.pw));
        let cp_min = body.iter().map(|w| cp(w.pw)).fold(f64::INFINITY, f64::min);
        let base_faces: Vec<_> = samples.iter().filter(|w| w.class == Wall::Base).collect();
        let area: f64 = base_faces.iter().map(|w| w.face.area).sum();
        let cp_base = (area > 0.0).then(|| base_faces.iter().map(|w| cp(w.pw) * w.face.area).sum::<f64>() / area);
        let finite = |x: f64| if x.is_finite() { json!(x) } else { Value::Null };
        Ok(json!({
            "iterations": p.iterations,
            "residual_finite": p.drop_orders.is_finite() && !format!("{:?}", p.stop_reason).contains("NonFinite"),
            "drop_orders": p.drop_orders,
            "stop_reason": format!("{:?}", p.stop_reason),
            "min_rho": min_rho,
            "min_p": min_p,
            "cd_nose": d.nose,
            "cp_tip": tip.map_or(Value::Null, finite),
            "cp_min": cp_min,
            "cp_base": cp_base.map_or(Value::Null, finite),
            "cd_base": if base { json!(d.base) } else { json!(0.0) },
            "cd_base_plus_cp_base": cp_base.map_or(Value::Null, |c| json!(d.base + c)),
            "cd_base_minus_5_abs_cd_nose": if base { json!(d.base - 5.0 * d.nose.abs()) } else { Value::Null },
        }))
    }

    fn gpu_vs_cpu(&self, domain: TinyDomain, precision: Prec, scheme: Scheme) -> Result<StateDiff, String> {
        use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
        let mach = 2.0;
        let case = match domain {
            TinyDomain::Wedge => {
                let mut c = Case3::wedge(mach, BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0), 0);
                let p = c.grid.base.as_mut().ok_or("no base grid")?;
                p.n_up = 6;
                p.n_nose = 10;
                p.n_seg = p.n_seg.iter().map(|&n| (n / 4).max(4)).collect();
                p.n_wake = 8;
                p.n_gap = 6;
                p.nj_out = 10;
                c
            }
            TinyDomain::Sector => {
                let nose = Body { nose: Nose::Cone { half_angle: 10f64.to_radians() } };
                let body = BaseBody::cylinder(nose, 10.0 - nose.nose_length());
                let fins = Fins3 {
                    count: 4,
                    root_chord: 1.0,
                    tip_chord: 1.0,
                    span: 1.0,
                    sweep: 0.0,
                    thickness: 0.08,
                    te_to_base: 0.0,
                    profile: bowshock_core::FinProfile::Square,
                    planform: None,
                };
                let mut c = Case3::sector(mach, body, fins, Tier::Euler, 0);
                c.grid.nk = 6;
                let s = &mut c.grid.sting;
                s.n_up = 10;
                s.n_nose = 16;
                s.n_seg = vec![12];
                s.n_ext = 10;
                s.nj = 24;
                c.grid.fin_chord_cells = Some(16);
                c
            }
        };
        let grid = body_grid3(&case.grid).map_err(err)?;
        let implicit = Numerics3::auto(mach, 0).implicit_for(false);
        let scheme3 = match scheme {
            Scheme::Explicit => Scheme3::Explicit,
            Scheme::Implicit => Scheme3::Implicit(implicit),
        };
        let cfg = Config3 {
            freestream: case.freestream(),
            cfl: 0.5,
            limiter: case.numerics.limiter,
            first_order_iters: 5,
            low_mach: case.numerics.low_mach,
            scheme: scheme3,
        };
        let mut cpu = Solver3::new(grid.clone(), cfg.clone());
        let u0 = cpu.u.clone();
        for _ in 0..30 {
            cpu.step();
        }
        let ctx = luma::default_context().map_err(err)?;
        let prec = if precision == Prec::F32 { Precision::F32 } else { Precision::Extended };
        let mut gpu = GpuSolver3::new(&ctx, &grid, &cfg, prec).map_err(err)?;
        gpu.set_state(&u0).map_err(err)?;
        gpu.run(30).map_err(err)?;
        let u = gpu.state_f64().map_err(err)?;
        let state = u.iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| (0..5).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
        Ok(StateDiff { state, k: 0.0, omega: 0.0 })
    }
}

/// The finless Euler table of the example rocket on the wedge (the fixture's `table-euler` points: each Mach number on its own grid, cold start), wall time
/// and iterations against the 2D table of the fixture. `PERF` line for the ledger.
#[test]
#[ignore = "GPU: run through tools/gpuq (BOWSHOCK_WEDGE_BACKEND is not needed)"]
fn gpu_euler_wedge_table_perf() {
    let cases = facade_cases("G-V4");
    let cases: Vec<&FacadeCase> = cases.iter().filter(|c| c.group == "table-euler").collect();
    assert!(!cases.is_empty(), "the fixture has no table-euler cases");
    // the 2D table ran on the GPU in df32 (the fixture's backend and precision): both precisions are timed
    for (label, precision) in [("df32", Precision::Extended), ("f32", Precision::F32)] {
        let runner = GpuWedge { precision: Some(precision) };
        let (t0, mut iters, mut worst) = (Instant::now(), 0usize, 0.0f64);
        println!(
            "{label}\n| Mach | grid | iterations | stop | cd_total | 2D cd_total | rel diff | 2D iterations | wall s |\n|---|---|---|---|---|---|---|---|---|"
        );
        for c in &cases {
            let t = Instant::now();
            let m = runner.facade_case(c).expect("wedge case");
            let rel = (m.cd_total - c.reference.cd_total) / c.reference.cd_total;
            worst = worst.max(rel.abs());
            iters += m.iterations;
            println!(
                "| {} | - | {} | {} | {:.5} | {:.5} | {:+.2} % | {} | {:.2} |",
                c.mach,
                m.iterations,
                m.stop_reason,
                m.cd_total,
                c.reference.cd_total,
                100.0 * rel,
                c.reference.iterations,
                t.elapsed().as_secs_f64()
            );
        }
        let wall = t0.elapsed().as_secs_f64();
        let (ref_wall, ref_iters): (f64, usize) = (cases.iter().map(|c| c.wall_s).sum(), cases.iter().map(|c| c.reference.iterations).sum());
        println!(
            "TABLE wedge euler {label}: {} points, {wall:.1} s, {iters} iterations; 2D table {ref_wall:.1} s, {ref_iters} iterations; worst |cd_total| difference {:.2} %",
            cases.len(),
            100.0 * worst
        );
        println!(
            "PERF {}",
            json!({"case": "3d-wedge-euler-table", "precision": label, "cells": null, "iters": iters, "us_per_iter": 1e6 * wall / iters as f64, "wall_s": wall})
        );
    }
}
