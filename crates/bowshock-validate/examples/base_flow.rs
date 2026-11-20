//! `base_flow`: the Herrin & Dutton (1994) supersonic cylinder base flow (Mach 2.46, `D = 63.5 mm`, `Re_D = 3.3e6`, adiabatic wall, `T_inf = 131 K`, measured mean base pressure
//! coefficient `Cp_b = -0.102`, rear stagnation point `x/R = 2.65` on the axis, maximum reverse velocity 0.27 `U_inf` at `x/R = 1.5`) on the single-cell wedge of the 3D solver
//! with the finless base-wake grid (no sting: the wake reaches the axis), the Coarse tier, `GpuRunner3`, to the case's stop rule (DESIGN3D item 54).
//!
//! ```text
//! cargo run --release -p bowshock-validate --example base_flow -- [--comp off|sarkar|wilcox|zeman|wilcox-shear] [--mach 2.46] [--half-angle DEG] [--len D] [--refine S]
//!     [--re-d 3.3e6] [--max-iter N] [--precision f32|df32] [--no-control]
//! ```
//!
//! Lines: `BASEFLOW` (grid, iterations, stop, mean base `Cp` against the measured -0.102, drag parts), `BASELAYER` (the approach boundary layer 1 mm ahead of the base corner: `delta99`,
//! `delta*`, `theta` in mm against the measured 3.2 (read from the profile), 0.96, 0.26), `BASEPROFILE` (area-averaged base Cp by radius), `BASEAXIS` (the axis: rear stagnation
//! point `x/R`, maximum reverse velocity and where).

use bowshock_cfd::gas3::cons_to_prim3;
use bowshock_cfd::gpu::Precision;
use bowshock_cfd::grid3::Part;
use bowshock_cfd::sst::Compressibility;
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::run3::{drag_parts_on, run3_on, wall_samples, Wall};

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let num = |name: &str, default: f64| value(name).map_or(Ok(default), |v| v.parse::<f64>().map_err(|e| format!("{name}: {e}")));
    let mach = num("--mach", 2.46)?;
    let re_d = num("--re-d", 3.3e6)?;
    let half_angle = num("--half-angle", 7.0)?;
    let len = num("--len", 0.5)?;
    let refine = num("--refine", 1.0)?;
    let comp = match value("--comp") {
        None => Compressibility::Off,
        Some(c) => Compressibility::parse(&c).ok_or_else(|| format!("--comp: unknown `{c}`"))?,
    };
    // `rho c d / mu` from `Re_D = rho U d / mu`; Sutherland over the free-stream temperature of the Herrin & Dutton nozzle exit (131 K)
    let flow = WallFlow { re_c: re_d / mach, mach_max: mach, ks_body: 0.0, ks_fin: 0.0, sutherland: 110.4 / 131.0 };
    let body = BaseBody::cylinder(Body { nose: Nose::Cone { half_angle: half_angle.to_radians() } }, len);
    let r_body = 0.5;
    let mut case = Case3::wedge(mach, body, 0).with_tier(Tier::Coarse(flow)).with_compressibility(comp);
    if refine != 1.0 {
        case = case.with_refinement(refine);
    }
    if let Some(n) = value("--max-iter") {
        case.numerics.max_iter = n.parse().map_err(|e| format!("--max-iter: {e}"))?;
    }
    if args.iter().any(|a| a == "--no-control") {
        case.numerics.cfl_control = false;
    }
    let precision = match value("--precision").as_deref() {
        None | Some("f32") => Precision::F32,
        Some("df32") => Precision::Extended,
        Some(other) => return Err(format!("--precision f32|df32, not {other}")),
    };
    let t = std::time::Instant::now();
    // `--cpu`: the f64 CPU solver (no GPU queue); else `GpuRunner3`
    let (p, d, g, u, fs, turb) = if args.iter().any(|a| a == "--cpu") {
        let mut s = case.build_solver().map_err(|e| e.to_string())?;
        let p = run3_on(&case, &mut s, false);
        let d = drag_parts_on(&case, &s);
        let turb = s.rans().map(|r| r.turb.clone()).unwrap_or_default();
        (p, d, s.grid.clone(), s.u.clone(), s.cfg.freestream, turb)
    } else {
        let mut run = GpuRunner3::new(&case, precision).map_err(|e| e.to_string())?;
        let p = run.run_on(&case, false).map_err(|e| e.to_string())?;
        let d = run.drag_parts(&case).map_err(|e| e.to_string())?;
        let u = run.gpu.state_f64().map_err(|e| e.to_string())?;
        let turb = run.gpu.turb_state().map_err(|e| e.to_string())?.map(|t| t.0).unwrap_or_default();
        (p, d, run.host.grid.clone(), u, run.host.cfg.freestream, turb)
    };
    let wall = t.elapsed().as_secs_f64();
    let g = &g;
    // the turbulent Mach number `M_t^2 = 2 k / a^2` of the fluid cells: where the compressibility corrections can act (Wilcox from `M_t^2 > 1/16`)
    let (mut mt2_max, mut at, mut above) = (0.0f64, [0.0; 2], 0usize);
    for c in 0..u.len().min(turb.len()) {
        if g.part[c] != bowshock_cfd::grid3::PART_FLUID {
            continue;
        }
        let w = cons_to_prim3(&u[c]);
        let mt2 = 2.0 * turb[c][0] / (bowshock_cfd::gas3::GAMMA * w[4] / w[0]);
        above += usize::from(mt2 > 0.0625);
        if mt2 > mt2_max {
            (mt2_max, at) =
                (mt2, [(g.xc[c][0] - case.body.station(case.body.segments.len()).0) / 0.5, (g.xc[c][1] * g.xc[c][1] + g.xc[c][2] * g.xc[c][2]).sqrt() / 0.5]);
        }
    }
    println!("BASEMT max M_t^2 {mt2_max:.4} at x/R {:.2} (from the base) r/R {:.2}; {above} fluid cells above the Wilcox threshold 1/16", at[0], at[1]);
    println!(
        "BASEFLOW comp {comp:?} M{mach} Re_D {re_d:.3e} nose {half_angle} deg len {len} D refine {refine}: {}x{}x{} = {} cells, {} iterations ({:?}), {wall:.1} s; Cp_base {:.4} (measured -0.102, {:+.1} %); CD {:.5} = friction {:.5} + nose {:.5} + afterbody {:.5} + base {:.5}",
        g.ni,
        g.nj,
        g.nk,
        g.ni * g.nj * g.nk,
        p.iterations,
        p.stop_reason,
        p.cp_base,
        100.0 * (p.cp_base / -0.102 - 1.0),
        d.total,
        d.body_friction(),
        d.nose,
        d.aft.iter().sum::<f64>(),
        d.base
    );
    let (rho_inf, u_inf, p_inf) = (fs[0], mach, fs[4]);
    // the base profile: wall pressure of the base faces by radius (Cp as `base_cp`)
    let q = 0.5 * mach * mach;
    let mut prof: Vec<(f64, f64)> = wall_samples(g, &u)
        .into_iter()
        .filter(|w| w.class == Wall::Base)
        .map(|w| ((w.face.c[1] * w.face.c[1] + w.face.c[2] * w.face.c[2]).sqrt() / r_body, (w.pw - p_inf) / q))
        .collect();
    prof.sort_by(|a, b| a.0.total_cmp(&b.0));
    let line: Vec<String> = prof.iter().map(|(r, cp)| format!("{r:.2}:{cp:.3}")).collect();
    println!("BASEPROFILE r/R:Cp {}", line.join(" "));
    // the approach boundary layer: the column of cells 1 mm (D = 63.5 mm) ahead of the base plane, from the wall to the free stream
    let x_base = case.body.station(case.body.segments.len()).0;
    let x_probe = x_base - 1.0 / 63.5;
    let (mut best, mut col) = (f64::INFINITY, 0usize);
    for blk in g.solid.iter().filter(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(Part::Body(_)))) {
        for i in blk.i.clone() {
            let c = g.cell(i, blk.j.end, 0);
            if (g.xc[c][0] - x_probe).abs() < best {
                best = (g.xc[c][0] - x_probe).abs();
                col = c % g.ni + 1_000_000 * blk.j.end;
            }
        }
    }
    let (i0, j0) = (col % 1_000_000, col / 1_000_000);
    let mut cells = Vec::new();
    for j in j0..g.nj {
        let c = g.cell(i0, j, 0);
        let w = cons_to_prim3(&u[c]);
        let y = (g.xc[c][1] * g.xc[c][1] + g.xc[c][2] * g.xc[c][2]).sqrt() - r_body;
        cells.push((y, w[0], w[1]));
        if y > 0.2 {
            break;
        }
    }
    let ue = cells.iter().map(|c| c.2).fold(0.0f64, f64::max);
    let rho_e = cells.iter().find(|c| c.2 >= 0.99 * ue).map_or(rho_inf, |c| c.1);
    let (mut dstar, mut theta, mut delta) = (0.0, 0.0, 0.0);
    let mut y_prev = 0.0;
    for &(y, rho, uu) in &cells {
        let dy = y - y_prev;
        if y_prev == 0.0 || uu <= ue {
            dstar += (1.0 - rho * uu / (rho_e * ue)) * dy;
            theta += rho * uu / (rho_e * ue) * (1.0 - uu / ue) * dy;
        }
        if uu < 0.99 * ue {
            delta = y;
        }
        y_prev = y;
    }
    let mm = 63.5;
    println!(
        "BASELAYER x {:.4} ({:.2} mm ahead of the base): delta99 {:.2} mm, delta* {:.3} mm, theta {:.3} mm (measured: delta* 0.96, theta 0.26), edge Mach speed {:.3} (U_inf {u_inf:.3}), first cell y {:.2e} mm",
        g.xc[g.cell(i0, j0, 0)][0],
        (x_base - g.xc[g.cell(i0, j0, 0)][0]) * mm,
        delta * mm,
        dstar * mm,
        theta * mm,
        ue,
        cells[0].0 * mm
    );
    // the axis: `u` on the cells of the first row `j = 0` behind the base, rear stagnation point and the maximum reverse velocity
    let mut axis: Vec<(f64, f64)> = (0..g.ni)
        .map(|i| {
            let c = g.cell(i, 0, 0);
            (g.xc[c][0], cons_to_prim3(&u[c])[1] / u_inf)
        })
        .filter(|&(x, _)| x > x_base)
        .collect();
    axis.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (xmin, umin) = axis.iter().copied().fold((0.0, 0.0), |m, c| if c.1 < m.1 { c } else { m });
    let stag = axis.windows(2).find(|w| w[0].1 < 0.0 && w[1].1 >= 0.0).map(|w| w[0].0 + (w[1].0 - w[0].0) * (-w[0].1) / (w[1].1 - w[0].1));
    println!(
        "BASEAXIS: rear stagnation x/R {} (measured 2.65), maximum reverse velocity {:.3} U_inf at x/R {:.2} (measured 0.27 at 1.5)",
        stag.map_or("none".to_string(), |x| format!("{:.2}", (x - x_base) / r_body)),
        -umin,
        (xmin - x_base) / r_body
    );
    Ok(())
}
