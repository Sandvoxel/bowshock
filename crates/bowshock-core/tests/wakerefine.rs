//! Wake and boattail grid refinement study (DESIGN3D item 65): which part of the base-wake grid sets the boattail pressure drag and the base pressure of a boattailed body.
//!
//! Models: `fox` (default; NASA TM X-2439 configuration 4, tangent ogive 2 d, cylinder 2 d, 10 degree boattail of 1 d, sting 0.13 d; boattail drag and base `Cp` of the data) and
//! `arcas` (the long Arcas model of G-X4 without fins, `C_A,corr` of the data; the 1 in sting of the test with the token `sting`).
//! `WR_MACHS` (comma list; default the Fox Mach numbers, `0.6,0.8` for `arcas`), `WR_VARIANTS` (comma list, default `base`), tokens joined by `+`:
//! - `fox` / `arcas`, `euler` (Euler tier; the Fox default is RANS, Coarse tier at Re_D 0.5e6), `sting` (Arcas only);
//! - `lvl<N>`, `ref<NN>` (`with_refinement(NN / 10)`), `iters<N>`;
//! - region refinements of the base-wake grid: `seg<k>` the boattail cells, `cyl<k>` the cylinder cells, `wake<k>` the cells behind the base, `gap<k>` the rows between sting and base
//!   radius, `njo<k>` the rows above the body, `dx<k>` the end-cell size at the nose tip, shoulder and base corner, `drw<k>` the first cell height (each divided / multiplied by `k`);
//! - `near<L>_<n>`: the first `n` cells behind the base cover `L` diameters (`BaseGridParams::wake_near`); `rfar<k>`, `xdown<k>`: the outer radius and the outflow distance times `k`;
//! - `twosided`: the gap rows clustered at both ends (sting wall resolved).
//!
//! Prints `WR <variant> M<mach> ...` lines. Needs `BOWSHOCK_WEDGE_BACKEND=gpu` for the GPU.
#[path = "fin_gates/common.rs"]
mod common;
#[path = "fin_gates/rockets.rs"]
mod rockets;

use bowshock_cases::subsonic::fox_tm_x2439 as fox;
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::run3::{wall_samples, Wall};
use bowshock_reference::{arcas, arcas_stab};
use common::{tier, Session, Solved, TierKind};
use rockets::arcas_rocket;

const STING_ARCAS: f64 = 0.5 / 2.25;

fn host(s: &Session) -> &bowshock_cfd::solver3::Solver3 {
    match s {
        Session::Cpu(sol) => sol,
        #[cfg(feature = "gpu")]
        Session::Gpu(r) => &r.host,
    }
}

/// Fox Reynolds number over the diameter at the Mach numbers of the tables (0.44e6 .. 0.68e6; 0.5e6 at Mach 0.608).
fn fox_re_d(m: f64) -> f64 {
    0.5e6 * (0.44 + 0.24 * (m - 0.404) / (0.831 - 0.404)) / 0.5
}

/// The case of a variant: (case, the data's boattail drag or NaN, the data's base `Cp` or NaN, the data's `C_A` or NaN, area factor).
fn case_of(m: f64, variant: &str) -> (Case3, f64, f64, f64, f64) {
    let parts: Vec<&str> = variant.split('+').collect();
    let level = parts.iter().find_map(|p| p.strip_prefix("lvl")).map_or(0, |v| v.parse::<u32>().expect("lvlN"));
    let euler = parts.contains(&"euler");
    let kind = if euler { TierKind::Euler } else { TierKind::Coarse };
    let (mut c, bt, cpb, ca, af) = if parts.contains(&"arcas") {
        let r = arcas_rocket(arcas_stab::LONG_TUBE_LENGTH - arcas::TUBE_LENGTH, arcas::TRANSONIC_RE_PER_M, arcas::TRANSONIC_T0_K);
        let (mut body, _) = r.body.to_internal().expect("body");
        if parts.contains(&"sting") {
            body = body.with_sting(STING_ARCAS);
        }
        let ca = arcas_stab::LONG_TRANSONIC.iter().find(|x| (x.0 - m).abs() < 1e-9).map_or(f64::NAN, |x| x.2);
        let c = Case3::wedge(m, body, level).with_tier(tier(kind, r.flow(m, 1.2)));
        (c, f64::NAN, if m < 0.7 { 0.055 } else { 0.08 }, ca, r.area_factor())
    } else {
        let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 2.0 } }, 2.0).with_boattail(1.0, 10f64.to_radians()).with_sting(fox::STING_RADIUS);
        let k = fox::MACH.iter().position(|x| (x - m).abs() < 1e-9).expect("a Fox Mach number");
        let flow = WallFlow { re_c: fox_re_d(m) / m, mach_max: m, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
        let c = Case3::wedge(m, body, level);
        let c = if euler { c } else { c.with_tier(Tier::Coarse(flow)) };
        (c, [0.0295, 0.0283, 0.0307][k], fox::BASE_CP_OGIVE[k], f64::NAN, 1.0)
    };
    for p in &parts {
        if let Some(v) = p.strip_prefix("ref") {
            c = c.with_refinement(v.parse::<f64>().expect("refNN") / 10.0);
        }
        if let Some(v) = p.strip_prefix("iters") {
            c.numerics.max_iter = v.parse().expect("itersN");
        }
    }
    let nseg = c.body.segments.len();
    for p in &parts {
        let f = |name: &str| p.strip_prefix(name).and_then(|v| v.parse::<f64>().ok());
        if let Some(base) = c.grid.base.as_mut() {
            let even = |n: usize, k: f64| (((n as f64 * k) / 2.0).round() as usize * 2).max(2);
            if let Some(k) = f("seg") {
                base.n_seg[nseg - 1] = even(base.n_seg[nseg - 1], k);
            }
            if let Some(k) = f("cyl") {
                base.n_seg[nseg - 2] = even(base.n_seg[nseg - 2], k);
            }
            if let Some(k) = f("wake") {
                base.n_wake = even(base.n_wake, k);
            }
            if let Some(k) = f("gap") {
                base.n_gap = even(base.n_gap, k);
            }
            if let Some(k) = f("njo") {
                base.nj_out = even(base.nj_out, k);
            }
            if let Some(k) = f("dx") {
                base.dx_tip /= k;
            }
            if let Some(k) = f("drw") {
                base.dr_wall /= k;
            }
            if let Some(k) = f("rfar") {
                base.r_far *= k;
            }
            if let Some(k) = f("xdown") {
                base.x_down *= k;
            }
            if let Some(v) = p.strip_prefix("near") {
                let (l, n) = v.split_once('_').expect("nearL_n");
                base.wake_near = Some((l.parse().expect("nearL"), n.parse().expect("near_n")));
            }
            if *p == "twosided" {
                base.gap_one_sided = false;
            }
        }
    }
    (c, bt, cpb, ca, af)
}

#[test]
#[ignore = "study: wake grid refinement"]
fn wr1_study() {
    let variants: Vec<String> = std::env::var("WR_VARIANTS").unwrap_or_else(|_| "base".into()).split(',').map(|s| s.trim().to_string()).collect();
    for variant in &variants {
        let arcas_model = variant.split('+').any(|t| t == "arcas");
        let machs: Vec<f64> = std::env::var("WR_MACHS")
            .map(|v| v.split(',').map(|x| x.parse().unwrap()).collect())
            .unwrap_or_else(|_| if arcas_model { vec![0.6, 0.8] } else { fox::MACH.to_vec() });
        for &m in &machs {
            let (case, d_bt, d_cpb, d_ca, af) = case_of(m, variant);
            let tag = format!("WR {variant} M{m:.3}");
            let t0 = std::time::Instant::now();
            let mut s = match Session::new(&case) {
                Ok(s) => s,
                Err(e) => {
                    println!("{tag} FAILED build: {e}");
                    continue;
                }
            };
            let sol: Solved = match s.run(&case, false) {
                Ok(x) => x,
                Err(e) => {
                    println!("{tag} FAILED run: {e}");
                    continue;
                }
            };
            let h = host(&s);
            let g = &h.grid;
            let p = &sol.parts;
            let bt = *p.aft.last().unwrap();
            let v = af * (p.total - p.base - p.friction.base);
            println!(
                "{tag} cells {}x{}={} iters {} stop {:?} t {:.1}s | nose {:+.4} boattail {:+.4} (data {d_bt:.4}) base {:+.4} cp_base {:+.4} (data {d_cpb:.4}) | v {v:.4} (data {d_ca:.4}, {:+.1}%) friction {:.4} y+ {:.2}",
                g.ni,
                g.nj,
                g.ni * g.nj,
                sol.point.iterations,
                sol.point.stop_reason,
                t0.elapsed().as_secs_f64(),
                p.nose,
                bt,
                p.base,
                sol.point.cp_base,
                100.0 * (v / d_ca - 1.0),
                p.friction.total,
                sol.point.y_plus_max
            );
            // the wall Cp over the last body segment, extrapolated wall pressure, in 12 bins
            let p_inf = h.cfg.freestream[4];
            let q = 0.5 * m * m;
            let x_bt = case.body.station(case.body.segments.len() - 1).0;
            let x_end = case.body.x_base();
            let mut cps: Vec<(f64, f64)> = wall_samples(g, &h.u)
                .into_iter()
                .filter(|w| matches!(w.class, Wall::Body(_)) && w.face.c[0] > x_bt - 0.3)
                .map(|w| (w.face.c[0], (w.pw - p_inf) / q))
                .collect();
            cps.sort_by(|a, b| a.0.total_cmp(&b.0));
            let (mut cp_min, mut x_min) = (f64::INFINITY, 0.0);
            for &(x, c) in &cps {
                if c < cp_min {
                    (cp_min, x_min) = (c, x);
                }
            }
            let at = |x: f64| -> f64 {
                let i = cps.partition_point(|c| c.0 < x).clamp(1, cps.len() - 1);
                let (a, b) = (cps[i - 1], cps[i]);
                a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0)
            };
            let probes: Vec<String> = [0.0, 0.25, 0.5, 0.75, 1.0].iter().map(|f| format!("{:+.3}", at(x_bt + f * (x_end - x_bt) * 0.999))).collect();
            println!("{tag} boattail wall Cp at 0 / 25 / 50 / 75 / 100 %: {} ; min {cp_min:+.3} at x {x_min:.3} (x_bt {x_bt:.3}, x_base {x_end:.3})", probes.join(" "));
            // grid sizes: axial cell at the shoulder, the base, 1 / 2 / 5 d behind it; the radial cell at 0.25 / 0.5 / 1 d above the axis row at those columns
            let xc: Vec<f64> = (0..g.ni).map(|i| g.xc[g.cell(i, 0, 0)][0]).collect();
            let dx_at = |x: f64| -> f64 {
                let i = xc.partition_point(|&c| c < x).clamp(1, g.ni - 1);
                xc[i] - xc[i - 1]
            };
            let col = |x: f64| xc.partition_point(|&c| c < x).clamp(0, g.ni - 1);
            let row_sizes = |x: f64| -> String {
                let i = col(x);
                let ycen = |j: usize| {
                    let c = g.xc[g.cell(i, j, 0)];
                    c[1].hypot(c[2])
                };
                let jw = (0..g.nj).find(|&j| !g.is_solid(i, j, 0)).unwrap();
                let rw = ycen(jw);
                let mut out = String::new();
                for off in [0.1, 0.25, 0.5, 1.0] {
                    let j = (jw..g.nj - 1).find(|&j| ycen(j) - rw >= off).unwrap_or(g.nj - 2);
                    out += &format!(" dr({off})={:.4}", ycen(j + 1) - ycen(j));
                }
                out
            };
            println!(
                "{tag} grid dx: shoulder {:.4} base- {:.4} base+0 {:.4} +1d {:.4} +2d {:.4} +5d {:.4}; first fluid-row size at base+1d:{}",
                dx_at(x_bt),
                dx_at(x_end - 0.01),
                dx_at(x_end + 0.01),
                dx_at(x_end + 1.0),
                dx_at(x_end + 2.0),
                dx_at(x_end + 5.0),
                row_sizes(x_end + 1.0)
            );
            let _ = d_ca;
        }
    }
}

/// The grids of a few variants build (CPU, no solve).
#[test]
fn variants_build() {
    for v in ["fox", "fox+euler", "arcas", "arcas+sting+twosided", "fox+seg2+wake2+gap2+njo2+dx2+drw2", "fox+near3_60+rfar1.5+xdown2"] {
        let (c, ..) = case_of(0.608, v);
        let s = c.build_solver_cached().unwrap_or_else(|e| panic!("{v}: {e}"));
        println!("{v}: {}x{}", s.grid.ni, s.grid.nj);
    }
}
