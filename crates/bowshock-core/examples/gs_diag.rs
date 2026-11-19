//! G-S diagnosis (DESIGN3D item 52): one side of the sector-vs-half comparison with its stop rule under control, the drag parts and the history of CA, and a node-by-node
//! comparison of the sector grid with the half domain's.
//!
//! ```text
//! gs_diag grid <n> [chord|-] [across]                     CPU only: the two grids of the G-S gate (Coarse tier) compared
//! gs_diag run <sector|half> <n> <mach> [--chord C] [--across A] [--iters N] [--drift-tol X] [--sub S] [--cfl C] [--ext]
//! ```
//! `--iters N` runs exactly `N` iterations (every stop rule off); `--drift-tol X` keeps the default rules with that drift tolerance; `--ext` is the df32 precision. GPU runs
//! need `BOWSHOCK_WEDGE_BACKEND=gpu`.
#[path = "../tests/fin_gates/common.rs"]
mod common;

use bowshock_cfd::grid3::Grid3;
use bowshock_core::engine::case3::{Case3, WallFlow};
use bowshock_core::engine::grid3::{body_grid3, Domain3, FinPhase, Fins3};
use bowshock_core::engine::run3::{wall_samples, Wall};
use common::{tier, Session, TierKind};
use std::f64::consts::PI;

fn re_c_sea_level(d: f64) -> f64 {
    1.225 * 340.294 * d / 1.7894e-5
}

fn case_of(domain: &str, n: u32, mach: f64, chord: Option<usize>, across: usize) -> Case3 {
    let fins = Fins3 { count: n, ..Case3::anf_fins() };
    let flow = WallFlow { re_c: re_c_sea_level(0.03), mach_max: mach.max(2.0), ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    let mut c = if domain == "sector" {
        Case3::sector(mach, Case3::anf_body(), fins, tier(TierKind::Coarse, flow), 0)
    } else {
        let mut h = Case3::base_half(mach, 0.0, Case3::anf_body(), Some(fins), tier(TierKind::Coarse, flow), 0);
        h.grid.domain = Domain3::Half { phase: if n % 2 == 0 { FinPhase::Between } else { FinPhase::InPlane } };
        h
    };
    if let Some(cc) = chord {
        c.grid.fin_chord_cells = Some(cc);
    }
    c.with_fin_cells_across(across)
}

fn node(g: &Grid3, i: usize, j: usize, k: usize) -> [f64; 3] {
    let id = (k * (g.nj + 1) + j) * (g.ni + 1) + i;
    [g.x[id], g.y[id], g.z[id]]
}

/// Azimuth of a node, from `+z` towards `+y`, and its radius.
fn polar(p: [f64; 3]) -> (f64, f64) {
    (p[1].atan2(p[2]), p[1].hypot(p[2]))
}

fn fin_boxes(g: &Grid3) -> Vec<(std::ops::Range<usize>, std::ops::Range<usize>, std::ops::Range<usize>)> {
    g.solid.iter().filter(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(bowshock_cfd::grid3::Part::Fin(_)))).map(|b| (b.i.clone(), b.j.clone(), b.k.clone())).collect()
}

fn describe(label: &str, g: &Grid3) {
    println!("{label}: ni {} nj {} nk {} cells {} theta_span {:.5}", g.ni, g.nj, g.nk, g.ni * g.nj * g.nk, g.theta_span);
    for (i, j, k) in fin_boxes(g) {
        println!("  fin box i {i:?} ({} chord cells) j {j:?} ({} span cells) k {k:?}", i.len(), j.len());
    }
}

fn grid_compare(n: u32, chord: Option<usize>, across: usize) {
    let (cs, ch) = (case_of("sector", n, 0.8, chord, across), case_of("half", n, 0.8, chord, across));
    let (gs, gh) = (body_grid3(&cs.grid).expect("sector grid"), body_grid3(&ch.grid).expect("half grid"));
    describe("sector", &gs);
    describe("half  ", &gh);
    let mirror = n % 2 == 0;
    let th_of = |g: &Grid3, i: usize, j: usize, k: usize| polar(node(g, i, j, k)).0;
    // meridian nodes: the axial and radial coordinates of the sector's cells against the half domain's, k for k matched by azimuth (the sector's theta, or pi/n - theta when the
    // half domain's fin at pi/n is the sector's mirror image)
    let (mut dx, mut dr, mut unmatched, mut worst_th) = (0f64, 0f64, 0usize, 0f64);
    let mut checked = 0usize;
    for j in 0..=gs.nj {
        for i in 0..=gs.ni {
            for ks in 0..=gs.nk {
                let ths = th_of(&gs, i, j, ks);
                let target = if mirror { PI / f64::from(n) - ths } else { ths };
                let found = (0..=gh.nk).map(|kh| (kh, (th_of(&gh, i, j, kh) - target).abs())).min_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
                worst_th = worst_th.max(found.1);
                if found.1 > 1e-6 {
                    unmatched += 1;
                    continue;
                }
                let (ps, ph) = (node(&gs, i, j, ks), node(&gh, i, j, found.0));
                dx = dx.max((ps[0] - ph[0]).abs());
                dr = dr.max((polar(ps).1 - polar(ph).1).abs());
                checked += 1;
            }
        }
    }
    println!("node compare (mirror {mirror}): {checked} sector nodes matched by azimuth, {unmatched} unmatched (worst azimuth miss {worst_th:.2e}); max |dx| {dx:.3e}, max |dr| {dr:.3e}");
    // the fin chord spacing: first cells at the LE and TE, in units of the chord, from the root row of the first fin box of each grid
    for (label, g) in [("sector", &gs), ("half", &gh)] {
        let Some((i, j, k)) = fin_boxes(g).into_iter().next() else { continue };
        let xs: Vec<f64> = (i.start..=i.end).map(|ii| node(g, ii, j.start, k.start)[0]).collect();
        let dxs: Vec<f64> = xs.windows(2).map(|w| w[1] - w[0]).collect();
        let (mn, mx) = (dxs.iter().copied().fold(f64::INFINITY, f64::min), dxs.iter().copied().fold(0.0, f64::max));
        println!(
            "{label}: fin chord cells {} from x {:.5} to {:.5}; dx first {:.5} {:.5} {:.5}, last {:.5} {:.5} {:.5}; min {mn:.5} max {mx:.5}",
            dxs.len(),
            xs[0],
            xs[xs.len() - 1],
            dxs[0],
            dxs[1],
            dxs[2],
            dxs[dxs.len() - 3],
            dxs[dxs.len() - 2],
            dxs[dxs.len() - 1]
        );
    }
    // the azimuthal gaps at the root row of the mid-chord column: cell angles of the sector, then of the half domain
    for (label, g) in [("sector", &gs), ("half", &gh)] {
        let (i, j) = (fin_boxes(g)[0].0.start + fin_boxes(g)[0].0.len() / 2, 0);
        let solid: Vec<usize> = (0..g.nk).filter(|&k| g.is_solid(i, j, k)).collect();
        let th: Vec<f64> = (0..=g.nk).map(|k| th_of(g, i, j, k).to_degrees()).collect();
        let cells: Vec<String> = th.windows(2).map(|w| format!("{:.3}", w[1] - w[0])).collect();
        println!("{label}: nk {} solid k {solid:?}; cell angles [deg] at mid-chord root: {}", g.nk, cells.join(" "));
    }
}

fn arg<T: std::str::FromStr>(args: &[String], name: &str) -> Option<T> {
    args.iter().position(|a| a == name).and_then(|p| args.get(p + 1)).map(|v| v.parse().unwrap_or_else(|_| panic!("{name}: bad value {v}")))
}

fn run(args: &[String]) {
    let (domain, n, mach): (&str, u32, f64) = (&args[0], args[1].parse().unwrap(), args[2].parse().unwrap());
    let mut case = case_of(domain, n, mach, arg(args, "--chord"), arg(args, "--across").unwrap_or(1));
    if let Some(it) = arg::<usize>(args, "--iters") {
        let nu = &mut case.numerics;
        (nu.max_iter, nu.drift_block, nu.mean_window, nu.incidence_window, nu.rel_tol) = (it, 0.0, 0, 0, 0.0);
    }
    if let Some(t) = arg::<f64>(args, "--drift-tol") {
        case.numerics.drift_tol = t;
    }
    if let Some(s) = arg::<usize>(args, "--sub") {
        case.numerics.implicit.sub_iters = s;
    }
    if let Some(c) = arg::<f64>(args, "--cfl") {
        case.numerics.implicit.cfl_max = c;
    }
    if args.iter().any(|a| a == "--ext") {
        std::env::set_var("BOWSHOCK_WEDGE_PRECISION", "ext");
    }
    println!(
        "case {domain} N{n} M{mach} chord {:?} cfl_max {} sub {} drift_tol {} drift_block {} max_iter {}",
        case.grid.fin_chord_cells, case.numerics.implicit.cfl_max, case.numerics.implicit.sub_iters, case.numerics.drift_tol, case.numerics.drift_block, case.numerics.max_iter
    );
    let t = std::time::Instant::now();
    let mut s = Session::new(&case).expect("session");
    let r = s.run(&case, false).expect("run");
    let p = &r.parts;
    println!(
        "RESULT {domain} N{n} M{mach} chord {:?}: {} it, {:?}; total {:.6} (CA {:.6}) nose {:.6} aft {:.6} base {:.6} fin_sides {:.6} fin_le {:.6} fin_te {:.6} fin_tips {:.6}; friction total {:.6} (nose {:.6} aft {:.6} fin_sides {:.6}); wall {:.0}s",
        case.grid.fin_chord_cells,
        r.point.iterations,
        r.point.stop_reason,
        p.total,
        r.point.ca,
        p.nose,
        p.aft.iter().sum::<f64>(),
        p.base,
        p.fin_sides,
        p.fin_le,
        p.fin_te,
        p.fin_tips,
        p.friction.total,
        p.friction.nose,
        p.friction.aft.iter().sum::<f64>(),
        p.friction.fin_sides,
        t.elapsed().as_secs_f64()
    );
    per_fin(&case, n, &s);
    println!("cfl_events {:?}", r.point.cfl_events.iter().map(|e| (e.iteration, e.cfl_after)).collect::<Vec<_>>());
    let h = &r.point.cn_history;
    let line: Vec<String> = h.iter().filter(|(it, _)| it % 250 == 0).map(|(it, c)| format!("{it}:{c:.5}")).collect();
    println!("CA history (every 250 it): {}", line.join(" "));
    for w in [500usize, 1000, 2000] {
        let tail: Vec<f64> = h.iter().rev().take(w / 50).map(|x| x.1).collect();
        if tail.len() == w / 50 {
            let mean = tail.iter().sum::<f64>() / tail.len() as f64;
            let (lo, hi) = tail.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, &c| (a.0.min(c), a.1.max(c)));
            println!("CA last {w} it: mean {mean:.6} min {lo:.6} max {hi:.6} half-range {:.3} %", 50.0 * (hi - lo) / mean);
        }
    }
    if let Some(w) = r.point.window {
        println!("window mean CA {:.6} oscillation {:.3} %", w.ca, 100.0 * w.oscillation);
    }
}

/// The pressure drag of the fin faces by fin (the fin nearest in azimuth), each as `N x` the force of one whole fin on `pi/4`: the half fin on a symmetry plane counts twice,
/// a whole fin once, so the entries of a sector and of a half domain are comparable fin by fin and their mean is the `fin_*` part. The wall pressures are those of the device
/// state downloaded at the end of the run (the instantaneous state, not an average).
fn per_fin(case: &Case3, n: u32, s: &Session) {
    #[cfg(feature = "gpu")]
    if let Session::Gpu(r) = s {
        let g = &r.host.grid;
        let angles: Vec<f64> = match case.grid.domain {
            Domain3::Sector => vec![0.0],
            Domain3::Half { phase } => {
                let phi0 = if phase == FinPhase::Between { PI / f64::from(n) } else { 0.0 };
                (0..n).map(|m| phi0 + 2.0 * PI * f64::from(m) / f64::from(n)).filter(|&t| t <= PI + 1e-9).collect()
            }
            Domain3::Wedge { .. } => return,
        };
        let q = 0.5 * case.mach * case.mach * std::f64::consts::FRAC_PI_4;
        let p_inf = r.host.cfg.freestream[4];
        let mut acc = vec![[0.0f64; 3]; angles.len()];
        // the mean trailing-edge base pressure (Cp) on the two sides of each fin's mid-plane (azimuth below / above the fin's): equal when the wake is symmetric about the fin
        let mut sides = vec![[(0.0f64, 0.0f64); 2]; angles.len()];
        for w in wall_samples(g, &r.host.u) {
            let slot = match w.class {
                Wall::FinLe => 0,
                Wall::FinTe => 1,
                Wall::FinSide => 2,
                _ => continue,
            };
            let th = w.face.c[1].atan2(w.face.c[2]);
            let f = (0..angles.len()).min_by(|&a, &b| (angles[a] - th).abs().total_cmp(&(angles[b] - th).abs())).unwrap();
            acc[f][slot] += (w.pw - p_inf) * w.face.area * w.sign * w.face.n[0];
            if slot == 1 {
                let side = usize::from(th > angles[f]);
                sides[f][side].0 += (w.pw - p_inf) * w.face.area;
                sides[f][side].1 += w.face.area;
            }
        }
        for (f, a) in acc.iter().enumerate() {
            let on_plane = angles[f] < 1e-9 || (PI - angles[f]).abs() < 1e-9;
            let k = if on_plane { 2.0 } else { 1.0 } * f64::from(n) / q;
            let cp = |(a, b): (f64, f64)| if b > 0.0 { a / b / (0.5 * case.mach * case.mach) } else { f64::NAN };
            println!(
                "FIN theta {:6.2} deg ({}): N x per-fin CD  le {:.6} te {:.6} sides {:.6}; TE face Cp below / above the fin plane {:.4} / {:.4}",
                angles[f].to_degrees(),
                if on_plane { "half fin on a symmetry plane" } else { "whole fin" },
                k * a[0],
                k * a[1],
                k * a[2],
                cp(sides[f][0]),
                cp(sides[f][1])
            );
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("grid") => grid_compare(args[1].parse().unwrap(), args.get(2).filter(|c| *c != "-").map(|c| c.parse().unwrap()), args.get(3).map_or(1, |a| a.parse().unwrap())),
        Some("run") => run(&args[1..]),
        _ => eprintln!("usage: gs_diag grid <n> [chord] | run <sector|half> <n> <mach> [--chord C] [--across A] [--iters N] [--drift-tol X] [--sub S] [--cfl C] [--ext]"),
    }
}
