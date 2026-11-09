//! Convergence diagnostics: run a cone case for a fixed number of iterations (no stopping criterion)
//! and dump per-cell residual statistics over a trailing window to `out/diag/`.
//!
//! `cargo run --release --example diag -- <theta_deg> <ni> <nj> <minmod|vanalbada> <iters> [window]`
use bowshock_cfd::cone::{build_solver, ConeCase};
use bowshock_cfd::flux::Limiter;
use bowshock_cfd::taylor_maccoll::solve_cone;
use std::io::Write;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let theta: f64 = a[0].parse().unwrap();
    let ni: usize = a[1].parse().unwrap();
    let nj: usize = a[2].parse().unwrap();
    let lim = match a[3].as_str() { "vanalbada" => Limiter::VanAlbada, "first" => Limiter::FirstOrder, _ => Limiter::Minmod };
    let iters: usize = a[4].parse().unwrap();
    let win: usize = a.get(5).map(|s| s.parse().unwrap()).unwrap_or(400);
    let tm = solve_cone(2.0, theta.to_radians()).unwrap();
    let mut case = ConeCase::new(2.0, theta, ni, nj);
    case.limiter = lim;
    if let Ok(c) = std::env::var("FO") { case.first_order_iters = c.parse().unwrap(); }
    if let Ok(c) = std::env::var("CFL") { case.cfl = c.parse().unwrap(); }
    let mut s = build_solver(&case, tm.beta);
    let n = ni * nj;
    let (mut mean, mut sq, mut mx) = (vec![0.0; n], vec![0.0; n], vec![0.0f64; n]);
    let mut cnt = 0usize;
    let mut loc = Vec::new();
    for it in 0..iters {
        let r = s.step();
        if let Ok(pr) = std::env::var("PROBE") {
            let v: Vec<usize> = pr.split(',').map(|x| x.parse().unwrap()).collect();
            if it + v[3] >= iters && it % v[4] == 0 {
                let line: Vec<String> = (v[1]..v[2]).map(|j| format!("{:.4}", s.prim(v[0], j)[3] / s.cfg.freestream[3])).collect();
                println!("P it {it}: {}", line.join(" "));
            }
        }
        if it % 1000 == 999 { println!("  it {} L2 {:.3e}", it + 1, r); }
        if it + win >= iters {
            for c in 0..n {
                let v = s.res[c][0];
                mean[c] += v;
                sq[c] += v * v;
                mx[c] = mx[c].max(v.abs());
            }
            cnt += 1;
        }
        // location of max |res| (of the state at the start of this step: res holds stage-2 residual; fine)
        if it % 200 == 0 || it + 20 >= iters {
            let (mut best, mut bc) = (0.0, 0);
            for c in 0..n {
                if s.res[c][0].abs() > best {
                    best = s.res[c][0].abs();
                    bc = c;
                }
            }
            loc.push((it, r, bc % ni, bc / ni, best));
        }
    }
    let tag = format!("{}deg_{}x{}_{}", theta as i32, ni, nj, a[3]);
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../out/diag");
    std::fs::create_dir_all(&dir).unwrap();
    let mut f = std::fs::File::create(dir.join(format!("resmap_{tag}.csv"))).unwrap();
    writeln!(f, "i,j,x,r,p_over_pinf,mean_res_rho,rms_res_rho,max_abs_res_rho").unwrap();
    let pinf = s.cfg.freestream[3];
    let mut tot = 0.0;
    let mut regions = std::collections::BTreeMap::<&str, f64>::new();
    let mut osc = 0.0; // sum of (rms^2 - mean^2) : oscillatory part
    let mut stdy = 0.0;
    for j in 0..nj {
        for i in 0..ni {
            let c = j * ni + i;
            let m = mean[c] / cnt as f64;
            let q = (sq[c] / cnt as f64).sqrt();
            writeln!(f, "{i},{j},{:.6},{:.6},{:.6},{:e},{:e},{:e}", s.grid.xc[c], s.grid.rc[c], s.pressure(i, j) / pinf, m, q, mx[c]).unwrap();
            let e = q * q;
            tot += e;
            stdy += m * m;
            osc += q * q - m * m;
            let x = s.grid.xc[c];
            let rsh = x.max(0.0) * tm.beta.tan(); // shock radius
            let key = if x < 0.0 { "axis (x<0)" }
                else if i + 3 >= ni { "outflow (last 3 cols)" }
                else if j < 3 { "wall (j<3)" }
                else if x < 0.1 { "tip (x<0.1)" }
                else if (s.grid.rc[c] - rsh).abs() < 0.05 { "shock band" }
                else if s.grid.rc[c] > rsh + 0.05 { "freestream side" }
                else { "shock layer" };
            *regions.entry(key).or_default() += e;
        }
    }
    println!("{tag}: window {cnt}; sum rms^2 = {tot:.3e}; steady part {:.1}%, oscillatory part {:.1}%", 100.0 * stdy / tot, 100.0 * osc / tot);
    for (k, v) in &regions {
        println!("  {k:24} {:5.1}% of sum rms^2", 100.0 * v / tot);
    }
    println!("  max |res| location history (iter, L2, i, j, max):");
    for l in loc.iter().rev().take(14).rev() {
        println!("    {:6} {:.3e}  i={:4} j={:4} x={:.3} r={:.3} max={:.3e}", l.0, l.1, l.2, l.3, s.grid.xc[l.3 * ni + l.2], s.grid.rc[l.3 * ni + l.2], l.4);
    }
}
