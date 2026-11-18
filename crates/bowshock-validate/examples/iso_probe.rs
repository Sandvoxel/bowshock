//! Temporary probe of the start-up transient of the cold-wall ANF sector: runs `--iters` iterations in chunks of `--every` and prints where the state is most extreme.
use bowshock_cfd::gpu::Precision;
use bowshock_cfd::sst::WallThermal;
use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::gpu3::GpuRunner3;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let mach: f64 = value("--mach").ok_or("--mach")?.parse().map_err(|e| format!("{e}"))?;
    let iters: usize = value("--iters").map_or(Ok(300), |v| v.parse().map_err(|e| format!("{e}")))?;
    let every: usize = value("--every").map_or(Ok(10), |v| v.parse().map_err(|e| format!("{e}")))?;
    let flow = WallFlow { re_c: 1.225 * 340.294 * 0.03 / 1.7894e-5, mach_max: 3.147, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    let mut case = if args.iter().any(|a| a == "--wedge") {
        Case3::wedge(mach, Case3::anf_body(), 0).with_tier(Tier::Coarse(flow))
    } else {
        Case3::sector(mach, Case3::anf_body(), Case3::anf_fins(), Tier::Coarse(flow), 0)
    };
    if let Some(t) = value("--tw") {
        case = case.with_wall_thermal(WallThermal::Isothermal(t.parse().map_err(|e| format!("{e}"))?));
    }
    if let Some(c) = value("--cfl-max") {
        case.numerics.implicit.cfl_max = c.parse().map_err(|e| format!("{e}"))?;
    }
    let mut run = GpuRunner3::new(&case, Precision::F32).map_err(|e| e.to_string())?;
    let g = run.host.grid.clone();
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let dwall = run.host.rans().map(|r| r.dwall.clone()).unwrap_or_default();
    let mut prev = run.gpu.state_f64().map_err(|e| e.to_string())?;
    let mut done = 0;
    while done < iters {
        let h = run.gpu.run(every).map_err(|e| e.to_string())?;
        done += every;
        let u = run.gpu.state_f64().map_err(|e| e.to_string())?;
        let mut worst = (0.0f64, 0usize);
        let (mut tmax, mut tmax_c, mut pmin, mut pmin_c) = (0.0f64, 0usize, f64::INFINITY, 0usize);
        for c in 0..u.len() {
            if g.part[c] == bowshock_cfd::grid3::PART_FLUID {
                let q = u[c];
                let rho = q[0];
                let ke = 0.5 * (q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) / rho;
                let p = (1.4 - 1.0) * (q[4] - ke);
                let t = 1.4 * p / rho;
                if t.is_finite() && t > tmax {
                    tmax = t;
                    tmax_c = c;
                }
                if p.is_finite() && p < pmin {
                    pmin = p;
                    pmin_c = c;
                }
                let d = (q[0] - prev[c][0]).abs() / prev[c][0];
                if d.is_finite() && d > worst.0 {
                    worst = (d, c);
                }
                if !d.is_finite() {
                    worst = (f64::INFINITY, c);
                }
            }
        }
        let loc = |c: usize| {
            let (i, j, k) = (c % ni, (c / ni) % nj, c / (ni * nj));
            format!("({i},{j},{k}) x {:.3} y {:.4} z {:.4} dwall {:.2e}", g.xc[c][0], g.xc[c][1], g.xc[c][2], dwall.get(c).copied().unwrap_or(f64::NAN))
        };
        println!(
            "it {done:5} res {:.3e}  max T {tmax:.3} at {}  min p {pmin:.4} at {}  max drho/rho {:.2e} at {}",
            h.last().copied().unwrap_or(f64::NAN),
            loc(tmax_c),
            loc(pmin_c),
            worst.0,
            loc(worst.1)
        );
        if !h.last().is_some_and(|r| r.is_finite()) {
            break;
        }
        prev = u;
    }
    let _ = nk;
    Ok(())
}
