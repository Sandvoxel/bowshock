//! `sector_vs_half`: a fin sector against the half domain of the same body at zero incidence, on the GPU (stage S2, G-W3; the CPU twin is
//! `bowshock-core/tests/wedge.rs::sector_vs_half`, on the tiny ANF grid).
//!
//! ```text
//! cargo run --release -p bowshock-validate --example sector_vs_half -- [--n 4,3] [--mach 0.8,2.0] [--nk-sector 6] [--precision f32|df32]
//! ```
//!
//! Level 0 of the Army-Navy Basic Finner (`Case3::anf_body`, `Case3::anf_fins` with `N` fins), the sector `[0, pi/N]` with `nk-sector` azimuthal cells against
//! the half domain `[0, pi]` with the cell count that replicates the sector grid (`N` sectors, minus the layers shared on the symmetry planes: `N nk - 2` for
//! four fins "x", `N nk - 1` for three fins with one in the pitch plane). One line per (N, Mach) with the drag split by surface and the cell widths.

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::case3::{Case3, Tier};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::{Domain3, FinPhase};
use bowshock_core::engine::run3::DragParts;

fn list<T: std::str::FromStr>(v: Option<String>, default: &str) -> Vec<T> {
    v.as_deref().unwrap_or(default).split(',').map(|x| x.trim().parse().unwrap_or_else(|_| panic!("cannot parse `{x}`"))).collect()
}

struct Run {
    d: DragParts,
    iterations: usize,
    stop: String,
    theta_cell_deg: f64,
    cells: String,
}

fn run(case: &Case3, prec: Precision) -> Result<Run, String> {
    let mut r = GpuRunner3::new(case, prec).map_err(|e| e.to_string())?;
    let p = r.run_on(case, false).map_err(|e| e.to_string())?;
    let d = r.drag_parts(case).map_err(|e| e.to_string())?;
    let g = &r.host.grid;
    Ok(Run {
        d,
        iterations: p.iterations,
        stop: format!("{:?}", p.stop_reason),
        theta_cell_deg: (g.theta_span / g.nk as f64).to_degrees(),
        cells: format!("{} x {} x {}", g.ni, g.nj, g.nk),
    })
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let ns: Vec<u32> = list(value("--n"), "4,3");
    let machs: Vec<f64> = list(value("--mach"), "0.8,2.0");
    let nk_s: usize = value("--nk-sector").map_or(6, |v| v.parse().expect("--nk-sector"));
    let prec = if value("--precision").as_deref() == Some("df32") { Precision::Extended } else { Precision::F32 };
    println!("| N | Mach | domain | cells | cell width (deg) | iterations | stop | fins | nose | aft | base | total | d total vs half |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for &n in &ns {
        let mut fins = Case3::anf_fins();
        fins.count = n;
        let (phase, nk_h) = if n % 2 == 0 { (FinPhase::Between, n as usize * nk_s - 2) } else { (FinPhase::InPlane, n as usize * nk_s - 1) };
        for &mach in &machs {
            let mut sector = Case3::sector(mach, Case3::anf_body(), fins.clone(), Tier::Euler, 0);
            sector.grid.nk = nk_s;
            let mut half = Case3::new(mach, 0.0, Case3::anf_body(), Some(fins.clone()), 0);
            half.grid.domain = Domain3::Half { phase };
            half.grid.nk = nk_h;
            let (s, h) = (run(&sector, prec), run(&half, prec));
            let total_h = h.as_ref().map_or(f64::NAN, |h| h.d.total);
            for (name, r) in [("sector", &s), ("half", &h)] {
                match r {
                    Ok(r) => println!(
                        "| {n} | {mach} | {name} | {} | {:.2} | {} | {} | {:+.5} | {:+.5} | {:+.5} | {:+.5} | {:+.5} | {} |",
                        r.cells,
                        r.theta_cell_deg,
                        r.iterations,
                        r.stop,
                        r.d.fin_sides + r.d.fin_edges + r.d.fin_tips,
                        r.d.nose,
                        r.d.aft.iter().sum::<f64>(),
                        r.d.base,
                        r.d.total,
                        if name == "sector" { format!("{:+.2} %", 100.0 * (r.d.total / total_h - 1.0)) } else { "-".into() }
                    ),
                    Err(e) => println!("| {n} | {mach} | {name} | error: {e} | | | | | | | | | |"),
                }
            }
        }
    }
}
