//! The Lift-tier half domain of the product's incidence points (`Analysis`, `Kind::Half`) with 1 and 2 cells across the fin (DESIGN3D item 53): CN, CA, x_cp, iterations and
//! wall time per point on the Army-Navy Basic Finner.
//!
//! ```text
//! BOWSHOCK_WEDGE_BACKEND=gpu half_across <alpha_deg> <across,..> <mach,..>
//! ```
#[path = "../tests/fin_gates/common.rs"]
mod common;

use bowshock_core::engine::case3::{Case3, WallFlow};
use common::{tier, Session, TierKind};

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let list = |i: usize| -> Vec<f64> { args[i].split(',').map(|x| x.parse().expect("number")).collect() };
    let alpha = args[0].parse::<f64>().map_err(|e| e.to_string())?;
    let (acrosses, machs) = (list(1), list(2));
    // the grid is sized for max(Mach, 2); the sea-level Reynolds number of the 30 mm base
    let flow = |m: f64| WallFlow { re_c: 1.225 * 340.294 * 0.03 / 1.7894e-5, mach_max: m.max(2.0), ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    for &m in &machs {
        for &a in &acrosses {
            let case = Case3::base_half(m, alpha.to_radians(), Case3::anf_body(), Some(Case3::anf_fins()), tier(TierKind::Lift, flow(m)), 0)
                .with_fin_cells_across(a as usize);
            let t = std::time::Instant::now();
            let mut s = Session::new(&case)?;
            let r = s.run(&case, false)?;
            let l = r.lift;
            println!(
                "ACROSS mach {m} alpha {alpha} across {a}: CN {:+.5} CA {:+.5} x_cp {:.4} d  ({}), {:.1} s",
                l.cn,
                l.ca,
                l.x_cp,
                r.stop(),
                t.elapsed().as_secs_f64()
            );
        }
    }
    Ok(())
}
