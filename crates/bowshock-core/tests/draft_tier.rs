//! The Draft tier against the Coarse tier on the finless example body (ogive nose 3 d, tube 12 d) as a wedge: cells and drag at one Mach number. `#[ignore]`d: minutes of CPU.
//! `TIER=coarse|draft MACH=0.5 YP=40 cargo test -p bowshock-core --test draft_tier -- --ignored --nocapture`.

use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::run3::{drag_parts_on, run3_on};

fn env<T: std::str::FromStr>(key: &str, default: T) -> T {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

#[test]
#[ignore = "RANS on the CPU: minutes"]
fn the_draft_tier_against_the_coarse_tier() {
    let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 0.2286 / 0.0762 } }, 0.9134 / 0.0762);
    // the example rocket at 120 m: Re_c of the nose diameter
    let mach = env("MACH", 0.5);
    let flow = WallFlow { re_c: 1.756e6, mach_max: env("GRID_MACH", 3.0), ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
    let tier_name: String = env("TIER", "draft".to_string());
    let tier = match tier_name.as_str() {
        "coarse" => Tier::Coarse(flow),
        "lift" => Tier::Lift(flow),
        _ => Tier::Draft(flow, env("YP", 40.0)),
    };
    let mut case = Case3::wedge(mach, body, 0).with_tier(tier);
    if let Some(w) = std::env::var("MEAN_WINDOW").ok().and_then(|v| v.parse().ok()) {
        case.numerics.mean_window = w;
    }
    let mut s = case.build_solver().expect("grid");
    let (ni, nj) = (s.grid.ni, s.grid.nj);
    let t0 = std::time::Instant::now();
    let r = run3_on(&case, &mut s, false);
    let p = drag_parts_on(&case, &s);
    println!(
        "DRAFT {tier_name} M {mach}: {ni} x {nj} = {} cells, {} iterations, {:?}, {:.0} s ({:.0} ms / iteration); CD total {:.5} (pressure {:.5} friction {:.5} base {:.5} nose {:.5}), y+max {:.2}, cp_base {:.4}",
        ni * nj,
        r.iterations,
        r.stop_reason,
        t0.elapsed().as_secs_f64(),
        1e3 * t0.elapsed().as_secs_f64() / r.iterations.max(1) as f64,
        p.total,
        p.total - p.friction.total,
        p.friction.total,
        p.base,
        p.nose,
        r.y_plus_max,
        r.cp_base
    );
}
