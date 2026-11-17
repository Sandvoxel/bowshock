//! First RANS runs with fins through `Case3` on the CPU (`Solver3` with the SST model): the ANF body and fins at Mach 2, a finned sector on the wall-resolved
//! `Coarse` tier (fin stack, `y+` ~ 1 on the body, 15 on the fin sides) and a half domain at 2 degrees on the `Lift` tier (`y+` 60), a few hundred iterations each.
//! They print, every 50 iterations, the density residual, the pressure and friction drag of body and fins, the smallest density / pressure / `k` / `omega`, and where the
//! largest residual sits (the region of the grid: axis, nose, body, fin leading edge / trailing edge / tip / junction, wake). `#[ignore]`d: minutes of CPU.
//!
//! `cargo test -p bowshock-core --test viscous_fins -- --ignored --nocapture <name>`; knobs: `ITERS` (default 300), `MACH` (2).

use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::grid3::{Domain3, FinPhase};
use bowshock_core::engine::run3::{drag_parts_on, lift_forces, y_plus_max};
use bowshock_core::FinProfile;

fn env<T: std::str::FromStr>(key: &str, default: T) -> T {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// The ANF's nose diameter is 0.03 m: `Re_c` of the sea-level atmosphere.
fn flow(mach_max: f64) -> WallFlow {
    WallFlow { re_c: 1.225 * 340.294 * 0.03 / 1.7894e-5, mach_max, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL }
}

/// Mean skin friction of a flat plate of length `x` (in diameters) at `re_c`, `mach`: the facade's fin model (Niskanen 2009 eqs. 3.78-3.81).
fn plate_cf(re_c: f64, mach: f64, x: f64) -> f64 {
    let cf = 1.0 / (1.50 * (re_c * mach * x).ln() - 5.6).powi(2);
    if mach < 1.0 {
        cf * (1.0 - 0.1 * mach * mach)
    } else {
        cf / (1.0 + 0.15 * mach * mach).powf(0.58)
    }
}

/// Where in the grid is the cell `c`: a name for the region.
fn region(case: &Case3, g: &bowshock_cfd::grid3::Grid3, c: usize) -> String {
    let (i, j, k) = (c % g.ni, (c / g.ni) % g.nj, c / (g.ni * g.nj));
    let x = g.xc[c];
    let r = x[1].hypot(x[2]);
    let th = x[1].atan2(x[2]).to_degrees();
    let xb = case.body.x_base();
    let what = if r < 0.05 {
        "on the axis"
    } else if x[0] > xb {
        "in the wake"
    } else if x[0] < case.body.body.nose_length() {
        "on the nose"
    } else if (0..g.nk).any(|kk| g.is_solid(i, j, kk)) {
        "beside a solid block"
    } else {
        "in the flow"
    };
    format!("cell ({i},{j},{k}) at x = {:.3}, r = {r:.3}, theta = {th:.1} deg, {what}", x[0])
}

fn run(case: &Case3, iters: usize, what: &str) {
    let mut s = case.build_solver().unwrap_or_else(|e| panic!("{what}: {e}"));
    let g = s.grid.clone();
    let (re_c, mach) = (case.viscous.expect("a viscous case").re_c, case.mach);
    // flat-plate estimate of the friction drag on pi/4: the wetted area of the ANF body (cone + cylinder) and of the fins (two sides, 1 d chord), the plate friction at
    // the length of each
    let l_nose = case.body.body.nose_length();
    let a_body = std::f64::consts::PI * 0.5 * (0.5f64.hypot(l_nose)) + std::f64::consts::PI * (case.body.x_base() - l_nose);
    let n = case.grid.fins.as_ref().map_or(0.0, |f| f64::from(f.count));
    let (a_fin, c_fin) = case.grid.fins.as_ref().map_or((0.0, 1.0), |f| (2.0 * n * f.span * 0.5 * (f.root_chord + f.tip_chord), f.root_chord));
    let est_body = plate_cf(re_c, mach, case.body.x_base()) * a_body / std::f64::consts::FRAC_PI_4;
    let est_fin = plate_cf(re_c, mach, c_fin) * a_fin / std::f64::consts::FRAC_PI_4;
    println!(
        "{what}: {} x {} x {} cells, Re_c {re_c:.3e}, Mach {mach}, flat-plate estimate of the friction: body {est_body:.4}, fins {est_fin:.4}",
        g.ni, g.nj, g.nk
    );
    let t0 = std::time::Instant::now();
    let mut r0 = f64::NAN;
    for it in 1..=iters {
        let r = s.step();
        if it == 20 {
            r0 = r;
        }
        if !r.is_finite() {
            let big = (0..s.res.len()).max_by(|&a, &b| s.res[a][0].abs().total_cmp(&s.res[b][0].abs())).unwrap();
            println!("  it {it}: NON-FINITE residual; largest |res| {}", region(case, &g, big));
            return;
        }
        if it % 50 == 0 || it == iters {
            let p = drag_parts_on(case, &s);
            let rans = s.rans().expect("rans");
            let fluid: Vec<usize> = (0..g.part.len()).filter(|&c| g.part[c] == 0).collect();
            let (kmin, wmin) = fluid.iter().fold((f64::INFINITY, f64::INFINITY), |a, &c| (a.0.min(rans.turb[c][0]), a.1.min(rans.turb[c][1])));
            let (rmin, pmin) = fluid.iter().fold((f64::INFINITY, f64::INFINITY), |a, &c| {
                let w = bowshock_cfd::gas3::cons_to_prim3(&s.u[c]);
                (a.0.min(w[0]), a.1.min(w[4]))
            });
            let big = fluid.iter().copied().max_by(|&a, &b| s.res[a][0].abs().total_cmp(&s.res[b][0].abs())).unwrap();
            println!(
                "  it {it:4}: residual {r:.3e} ({:+.1} orders from it 20), pressure drag body {:.4} fins {:.4} base {:.4}; friction body {:.4} fins {:.4}; min rho {rmin:.3} p {pmin:.3} k {kmin:.2e} omega {wmin:.2e}; largest |res| {}",
                (r / r0).log10(),
                p.nose + p.aft.iter().sum::<f64>(),
                p.fin_sides + p.fin_edges + p.fin_tips,
                p.base,
                p.body_friction(),
                p.fin_friction(),
                region(case, &g, big)
            );
        }
    }
    let p = drag_parts_on(case, &s);
    let f = lift_forces(case, &s);
    println!(
        "{what}: {iters} iterations in {:.0} s: friction body {:.4} (estimate {est_body:.4}, ratio {:.2}) fins {:.4} (estimate {est_fin:.4}, ratio {:.2}); CA {:.4} CN {:.4} x_cp {:.3}; y+max {:.2}",
        t0.elapsed().as_secs_f64(),
        p.body_friction(),
        p.body_friction() / est_body,
        p.fin_friction(),
        if est_fin > 0.0 { p.fin_friction() / est_fin } else { f64::NAN },
        f.ca,
        f.cn,
        f.x_cp,
        y_plus_max(&s)
    );
    assert!(p.body_friction() > 0.0 && (est_fin == 0.0 || p.fin_friction() > 0.0), "friction of body and fins must be positive");
}

/// The ANF body with its four fins, a sector on the wall-resolved `Coarse` tier (the fin stack: `y+` 15 on the fin sides), Mach 2.
#[test]
#[ignore = "RANS on the CPU: minutes"]
fn the_finned_sector_on_the_coarse_tier_runs() {
    let mach = env("MACH", 2.0);
    let fins = bowshock_core::engine::grid3::Fins3 { profile: FinProfile::DoubleWedge, ..Case3::anf_fins() };
    let case = Case3::sector(mach, Case3::anf_body(), fins, Tier::Coarse(flow(mach)), 0);
    run(&case, env("ITERS", 300), "sector, Coarse tier (fin stack)");
}

/// The half domain at 2 degrees on the `Lift` tier (`y+` 60, wall treatment), fins between the pitch plane and each other, Mach 2.
#[test]
#[ignore = "RANS on the CPU: minutes"]
fn the_half_domain_at_two_degrees_on_the_lift_tier_runs() {
    let mach = env("MACH", 2.0);
    let fins = bowshock_core::engine::grid3::Fins3 { profile: FinProfile::DoubleWedge, ..Case3::anf_fins() };
    let mut case = Case3::base_half(mach, 2f64.to_radians(), Case3::anf_body(), Some(fins), Tier::Lift(flow(mach)), 0);
    case.grid.domain = Domain3::Half { phase: FinPhase::Between };
    run(&case, env("ITERS", 300), "half domain at 2 degrees, Lift tier");
}
