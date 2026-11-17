//! Euler gates of the fins on the product grid (stage S6, DESIGN3D G-F2 and G-F3): the ANF body with a four-fin sector on the base-wake grid, the wall-resolved
//! tier's fin stack (`Tier::Coarse`, the grid the viscous runs use) solved with the Euler equations on the CPU. The cases of a gate run in parallel threads and
//! share their grid through the engine's grid cache. Slow: `#[ignore]`d, run with `--run-ignored only --no-capture`; the output lines are
//! `GATE <id> <case> <measured> <reference> <band> PASS|FAIL`.

use bowshock_cfd::solver3::Solver3;
use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::grid3::Fins3;
use bowshock_core::engine::run3::{drag_parts, run3_on, wall_forces, DragParts, Wall};
use bowshock_core::FinProfile;
use bowshock_reference::shock_expansion::{mach_from_pm, prandtl_meyer};
use std::f64::consts::{FRAC_PI_4, PI};

const GAMMA: f64 = 1.4;
const N_FINS: u32 = 4;

/// What one fin case gives: the pressure-drag parts (coefficients on `pi/4`), the leading-edge face's drag on the frontal area of the fins, the iterations.
struct FinRun {
    parts: DragParts,
    /// `F_x(leading-edge faces) / (q N t span)`.
    cd_le: f64,
    iterations: usize,
    stop: String,
}

impl FinRun {
    fn fin_drag(&self) -> f64 {
        self.parts.fin_sides + self.parts.fin_edges + self.parts.fin_tips
    }
}

/// The wall-resolved grid is sized for Mach 3 at `Re_c` 1e6 for every case (one grid, through the cache): the first body cell for `y+` 1, the fin stack for 15.
fn flow() -> WallFlow {
    WallFlow { re_c: 1.0e6, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0 }
}

fn run(mach: f64, fins: Fins3, max_iter: usize) -> FinRun {
    let (t, span) = (fins.thickness, fins.span);
    let mut case = Case3::sector(mach, Case3::anf_body(), fins, Tier::Coarse(flow()), 0);
    case.numerics.max_iter = max_iter;
    let mut s: Solver3 = case.build_solver_cached().expect("grid");
    let r = run3_on(&case, &mut s, false);
    let p_inf = s.cfg.freestream[4];
    let parts = drag_parts(&case, &s.grid, &s.u, p_inf);
    let w = wall_forces(&s.grid, &s.u, p_inf, f64::INFINITY);
    let q = 0.5 * mach * mach;
    let cd_le = w.get(Wall::FinLe)[0] * (2.0 * PI / s.grid.theta_span) / (q * f64::from(N_FINS) * t * span);
    FinRun { parts, cd_le, iterations: r.iterations, stop: format!("{}", r.stop_reason) }
}

fn fins(profile: FinProfile, chord: f64, span: f64, thickness: f64) -> Fins3 {
    Fins3 { count: N_FINS, root_chord: chord, tip_chord: chord, span, sweep: 0.0, thickness, te_to_base: 0.0, profile }
}

fn report(id: &str, case: &str, measured: f64, reference: f64, band: &str, pass: bool) -> bool {
    println!("GATE {id} {case} {measured:.6} {reference:.6} {band} {}", if pass { "PASS" } else { "FAIL" });
    pass
}

/// Pressure drag per unit span of a double wedge of thickness ratio `tau` at Mach `m` by oblique shock and Prandtl-Meyer expansion: the front faces turn the flow
/// by `atan(tau)` (weak shock), the rear faces expand it by twice that from the front faces' state; `cd = tau (p_front - p_rear) / q` on the planform area.
fn double_wedge_shock_expansion(m: f64, tau: f64) -> f64 {
    let theta = tau.atan();
    let theta_of = |beta: f64| {
        let num = 2.0 / beta.tan() * (m * m * beta.sin().powi(2) - 1.0);
        let den = m * m * (GAMMA + (2.0 * beta).cos()) + 2.0;
        (num / den).atan()
    };
    // weak solution: the first crossing of the turning angle as the shock angle grows from the Mach angle
    let mu = (1.0 / m).asin();
    let n = 100_000;
    let step = (0.5 * PI - mu) / n as f64;
    let mut beta = mu;
    for i in 1..=n {
        let b = mu + step * i as f64;
        if theta_of(b) >= theta {
            let (mut lo, mut hi) = (b - step, b);
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if theta_of(mid) < theta {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            beta = 0.5 * (lo + hi);
            break;
        }
    }
    let mn1 = m * beta.sin();
    let p2 = 1.0 + 2.0 * GAMMA / (GAMMA + 1.0) * (mn1 * mn1 - 1.0);
    let mn2 = ((1.0 + 0.5 * (GAMMA - 1.0) * mn1 * mn1) / (GAMMA * mn1 * mn1 - 0.5 * (GAMMA - 1.0))).sqrt();
    let m2 = mn2 / (beta - theta).sin();
    let m3 = mach_from_pm(prandtl_meyer(m2) + 2.0 * theta);
    let iso = |mm: f64| 1.0 + 0.5 * (GAMMA - 1.0) * mm * mm;
    let p3 = p2 * (iso(m2) / iso(m3)).powf(GAMMA / (GAMMA - 1.0));
    tau * (p2 - p3) / (0.5 * GAMMA * m * m)
}

/// G-F2: a sharp double-wedge fin (thickness ratio 0.08, chord 0.15 d, exposed span 1 d, the planform of four fins on the ANF body, flush with the base), Euler,
/// Mach 1.5, 2 and 3 on the fin-stack grid: the fins' pressure drag (sides, edges, tips) against the shock-expansion drag of the section on the planform area. The
/// fin has tips: the tip Mach cones relieve the pressure on `1 / (2 beta AR)` of the planform (the rectangular wing of linear theory, `AR` = span / chord), so
/// the reference is the section's drag times `1 - 1 / (2 beta AR)`; the gate is 8 % of that, and the numbers against the section's drag alone and against
/// Ackeret's wave drag (`bowshock_reference::fin_wave`) are printed.
#[test]
#[ignore = "validation: CPU, minutes (three 650k-cell Euler runs in parallel threads)"]
fn gate_f2_double_wedge_fin_pressure_drag_matches_shock_expansion() {
    let (chord, span, tau) = (0.15, 1.0, 0.08);
    let machs = [1.5, 2.0, 3.0];
    let runs: Vec<FinRun> = std::thread::scope(|sc| {
        let hs: Vec<_> = machs.iter().map(|&m| sc.spawn(move || run(m, fins(FinProfile::DoubleWedge, chord, span, tau * chord), 3000))).collect();
        hs.into_iter().map(|h| h.join().expect("run")).collect()
    });
    let mut ok = true;
    for (&m, r) in machs.iter().zip(&runs) {
        let beta = (m * m - 1.0).sqrt();
        let to_pi4 = f64::from(N_FINS) * chord * span / FRAC_PI_4;
        let section = double_wedge_shock_expansion(m, tau) * to_pi4;
        let tip = 1.0 - 1.0 / (2.0 * beta * span / chord);
        let ackeret = bowshock_reference::fin_wave::wave_drag_coefficient(bowshock_reference::fin_wave::double_wedge_slope_integral(), tau, m) * to_pi4;
        let measured = r.fin_drag();
        println!(
            "INFO M {m}: fin drag {measured:+.5} (sides {:+.5} edges {:+.5} tips {:+.5}; {} it, {}); shock-expansion section {section:.5} ({:+.1} %), with tip relief {:.5} ({:+.1} %), Ackeret {ackeret:.5} ({:+.1} %)",
            r.parts.fin_sides,
            r.parts.fin_edges,
            r.parts.fin_tips,
            r.iterations,
            r.stop,
            100.0 * (measured / section - 1.0),
            section * tip,
            100.0 * (measured / (section * tip) - 1.0),
            100.0 * (measured / ackeret - 1.0)
        );
        ok &= report(
            "G-F2",
            &format!("double-wedge-M{m:.1}:cd_fin_pressure"),
            measured,
            section * tip,
            "rel8%",
            (measured / (section * tip) - 1.0).abs() <= 0.08,
        );
    }
    assert!(ok, "G-F2: a case is outside 8 %");
}

/// Stagnation pressure over the free-stream dynamic pressure, the Rayleigh pitot coefficient fitted by Niskanen (2009, eq. 3.85 with B.1-B.2), copied from the
/// facade's fin model (`bowshock_core::fins`, to be deleted with the empirical add-on).
fn niskanen_stagnation(mach: f64) -> f64 {
    let m2 = mach * mach;
    if mach < 1.0 {
        1.0 + m2 / 4.0 + m2 * m2 / 40.0
    } else {
        1.84 - 0.76 / m2 + 0.166 / (m2 * m2) + 0.035 / (m2 * m2 * m2)
    }
}

/// G-F3: the blunt leading edge of a flat (`Square`) fin, Euler on the fin-stack grid: the drag of the leading-edge faces on their frontal area against Niskanen's
/// `0.85 q_stag / q` (0.9045, 1.3074, 1.4941 at Mach 0.5, 1.5, 3), 25 % each (reported); and the rounded section's total fin pressure drag below the square one's
/// at Mach 0.5.
#[test]
#[ignore = "validation: CPU, minutes (four 650k-cell Euler runs in parallel threads)"]
fn gate_f3_blunt_leading_edge_drag_matches_niskanen() {
    let (chord, span, thickness) = (0.5, 1.0, 0.05);
    let square = |m: f64| move || run(m, fins(FinProfile::Square, chord, span, thickness), if m < 1.0 { 2500 } else { 3000 });
    let (runs, rounded) = std::thread::scope(|sc| {
        let hs: Vec<_> = [0.5, 1.5, 3.0].iter().map(|&m| sc.spawn(square(m))).collect();
        let hr = sc.spawn(|| run(0.5, fins(FinProfile::Rounded, chord, span, thickness), 2500));
        (hs.into_iter().map(|h| h.join().expect("run")).collect::<Vec<_>>(), hr.join().expect("run"))
    });
    let mut ok = true;
    for (&m, r) in [0.5, 1.5, 3.0].iter().zip(&runs) {
        let reference = 0.85 * niskanen_stagnation(m);
        println!(
            "INFO M {m}: square leading edge drag {:+.4} of the frontal area ({} it, {}); fin pressure drag {:+.5}",
            r.cd_le,
            r.iterations,
            r.stop,
            r.fin_drag()
        );
        ok &= report("G-F3", &format!("square-M{m:.1}:cd_le"), r.cd_le, reference, "rel25%", (r.cd_le / reference - 1.0).abs() <= 0.25);
    }
    println!(
        "INFO M 0.5: rounded fin pressure drag {:+.5} ({} it, {}), square {:+.5}; rounded leading edge drag {:+.4} of the frontal area",
        rounded.fin_drag(),
        rounded.iterations,
        rounded.stop,
        runs[0].fin_drag(),
        rounded.cd_le
    );
    ok &= report("G-F3", "rounded-below-square-M0.5:cd_fin_pressure", rounded.fin_drag(), runs[0].fin_drag(), "below", rounded.fin_drag() < runs[0].fin_drag());
    assert!(ok, "G-F3: a case failed");
}

#[test]
fn the_double_wedge_reference_is_the_thin_airfoil_result_at_small_thickness() {
    // the oblique-shock + expansion drag tends to Ackeret's `4 tau^2 / beta` as the section gets thin
    for m in [1.5, 2.0, 3.0] {
        let tau = 0.01;
        let ack = 4.0 * tau * tau / (m * m - 1.0f64).sqrt();
        let se = double_wedge_shock_expansion(m, tau);
        assert!((se / ack - 1.0).abs() < 0.03, "M {m}: shock-expansion {se} vs Ackeret {ack}");
    }
}

#[test]
fn niskanen_stagnation_numbers() {
    for (m, cd) in [(0.5, 0.9045), (1.5, 1.3074), (3.0, 1.4941)] {
        assert!((0.85 * niskanen_stagnation(m) - cd).abs() < 1e-3, "M {m}");
    }
}
