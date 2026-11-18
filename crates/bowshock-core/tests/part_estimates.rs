//! Independent estimates of the parts of the zero-incidence drag of the Army-Navy Finner (DESIGN3D item 40), against which the 3D parts of `tests/arcas_study.rs` are read:
//! Taylor-Maccoll for the nose, an Eckert reference-temperature flat plate (Schlichting's `0.455 / log10(Re)^2.58`) for the friction of the body and of the fins, the
//! Fleeman / Niskanen base drag `0.25 / M` for the body base and for the blunt trailing edges of the fins, an exact oblique shock for the wedge faces of the fins and
//! the Rayleigh pitot pressure for the leading-edge faces. Run: `cargo test -p bowshock-core --test part_estimates -- --nocapture`.

use bowshock_reference::{anf, base_drag, taylor_maccoll};
use std::f64::consts::FRAC_PI_4;

const G: f64 = 1.4;
const T_INF: f64 = 288.15;

/// Sutherland viscosity ratio `mu(T) / mu(T_INF)`.
fn mu_ratio(t: f64) -> f64 {
    (t / T_INF).powf(1.5) * (T_INF + 110.4) / (t + 110.4)
}

/// Mean turbulent skin friction of a flat plate over `length` in body diameters at Mach `m`, adiabatic wall, Eckert's reference temperature.
fn flat_plate_cf(m: f64, length: f64) -> f64 {
    let re_d = 1.225 * 340.294 * 0.03 / 1.7894e-5 * m;
    let t_star = T_INF * (1.0 + 0.032 * m * m + 0.58 * (0.89 * 0.2 * m * m - 0.032 * m * m * 0.0));
    let rho_ratio = T_INF / t_star;
    let re_star = re_d * length * rho_ratio / mu_ratio(t_star);
    rho_ratio * 0.455 / re_star.log10().powf(2.58)
}

/// Pressure coefficient of the weak oblique shock of a wedge of half angle `delta`.
fn wedge_cp(m: f64, delta: f64) -> f64 {
    let (mut lo, mut hi) = ((1.0 / m).asin() + 1e-9, 0.5 * std::f64::consts::PI - 1e-9);
    // the deflection rises from zero at the Mach angle to its maximum: bisect on the weak branch below the maximum
    let mut b_max = lo;
    let mut d_max = 0.0;
    for k in 0..20_000 {
        let b = lo + (hi - lo) * k as f64 / 20_000.0;
        let d = taylor_maccoll::oblique_deflection(m, b);
        if d > d_max {
            d_max = d;
            b_max = b;
        }
    }
    hi = b_max;
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if taylor_maccoll::oblique_deflection(m, mid) < delta {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (_, p21) = taylor_maccoll::oblique_post_shock(m, 0.5 * (lo + hi));
    (p21 - 1.0) / (0.5 * G * m * m)
}

/// Pressure coefficient at the stagnation point behind a normal shock (Rayleigh pitot).
fn pitot_cp(m: f64) -> f64 {
    let p02 = ((G + 1.0) * m * m / 2.0).powf(G / (G - 1.0)) / ((2.0 * G * m * m - (G - 1.0)) / (G + 1.0)).powf(1.0 / (G - 1.0));
    (p02 - 1.0) / (0.5 * G * m * m)
}

#[test]
fn the_finner_parts_against_their_independent_estimates() {
    let s_ref = FRAC_PI_4;
    let l_nose = 0.5 / 10f64.to_radians().tan();
    let s_wet_body = std::f64::consts::PI * 0.5 * (l_nose / 10f64.to_radians().cos()) + std::f64::consts::PI * (10.0 - l_nose);
    let (n, t, span) = (4.0, anf::FIN_THICKNESS_CAL, 1.0);
    let a_side = n * t * span / s_ref; // frontal area of the wedge faces and of the trailing edge faces, in body areas
    let a_le = n * 2.0 * (0.1 * 0.5 * t) * span / s_ref; // the leading edge face of half thickness 0.1 t/2
    let s_wet_fins = n * 2.0 * 1.0 * span / s_ref;
    let half_angle = (0.5 * t / 1.0).atan();
    println!("ANF: nose {l_nose:.3} d, wetted body {:.3} d^2 = {:.2} body areas, fins {s_wet_fins:.2} body areas wetted, frontal {a_side:.4}, leading edge {a_le:.4}, wedge half angle {:.2} deg", s_wet_body, s_wet_body / s_ref, half_angle.to_degrees());
    println!("mach | nose TM | friction body | friction fins | base 0.25/M | wedge sides | LE face (0.85 pitot) | TE face (0.25/M) | sum | table");
    for (m, table) in anf::CD0.iter().filter(|(m, _)| *m > 1.5) {
        let nose = taylor_maccoll::solve_cone(*m, 10f64.to_radians()).unwrap().cp_surface;
        let f_body = flat_plate_cf(*m, 10.0) * s_wet_body / s_ref;
        let f_fins = flat_plate_cf(*m, 1.0) * s_wet_fins;
        let base = base_drag::base_drag_coefficient(*m);
        let wedge = wedge_cp(*m, half_angle) * a_side;
        let le = 0.85 * pitot_cp(*m) * a_le;
        let te = base_drag::base_drag_coefficient(*m) * a_side;
        let sum = nose + f_body + f_fins + base + wedge + le + te;
        println!(
            "{m:.3} | {nose:.4} | {f_body:.4} | {f_fins:.4} | {base:.4} | {wedge:.4} | {le:.4} | {te:.4} | {sum:.4} | {table:.3} ({:+.1} %)",
            100.0 * (sum / table - 1.0)
        );
    }
}
