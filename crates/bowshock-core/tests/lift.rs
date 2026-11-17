//! 3D Euler lift engine: tiny ANF-like case on the CPU.

use bowshock_cfd::gas3::cons_to_prim3;
use bowshock_core::engine::case::{Case, Wake};
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::numerics::Numerics;
use bowshock_core::engine::run::run_cold;
use bowshock_core::engine::run3::{lift_forces_upto, run3_cold, run3_on};
use bowshock_core::StopReason;
use std::time::Instant;

#[test]
fn tiny_anf_case_reaches_stationary_normal_force() {
    let alpha = 2f64.to_radians();
    let case = Case3::tiny_anf(2.0, alpha);
    let x_base = case.body.x_base();
    let t0 = Instant::now();
    let r = run3_cold(&case).expect("grid");
    let wall = t0.elapsed().as_secs_f64();
    println!(
        "tiny ANF M2 a2: CN {:.5} CA {:.5} x_cp {:.4} d ({:.4} cal-from-tip) CN_alpha {:.3}/rad iters {} stop {:?} drop {:.2} orders wall {:.1}s",
        r.cn,
        r.ca,
        r.x_cp,
        r.x_cp,
        r.cn / alpha,
        r.iterations,
        r.stop_reason,
        r.drop_orders,
        wall
    );
    assert!(matches!(r.stop_reason, StopReason::ResidualDrop | StopReason::CdStationary), "stop {:?}", r.stop_reason);
    assert!(r.iterations <= 3000);
    assert!(r.cn > 0.0, "cn {}", r.cn);
    assert!(r.x_cp > 0.0 && r.x_cp < x_base, "x_cp {} base {}", r.x_cp, x_base);
}

#[test]
fn normal_force_is_antisymmetric_in_alpha() {
    let a = 2f64.to_radians();
    let t0 = Instant::now();
    let p = run3_cold(&Case3::tiny_anf(2.0, a)).expect("grid");
    let m = run3_cold(&Case3::tiny_anf(2.0, -a)).expect("grid");
    let rel_cn = (m.cn + p.cn).abs() / p.cn.abs();
    let rel_xcp = (m.x_cp - p.x_cp).abs() / p.x_cp.abs();
    println!(
        "antisymmetry: CN(+2) {:.12} CN(-2) {:.12} rel {:.2e}; x_cp {:.12} {:.12} rel {:.2e}; iters {} {}; wall {:.1}s",
        p.cn,
        m.cn,
        rel_cn,
        p.x_cp,
        m.x_cp,
        rel_xcp,
        p.iterations,
        m.iterations,
        t0.elapsed().as_secs_f64()
    );
    assert!(rel_cn < 1e-8, "CN relative asymmetry {rel_cn:e}");
    assert!(rel_xcp < 1e-8, "x_cp relative asymmetry {rel_xcp:e}");
}

#[test]
fn zero_incidence_has_no_normal_force() {
    let t0 = Instant::now();
    let r = run3_cold(&Case3::tiny_anf(2.0, 0.0)).expect("grid");
    println!("alpha 0: CN {:.3e} CA {:.5} iters {} stop {:?} wall {:.1}s", r.cn, r.ca, r.iterations, r.stop_reason, t0.elapsed().as_secs_f64());
    assert!(r.cn.abs() < 1e-10, "CN {}", r.cn);
}

/// Nose pressure drag of the 3D solver at alpha 0 (no fins) against the 2D Euler solver on the same 2D grid, both with the wall
/// pressure extrapolated from the two first cells (`cd_nose`); the first-cell values are printed for reference.
#[test]
fn axisymmetric_nose_drag_matches_2d() {
    for mach in [0.8, 2.0] {
        let t0 = Instant::now();
        let mut c3 = Case3::tiny_anf(mach, 0.0);
        c3.grid.fins = None;
        c3.grid.nk = 24;
        c3.numerics.max_iter = 8000;
        let l_nose = c3.body.body.nose_length();
        let mut s = c3.build_solver().expect("grid");
        let r3 = run3_on(&c3, &mut s, false);
        let (g, p_inf) = (&s.grid, s.cfg.freestream[4]);
        let mut fx = 0.0;
        for k in 0..g.nk {
            for i in g.axis[0].i.end..g.ni {
                let f = &g.jface[k * (g.nj + 1) * g.ni + i];
                if f.c[0] <= l_nose {
                    let p = cons_to_prim3(&s.u[g.cell(i, 0, k)])[4];
                    fx += (p - p_inf) * f.area * -f.n[0];
                }
            }
        }
        let cd3 = 2.0 * fx / (0.5 * mach * mach * std::f64::consts::FRAC_PI_4);
        let t3 = t0.elapsed().as_secs_f64();

        let t1 = Instant::now();
        let mut c2 = Case::new(mach, c3.body.clone(), 0, Wake::Sting, Numerics::auto(mach, 0, Wake::Sting));
        c2.grid_sting = c3.grid.sting.clone();
        let r2 = run_cold(&c2);
        let t2 = t1.elapsed().as_secs_f64();
        let cd3x = lift_forces_upto(&c3, &s, l_nose).ca;
        let diff = (cd3x - r2.cd_nose) / r2.cd_nose * 100.0;
        let diff_cell = (cd3 - r2.cd_nose_cell) / r2.cd_nose_cell * 100.0;
        println!(
            "M {mach}: 3D nose CA {cd3:.5} ({} it {:?}, {t3:.1}s) | 2D nose CD cell {:.5} extrapolated {:.5} ({} it {:?}, {t2:.1}s) | 3D total CA {:.5} CN {:.2e}",
            r3.iterations, r3.stop_reason, r2.cd_nose_cell, r2.cd_nose, r2.iters, r2.stop_reason, r3.ca, r3.cn
        );
        println!(
            "M {mach}: 3D nose CA extrapolated {cd3x:.5} vs 2D cd_nose {:.5} ({diff:+.3}%); first-cell {cd3:.5} vs {:.5} ({diff_cell:+.3}%)",
            r2.cd_nose, r2.cd_nose_cell
        );
        assert!(diff.abs() < 1.0, "M {mach}: 3D {cd3x} vs 2D {} ({diff:+.3}%)", r2.cd_nose);
    }
}
