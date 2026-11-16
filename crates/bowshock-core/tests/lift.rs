//! 3D Euler lift engine: tiny ANF-like case on the CPU.

use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::run3::run3_cold;
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
