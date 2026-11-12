//! Driver for the intermittent device loss of `bench`: runs the lazy solver of one grid in one staging, precision and loop form
//! (optionally after the hand-written-kernel solver on a second Luma device, as `bench` does), `--reps` times.
//!
//! `cargo run --release --example hang -- [--old] [--staging Fusion|Lines|Fluxes|Split] [--prec f32|df32|both]
//!  [--forms until,replay,eval] [--n 2000] [--sync 50] [--grid 100x50] [--reps 1]`

use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_gpu::{GpuSolver, Mode, Precision};
use bowshock_lazy::Staging;
use bowshock_reference::taylor_maccoll::solve_cone;
use std::io::Write;
use std::time::Instant;

fn say(text: &str) {
    println!("{text}");
    let _ = std::io::stdout().flush();
}

macro_rules! lazy {
    ($module:ident, $staging:expr, $ctx:expr, $host:expr, $u0:expr, $n:expr, $sync:expr, $forms:expr, $label:expr) => {{
        use bowshock_lazy::$module::LazySolver;
        let t0 = Instant::now();
        let mut s = LazySolver::new($ctx, &$host.grid, &$host.cfg).unwrap();
        s.staging = $staging;
        s.run($sync, $sync).unwrap();
        say(&format!("  {} first call {:.2} s", $label, t0.elapsed().as_secs_f64()));
        if $forms.contains(&"until") {
            let mut best = f64::INFINITY;
            for _ in 0..3 {
                s.set_state($u0).unwrap();
                s.sync().unwrap();
                let t0 = Instant::now();
                s.run($n, $sync).unwrap();
                best = best.min(1e6 * t0.elapsed().as_secs_f64() / $n as f64);
            }
            say(&format!("  {} until {best:.1} us/iter", $label));
        }
        if $forms.contains(&"replay") {
            s.run_replay($sync).unwrap();
            s.sync().unwrap();
            let mut best = f64::INFINITY;
            for _ in 0..3 {
                s.set_state($u0).unwrap();
                s.sync().unwrap();
                let t0 = Instant::now();
                s.run_replay($n).unwrap();
                s.sync().unwrap();
                best = best.min(1e6 * t0.elapsed().as_secs_f64() / $n as f64);
            }
            say(&format!("  {} replay {best:.1} us/iter", $label));
        }
        if $forms.contains(&"eval") {
            s.run_eval($sync).unwrap();
            s.sync().unwrap();
            let n_eval = $n.min(300);
            let mut best = f64::INFINITY;
            for _ in 0..3 {
                s.set_state($u0).unwrap();
                s.sync().unwrap();
                let t0 = Instant::now();
                let h = s.run_eval(n_eval).unwrap();
                s.sync().unwrap();
                best = best.min(1e6 * t0.elapsed().as_secs_f64() / n_eval as f64);
                drop(h);
            }
            say(&format!("  {} eval {best:.1} us/iter", $label));
        }
    }};
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flag = |name: &str| args.iter().any(|a| a == name);
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let n: usize = value("--n").and_then(|s| s.parse().ok()).unwrap_or(2000);
    let sync: usize = value("--sync").and_then(|s| s.parse().ok()).unwrap_or(50);
    let reps: usize = value("--reps").and_then(|s| s.parse().ok()).unwrap_or(1);
    let (ni, nj) = value("--grid")
        .and_then(|g| g.split_once('x').and_then(|(a, b)| Some((a.parse().ok()?, b.parse().ok()?))))
        .unwrap_or((100usize, 50usize));
    let staging_name = value("--staging").unwrap_or_else(|| "Fusion,Lines,Fluxes,Split".to_string());
    let stagings: Vec<Staging> = staging_name
        .split(',')
        .map(|name| match name {
            "Fusion" => Staging::Fusion,
            "Lines" => Staging::Lines,
            "Fluxes" => Staging::Fluxes,
            _ => Staging::Split,
        })
        .collect();
    let prec = value("--prec").unwrap_or_else(|| "both".to_string());
    let forms_arg = value("--forms").unwrap_or_else(|| "until,replay,eval".to_string());
    let forms: Vec<&str> = forms_arg.split(',').collect();

    let ctx = luma::default_context().expect("open GPU (lazy Luma)");
    let old = flag("--old").then(|| luma_old::default_context().expect("open GPU (Luma 0.2.4 of bowshock-gpu)"));
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let mut case = ConeCase::new(2.0, 15.0, ni, nj);
    case.first_order_iters = 0;
    let host = build_solver(&case, tm.beta);
    let u0 = host.u.clone();
    say(&format!("hang driver: {ni}x{nj}, {staging_name}, prec {prec}, forms {forms_arg}, n {n}, sync {sync}, reps {reps}, old {}", old.is_some()));
    for rep in 0..reps {
        say(&format!("rep {rep}"));
        if let Some(old) = &old {
            for prec in [Precision::F32, Precision::Extended] {
                let t0 = Instant::now();
                let mut g = GpuSolver::new_with(old, &host.grid, &host.cfg, Mode::Guarded, prec).unwrap();
                g.run(1).unwrap();
                g.run(sync).unwrap();
                for _ in 0..n / sync {
                    g.run(sync).unwrap();
                }
                g.context().sync().unwrap();
                say(&format!("  custom {prec:?} {:.2} s", t0.elapsed().as_secs_f64()));
            }
        }
        for &staging in &stagings {
            say(&format!(" staging {staging:?}"));
            if prec != "df32" {
                lazy!(f32s, staging, &ctx, host, &u0, n, sync, forms, "f32");
            }
            if prec != "f32" {
                lazy!(df32s, staging, &ctx, host, &u0, n, sync, forms, "df32");
            }
        }
    }
    say("done");
}
