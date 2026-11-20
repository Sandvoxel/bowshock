//! End to end over real ssh. Needs `BOWSHOCK_REMOTE_TEST_HOST=user@host` (a machine with a GPU and key-based ssh):
//!
//! `BOWSHOCK_REMOTE_TEST_HOST=sandvoxel@192.168.1.88 cargo test --release -p bowshock-remote --test remote_e2e -- --ignored --nocapture --test-threads=1`
//!
//! The tests delete `~/.cache/bowshock/worker` on that machine first (set `BOWSHOCK_REMOTE_TEST_KEEP=1` to keep it).

use bowshock_core::{AftSpec, Analysis, AnalysisSpec, Backend, BodySpec, Condition, Error, FinProfile, FinSpec, NoseShape, NoseSpec, Resolution};
use bowshock_remote::provision::{sh_command, NO_PREBUILT_ENV};
use bowshock_remote::{Remote, RemoteError, RemoteHost, SetupStatus};
use std::ops::ControlFlow;
use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use std::time::Instant;

const D: f64 = 0.04;

fn host() -> RemoteHost {
    let target = std::env::var("BOWSHOCK_REMOTE_TEST_HOST").expect("set BOWSHOCK_REMOTE_TEST_HOST=user@host");
    RemoteHost { target, port: None }
}

fn wipe(host: &RemoteHost) {
    if std::env::var_os("BOWSHOCK_REMOTE_TEST_KEEP").is_some() {
        return;
    }
    let out = Command::new("ssh").args(["-o", "BatchMode=yes", &host.target]).arg(sh_command("rm -rf \"$HOME/.cache/bowshock/worker\"")).output().expect("ssh");
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
}

/// Connects and prints the setup steps with times.
fn connect(host: &RemoteHost) -> (Remote, f64) {
    let t = Instant::now();
    let last = Mutex::new(String::new());
    let build_lines = AtomicUsize::new(0);
    let remote = Remote::connect(host, &|s| {
        let label = match &s {
            SetupStatus::Building { line } => {
                let mut l = last.lock().unwrap();
                *l = line.clone();
                if build_lines.fetch_add(1, Ordering::SeqCst).is_multiple_of(25) {
                    println!("  [{:7.2} s] build: {line}", t.elapsed().as_secs_f64());
                }
                return;
            }
            SetupStatus::Uploading { bytes } if bytes % (1 << 22) >= (1 << 20) => return,
            other => format!("{other:?}"),
        };
        println!("  [{:7.2} s] {label}", t.elapsed().as_secs_f64());
    })
    .unwrap_or_else(|e| panic!("connect failed: {e}\nlast build line: {}", last.lock().unwrap()));
    let s = t.elapsed().as_secs_f64();
    println!("connected in {s:.2} s to {:?}", remote.device());
    (remote, s)
}

fn body() -> BodySpec {
    let nose_len = 0.5 * D / 10f64.to_radians().tan();
    BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: D }, aft: vec![AftSpec::Tube { length: 10.0 * D - nose_len }] }
}

fn lift_spec() -> AnalysisSpec {
    let fins = FinSpec {
        count: 4,
        root_chord: D,
        tip_chord: D,
        span: D,
        sweep: 0.0,
        thickness: 0.08 * D,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Square,
    };
    Analysis::legacy(&body()).unwrap().fins(fins).backend(Backend::Gpu).resolution(Resolution::Coarse).spec().unwrap()
}

fn drag_spec() -> AnalysisSpec {
    Analysis::legacy(&body()).unwrap().backend(Backend::Gpu).resolution(Resolution::Coarse).spec().unwrap()
}

#[test]
#[ignore = "needs BOWSHOCK_REMOTE_TEST_HOST and takes minutes"]
fn provision_from_scratch_then_run_jobs_and_cancel() {
    let host = host();
    std::env::set_var(NO_PREBUILT_ENV, "1");
    wipe(&host);
    println!("first connect (build on the remote):");
    let (remote, first) = connect(&host);
    assert!(remote.device().gpu.is_some());
    drop(remote);
    println!("second connect (cached worker):");
    let (mut remote, second) = connect(&host);
    println!("FIRST CONNECT {first:.1} s, CACHED CONNECT {second:.2} s");

    // lift: 3D, α = 2°
    let spec = lift_spec();
    let frames = AtomicUsize::new(0);
    let t = Instant::now();
    let remote_lift = remote
        .solve(&spec, &[Condition::new(2.0, 2.0)], &|_| {
            frames.fetch_add(1, Ordering::SeqCst);
            ControlFlow::Continue(())
        })
        .unwrap();
    println!("remote lift {:.1} s, {} progress frames", t.elapsed().as_secs_f64(), frames.load(Ordering::SeqCst));
    assert!(frames.load(Ordering::SeqCst) > 0);
    let t = Instant::now();
    let local_lift = Analysis::from_spec(&spec).unwrap().run(&[Condition::new(2.0, 2.0)]).unwrap();
    println!("local lift {:.1} s", t.elapsed().as_secs_f64());
    let (r, l) = (&remote_lift.points[0], &local_lift.points[0]);
    let (rf, lf) = (r.forces.unwrap(), l.forces.unwrap());
    println!(
        "lift remote CN {:.5} xcp {:.5} ({} it) | local CN {:.5} xcp {:.5} ({} it)",
        rf.cn, rf.x_cp, r.detail.iterations, lf.cn, lf.x_cp, l.detail.iterations
    );
    assert!((rf.cn - lf.cn).abs() <= 0.005 * lf.cn.abs(), "CN {} vs {}", rf.cn, lf.cn);

    // drag: 2D Euler
    let spec = drag_spec();
    let frames = AtomicUsize::new(0);
    let t = Instant::now();
    let remote_drag = remote
        .solve(&spec, &[Condition::from(0.8), Condition::from(2.0)], &|_| {
            frames.fetch_add(1, Ordering::SeqCst);
            ControlFlow::Continue(())
        })
        .unwrap();
    println!("remote drag {:.1} s, {} progress frames", t.elapsed().as_secs_f64(), frames.load(Ordering::SeqCst));
    assert!(frames.load(Ordering::SeqCst) > 0);
    let local_drag = Analysis::from_spec(&spec).unwrap().run(&[0.8, 2.0]).unwrap();
    for (r, l) in remote_drag.points.iter().zip(&local_drag.points) {
        println!("drag M {}: remote Cd {:.5} local Cd {:.5}", r.mach, r.drag.total, l.drag.total);
        assert!((r.drag.total - l.drag.total).abs() <= 0.005 * l.drag.total.abs());
    }

    // cancel
    let t = Instant::now();
    let r = remote.solve(&spec_lift_again(), &[Condition::new(2.0, 2.0), Condition::new(2.0, 6.0)], &|_| ControlFlow::Break(()));
    println!("cancelled after {:.2} s", t.elapsed().as_secs_f64());
    assert!(matches!(r, Err(RemoteError::Solver(Error::Cancelled))), "{r:?}");
    // and the connection still works
    let again = remote.solve(&drag_spec(), &[Condition::from(2.0)], &|_| ControlFlow::Continue(())).unwrap();
    assert_eq!(again.points.len(), 1);
}

fn spec_lift_again() -> AnalysisSpec {
    lift_spec()
}

#[test]
#[ignore = "needs BOWSHOCK_REMOTE_TEST_HOST and a locally built bowshock-worker (cargo build --profile worker -p bowshock-worker)"]
fn a_local_worker_binary_of_the_same_platform_is_copied_instead_of_built() {
    let host = host();
    std::env::remove_var(NO_PREBUILT_ENV);
    wipe(&host);
    let (remote, s) = connect(&host);
    println!("connect with the copied binary: {s:.1} s");
    assert!(remote.device().gpu.is_some());
}

#[test]
fn an_unreachable_host_is_an_ssh_error() {
    let host = RemoteHost { target: "nobody@127.0.0.1".into(), port: Some(1) };
    assert!(matches!(Remote::connect(&host, &|_| {}), Err(RemoteError::Ssh(_))));
}
