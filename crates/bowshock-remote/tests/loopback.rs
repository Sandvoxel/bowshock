//! The client against the worker's serve loop in this process, over a TCP socket on localhost (what ssh's stdin/stdout carry).

use bowshock_core::{AftSpec, Analysis, AnalysisSpec, Backend, BodySpec, Condition, Error, FinProfile, FinSpec, NoseShape, NoseSpec, Resolution};
use bowshock_remote::proto::{read_frame, write_frame, ClientMsg, WorkerMsg, PROTOCOL};
use bowshock_remote::{worker, Remote, RemoteError, WORKER_KEY};
use std::net::{TcpListener, TcpStream};
use std::ops::ControlFlow;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

const D: f64 = 0.04;

fn body() -> BodySpec {
    let nose_len = 0.5 * D / 10f64.to_radians().tan();
    BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: D }, aft: vec![AftSpec::Tube { length: 10.0 * D - nose_len }] }
}

fn fins() -> FinSpec {
    FinSpec {
        count: 4,
        root_chord: D,
        tip_chord: D,
        span: D,
        sweep: 0.0,
        thickness: 0.08 * D,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Square,
    }
}

fn spec() -> AnalysisSpec {
    Analysis::legacy(&body()).unwrap().fins(fins()).backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(3000).spec().unwrap()
}

/// A fin-less Euler drag analysis.
fn drag_spec(iterations: usize) -> AnalysisSpec {
    AnalysisSpec { fins: None, max_iterations: Some(iterations), ..spec() }
}

/// The tiny 3D case of the facade's own tests at an angle of attack.
fn tiny(a: Analysis) -> Analysis {
    a.case3_hook(|c| {
        let s = &mut c.grid.sting;
        s.n_up = 10;
        s.n_nose = 16;
        s.n_seg = vec![12];
        s.n_ext = 10;
        s.nj = 24;
        c.grid.nk = 8;
    })
}

/// A connection to a worker thread that serves one client.
fn connect() -> (Remote, std::thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let server = std::thread::spawn(move || {
        let (s, _) = listener.accept().unwrap();
        worker::serve(s.try_clone().unwrap(), s, tiny).unwrap();
    });
    let s = TcpStream::connect(addr).unwrap();
    let remote = Remote::over_streams(s.try_clone().unwrap(), s, WORKER_KEY, false).unwrap();
    (remote, server)
}

/// The JSON of `x` with the residual drop rounded to 9 digits: the 3D CPU solver's residual is a parallel sum, whose last bits depend on how the threads were scheduled
/// (the points and everything else are compared exactly).
fn json<T: serde::Serialize>(x: &T) -> String {
    fn round(v: &mut serde_json::Value) {
        match v {
            serde_json::Value::Object(m) => {
                for (k, v) in m.iter_mut() {
                    match (k.as_str(), v.as_f64()) {
                        ("residual_drop_orders", Some(x)) => *v = serde_json::json!((x * 1e9).round() / 1e9),
                        _ => round(v),
                    }
                }
            }
            serde_json::Value::Array(a) => a.iter_mut().for_each(round),
            _ => {}
        }
    }
    let mut v = serde_json::to_value(x).unwrap();
    round(&mut v);
    serde_json::to_string(&v).unwrap()
}

#[test]
fn a_drag_job_matches_a_local_run_and_reports_progress() {
    let (mut remote, server) = connect();
    assert_eq!(remote.device().worker_key, WORKER_KEY);
    let spec = drag_spec(200);
    let local = Analysis::from_spec(&spec).unwrap().run(&[2.0]).unwrap();
    let frames = AtomicUsize::new(0);
    let polar = remote
        .solve(&spec, &[Condition::from(2.0)], &|p| {
            assert_eq!(p.mach, 2.0);
            frames.fetch_add(1, Ordering::SeqCst);
            ControlFlow::Continue(())
        })
        .unwrap();
    assert!(frames.load(Ordering::SeqCst) >= 1, "no progress frame");
    assert_eq!(json(&polar), json(&local));
    // the same connection serves the next job
    let again = remote.solve(&spec, &[Condition::from(2.0)], &|_| ControlFlow::Continue(())).unwrap();
    assert_eq!(json(&again), json(&local));
    drop(remote);
    server.join().unwrap();
}

#[test]
fn a_drag_job_that_keeps_fields_brings_back_the_field_of_a_local_run() {
    let (mut remote, server) = connect();
    let spec = AnalysisSpec { keep_fields: true, ..drag_spec(200) };
    let local = Analysis::from_spec(&spec).unwrap().run(&[2.0]).unwrap();
    let polar = remote.solve(&spec, &[Condition::from(2.0)], &|_| ControlFlow::Continue(())).unwrap();
    let (remote_field, local_field) = (polar.points[0].field.as_ref().expect("the worker kept the field"), local.points[0].field.as_ref().unwrap());
    assert!(remote_field.cells() > 1000);
    assert_eq!(remote_field, local_field);
    // without the switch the answer carries none
    let plain = remote.solve(&drag_spec(200), &[Condition::from(2.0)], &|_| ControlFlow::Continue(())).unwrap();
    assert!(plain.points[0].field.is_none());
    drop(remote);
    server.join().unwrap();
}

#[test]
fn a_job_at_angles_of_attack_matches_a_local_run() {
    let (mut remote, server) = connect();
    let spec = spec();
    let t = Instant::now();
    let local = tiny(Analysis::from_spec(&spec).unwrap()).run(&[Condition::new(2.0, 2.0), Condition::new(2.0, 6.0)]).unwrap();
    println!("local lift {:.1} s", t.elapsed().as_secs_f64());
    let frames = AtomicUsize::new(0);
    let polar = remote
        .solve(&spec, &[Condition::new(2.0, 2.0), Condition::new(2.0, 6.0)], &|_| {
            frames.fetch_add(1, Ordering::SeqCst);
            ControlFlow::Continue(())
        })
        .unwrap();
    assert!(frames.load(Ordering::SeqCst) >= 1);
    assert_eq!(polar.points.len(), 2);
    assert_eq!(json(&polar), json(&local));
    drop(remote);
    server.join().unwrap();
}

#[test]
fn a_job_that_keeps_fields_brings_back_the_3d_field_of_each_point_in_its_own_frame() {
    let (mut remote, server) = connect();
    let spec = AnalysisSpec { keep_fields: true, max_iterations: Some(150), ..spec() };
    let conditions = [Condition::new(2.0, 6.0), Condition::new(2.0, 2.0)];
    let local = tiny(Analysis::from_spec(&spec).unwrap()).run(&conditions).unwrap();
    let polar = remote.solve(&spec, &conditions, &|_| ControlFlow::Continue(())).unwrap();
    assert_eq!(polar.points.len(), 2);
    for (r, l) in polar.points.iter().zip(&local.points) {
        let (rf, lf) = (r.field3.as_ref().expect("the worker kept the field"), l.field3.as_ref().unwrap());
        assert!(rf.nk > 1 && rf.cells() > 10_000);
        assert_eq!((rf.alpha_deg.to_bits(), rf.mesh_id), (lf.alpha_deg.to_bits(), lf.mesh_id));
        assert_eq!(rf, lf, "the field that crossed the wire is the local one");
    }
    // without the switch the answer carries none
    let plain = remote.solve(&AnalysisSpec { keep_fields: false, ..spec }, &conditions[..1], &|_| ControlFlow::Continue(())).unwrap();
    assert!(plain.points[0].field3.is_none());
    drop(remote);
    server.join().unwrap();
}

#[test]
fn cancel_ends_the_job_and_the_connection_serves_the_next() {
    let (mut remote, server) = connect();
    let spec = spec();
    let seen = AtomicUsize::new(0);
    let t = Instant::now();
    let r = remote.solve(&spec, &[Condition::new(2.0, 2.0), Condition::new(2.0, 6.0)], &|_| {
        seen.fetch_add(1, Ordering::SeqCst);
        ControlFlow::Break(())
    });
    println!("cancelled after {:.2} s", t.elapsed().as_secs_f64());
    assert!(matches!(r, Err(RemoteError::Solver(Error::Cancelled))), "{r:?}");
    assert_eq!(seen.load(Ordering::SeqCst), 1, "the callback is not called again after Break");
    // a later job is not hit by the old cancel
    let ok = remote.solve(&drag_spec(60), &[Condition::from(2.0)], &|_| ControlFlow::Continue(())).unwrap();
    assert_eq!(ok.points.len(), 1);
    drop(remote);
    server.join().unwrap();
}

#[test]
fn solver_errors_cross_the_wire_with_their_kind() {
    let (mut remote, server) = connect();
    let r = remote.solve(&drag_spec(60), &[Condition::from(9.0)], &|_| ControlFlow::Continue(()));
    assert!(matches!(r, Err(RemoteError::Solver(Error::InvalidInput(_)))), "{r:?}");
    let r = remote.solve(&spec(), &[Condition::new(2.0, 30.0)], &|_| ControlFlow::Continue(()));
    assert!(matches!(r, Err(RemoteError::Solver(Error::InvalidInput(_)))), "{r:?}");
    drop(remote);
    server.join().unwrap();
}

#[test]
fn a_worker_of_another_version_is_refused() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let server = std::thread::spawn(move || {
        let (s, _) = listener.accept().unwrap();
        let _ = worker::serve(s.try_clone().unwrap(), s, |a| a);
    });
    let s = TcpStream::connect(addr).unwrap();
    let r = Remote::over_streams(s.try_clone().unwrap(), s, "not-the-worker-key", false);
    assert!(matches!(&r, Err(RemoteError::Protocol(m)) if m.contains("version")), "{:?}", r.err());
    server.join().unwrap();
}

#[test]
fn output_before_the_handshake_is_skipped_and_a_wrong_protocol_is_refused() {
    use std::io::Write;
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let server = std::thread::spawn(move || {
        let (mut s, _) = listener.accept().unwrap();
        s.write_all(b"Last login: yesterday\nWelcome!\n").unwrap();
        s.write_all(bowshock_remote::proto::MAGIC).unwrap();
        let mut r = s.try_clone().unwrap();
        let _hello: Option<ClientMsg> = read_frame(&mut r).unwrap();
        let device = worker::device_info();
        write_frame(&mut s, &WorkerMsg::Ready { protocol: PROTOCOL + 1, key: WORKER_KEY.into(), device }).unwrap();
    });
    let s = TcpStream::connect(addr).unwrap();
    let r = Remote::over_streams(s.try_clone().unwrap(), s, WORKER_KEY, false);
    assert!(matches!(&r, Err(RemoteError::Protocol(m)) if m.contains("protocol")), "{:?}", r.err());
    server.join().unwrap();
}

#[test]
fn messages_round_trip_through_cbor_frames() {
    let msg = WorkerMsg::Progress(bowshock_core::Progress {
        mach: 2.0,
        alpha: 0.03,
        iteration: 50,
        max_iterations: 3000,
        residual_drop_orders: 1.5,
        monitor: f64::NAN,
        spread: None,
        tolerance: Some(0.002),
    });
    let mut buf = Vec::new();
    write_frame(&mut buf, &msg).unwrap();
    let WorkerMsg::Progress(p) = read_frame::<_, WorkerMsg>(&mut buf.as_slice()).unwrap().unwrap() else { panic!() };
    assert!(p.monitor.is_nan() && p.iteration == 50 && p.tolerance == Some(0.002));
    let job = ClientMsg::Job { id: 3, job: bowshock_remote::Job::Solve { spec: spec(), conditions: vec![Condition::from(0.5), Condition::new(2.0, 6.0)] } };
    let mut buf = Vec::new();
    write_frame(&mut buf, &job).unwrap();
    assert_eq!(read_frame::<_, ClientMsg>(&mut buf.as_slice()).unwrap().unwrap(), job);
    assert!(read_frame::<_, ClientMsg>(&mut [].as_slice()).unwrap().is_none());
}
