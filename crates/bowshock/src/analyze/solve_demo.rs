//! Scripted solves for screenshots and tests: a developer hook, not a feature.
//!
//! A real solve of the example takes minutes and the headless driver (`bowshock drive`) waits for every running task before its
//! next step, so a screenshot of a solve *in progress* cannot be taken from a real one. With `BOWSHOCK_CFD_DEMO` set to a comma
//! separated list of [`Demo`] names, the next solves the screen starts play a scripted stream of the same [`cfd::Event`]s the
//! worker would send (the screen cannot tell them apart) instead of running the solver, one name per solve:
//!
//! ```text
//! BOWSHOCK_CFD_DEMO=mid cargo run -q -p bowshock -- drive --example crates/bowshock/scripts/cfd-progress.drive out/cfd-progress
//! ```
//!
//! The numbers are made up (a smooth transonic drag rise, iteration counts of the order the HANDOFF reports, one point that hits
//! the iteration cap) and so are the flow fields (`synthetic_field`: a cone shock, a stagnation region, a shoulder expansion and a
//! wake painted on the real grid of the body); nothing here comes from the solver. `mid` plays the picture of the point in flight
//! (at iterations 0, 500 and 1 000) and `done` the final field of every point. The real event stream is covered by the ignored tests
//! of [`super::cfd`].

use std::collections::VecDeque;

use bowshock_model::{CfdLiftPoint, CfdLiftTable, CfdPoint, CfdQuality};

use super::cfd::{self, Event, Job, Tally};
use super::solve::{Iteration, MachineInfo, MachineState, Outcome, PointDone};
use bowshock_core as solver;
use std::sync::Arc;

/// Which scripted solve to play.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Demo {
    /// A drag solve stopped partway: four points solved, the fifth running.
    Mid,
    /// A solve that finished: the drag table and, when the job has them, the 3D angles.
    Done,
    /// A drag solve that was cancelled with five points solved.
    Stopped,
    /// A solve whose drag table is done and whose 3D angles are stopped partway: the third Mach number is at its second angle.
    AngleMid,
    /// [`Demo::AngleMid`], cancelled.
    AngleStopped,
    /// A drag solve that is still setting up a remote GPU (building the worker there).
    RemoteSetup,
    /// A drag solve stopped partway on a remote GPU.
    RemoteMid,
    /// A drag solve shared by three machines (this computer, a remote GPU, one still building its worker), stopped partway with
    /// two points in flight.
    ShareMid,
    /// The angles of a solve shared by the same three machines, stopped partway.
    AngleShareMid,
    /// [`Demo::AngleMid`] with 3D flow fields: the finned finner of the fixture (`BOWSHOCK_FLOW3_FIXTURE`, default the one in
    /// `crates/bowshock/tests/data`) with its velocities scaled to the point's Mach number, as live snapshots and final fields.
    Flow3dMid,
    /// [`Demo::Done`] with 3D flow fields at the angle nodes.
    Flow3dDone,
}

impl Demo {
    fn parse(name: &str) -> Option<Demo> {
        Some(match name.trim() {
            "mid" => Demo::Mid,
            "done" => Demo::Done,
            "stopped" => Demo::Stopped,
            "angle-mid" => Demo::AngleMid,
            "angle-stopped" => Demo::AngleStopped,
            "remote-setup" => Demo::RemoteSetup,
            "remote-mid" => Demo::RemoteMid,
            "share-mid" => Demo::ShareMid,
            "angle-share-mid" => Demo::AngleShareMid,
            "flow3d-mid" => Demo::Flow3dMid,
            "flow3d-done" => Demo::Flow3dDone,
            _ => return None,
        })
    }
}

/// The scripts named by `BOWSHOCK_CFD_DEMO` (`Env::cfd_demo`), in order.
pub fn from_names(names: &[String]) -> VecDeque<Demo> {
    names.iter().filter_map(|n| Demo::parse(n)).collect()
}

/// The iteration cap the script's solver has (the real coarse viscous cap is 80 000; a smaller one keeps the stalled point's time
/// believable on a screenshot).
const CAP: usize = 30_000;
/// Seconds per iteration and per point of set-up the script's clock runs at.
const PER_ITERATION: f64 = 0.008;
const SETUP: f64 = 3.0;

/// A smooth zero-lift drag rise through the transonic range.
fn cd(mach: f64) -> f64 {
    0.40 + 0.30 * (-((mach - 1.15) / 0.45).powi(2)).exp()
}

/// Iterations of a point: more through the transonic range.
fn iterations(mach: f64) -> usize {
    let n = 2400.0 + 3800.0 * (-((mach - 1.0) / 0.3).powi(2)).exp();
    (n / 50.0).round() as usize * 50
}

/// The script's point at `mach`: the Mach number nearest 0.9 hits the iteration cap.
fn outcome(mach: f64, nodes: &[f64]) -> Outcome {
    let stalled = nodes.iter().copied().min_by(|a, b| (a - 0.9).abs().total_cmp(&(b - 0.9).abs()));
    if stalled == Some(mach) && (mach - 0.9).abs() < 0.2 {
        Outcome::Stalled
    } else if mach < 0.3 {
        Outcome::Settled
    } else {
        Outcome::Converged
    }
}

fn point(mach: f64, quality: CfdQuality) -> CfdPoint {
    let total = cd(mach);
    let (friction, base, fins) = (0.11, 0.09 + 0.02 * mach.min(1.0), 0.06);
    CfdPoint { mach, cd: total, friction, pressure: total - friction - base - fins, base, fins, other: 0.0, interference: 0.0, quality, restart: None }
}

fn quality(o: Outcome) -> CfdQuality {
    match o {
        Outcome::Converged => CfdQuality::Converged,
        Outcome::Settled => CfdQuality::Stationary,
        Outcome::Stalled | Outcome::Fallback => CfdQuality::Stalled,
    }
}

/// Snapshots of the drag point at `mach` at `iterations` (0 is the grid with the free stream on it), as a live local solve sends them.
fn snapshots(job: &Job, mach: f64, iterations: &[usize]) -> Vec<Event> {
    iterations.iter().filter_map(|&it| synthetic_field(job, mach, it)).map(|f| Event::Field(cfd::FieldRef(f))).collect()
}

/// The same for the 3D fixture's flow at `alpha_deg`.
fn fixture_snapshots(mach: f64, alpha_deg: f64, iterations: &[usize]) -> Vec<Event> {
    iterations.iter().filter_map(|&it| fixture_field(mach, alpha_deg, it)).map(|f| Event::Field(cfd::FieldRef(f))).collect()
}

/// The angle of attack of the fixture's flow (degrees).
const FIXTURE_ALPHA: f64 = 4.0;

/// The 3D fixture the `flow3d` demos play, read once.
fn fixture3d() -> Option<&'static Arc<solver::Field3>> {
    static FIXTURE: std::sync::OnceLock<Option<Arc<solver::Field3>>> = std::sync::OnceLock::new();
    FIXTURE
        .get_or_init(|| {
            let path =
                std::env::var("BOWSHOCK_FLOW3_FIXTURE").unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/anf_m2_a4.field3").to_string());
            let decoded = std::fs::read(&path).map_err(|e| e.to_string()).and_then(|b| super::flow_view::decode_fixture(&b));
            match decoded {
                Ok(f) => Some(Arc::new(f)),
                Err(e) => {
                    eprintln!("BOWSHOCK_CFD_DEMO=flow3d: {path}: {e}");
                    None
                }
            }
        })
        .as_ref()
}

/// A made-up 3D field after `iteration` iterations at `mach` and `alpha_deg`: the fixture's flow with its velocities scaled to the Mach
/// number, its disturbance (the part of the state and the wall pressures that differ from the free stream) scaled with the angle of
/// attack over the fixture's and with how far the solve has got (settled from iteration 2 000). Nothing here is solved.
fn fixture_field(mach: f64, alpha_deg: f64, iteration: usize) -> Option<Arc<solver::Field3>> {
    let base = fixture3d()?;
    let develop = if iteration >= 2000 { 1.0 } else { 1.0 - (-(iteration as f64) / 700.0).exp() } as f32;
    let strength = develop * (alpha_deg / FIXTURE_ALPHA) as f32;
    let speed = (mach / base.mach) as f32;
    let fs = base.freestream;
    let mut f = (**base).clone();
    f.mach = mach;
    f.alpha_deg = alpha_deg;
    f.iteration = iteration;
    f.freestream = [fs[0], fs[1] * f64::from(speed), fs[2], fs[3] * f64::from(speed), fs[4]];
    for (c, q) in f.prim.iter_mut().enumerate() {
        if f.solid[c] {
            continue;
        }
        let free = [fs[0] as f32, fs[1] as f32, fs[2] as f32, fs[3] as f32, fs[4] as f32];
        for m in 0..5 {
            q[m] = free[m] + strength * (q[m] - free[m]);
        }
        for m in 1..4 {
            q[m] *= speed;
        }
    }
    for w in &mut f.walls {
        w.cp *= strength;
    }
    Some(Arc::new(f))
}

/// A made-up flow field of the body of `job` at `mach` after `iteration` iterations, on the grid the real solver would use: a cone
/// shock (a bow shock for M just above 1, none below), the stagnation region at the nose, the suction at the shoulder, a thin
/// boundary layer and a dead-water wake behind the base. It is shaped like a flow, not computed from one; its shock grows out of the
/// body as the iterations go (settled from about 2 000).
fn synthetic_field(job: &Job, mach: f64, iteration: usize) -> Option<Arc<solver::Field3>> {
    let mut f = solver::Analysis::new(&job.plan.body)
        .ok()?
        .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: job.altitude.clamp(0.0, 20_000.0) } })
        .resolution(match job.grid {
            bowshock_model::CfdGrid::Coarse => solver::Resolution::Coarse,
            bowshock_model::CfdGrid::Draft => solver::Resolution::Draft,
            bowshock_model::CfdGrid::Medium => solver::Resolution::Medium,
        })
        .initial_field(mach)
        .ok()?;
    let develop = if iteration >= 2000 { 1.0 } else { 1.0 - (-(iteration as f64) / 700.0).exp() };
    if develop > 0.0 {
        paint(&mut f, develop);
    }
    f.iteration = iteration;
    Some(cfd::FieldRef::from_2d(&f).0)
}

/// Fills the flow cells of `f` with the made-up flow, `develop` in 0 to 1 of the way to its settled shape.
fn paint(f: &mut solver::Field, develop: f64) {
    let g = solver::field::GAMMA;
    let m = f.mach;
    // the body: its profile from the nose tip to the base corner
    let surface = f.surface();
    let start = surface.iter().position(|p| p[0] >= -1e-9).unwrap_or(0);
    let end = surface.iter().rposition(|p| p[1] > 1e-9).map_or(start, |e| e + 1);
    let profile: Vec<(f64, f64)> = surface[start..end].iter().map(|p| (f64::from(p[0]), f64::from(p[1]))).collect();
    let (Some(&(length, base_r)), true) = (profile.last(), profile.len() > 1) else { return };
    let radius = |x: f64| -> f64 {
        if !(0.0..=length).contains(&x) {
            return 0.0;
        }
        let i = profile.partition_point(|p| p.0 < x).clamp(1, profile.len() - 1);
        let (a, b) = (profile[i - 1], profile[i]);
        if b.0 > a.0 {
            a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0)
        } else {
            b.1
        }
    };
    let h = 0.01 * length;
    let slope = |x: f64| {
        let (a, b) = ((x - h).max(0.0), (x + h).min(length));
        (radius(b) - radius(a)) / (b - a).max(1e-9)
    };
    let nose_angle = (0..40).map(|k| slope(0.4 * length * f64::from(k) / 40.0)).fold(0.0f64, f64::max).atan();
    let r_max = profile.iter().map(|q| q.1).fold(0.0, f64::max);
    let shoulder = profile.iter().find(|p| p.1 >= 0.99 * r_max).map_or(0.3 * length, |p| p.0);

    // the shock: a cone of the Mach angle plus most of the nose angle, with a rounded tip; its strength fades behind the shoulder
    let supersonic = m > 0.97;
    // a transonic body gets a weak bow shock standing off the nose, as if it flew a little above Mach 1
    let m_shock = m.max(1.12);
    let beta = ((1.0 / m_shock).asin() + 0.9 * nose_angle).min(1.45);
    let (x_a, stand) = (-0.03 * length, 0.02 * length * (1.0 + 6.0 / (m_shock * m_shock)));
    let shock_r = |x: f64| beta.tan() * ((x - x_a).max(0.0) * (x - x_a + 2.0 * stand)).sqrt();
    let ms2 = (m_shock * beta.sin()).powi(2);
    let p_ratio = 1.0 + 2.0 * g / (g + 1.0) * (ms2 - 1.0).max(0.0);
    let rho_ratio = (g + 1.0) * ms2 / ((g - 1.0) * ms2 + 2.0);
    let entropy = rho_ratio / p_ratio.powf(1.0 / g);
    let cp_shock = 2.0 * (p_ratio - 1.0) / (g * m * m);
    let cp_wall = |x: f64| -> f64 {
        let t = slope(x).atan();
        let dip = if x > shoulder { -0.1 * (-(x - shoulder) / (0.05 * length)).exp() } else { 0.0 };
        (2.0 * t.sin().powi(2) + dip) * if x <= length { 1.0 } else { 0.0 }
    };
    let amp = 0.4 + 0.6 * develop;
    let (p_inf, h0) = (1.0 / g, 1.0 / (g - 1.0) + 0.5 * m * m);
    let width = 0.006 * length;

    let (ni, nj) = (f.ni, f.nj);
    for j in 0..nj {
        for i in 0..ni {
            let c = j * ni + i;
            if f.solid[c] {
                continue;
            }
            let (x, r) = f.centre(i, j);
            let (x, r) = (f64::from(x), f64::from(r));
            let rb = radius(x);
            let d = (r - rb).max(0.0);
            let wake =
                if x > length { (-(x - length) / (3.5 * base_r)).exp() * if r < base_r * 1.05 { 1.0 } else { (-(r - base_r) / base_r).exp() } } else { 0.0 };
            let (mut cp, mut inside, mut front) = (0.0, 0.0, 0.0);
            if supersonic {
                let rs = rb + (shock_r(x) - rb) * develop;
                if x > x_a && rs > rb {
                    front = 1.0 / (1.0 + (-(rs - r) / width).exp());
                    inside = ((r - rb) / (rs - rb)).clamp(0.0, 1.0);
                    let fade = if x > shoulder { 1.0 / (1.0 + (x - shoulder) / (0.9 * length)) } else { 1.0 };
                    cp = (cp_wall(x) + (cp_shock * fade - cp_wall(x)) * inside * inside) * front;
                }
            } else {
                cp = cp_wall(x) * (-d / (0.12 * length)).exp() / (1.0 - (m * m).min(0.9)).sqrt();
            }
            cp = cp * amp - 0.12 * wake * develop;
            let p = p_inf + cp * 0.5 * m * m;
            let rho = (p / p_inf).max(0.05).powf(1.0 / g) * (1.0 + (entropy - 1.0) * front);
            let mut speed = (2.0 * (h0 - g / (g - 1.0) * p / rho)).max(0.01).sqrt();
            if (0.0..=length).contains(&x) {
                let delta = 0.0025 + 0.01 * (x / length);
                speed *= (d / delta).tanh().max(0.0);
            }
            speed *= 1.0 - 0.85 * wake * develop;
            let angle = if supersonic { slope(x).atan() * (1.0 - inside) } else { slope(x).atan() * (-d / (0.1 * length)).exp() };
            f.rho[c] = rho as f32;
            f.u[c] = (speed * angle.cos()) as f32;
            f.v[c] = (speed * angle.sin()) as f32;
            f.p[c] = p as f32;
        }
    }
}

/// Reports of a unit settling from a transient: the monitor `target` approached with a decaying wobble, a report every 50
/// iterations up to `until`, the clock starting at `from` (s) and running at the script's rate, reported by `machine`.
fn trace(machine: usize, mach: f64, alpha_deg: Option<f64>, target: f64, from: f64, until: usize) -> Vec<Event> {
    let mut samples: Vec<f64> = Vec::new();
    (1..=until / 50)
        .map(|k| {
            let it = k * 50;
            let m = target * (1.0 + 0.08 * (-(it as f64) / 500.0).exp() * (it as f64 / 90.0).cos());
            samples.push(m);
            let window = &samples[samples.len().saturating_sub(10)..];
            let (lo, hi) = window.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), v| (l.min(*v), h.max(*v)));
            let spread = (window.len() == 10).then(|| (hi - lo) / (window.iter().sum::<f64>() / 10.0));
            Event::Iteration(Iteration {
                machine,
                at: from + it as f64 * PER_ITERATION,
                mach,
                alpha_deg,
                iteration: it,
                max_iterations: CAP,
                residual_orders: (it as f64 / 450.0).min(3.6),
                monitor: m,
                spread,
                tolerance: Some(0.002),
            })
        })
        .collect()
}

/// The machines of a script and where each is: the first events of every script.
fn fleet(demo: Demo) -> Vec<Event> {
    let info = |name: &str, remote: bool| MachineInfo { name: name.to_string(), remote };
    let ready = |index: usize, device: &str| Event::Machine { index, at: 0.0, state: MachineState::Ready(device.to_string()) };
    let building = |index: usize| Event::Machine {
        index,
        at: 0.0,
        state: MachineState::Connecting("Building the worker: Compiling bowshock-cfd v0.1.0 (31/118)".to_string()),
    };
    match demo {
        Demo::RemoteSetup => vec![Event::Machines(vec![info("omarchy", true)]), building(0)],
        Demo::RemoteMid => vec![Event::Machines(vec![info("omarchy", true)]), ready(0, "NVIDIA GeForce RTX 3080 Laptop GPU on omarchy")],
        Demo::ShareMid | Demo::AngleShareMid => vec![
            Event::Machines(vec![info("This computer", false), info("omarchy", true), info("lab", true)]),
            ready(0, "NVIDIA GeForce RTX 3090"),
            ready(1, "NVIDIA GeForce RTX 3080 Laptop GPU on omarchy"),
            building(2),
        ],
        _ => vec![Event::Machines(vec![info("This computer", false)]), ready(0, "Apple M1 Max")],
    }
}

/// How far a script has got.
struct Shape {
    /// Drag nodes solved.
    drag: usize,
    /// Stability Mach numbers solved (both angles).
    stability: usize,
    /// What follows the solved nodes.
    end: End,
}

enum End {
    /// The solve is over, with its table.
    Finished,
    /// The solve is cancelled.
    Cancelled,
    /// The solve is still running: the next node of the part that has not finished is in flight.
    Running,
}

impl Demo {
    fn shape(self, nodes: usize, stability: usize) -> Shape {
        let half = (stability / 2).min(stability.saturating_sub(1));
        match self {
            Demo::Mid | Demo::RemoteMid | Demo::ShareMid => Shape { drag: 4, stability: 0, end: End::Running },
            Demo::Stopped => Shape { drag: 5.min(nodes), stability: 0, end: End::Cancelled },
            Demo::AngleMid | Demo::Flow3dMid => Shape { drag: nodes, stability: half, end: End::Running },
            Demo::AngleStopped => Shape { drag: nodes, stability: half, end: End::Cancelled },
            Demo::AngleShareMid => Shape { drag: nodes, stability: 2.min(stability.saturating_sub(2)), end: End::Running },
            Demo::Done | Demo::Flow3dDone | Demo::RemoteSetup => Shape { drag: nodes, stability, end: End::Finished },
        }
    }

    /// The angle part has 3D fields (the fixture's), live and final.
    fn fields3d(self) -> bool {
        matches!(self, Demo::Flow3dMid | Demo::Flow3dDone)
    }
}

/// The events of a scripted solve of `job`: its drag nodes, then (when the job has stability nodes) its angles.
pub fn events(demo: Demo, job: &Job) -> Vec<Event> {
    let nodes = &job.nodes;
    let mut out = fleet(demo);
    if demo == Demo::RemoteSetup {
        return out;
    }
    let shape = demo.shape(nodes.len(), job.stability.len());
    // Shared by three machines, the first points were taken by the two that were ready.
    let owner = |i: usize| if demo == Demo::ShareMid { [0, 1, 0, 0][i % 4] } else { 0 };
    let mut at = 0.0;
    let mut tally = Tally { points: Vec::new(), notes: job.plan.notes.clone(), gpu: 0, cpu: 0, fallback: None };
    for (i, &mach) in nodes[..shape.drag].iter().enumerate() {
        let o = outcome(mach, nodes);
        let its = if o == Outcome::Stalled { CAP } else { iterations(mach) };
        let p = point(mach, quality(o));
        at += SETUP + its as f64 * PER_ITERATION;
        out.extend(synthetic_field(job, mach, its).map(|field| Event::PointField { field: cfd::FieldRef(field) }));
        out.push(Event::Point(PointDone {
            machine: owner(i),
            at,
            mach,
            alpha_deg: None,
            outcome: o,
            point: Some(p),
            lift: None,
            iterations: its,
            cpu: false,
            fallback: None,
        }));
        tally.points.push(p);
        tally.gpu += 1;
    }
    let angles = angle_events(demo, job, &shape, &mut out, &mut at);
    if shape.drag < nodes.len() {
        // the next drag point is in flight
        match demo {
            Demo::Mid => {
                // it has reached iteration 1 450 and has shown its flow at 500 and 1 000
                let mach = nodes[shape.drag];
                out.extend(trace(0, mach, None, cd(mach), at + SETUP, 1450));
                out.extend(snapshots(job, mach, &[0, 500, 1000]));
            }
            Demo::RemoteMid => {
                // a remote machine sends no snapshots
                let mach = nodes[shape.drag];
                out.extend(trace(0, mach, None, cd(mach), at + SETUP, 1450));
            }
            Demo::ShareMid => {
                // the next two points are in flight, one on each machine that is ready
                let (a, b) = (nodes[shape.drag], nodes[shape.drag + 1]);
                out.extend(trace(0, a, None, cd(a), at + SETUP, 1450));
                out.extend(snapshots(job, a, &[0, 500, 1000]));
                out.extend(trace(1, b, None, cd(b), at + SETUP, 800));
            }
            _ => out.extend(snapshots(job, nodes[shape.drag], &[0, 500])),
        }
    }
    match shape.end {
        End::Running => {}
        End::Cancelled => out.push(Event::Finished(Err(cfd::CANCELLED.to_string()))),
        End::Finished => {
            let mut table = cfd::table_from(job, tally, nodes.len(), at);
            table.lift = angles.map(|points| CfdLiftTable {
                key: cfd::lift_key(job.plan.geometry, &job.stability),
                solved_at: cfd::now(),
                solve_seconds: at,
                backend: "GPU (Luma)".to_string(),
                points,
                notes: Vec::new(),
            });
            out.push(Event::Finished(Ok(table)));
        }
    }
    out
}

/// The stability part of the script (`shape.stability` Mach numbers solved, then the one or two in flight): appends its events to `out`
/// and returns the points of the solved ones, `None` when the job has no stability nodes.
fn angle_events(demo: Demo, job: &Job, shape: &Shape, out: &mut Vec<Event>, at: &mut f64) -> Option<Vec<CfdLiftPoint>> {
    let nodes = &job.stability;
    if nodes.is_empty() || shape.drag < job.nodes.len() {
        return None;
    }
    let owner = |i: usize| if demo == Demo::AngleShareMid { i % 2 } else { 0 };
    let mut points: Vec<CfdLiftPoint> = Vec::new();
    // CN grows with the angle (slightly sub-linearly) and with Mach up to the transonic range; CP sits about 60 % of the length aft.
    let lift = |mach: f64, a: f64| CfdLiftPoint {
        mach,
        alpha_deg: a,
        cn: a.to_radians() * (2.4 + 0.5 * mach.min(1.2)) * (1.0 - 0.01 * a),
        x_cp: 0.85 + 0.04 * mach.min(1.5),
        cd: None,
        cm: None,
        quality: CfdQuality::Converged,
    };
    for (i, &mach) in nodes[..shape.stability].iter().enumerate() {
        let mut hand = Vec::new();
        for (k, &a) in cfd::LIFT_ALPHAS.iter().enumerate() {
            let its = if k == 0 { 1800 } else { 700 };
            out.extend(trace(owner(i), mach, Some(a), lift(mach, a).cn, *at + SETUP, its));
            *at += SETUP + its as f64 * PER_ITERATION;
            let p = lift(mach, a);
            if demo.fields3d() {
                out.extend(fixture_field(mach, a, its).map(|field| Event::PointField { field: cfd::FieldRef(field) }));
            }
            hand.push(PointDone {
                machine: owner(i),
                at: 0.0,
                mach,
                alpha_deg: Some(a),
                outcome: Outcome::Converged,
                point: None,
                lift: Some(p),
                iterations: its,
                cpu: false,
                fallback: None,
            });
            points.push(p);
        }
        for mut d in hand {
            d.at = *at;
            out.push(Event::Point(d));
        }
    }
    if shape.stability < nodes.len() {
        let [a2, a6] = cfd::LIFT_ALPHAS;
        if demo == Demo::AngleShareMid {
            // the next two Mach numbers, one on each machine that is ready: the first has run its first angle and is at iteration 600
            // of its second; the other is at iteration 900 of its first
            let (a, b) = (nodes[shape.stability], nodes[shape.stability + 1]);
            out.extend(trace(0, a, Some(a2), lift(a, a2).cn, *at + SETUP, 1800));
            out.extend(trace(0, a, Some(a6), lift(a, a6).cn, *at + SETUP + 1800.0 * PER_ITERATION, 600));
            out.extend(trace(1, b, Some(a2), lift(b, a2).cn, *at + SETUP, 900));
        } else {
            // the next Mach number: its first angle has run, its second is at iteration 600, and shows its flow at 500 and 1 000
            let mach = nodes[shape.stability];
            out.extend(trace(0, mach, Some(a2), lift(mach, a2).cn, *at + SETUP, 1800));
            *at += SETUP + 1800.0 * PER_ITERATION;
            out.extend(trace(0, mach, Some(a6), lift(mach, a6).cn, *at, 600));
            if demo.fields3d() {
                out.extend(fixture_snapshots(mach, a6, &[0, 500, 1000]));
            }
        }
    }
    Some(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analyze::solve::{Slot, Tracker};

    /// A tracker fed the events of a script: the machines, iterations and points (not the end).
    fn tracked(units: Vec<crate::analyze::solve::Unit>, events: &[Event]) -> Tracker {
        let mut t = Tracker::new(units);
        for e in events {
            match e {
                Event::Machines(m) => t.set_machines(m),
                Event::Machine { index, at, state } => t.on_machine(*index, *at, state.clone()),
                Event::Iteration(i) => t.on_iteration(i),
                Event::Point(p) => t.on_done(p),
                _ => {}
            }
        }
        t
    }

    #[test]
    fn the_names_parse_in_order_and_unknown_names_are_skipped() {
        let parsed: VecDeque<Demo> = "done, angle-mid,nonsense,stopped".split(',').filter_map(Demo::parse).collect();
        assert_eq!(parsed, VecDeque::from([Demo::Done, Demo::AngleMid, Demo::Stopped]));
    }

    #[test]
    fn the_script_is_the_stream_the_tracker_expects() {
        let nodes = [0.1, 0.5, 0.8, 0.9, 1.0, 1.1, 1.3, 1.7, 2.3, 3.0];
        let job = Job {
            plan: crate::analyze::cfd::test_plan(),
            altitude: 0.0,
            nodes: nodes.to_vec(),
            stage: 1,
            grid: bowshock_model::CfdGrid::Coarse,
            compute: Default::default(),
            cpu_only: false,
            stability: Vec::new(),
        };
        let t = tracked(Tracker::drag_units(&nodes), &events(Demo::Mid, &job));
        assert_eq!(t.done_count(), 4);
        assert_eq!(t.lives().next().unwrap().1.iteration, 1450);
        assert!(matches!(t.slot(3), Slot::Done(d) if d.outcome == Outcome::Stalled), "Mach 0.9 hit the cap");
        assert!(t.eta().is_some(), "four points are solved: the estimate is up");
    }

    // The screen-level tests below play scripted solves through the real `State::update`: choosing the method starts a (scripted)
    // solve, the script's events go in as the worker's would, and what the app and the document hold afterwards is what the user
    // would have. Nothing runs a solver.

    use crate::analyze::{solve_view, Message, Method};
    use crate::app::{App, Message as AppMessage, Mode};

    /// The Analyze screen with the CFD table method chosen and `scripts` queued: the drag solve has started (scripted).
    fn app_with(scripts: &[Demo]) -> (App, u64) {
        app_with_angles(scripts, true)
    }

    /// [`app_with`], with the 3D angles option set as given before the solve starts.
    fn app_with_angles(scripts: &[Demo], angles: bool) -> (App, u64) {
        let mut app = App::with_example();
        app.analyze_mut().cfd.angles = angles;
        app.analyze_mut().cfd.demo = scripts.iter().copied().collect();
        drop(app.update(AppMessage::Mode(Mode::Analyze)));
        drop(app.update(AppMessage::Analyze(Message::Method(Method::CfdTable))));
        let serial = app.analyze().cfd.solving.as_ref().expect("choosing the CFD table method starts a solve").stream.serial();
        (app, serial)
    }

    fn feed(app: &mut App, serial: u64, events: Vec<Event>) {
        for e in events {
            drop(app.update(AppMessage::Analyze(Message::Cfd(serial, e))));
        }
    }

    fn press(app: &mut App, message: Message) {
        drop(app.update(AppMessage::Analyze(message)));
    }

    fn job(app: &App) -> std::sync::Arc<Job> {
        app.analyze().cfd.solving.as_ref().and_then(|s| s.job.clone()).expect("a drag solve is running")
    }

    fn notice(app: &App) -> String {
        app.ctx().notice.as_ref().map(|n| n.0.clone()).unwrap_or_default()
    }

    #[test]
    fn a_solve_in_progress_has_its_point_its_partial_curve_and_an_estimate_and_stores_nothing() {
        let (mut app, serial) = app_with(&[Demo::Mid]);
        let script = events(Demo::Mid, &job(&app));
        feed(&mut app, serial, script);
        let s = app.analyze().cfd.solving.as_ref().expect("still solving");
        let t = &s.tracker;
        let total = t.total();
        assert!(total >= 6, "the example's table has its Mach nodes: {total}");
        assert_eq!((t.done_count(), t.lives().next().map(|(u, l)| (u.mach, l.iteration))), (4, Some((t.units()[4].mach, 1450))));
        assert_eq!(t.device(), "Apple M1 Max");
        assert!(t.fraction() > 0.0 && t.fraction() < 1.0 && t.eta().is_some_and(|s| s > 0.0));
        // the curve: four solved points (one flagged), the one in flight and the rest to come
        let data = solve_view::plot_data(t, None);
        assert_eq!(data.curves[0].solved.len(), 4);
        assert_eq!(data.curves[0].solved.iter().filter(|m| !m.outcome.is_settled()).count(), 1);
        assert_eq!(data.curves[0].pending.len(), job(&app).nodes.len() - 4, "the drag part of the solve is on the chart; the angles come after it");
        assert!(!data.live.is_empty());
        assert!(app.ctx().doc.rocket().cfd_table(1).is_none(), "nothing is stored before the solve ends");
        assert!(solve_view::active(app.analyze()));
    }

    #[test]
    fn cancelling_keeps_the_solved_points_on_offer_and_keep_stores_them_as_a_partial_table() {
        let (mut app, serial) = app_with(&[Demo::Mid]);
        let nodes = job(&app).nodes.clone();
        let script = events(Demo::Mid, &job(&app));
        feed(&mut app, serial, script);
        press(&mut app, Message::CfdCancel);
        assert!(app.analyze().cfd.solving.is_none(), "the solve is over");
        assert!(notice(&app).contains("Cancelled after 4 of"), "{}", notice(&app));
        let stopped = app.analyze().cfd.stopped.as_ref().expect("four points are on offer");
        assert_eq!(stopped.table.as_ref().map(|t| t.points.len()), Some(4));
        assert_eq!(stopped.keep_label, "Keep 4 solved points");
        assert!(app.ctx().doc.rocket().cfd_table(1).is_none(), "nothing is stored until Keep");
        // still on show, with the Keep and Discard buttons
        assert!(solve_view::active(app.analyze()));
        // no new solve starts by itself
        press(&mut app, Message::Ignore);
        assert!(app.analyze().cfd.solving.is_none());

        press(&mut app, Message::CfdKeep);
        assert!(app.analyze().cfd.stopped.is_none());
        let kept = app.ctx().doc.rocket().cfd_table(1).expect("the points were kept");
        assert_eq!(kept.points.len(), 4);
        assert_eq!(kept.points.iter().map(|p| p.mach).collect::<Vec<_>>(), nodes[..4]);
        assert!(notice(&app).starts_with("Kept 4 solved points"), "{}", notice(&app));
        // it reads as a partial table of the range, not as a fresh one, and nothing re-solves behind the user's back
        let (c, cond) = (&app.analyze().cfd, &app.analyze().cond);
        assert_eq!(c.status, crate::analyze::cfd::Status::Stale);
        assert_eq!(crate::analyze::cfd::partial_of(&c.plan, c.table.as_ref().unwrap(), cond.altitude, cond.mach), Some((4, nodes.len())));
        assert!(!solve_view::active(app.analyze()));
    }

    #[test]
    fn discard_drops_the_points_and_a_cancel_with_fewer_than_two_points_offers_none() {
        let (mut app, serial) = app_with(&[Demo::Mid, Demo::Mid]);
        let script = events(Demo::Mid, &job(&app));
        feed(&mut app, serial, script);
        press(&mut app, Message::CfdCancel);
        assert!(app.analyze().cfd.stopped.is_some());
        press(&mut app, Message::CfdDiscard);
        assert!(app.analyze().cfd.stopped.is_none() && app.ctx().doc.rocket().cfd_table(1).is_none());
        assert!(!solve_view::active(app.analyze()));

        // Re-solve; this time the cancel comes after one point only
        press(&mut app, Message::Resolve);
        let serial = app.analyze().cfd.solving.as_ref().expect("Re-solve starts a solve").stream.serial();
        // the machines, then the first point's final field and the point itself
        let first: Vec<Event> = events(Demo::Mid, &job(&app)).into_iter().take_while(|e| !matches!(e, Event::Iteration(_))).take(4).collect();
        assert!(matches!(first.last(), Some(Event::Point(_))));
        feed(&mut app, serial, first);
        press(&mut app, Message::CfdCancel);
        assert!(app.analyze().cfd.solving.is_none() && app.analyze().cfd.stopped.is_none(), "one point is not a table");
        assert_eq!(notice(&app), "CFD solve cancelled");
        assert!(app.ctx().doc.rocket().cfd_table(1).is_none());
    }

    #[test]
    fn a_finished_solve_is_stored_and_summarised() {
        let (mut app, serial) = app_with_angles(&[Demo::Done], false);
        let script = events(Demo::Done, &job(&app));
        feed(&mut app, serial, script);
        assert!(app.analyze().cfd.solving.is_none());
        let table = app.ctx().doc.rocket().cfd_table(1).expect("the table is stored");
        let total = table.points.len();
        // the toast is short (the status strip is narrow); the row under the provenance line has the whole account
        let toast = notice(&app);
        assert_eq!(toast, format!("Solved {total} points · 1 not converged · 8 min 53 s"));
        let (key, row) = app.analyze().cfd.summary.clone().expect("the summary row");
        assert_eq!(key, table.key);
        assert!(row.starts_with(&format!("Solved {total} Mach points in ")), "{row}");
        assert!(row.contains(" on Apple M1 Max · ") && row.contains("1 not settled (Mach 0.90)"), "{row}");
        assert!(row.contains(&format!("{} converged", total - 1)), "{row}");
        assert_eq!(app.analyze().cfd.status, crate::analyze::cfd::Status::Fresh);
        assert!(!solve_view::active(app.analyze()));
    }

    #[test]
    fn the_script_runs_the_angles_after_the_drag_nodes_in_one_stream() {
        let (app, _) = app_with(&[Demo::AngleMid]);
        let job = job(&app);
        assert!(!job.stability.is_empty(), "the angles are on by default: the job has stability nodes");
        let t = tracked(job.units(), &events(Demo::AngleMid, &job));
        let (drag, stab) = (job.nodes.len(), job.stability.len());
        assert_eq!(t.total(), drag + 2 * stab, "a unit per drag node, then one per Mach number and angle");
        assert_eq!(t.phase(), crate::analyze::solve::Phase::Stability);
        let (unit, live) = t.lives().next().unwrap();
        assert_eq!((unit.alpha_deg, live.iteration), (Some(cfd::LIFT_ALPHAS[1]), 600));
        let kept = stab / 2;
        assert_eq!(t.done_count(), drag + 2 * kept);
        assert_eq!(t.stability_kept().1, kept);
        // the picture is one curve per angle once the angles run
        assert_eq!(solve_view::plot_data(&t, None).curves.len(), 2);
        // the whole stream ends in one table with both parts
        let done = events(Demo::Done, &job);
        let Some(Event::Finished(Ok(table))) = done.last() else { panic!("a finished solve ends in its table") };
        assert_eq!(table.points.len(), drag);
        assert_eq!(table.lift.as_ref().map(|l| l.points.len()), Some(2 * stab));
        assert_eq!(table.lift.as_ref().map(|l| l.key), Some(cfd::lift_key(job.plan.geometry, &job.stability)));
    }

    #[test]
    fn a_solve_without_angles_is_the_drag_table_alone_and_asking_for_them_later_makes_it_stale() {
        let (mut app, serial) = app_with_angles(&[Demo::Done], false);
        let j = job(&app);
        assert!(j.stability.is_empty(), "the option is off: the job has drag nodes only");
        assert_eq!(solve_view::plot_data(&Tracker::new(j.units()), None).curves.len(), 1);
        feed(&mut app, serial, events(Demo::Done, &j));
        let table = app.ctx().doc.rocket().cfd_table(1).expect("stored");
        assert!(table.lift.is_none());
        assert_eq!(app.analyze().solve_status(), cfd::Status::Fresh, "asked for the drag alone, the drag is the whole table");
        // asking for the angles makes it stale (the table is shown meanwhile); turning them off again makes it fresh
        press(&mut app, Message::Angles(true));
        assert_eq!((app.analyze().solve_status(), app.analyze().cfd.status.clone()), (cfd::Status::Stale, cfd::Status::Fresh));
        press(&mut app, Message::Angles(false));
        assert_eq!(app.analyze().solve_status(), cfd::Status::Fresh);
    }

    #[test]
    fn a_cancel_in_the_angles_keeps_the_drag_table_and_the_mach_numbers_with_both_angles() {
        let (mut app, serial) = app_with(&[Demo::AngleStopped]);
        let j = job(&app);
        feed(&mut app, serial, events(Demo::AngleStopped, &j));
        assert!(app.analyze().cfd.solving.is_none(), "the script ended in the cancel");
        let stopped = app.analyze().cfd.stopped.as_ref().expect("the drag points and the angles solved are on offer");
        let table = stopped.table.as_ref().expect("a table");
        let kept = j.stability.len() / 2;
        assert_eq!(table.points.len(), j.nodes.len());
        let lift = table.lift.as_ref().expect("the 3D angles that were in");
        assert_eq!(lift.points.len(), 2 * kept);
        assert_eq!(lift.key, cfd::lift_key(j.plan.geometry, &j.stability[..kept]), "the key is the partial list's: it reads as stale");
        assert_eq!(stopped.keep_label, format!("Keep {} points · {} at {kept} Mach", j.nodes.len(), if three_d() { "angles" } else { "3D" }));
        assert!(
            table
                .notes
                .iter()
                .any(|n| n.starts_with("Stopped early") && n.contains(if three_d() { "angle Mach numbers solved" } else { "3D Mach numbers solved" })),
            "{:?}",
            table.notes
        );
        assert!(app.ctx().doc.rocket().cfd_table(1).is_none(), "nothing is stored until Keep");
        press(&mut app, Message::CfdKeep);
        let stored = app.ctx().doc.rocket().cfd_table(1).expect("kept").clone();
        assert_eq!(stored.lift.as_ref().map(|l| l.points.len()), Some(2 * kept));
        assert_eq!(notice(&app), format!("Kept {} points, {} at {kept} Mach numbers", j.nodes.len(), if three_d() { "angles" } else { "3D" }));
        // it is a partial table of the range, not a fresh one
        assert_eq!(app.analyze().solve_status(), cfd::Status::Stale);
    }

    #[test]
    fn a_mach_number_with_only_one_of_its_angles_is_left_out_of_a_stopped_table() {
        let (app, _) = app_with(&[Demo::AngleMid]);
        let j = job(&app);
        let mut events = events(Demo::AngleMid, &j);
        // the running Mach number's first angle ends (a remote machine sends each angle as it ends), its second does not
        let (mach, a2) = (j.stability[j.stability.len() / 2], cfd::LIFT_ALPHAS[0]);
        events.push(Event::Point(PointDone {
            machine: 0,
            at: 99.0,
            mach,
            alpha_deg: Some(a2),
            outcome: Outcome::Converged,
            point: None,
            lift: Some(CfdLiftPoint { mach, alpha_deg: a2, cn: 0.1, x_cp: 1.0, cd: None, cm: None, quality: CfdQuality::Converged }),
            iterations: 100,
            cpu: false,
            fallback: None,
        }));
        let t = tracked(j.units(), &events);
        let (points, kept, half) = t.stability_kept();
        assert_eq!((kept, points.len(), half), (j.stability.len() / 2, 2 * (j.stability.len() / 2), Some(mach)));
        let table = cfd::partial_table(&j, &t, 60.0).expect("the drag points are a table");
        assert!(table.notes.contains(&cfd::half_angle_note(mach)), "{:?}", table.notes);
        assert_eq!(table.lift.as_ref().map(|l| l.points.len()), Some(2 * kept));
    }

    #[test]
    fn a_finished_solve_with_angles_is_stored_with_them_and_says_so() {
        let (mut app, serial) = app_with(&[Demo::Done]);
        let j = job(&app);
        feed(&mut app, serial, events(Demo::Done, &j));
        let table = app.ctx().doc.rocket().cfd_table(1).expect("stored").clone();
        assert_eq!(table.lift.as_ref().map(|l| l.points.len()), Some(2 * j.stability.len()));
        assert_eq!(
            notice(&app),
            format!("Solved {} points and {} · 1 not converged · 12 min 21 s", table.points.len(), if three_d() { "the angles" } else { "3D stability" })
        );
        let (_, row) = app.analyze().cfd.summary.clone().expect("the account");
        assert!(
            row.starts_with(&format!(
                "Solved {} Mach points and {} at {} Mach numbers in ",
                table.points.len(),
                if three_d() { "the angles" } else { "3D stability" },
                j.stability.len()
            )),
            "{row}"
        );
        assert_eq!(app.analyze().solve_status(), cfd::Status::Fresh);
        assert_eq!(app.analyze().cfd.status, cfd::Status::Fresh);
    }

    #[test]
    fn the_compute_setting_goes_with_the_solve_and_a_failed_connect_shows_on_the_screen() {
        use crate::app::settings::{Compute, Machine, MachineRef, Mode as ComputeMode};
        let mut app = App::with_example();
        app.analyze_mut().cfd.demo = [Demo::RemoteSetup].into_iter().collect();
        drop(app.update(AppMessage::Mode(Mode::Analyze)));
        let remote = Compute {
            machines: vec![Machine::parse("nobody@invalid.invalid")],
            mode: ComputeMode::Pick(MachineRef::Machine("nobody@invalid.invalid".into())),
            ..Compute::default()
        };
        app.ctx_mut().compute = remote.clone();
        // The solve the method starts carries it, and shows the set-up step where the device goes.
        drop(app.update(AppMessage::Analyze(Message::Method(Method::CfdTable))));
        let solving = app.analyze().cfd.solving.as_ref().expect("the method starts a solve");
        let (serial, job) = (solving.stream.serial(), job(&app));
        assert_eq!(job.compute, remote);
        feed(&mut app, serial, events(Demo::RemoteSetup, &job));
        let t = &app.analyze().cfd.solving.as_ref().unwrap().tracker;
        assert_eq!(t.device(), "");
        assert_eq!(t.machines()[0].state, MachineState::Connecting("Building the worker: Compiling bowshock-cfd v0.1.0 (31/118)".to_string()));
        assert!(solve_view::compact(app.analyze()).len() > 1);
        // A connect that fails is the solve's error: no table, the text on the screen, no CPU fallback.
        let why = crate::analyze::cfd::remote::error_text(
            "nobody@invalid.invalid",
            &bowshock_remote::RemoteError::Ssh("Could not resolve hostname invalid.invalid".to_string()),
        );
        feed(&mut app, serial, vec![Event::Finished(Err(why.clone()))]);
        assert!(app.analyze().cfd.solving.is_none());
        assert_eq!(app.analyze().cfd.failed.as_ref().map(|f| f.1.as_str()), Some(why.as_str()));
        assert!(notice(&app).contains("Could not resolve hostname"), "{}", notice(&app));
        assert!(app.ctx().doc.rocket().cfd_table(1).is_none());
    }

    #[test]
    fn a_shared_solve_has_a_point_in_flight_per_machine_and_a_stop_keeps_what_the_machines_solved() {
        use crate::analyze::solve::MachineState;
        let (mut app, serial) = app_with(&[Demo::ShareMid]);
        let script = events(Demo::ShareMid, &job(&app));
        feed(&mut app, serial, script);
        let t = &app.analyze().cfd.solving.as_ref().expect("still solving").tracker;
        assert_eq!(t.machines().iter().map(|m| m.name.as_str()).collect::<Vec<_>>(), ["This computer", "omarchy", "lab"]);
        assert_eq!(t.machines().iter().map(|m| m.done).collect::<Vec<_>>(), [3, 1, 0]);
        assert!(matches!(t.machines()[2].state, MachineState::Connecting(ref step) if step.starts_with("Building the worker")));
        let lives: Vec<(usize, usize)> = t.lives().map(|(_, l)| (l.machine, l.iteration)).collect();
        assert_eq!(lives, vec![(0, 1450), (1, 800)], "one point in flight on each machine that is ready");
        assert_eq!(solve_view::plot_data(t, None).live.len(), 2, "both are ringed on the chart");
        assert!(t.eta().is_some(), "the time left comes from the machines' combined rate");
        assert_eq!(t.device(), "NVIDIA GeForce RTX 3090, NVIDIA GeForce RTX 3080 Laptop GPU on omarchy");
        // the left panel lists the three machines under the device line
        assert!(solve_view::compact(app.analyze()).len() >= 8);

        // Cancel: the four points solved are a table, and it says which machines made it
        press(&mut app, Message::CfdCancel);
        let table = app.analyze().cfd.stopped.as_ref().and_then(|s| s.table.as_ref()).expect("four points are on offer");
        assert_eq!(table.points.len(), 4);
        assert_eq!(table.backend, "GPU (Luma) · NVIDIA GeForce RTX 3090 on this computer, NVIDIA GeForce RTX 3080 Laptop GPU on omarchy");
    }

    #[test]
    fn a_shared_solve_has_a_run_in_flight_per_machine_in_the_angles_too() {
        let (mut app, serial) = app_with(&[Demo::AngleShareMid]);
        let j = job(&app);
        feed(&mut app, serial, events(Demo::AngleShareMid, &j));
        let t = &app.analyze().cfd.solving.as_ref().expect("still solving").tracker;
        let lives: Vec<(usize, Option<f64>, usize)> = t.lives().map(|(u, l)| (l.machine, u.alpha_deg, l.iteration)).collect();
        assert_eq!(lives, vec![(0, Some(cfd::LIFT_ALPHAS[1]), 600), (1, Some(cfd::LIFT_ALPHAS[0]), 900)]);
        assert_eq!(t.done_count(), j.nodes.len() + 4, "every drag node, then two Mach numbers at two angles");
        assert_eq!(solve_view::plot_data(t, None).live.len(), 2);
    }

    use crate::analyze::flow_view;

    /// Whether the screens speak for the 3D engine (`BOWSHOCK_ENGINE=3d`): the app's tests hold under both settings.
    fn three_d() -> bool {
        solver::Engine::from_env() == solver::Engine::Solver3
    }

    /// Plays the script of `demo` into the solve `serial` of `app`.
    fn play(app: &mut App, serial: u64, demo: Demo) {
        let script = events(demo, &job(app));
        feed(app, serial, script);
    }

    #[test]
    fn a_solve_in_flight_shows_the_latest_picture_of_its_point_and_a_remote_machine_says_why_there_is_none() {
        let (mut app, serial) = app_with(&[Demo::Mid]);
        play(&mut app, serial, Demo::Mid);
        let s = app.analyze().cfd.solving.as_ref().expect("still solving");
        let live = s.picture.as_ref().expect("the point in flight has shown its flow");
        assert_eq!(live.iteration, 1000, "the latest snapshot");
        assert!(live.cells() > 10_000, "the real grid of the body: {} cells", live.cells());
        assert_eq!(s.tracker.lives().next().map(|(u, _)| u.mach), Some(live.mach));
        // the viewer holds it, and no solve screen asks for a notice
        let info = app.analyze().flow.picture_info().expect("the viewer shows the picture");
        assert_eq!((info.0, info.1), (live.mach, 1000));
        assert_eq!(solve_view::live_notice(app.analyze(), &s.tracker), None);
        // a solve on a remote machine sends no snapshots: the picture area says so
        let (mut app, serial) = app_with(&[Demo::RemoteMid]);
        play(&mut app, serial, Demo::RemoteMid);
        let s = app.analyze().cfd.solving.as_ref().expect("still solving");
        let notice = solve_view::live_notice(app.analyze(), &s.tracker).expect("a notice");
        assert!(notice.starts_with("No live picture: solving on omarchy"), "{notice}");
        // the final field of each point that ended does arrive, and is the picture until the next point shows its own
        assert!(s.picture.is_some() && s.tracker.done_count() == 4);
    }

    #[test]
    fn the_fields_of_a_finished_solve_are_kept_per_mach_node_for_the_session_and_dropped_with_the_geometry() {
        use crate::analyze::cfd::Status;
        let (mut app, serial) = app_with(&[Demo::Done]);
        play(&mut app, serial, Demo::Done);
        let table = app.ctx().doc.rocket().cfd_table(1).expect("the table is stored").clone();
        let store = &app.analyze().cfd.fields;
        assert_eq!(store.len(), table.points.len(), "a field for each Mach node");
        let mut bytes = 0;
        for p in &table.points {
            let f = store.get(&table, p.mach, 0.0).unwrap_or_else(|| panic!("no field at Mach {}", p.mach));
            assert_eq!(f.mach, p.mach);
            bytes += 4 * 4 * f.cells();
        }
        // session only: the document is far smaller than the fields and has none of them
        let saved = app.ctx().doc.rocket().to_bytes();
        assert!(saved.len() * 4 < bytes, "{} bytes saved against {bytes} of fields", saved.len());

        // the Flow field view shows the selected Mach number's field and the stepper moves it
        press(&mut app, Message::Flow(flow_view::Msg::Open(true)));
        let flow = &app.analyze().flow;
        assert!(flow.open);
        let first = flow.selected.expect("a Mach number is selected");
        assert_eq!(flow.picture_mach(), Some(first));
        let at = flow.machs.iter().position(|m| *m == first).unwrap();
        let step = if at + 1 < flow.machs.len() { 1 } else { -1 };
        press(&mut app, Message::Flow(flow_view::Msg::Step(step)));
        let next = app.analyze().flow.picture_mach().expect("the next point's picture");
        assert_ne!(next, first);
        assert_eq!(app.analyze().flow.selected, Some(next));
        // opening it closes the table view, and a new solve closes it
        press(&mut app, Message::Table(true));
        assert!(!app.analyze().flow.open);

        // another table of the same geometry (a file opened, an undo) has none of these fields
        press(&mut app, Message::Flow(flow_view::Msg::Open(true)));
        let other = bowshock_model::CfdTable { solved_at: table.solved_at + 1, ..table.clone() };
        assert!(app.ctx_mut().doc.store_cfd_table(other));
        press(&mut app, Message::Ignore);
        assert!(app.analyze().cfd.fields.get(app.analyze().cfd.table.as_ref().unwrap(), table.points[0].mach, 0.0).is_none());
        assert!(app.analyze().flow.picture_mach().is_none(), "the viewer says there is no field");

        // solved again, then the geometry changes: the table is stale and the fields are gone
        let (mut app, serial) = app_with(&[Demo::Done]);
        play(&mut app, serial, Demo::Done);
        assert!(!app.analyze().cfd.fields.is_empty());
        let nose = app.ctx().doc.rocket().parts()[0].id;
        app.ctx_mut().doc.edit("Lengthen nose", |r| r.set_dim(nose, bowshock_model::Dim::Length, 0.3));
        press(&mut app, Message::Ignore);
        assert_eq!(app.analyze().cfd.status, Status::Stale);
        assert!(app.analyze().cfd.fields.is_empty(), "dropped with the geometry they were solved for");
    }

    #[test]
    fn a_stopped_solve_keeps_the_fields_of_its_points_with_the_table_it_keeps() {
        let (mut app, serial) = app_with(&[Demo::Mid]);
        play(&mut app, serial, Demo::Mid);
        press(&mut app, Message::CfdCancel);
        assert!(app.analyze().cfd.fields.is_empty(), "nothing before Keep");
        press(&mut app, Message::CfdKeep);
        let table = app.ctx().doc.rocket().cfd_table(1).expect("kept").clone();
        assert_eq!(app.analyze().cfd.fields.len(), 4);
        assert!(table.points.iter().all(|p| app.analyze().cfd.fields.get(&table, p.mach, 0.0).is_some()));
    }

    #[test]
    fn the_3d_demos_show_a_finned_field_live_at_an_angle_node_and_two_angles_per_stability_mach_when_done() {
        // in flight at an angle node: the latest snapshot is a 3D field, and the viewer takes it as the scene
        let (mut app, serial) = app_with(&[Demo::Flow3dMid]);
        // the live picture follows what runs: the last drag node's field is the axisymmetric solver's (side view), the angles' the 3D scene
        let mut script = events(Demo::Flow3dMid, &job(&app));
        let drag_end = script.iter().rposition(|e| matches!(e, Event::Point(p) if p.alpha_deg.is_none())).expect("the drag nodes ended");
        let rest = script.split_off(drag_end + 1);
        feed(&mut app, serial, script);
        assert_eq!(app.analyze().flow.mode(), flow_view::Mode::Side, "a drag node is in 2D");
        feed(&mut app, serial, rest);
        let live = app.analyze().cfd.solving.as_ref().and_then(|s| s.picture.clone()).expect("the angle in flight has shown its flow");
        assert!(live.nk > 1 && live.iteration == 1000 && !live.fin_theta.is_empty(), "nk {} fins {:?}", live.nk, live.fin_theta);
        assert_eq!((live.alpha_deg, app.analyze().flow.mode()), (cfd::LIFT_ALPHAS[1], flow_view::Mode::Scene));
        assert!(app.analyze().flow.scene.picture().is_some());

        // done: a field for each drag node, and for each angle of each stability Mach number; the stepper walks the angles
        let (mut app, serial) = app_with(&[Demo::Flow3dDone]);
        play(&mut app, serial, Demo::Flow3dDone);
        let table = app.ctx().doc.rocket().cfd_table(1).expect("the table is stored").clone();
        let lift = table.lift.clone().expect("with its angles");
        let machs: Vec<f64> = lift.points.iter().map(|p| p.mach).fold(Vec::new(), |mut v, m| {
            if !v.contains(&m) {
                v.push(m);
            }
            v
        });
        let store = &app.analyze().cfd.fields;
        assert_eq!(store.len(), table.points.len() + 2 * machs.len());
        for &m in &machs {
            let angled: Vec<f64> = store.alphas_at(&table, m).into_iter().filter(|a| *a > 0.0).collect();
            assert_eq!(angled, cfd::LIFT_ALPHAS.to_vec(), "Mach {m}");
            assert!(store.has_mach(&table, m));
        }
        // a stability Mach number that is not a drag node has only its angles
        let mach = machs.iter().copied().find(|m| table.points.iter().all(|p| (p.mach - m).abs() > 1e-6)).expect("the stability nodes are not the table's");
        press(&mut app, Message::Flow(flow_view::Msg::Open(true)));
        press(&mut app, Message::Flow(flow_view::Msg::Pick(mach)));
        let flow = &app.analyze().flow;
        assert_eq!((flow.alphas.clone(), flow.selected_alpha), (cfd::LIFT_ALPHAS.to_vec(), Some(cfd::LIFT_ALPHAS[0])));
        assert!(flow.caption().contains(&format!("α {:.1}°", cfd::LIFT_ALPHAS[0])), "{}", flow.caption());
        press(&mut app, Message::Flow(flow_view::Msg::AlphaStep(1)));
        let flow = &app.analyze().flow;
        assert_eq!(flow.selected_alpha, Some(cfd::LIFT_ALPHAS[1]));
        assert!(flow.caption().contains(&format!("α {:.1}°", cfd::LIFT_ALPHAS[1])), "{}", flow.caption());
    }

    #[test]
    fn a_stopped_solve_keeps_the_fields_of_the_angles_it_keeps() {
        let (mut app, serial) = app_with(&[Demo::Flow3dMid]);
        play(&mut app, serial, Demo::Flow3dMid);
        press(&mut app, Message::CfdCancel);
        let table = app.analyze().cfd.stopped.as_ref().and_then(|s| s.table.clone()).expect("the drag points and the angles solved are on offer");
        let kept = table.lift.as_ref().map_or(0, cfd::distinct_machs);
        assert!(kept > 0);
        assert!(app.analyze().cfd.fields.is_empty(), "nothing before Keep");
        press(&mut app, Message::CfdKeep);
        let store = &app.analyze().cfd.fields;
        assert_eq!(store.len(), table.points.len() + 2 * kept, "the drag nodes' fields and both angles of each kept Mach number; the running one is not kept");
    }

    /// A real solve of the example (GPU, minutes) through the app's own path: the events of the one `solve` (the drag nodes, then the 3D
    /// angles with the live snapshots of a running angle and the final field of each) reach the app, which stores the table with its
    /// angles and keeps the fields. `cargo nextest run --release -p bowshock a_real_solve --run-ignored only --no-capture` (through
    /// `tools/gpuq/gpuq`).
    #[test]
    #[ignore = "solves the example's whole table, drag and 3D angles, on the GPU"]
    fn a_real_solve_through_the_app_ends_with_its_table_and_fields_in_the_store() {
        // the screen's solve is scripted down to its set-up so that nothing else runs; the job it built is the real one
        let (mut app, serial) = app_with(&[Demo::RemoteSetup]);
        let job = job(&app);
        assert!(!job.stability.is_empty());
        let log = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let sink = log.clone();
        let t = std::time::Instant::now();
        let result = cfd::solve(&job, &std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)), move |e| sink.lock().unwrap().push(e));
        let seconds = t.elapsed().as_secs_f64();
        let mut events = std::mem::take(&mut *log.lock().unwrap());
        let (snapshots, finals) =
            (events.iter().filter(|e| matches!(e, Event::Field(_))).count(), events.iter().filter(|e| matches!(e, Event::PointField { .. })).count());
        println!(
            "REAL solve {seconds:.1} s, {} drag nodes + {} stability nodes, {snapshots} live snapshots, {finals} final fields",
            job.nodes.len(),
            job.stability.len()
        );
        events.push(Event::Finished(result));
        feed(&mut app, serial, events);
        let table = app.ctx().doc.rocket().cfd_table(1).expect("the solve is stored").clone();
        assert!(table.lift.is_some(), "the 3D angles are in the table: {}", notice(&app));
        println!("REAL table {} points, lift {:?}; notice: {}", table.points.len(), table.lift.as_ref().map(|l| l.points.len()), notice(&app));
        assert_eq!(app.analyze().solve_status(), cfd::Status::Fresh);
        assert!(snapshots >= job.stability.len() && finals >= 2 * job.stability.len(), "{snapshots} snapshots, {finals} final fields");
        let store = &app.analyze().cfd.fields;
        for &m in &job.stability {
            assert_eq!(store.alphas_at(&table, m).iter().filter(|a| **a > 0.0).count(), 2, "Mach {m}: both angles have a field");
        }
        // the viewer shows a real one: cells in the hundreds of thousands, the fins, the angle on the caption
        press(&mut app, Message::Flow(flow_view::Msg::Open(true)));
        press(&mut app, Message::Flow(flow_view::Msg::Pick(job.stability[job.stability.len() / 2])));
        for _ in 0..2 {
            press(&mut app, Message::Flow(flow_view::Msg::AlphaStep(1)));
        }
        let flow = &app.analyze().flow;
        let picture = flow.scene.picture().expect("the 3D scene shows the real field");
        println!(
            "REAL field {} cells, {} wall faces, {} tiles, caption {}",
            picture.field.cells(),
            picture.field.walls.len(),
            picture.tiles.len(),
            flow.caption()
        );
        assert!(picture.field.cells() > 100_000 && !picture.field.fin_theta.is_empty() && picture.field.iteration > 100);
    }

    /// A real solve of the finless example with the 3D engine (`BOWSHOCK_ENGINE=3d`, GPU): the drag nodes are wedges, the angles the half domain on the base-wake grid.
    /// The events reach the app like the legacy engine's, the drag nodes' final fields are wedge `Field3`s that the viewer shows side-on, and the table is stored.
    /// `tools/gpuq/gpuq submit --host local --name engine3d-app -- env BOWSHOCK_ENGINE=3d cargo nextest run --release -p bowshock a_real_finless_3d_engine --run-ignored only --no-capture`.
    #[test]
    #[ignore = "BOWSHOCK_ENGINE=3d: solves a finless table and its angles with the 3D engine on the GPU"]
    fn a_real_finless_3d_engine_table_through_the_app_has_wedge_fields() {
        assert_eq!(solver::Engine::from_env(), solver::Engine::Solver3, "run with BOWSHOCK_ENGINE=3d");
        let (mut app, serial) = app_with(&[Demo::RemoteSetup]);
        let mut job = (*job(&app)).clone();
        job.plan.fins = None;
        let log = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let sink = log.clone();
        let t = std::time::Instant::now();
        let result = cfd::solve(&job, &std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)), move |e| sink.lock().unwrap().push(e));
        let seconds = t.elapsed().as_secs_f64();
        let mut events = std::mem::take(&mut *log.lock().unwrap());
        let (snapshots, finals) =
            (events.iter().filter(|e| matches!(e, Event::Field(_))).count(), events.iter().filter(|e| matches!(e, Event::PointField { .. })).count());
        println!(
            "ENGINE3D solve {seconds:.1} s, {} drag nodes + {} stability nodes, {snapshots} live snapshots, {finals} final fields",
            job.nodes.len(),
            job.stability.len()
        );
        events.push(Event::Finished(result));
        feed(&mut app, serial, events);
        let table = app.ctx().doc.rocket().cfd_table(1).expect("the solve is stored").clone();
        println!("ENGINE3D table {} points; lift {:?}; notice: {}", table.points.len(), table.lift.as_ref().map(|l| l.points.len()), notice(&app));
        println!("| Mach | Cd | pressure | base | quality |\n|---|---|---|---|---|");
        for p in &table.points {
            println!("| {:.2} | {:.4} | {:.4} | {:.4} | {:?} |", p.mach, p.cd, p.pressure, p.base, p.quality);
        }
        assert_eq!(table.points.len(), job.nodes.len(), "every drag node gave a point");
        assert!(table.lift.is_some(), "the angles are in the table");
        let store = &app.analyze().cfd.fields;
        let drag_field = store.get(&table, table.points[table.points.len() / 2].mach, 0.0).expect("a wedge field at a drag node").clone();
        println!(
            "ENGINE3D drag field: {} cells, symmetry {:?}, caption M {} a {}",
            drag_field.cells(),
            drag_field.symmetry,
            drag_field.mach,
            drag_field.alpha_deg
        );
        assert!(matches!(drag_field.symmetry, solver::Symmetry::Axisymmetric { .. }));
        // the viewer shows it side-on, like a 2D field
        let mach = drag_field.mach;
        press(&mut app, Message::Flow(flow_view::Msg::Open(true)));
        press(&mut app, Message::Flow(flow_view::Msg::Pick(mach)));
        assert_eq!(app.analyze().flow.mode(), flow_view::Mode::Side);
    }
}
