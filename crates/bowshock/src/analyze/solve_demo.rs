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
//! the iteration cap); nothing here comes from the solver. The real event stream is covered by the ignored tests of [`super::cfd`].

use std::collections::VecDeque;

use bowshock_model::{CfdLiftPoint, CfdLiftTable, CfdPoint, CfdQuality};

use super::cfd::{self, Event, Job, LiftJob, Tally};
use super::solve::{Iteration, Outcome, PointDone};

/// Which scripted solve to play.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Demo {
    /// A drag solve stopped partway: four points solved, the fifth running.
    Mid,
    /// A drag solve that finished.
    Done,
    /// A drag solve that was cancelled with five points solved.
    Stopped,
    /// A 3D solve stopped partway.
    LiftMid,
    /// A 3D solve that finished.
    LiftDone,
}

impl Demo {
    fn parse(name: &str) -> Option<Demo> {
        Some(match name.trim() {
            "mid" => Demo::Mid,
            "done" => Demo::Done,
            "stopped" => Demo::Stopped,
            "lift-mid" => Demo::LiftMid,
            "lift-done" => Demo::LiftDone,
            _ => return None,
        })
    }
}

/// The scripts named by `BOWSHOCK_CFD_DEMO`, in order.
pub fn from_env() -> VecDeque<Demo> {
    std::env::var("BOWSHOCK_CFD_DEMO").map(|v| v.split(',').filter_map(Demo::parse).collect()).unwrap_or_default()
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
    CfdPoint { mach, cd: total, friction, pressure: total - friction - base - fins, base, fins, other: 0.0, quality }
}

fn quality(o: Outcome) -> CfdQuality {
    match o {
        Outcome::Converged => CfdQuality::Converged,
        Outcome::Settled => CfdQuality::Stationary,
        Outcome::Stalled | Outcome::Fallback => CfdQuality::Stalled,
    }
}

/// Reports of a unit settling from a transient: the monitor `target` approached with a decaying wobble, a report every 50
/// iterations up to `until`, the clock starting at `from` (s) and running at the script's rate.
fn trace(mach: f64, alpha_deg: Option<f64>, target: f64, from: f64, until: usize) -> Vec<Event> {
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

/// The events of a scripted drag solve of `job`.
pub fn events(demo: Demo, job: &Job) -> Vec<Event> {
    let nodes = &job.nodes;
    let finished = match demo {
        Demo::Mid => 4,
        Demo::Stopped => 5.min(nodes.len()),
        _ => nodes.len(),
    };
    let mut out = vec![Event::Device("Apple M1 Max".to_string())];
    let mut at = 0.0;
    let mut tally = Tally { points: Vec::new(), notes: job.plan.notes.clone(), gpu: 0, cpu: 0, fallback: None };
    for &mach in &nodes[..finished] {
        let o = outcome(mach, nodes);
        let its = if o == Outcome::Stalled { CAP } else { iterations(mach) };
        let p = point(mach, quality(o));
        at += SETUP + its as f64 * PER_ITERATION;
        out.push(Event::Point(PointDone { at, mach, alpha_deg: None, outcome: o, point: Some(p), lift: None, iterations: its, cpu: false, fallback: None }));
        tally.points.push(p);
        tally.gpu += 1;
    }
    match demo {
        Demo::Mid => {
            // the next point has reached iteration 1 450
            let mach = nodes[finished];
            out.extend(trace(mach, None, cd(mach), at + SETUP, 1450));
        }
        Demo::Stopped => out.push(Event::Finished(Err(cfd::CANCELLED.to_string()))),
        _ => out.push(Event::Finished(Ok(cfd::table_from(job, tally, nodes.len(), at)))),
    }
    out
}

/// The events of a scripted 3D stability solve of `job`.
pub fn lift_events(demo: Demo, job: &LiftJob) -> Vec<Event> {
    let nodes = &job.nodes;
    let finished = if demo == Demo::LiftMid { (nodes.len() / 2).min(nodes.len().saturating_sub(1)) } else { nodes.len() };
    let mut out = vec![Event::Device("Apple M1 Max".to_string())];
    let mut at = 0.0;
    let mut points: Vec<CfdLiftPoint> = Vec::new();
    // CN grows with the angle (slightly sub-linearly) and with Mach up to the transonic range; CP sits about 60 % of the length aft.
    let lift = |mach: f64, a: f64| CfdLiftPoint {
        mach,
        alpha_deg: a,
        cn: a.to_radians() * (2.4 + 0.5 * mach.min(1.2)) * (1.0 - 0.01 * a),
        x_cp: 0.85 + 0.04 * mach.min(1.5),
        quality: CfdQuality::Converged,
    };
    for &mach in &nodes[..finished] {
        let mut hand = Vec::new();
        for (k, &a) in cfd::LIFT_ALPHAS.iter().enumerate() {
            let its = if k == 0 { 1800 } else { 700 };
            out.extend(trace(mach, Some(a), lift(mach, a).cn, at + SETUP, its));
            at += SETUP + its as f64 * PER_ITERATION;
            let p = lift(mach, a);
            hand.push(PointDone {
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
            d.at = at;
            out.push(Event::Point(d));
        }
    }
    if demo == Demo::LiftMid {
        // the next Mach number: its first angle has run, its second is at iteration 600
        let mach = nodes[finished];
        out.extend(trace(mach, Some(cfd::LIFT_ALPHAS[0]), lift(mach, cfd::LIFT_ALPHAS[0]).cn, at + SETUP, 1800));
        at += SETUP + 1800.0 * PER_ITERATION;
        out.extend(trace(mach, Some(cfd::LIFT_ALPHAS[1]), lift(mach, cfd::LIFT_ALPHAS[1]).cn, at, 600));
    } else {
        out.push(Event::LiftFinished(Ok(CfdLiftTable {
            key: cfd::lift_key(job.plan.geometry, nodes),
            solved_at: cfd::now(),
            solve_seconds: at,
            backend: "GPU (Luma)".to_string(),
            points,
            notes: Vec::new(),
        })));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analyze::solve::{Kind, Slot, Tracker};

    #[test]
    fn the_names_parse_in_order_and_unknown_names_are_skipped() {
        let parsed: VecDeque<Demo> = "done, lift-mid,nonsense,stopped".split(',').filter_map(Demo::parse).collect();
        assert_eq!(parsed, VecDeque::from([Demo::Done, Demo::LiftMid, Demo::Stopped]));
    }

    #[test]
    fn the_script_is_the_stream_the_tracker_expects() {
        let nodes = [0.1, 0.5, 0.8, 0.9, 1.0, 1.1, 1.3, 1.7, 2.3, 3.0];
        let job = Job { plan: crate::analyze::cfd::test_plan(), altitude: 0.0, nodes: nodes.to_vec(), stage: 1, grid: bowshock_model::CfdGrid::Coarse };
        let mut t = Tracker::new(Kind::Drag, Tracker::drag_units(&nodes), "");
        for e in events(Demo::Mid, &job) {
            match e {
                Event::Iteration(i) => t.on_iteration(&i),
                Event::Point(p) => t.on_done(&p),
                _ => {}
            }
        }
        assert_eq!(t.done_count(), 4);
        assert_eq!(t.live().unwrap().1.iteration, 1450);
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
        let mut app = App::with_example();
        app.analyze_mut().cfd.demo = scripts.iter().copied().collect();
        drop(app.update(AppMessage::Mode(Mode::Analyze)));
        drop(app.update(AppMessage::Analyze(Message::Method(Method::CfdTable))));
        let serial = app.analyze().cfd.solving.as_ref().expect("choosing the CFD table method starts a solve").serial;
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
        assert_eq!((t.done_count(), t.live().map(|(u, l)| (u.mach, l.iteration))), (4, Some((t.units()[4].mach, 1450))));
        assert_eq!(t.device, "Apple M1 Max");
        assert!(t.fraction() > 0.0 && t.fraction() < 1.0 && t.eta().is_some_and(|s| s > 0.0));
        // the curve: four solved points (one flagged), the one in flight and the rest to come
        let data = solve_view::plot_data(t, None);
        assert_eq!(data.curves[0].solved.len(), 4);
        assert_eq!(data.curves[0].solved.iter().filter(|m| !m.outcome.is_settled()).count(), 1);
        assert_eq!(data.curves[0].pending.len(), total - 4);
        assert!(data.live.is_some());
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
        assert!(notice(&app).contains("cancelled after 4 of"), "{}", notice(&app));
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
        let serial = app.analyze().cfd.solving.as_ref().expect("Re-solve starts a solve").serial;
        let first: Vec<Event> = events(Demo::Mid, &job(&app)).into_iter().take_while(|e| !matches!(e, Event::Iteration(_))).take(2).collect();
        assert!(matches!(first.last(), Some(Event::Point(_))));
        feed(&mut app, serial, first);
        press(&mut app, Message::CfdCancel);
        assert!(app.analyze().cfd.solving.is_none() && app.analyze().cfd.stopped.is_none(), "one point is not a table");
        assert_eq!(notice(&app), "CFD solve cancelled");
        assert!(app.ctx().doc.rocket().cfd_table(1).is_none());
    }

    #[test]
    fn a_finished_solve_is_stored_and_summarised() {
        let (mut app, serial) = app_with(&[Demo::Done]);
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
    fn the_3d_solve_reports_per_angle_and_a_cancel_discards_it_cleanly() {
        let (mut app, serial) = app_with(&[Demo::Done, Demo::LiftMid, Demo::LiftDone]);
        let script = events(Demo::Done, &job(&app));
        feed(&mut app, serial, script);
        assert!(app.ctx().doc.rocket().cfd_table(1).is_some());

        press(&mut app, Message::SolveLift);
        let lift = app.analyze().cfd.lift_solving.as_ref().expect("Solve stability (3D) starts the 3D solve");
        let (serial, job) =
            (lift.serial, LiftJob { plan: app.analyze().cfd.plan.as_ref().clone().unwrap(), nodes: cfd::lift_nodes(app.analyze().cond.mach).unwrap() });
        assert_eq!(lift.tracker.total(), 2 * job.nodes.len(), "a run per Mach number and angle");
        feed(&mut app, serial, lift_events(Demo::LiftMid, &job));
        let t = &app.analyze().cfd.lift_solving.as_ref().unwrap().tracker;
        let (unit, live) = t.live().unwrap();
        assert_eq!((unit.alpha_deg, live.iteration), (Some(cfd::LIFT_ALPHAS[1]), 600));
        assert!(t.done_count() >= 2);
        assert_eq!(solve_view::plot_data(t, None).curves.len(), 2, "a curve per angle");

        press(&mut app, Message::CfdCancel);
        assert!(app.analyze().cfd.lift_solving.is_none() && app.analyze().cfd.stopped.is_none());
        assert_eq!(notice(&app), "3D stability solve cancelled");
        assert!(app.ctx().doc.rocket().cfd_table(1).is_some_and(|t| t.lift.is_none()), "the 3D solve left the drag table as it was");

        // and one that finishes is stored on the drag table
        press(&mut app, Message::SolveLift);
        let serial = app.analyze().cfd.lift_solving.as_ref().expect("a second 3D solve").serial;
        feed(&mut app, serial, lift_events(Demo::LiftDone, &job));
        assert!(app.analyze().cfd.lift_solving.is_none());
        assert!(app.ctx().doc.rocket().cfd_table(1).is_some_and(|t| t.lift.is_some()));
        assert_eq!(notice(&app), "Solved 3D stability · all converged · 3 min 28 s");
        let row = app.analyze().cfd.lift_summary.as_ref().map(|(_, s)| s.clone()).unwrap_or_default();
        assert!(row.starts_with("Solved 3D stability at ") && row.contains(" on Apple M1 Max · all converged"), "{row}");
    }
}
