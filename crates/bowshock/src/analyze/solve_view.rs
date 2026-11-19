//! The Analyze screen while a CFD solve runs, and after one was cancelled with points solved: the progress, the curve as it fills
//! in, the point in flight and the points done.
//!
//! The centre column swaps its plots for this view for as long as the solve (or its stopped state) is on show. Top to bottom:
//!
//! * the headline (`Mach 0.80 · point 4 of 10`), the overall bar with the elapsed time and the time left from the measured
//!   throughput ("estimating…" until two points are done), and where it runs (the GPU's name, or CPU with the reason);
//! * the curve of the part that is running: Cd(Mach) of the drag table, then CN(Mach) at each angle, with the solved points
//!   solid (a warning triangle where a point did not settle), the points to come hollow on a dashed line at the reference
//!   method's value, and the point in flight ringed in `accent` at the value the solver's monitor has now;
//! * the point in flight: its iteration, how far the residual has dropped, the monitor's value and its range against the stop
//!   rule's tolerance (the trace is drawn with the band the range must fit in);
//! * every point with its outcome, value, iterations and time.
//!
//! [`plot_data`] is the pure part (what is drawn where), tested here; the canvases only draw it.

use iced::widget::canvas::{self, Frame, Geometry, LineDash, LineJoin, Path, Stroke, Text};
use iced::widget::{column, container, row, scrollable, text, Space};
use iced::{alignment, mouse, Color, Element, Fill, Length, Pixels, Point, Rectangle, Renderer, Theme, Widget};

use bowshock_model::CfdGrid;

use super::compute::Data;
use super::flow_view::{Emphasis, Msg};
use super::solve::{duration_text, eta_text, thousands, Live, MachineRow, MachineState, Outcome, Phase, Slot, Tracker, Unit};
use super::{Message, State, Stopped};
use crate::app::{ui, Ctx};
use crate::theme::{self, MONO, SANS_SEMIBOLD};
use crate::tokens::{Tokens, SPACE_1, SPACE_2, SPACE_3, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::button::{button as push, Weight};
use crate::widgets::chip::{chip, Severity};
use crate::widgets::field::rule;
use crate::widgets::glyph::{self, Mark, Tone};
use crate::widgets::plot::ticks::{fmt_fixed, nice_ticks};
use crate::widgets::plot::SeriesColor;
use crate::widgets::segmented::segmented;

// ---------------------------------------------------------------------------------------------------------------------
// What is on show
// ---------------------------------------------------------------------------------------------------------------------

/// The solve on show: the one running, or the one that was stopped.
struct Shown<'a> {
    tracker: &'a Tracker,
    /// Seconds since the solve began (the whole run of a stopped one).
    elapsed: f64,
    running: bool,
    cancelling: bool,
    grid: CfdGrid,
    /// The grid a draft-first solve goes on to.
    then: Option<CfdGrid>,
    /// The altitude of the Reynolds number; `None` for the Euler 3D solve.
    altitude: Option<f64>,
    stopped: Option<&'a Stopped>,
}

fn shown(state: &State) -> Option<Shown<'_>> {
    let cfd = &state.cfd;
    if let Some(s) = cfd.solving.as_ref() {
        return Some(Shown {
            tracker: &s.tracker,
            elapsed: s.started.elapsed().as_secs_f64().max(s.tracker.clock()),
            running: true,
            cancelling: s.cancelling,
            grid: s.grid,
            then: s.then,
            altitude: s.altitude,
            stopped: None,
        });
    }
    let s = cfd.stopped.as_ref()?;
    Some(Shown {
        tracker: &s.tracker,
        elapsed: s.seconds,
        running: false,
        cancelling: false,
        grid: s.grid,
        then: None,
        altitude: Some(s.altitude),
        stopped: Some(s),
    })
}

/// Whether the solve (or its stopped state) is what the screen shows: the CFD table method on an analysis that uses it.
pub fn active(state: &State) -> bool {
    state.method == super::Method::CfdTable && super::uses_cfd(state.analysis) && shown(state).is_some()
}

fn name(phase: Phase) -> &'static str {
    match phase {
        Phase::Drag => "Cd",
        Phase::Stability => "CN",
    }
}

/// The phase a unit is in.
fn phase_of(unit: &Unit) -> Phase {
    if unit.alpha_deg.is_some() {
        Phase::Stability
    } else {
        Phase::Drag
    }
}

/// `Mach 0.80 · point 4 of 10`, or with the angle `Mach 0.80 · 6° · run 4 of 16`; with several in flight at once `3 points running ·
/// 4 of 10 done`.
fn headline(t: &Tracker) -> String {
    let mut lives = t.lives();
    match (lives.next(), lives.next()) {
        (Some((u, _)), None) => {
            // counted among the units of its own part of the solve: drag points, then 3D runs
            let same: Vec<&Unit> = t.units().iter().filter(|x| phase_of(x) == phase_of(u)).collect();
            let at = same.iter().position(|x| *x == u).map_or(1, |i| i + 1);
            match u.alpha_deg {
                Some(a) => format!("Mach {:.2} · {a:.0}° · run {at} of {}", u.mach, same.len()),
                None => format!("Mach {:.2} · point {at} of {}", u.mach, same.len()),
            }
        }
        (Some(_), Some(_)) => {
            let n = t.lives().count();
            let what = if t.phase() == Phase::Drag { "points" } else { "runs" };
            format!("{n} {what} running · {} of {} done", t.done_count(), t.total())
        }
        (None, _) if t.done_count() == 0 => "Starting the solver".to_string(),
        (None, _) => format!("{} of {} done", t.done_count(), t.total()),
    }
}

/// `4 of 10 Mach points`, with the 3D runs when the solve has them: how much of the solve is in.
pub(super) fn how_far(t: &Tracker) -> String {
    super::cfd::progress_text(counts(t).0, counts(t).1)
}

/// The toast's form of [`how_far`].
pub(super) fn how_far_brief(t: &Tracker) -> String {
    super::cfd::progress_brief(counts(t).0, counts(t).1)
}

/// (drag units done, in all), (angle units done, in all).
fn counts(t: &Tracker) -> ((usize, usize), (usize, usize)) {
    ((t.drag_done(), t.drag_count()), (t.done_count() - t.drag_done(), t.total() - t.drag_count()))
}

/// `1 min 12 s elapsed`, and `about 1 min 40 s left` or `estimating…`.
fn clock(t: &Tracker, elapsed: f64) -> (String, String) {
    (format!("{} elapsed", duration_text(elapsed)), t.eta().map_or_else(|| "estimating…".to_string(), |s| format!("{} left", eta_text(s))))
}

/// How one machine reads: `Apple M1 Max · GPU (Luma)`, `CPU`, the set-up step, or why it left.
fn machine_text(m: &MachineRow) -> String {
    match &m.state {
        MachineState::Connecting(step) if step.is_empty() => "starting the solver".to_string(),
        MachineState::Connecting(step) => step.clone(),
        MachineState::Ready(d) if d == "CPU" => "CPU".to_string(),
        MachineState::Ready(gpu) => format!("{gpu} · GPU (Luma)"),
        MachineState::Skipped(why) => format!("left out: {why}"),
        MachineState::Dropped(why) => format!("lost: {why}"),
    }
}

/// Where it runs: `Apple M1 Max · GPU (Luma)`, or `CPU`; with several machines how many are working.
fn device(t: &Tracker) -> String {
    match t.machines() {
        [] => "starting the solver".to_string(),
        [one] => machine_text(one),
        many => {
            let alive = many.iter().filter(|m| m.alive()).count();
            format!("{alive} of {} machines", many.len())
        }
    }
}

/// One line per machine when there are several: its name, what it is doing or why it left, and how many it has finished.
fn machine_lines<'a>(t: &Tracker, running: bool) -> Vec<Element<'a, Message>> {
    if t.machines().len() < 2 {
        return Vec::new();
    }
    t.machines()
        .iter()
        .enumerate()
        .map(|(i, m)| {
            let doing = t.lives().find(|(_, l)| l.machine == i).map(|(u, _)| match u.alpha_deg {
                Some(a) => format!("Mach {:.2} · {a:.0}°", u.mach),
                None => format!("Mach {:.2}", u.mach),
            });
            let state = match (&m.state, doing) {
                (MachineState::Ready(_), Some(at)) => format!("{} · {at}", machine_text(m)),
                (MachineState::Ready(_), None) if running => format!("{} · idle", machine_text(m)),
                _ => machine_text(m),
            };
            let done = if m.done > 0 { format!(" · {} done", m.done) } else { String::new() };
            muted(format!("{} · {state}{done}", m.name), 12.0)
        })
        .collect()
}

fn grid_name(grid: CfdGrid) -> &'static str {
    match grid {
        CfdGrid::Coarse => "coarse grid",
        CfdGrid::Draft => "draft grid",
        CfdGrid::Medium => "medium grid",
    }
}

/// The grid of a solve on show, with the one it goes on to: `draft grid, then coarse`.
fn grid_label(s: &Shown<'_>) -> String {
    match s.then {
        Some(next) => format!("{}, then {}", grid_name(s.grid), grid_name(next).trim_end_matches(" grid")),
        None => grid_name(s.grid).to_string(),
    }
}

fn muted<'a>(s: impl text::IntoFragment<'a>, size: f32) -> Element<'a, Message> {
    text(s).size(size).style(theme::muted).boxed()
}

fn mono<'a>(s: impl text::IntoFragment<'a>, size: f32, color: fn(&Tokens) -> Color) -> Element<'a, Message> {
    text(s).font(MONO).size(size).style(theme::colored(color)).boxed()
}

/// The overall bar: a 1 px framed track with an `ink` fill (accent is for what you act on, not for progress).
fn bar<'a>(fraction: f64) -> Element<'a, Message> {
    let filled = (fraction.clamp(0.0, 1.0) * 1000.0).round() as u16;
    let part = |portion: u16, fill: bool| {
        container(Space::new())
            .width(Length::FillPortion(portion))
            .height(6)
            .style(move |t: &Theme| container::Style { background: fill.then(|| theme::tokens(t).ink.into()), ..container::Style::default() })
    };
    container(row![part(filled, true), part(1000 - filled, false)])
        .width(Fill)
        .padding(1)
        .style(|t: &Theme| container::Style {
            background: Some(theme::tokens(t).bg_300.into()),
            border: theme::frame(theme::tokens(t).line_200),
            ..container::Style::default()
        })
        .boxed()
}

// ---------------------------------------------------------------------------------------------------------------------
// Left panel
// ---------------------------------------------------------------------------------------------------------------------

/// The lines of the Method section while a solve runs or is stopped: where it is, the bar, the clock and the device. Empty when
/// there is nothing to say.
pub fn compact(state: &State) -> Vec<Element<'_, Message>> {
    let Some(s) = shown(state).filter(|_| state.method == super::Method::CfdTable) else { return Vec::new() };
    let t = s.tracker;
    let mut out: Vec<Element<'_, Message>> = Vec::new();
    if let Some(stopped) = s.stopped {
        out.push(text(format!("Stopped after {}", how_far(t))).size(TEXT_BODY).boxed());
        out.push(muted(format!("{} on {}", duration_text(stopped.seconds), device(t)), 12.0));
        return out;
    }
    out.push(text(headline(t)).size(TEXT_BODY).boxed());
    out.push(bar(t.fraction()));
    let (elapsed, left) = clock(t, s.elapsed);
    out.push(muted(format!("{:.0} % · {elapsed}", 100.0 * t.fraction()), 12.0));
    out.push(muted(left, 12.0));
    out.push(muted(format!("{} · {}", device(t), grid_label(&s)), 12.0));
    out.extend(machine_lines(t, true));
    out
}

/// What a solve on the CPU after the GPU failed says (`GPU unavailable: …`), if it does.
fn fallback_chip<'a>(t: &Tracker) -> Option<Element<'a, Message>> {
    t.machines().iter().any(|m| !m.remote && m.device() == Some("CPU")).then(|| {
        let why = t.fallback.as_deref().unwrap_or("no GPU");
        chip(Severity::Warn, format!("Solving on the CPU · {why}")).into()
    })
}

// ---------------------------------------------------------------------------------------------------------------------
// The centre
// ---------------------------------------------------------------------------------------------------------------------

/// The reference curve: the Cd(Mach) of the method the plots show, for the drag table; `None` for the 3D solve (the plots have no
/// CN(Mach) of another method) and when there is no Mach sweep.
fn reference(state: &State, ctx: &Ctx, phase: Phase) -> Option<(String, Vec<(f64, f64)>)> {
    if phase != Phase::Drag {
        return None;
    }
    let out = state.output()?;
    let active = ctx.doc.rocket().active_config_id();
    let run =
        out.runs.iter().find(|r| r.id == active && matches!(r.data, Data::Mach(_))).or_else(|| out.runs.iter().find(|r| matches!(r.data, Data::Mach(_))))?;
    let Data::Mach(points) = &run.data else { return None };
    let label = if out.method == super::Method::CfdTable { "previous CFD table".to_string() } else { out.method.name().to_string() };
    Some((label, points.iter().map(|p| (p.mach, p.drag.total)).collect()))
}

/// The solve view for the centre column, when a solve (or its stopped state) is what is on show.
pub fn centre<'a>(state: &'a State, ctx: &'a Ctx) -> Option<Element<'a, Message>> {
    if !active(state) {
        return None;
    }
    let s = shown(state)?;
    let t = s.tracker;
    let pad = |e: Element<'a, Message>| container(e).padding([SPACE_2, SPACE_3]).width(Fill).boxed();

    // Headline row: what it is, and the button.
    let title = if s.running { super::cfd::solving_title(t.phase() == Phase::Stability) } else { super::cfd::STOPPED_TITLE };
    let action: Element<'a, Message> = match (s.running, s.stopped) {
        (true, _) if s.cancelling => push("Cancelling…", Weight::Secondary).disabled("The solver is ending the run in flight").into(),
        (true, _) => push("Cancel", Weight::Secondary).on_press(Message::CfdCancel).into(),
        (false, Some(stopped)) => {
            let keep: Element<'a, Message> = match &stopped.table {
                Some(_) => push(&stopped.keep_label, Weight::Primary).on_press(Message::CfdKeep).into(),
                None => push("Keep solved points", Weight::Secondary).disabled("A table needs at least two solved points").into(),
            };
            let discard: Element<'a, Message> = push("Discard", Weight::Secondary).on_press(Message::CfdDiscard).into();
            row![keep, discard].spacing(SPACE_2).boxed()
        }
        (false, None) => Space::new().boxed(),
    };
    // A solve has a picture of its flow (the drag solve's meridian, the stability solve's 3D scene): how the room is shared with the
    // curve is the user's to say.
    let live = s.running;
    let emphasis = state.flow.emphasis;
    let toggle: Element<'a, Message> = if live {
        segmented(&[(Emphasis::Flow, "Flow"), (Emphasis::Curve, "Curve")], emphasis, |e| Message::Flow(Msg::Emphasis(e)))
    } else {
        Space::new().boxed()
    };
    let head = row![ui::label(title), Space::new().width(Fill), toggle, Space::new().width(SPACE_2), action].align_y(iced::Center);

    let line = if s.running {
        text(headline(t)).size(20).font(SANS_SEMIBOLD).boxed()
    } else {
        text(format!("{} solved", how_far(t))).size(20).font(SANS_SEMIBOLD).boxed()
    };

    let (elapsed, left) = clock(t, s.elapsed);
    let figures = if s.running {
        row![mono(format!("{:.0} %", 100.0 * t.fraction()), 13.0, |t| t.ink), muted(elapsed, 12.0), muted("·", 12.0), muted(left, 12.0)]
            .spacing(SPACE_2)
            .align_y(iced::Center)
    } else {
        row![muted(format!("ran {}", duration_text(s.elapsed)), 12.0)]
    };
    let altitude = crate::units::with_unit_scaled(s.altitude.unwrap_or_default(), Quantity::Altitude, ctx.units, 0);
    let place = super::cfd::place(&device(t), &grid_label(&s), &altitude, s.running && t.phase() == Phase::Stability);
    let mut place_row: Vec<Element<'a, Message>> = vec![muted(place, 12.0)];
    place_row.extend(fallback_chip(t));
    let machines = machine_lines(t, s.running);

    let mut top: Vec<Element<'a, Message>> = vec![head.boxed(), line];
    if s.running {
        top.push(bar(t.fraction()));
    }
    top.push(figures.boxed());
    top.push(row(place_row).spacing(SPACE_3).align_y(iced::Center).boxed());
    if !machines.is_empty() {
        top.push(column(machines).spacing(2).boxed());
    }
    let header = column(top).spacing(SPACE_2);

    let data = plot_data(t, reference(state, ctx, t.phase()));
    let curve = container(canvas::Canvas::new(SolvePlot { data }).width(Fill).height(Fill)).width(Fill).padding([0.0, SPACE_3]);

    let mut body: Vec<Element<'a, Message>> = vec![pad(header.boxed())];
    let mut rows = 10;
    if live {
        let compact = emphasis == Emphasis::Curve;
        let caption = state.flow.caption();
        let flow = container(state.flow.viewer(ctx, caption, live_notice(state, t), compact).map(Message::Flow)).padding([0.0, SPACE_3]).width(Fill);
        match emphasis {
            Emphasis::Flow => {
                body.push(flow.height(Fill).boxed());
                body.push(curve.height(Length::Fixed(CURVE_STRIP)).boxed());
                rows = 4;
            }
            Emphasis::Curve => {
                body.push(flow.height(Length::Fixed(FLOW_STRIP)).boxed());
                body.push(curve.height(Fill).boxed());
            }
        }
    } else {
        body.push(curve.height(Fill).boxed());
    }
    if s.running {
        let named = t.machines().len() > 1;
        for (unit, live) in t.lives() {
            let machine = named.then(|| t.machines()[live.machine].name.as_str());
            body.push(pad(now(unit, live, machine)));
        }
    }
    body.push(rule());
    body.push(container(points(t)).height(Length::Fixed(list_height(t, rows))).padding([0.0, SPACE_3]).boxed());
    Some(column(body).height(Fill).boxed())
}

/// Height of the Cd curve while the flow has the room, and of the flow while the curve has it (px).
const CURVE_STRIP: f32 = 126.0;
const FLOW_STRIP: f32 = 150.0;

/// What the live picture says when it is not the flow of the point in flight: a remote machine sends no snapshots (its final field of
/// each point arrives when the point ends), and before the first one there is nothing yet.
pub fn live_notice(state: &State, t: &Tracker) -> Option<String> {
    let shown = state.flow.picture_mach();
    let flying: Vec<(usize, f64)> = t.lives().map(|(u, l)| (l.machine, u.mach)).collect();
    let on = |m: &usize| t.machines().get(*m);
    let elsewhere = !flying.is_empty() && flying.iter().all(|(m, _)| on(m).is_some_and(|r| r.remote));
    let current = shown.is_some_and(|pm| flying.iter().any(|(_, m)| (m - pm).abs() < 1e-9));
    match (elsewhere, shown) {
        (true, _) if !current => {
            let name = flying.first().and_then(|(m, _)| on(m)).map_or("a remote machine", |r| r.name.as_str());
            Some(format!("No live picture: solving on {name}. The field of each point arrives when it ends."))
        }
        (_, None) => Some("Waiting for the first picture".to_string()),
        _ => None,
    }
}

/// Height of the list of points: every row up to `rows`, then it scrolls.
fn list_height(t: &Tracker, rows: usize) -> f32 {
    (ROW_H * (t.total().min(rows) as f32 + 1.0) + SPACE_2).max(120.0)
}

const ROW_H: f32 = 20.0;

/// A point in flight: iteration, residual, the monitor and its range against the tolerance, and the monitor's trace; with several
/// machines working there is one of these per busy machine, headed with its name.
fn now<'a>(unit: &Unit, live: &'a Live, machine: Option<&'a str>) -> Element<'a, Message> {
    let q = name(phase_of(unit));
    let at = match unit.alpha_deg {
        Some(a) => format!("Mach {:.2} · {a:.0}°", unit.mach),
        None => format!("Mach {:.2}", unit.mach),
    };
    let value = if live.monitor.is_finite() { format!("{q} {:.4}", live.monitor) } else { format!("{q} not sampled yet") };
    let settle = match (live.spread, live.tolerance) {
        (Some(s), Some(tol)) => format!("{q} range {:.2} % over the last 500 iterations · the point ends when it is under {:.2} %", 100.0 * s, 100.0 * tol),
        (None, Some(tol)) => format!("collecting the first 500 iterations · the point ends when the {q} range is under {:.2} %", 100.0 * tol),
        _ => "the point ends when the residual has dropped enough".to_string(),
    };
    let head = row![ui::label(machine.unwrap_or("NOW")), mono(at, 12.5, |t| t.ink), mono(format!("iteration {}", thousands(live.iteration)), 12.5, |t| t.ink)]
        .spacing(SPACE_2)
        .align_y(iced::Center);
    let figures = row![
        muted(format!("residual down {:.1} orders", live.residual_orders), 12.0),
        muted("·", 12.0),
        mono(value, 12.0, |t| t.ink_muted),
        muted(format!("· cap {}", thousands(live.max_iterations)), 12.0)
    ]
    .spacing(SPACE_1);
    // With several machines at work each strip is short: the stop rule's sentence is left to the single-machine screen.
    let text_col = if machine.is_some() { column![head, figures] } else { column![head, figures, muted(settle, 12.0)] }.spacing(SPACE_1).width(Fill);
    let trace_h = if machine.is_some() { 40.0 } else { 56.0 };
    let trace = canvas::Canvas::new(Trace { history: &live.history, tolerance: live.tolerance }).width(Length::Fixed(220.0)).height(Length::Fixed(trace_h));
    row![text_col, trace].spacing(SPACE_3).align_y(iced::Center).boxed()
}

/// Every unit with its state: a mark, Mach (and angle), outcome, value, iterations and time.
fn points<'a>(t: &'a Tracker) -> Element<'a, Message> {
    let cell = |s: String, width: Length, color: fn(&Tokens) -> Color| -> Element<'a, Message> { container(mono(s, 12.0, color)).width(width).boxed() };
    let several = t.machines().len() > 1;
    let machine_of = |i: usize| -> String {
        match t.slot(i) {
            Slot::Done(d) => t.machines().get(d.machine).map(|m| m.name.clone()),
            Slot::Running { machine, .. } => t.machines().get(*machine).map(|m| m.name.clone()),
            Slot::Pending => None,
        }
        .unwrap_or_default()
    };
    let mut rows: Vec<Element<'a, Message>> = Vec::new();
    for (i, u) in t.units().iter().enumerate() {
        let what = match u.alpha_deg {
            Some(a) => format!("M {:.2} · {a:.0}°", u.mach),
            None => format!("M {:.2}", u.mach),
        };
        let running = t.lives().any(|(l, _)| l == u);
        let (mark, state, value, iters, time): (Element<'a, Message>, (String, fn(&Tokens) -> Color), String, String, String) = match t.slot(i) {
            Slot::Done(d) => {
                // a point the solver had to restart at a lower CFL limit says so, in the warning tone
                let restart = d.point.and_then(|p| p.restart);
                let (mark, tone): (Element<'a, Message>, fn(&Tokens) -> Color) = if d.outcome.is_settled() && restart.is_none() {
                    (glyph::icon(Mark::Check, Tone::Ok, 12.0), |t| t.ink)
                } else {
                    (glyph::icon(Mark::Warning, Tone::Warn, 12.0), |t| t.warn)
                };
                let value = d.value.map_or_else(|| "interpolated".to_string(), |v| format!("{} {v:.4}", name(phase_of(u))));
                let value = match d.cp {
                    Some(cp) => format!("{value} · CP {cp:.3} m"),
                    None => value,
                };
                let word = match &restart {
                    Some(r) => format!("{} · {}", d.outcome.word(), super::cfd::restart_tag(r)),
                    None => d.outcome.word().to_string(),
                };
                (mark, (word, tone), value, format!("{} it", thousands(d.iterations)), duration_text(d.seconds))
            }
            Slot::Running { iterations, ended, .. } => {
                let it = if running { t.lives().find(|(l, _)| *l == u).map_or(*iterations, |(_, l)| l.iteration) } else { *iterations };
                let (tone, word): (Tone, &str) = match (running, ended) {
                    (true, _) => (Tone::Accent, "solving"),
                    (false, Some(_)) => (Tone::Muted, "finishing"),
                    (false, None) => (Tone::Faint, "stopped"),
                };
                (
                    glyph::icon(Mark::Play, tone, 12.0),
                    (word.to_string(), if running { |t| t.ink } else { |t| t.ink_muted }),
                    String::new(),
                    format!("{} it", thousands(it)),
                    String::new(),
                )
            }
            Slot::Pending => (Space::new().width(12).height(12).boxed(), ("waiting".to_string(), |t| t.ink_faint), String::new(), String::new(), String::new()),
        };
        let dim: fn(&Tokens) -> Color = if matches!(t.slot(i), Slot::Pending) { |t| t.ink_faint } else { |t| t.ink };
        rows.push(
            container(
                row![
                    mark,
                    cell(what, 112.0.into(), dim),
                    cell(state.0, if several { 80.0 } else { 88.0 }.into(), state.1),
                    cell(if several { machine_of(i) } else { String::new() }, if several { Length::Fixed(104.0) } else { Length::Shrink }, |t| t.ink_muted),
                    cell(value, Fill, |t| t.ink_muted),
                    cell(iters, if several { 74.0 } else { 84.0 }.into(), |t| t.ink_muted),
                    cell(time, if several { 64.0 } else { 76.0 }.into(), |t| t.ink_muted)
                ]
                .spacing(SPACE_2)
                .align_y(iced::Center),
            )
            .height(ROW_H)
            .center_y(ROW_H)
            .boxed(),
        );
    }
    scrollable(column(rows)).width(Fill).height(Fill).style(theme::scroll).boxed()
}

// ---------------------------------------------------------------------------------------------------------------------
// The curve
// ---------------------------------------------------------------------------------------------------------------------

/// A solved point on the curve.
#[derive(Debug, Clone, PartialEq)]
pub struct Mark2 {
    /// Mach number.
    pub x: f64,
    /// The value (the reference's, or the last solved, where the solver gave none).
    pub y: f64,
    /// How it ended.
    pub outcome: Outcome,
}

/// One curve: the points solved and the points to come.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveData {
    /// The curve's name (`Cd`, `CN at 6°`).
    pub label: String,
    /// Its colour.
    pub color: SeriesColor,
    /// The solved points, in Mach order.
    pub solved: Vec<Mark2>,
    /// The points to come (the one in flight included) at the value they are expected at.
    pub pending: Vec<(f64, f64)>,
}

/// What the curve view draws.
#[derive(Debug, Clone, PartialEq)]
pub struct PlotData {
    /// Mach range.
    pub x: (f64, f64),
    /// Value range (from zero).
    pub y: (f64, f64),
    /// The reference method's curve and its name.
    pub reference: Option<(String, Vec<(f64, f64)>)>,
    /// The curves: one for the drag table, one per angle for the 3D solve.
    pub curves: Vec<CurveData>,
    /// The points in flight (one per busy machine) at the monitor's value now.
    pub live: Vec<(f64, f64)>,
    /// The quantity (`Cd` or `CN`).
    pub quantity: &'static str,
}

/// Value of the piecewise-linear `curve` at `x`, held at its ends.
fn at(curve: &[(f64, f64)], x: f64) -> Option<f64> {
    let (first, last) = (curve.first()?, curve.last()?);
    if x <= first.0 {
        return Some(first.1);
    }
    if x >= last.0 {
        return Some(last.1);
    }
    let i = curve.partition_point(|p| p.0 <= x);
    let (a, b) = (curve[i - 1], curve[i]);
    Some(a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0))
}

/// What the curve view draws for `t`: where each solved point and each point to come is, the range, the reference.
/// A point to come is expected on the reference curve, or at the nearest solved point of its curve without one, or at 0.5; a point
/// the solver gave no value for sits there too.
pub fn plot_data(t: &Tracker, reference: Option<(String, Vec<(f64, f64)>)>) -> PlotData {
    let phase = t.phase();
    let in_phase = |u: &Unit| phase_of(u) == phase;
    let mut machs: Vec<f64> = Vec::new();
    for u in t.units().iter().filter(|u| in_phase(u)) {
        if machs.last().is_none_or(|l| (l - u.mach).abs() > 1e-9) {
            machs.push(u.mach);
        }
    }
    let (lo, hi) = (machs.first().copied().unwrap_or(0.0), machs.last().copied().unwrap_or(1.0));
    let pad = if hi > lo { 0.04 * (hi - lo) } else { 0.1 };
    let mut alphas: Vec<Option<f64>> = Vec::new();
    for u in t.units().iter().filter(|u| in_phase(u)) {
        if !alphas.contains(&u.alpha_deg) {
            alphas.push(u.alpha_deg);
        }
    }
    let quantity = name(phase);
    let ref_curve = reference.as_ref().map(|r| r.1.as_slice()).unwrap_or_default();
    let mut curves = Vec::new();
    for (k, alpha) in alphas.iter().enumerate() {
        let units: Vec<(usize, &Unit)> = t.units().iter().enumerate().filter(|(_, u)| u.alpha_deg == *alpha).collect();
        let solved_values: Vec<(f64, f64)> =
            units.iter().filter_map(|(i, u)| if let Slot::Done(d) = t.slot(*i) { d.value.map(|v| (u.mach, v)) } else { None }).collect();
        let expected = |mach: f64| -> f64 {
            at(ref_curve, mach).or_else(|| solved_values.iter().min_by(|a, b| (a.0 - mach).abs().total_cmp(&(b.0 - mach).abs())).map(|p| p.1)).unwrap_or(0.5)
        };
        let (mut solved, mut pending) = (Vec::new(), Vec::new());
        for (i, u) in &units {
            match t.slot(*i) {
                Slot::Done(d) => solved.push(Mark2 { x: u.mach, y: d.value.unwrap_or_else(|| expected(u.mach)), outcome: d.outcome }),
                _ => pending.push((u.mach, expected(u.mach))),
            }
        }
        let (label, color) = match alpha {
            None => (quantity.to_string(), SeriesColor::Ink),
            Some(a) => (format!("{quantity} at {a:.0}°"), SeriesColor::Series(k as u8 + 1)),
        };
        curves.push(CurveData { label, color, solved, pending });
    }
    let mut top = ref_curve.iter().filter(|p| p.0 >= lo - pad && p.0 <= hi + pad).map(|p| p.1).fold(0.0f64, f64::max);
    for c in &curves {
        top = c.solved.iter().map(|m| m.y).chain(c.pending.iter().map(|p| p.1)).fold(top, f64::max);
    }
    let y = (0.0, (top * 1.12).max(0.1));
    // The solver's first iterations swing far off the answer (a real run reports a Cd of 1.43 before it settles at 0.41): the ring
    // is held at the edge of the range, not the range stretched to it.
    let live = t.lives().filter_map(|(u, l)| l.monitor.is_finite().then_some((u.mach, l.monitor.clamp(y.0, y.1)))).collect();
    PlotData { x: (lo - pad, hi + pad), y, reference, curves, live, quantity }
}

/// The canvas of [`PlotData`].
struct SolvePlot {
    data: PlotData,
}

const GUTTER_L: f32 = 46.0;
const GUTTER_R: f32 = 18.0;
const GUTTER_T: f32 = 34.0;
const GUTTER_B: f32 = 40.0;

fn put(frame: &mut Frame, content: &str, at: Point, color: Color, size: f32, ax: iced::advanced::text::Alignment, ay: alignment::Vertical) {
    frame.fill_text(Text { content: content.to_string(), position: at, color, size: Pixels(size), font: MONO, align_x: ax, align_y: ay, ..Text::default() });
}

fn stroke<'a>(color: Color, width: f32, dash: Option<&'a [f32]>) -> Stroke<'a> {
    let s = Stroke::default().with_color(color).with_width(width).with_line_join(LineJoin::Round);
    match dash {
        Some(segments) => Stroke { line_dash: LineDash { segments, offset: 0 }, ..s },
        None => s,
    }
}

impl canvas::Program<Message> for SolvePlot {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        use iced::advanced::text::Alignment::{Center, Left, Right};
        let t = theme::tokens(theme);
        let d = &self.data;
        let mut f = Frame::new(renderer, bounds.size());
        let plot = Rectangle {
            x: GUTTER_L,
            y: GUTTER_T,
            width: (bounds.width - GUTTER_L - GUTTER_R).max(10.0),
            height: (bounds.height - GUTTER_T - GUTTER_B).max(10.0),
        };
        let px = |x: f64| plot.x + ((x - d.x.0) / (d.x.1 - d.x.0)) as f32 * plot.width;
        let py = |y: f64| plot.y + plot.height - ((y - d.y.0) / (d.y.1 - d.y.0)) as f32 * plot.height;

        // grid and ticks
        for v in nice_ticks(d.y.0, d.y.1, ((plot.height / 24.0) as usize).clamp(2, 5)) {
            let y = py(v).floor() + 0.5;
            f.stroke(&Path::line(Point::new(plot.x, y), Point::new(plot.x + plot.width, y)), stroke(t.line_100, 1.0, None));
            put(&mut f, &fmt_fixed(v, if d.y.1 > 2.0 { 1 } else { 2 }), Point::new(plot.x - 6.0, y), t.ink_faint, 10.5, Right, alignment::Vertical::Center);
        }
        for v in nice_ticks(d.x.0, d.x.1, 8) {
            let x = px(v).floor() + 0.5;
            f.stroke(&Path::line(Point::new(x, plot.y), Point::new(x, plot.y + plot.height)), stroke(t.line_100, 1.0, None));
            put(&mut f, &fmt_fixed(v, 1), Point::new(x, plot.y + plot.height + 5.0), t.ink_faint, 10.5, Center, alignment::Vertical::Top);
        }
        put(&mut f, "Mach number", Point::new(plot.x + plot.width / 2.0, plot.y + plot.height + 20.0), t.ink_faint, 10.5, Center, alignment::Vertical::Top);
        put(&mut f, d.quantity, Point::new(plot.x - 6.0, plot.y - 2.0), t.ink_faint, 10.5, Right, alignment::Vertical::Bottom);
        f.stroke(&Path::rectangle(Point::new(plot.x, plot.y), plot.size()), stroke(t.line_200, 1.0, None));

        // the reference: faint, under everything
        if let Some((label, curve)) = &d.reference {
            let pts: Vec<Point> = curve.iter().filter(|p| p.0 >= d.x.0 && p.0 <= d.x.1).map(|p| Point::new(px(p.0), py(p.1))).collect();
            if pts.len() > 1 {
                f.stroke(
                    &Path::new(|b| {
                        b.move_to(pts[0]);
                        pts[1..].iter().for_each(|p| b.line_to(*p));
                    }),
                    stroke(t.ink_faint, 1.4, None),
                );
                // the reference's name: in the header row, left, clear of the data
                put(&mut f, label, Point::new(plot.x, 10.0), t.ink_faint, 10.5, Left, alignment::Vertical::Center);
            }
        }

        for c in &d.curves {
            let color = c.color.resolve(t);
            // the solved curve, solid
            let solved: Vec<Point> = c.solved.iter().map(|m| Point::new(px(m.x), py(m.y))).collect();
            if solved.len() > 1 {
                f.stroke(
                    &Path::new(|b| {
                        b.move_to(solved[0]);
                        solved[1..].iter().for_each(|p| b.line_to(*p));
                    }),
                    stroke(color, 1.8, None),
                );
            }
            // the rest, dashed from the last solved point through the points to come
            let mut rest: Vec<Point> = solved.last().copied().into_iter().collect();
            rest.extend(c.pending.iter().map(|p| Point::new(px(p.0), py(p.1))));
            if rest.len() > 1 {
                f.stroke(
                    &Path::new(|b| {
                        b.move_to(rest[0]);
                        rest[1..].iter().for_each(|p| b.line_to(*p));
                    }),
                    stroke(t.ink_muted, 1.2, Some(&[5.0, 4.0])),
                );
            }
            for p in &c.pending {
                f.fill(&Path::circle(Point::new(px(p.0), py(p.1)), 3.6), t.bg_000);
                f.stroke(&Path::circle(Point::new(px(p.0), py(p.1)), 3.6), stroke(t.ink_muted, 1.3, None));
            }
            for m in &c.solved {
                let at = Point::new(px(m.x), py(m.y));
                if m.outcome.is_settled() {
                    f.fill(&Path::circle(at, 3.8), color);
                } else {
                    // a point that did not settle (or has no value): a warning triangle
                    let tri = Path::new(|b| {
                        b.move_to(Point::new(at.x, at.y - 6.0));
                        b.line_to(Point::new(at.x + 5.5, at.y + 4.0));
                        b.line_to(Point::new(at.x - 5.5, at.y + 4.0));
                        b.close();
                    });
                    f.fill(&tri, t.warn);
                    if m.outcome == Outcome::Fallback {
                        f.stroke(&tri, stroke(t.bg_000, 1.2, None));
                    }
                }
            }
        }
        for &(mach, y) in &d.live {
            let at = Point::new(px(mach), py(y));
            f.stroke(&Path::circle(at, 7.0), stroke(t.accent, 1.8, None));
            f.fill(&Path::circle(at, 2.2), t.accent);
        }

        // the key: a header row above the plot, right-aligned, so no label sits on data
        let labels = ["solved", "to come", "not settled", "solving now"];
        let shown = if d.live.is_empty() { &labels[..3] } else { &labels[..] };
        let width: f32 = shown.iter().map(|l| 28.0 + 6.3 * l.chars().count() as f32).sum::<f32>() - 14.0;
        let mut x = (plot.x + plot.width - width).max(plot.x);
        let y = 10.0;
        let mut key = |f: &mut Frame, glyph: &dyn Fn(&mut Frame, Point), label: &str| {
            glyph(f, Point::new(x + 4.0, y));
            put(f, label, Point::new(x + 14.0, y), t.ink_muted, 10.5, Left, alignment::Vertical::Center);
            x += 14.0 + 6.3 * label.chars().count() as f32 + 14.0;
        };
        let ink = d.curves.first().map_or(t.ink, |c| c.color.resolve(t));
        key(&mut f, &|f, p| f.fill(&Path::circle(p, 3.8), ink), "solved");
        key(
            &mut f,
            &|f, p| {
                f.fill(&Path::circle(p, 3.6), t.bg_000);
                f.stroke(&Path::circle(p, 3.6), stroke(t.ink_muted, 1.3, None));
            },
            "to come",
        );
        key(
            &mut f,
            &|f, p| {
                f.fill(
                    &Path::new(|b| {
                        b.move_to(Point::new(p.x, p.y - 5.0));
                        b.line_to(Point::new(p.x + 5.0, p.y + 4.0));
                        b.line_to(Point::new(p.x - 5.0, p.y + 4.0));
                        b.close();
                    }),
                    t.warn,
                );
            },
            "not settled",
        );
        if !d.live.is_empty() {
            key(
                &mut f,
                &|f, p| {
                    f.stroke(&Path::circle(p, 5.5), stroke(t.accent, 1.6, None));
                    f.fill(&Path::circle(p, 2.0), t.accent);
                },
                "solving now",
            );
        }
        vec![f.into_geometry()]
    }
}

/// The trace of the monitor of the point in flight, with the band the stop rule wants its range to fit in.
struct Trace<'a> {
    history: &'a [(usize, f64)],
    tolerance: Option<f64>,
}

impl<M> canvas::Program<M> for Trace<'_> {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let mut f = Frame::new(renderer, bounds.size());
        f.stroke(&Path::rectangle(Point::ORIGIN, bounds.size()), stroke(t.line_100, 1.0, None));
        let h = self.history;
        if h.len() >= 2 {
            let window = &h[h.len().saturating_sub(10)..];
            let mean = window.iter().map(|p| p.1).sum::<f64>() / window.len() as f64;
            let (mut lo, mut hi) = h.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, u), p| (l.min(p.1), u.max(p.1)));
            let band = self.tolerance.map(|tol| tol * mean.abs());
            if let Some(b) = band {
                lo = lo.min(mean - 0.5 * b);
                hi = hi.max(mean + 0.5 * b);
            }
            let span = (hi - lo).max(1e-12);
            let (x0, x1) = (h[0].0 as f64, h[h.len() - 1].0 as f64);
            let pad = 6.0;
            let px = |i: usize| pad + ((i as f64 - x0) / (x1 - x0).max(1.0)) as f32 * (bounds.width - 2.0 * pad);
            let py = |v: f64| pad + (1.0 - ((v - lo) / span) as f32) * (bounds.height - 2.0 * pad);
            if let Some(b) = band {
                let (top, bottom) = (py(mean + 0.5 * b), py(mean - 0.5 * b));
                f.fill_rectangle(
                    Point::new(px(window[0].0), top),
                    iced::Size::new(bounds.width - pad - px(window[0].0), (bottom - top).max(1.5)),
                    t.ok.scale_alpha(0.25),
                );
            }
            f.stroke(
                &Path::new(|b| {
                    b.move_to(Point::new(px(h[0].0), py(h[0].1)));
                    h[1..].iter().for_each(|p| b.line_to(Point::new(px(p.0), py(p.1))));
                }),
                stroke(t.ink, 1.4, None),
            );
        }
        vec![f.into_geometry()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analyze::solve::{Iteration, PointDone};
    use bowshock_model::{CfdLiftPoint, CfdPoint, CfdQuality};

    fn drag_done(mach: f64, cd: Option<f64>, q: Option<CfdQuality>, at: f64) -> PointDone {
        PointDone {
            machine: 0,
            at,
            mach,
            alpha_deg: None,
            outcome: Outcome::from_quality(q),
            point: cd.zip(q).map(|(cd, quality)| CfdPoint {
                mach,
                cd,
                friction: 0.1,
                pressure: 0.1,
                base: 0.1,
                fins: 0.1,
                other: 0.0,
                interference: 0.0,
                quality,
                restart: None,
            }),
            lift: None,
            iterations: 2000,
            cpu: false,
            fallback: None,
        }
    }

    fn reference() -> Option<(String, Vec<(f64, f64)>)> {
        Some(("Extended".into(), vec![(0.1, 0.4), (1.0, 0.8), (3.0, 0.4)]))
    }

    #[test]
    fn solved_points_are_solid_the_rest_sit_on_the_reference_and_a_fallback_has_no_value_of_its_own() {
        let mut t = Tracker::new(Tracker::drag_units(&[0.1, 0.5, 1.0, 2.0]));
        t.on_done(&drag_done(0.1, Some(0.42), Some(CfdQuality::Converged), 10.0));
        t.on_done(&drag_done(0.5, None, None, 20.0));
        let d = plot_data(&t, reference());
        assert_eq!(d.curves.len(), 1);
        let c = &d.curves[0];
        assert_eq!(c.solved.iter().map(|m| (m.x, m.outcome)).collect::<Vec<_>>(), vec![(0.1, Outcome::Converged), (0.5, Outcome::Fallback)]);
        assert_eq!(c.solved[0].y, 0.42);
        // the fallback is drawn where the reference has it: 0.4 + (0.8 - 0.4) * (0.5 - 0.1) / 0.9
        assert!((c.solved[1].y - (0.4 + 0.4 * 0.4 / 0.9)).abs() < 1e-12);
        assert_eq!(c.pending, vec![(1.0, 0.8), (2.0, 0.8 - 0.4 * (1.0 / 2.0))]);
        assert_eq!(d.quantity, "Cd");
        assert!(d.live.is_empty());
        // from zero, to the largest value with headroom; the Mach range is padded
        assert_eq!(d.y.0, 0.0);
        assert!((d.y.1 - 0.8 * 1.12).abs() < 1e-12);
        assert!(d.x.0 < 0.1 && d.x.1 > 2.0);
    }

    #[test]
    fn the_point_in_flight_is_at_the_monitors_value_and_without_a_reference_the_rest_follow_the_last_solved_point() {
        let mut t = Tracker::new(Tracker::drag_units(&[0.5, 0.9, 1.0]));
        t.on_done(&drag_done(0.5, Some(0.45), Some(CfdQuality::Converged), 10.0));
        t.on_iteration(&Iteration {
            machine: 0,
            at: 12.0,
            mach: 0.9,
            alpha_deg: None,
            iteration: 400,
            max_iterations: 80_000,
            residual_orders: 1.0,
            monitor: 0.47,
            spread: None,
            tolerance: Some(0.002),
        });
        let d = plot_data(&t, None);
        assert_eq!(d.live, vec![(0.9, 0.47)]);
        assert_eq!(d.curves[0].pending, vec![(0.9, 0.45), (1.0, 0.45)], "no reference: held at the nearest solved point");
        // the first iterations swing far off the answer (a real run reports Cd 1.43 before it settles at 0.41): the ring is held
        // at the top of the range instead of the range being stretched to it
        t.on_iteration(&Iteration {
            machine: 0,
            at: 13.0,
            mach: 0.9,
            alpha_deg: None,
            iteration: 450,
            max_iterations: 80_000,
            residual_orders: 1.0,
            monitor: 1.43,
            spread: None,
            tolerance: Some(0.002),
        });
        let d = plot_data(&t, None);
        assert_eq!(d.live, vec![(0.9, d.y.1)]);
        assert!((d.y.1 - 0.45 * 1.12).abs() < 1e-12);
        // nothing known at all: a visible default
        let empty = plot_data(&Tracker::new(Tracker::drag_units(&[0.5, 1.0])), None);
        assert_eq!(empty.curves[0].pending, vec![(0.5, 0.5), (1.0, 0.5)]);
    }

    #[test]
    fn the_3d_solve_has_a_curve_per_angle() {
        let units = Tracker::stability_units(&[0.8, 1.1], &[2.0, 6.0]);
        let mut t = Tracker::new(units);
        for (a, cn) in [(2.0, 0.09), (6.0, 0.3)] {
            t.on_done(&PointDone {
                machine: 0,
                at: 10.0,
                mach: 0.8,
                alpha_deg: Some(a),
                outcome: Outcome::Converged,
                point: None,
                lift: Some(CfdLiftPoint { mach: 0.8, alpha_deg: a, cn, x_cp: 0.9, cd: None, cm: None, quality: CfdQuality::Converged }),
                iterations: 1500,
                cpu: false,
                fallback: None,
            });
        }
        let d = plot_data(&t, None);
        assert_eq!(d.quantity, "CN");
        assert_eq!(d.curves.iter().map(|c| c.label.as_str()).collect::<Vec<_>>(), vec!["CN at 2°", "CN at 6°"]);
        assert_eq!(d.curves[0].solved[0].y, 0.09);
        assert_eq!(d.curves[1].solved[0].y, 0.3);
        assert_eq!(d.curves[1].pending, vec![(1.1, 0.3)], "held at that angle's last solved point");
    }

    #[test]
    fn the_headline_names_the_point_and_the_clock_waits_for_two_points() {
        let mut t = Tracker::new(Tracker::drag_units(&[0.1, 0.5, 0.8]));
        assert_eq!(headline(&t), "Starting the solver");
        t.on_iteration(&Iteration {
            machine: 0,
            at: 3.0,
            mach: 0.5,
            alpha_deg: None,
            iteration: 50,
            max_iterations: 1000,
            residual_orders: 0.0,
            monitor: f64::NAN,
            spread: None,
            tolerance: None,
        });
        assert_eq!(headline(&t), "Mach 0.50 · point 2 of 3");
        assert_eq!(clock(&t, 61.0), ("1 min 1 s elapsed".to_string(), "estimating…".to_string()));
    }
}
