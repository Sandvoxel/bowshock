//! Drawing of the Analyze screen: the left panel (analyses, method, conditions), the centre (stability gauge, linked plots,
//! landing map, table view) and the right panel (values at the cursor, export).

use std::time::Instant;

use iced::widget::{button, column, container, row, scrollable, text, tooltip, Space};
use iced::{alignment, Element, Fill, Length, Theme, Widget};

use super::cfd;
use super::compute::{Analysis, FinishChoice, Method};
use super::import::Source;
use super::plots::Built;
use super::readout::{self, Row, Tag};
use super::{Comparison, MachField, Message, State};
use crate::app::{ui, Ctx};
use crate::theme::{self, MONO, SANS_SEMIBOLD};
use crate::tokens::{BIN_W, CONTROL_H, RAIL_W, RIGHT_PANEL_W, SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::units::Quantity;
use crate::widgets::button::{button as push, Weight};
use crate::widgets::chip::{chip, Severity};
use crate::widgets::field::{self, rule};
use crate::widgets::ground_track::GroundTrackView;
use crate::widgets::plot::{self, PlotView, SeriesColor};
use crate::widgets::unit_input::unit_input;
use bowshock_model::CfdGrid;

/// Width of the label column of the Conditions rows (wider than the inspector's: "Angle of attack" must fit).
const LABEL_W: f32 = 112.0;
/// Height of an analyses list row.
const ROW_H: f32 = 36.0;

static FINISHES: [FinishChoice; 6] = FinishChoice::ALL;

const RASAERO_REASON: &str = "RASAero's .CDX1 is a proprietary project format; exporting it is not implemented";
const NO_COMPARISON: &str = "No comparison is loaded";
const NO_CFD_TABLE: &str = "Solve the CFD table first: choose CFD table";

/// The whole screen.
pub fn screen<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let panel = |content: Element<'a, Message>, width: f32| container(content).width(width).height(Fill).style(theme::panel);
    row![
        panel(left(state, ctx), RAIL_W + BIN_W),
        container(centre(state, ctx)).width(Fill).height(Fill).style(plot::surface),
        panel(right(state, ctx), RIGHT_PANEL_W),
    ]
    .height(Fill)
    .boxed()
}

fn muted<'a>(s: impl text::IntoFragment<'a>, size: f32) -> Element<'a, Message> {
    text(s).size(size).style(theme::muted).boxed()
}

// ---------------------------------------------------------------------------------------------------------------------
// Left
// ---------------------------------------------------------------------------------------------------------------------

fn left<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let head = |title: &'a str, ruled: bool| -> Element<'a, Message> {
        let label = container(ui::label(title)).padding([SPACE_2, SPACE_3]);
        if ruled {
            column![rule(), label].boxed()
        } else {
            label.boxed()
        }
    };
    let controls: Vec<Element<'a, Message>> = if state.analysis == Analysis::Optimize {
        vec![head("OPTIMIZE", true), state.optimize.left(ctx), Space::new().height(SPACE_3).boxed()]
    } else {
        vec![
            head("METHOD", false),
            container(method(state, ctx)).padding([0.0, SPACE_3]).boxed(),
            head("CONDITIONS", true),
            container(conditions(state, ctx)).padding([0.0, SPACE_3]).boxed(),
            Space::new().height(SPACE_3).boxed(),
        ]
    };
    let body = column![head("ANALYSES", false), analyses(state), column(controls).spacing(SPACE_1)].spacing(SPACE_1);
    scrollable(body).width(Fill).height(Fill).boxed()
}

fn analyses(state: &State) -> Element<'_, Message> {
    let rows: Vec<Element<'_, Message>> = Analysis::ALL
        .iter()
        .map(|&a| {
            let active = a == state.analysis;
            let content = row![text(a.name()).size(TEXT_BODY), Space::new().width(Fill), text(a.hint()).font(MONO).size(TEXT_LABEL).style(theme::faint)]
                .align_y(iced::Center);
            let entry = button(content).width(Fill).height(ROW_H).padding([0.0, SPACE_3]).on_press(Message::Select(a)).style(move |theme: &Theme, status| {
                let t = theme::tokens(theme);
                let hot = matches!(status, button::Status::Hovered | button::Status::Pressed);
                button::Style {
                    background: if active {
                        Some(t.bg_300.into())
                    } else if hot {
                        Some(t.bg_400.into())
                    } else {
                        None
                    },
                    text_color: t.ink,
                    border: iced::Border::default(),
                    ..button::Style::default()
                }
            });
            let edge = container(Space::new()).width(2).height(ROW_H).style(theme::underline(active));
            column![row![edge, entry], rule()].boxed()
        })
        .collect();
    column![rule(), column(rows)].boxed()
}

fn bubble<'a>(tip: &'a str) -> Element<'a, Message> {
    container(text(tip).size(TEXT_BODY)).width(260).padding([SPACE_1, SPACE_2]).style(|t: &Theme| theme::tooltip(t)).boxed()
}

fn method<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let mut cells: Vec<Element<'_, Message>> = Vec::new();
    for (i, m) in [Method::Barrowman, Method::Extended, Method::CfdTable].into_iter().enumerate() {
        if i > 0 {
            cells.push(container(Space::new()).width(1).height(CONTROL_H - 2.0).style(theme::hairline).boxed());
        }
        let active = m == state.method;
        let b = button(text(m.name()).size(TEXT_BODY))
            .height(CONTROL_H - 4.0)
            .padding([0.0, SPACE_3])
            .style(move |theme: &Theme, status| theme::segment(active)(theme, status))
            .on_press(Message::Method(m));
        let cell = column![b, container(Space::new()).width(Fill).height(2).style(theme::underline(active))];
        cells.push(tooltip(cell, bubble(m.tip())).position(tooltip::Position::Bottom).gap(SPACE_1).boxed());
    }
    let strip = container(row(cells)).padding(1).style(|theme: &Theme| theme::segmented_frame(theme));

    let cfd_line = state.method == Method::CfdTable && super::uses_cfd(state.analysis);
    let mut lines: Vec<Element<'_, Message>> = Vec::new();
    match (state.output(), &state.error) {
        (_, Some(e)) => lines.push(muted(format!("Cannot analyse: {e}"), 12.0)),
        _ if cfd_line => {
            lines.extend(super::solve_view::compact(state));
            lines.extend(cfd_lines(state).into_iter().map(|l| muted(l, 12.0)));
        }
        (None, None) => lines.push(muted("Solving…", 12.0)),
        (Some(out), None) => {
            let busy = if state.pending { "solving…".to_string() } else { state.computed_at.map_or_else(String::new, ago) };
            let took =
                if out.elapsed.as_millis() < 1000 { format!("{} ms", out.elapsed.as_millis().max(1)) } else { format!("{:.1} s", out.elapsed.as_secs_f64()) };
            lines.push(muted(format!("{} · computed on CPU in {took} · {busy}", out.points_text()), 12.0));
        }
    }
    if let Some(note) = &state.skipped_note {
        lines.push(muted(note.clone(), 12.0));
    }
    let solving = cfd_line && state.cfd.solving.is_some();
    let stale = cfd_line && state.solve_status() == cfd::Status::Stale;
    let cancelling = state.cfd.solving.as_ref().is_some_and(|s| s.cancelling);
    let resolve: Element<'_, Message> = if solving && cancelling {
        push("Cancelling…", Weight::Secondary).disabled("The solver is ending the run in flight").into()
    } else if solving {
        push("Cancel", Weight::Secondary).on_press(Message::CfdCancel).into()
    } else {
        push("Re-solve", if stale { Weight::Primary } else { Weight::Secondary }).on_press(Message::Resolve).into()
    };
    let compare: Element<'_, Message> = push("Compare ▾", Weight::Ghost).on_press(Message::CompareMenu).into();
    let mut buttons: Vec<Element<'_, Message>> = vec![resolve];
    let shown = matches!(state.cfd.status, cfd::Status::Fresh | cfd::Status::Stale);
    let solved_grid = state.cfd.table.as_ref().map(|t| t.grid);
    if cfd_line && !solving && shown && solved_grid.is_some_and(|g| g != CfdGrid::Draft) && cfd::solves_3d() {
        let quick: Element<'_, Message> = push("Quick", Weight::Secondary).on_press(Message::Quick).into();
        buttons.push(tooltip(quick, bubble(cfd::DRAFT_TOOLTIP)).position(tooltip::Position::Bottom).gap(SPACE_1).boxed());
    }
    if cfd_line && !solving && shown && matches!(solved_grid, Some(CfdGrid::Coarse | CfdGrid::Draft)) {
        let refine: Element<'_, Message> = push("Refine", Weight::Secondary).on_press(Message::Refine).into();
        buttons.push(
            tooltip(refine, bubble("Re-solve on the medium grid: about 3.5x slower, Cd about 1 % different"))
                .position(tooltip::Position::Bottom)
                .gap(SPACE_1)
                .boxed(),
        );
    }
    buttons.push(compare);
    let actions = row(buttons).spacing(SPACE_2);
    let mut col = column![strip, column(lines).spacing(SPACE_1), actions].spacing(SPACE_2).padding([0.0, 0.0]);
    if cfd_line {
        col = col.push(runs_on(ctx));
    }
    if let Some(stopped) = state.cfd.stopped.as_ref().filter(|s| cfd_line && !solving && s.table.is_some()) {
        let keep: Element<'_, Message> = push(&stopped.keep_label, Weight::Primary).on_press(Message::CfdKeep).into();
        let discard: Element<'_, Message> = push("Discard", Weight::Secondary).on_press(Message::CfdDiscard).into();
        // the label with the 3D angles in it is too long to leave room for Discard beside it in the narrow panel
        col = col.push(if stopped.keep_label.chars().count() > 22 {
            column![keep, discard].spacing(SPACE_2).boxed()
        } else {
            row![keep, discard].spacing(SPACE_2).boxed()
        });
    }
    if state.compare_open {
        let item = |label: &'static str, message: Message| -> Element<'_, Message> { push(label, Weight::Secondary).on_press(message).small().into() };
        let mut items: Vec<Element<'_, Message>> = Vec::new();
        for m in [Method::Barrowman, Method::Extended, Method::CfdTable].into_iter().filter(|m| *m != state.method) {
            let label = match m {
                Method::Barrowman => "Compare with Barrowman",
                Method::Extended => "Compare with Extended",
                Method::CfdTable => "Compare with CFD table",
            };
            items.push(if m == Method::CfdTable && !state.cfd_ready() {
                push(label, Weight::Secondary).small().disabled(NO_CFD_TABLE).into()
            } else {
                item(label, Message::CompareWith(m))
            });
        }
        items.push(item("Import OpenRocket CSV…", Message::ImportCompare(Source::OpenRocket)));
        items.push(item("Import RASAero CSV…", Message::ImportCompare(Source::RasAero)));
        items.push(if state.compare.is_some() {
            item("Clear comparison", Message::ClearCompare)
        } else {
            push("Clear comparison", Weight::Secondary).small().disabled(NO_COMPARISON).into()
        });
        col = col.push(column(items).spacing(SPACE_1).boxed());
    }
    if let Some(c) = &state.compare {
        let from = match c {
            Comparison::Method(Method::CfdTable) if !state.cfd_ready() => " · not solved yet".to_string(),
            Comparison::Method(_) => String::new(),
            Comparison::File { name, .. } => format!(" · {name}"),
        };
        col = col.push(muted(format!("Comparing with {}{from}", c.label()), 12.0));
    }
    col.boxed()
}

/// `Runs on: This computer`, `Runs on: <gpu> on <host>` or `Runs on: Shared: 3 machines`, and the button that opens Preferences at Compute.
fn runs_on<'a>(ctx: &'a Ctx) -> Element<'a, Message> {
    let place = ctx.compute.runs_on(&ctx.compute_devices);
    let change: Element<'_, Message> = push("Change…", Weight::Ghost).on_press(Message::OpenPrefs).small().into();
    row![muted(format!("Runs on: {place}"), 12.0), change].spacing(SPACE_2).align_y(iced::Center).boxed()
}

/// The provenance of the CFD table method, line by line: the solve in progress, or what the table is and was not (the kit's
/// `42 Mach × 9 AoA points · solved on GPU (Luma) · 6 min ago`: here the drag at zero incidence and, when the solve took them, the 3D
/// angles), then why the plots show another method if they do. What each part comes from is worded in [`cfd`] (`wording.rs`).
pub(super) fn cfd_lines(state: &State) -> Vec<String> {
    let cfd = &state.cfd;
    let base = state.base_method.name();
    let mut lines = Vec::new();
    if cfd.solving.is_some() {
        // the solve's own block (solve_view::compact) says where it is
    } else {
        match &cfd.status {
            cfd::Status::Unsupported(why) => {
                lines.push(format!("CFD table unavailable: {why}."));
                lines.push(format!("Showing {base} instead."));
            }
            cfd::Status::Missing => {
                lines.push(match &cfd.failed {
                    Some((_, why)) => format!("No CFD table: {why}."),
                    None => "No CFD table yet.".to_string(),
                });
                lines.push(format!("Showing {base} until one is solved; Re-solve starts a solve."));
            }
            cfd::Status::Fresh | cfd::Status::Stale => {
                lines.extend(cfd.failed.iter().filter(|(_, why)| why != "solve cancelled").map(|(_, why)| format!("Solve stopped: {why}")));
            }
        }
    }
    if let Some((out, info)) = state.output().and_then(|o| o.cfd.as_ref().map(|i| (o, i))) {
        if let Some((_, summary)) = cfd.summary.as_ref().filter(|(key, _)| cfd.table.as_ref().is_some_and(|t| t.key == *key)) {
            lines.push(summary.clone());
        }
        let table = cfd.table.as_deref();
        let partial = table.and_then(|t| cfd::partial_of(&cfd.plan, t, state.cond.altitude, state.cond.mach));
        let angles_partial = table.and_then(|t| t.lift.as_ref()).and_then(|l| cfd::lift_partial_of(&cfd.plan, l, state.cond.mach));
        let verdict = state.solve_status();
        let stale = if partial.is_some() || angles_partial.is_some() {
            "partial · "
        } else if verdict == cfd::Status::Stale {
            "stale · "
        } else {
            ""
        };
        let lift = cfd::lift_status(&cfd.plan, table.and_then(|t| t.lift.as_ref()), state.cond.mach);
        let fresh_lift = lift == cfd::Status::Fresh;
        let shown_lift = table.and_then(|t| t.lift.as_ref()).filter(|_| fresh_lift || angles_partial.is_some());
        let points = match out.analysis {
            super::Analysis::Stability => format!("{} · at Mach {:.2}", cfd::table_points(info.nodes, shown_lift), out.conditions.mach_fixed),
            _ => cfd::table_points(info.nodes, shown_lift),
        };
        let grid = match info.grid {
            CfdGrid::Coarse => "coarse",
            CfdGrid::Draft => "draft",
            CfdGrid::Medium => "medium",
        };
        let below = info.scaled_below.map_or_else(String::new, |m| format!(" · M<{m:.1} scaled from Extended"));
        lines.push(format!("{stale}{points} · solved on {} · {grid} grid · {}{below}", info.backend, cfd::ago(info.solved_at)));
        if info.grid == CfdGrid::Draft && cfd.solving.is_some() {
            lines.push(cfd::REFINING_NOTE.to_string());
        }
        if partial.is_some() || angles_partial.is_some() {
            lines.push(cfd::PARTIAL_ACTION.to_string());
        } else if verdict == cfd::Status::Stale {
            lines.push(cfd::stale_reason(cfd.status == cfd::Status::Stale, table.is_some_and(|t| t.lift.is_none()), lift == cfd::Status::Stale));
        }
        lines.push(if fresh_lift { cfd::stability_from_3d() } else { cfd::stability_from_extended() });
        lines.extend(info.fallback.iter().map(|why| format!("The GPU was not used: {why}.")));
        lines.extend(info.notes.iter().cloned());
        if fresh_lift {
            lines.extend(table.and_then(|t| t.lift.as_ref()).into_iter().flat_map(|l| l.notes.iter().cloned()));
        }
    } else if matches!(cfd.status, cfd::Status::Fresh | cfd::Status::Stale) && cfd.solving.is_none() {
        lines.push(format!("The stored table does not cover this Mach range. Showing {base} instead; Re-solve for this range."));
    }
    lines
}

/// A Conditions row: the label column and `control`.
fn row_of<'a>(name: &'a str, control: Element<'a, Message>) -> Element<'a, Message> {
    let label = container(text(name).size(TEXT_BODY).style(theme::muted)).width(LABEL_W).height(CONTROL_H).center_y(CONTROL_H);
    row![label, container(control).width(Fill)].spacing(SPACE_2).align_y(iced::Center).boxed()
}

fn mach_input<'a>(state: &'a State, field: MachField) -> Element<'a, Message> {
    iced::widget::text_input("", &state.mach_text[field as usize])
        .on_input(move |s| Message::MachText(field, s))
        .on_submit(Message::MachCommit)
        .font(MONO)
        .size(TEXT_BODY)
        .padding([4.0, SPACE_2])
        .style(theme::input)
        .width(Fill)
        .boxed()
}

/// The option to leave the 3D angles out of the CFD solve: a drag-only table is much quicker. Only with the CFD table method.
fn angles_row(state: &State) -> Option<Element<'_, Message>> {
    if state.method != Method::CfdTable {
        return None;
    }
    let strip = crate::widgets::segmented::segmented(&[(true, "On"), (false, "Off")], state.cfd.angles, Message::Angles);
    let tip: Element<'_, Message> = tooltip(strip, bubble(cfd::angles_tip())).position(tooltip::Position::Bottom).gap(SPACE_1).boxed();
    Some(row_of(cfd::ANGLES_LABEL, tip))
}

fn conditions<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let c = &state.cond;
    let units = &ctx.units;
    let burnout = row_of("CG for margin", crate::widgets::segmented::segmented(&[(false, "Liftoff"), (true, "Burnout")], c.burnout, Message::Burnout));
    let finish = row_of("Finish", field::select(&FINISHES[..], Some(c.finish), Message::Finish));
    let input = |label: &'static str, value: f64, quantity: Quantity, on: fn(f64) -> Message| -> Element<'a, Message> {
        unit_input(label, value, quantity, units).label_width(LABEL_W).on_commit(on).on_scrub(on).into()
    };
    let mut rows: Vec<Element<'a, Message>> = Vec::new();
    match state.analysis {
        Analysis::Aerodynamics | Analysis::Drag => {
            rows.push(row_of(
                "Mach range",
                row![mach_input(state, MachField::Low), muted("–", TEXT_BODY), mach_input(state, MachField::High)]
                    .spacing(SPACE_2)
                    .align_y(iced::Center)
                    .boxed(),
            ));
            let alpha = unit_input("Angle of attack", c.alpha, Quantity::Angle, units).label_width(LABEL_W).on_commit(Message::Alpha).on_scrub(Message::Alpha);
            rows.push(match state.method {
                Method::Barrowman => alpha.hint("Barrowman ignores the angle (α = 0)").into(),
                Method::CfdTable => alpha.hint(cfd::alpha_hint(state.cfd.angles)).into(),
                Method::Extended => alpha.into(),
            });
            rows.push(input("Altitude", c.altitude, Quantity::Altitude, Message::Altitude));
            rows.extend(angles_row(state));
            rows.push(burnout);
            rows.push(finish);
        }
        Analysis::Stability => {
            rows.push(row_of("Mach", mach_input(state, MachField::Fixed)));
            rows.extend(angles_row(state));
            rows.push(input("Largest angle", c.alpha_max, Quantity::Angle, Message::AlphaMax));
            rows.push(burnout);
            rows.push(finish);
        }
        Analysis::Flutter => {
            rows.push(muted("Martin (NACA TN 4197) bending–torsion estimate · preliminary-design accuracy.", 12.0));
            rows.push(muted(
                "Flutter speed from each fin set's planform, thickness and the shear modulus of its stock, in the air at every moment of the flight up to apogee, against the rocket's speed. The lowest ratio decides.",
                12.0,
            ));
            rows.push(muted("The fin root is taken as clamped, as for a glued or through-the-wall fin; a tab earns no credit.", 12.0));
        }
        Analysis::Dispersion => {
            rows.push(row_of("Runs", crate::widgets::segmented::segmented(&[(50, "50"), (100, "100"), (200, "200")], c.runs, Message::Runs)));
            rows.push(input("Mean wind", c.wind, Quantity::Velocity, Message::Wind));
            rows.push(muted("One-sigma spreads: wind 1 m/s and 10°, rail tilt 1°, thrust 3 %, drag 5 %, dry mass 2 %.", 12.0));
        }
        Analysis::Optimize => {}
    }
    column(rows).spacing(SPACE_2).boxed()
}

/// `just now`, `12 s ago`, `3 min ago`.
fn ago(at: Instant) -> String {
    let s = at.elapsed().as_secs();
    match s {
        0..=4 => "just now".to_string(),
        5..=59 => format!("{s} s ago"),
        _ => format!("{} min ago", s / 60),
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Centre
// ---------------------------------------------------------------------------------------------------------------------

fn framed<'a>(content: Element<'a, Message>) -> Element<'a, Message> {
    container(content).width(Fill).height(Fill).style(theme::ground).boxed()
}

fn centred<'a>(message: String) -> Element<'a, Message> {
    container(text(message).size(TEXT_BODY).style(theme::muted)).center(Fill).boxed()
}

fn centre<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    if state.analysis == Analysis::Optimize {
        return state.optimize.centre(ctx);
    }
    if let Some(solving) = super::solve_view::centre(state, ctx) {
        return solving;
    }
    if let Some(flow) = super::flow_view::centre(state, ctx) {
        return flow;
    }
    let (Some(out), Some(built)) = (state.output(), state.built.as_ref()) else {
        return centred(state.error.as_ref().map_or_else(|| "Solving…".to_string(), |e| format!("Nothing to analyse: {e}")));
    };
    let mut notes: Vec<Element<'a, Message>> = Vec::new();
    if !built.legend.is_empty() {
        notes.push(plot::legend(&built.legend));
    }
    if state.omitted > 0 {
        notes.push(chip(Severity::Info, format!("Showing {} of {} configurations", built.legend.len(), built.legend.len() + state.omitted)).into());
    }
    let bar = |e: Element<'a, Message>| container(e).padding([SPACE_2, SPACE_3]).boxed();

    let body: Element<'a, Message> = if state.show_table {
        container(framed(table(state))).padding([0.0, SPACE_3]).height(Fill).boxed()
    } else if out.analysis == Analysis::Dispersion {
        dispersion(state, built)
    } else {
        // The kit's centre column: the zoom breadcrumb and hover hint on one row at the top, then the plots down to the strip.
        let plots: Vec<Element<'a, Message>> = vec![
            bar(plot::header(&built.group, &state.plot).map(Message::Plot).boxed()),
            container(PlotView::new(&built.group, &state.plot).element().map(Message::Plot).boxed()).width(Fill).height(Fill).boxed(),
        ];
        column(plots).height(Fill).boxed()
    };
    if notes.is_empty() {
        body
    } else {
        column![container(row(notes).spacing(SPACE_3).align_y(alignment::Vertical::Center)).padding([SPACE_2, SPACE_3]), body].height(Fill).boxed()
    }
}

fn dispersion<'a>(state: &'a State, built: &'a Built) -> Element<'a, Message> {
    let map: Element<'a, Message> = match &built.ground {
        Some(ground) => framed(GroundTrackView::new(ground, None).element().map(|_| Message::Ignore).boxed()),
        None => Space::new().boxed(),
    };
    let caption = |s: &'static str| -> Element<'a, Message> { container(ui::label(s)).padding([SPACE_2, SPACE_3]).boxed() };
    column![
        caption("LANDING SCATTER · 95 % ELLIPSE · PAD AT CENTRE"),
        container(map).padding([0.0, SPACE_3]).height(Length::FillPortion(3)),
        caption("APOGEE SPREAD"),
        container(plot::header(&built.group, &state.plot).map(Message::Plot)).padding([SPACE_2, SPACE_3]),
        container(PlotView::new(&built.group, &state.plot).element().map(Message::Plot)).width(Fill).height(Length::FillPortion(2)),
    ]
    .height(Fill)
    .boxed()
}

fn table(state: &State) -> Element<'_, Message> {
    let Some(table) = &state.table else { return Space::new().boxed() };
    let cell = |s: String, header: bool, first: bool| -> Element<'_, Message> {
        let t = text(s).font(MONO).size(if header { TEXT_LABEL } else { 12.0 }).width(if first { Length::Fixed(84.0) } else { Length::FillPortion(1) });
        let t = if first { t } else { t.align_x(alignment::Horizontal::Right) };
        (if header { t.style(theme::faint) } else { t.style(theme::ink) }).boxed()
    };
    let head = container(row(table.headers.iter().enumerate().map(|(i, h)| cell(h.clone(), true, i == 0)).collect::<Vec<_>>()).spacing(SPACE_2))
        .padding([SPACE_2, SPACE_3]);
    let rows = column(
        table
            .rows
            .iter()
            .map(|r| row(r.iter().enumerate().map(|(i, c)| cell(c.clone(), false, i == 0)).collect::<Vec<_>>()).spacing(SPACE_2).boxed())
            .collect::<Vec<_>>(),
    )
    .spacing(2)
    .padding(iced::Padding { top: 0.0, right: SPACE_3 + 8.0, bottom: 0.0, left: SPACE_3 });
    column![head, rule(), scrollable(rows).height(Fill)].boxed()
}

// ---------------------------------------------------------------------------------------------------------------------
// Right
// ---------------------------------------------------------------------------------------------------------------------

/// The right panel of the Optimize analysis: the best design against the current one, the goal and the constraints.
fn optimize_right<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let head = container(text("Best design").size(15).font(SANS_SEMIBOLD)).padding([SPACE_3, SPACE_3]).width(Fill);
    let mut sections: Vec<Element<'a, Message>> = vec![head.boxed(), rule()];
    for (i, sec) in state.optimize.readout(ctx).into_iter().enumerate() {
        let rows: Vec<Element<'a, Message>> = sec.rows.into_iter().map(row_view).collect();
        sections.push(section(i + 1, &sec.title, i > 0, rows));
    }
    scrollable(column(sections)).width(Fill).height(Fill).boxed()
}

fn right<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    if state.analysis == Analysis::Optimize {
        return optimize_right(state, ctx);
    }
    let active = ctx.doc.rocket().active_config_id();
    let shown = state.output().zip(state.built.as_ref());
    let hover = shown.and_then(|(_, b)| state.plot.hover_x().map(|x| b.scale.to_si(x)));
    let readout = shown.map(|(out, _)| readout::readout(out, hover, active, ctx.units));

    let cursor = readout.as_ref().map_or(String::new(), |r| r.cursor.clone());
    let head = container(
        row![text("At cursor").size(15).font(SANS_SEMIBOLD), Space::new().width(Fill), text(cursor).font(MONO).size(12).style(theme::colored(|t| t.accent))]
            .align_y(iced::Center),
    )
    .padding([SPACE_3, SPACE_3]);

    let mut sections: Vec<Element<'a, Message>> = vec![head.boxed(), rule()];
    let mut count = 0;
    if let Some(readout) = readout {
        count = readout.sections.len();
        for (i, sec) in readout.sections.into_iter().enumerate() {
            let rows: Vec<Element<'a, Message>> = sec.rows.into_iter().map(row_view).collect();
            sections.push(section(i + 1, &sec.title, i > 0, rows));
        }
    }
    if let (Some((out, _)), Some(overlay)) = (shown, state.overlay.as_ref()) {
        if let Some(sec) = readout::compare_section(out, overlay, hover, active, ctx.units) {
            let rows: Vec<Element<'a, Message>> = sec.rows.into_iter().map(row_view).collect();
            sections.push(section(count + 1, &sec.title, true, rows));
            count += 1;
        }
    }
    sections.push(export(state, shown.is_some(), count + 1));
    scrollable(column(sections)).width(Fill).height(Fill).boxed()
}

fn swatch<'a>(color: SeriesColor) -> Element<'a, Message> {
    container(Space::new())
        .width(10)
        .height(10)
        .style(move |theme: &Theme| container::Style { background: Some(color.resolve(theme::tokens(theme)).into()), ..container::Style::default() })
        .boxed()
}

/// A tooltip in the kit's style: a raised surface, 1 px border, 11 px text wrapped at about 320 px.
fn help_bubble<'a>(tip: &'static str) -> Element<'a, Message> {
    container(text(tip).size(TEXT_LABEL)).width(320).padding([SPACE_1, SPACE_2]).style(|t: &Theme| theme::tooltip(t)).boxed()
}

fn row_view<'a>(r: Row) -> Element<'a, Message> {
    let mut parts: Vec<Element<'a, Message>> = Vec::new();
    if let Some(color) = r.swatch {
        parts.push(swatch(color));
    }
    parts.push(text(r.label).size(TEXT_BODY).style(if r.muted { theme::faint } else { theme::muted }).boxed());
    parts.push(Space::new().width(Fill).boxed());
    match r.tag {
        Some(Tag::Chip(severity, label)) => parts.push(chip(severity, label).into()),
        Some(Tag::Plain(label)) => parts.push(text(label).font(MONO).size(TEXT_LABEL).style(theme::muted).boxed()),
        None => {}
    }
    if let Some(share) = r.share {
        parts.push(text(share).font(MONO).size(TEXT_LABEL).style(theme::faint).boxed());
    }
    let value = text(r.value).font(MONO).style(if r.muted { theme::muted } else { theme::ink });
    parts.push(if r.big { value.size(20).boxed() } else { value.size(12.5).boxed() });
    if !r.unit.is_empty() {
        parts.push(text(r.unit).font(MONO).size(TEXT_LABEL).style(theme::muted).boxed());
    }
    let line = container(row(parts).spacing(SPACE_2).align_y(iced::Center)).height(if r.big { 30.0 } else { 26.0 }).width(Fill).boxed();
    match r.help {
        Some(tip) => tooltip(line, help_bubble(tip)).position(tooltip::Position::Bottom).gap(SPACE_1).boxed(),
        None => line,
    }
}

fn export<'a>(state: &'a State, ready: bool, index: usize) -> Element<'a, Message> {
    let mut table = push(if state.show_table { "Plots" } else { "Table view" }, Weight::Secondary);
    let mut csv = push("CSV", Weight::Secondary);
    table = if ready { table.on_press(Message::Table(!state.show_table)) } else { table.disabled("Nothing computed yet") };
    csv = if ready { csv.on_press(Message::ExportCsv) } else { csv.disabled("Nothing computed yet") };
    let cdx = push("RASAero .CDX1", Weight::Secondary).disabled(RASAERO_REASON);
    let (table, csv, cdx): (Element<'a, Message>, Element<'a, Message>, Element<'a, Message>) = (table.into(), csv.into(), cdx.into());
    let flow: Element<'a, Message> = flow_button(state).into();
    section(index, "Export", true, vec![row![table, flow].spacing(SPACE_2).boxed(), row![csv, cdx].spacing(SPACE_2).boxed()])
}

/// The `Flow field` button beside `Table view`: the flow fields of the CFD table solved in this session, or why there are none.
fn flow_button(state: &State) -> crate::widgets::button::Button<'_, Message> {
    let open = state.flow.open;
    let b = push(if open { "Plots" } else { "Flow field" }, Weight::Secondary);
    if open {
        return b.on_press(Message::Flow(super::flow_view::Msg::Open(false)));
    }
    let cfd = state.method == Method::CfdTable && super::uses_cfd(state.analysis);
    let table = state.cfd.table.as_ref().filter(|_| state.cfd_ready());
    match table {
        _ if !cfd => b.disabled("Choose the CFD table method: the flow fields are what the solver left of each point"),
        _ if state.cfd.solving.is_some() => b.disabled("A solve is running: its flow is on show"),
        None => b.disabled("Solve the CFD table first"),
        Some(t) if !state.cfd.fields.has_any(t) => {
            b.disabled("No flow fields for this table: it was loaded from a file or its geometry changed. Re-solve to see them")
        }
        Some(_) => b.on_press(Message::Flow(super::flow_view::Msg::Open(true))).tooltip("The flow of each solved Mach number"),
    }
}

/// A numbered section of the right panel: tighter than the inspector's so the whole readout fits in 800 px.
fn section<'a>(index: usize, title: &str, ruled: bool, rows: Vec<Element<'a, Message>>) -> Element<'a, Message> {
    let mut body: Vec<Element<'a, Message>> = Vec::new();
    if ruled {
        body.push(rule());
    }
    body.push(container(field::section_head(index, title)).padding([SPACE_2, 0.0]).boxed());
    body.extend(rows);
    container(column(body)).padding([SPACE_1, SPACE_3]).width(Fill).boxed()
}
