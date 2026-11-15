//! Drawing of the Analyze screen: the left panel (analyses, method, conditions), the centre (stability gauge, linked plots,
//! landing map, table view) and the right panel (values at the cursor, export).

use std::time::Instant;

use iced::widget::{button, column, container, row, scrollable, text, tooltip, Space};
use iced::{alignment, Element, Fill, Length, Theme, Widget};

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
use crate::widgets::gauge;
use crate::widgets::ground_track::GroundTrackView;
use crate::widgets::plot::{self, PlotView, SeriesColor};
use crate::widgets::unit_input::unit_input;

/// Width of the label column of the Conditions rows (wider than the inspector's: "Angle of attack" must fit).
const LABEL_W: f32 = 112.0;
/// Height of an analyses list row.
const ROW_H: f32 = 36.0;
/// Width of the gauge card.
const GAUGE_W: f32 = 270.0;

static FINISHES: [FinishChoice; 6] = FinishChoice::ALL;

const RASAERO_REASON: &str = "RASAero's .CDX1 is a proprietary project format; exporting it is not implemented";
const NO_COMPARISON: &str = "No comparison is loaded";

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
    let body = column![
        head("ANALYSES", false),
        analyses(state),
        head("METHOD", false),
        container(method(state)).padding([0.0, SPACE_3]),
        head("CONDITIONS", true),
        container(conditions(state, ctx)).padding([0.0, SPACE_3]),
        Space::new().height(SPACE_3),
    ]
    .spacing(SPACE_1);
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

fn method(state: &State) -> Element<'_, Message> {
    let mut cells: Vec<Element<'_, Message>> = Vec::new();
    for (i, m) in [Method::Barrowman, Method::Extended, Method::CfdTable].into_iter().enumerate() {
        if i > 0 {
            cells.push(container(Space::new()).width(1).height(CONTROL_H - 2.0).style(theme::hairline).boxed());
        }
        let active = m == state.method;
        let enabled = m != Method::CfdTable;
        let mut b = button(text(m.name()).size(TEXT_BODY)).height(CONTROL_H - 4.0).padding([0.0, SPACE_3]).style(move |theme: &Theme, status| {
            let mut s = theme::segment(active)(theme, status);
            if !enabled {
                s.text_color = theme::tokens(theme).ink_faint;
            }
            s
        });
        if enabled {
            b = b.on_press(Message::Method(m));
        }
        let cell = column![b, container(Space::new()).width(Fill).height(2).style(theme::underline(active))];
        cells.push(tooltip(cell, bubble(m.tip())).position(tooltip::Position::Bottom).gap(SPACE_1).boxed());
    }
    let strip = container(row(cells)).padding(1).style(|theme: &Theme| theme::segmented_frame(theme));

    let line: Element<'_, Message> = match (state.output(), &state.error) {
        (_, Some(e)) => muted(format!("Cannot analyse: {e}"), 12.0),
        (None, None) => muted("Solving…", 12.0),
        (Some(out), None) => {
            let busy = if state.pending { "solving…".to_string() } else { state.computed_at.map_or_else(String::new, ago) };
            let took =
                if out.elapsed.as_millis() < 1000 { format!("{} ms", out.elapsed.as_millis().max(1)) } else { format!("{:.1} s", out.elapsed.as_secs_f64()) };
            muted(format!("{} · computed on CPU in {took} · {busy}", out.points_text()), 12.0)
        }
    };
    let resolve: Element<'_, Message> = push("Re-solve", Weight::Secondary).on_press(Message::Resolve).into();
    let compare: Element<'_, Message> = push("Compare ▾", Weight::Ghost).on_press(Message::CompareMenu).into();
    let actions = row![resolve, compare].spacing(SPACE_2);
    let mut col = column![strip, line, actions].spacing(SPACE_2).padding([0.0, 0.0]);
    if state.compare_open {
        let other = super::other_method(state.method);
        let with = match other {
            Method::Barrowman => "Compare with Barrowman",
            _ => "Compare with Extended",
        };
        let item = |label: &'static str, message: Message| -> Element<'_, Message> { push(label, Weight::Secondary).on_press(message).small().into() };
        let clear: Element<'_, Message> = if state.compare.is_some() {
            item("Clear comparison", Message::ClearCompare)
        } else {
            push("Clear comparison", Weight::Secondary).small().disabled(NO_COMPARISON).into()
        };
        col = col.push(
            column![
                item(with, Message::CompareWith(other)),
                item("Import OpenRocket CSV…", Message::ImportCompare(Source::OpenRocket)),
                item("Import RASAero CSV…", Message::ImportCompare(Source::RasAero)),
                clear,
            ]
            .spacing(SPACE_1)
            .boxed(),
        );
    }
    if let Some(c) = &state.compare {
        let from = match c {
            Comparison::Method(_) => String::new(),
            Comparison::File { name, .. } => format!(" · {name}"),
        };
        col = col.push(muted(format!("Comparing with {}{from}", c.label()), 12.0));
    }
    col.boxed()
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
            rows.push(if state.method == Method::Barrowman { alpha.hint("Barrowman ignores the angle (α = 0)").into() } else { alpha.into() });
            rows.push(input("Altitude", c.altitude, Quantity::Altitude, Message::Altitude));
            rows.push(burnout);
            rows.push(finish);
        }
        Analysis::Stability => {
            rows.push(row_of("Mach", mach_input(state, MachField::Fixed)));
            rows.push(input("Largest angle", c.alpha_max, Quantity::Angle, Message::AlphaMax));
            rows.push(burnout);
            rows.push(finish);
        }
        Analysis::Flutter => {
            rows.push(muted("Flutter velocity (NACA TN 4197) from each fin's geometry and its material's shear modulus, against the maximum velocity of the configuration's flight.", 12.0));
        }
        Analysis::Dispersion => {
            rows.push(row_of("Runs", crate::widgets::segmented::segmented(&[(50, "50"), (100, "100"), (200, "200")], c.runs, Message::Runs)));
            rows.push(input("Mean wind", c.wind, Quantity::Velocity, Message::Wind));
            rows.push(muted("One-sigma spreads: wind 1 m/s and 10°, rail tilt 1°, thrust 3 %, drag 5 %, dry mass 2 %.", 12.0));
        }
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
    let (Some(out), Some(built)) = (state.output(), state.built.as_ref()) else {
        return centred(state.error.as_ref().map_or_else(|| "Solving…".to_string(), |e| format!("Nothing to analyse: {e}")));
    };
    let active = ctx.doc.rocket().active_config_id();
    let hover = state.plot.hover_x().map(|x| built.scale.to_si(x));
    let length = readout::margin_at(out, hover, active).map(|(_, m)| ui::with_unit(m, Quantity::Length, ctx.units));
    let bands: gauge::Bands = ctx.doc.rocket().stability_bands().into();
    let card = container(gauge::full(&state.gauge, &bands, length)).width(GAUGE_W);

    let mut notes: Vec<Element<'a, Message>> = Vec::new();
    if !built.legend.is_empty() {
        notes.push(plot::legend(&built.legend));
    }
    if state.omitted > 0 {
        notes.push(chip(Severity::Info, format!("Showing {} of {} configurations", built.legend.len(), built.legend.len() + state.omitted)).into());
    }
    let top =
        container(row![card, Space::new().width(Fill), column(notes).spacing(SPACE_2).align_x(alignment::Horizontal::Right)].align_y(alignment::Vertical::Top))
            .padding([SPACE_3, SPACE_3]);
    let bar = |e: Element<'a, Message>| container(e).padding([SPACE_2, SPACE_3]).boxed();

    let body: Element<'a, Message> = if state.show_table {
        container(framed(table(state))).padding([0.0, SPACE_3]).height(Fill).boxed()
    } else if out.analysis == Analysis::Dispersion {
        dispersion(state, built)
    } else {
        column![
            bar(plot::header(&built.group, &state.plot).map(Message::Plot).boxed()),
            container(PlotView::new(&built.group, &state.plot).element().map(Message::Plot).boxed()).width(Fill).height(Fill)
        ]
        .height(Fill)
        .boxed()
    };
    column![top, body].boxed()
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

fn right<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
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

fn row_view<'a>(r: Row) -> Element<'a, Message> {
    let mut parts: Vec<Element<'a, Message>> = Vec::new();
    if let Some(color) = r.swatch {
        parts.push(swatch(color));
    }
    parts.push(text(r.label).size(TEXT_BODY).style(theme::muted).boxed());
    parts.push(Space::new().width(Fill).boxed());
    match r.tag {
        Some(Tag::Chip(severity, label)) => parts.push(chip(severity, label).into()),
        Some(Tag::Plain(label)) => parts.push(text(label).font(MONO).size(TEXT_LABEL).style(theme::muted).boxed()),
        None => {}
    }
    let value = text(r.value).font(MONO).style(theme::ink);
    parts.push(if r.big { value.size(20).boxed() } else { value.size(12.5).boxed() });
    if !r.unit.is_empty() {
        parts.push(text(r.unit).font(MONO).size(TEXT_LABEL).style(theme::muted).boxed());
    }
    container(row(parts).spacing(SPACE_2).align_y(iced::Center)).height(if r.big { 30.0 } else { 26.0 }).boxed()
}

fn export<'a>(state: &'a State, ready: bool, index: usize) -> Element<'a, Message> {
    let mut table = push(if state.show_table { "Plots" } else { "Table view" }, Weight::Secondary);
    let mut csv = push("CSV", Weight::Secondary);
    table = if ready { table.on_press(Message::Table(!state.show_table)) } else { table.disabled("Nothing computed yet") };
    csv = if ready { csv.on_press(Message::ExportCsv) } else { csv.disabled("Nothing computed yet") };
    let cdx = push("RASAero .CDX1", Weight::Secondary).disabled(RASAERO_REASON);
    let (table, csv, cdx): (Element<'a, Message>, Element<'a, Message>, Element<'a, Message>) = (table.into(), csv.into(), cdx.into());
    section(index, "Export", true, vec![row![table, csv, cdx].spacing(SPACE_2).boxed()])
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
