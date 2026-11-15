//! The left column of the Simulate screen: the FLIGHTS list (with the compare checkboxes) and the CONDITIONS of the selected
//! flight, then Run and Run all.

use std::fmt;

use bowshock_sim::Turbulence;
use iced::widget::{button, column, container, mouse_area, row, scrollable, text, text_input, tooltip, Space};
use iced::{Background, Border, Element, Fill, Length, Theme, Widget};

use super::flight::{self, Flight, MotorChoice, Spec};
use super::{format, Edit, Message, State, MAX_COMPARED};
use crate::app::{ui, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{Tokens, SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::units::{Quantity, UnitSystem};
use crate::widgets::button::{button as push, icon_button, Weight};
use crate::widgets::chip::Severity;
use crate::widgets::field::{labeled, rule, select};
use crate::widgets::glyph::Mark;
use crate::widgets::plot::SeriesColor;
use crate::widgets::segmented::segmented_fill;
use crate::widgets::unit_input::unit_input;

/// Height of a flight row (the kit's 44 px list rows).
const ROW_H: f32 = 44.0;
/// The widget id of the rename field.
pub(super) const RENAME_ID: &str = "flight-rename";
/// Width of the accent edge of the selected row.
const EDGE_W: f32 = 2.0;

/// A choice in the motor select.
#[derive(Debug, Clone, PartialEq)]
struct MotorOption {
    choice: MotorChoice,
    label: String,
}

impl fmt::Display for MotorOption {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.label)
    }
}

/// The whole column.
pub(super) fn view<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let flight = state.selected_flight();

    let title = if state.shown.compare { format!("FLIGHTS · {} COMPARED", state.ticked()) } else { "FLIGHTS".to_string() };
    let add: Element<'a, Message> = push("compare +", Weight::Ghost).tooltip("Add a flight to compare").on_press(Message::AddFlight).into();
    let head = container(row![ui::label(title), Space::new().width(Fill), add].align_y(iced::Center)).padding([0.0, SPACE_2]).height(32);

    let revision = state.revision(ctx);
    let rows = column(state.flights.iter().map(|f| flight_row(state, ctx, f, revision)));

    let mut body: Vec<Element<'a, Message>> = vec![rows.boxed()];
    if ctx.doc.rocket().configurations().len() > 1 {
        let each: Element<'a, Message> = push("one flight per configuration", Weight::Ghost)
            .tooltip("Fly every motor configuration and compare them")
            .on_press(Message::PerConfiguration)
            .into();
        body.push(container(each).padding([SPACE_1, SPACE_2]).boxed());
    }
    if state.shown.compare {
        let hint =
            "Tick up to four flights to overlay them. The highlighted one is the reference: its events are drawn and the others are compared against it.";
        body.push(container(text(hint).size(TEXT_BODY - 1.0).style(theme::muted)).padding([SPACE_2, SPACE_3]).boxed());
    }
    body.push(rule());
    body.push(conditions(ctx, flight, state.shown.compare, state.renaming.as_ref().filter(|(id, _)| *id == flight.id).map(|(_, b)| b.as_str())));

    let run: Element<'a, Message> = push("Run", Weight::Primary).icon(Mark::Play).width(Fill).on_press(Message::Run).into();
    let run_all: Element<'a, Message> = push("Run all", Weight::Secondary).tooltip("Fly every flight").on_press(Message::RunAll).into();
    let footer = container(row![run, run_all].spacing(SPACE_2)).padding(SPACE_3);

    column![head, scrollable(column(body)).direction(super::thin_scrollbar()).style(super::scroll_style).height(Fill), rule(), footer].height(Fill).boxed()
}

/// The inline text field that renames a flight (Enter commits, Esc cancels).
fn rename_field<'a>(buffer: &'a str) -> Element<'a, Message> {
    text_input("Flight name", buffer)
        .id(RENAME_ID)
        .on_input(Message::RenameInput)
        .on_submit(Message::CommitRename)
        .size(TEXT_BODY)
        .padding([1.0, SPACE_1])
        .style(theme::input)
        .width(Fill)
        .boxed()
}

/// The CONDITIONS section of `flight`: its fields, with the `F5` keycap and a remove button in the head.
fn conditions<'a>(ctx: &'a Ctx, flight: &'a Flight, compare: bool, renaming: Option<&'a str>) -> Element<'a, Message> {
    let spec = &flight.spec;
    let system = &ctx.units;
    let field = |label: &'a str, value: f64, quantity: Quantity, make: fn(f64) -> Edit| -> Element<'a, Message> {
        unit_input(label, value, quantity, system).on_commit(move |v| Message::Edit(make(v))).on_scrub(move |v| Message::Edit(make(v))).into()
    };

    let options = motor_options(ctx, spec);
    let current = options.iter().find(|o| o.choice == spec.motor).cloned();
    let motor = labeled("Motor", select(options, current, |o: MotorOption| Message::Edit(Edit::Motor(o.choice))));
    let turbulence = labeled(
        "Turbulence",
        segmented_fill(&[(Turbulence::Off, "Off"), (Turbulence::Low, "Low"), (Turbulence::Med, "Med"), (Turbulence::High, "High")], spec.turbulence, |t| {
            Message::Edit(Edit::Turbulence(t))
        }),
    );

    let keycap = container(text("F5").font(MONO).size(10)).padding([1.0, SPACE_1]).style(theme::keycap);
    let remove: Element<'a, Message> = icon_button(Mark::Close, "Remove this flight").on_press(Message::RemoveFlight).width(22).into();
    let title: Element<'a, Message> = match renaming {
        Some(buffer) => rename_field(buffer),
        None => mouse_area(ui::label(format!("CONDITIONS · {}", flight.name.to_uppercase()))).on_double_click(Message::StartRename(flight.id)).boxed(),
    };
    let head = row![title, Space::new().width(Fill), remove, keycap].spacing(SPACE_2).align_y(iced::Center);

    // In a comparison the flights differ by motor, delay, turbulence, site and temperature: those fields are not comparable
    // across a ticked set, so only the four shared launch conditions are listed (SimulateCompare).
    let mut fields: Vec<Element<'a, Message>> = Vec::new();
    if !compare {
        fields.push(motor);
        fields.push(field("Delay", flight::effective_delay(ctx, spec), Quantity::Time, Edit::Delay));
    }
    fields.push(field("Wind", spec.wind_speed, Quantity::Velocity, Edit::Wind));
    fields.push(field("Wind from", spec.wind_from, Quantity::Angle, Edit::WindFrom));
    fields.push(field("Launch angle", spec.launch_angle, Quantity::Angle, Edit::LaunchAngle));
    if !compare {
        fields.push(field("Rail length", spec.rail_length, Quantity::Length, Edit::RailLength));
        fields.push(turbulence);
        fields.push(field("Site altitude", spec.site_altitude, Quantity::Altitude, Edit::SiteAltitude));
        fields.push(field("Temperature", spec.temperature, Quantity::Temperature, Edit::Temperature));
    }
    fields.push(field("Field radius", spec.field_radius, Quantity::Altitude, Edit::FieldRadius));
    container(column![head, column(fields).spacing(SPACE_2)].spacing(SPACE_3)).padding(SPACE_3).boxed()
}

/// The motor select's options: the active configuration, each configuration, then the motors that fit the first mount.
fn motor_options(ctx: &Ctx, spec: &Spec) -> Vec<MotorOption> {
    let rocket = ctx.doc.rocket();
    let name = |id| rocket.configuration_name(id).unwrap_or_default();
    let mut options = vec![MotorOption { choice: MotorChoice::Active, label: format!("Current · {}", name(rocket.active_config_id())) }];
    for c in rocket.configurations() {
        options.push(MotorOption { choice: MotorChoice::Config(c.id), label: format!("Config · {}", name(c.id)) });
    }
    if let Some(mount) = flight::first_mount(ctx) {
        let reach = mount.length + mount.overhang;
        let mut fitting: Vec<_> = ctx.physics.motors().fitting(mount.inner_diameter, Some(reach)).collect();
        fitting.sort_by(|a, b| a.total_impulse().total_cmp(&b.total_impulse()));
        for m in fitting {
            options.push(MotorOption {
                choice: MotorChoice::Motor(bowshock_model::MotorRef::new(m.designation.clone(), m.manufacturer.clone())),
                label: format!("{} {}", m.manufacturer, m.designation),
            });
        }
    }
    // A flight can name a motor that does not fit this mount (the kit's Bigger motor on another rocket): keep it selectable.
    if let MotorChoice::Motor(m) = &spec.motor {
        if !options.iter().any(|o| o.choice == spec.motor) {
            options.push(MotorOption { choice: spec.motor.clone(), label: format!("{} {}", m.manufacturer, m.designation) });
        }
    }
    options
}

/// The condition that tells a flight apart, for the list's second line: the temperature of a hot or cold day, else the wind.
fn key_condition(spec: &Spec, system: UnitSystem) -> String {
    if (spec.temperature - 288.15).abs() > 5.0 {
        format::with_unit(spec.temperature, Quantity::Temperature, system, 0)
    } else {
        format::with_unit(spec.wind_speed, Quantity::Velocity, system, 1)
    }
}

fn status_color(severity: Option<Severity>, stale: bool) -> fn(&Tokens) -> iced::Color {
    match (severity, stale) {
        (None, _) => |t| t.line_200,
        (Some(_), true) => |t| t.ink_faint,
        (Some(Severity::Ok), _) => |t| t.ok,
        (Some(Severity::Warn), _) => |t| t.warn,
        (Some(Severity::Err), _) => |t| t.err,
        (Some(Severity::Info), _) => |t| t.ink_muted,
    }
}

/// One row: the compare checkbox, name and `motor · condition`, apogee and the status square.
fn flight_row<'a>(state: &'a State, ctx: &'a Ctx, flight: &'a Flight, revision: u64) -> Element<'a, Message> {
    let selected = flight.id == state.selected;
    let stale = flight.is_stale(revision);
    let is_ref = state.shown.compare && state.shown.ids.get(state.shown.reference) == Some(&flight.id);

    let slot = flight.slot;
    let box_ = container(Space::new()).width(12).height(12).style(move |theme: &Theme| {
        let t = theme::tokens(theme);
        match slot {
            Some(s) => {
                let c = SeriesColor::Series(s).resolve(t);
                container::Style { background: Some(Background::Color(c)), border: theme::frame(c), ..container::Style::default() }
            }
            None => container::Style { border: theme::frame(t.ink_faint), ..container::Style::default() },
        }
    });
    let tick = button(container(box_).center_y(Fill))
        .padding([0.0, SPACE_2])
        .height(Fill)
        .style(|theme: &Theme, status| {
            let t = theme::tokens(theme);
            let hot = matches!(status, button::Status::Hovered | button::Status::Pressed);
            button::Style { background: hot.then_some(Background::Color(t.bg_400)), ..button::Style::default() }
        })
        .on_press(Message::Tick(flight.id));
    let tick: Element<'a, Message> = if slot.is_none() && state.ticked() >= MAX_COMPARED {
        let tip = container(text(format!("Compare up to {MAX_COMPARED} flights")).size(TEXT_BODY)).padding([SPACE_1, SPACE_2]).style(theme::tooltip);
        tooltip(tick, tip).position(tooltip::Position::Right).boxed()
    } else {
        tick.boxed()
    };

    let title: Element<'a, Message> = match state.renaming.as_ref().filter(|(id, _)| *id == flight.id) {
        Some((_, buffer)) => rename_field(buffer),
        None => mouse_area(text(flight.name.as_str()).size(TEXT_BODY)).on_double_click(Message::StartRename(flight.id)).boxed(),
    };
    let mut name: Vec<Element<'a, Message>> = vec![title];
    if is_ref {
        name.push(container(text("REF").font(MONO).size(9)).padding([0.0, 3.0]).style(theme::keycap).boxed());
    }
    let sub = format!("{} · {}", flight::motor_label(ctx, &flight.spec), key_condition(&flight.spec, ctx.units));
    let left = column![row(name).spacing(SPACE_2).align_y(iced::Center), text(sub).font(MONO).size(TEXT_LABEL - 1.0).style(theme::faint)].spacing(1);

    let severity = if flight.error.is_some() { Some(Severity::Err) } else { flight.run.as_ref().map(|r| r.severity()) };
    let pick = status_color(severity, stale);
    let square = container(Space::new()).width(6).height(6).style(move |theme: &Theme| {
        let c = pick(theme::tokens(theme));
        if severity.is_some() {
            container::Style { background: Some(Background::Color(c)), ..container::Style::default() }
        } else {
            container::Style { border: Border { color: c, width: 1.0, radius: 0.0.into() }, ..container::Style::default() }
        }
    });
    let apogee: Element<'a, Message> = match &flight.run {
        Some(run) => {
            let (n, u) = format::num(run.result.summary.apogee, Quantity::Altitude, ctx.units, 0);
            let style = if stale { theme::faint } else { theme::ink };
            row![text(n).font(MONO).size(13).style(style), text(u).font(MONO).size(TEXT_LABEL).style(theme::faint)]
                .spacing(SPACE_1)
                .align_y(iced::Bottom)
                .boxed()
        }
        None if flight.running => text("running…").font(MONO).size(TEXT_LABEL).style(theme::faint).boxed(),
        None => Space::new().boxed(),
    };
    // Before a run there is nothing to show at the right: no dash and no empty status square.
    let right: Element<'a, Message> = if flight.run.is_none() && !flight.running && flight.error.is_none() {
        Space::new().boxed()
    } else {
        column![apogee, square].spacing(SPACE_1).align_x(iced::Right).boxed()
    };

    let content = row![left, Space::new().width(Fill), right].align_y(iced::Center);
    let main = button(content)
        .width(Fill)
        .height(Fill)
        .padding([0.0, SPACE_3])
        .style(move |theme: &Theme, status| {
            let t = theme::tokens(theme);
            let hot = matches!(status, button::Status::Hovered | button::Status::Pressed) && !selected;
            button::Style { background: hot.then_some(Background::Color(t.bg_300.scale_alpha(0.6))), text_color: t.ink, ..button::Style::default() }
        })
        .on_press(Message::Select(flight.id));

    let edge = container(Space::new()).width(EDGE_W).height(Fill).style(move |theme: &Theme| container::Style {
        background: selected.then(|| Background::Color(theme::tokens(theme).accent)),
        ..container::Style::default()
    });
    container(row![edge, tick, main].align_y(iced::Center))
        .width(Fill)
        .height(Length::Fixed(ROW_H))
        .style(move |theme: &Theme| container::Style {
            background: selected.then(|| Background::Color(theme::tokens(theme).bg_300)),
            ..container::Style::default()
        })
        .boxed()
}
