//! The left column of the Simulate screen: the FLIGHTS list (with the compare checkboxes) and the CONDITIONS of the flights
//! selected for editing (one, or several at once), then Run and Run all.

use std::fmt;

use bowshock_model::DragSource;
use bowshock_sim::Turbulence;
use iced::widget::{button, column, container, mouse_area, row, scrollable, text, text_input, tooltip, Space};
use iced::{Background, Border, Element, Fill, Length, Theme, Widget};

use super::flight::{self, Flight, MotorChoice, Shared, Spec};
use super::sites::{self, SITES};
use super::{keys, Edit, Message, State, MAX_COMPARED};
use crate::app::{ui, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{Tokens, SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::units::{self, Quantity, UnitSystem};
use crate::widgets::button::{button as push, icon_button, Weight};
use crate::widgets::chip::Severity;
use crate::widgets::field::{checkbox, labeled, rule, select};
use crate::widgets::glyph::Mark;
use crate::widgets::plot::SeriesColor;
use crate::widgets::segmented::segmented_fill;
use crate::widgets::spinner::spinner;
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
    let title = if state.shown.compare { format!("FLIGHTS · {} COMPARED", state.ticked()) } else { "FLIGHTS".to_string() };
    let add: Element<'a, Message> = push("compare +", Weight::Ghost).tooltip("Add a flight to compare").on_press(Message::AddFlight).into();
    let head = container(row![ui::label(title), Space::new().width(Fill), add].align_y(iced::Center)).padding([0.0, SPACE_2]).height(32);

    let rows = column(state.flights.iter().map(|f| flight_row(state, ctx, f)));

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
    body.push(conditions(state, ctx));

    let multi = state.selection.is_multi();
    let run: Element<'a, Message> = push(if multi { "Run selected" } else { "Run" }, Weight::Primary)
        .icon(Mark::Play)
        .tooltip(if multi { "Fly the selected flights" } else { "Fly this flight" })
        .width(Fill)
        .on_press(Message::Run)
        .into();
    let run_all: Element<'a, Message> = push("Run all", Weight::Secondary).tooltip("Fly every flight").on_press(Message::RunAll).into();
    let footer = container(row![run, run_all].spacing(SPACE_2)).padding(SPACE_3);

    // The tracker is invisible: it only keeps `state.modifiers` current for the row clicks.
    let held = keys::track(state.modifiers.unwrap_or(ctx.env.drive_modifiers), Message::Modifiers);
    column![held, head, scrollable(column(body)).direction(super::thin_scrollbar()).style(super::scroll_style).height(Fill), rule(), footer]
        .height(Fill)
        .boxed()
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

/// The CONDITIONS section of the flights selected for editing: every field, with the `F5` keycap and a remove button in the head.
/// With one flight the fields are its own. With several, a field they agree on shows the shared value and one they differ on the
/// mixed `—` until it is edited; an edit sets the field on all of them (the untouched fields stay per-flight).
fn conditions<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let flights = state.edit_flights();
    let count = flights.len();
    let flight = state.selected_flight();
    let spec = &flight.spec;
    let shared = Shared::of(ctx, &flights);
    let system = &ctx.units;
    // A field shows the value the flights share; when they differ it is mixed (and the selected flight's value stands behind it).
    let field = |label: &'a str, shown: Option<f64>, fallback: f64, quantity: Quantity, make: fn(f64) -> Edit| -> Element<'a, Message> {
        unit_input(label, shown.unwrap_or(fallback), quantity, system)
            .mixed(shown.is_none())
            .on_commit(move |v| Message::Edit(make(v)))
            .on_scrub(move |v| Message::Edit(make(v)))
            .into()
    };

    let specs: Vec<_> = flights.iter().map(|f| &f.spec).collect();
    let options = motor_options(ctx, &specs);
    let current = shared.motor.as_ref().and_then(|motor| options.iter().find(|o| o.choice == *motor)).cloned();
    let motor = labeled("Motor", select(options, current, |o: MotorOption| Message::Edit(Edit::Motor(o.choice))));
    let gusts = [Turbulence::Off, Turbulence::Low, Turbulence::Med, Turbulence::High].map(Some);
    let turbulence = labeled(
        "Turbulence",
        segmented_fill(&[(gusts[0], "Off"), (gusts[1], "Low"), (gusts[2], "Med"), (gusts[3], "High")], shared.turbulence, |t| {
            Message::Edit(Edit::Turbulence(t.unwrap_or_default()))
        }),
    );

    let drag = labeled(
        "Drag",
        segmented_fill(&[(DragSource::Extended, "Extended"), (DragSource::CfdTable, "CFD table")], spec.drag, |d| Message::Edit(Edit::Drag(d))),
    );
    let drag_note = flight::drag_note(ctx, spec);

    let keycap = container(text("F5").font(MONO).size(10)).padding([1.0, SPACE_1]).style(theme::keycap);
    let remove: Element<'a, Message> =
        icon_button(Mark::Close, if count > 1 { "Remove the selected flights" } else { "Remove this flight" }).on_press(Message::RemoveFlight).width(22).into();
    let renaming = state.renaming.as_ref().filter(|(id, _)| *id == flight.id).map(|(_, b)| b.as_str());
    let title: Element<'a, Message> = match (count, renaming) {
        (1, Some(buffer)) => rename_field(buffer),
        (1, None) => mouse_area(ui::label(format!("CONDITIONS · {}", flight.name.to_uppercase()))).on_double_click(Message::StartRename(flight.id)).boxed(),
        _ => ui::label(format!("CONDITIONS · {count} FLIGHTS")),
    };
    let head = row![title, Space::new().width(Fill), remove, keycap].spacing(SPACE_2).align_y(iced::Center);

    let mut fields: Vec<Element<'a, Message>> = Vec::new();
    if count > 1 {
        let note = format!("— flights differ · an edit sets all {count}");
        fields.push(text(note).size(TEXT_BODY - 1.0).style(theme::muted).boxed());
    }
    fields.push(site_row(&shared));
    fields.push(motor);
    fields.push(field("Delay", shared.delay, flight::effective_delay(ctx, spec), Quantity::Time, Edit::Delay));
    fields.push(field("Wind", shared.wind_speed, spec.wind_speed, Quantity::Velocity, Edit::Wind));
    fields.push(field("Wind from", shared.wind_from, spec.wind_from, Quantity::Angle, Edit::WindFrom));
    fields.extend(wind_profile(state, spec, &shared, count, system));
    fields.push(field("Launch angle", shared.launch_angle, spec.launch_angle, Quantity::Angle, Edit::LaunchAngle));
    fields.extend(start_rows(&shared, spec, system));
    if spec.air_start.is_none() {
        fields.push(field("Rail length", shared.rail_length, spec.rail_length, Quantity::Length, Edit::RailLength));
    }
    fields.push(turbulence);
    fields.push(drag);
    if let Some(note) = drag_note {
        fields.push(text(note).size(TEXT_LABEL).boxed());
    }
    fields.push(field("Site altitude", shared.site_altitude, spec.site_altitude, Quantity::Altitude, Edit::SiteAltitude));
    fields.push(coord_field(state, false, &shared));
    fields.push(coord_field(state, true, &shared));
    fields.push(field("Temperature", shared.temperature, spec.temperature, Quantity::Temperature, Edit::Temperature));
    fields.push(field("Field radius", shared.field_radius, spec.field_radius, Quantity::Altitude, Edit::FieldRadius));
    container(column![head, column(fields).spacing(SPACE_2)].spacing(SPACE_3)).padding(SPACE_3).boxed()
}

/// Where an air start begins when it is switched on: 1 km up at 100 m/s.
const AIR_START: bowshock_model::AirStart = bowshock_model::AirStart { altitude: 1000.0, speed: 100.0 };

/// `Start  Pad | Air start`, and while the selected flight starts in the air its altitude and speed. An edit of either sets the
/// pair on every selected flight.
fn start_rows<'a>(shared: &Shared, spec: &Spec, system: &'a UnitSystem) -> Vec<Element<'a, Message>> {
    let current = shared.air_start.map(|a| a.is_some());
    let toggle = segmented_fill(&[(Some(false), "Pad"), (Some(true), "Air start")], current, |on| {
        Message::Edit(Edit::AirStart((on == Some(true)).then_some(AIR_START)))
    });
    let mut rows = vec![labeled("Start", toggle)];
    if let Some(start) = spec.air_start {
        let mixed = shared.air_start.is_none();
        let altitude = unit_input("Start altitude", start.altitude, Quantity::Altitude, system)
            .mixed(mixed)
            .on_commit(move |altitude| Message::Edit(Edit::AirStart(Some(bowshock_model::AirStart { altitude, ..start }))));
        let speed = unit_input("Start speed", start.speed, Quantity::Velocity, system)
            .mixed(mixed)
            .on_commit(move |speed| Message::Edit(Edit::AirStart(Some(bowshock_model::AirStart { speed, ..start }))));
        rows.push(altitude.into());
        rows.push(speed.into());
    }
    rows
}

/// A choice of the Site select: no built-in site, or one of [`SITES`].
#[derive(Debug, Clone, Copy, PartialEq)]
enum SiteChoice {
    Custom,
    Site(usize),
}

impl fmt::Display for SiteChoice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SiteChoice::Custom => f.write_str("Custom"),
            SiteChoice::Site(i) => f.write_str(SITES[*i].name),
        }
    }
}

/// The Site select: the built-in site the flights are at (`Custom` for any other place, `—` when they differ). Picking a site
/// sets the site altitude and location (see [`Edit::Site`]).
fn site_row<'a>(shared: &Shared) -> Element<'a, Message> {
    let current = match (shared.location, shared.site_altitude) {
        (Some(location), Some(altitude)) => {
            Some(sites::matching(location, altitude).and_then(|s| SITES.iter().position(|x| x == s)).map_or(SiteChoice::Custom, SiteChoice::Site))
        }
        _ => None,
    };
    let options: Vec<SiteChoice> = std::iter::once(SiteChoice::Custom).chain((0..SITES.len()).map(SiteChoice::Site)).collect();
    labeled(
        "Site",
        select(options, current, |c| match c {
            SiteChoice::Custom => Message::Ignore,
            SiteChoice::Site(i) => Message::Edit(Edit::Site(i)),
        }),
    )
}

/// The latitude (`lon` false) or longitude field, in degrees: blank is no location (a flat Earth); Enter applies what is typed.
fn coord_field<'a>(state: &'a State, lon: bool, shared: &Shared) -> Element<'a, Message> {
    let placeholder = if shared.location.is_none() { "—" } else { "Set to include Earth rotation" };
    let input = text_input(placeholder, &state.coord_text[usize::from(lon)])
        .on_input(move |t| Message::CoordInput(lon, t))
        .on_submit(Message::CoordCommit)
        .size(TEXT_BODY)
        .padding([4.0, SPACE_2])
        .style(theme::input)
        .width(Fill);
    labeled(if lon { "Longitude °" } else { "Latitude °" }, input.boxed())
}

/// Heights of the 1/7 power-law profile (m).
const POWER_LAW_HEIGHTS: [f64; 7] = [50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0];

/// The wind of the 1/7 power law (Hellman's exponent for open, neutral-stability terrain) at `h` metres, for `ground` m/s
/// measured at the 10 m reference height.
fn power_law(ground: f64, h: f64) -> f64 {
    ground * (h / 10.0).powf(1.0 / 7.0)
}

/// The Wind profile row (`Constant` or `N levels`, which opens the table) and, while open, the table of levels: altitude, speed
/// and direction, each row removable, then Add level, Power law and Clear. With several flights the table is the selected
/// flight's, and an edit sets every one of them.
fn wind_profile<'a>(state: &'a State, spec: &'a Spec, shared: &Shared, count: usize, system: &'a UnitSystem) -> Vec<Element<'a, Message>> {
    let levels = &spec.wind_levels;
    let summary = match levels.len() {
        0 => "Constant".to_string(),
        1 => "1 level".to_string(),
        n => format!("{n} levels"),
    };
    let open = state.wind_open;
    let toggle: Element<'a, Message> =
        push(if open { "Hide" } else { "Edit" }, Weight::Ghost).tooltip("The wind at altitudes above the ground").on_press(Message::WindTable(!open)).into();
    let shown = row![text(summary).size(TEXT_BODY), Space::new().width(Fill), toggle].align_y(iced::Center).boxed();
    let mut out = vec![labeled("Wind profile", shown)];
    if !open {
        return out;
    }
    if count > 1 && shared.wind_levels.is_none() {
        out.push(text(format!("— flights differ · an edit sets all {count}")).size(TEXT_BODY - 1.0).style(theme::muted).boxed());
    }
    // One line a level (`1000 m · 4.2 m/s · 250°`); clicking it opens its three fields under it, full width.
    for (i, l) in levels.iter().enumerate() {
        let selected = state.wind_level == Some(i);
        let summary = format!(
            "{} · {} · {}",
            units::with_unit_scaled(l.altitude, Quantity::Altitude, *system, 0),
            units::with_unit_scaled(l.speed, Quantity::Velocity, *system, 1),
            units::with_unit_scaled(l.from, Quantity::Angle, *system, 0)
        );
        let line = button(text(summary).font(MONO).size(TEXT_BODY - 1.0))
            .width(Fill)
            .padding([2.0, SPACE_1])
            .style(move |theme: &Theme, status| {
                let t = theme::tokens(theme);
                let hot = matches!(status, button::Status::Hovered | button::Status::Pressed);
                button::Style {
                    background: (selected || hot).then_some(Background::Color(if selected { t.bg_300 } else { t.bg_400 })),
                    text_color: t.ink,
                    ..button::Style::default()
                }
            })
            .on_press(Message::WindLevel((!selected).then_some(i)));
        let mut without = levels.clone();
        without.remove(i);
        let remove: Element<'a, Message> = icon_button(Mark::Close, "Remove this level").on_press(Message::Edit(Edit::WindLevels(without))).width(22).into();
        out.push(row![line, remove].spacing(SPACE_1).align_y(iced::Center).boxed());
        if selected {
            let cell = |label: &'a str, value: f64, quantity: Quantity, set: fn(&mut bowshock_model::WindLevel, f64)| -> Element<'a, Message> {
                let base = levels.clone();
                unit_input(label, value, quantity, system)
                    .on_commit(move |v| {
                        let mut next = base.clone();
                        set(&mut next[i], v);
                        Message::Edit(Edit::WindLevels(next))
                    })
                    .into()
            };
            out.push(
                container(
                    column![
                        cell("Altitude", l.altitude, Quantity::Altitude, |l, v| l.altitude = v),
                        cell("Speed", l.speed, Quantity::Velocity, |l, v| l.speed = v),
                        cell("From", l.from, Quantity::Angle, |l, v| l.from = v),
                    ]
                    .spacing(SPACE_1),
                )
                .padding(iced::Padding { top: 0.0, right: 0.0, bottom: SPACE_1, left: SPACE_2 })
                .boxed(),
            );
        }
    }
    let (top_altitude, top_speed, top_from) = levels.last().map_or((0.0, spec.wind_speed, spec.wind_from), |l| (l.altitude, l.speed, l.from));
    let mut added = levels.clone();
    added.push(bowshock_model::WindLevel { altitude: top_altitude + 500.0, speed: top_speed, from: top_from });
    let law: Vec<bowshock_model::WindLevel> =
        POWER_LAW_HEIGHTS.iter().map(|&h| bowshock_model::WindLevel { altitude: h, speed: power_law(spec.wind_speed, h), from: spec.wind_from }).collect();
    let add: Element<'a, Message> = push("Add level", Weight::Ghost).on_press(Message::Edit(Edit::WindLevels(added))).into();
    let power: Element<'a, Message> = push("Power law", Weight::Ghost)
        .tooltip("Levels from 50 m to 5 km by the 1/7 power law of the ground wind")
        .on_press(Message::Edit(Edit::WindLevels(law)))
        .into();
    let mut clear = push("Clear", Weight::Ghost);
    if !levels.is_empty() {
        clear = clear.on_press(Message::Edit(Edit::WindLevels(Vec::new())));
    }
    let clear: Element<'a, Message> = clear.into();
    out.push(row![add, power, clear].spacing(SPACE_1).boxed());
    out.extend(forecast_rows(state));
    out.push(text("Weather data by Open-Meteo.com").size(TEXT_LABEL).style(theme::faint).boxed());
    out
}

/// `Fetch forecast…`, or while it is open the forecast row: the launch hour (site local time), whether the forecast also sets the
/// temperature, Fetch (a spinner while it runs) and why the last fetch failed.
fn forecast_rows<'a>(state: &'a State) -> Vec<Element<'a, Message>> {
    let Some(row) = &state.forecast else {
        let open: Element<'a, Message> = push("Fetch forecast…", Weight::Ghost)
            .tooltip("Ground and upper winds forecast at the launch location")
            .on_press(Message::ForecastOpen(true))
            .into();
        return vec![open];
    };
    let (date, hour) = picker_parts(&row.time);
    let mut fetch = push("Fetch", Weight::Secondary);
    if !row.busy {
        fetch = fetch.on_press(Message::ForecastFetch);
    }
    let fetch: Element<'a, Message> = fetch.into();
    let close: Element<'a, Message> = icon_button(Mark::Close, "Close the forecast").on_press(Message::ForecastOpen(false)).width(22).into();
    let busy: Element<'a, Message> = if row.busy { spinner(14.0) } else { Space::new().width(14).boxed() };
    let mut out = vec![
        labeled("Launch date", date),
        labeled("Launch hour", hour),
        checkbox("Use forecast temperature", row.temperature, Message::ForecastTemperature),
        row![fetch, busy, Space::new().width(Fill), close].spacing(SPACE_2).align_y(iced::Center).boxed(),
    ];
    if let Some(e) = &row.error {
        out.push(text(e.as_str()).size(TEXT_LABEL).style(theme::colored(|t| t.warn)).boxed());
    }
    out
}

/// An hour of the day in the launch-hour select (`14:00`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct HourChoice(u32);

impl fmt::Display for HourChoice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:02}:00", self.0)
    }
}

/// Days ahead of today the forecast covers (Open-Meteo's forecast reaches 16 days).
const FORECAST_DAYS: i64 = 15;

/// The date stepper (`‹  Mon 16 Nov 2026  ›`, within today and the forecast's last day) and the hour select of the forecast
/// launch time `time` (`YYYY-MM-DDTHH:00`, site local); each sends the whole new time as [`Message::ForecastTime`].
fn picker_parts<'a>(time: &str) -> (Element<'a, Message>, Element<'a, Message>) {
    use chrono::{Duration, NaiveDateTime, Timelike};
    let at = NaiveDateTime::parse_from_str(time, "%Y-%m-%dT%H:%M")
        .unwrap_or_else(|_| NaiveDateTime::parse_from_str(&super::forecast::next_local_hour(), "%Y-%m-%dT%H:%M").unwrap_or_default());
    let format = |t: NaiveDateTime| t.format("%Y-%m-%dT%H:00").to_string();
    let today = chrono::Local::now().date_naive();
    let step = |days: i64, label: &'static str, tip: &'static str| -> Element<'a, Message> {
        let target = at + Duration::days(days);
        let within = (0..=FORECAST_DAYS).contains(&(target.date() - today).num_days());
        let mut b = push(label, Weight::Ghost).tooltip(tip);
        if within {
            b = b.on_press(Message::ForecastTime(format(target)));
        }
        b.into()
    };
    let date = row![
        step(-1, "‹", "The day before"),
        text(at.format("%a %-d %b").to_string()).size(TEXT_BODY).width(Fill).align_x(iced::alignment::Horizontal::Center),
        step(1, "›", "The day after"),
    ]
    .align_y(iced::Center)
    .boxed();
    let hours: Vec<HourChoice> = (0..24).map(HourChoice).collect();
    let day = at.date();
    let hour = select(hours, Some(HourChoice(at.hour())), move |h: HourChoice| Message::ForecastTime(format(day.and_hms_opt(h.0, 0, 0).unwrap_or_default())));
    (date, hour)
}

/// The motor select's options: the active configuration, each configuration, then the motors that fit the first mount, then any
/// motor a flight in `specs` names that does not.
fn motor_options(ctx: &Ctx, specs: &[&Spec]) -> Vec<MotorOption> {
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
        for m in &fitting {
            // Two curves can share a designation (a user's file beside the catalogue's): tell them apart by impulse.
            let shared = fitting.iter().filter(|o| o.designation == m.designation && o.manufacturer == m.manufacturer).count() > 1;
            let label = if shared {
                format!("{} {} · {:.0} N·s", m.manufacturer, m.designation, m.total_impulse())
            } else {
                format!("{} {}", m.manufacturer, m.designation)
            };
            options.push(MotorOption { choice: MotorChoice::Motor(bowshock_sim::from_model::motor_ref(m)), label });
        }
    }
    // A flight can name a motor that does not fit this mount (the kit's Bigger motor on another rocket): keep it selectable.
    for spec in specs {
        let MotorChoice::Motor(m) = &spec.motor else { continue };
        // A reference saved without an id (an imported flight) is the option of the curve it resolves to.
        let resolved = bowshock_sim::from_model::find_motor(ctx.physics.motors(), m).map(|motor| motor.id.as_str());
        let same = |o: &MotorOption| o.choice == spec.motor || matches!(&o.choice, MotorChoice::Motor(r) if m.id.is_empty() && Some(r.id.as_str()) == resolved);
        match options.iter_mut().find(|o| same(o)) {
            Some(option) => option.choice = spec.motor.clone(),
            None => options.push(MotorOption { choice: spec.motor.clone(), label: format!("{} {}", m.manufacturer, m.designation) }),
        }
    }
    options
}

/// The condition that tells a flight apart, for the list's second line: the temperature of a hot or cold day, else the wind.
fn key_condition(spec: &Spec, system: UnitSystem) -> String {
    if (spec.temperature - 288.15).abs() > 5.0 {
        units::with_unit_scaled(spec.temperature, Quantity::Temperature, system, 0)
    } else {
        units::with_unit_scaled(spec.wind_speed, Quantity::Velocity, system, 1)
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
fn flight_row<'a>(state: &'a State, ctx: &'a Ctx, flight: &'a Flight) -> Element<'a, Message> {
    let selected = state.is_selected(flight.id);
    let stale = flight.is_stale(state.stamp(ctx, &flight.spec));
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
            let (n, u) = units::num_scaled(run.result.summary.apogee, Quantity::Altitude, ctx.units, 0);
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
        .on_press(Message::Click(flight.id));

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
