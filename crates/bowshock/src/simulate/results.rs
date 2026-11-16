//! The right column of the Simulate screen: the results of the selected flight (01 APOGEE, 02 ASCENT, 03 RECOVERY) or, with
//! several flights ticked, the comparison against the reference (SimulateCompare), and the ground track (04). Every number is a
//! button that moves the plots' crosshair to the moment it happened.

use bowshock_sim::roll::MIN_SPIN_RATE;
use iced::widget::{column, container, mouse_area, row, scrollable, text, Space};
use iced::{font, mouse, Element, Fill, Theme, Widget};

use super::flight::{required_margin_cal, Flight, Run, CFD_3D_TAG, DELAY_TOLERANCE, MAX_GROUND_HIT, MIN_OFF_RAIL_CAL, MIN_RAIL_EXIT};
use super::{format, Message, State};
use crate::app::Ctx;
use crate::theme::{self, MONO, SANS_SEMIBOLD};
use crate::tokens::{text as scale, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::units::{self, Quantity};
use crate::widgets::button::{button as push, Weight};
use crate::widgets::chip::{chip, Severity};
use crate::widgets::field::{rule, section, section_head};
use crate::widgets::ground_track::GroundTrackView;
use crate::widgets::plot::SeriesColor;

/// Height of a result row.
const ROW_H: f32 = 22.0;
/// Width of the value column and of the unit column.
const VALUE_W: f32 = 52.0;
const UNIT_W: f32 = 30.0;
/// Height of the ground track (a compact one: rings every 50 m).
const GROUND_H: f32 = 232.0;

/// A chip as an element.
fn badge<'a>(severity: Severity, label: impl Into<String>) -> Element<'a, Message> {
    chip(severity, label).into()
}

/// The whole column.
pub(super) fn view<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let flight = state.selected_flight();
    let mut parts: Vec<Element<'a, Message>> = Vec::new();

    if state.shown.compare {
        parts.extend(compare(state, ctx));
    } else {
        parts.extend(single(ctx, flight, state.stamp(ctx, &flight.spec)));
    }

    parts.push(rule());
    parts.push(ground_section(state, ctx, flight));
    scrollable(column(parts).width(Fill)).direction(super::thin_scrollbar()).style(super::scroll_style).height(Fill).boxed()
}

/// 04 GROUND TRACK: the map, the run-dispersion button and, once a study has flown, its caption.
fn ground_section<'a>(state: &'a State, ctx: &'a Ctx, flight: &'a Flight) -> Element<'a, Message> {
    // The study needs a result to vary: the button appears with the first run (one style, not a dashed stand-in), and the
    // numbering follows the sections above (01–03 once there is a result, otherwise this is the first section).
    let index = if state.shown.compare || flight.run.is_some() { 4 } else { 1 };
    let mut head: Vec<Element<'a, Message>> = vec![section_head(index, "Ground track"), Space::new().width(Fill).boxed()];
    if flight.run.is_some() {
        let mut disperse = push("Run dispersion", Weight::Ghost).tooltip("Fly this flight many times with wind, launch angle and thrust varied");
        disperse = if flight.dispersing { disperse.disabled("Dispersion is running") } else { disperse.on_press(Message::Disperse) };
        head.push(disperse.into());
    }
    let head = row(head).align_y(iced::Center);

    let ground = GroundTrackView::new(&state.shown.ground, state.plot.hover_x()).element().map(Message::Ground);
    let mut body: Vec<Element<'a, Message>> = vec![head.boxed(), container(ground).width(Fill).height(GROUND_H).boxed()];
    if let Some(caption) = dispersion_caption(state, ctx, flight) {
        body.push(caption);
    }
    container(column(body).spacing(SPACE_2)).padding(SPACE_3).boxed()
}

/// `Windy · 100 runs`, `95 % land in 281 × 132 m` and whether they stay in the field, for the selected flight's current study.
fn dispersion_caption<'a>(state: &'a State, ctx: &'a Ctx, flight: &'a Flight) -> Option<Element<'a, Message>> {
    let study = flight.dispersion_of_run()?;
    let track = state.shown.ids.iter().position(|id| *id == flight.id).and_then(|i| state.shown.ground.tracks.get(i))?;
    let ellipse = track.ellipse?;
    let size = |semi: f64| units::num_scaled(2.0 * semi, Quantity::Altitude, ctx.units, 0).0;
    let unit = Quantity::Altitude.unit_for(ctx.units).symbol;
    let (n, outside) = (study.landings.len(), study.outside(study.spec.field_radius));
    let verdict =
        if outside == 0 { badge(Severity::Ok, "all in field") } else { badge(Severity::Warn, format!("{:.0} % outside", 100.0 * outside as f64 / n as f64)) };
    let lines = column![
        text(format!("{} · {n} runs", flight.name)).size(TEXT_BODY).style(theme::muted),
        text(format!("95 % land in {} × {} {unit}", size(ellipse.semi_major), size(ellipse.semi_minor))).font(MONO).size(TEXT_LABEL).style(theme::faint),
    ]
    .spacing(2);
    Some(row![lines, Space::new().width(Fill), verdict].align_y(iced::Center).boxed())
}

fn header<'a>(title: impl text::IntoFragment<'a>, aside: Element<'a, Message>) -> Element<'a, Message> {
    container(row![text(title).font(SANS_SEMIBOLD).size(scale::UI_TITLE.size), Space::new().width(Fill), aside].align_y(iced::Center))
        .padding([SPACE_3, SPACE_3])
        .boxed()
}

fn aside<'a>(s: String) -> Element<'a, Message> {
    text(s).font(MONO).size(TEXT_LABEL).style(theme::faint).boxed()
}

/// A result row: `label … [chip] value unit`. With `seek` the whole row moves the plots' crosshair to that time.
fn line<'a>(
    label: impl text::IntoFragment<'a>,
    chip: Option<Element<'a, Message>>,
    value: String,
    unit: &'a str,
    stale: bool,
    seek: Option<f64>,
) -> Element<'a, Message> {
    let mut cells: Vec<Element<'a, Message>> = vec![text(label).size(TEXT_BODY).style(theme::muted).width(Fill).wrapping(text::Wrapping::None).boxed()];
    if let Some(chip) = chip {
        cells.push(chip);
    }
    // A value that is a phrase (`104 m · 068°`) takes the room it needs instead of the number column.
    let phrase = unit.is_empty() && value.chars().count() > 6;
    let value = text(value).font(MONO).size(scale::NUM_BODY.size).style(if stale { theme::faint } else { theme::ink });
    if phrase {
        cells.push(container(value).width(VALUE_W + SPACE_2 + UNIT_W).align_right(VALUE_W + SPACE_2 + UNIT_W).boxed());
    } else {
        cells.push(container(value).width(VALUE_W).align_right(VALUE_W).boxed());
        cells.push(container(text(unit).font(MONO).size(TEXT_LABEL).style(theme::faint)).width(UNIT_W).boxed());
    }
    pointer(row(cells).spacing(SPACE_2).align_y(iced::Center).height(ROW_H).boxed(), seek)
}

fn pointer<'a>(element: Element<'a, Message>, seek: Option<f64>) -> Element<'a, Message> {
    match seek {
        Some(t) => mouse_area(element).on_press(Message::Seek(t)).interaction(mouse::Interaction::Pointer).boxed(),
        None => element,
    }
}

/// The apogee as the headline: the big readout.
fn headline<'a>(value: String, unit: &'a str, stale: bool, seek: f64) -> Element<'a, Message> {
    let big = text(value).font(MONO.weight(font::Weight::Semibold)).size(scale::NUM_READOUT.size).style(if stale { theme::faint } else { theme::ink });
    let cells = row![
        text("Apogee").size(TEXT_BODY).style(theme::muted).width(Fill),
        container(big).width(VALUE_W).align_right(VALUE_W),
        container(text(unit).font(MONO).size(TEXT_LABEL).style(theme::faint)).width(UNIT_W),
    ]
    .spacing(SPACE_2)
    .align_y(iced::Center)
    .height(28);
    pointer(cells.boxed(), Some(seek))
}

/// What the screen says when the selected flight has no result: the reason or the way to get one.
fn waiting<'a>(flight: &'a Flight) -> Element<'a, Message> {
    let body: Element<'a, Message> = if flight.running {
        text("Running…").size(TEXT_BODY).style(theme::muted).boxed()
    } else if let Some(error) = &flight.error {
        badge(Severity::Err, error.clone())
    } else {
        text("Press Run (F5) to see this flight's apogee, ascent and recovery.").size(TEXT_BODY).style(theme::muted).boxed()
    };
    container(body).padding(SPACE_3).boxed()
}

fn single<'a>(ctx: &'a Ctx, flight: &'a Flight, revision: u64) -> Vec<Element<'a, Message>> {
    let info = match (&flight.run, flight.running) {
        (_, true) => aside("running…".to_string()),
        (Some(run), _) => aside(format!("run {:.2} s · RK4 · {} ms", run.took.as_secs_f64(), trim(run.dt * 1000.0))),
        _ => aside(String::new()),
    };
    let mut parts = vec![header(flight.name.as_str(), info), rule()];
    let Some(run) = &flight.run else {
        parts.push(waiting(flight));
        return parts;
    };
    parts.extend(sections(run, ctx, flight.is_stale(revision)));
    if let Some(error) = &flight.error {
        parts.push(container(badge(Severity::Err, error.clone())).padding([0.0, SPACE_3]).boxed());
    }
    parts
}

/// A millisecond count without a trailing `.0`.
fn trim(ms: f64) -> String {
    if ms.fract().abs() < 1e-9 {
        format!("{ms:.0}")
    } else {
        format!("{ms}")
    }
}

/// 01 APOGEE, 02 ASCENT, 03 RECOVERY of one run.
fn sections<'a>(run: &'a Run, ctx: &'a Ctx, stale: bool) -> Vec<Element<'a, Message>> {
    let s = &run.result.summary;
    let t = &run.times;
    let sys = ctx.units;
    let vel = |v: f64, decimals: usize| units::num_scaled(v, Quantity::Velocity, sys, decimals);

    // 01 APOGEE
    let (alt, alt_unit) = units::num_scaled(s.apogee, Quantity::Altitude, sys, 0);
    let off = run.delay - s.optimum_delay;
    let delay_chip = if !run.delay.is_finite() {
        None
    } else if off > DELAY_TOLERANCE {
        Some(badge(Severity::Warn, format!("{off:.1} s late")))
    } else if off < -DELAY_TOLERANCE {
        Some(badge(Severity::Warn, format!("{:.1} s early", -off)))
    } else {
        None
    };
    let apogee = section(
        1,
        "Apogee",
        false,
        vec![
            headline(alt, alt_unit, stale, t.apogee),
            line("Time to apogee", None, format!("{:.1}", s.time_to_apogee), "s", stale, Some(t.apogee)),
            line("Optimum delay", delay_chip, format!("{:.1}", s.optimum_delay), "s", stale, Some(t.apogee)),
        ],
    );

    // 02 ASCENT
    let (max_v, v_unit) = vel(s.max_velocity, 0);
    let (rail, rail_unit) = vel(s.rail_exit_velocity, 1);
    let rail_ok = s.rail_exit_velocity >= MIN_RAIL_EXIT;
    let rail_chip = badge(if rail_ok { Severity::Ok } else { Severity::Warn }, format!("{} {}", if rail_ok { "≥" } else { "<" }, vel(MIN_RAIL_EXIT, 0).0));
    let stable = s.stability_off_rail_cal >= MIN_OFF_RAIL_CAL;
    let stab_chip = badge(if stable { Severity::Ok } else { Severity::Err }, format!("{} {MIN_OFF_RAIL_CAL:.1}", if stable { "≥" } else { "<" }));
    // The tag replaces "Stability" so the label still fits beside its chip.
    let off_rail = if s.stability_from_lift { format!("Off rail {CFD_3D_TAG}") } else { "Stability off rail".to_string() };
    let mut ascent_rows = vec![
        line("Max velocity", None, max_v, v_unit, stale, Some(t.max_speed)),
        line("Max Mach", None, format!("{:.2}", s.max_mach), "", stale, Some(t.max_mach)),
        line("Max accel", None, format!("{:.1}", s.max_acceleration_g), "g", stale, Some(t.max_accel)),
        line("Rail exit", Some(rail_chip), rail, rail_unit, stale, Some(t.rail_exit)),
        line(off_rail, Some(stab_chip), format!("{:.2}", s.stability_off_rail_cal), "cal", stale, Some(t.rail_exit)),
    ];
    // The least margin between rail exit and apogee, against the guidance at the Mach number where it occurs.
    if let Some(min) = &s.min_margin {
        let need = required_margin_cal(min.mach);
        let enough = min.cal >= need;
        let severity = if enough {
            Severity::Ok
        } else if min.cal < 0.0 {
            Severity::Err
        } else {
            Severity::Warn
        };
        let chip = badge(severity, format!("{} {need:.1}", if enough { "≥" } else { "<" }));
        ascent_rows.push(line("Min margin in flight", Some(chip), format!("{:.2}", min.cal), "cal", stale, Some(min.time)));
        // The tag goes on the row beside it: the first has no room for it next to its chip.
        let at_mach = if s.stability_from_lift { format!("at Mach {CFD_3D_TAG}") } else { "at Mach".to_string() };
        ascent_rows.push(line(at_mach, None, format!("{:.2}", min.mach), "", stale, Some(min.time)));
    }
    // Canted fins: how fast the rocket spins, and whether the spin meets its pitch frequency on the way.
    if let Some(spin) = Spin::of(s) {
        let peak = badge(Severity::Info, format!("{:.0} rpm {}", spin.max_rpm, spin.sense));
        ascent_rows.push(line("Max spin", Some(peak), format!("{:.1}", spin.max_hz), "Hz", stale, Some(t.max_roll)));
        ascent_rows.push(line("Spin at burnout", None, format!("{:.1}", spin.burnout_hz), "Hz", stale, Some(t.burnout)));
        ascent_rows.push(line("Spin at apogee", None, format!("{:.1}", spin.apogee_hz), "Hz", stale, Some(t.apogee)));
        if let Some(r) = &spin.resonance {
            let chip = badge(Severity::Warn, r.chip.clone());
            ascent_rows.push(line(r.label, Some(chip), format!("{:.1}", r.time), "s", stale, Some(r.time)));
        }
    }
    let ascent = section(2, "Ascent", true, ascent_rows);

    // 03 RECOVERY
    let mut rows: Vec<Element<'a, Message>> = Vec::new();
    for (name, rate) in &s.descent_rates {
        let (n, u) = vel(*rate, 1);
        rows.push(line(format!("{name} descent"), None, n, u, stale, t.deploy));
    }
    let (hit, hit_unit) = vel(s.ground_hit_velocity, 1);
    let hard = s.ground_hit_velocity > MAX_GROUND_HIT;
    let hit_chip = badge(if hard { Severity::Warn } else { Severity::Ok }, format!("{} {}", if hard { ">" } else { "<" }, vel(MAX_GROUND_HIT, 1).0));
    rows.push(line("Ground hit", Some(hit_chip), hit, hit_unit, stale, Some(t.landing)));
    rows.push(line("Flight time", None, format!("{:.1}", s.flight_time), "s", stale, Some(t.landing)));
    let lands = badge(if run.in_field() { Severity::Ok } else { Severity::Warn }, if run.in_field() { "in field" } else { "outside field" });
    rows.push(line("Lands", Some(lands), format::landing(s.landing.0, s.landing.1, sys), "", stale, Some(t.landing)));
    if let Some(at) = run.spec.location {
        let (lat, lon) = landing_coordinates(at, s.landing_position);
        rows.push(line("Landing at", None, format!("{lat:.5}°, {lon:.5}°").replace('-', "−"), "", stale, Some(t.landing)));
    }
    let recovery = section(3, "Recovery", true, rows);

    vec![apogee, ascent, recovery]
}

/// The latitude and longitude (degrees) of the point `[east, north]` metres from the pad at `at`, on a sphere of the Earth's mean
/// radius (6 371 km): small offsets, so the local flat-Earth conversion holds.
fn landing_coordinates(at: bowshock_model::Location, offset: [f64; 2]) -> (f64, f64) {
    const R: f64 = 6_371_000.0;
    let lat = at.latitude + (offset[1] / R).to_degrees();
    let lon = at.longitude + (offset[0] / (R * at.latitude.to_radians().cos().max(1e-6))).to_degrees();
    (lat, (lon + 180.0).rem_euclid(360.0) - 180.0)
}

/// The spin of one run as the results column reads it: the fastest turn, the turn at burnout and at apogee (turns per second), and
/// where the spin rate meets the pitch frequency (see [`bowshock_sim::roll`]). Present only for a run that spins.
#[derive(Debug, Clone, PartialEq)]
struct Spin {
    /// The largest roll rate, rev/s and rpm.
    max_hz: f64,
    max_rpm: f64,
    /// Which way it turns seen from behind (`CW` or `CCW`).
    sense: &'static str,
    /// The roll rate at the last burnout and at apogee, rev/s.
    burnout_hz: f64,
    apogee_hz: f64,
    /// Where it meets the pitch frequency.
    resonance: Option<SpinResonance>,
}

/// A roll–pitch resonance as a results row: the label, the chip text and the time.
#[derive(Debug, Clone, PartialEq)]
struct SpinResonance {
    label: &'static str,
    chip: String,
    time: f64,
}

impl Spin {
    /// The spin of a run, `None` when it hardly turns (no canted fins, or a point-mass flight).
    fn of(s: &bowshock_sim::Summary) -> Option<Spin> {
        if s.max_roll_rate.abs() < MIN_SPIN_RATE {
            return None;
        }
        let resonance = s.roll_resonance.map(|r| {
            let pitch = r.pitch_frequency / std::f64::consts::TAU;
            if r.crossed {
                SpinResonance { label: "Spin crosses pitch", chip: format!("{pitch:.1} Hz"), time: r.time }
            } else {
                SpinResonance {
                    label: "Spin near pitch",
                    chip: format!("{:.0} % off {pitch:.1} Hz", 100.0 * (r.roll_rate / r.pitch_frequency - 1.0).abs()),
                    time: r.time,
                }
            }
        });
        Some(Spin {
            max_hz: format::rev_per_s(s.max_roll_rate),
            max_rpm: format::rpm(s.max_roll_rate),
            sense: format::spin_sense(s.max_roll_rate),
            burnout_hz: format::rev_per_s(s.roll_rate_at_burnout),
            apogee_hz: format::rev_per_s(s.roll_rate_at_apogee),
            resonance,
        })
    }
}

/// A key result of the comparison table: its title, quantity, decimals, the number in SI and when it happened.
struct Table {
    title: &'static str,
    quantity: Quantity,
    decimals: usize,
    value: fn(&Run) -> f64,
    time: fn(&Run) -> f64,
}

/// The comparison: one row per flight with its colour square, value and difference from the reference.
fn compare<'a>(state: &'a State, ctx: &'a Ctx) -> Vec<Element<'a, Message>> {
    let sys = ctx.units;
    let flights: Vec<&Flight> = state.shown.ids.iter().filter_map(|id| state.flight(*id)).collect();
    let Some(reference) = flights.get(state.shown.reference).copied() else { return vec![waiting(state.selected_flight())] };
    let ref_run = reference.run.as_ref();

    let mut parts = vec![header("Compare", aside(format!("Δ against {}", reference.name))), rule()];

    let tables = [
        Table { title: "Apogee", quantity: Quantity::Altitude, decimals: 0, value: |r| r.result.summary.apogee, time: |r| r.times.apogee },
        Table { title: "Max velocity", quantity: Quantity::Velocity, decimals: 0, value: |r| r.result.summary.max_velocity, time: |r| r.times.max_speed },
    ];
    for (i, table) in tables.iter().enumerate() {
        let unit = table.quantity.unit_for(sys);
        let rows = flights
            .iter()
            .filter_map(|f| {
                let run = f.run.as_ref()?;
                let (n, u) = units::num_scaled((table.value)(run), table.quantity, sys, table.decimals);
                let diff = match ref_run {
                    _ if f.id == reference.id => "ref".to_string(),
                    Some(r) => units::signed(unit.from_si((table.value)(run)) - unit.from_si((table.value)(r)), table.decimals),
                    None => String::new(),
                };
                Some(compare_row(f, n, diff, u, f.is_stale(state.stamp(ctx, &f.spec)), Some((table.time)(run))))
            })
            .collect();
        parts.push(section(i + 1, table.title, i > 0, rows));
    }

    let landing = flights
        .iter()
        .filter_map(|f| {
            let run = f.run.as_ref()?;
            let stale = f.is_stale(state.stamp(ctx, &f.spec));
            let verdict = badge(if run.in_field() { Severity::Ok } else { Severity::Warn }, if run.in_field() { "in" } else { "out" });
            let value = text(format::landing(run.result.summary.landing.0, run.result.summary.landing.1, sys))
                .font(MONO)
                .size(scale::NUM_BODY.size)
                .style(if stale { theme::faint } else { theme::ink });
            let cells = row![swatch(f), text(f.name.as_str()).size(TEXT_BODY).width(Fill).wrapping(text::Wrapping::None), value, verdict]
                .spacing(SPACE_2)
                .align_y(iced::Center)
                .height(ROW_H);
            Some(pointer(cells.boxed(), Some(run.times.landing)))
        })
        .collect();
    parts.push(section(3, "Landing", true, landing));
    parts
}

/// The flight's colour square.
fn swatch<'a>(flight: &Flight) -> Element<'a, Message> {
    let slot = flight.slot.unwrap_or(1);
    container(Space::new())
        .width(8)
        .height(8)
        .style(move |theme: &Theme| container::Style {
            background: Some(SeriesColor::Series(slot).resolve(theme::tokens(theme)).into()),
            ..container::Style::default()
        })
        .boxed()
}

fn compare_row<'a>(flight: &'a Flight, value: String, diff: String, unit: &'a str, stale: bool, seek: Option<f64>) -> Element<'a, Message> {
    let value = text(value).font(MONO).size(scale::NUM_BODY.size).style(if stale { theme::faint } else { theme::ink });
    let cells = row![
        swatch(flight),
        text(flight.name.as_str()).size(TEXT_BODY).width(Fill).wrapping(text::Wrapping::None),
        container(value).width(VALUE_W).align_right(VALUE_W),
        container(text(diff).font(MONO).size(scale::NUM_BODY.size).style(theme::muted)).width(VALUE_W).align_right(VALUE_W),
        container(text(unit).font(MONO).size(TEXT_LABEL).style(theme::faint)).width(UNIT_W),
    ]
    .spacing(SPACE_2)
    .align_y(iced::Center)
    .height(ROW_H);
    pointer(cells.boxed(), seek)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_sim::{RollResonance, Summary};

    #[test]
    fn a_run_that_does_not_spin_has_no_spin_rows() {
        assert_eq!(Spin::of(&Summary::default()), None);
        // A drift of a hundredth of a turn per second is not a spin.
        assert_eq!(Spin::of(&Summary { max_roll_rate: 0.01, ..Summary::default() }), None);
    }

    #[test]
    fn the_spin_reads_in_rev_per_s_and_rpm_with_its_sense_and_the_resonance_it_meets() {
        let tau = std::f64::consts::TAU;
        let summary = Summary {
            max_roll_rate: 20.0 * tau,
            roll_rate_at_burnout: 19.5 * tau,
            roll_rate_at_apogee: 4.0 * tau,
            roll_resonance: Some(RollResonance { time: 0.62, roll_rate: 2.4 * tau, pitch_frequency: 2.4 * tau, crossed: true }),
            ..Summary::default()
        };
        let spin = Spin::of(&summary).expect("spins");
        assert!((spin.max_hz - 20.0).abs() < 1e-9 && (spin.max_rpm - 1200.0).abs() < 1e-9 && spin.sense == "CW");
        assert!((spin.burnout_hz - 19.5).abs() < 1e-9 && (spin.apogee_hz - 4.0).abs() < 1e-9);
        assert_eq!(spin.resonance, Some(SpinResonance { label: "Spin crosses pitch", chip: "2.4 Hz".into(), time: 0.62 }));
        // The other way, and only coming near the pitch frequency.
        let back = Summary {
            max_roll_rate: -3.0 * tau,
            roll_resonance: Some(RollResonance { time: 3.0, roll_rate: 2.7 * tau, pitch_frequency: 3.0 * tau, crossed: false }),
            ..Summary::default()
        };
        let spin = Spin::of(&back).expect("spins");
        assert_eq!((spin.sense, spin.max_hz.round() as i32), ("CCW", 3));
        assert_eq!(spin.resonance.map(|r| (r.label, r.chip)), Some(("Spin near pitch", "10 % off 3.0 Hz".to_string())));
    }
}
