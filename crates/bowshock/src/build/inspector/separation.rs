//! The separation inspector (design/components/Staging): where the rocket separates (read-only, found from the motor mounts),
//! when it separates and when the next stage lights (both stored with the active configuration, so `PER CONFIG`), and the
//! sequence of events, which is the staging timeline of `bowshock-sim`: the same rules the flight uses.

use std::cell::RefCell;

use bowshock_model::{Configuration, EventKind, PartId, Rocket, Stage};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{vehicle, Phase};
use bowshock_sim::{has_altitude_trigger, staging_times, Launch, SimInput, Timeline, Vehicle};
use iced::widget::{column, container, row, text, Space};
use iced::{Element, Fill, Widget};

use super::edit::{Field, StageOn};
use super::form::{choose, text_row, Form};
use super::Message;
use crate::app::ui;
use crate::app::Ctx;
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_2, SPACE_3, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::chip::{chip, Severity};
use crate::widgets::field::{labeled, rule, section_head};

/// A coast longer than this between separation and the next ignition gets a warning (seconds).
pub const LONG_COAST: f64 = 3.0;

/// What a row of the sequence is about, which sets the colour of its square.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Subject {
    /// A motor lights or burns out (`sys-propulsion`).
    Motor,
    /// The separation (`ink`).
    Separation,
    /// The event being edited here: the next stage's ignition (`accent`).
    Edited,
}

/// One row of the sequence.
#[derive(Clone, Debug, PartialEq)]
pub struct SeqRow {
    /// Seconds from launch; `None` when the stage has no motor in this configuration (or an unknown one).
    pub t: Option<f64>,
    /// The colour of the row's square.
    pub subject: Subject,
    /// What happens.
    pub text: String,
}

/// The name a stage goes by: booster, sustainer for the top stage of two, else `Stage k`.
pub fn stage_name(index: usize, total: usize) -> String {
    match (index, total) {
        (1, t) if t > 1 => "Booster".to_string(),
        (2, 2) => "Sustainer".to_string(),
        (i, _) => format!("Stage {i}"),
    }
}

/// The staging timeline of the whole stack in the active configuration (index `k` is stage `k + 1`) and which stages have a
/// motor, from the simulation's own rules (see `bowshock_sim::staging`). `None` when the rocket cannot be simulated (a motor
/// that is not in the database, no parts). Altitude triggers need a flight, so the timeline is remembered for as long as the
/// vehicle is the same.
fn staging(rocket: &Rocket, motors: &MotorDb) -> Option<(Timeline, Vec<bool>)> {
    thread_local! {
        static CACHE: RefCell<Option<(Vehicle, Timeline)>> = const { RefCell::new(None) };
    }
    let v = vehicle(rocket, motors, Phase::Full).ok()?;
    let has_motor = v.stages.iter().map(|s| s.motor.is_some()).collect();
    if !has_altitude_trigger(&v) {
        return Some((Timeline::of(&v), has_motor));
    }
    let times = CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        match cache.as_ref() {
            Some((cached, times)) if *cached == v => times.clone(),
            _ => {
                let times = staging_times(&SimInput::new(v.clone(), Launch::default(), Vec::new()));
                *cache = Some((v, times.clone()));
                times
            }
        }
    });
    Some((times, has_motor))
}

/// The sequence of the separation of stage `n` from the stage above it: launch, the lower stage's ignition (when it is not at
/// launch) and burnout, the separation, the upper stage's ignition and burnout, in time order. A time is `None` when it is
/// unknown or never comes (no motor, a trigger that never fires).
pub fn sequence(rocket: &Rocket, motors: &MotorDb, stages: &[Stage], n: usize) -> Vec<SeqRow> {
    let total = stages.len();
    if n == 0 || n >= total {
        return Vec::new();
    }
    let (lower, upper) = (n - 1, n);
    let (times, has_motor) = staging(rocket, motors).unwrap_or_else(|| (Timeline::default(), Vec::new()));
    let motor_time = |at: &[Option<f64>], k: usize| if has_motor.get(k).copied().unwrap_or(false) { at.get(k).copied().flatten() } else { None };
    let (lower_name, upper_name) = (stage_name(n, total), stage_name(n + 1, total));
    let lit_at_launch = motor_time(&times.ignition, lower).is_none_or(|t| t.abs() < 1e-9);
    let mut rows = vec![SeqRow {
        t: Some(0.0),
        subject: Subject::Motor,
        text: if lit_at_launch { format!("Launch, {} ignites", lower_name.to_lowercase()) } else { "Launch".into() },
    }];
    if !lit_at_launch {
        rows.push(SeqRow { t: motor_time(&times.ignition, lower), subject: Subject::Motor, text: format!("{lower_name} ignites") });
    }
    let separation = times.separation.get(lower).copied().flatten();
    // A stage let go while its motor still burns never burns out (the flight does not report it either).
    let burnout = motor_time(&times.burnout, lower).filter(|b| !separation.is_some_and(|s| s < b - 1e-9));
    rows.push(SeqRow { t: burnout, subject: Subject::Motor, text: format!("{lower_name} burnout") });
    rows.push(SeqRow { t: separation, subject: Subject::Separation, text: "Separation".into() });
    rows.push(SeqRow { t: motor_time(&times.ignition, upper), subject: Subject::Edited, text: format!("Stage {} ignites", n + 1) });
    rows.push(SeqRow { t: motor_time(&times.burnout, upper), subject: Subject::Motor, text: format!("{upper_name} burnout") });
    // Events come in time order (unknown times last), whatever their triggers.
    rows.sort_by(|a, b| a.t.unwrap_or(f64::INFINITY).total_cmp(&b.t.unwrap_or(f64::INFINITY)));
    rows
}

/// The coast between the separation and the next stage's ignition, when it is long enough to warn about.
pub fn long_coast(rows: &[SeqRow]) -> Option<f64> {
    let separation = rows.iter().find(|r| r.subject == Subject::Separation)?.t?;
    let ignition = rows.iter().find(|r| r.subject == Subject::Edited)?.t?;
    let coast = ignition - separation;
    (coast > LONG_COAST).then_some(coast)
}

pub(super) fn badge<'a>(label: &'static str) -> Element<'a, Message> {
    container(text(label).font(MONO).size(9.5).style(theme::muted)).padding([1.0, 4.0]).style(theme::keycap).boxed()
}

fn head<'a>(index: usize, title: &str, per_config: bool) -> Element<'a, Message> {
    let mut content = row![section_head(index, title)].spacing(SPACE_2).align_y(iced::Center);
    if per_config {
        content = content.push(badge("PER CONFIG"));
    }
    container(content).padding([SPACE_2, 0.0]).boxed()
}

fn block<'a>(index: usize, title: &str, per_config: bool, ruled: bool, rows: Vec<Element<'a, Message>>) -> Element<'a, Message> {
    let mut body: Vec<Element<'a, Message>> = Vec::new();
    if ruled {
        body.push(rule());
    }
    body.push(head(index, title, per_config));
    body.extend(rows);
    container(column(body).spacing(SPACE_2)).padding([SPACE_3, SPACE_3]).width(Fill).boxed()
}

/// The title and the station tag of the header: `Separation · stage 1 / 2` and `870.0`.
pub fn title(n: usize, total: usize) -> String {
    format!("Separation · stage {n} / {total}")
}

/// The rows of one staging event: what it waits for, the altitude when that is the trigger, and the delay after it. `lower` is
/// the name of the stage whose burnout the "burnout" choice means. With `badged` the numbers carry `PER CONFIG` themselves
/// (outside a section that has the badge in its head).
pub(super) fn timing_rows<'a>(form: &Form<'a>, config: &Configuration, stage: usize, kind: EventKind, lower: &str, badged: bool) -> Vec<Element<'a, Message>> {
    let event = config.event(stage, kind);
    let on = StageOn::of(event.trigger);
    let mut options: Vec<(StageOn, String)> = Vec::new();
    if kind == EventKind::Ignition {
        options.push((StageOn::Separation, "Separation".to_string()));
    }
    options.push((StageOn::Burnout, format!("{lower} burnout")));
    options.push((StageOn::Launch, "Time from launch".to_string()));
    options.push((StageOn::Altitude, "Altitude".to_string()));
    let number = |label: &'static str, value: f64, quantity: Quantity, field: Field| {
        if badged {
            form.per_config(label, value, quantity, field)
        } else {
            form.num(label, value, quantity, field)
        }
    };
    let label = match kind {
        EventKind::Separation => "Separates on",
        EventKind::Ignition => "Ignites on",
    };
    let mut rows = vec![labeled(label, choose(options, &on, move |on| Message::StageOn(stage, kind, on)))];
    if let bowshock_model::StageTrigger::Altitude(h) = event.trigger {
        rows.push(number("Altitude", h, Quantity::Altitude, Field::StageAltitude(stage, kind)));
    }
    rows.push(number("Delay", event.delay, Quantity::Time, Field::StageDelay(stage, kind)));
    if kind == EventKind::Ignition {
        // The motor does not light if the rocket leans past this when the ignition comes; blank is off.
        let toggle = crate::widgets::field::checkbox("Tilt lockout", event.tilt_lockout.is_some(), move |on| Message::TiltLockout(stage, on));
        rows.push(labeled("Lockout", toggle));
        if let Some(limit) = event.tilt_lockout {
            rows.push(number("Max tilt", limit, Quantity::Angle, Field::TiltLockout(stage)));
        }
    }
    rows
}

/// The sections of the separation of stage `n`, or `None` when the document has no such separation.
pub fn view<'a>(ctx: &'a Ctx, n: usize) -> Option<(String, String, Vec<Element<'a, Message>>)> {
    let rocket = ctx.doc.rocket();
    let stages = ctx.physics.stages();
    let separation = stages.iter().find(|s| s.index == n).and_then(|s| s.separation)?;
    let upper = stages.iter().find(|s| s.index == n + 1);
    let total = stages.len();
    let form = Form { ctx, limit: None, first: None };
    let config = rocket.active_configuration();

    let found_from = {
        let name = config.slot(separation.found_from).and_then(|s| s.motor.as_ref()).map(|m| m.designation.clone());
        format!("Aft end of {}", name.unwrap_or_else(|| "the upper motor mount".to_string()))
    };
    let joint = {
        let name = |id: PartId| rocket.part(id).map_or_else(|| "?".to_string(), |p| p.display_name());
        format!("{} · {}", name(separation.joint_between.0), name(separation.joint_between.1))
    };
    let where_rows = vec![form.fixed("Position", separation.station, Quantity::Length), text_row("Found from", found_from), text_row("Joint", joint)];

    let lower_name = stage_name(n, total);
    let separates = timing_rows(&form, config, n, EventKind::Separation, &lower_name, false);
    let ignition_rows = if upper.is_some() { timing_rows(&form, config, n + 1, EventKind::Ignition, &lower_name, false) } else { Vec::new() };

    let rows = sequence(rocket, ctx.physics.motors(), stages, n);
    let colors = ctx.tokens();
    let mut list: Vec<Element<'a, Message>> = Vec::new();
    for r in &rows {
        let time = match r.t {
            Some(t) => format!("{t:.1} s"),
            None => "— s".to_string(),
        };
        let color = match r.subject {
            Subject::Motor => colors.sys_propulsion,
            Subject::Separation => colors.ink,
            Subject::Edited => colors.accent,
        };
        let square = container(Space::new())
            .width(8)
            .height(8)
            .style(move |_: &iced::Theme| container::Style { background: Some(color.into()), ..container::Style::default() });
        list.push(
            row![
                container(text(time).font(MONO).size(12.5).style(theme::muted)).width(52).align_x(iced::alignment::Horizontal::Right),
                square,
                text(r.text.clone()).size(TEXT_BODY).style(theme::ink)
            ]
            .spacing(SPACE_2)
            .align_y(iced::Center)
            .boxed(),
        );
    }
    if long_coast(&rows).is_some() {
        list.push(chip(Severity::Warn, "Long coast: check speed and tilt at ignition").into());
    }
    let name = rocket.configuration_name(rocket.active_config_id()).unwrap_or_default();

    let sections = vec![
        block(1, "Where", false, false, where_rows),
        block(2, "Separation", true, true, separates),
        block(3, &format!("Stage {} ignition", n + 1), true, true, ignition_rows),
        block(4, &format!("Sequence · {name}"), false, true, list),
    ];
    Some((title(n, total), ui::bare(separation.station, Quantity::Length, ctx.units), sections))
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use bowshock_model::{InternalKind, MotorRef, NoseShape, PartKind, StageEvent, StageTrigger};

    /// A two-stage rocket: nose, sustainer tube with an H128W mount, booster tube with an I284W mount; the sustainer lights 1.5 s
    /// after the separation.
    pub fn two_stage() -> Rocket {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
        let upper = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let lower = r.insert(2, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let mount = |r: &mut Rocket, parent: PartId, offset: f64| {
            r.attach_internal(
                parent,
                offset,
                InternalKind::InnerTube { outer_diameter: 0.0381, thickness: 0.002, length: 0.25, motor_mount: true, overhang: 0.0 },
            )
            .unwrap()
        };
        let (m2, m1) = (mount(&mut r, upper, 0.2), mount(&mut r, lower, 0.1));
        let config = r.active_config_id();
        r.set_motor(config, m1, Some(MotorRef::new("I284W", "AeroTech"))).unwrap();
        r.set_motor(config, m2, Some(MotorRef::new("H128W", "AeroTech"))).unwrap();
        set(&mut r, 2, EventKind::Ignition, StageTrigger::Separation, 1.5);
        r
    }

    /// Sets a staging event of the active configuration.
    pub fn set(r: &mut Rocket, stage: usize, kind: EventKind, trigger: StageTrigger, delay: f64) {
        let config = r.active_config_id();
        r.set_stage_event(config, StageEvent { stage, kind, trigger, delay, tilt_lockout: None }).unwrap();
    }

    fn times(r: &Rocket, db: &MotorDb) -> Vec<(String, Option<f64>)> {
        sequence(r, db, &r.stages(), 1).into_iter().map(|row| (row.text, row.t)).collect()
    }

    #[test]
    fn the_sequence_follows_the_configuration_like_the_kit() {
        let r = two_stage();
        let db = MotorDb::bundled();
        let stages = r.stages();
        assert_eq!(stages.len(), 2);
        assert!(stages[0].separation.is_some(), "{stages:?}");
        let burn = db.find("I284W").unwrap().burn_time();
        let sustain = db.find("H128W").unwrap().burn_time();
        let rows = sequence(&r, &db, &stages, 1);
        let texts: Vec<&str> = rows.iter().map(|r| r.text.as_str()).collect();
        assert_eq!(texts, ["Launch, booster ignites", "Booster burnout", "Separation", "Stage 2 ignites", "Sustainer burnout"]);
        let t = |i: usize| rows[i].t.unwrap();
        assert_eq!((t(0), t(1), t(2)), (0.0, burn, burn));
        assert!((t(3) - (burn + 1.5)).abs() < 1e-9 && (t(4) - (burn + 1.5 + sustain)).abs() < 1e-9);
        assert_eq!(rows[3].subject, Subject::Edited);
        assert_eq!(long_coast(&rows), None);
    }

    #[test]
    fn separation_and_ignition_are_different_instants_and_delays_move_every_later_time() {
        let mut r = two_stage();
        let db = MotorDb::bundled();
        set(&mut r, 1, EventKind::Separation, StageTrigger::Burnout, 2.0);
        let burn = db.find("I284W").unwrap().burn_time();
        let rows = sequence(&r, &db, &r.stages(), 1);
        // The separation is 2 s after the burnout; the ignition 1.5 s after the *separation*.
        assert!((rows[2].t.unwrap() - (burn + 2.0)).abs() < 1e-9);
        assert!((rows[3].t.unwrap() - (burn + 3.5)).abs() < 1e-9);
        assert_eq!(long_coast(&rows), None);
        // 3.5 s from separation to ignition is a long coast.
        set(&mut r, 2, EventKind::Ignition, StageTrigger::Separation, 3.5);
        assert_eq!(long_coast(&sequence(&r, &db, &r.stages(), 1)), Some(3.5));
    }

    #[test]
    fn the_sequence_is_what_the_flight_does_for_every_trigger_kind() {
        use bowshock_sim::from_model::sim_input;
        use bowshock_sim::{simulate, EventKind as Flown};
        let db = MotorDb::bundled();
        let burn = db.find("I284W").unwrap().burn_time();
        let cases = [
            (StageTrigger::Burnout, 0.5, StageTrigger::Separation, 0.25),
            (StageTrigger::TimeFromLaunch, burn + 1.0, StageTrigger::TimeFromLaunch, burn + 2.0),
            (StageTrigger::Altitude(60.0), 0.0, StageTrigger::Altitude(120.0), 0.5),
            (StageTrigger::Burnout, 0.0, StageTrigger::Burnout, 1.0),
        ];
        for (sep_trigger, sep_delay, ign_trigger, ign_delay) in cases {
            let mut r = two_stage();
            set(&mut r, 1, EventKind::Separation, sep_trigger, sep_delay);
            set(&mut r, 2, EventKind::Ignition, ign_trigger, ign_delay);
            let rows = sequence(&r, &db, &r.stages(), 1);
            let flight = simulate(&sim_input(&r, &db, Phase::Full, Launch::default()).unwrap());
            let flown = |kind: Flown, stage: usize| flight.trajectory.events.iter().find(|e| e.kind == kind && e.stage == Some(stage)).map(|e| e.t);
            let listed = |text: &str| rows.iter().find(|row| row.text == text).and_then(|row| row.t);
            for (text, kind, stage) in [("Separation", Flown::Separation, 0), ("Stage 2 ignites", Flown::Ignition, 1)] {
                let (a, b) = (listed(text), flown(kind.clone(), stage));
                assert!(a.zip(b).is_some_and(|(a, b)| (a - b).abs() < 0.02), "{sep_trigger:?}/{ign_trigger:?} {text}: listed {a:?}, flown {b:?}");
            }
        }
    }

    #[test]
    fn rows_come_in_time_order_and_a_stage_lit_after_launch_gets_its_own_row() {
        let mut r = two_stage();
        let db = MotorDb::bundled();
        // Separate at 1 s from launch, well before the booster burns out.
        set(&mut r, 1, EventKind::Separation, StageTrigger::TimeFromLaunch, 1.0);
        let t = times(&r, &db);
        let at = |text: &str| t.iter().position(|(name, _)| name == text).unwrap();
        assert!(at("Separation") < at("Booster burnout") || t[at("Booster burnout")].1.is_none(), "{t:?}");
        // The booster is lit 0.5 s after launch: it gets a row of its own.
        set(&mut r, 1, EventKind::Ignition, StageTrigger::TimeFromLaunch, 0.5);
        let t = times(&r, &db);
        assert_eq!(t[0], ("Launch".to_string(), Some(0.0)));
        assert_eq!(t[1], ("Booster ignites".to_string(), Some(0.5)));
    }

    #[test]
    fn a_stage_without_a_motor_has_no_times_and_names_follow_the_stage_count() {
        let mut r = two_stage();
        let db = MotorDb::bundled();
        let config = r.active_config_id();
        let stages = r.stages();
        r.set_motor(config, stages[0].mounts[0], None).unwrap();
        let rows = sequence(&r, &db, &r.stages(), 1);
        let at = |text: &str| rows.iter().find(|row| row.text == text).unwrap().t;
        assert_eq!(at("Booster burnout"), None);
        assert_eq!(at("Separation"), Some(0.0), "a stage without a motor separates when the launch happens (nothing to wait for)");
        assert!(sequence(&r, &db, &r.stages(), 5).is_empty());
        assert_eq!(
            (stage_name(1, 2), stage_name(2, 2), stage_name(2, 3), stage_name(1, 1)),
            ("Booster".into(), "Sustainer".into(), "Stage 2".into(), "Stage 1".into())
        );
        assert_eq!(title(1, 2), "Separation · stage 1 / 2");
    }
}
