//! The separation inspector (design/components/Staging): where the rocket separates (read-only, found from the motor mounts),
//! when it separates and when the next stage lights (both stored with the active configuration, so `PER CONFIG`), and the
//! sequence of events worked out from the configuration's motors.

use bowshock_model::{Configuration, Ignition, PartId, Rocket, Stage};
use bowshock_motors::{Motor, MotorDb};
use iced::widget::{column, container, row, text, Space};
use iced::{Element, Fill, Widget};

use super::edit::{Field, Ignite};
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

/// The motor the active configuration puts in `stage`: the first mount that holds one, with its slot.
fn stage_motor<'a>(rocket: &'a Rocket, motors: &'a MotorDb, stage: &Stage) -> Option<(&'a bowshock_model::MotorSlot, &'a Motor)> {
    let config: &Configuration = rocket.active_configuration();
    stage.mounts.iter().find_map(|id| {
        let slot = config.slot(*id)?;
        let reference = slot.motor.as_ref()?;
        let motor = motors
            .motors
            .iter()
            .find(|m| m.designation.eq_ignore_ascii_case(reference.designation.trim()) && m.manufacturer.eq_ignore_ascii_case(reference.manufacturer.trim()))
            .or_else(|| motors.find(&reference.designation))?;
        Some((slot, motor))
    })
}

/// When each stage lights and burns out: `(ignition, burnout)` in stage order. A stage lit at launch ignites at 0; one lit by
/// separation ignites its delay after the separation below it, which is the stage below's burnout plus its separation delay.
pub fn timeline(rocket: &Rocket, motors: &MotorDb, stages: &[Stage]) -> Vec<(f64, Option<f64>)> {
    let config = rocket.active_configuration();
    let mut out: Vec<(f64, Option<f64>)> = Vec::with_capacity(stages.len());
    for stage in stages {
        let motor = stage_motor(rocket, motors, stage);
        let ignition = match (stage.index, motor.map(|(slot, _)| slot.ignition)) {
            (i, Some(Ignition::StageSeparation { delay })) if i > 1 => {
                let below = out.get(i - 2).copied().unwrap_or((0.0, None));
                below.1.unwrap_or(below.0) + config.separation_delay(i - 1) + delay
            }
            _ => 0.0,
        };
        out.push((ignition, motor.map(|(_, m)| ignition + m.burn_time())));
    }
    out
}

/// The sequence of the separation of stage `n` from the stage above it: launch, the lower stage's burnout, separation, the
/// upper stage's ignition and burnout.
pub fn sequence(rocket: &Rocket, motors: &MotorDb, stages: &[Stage], n: usize) -> Vec<SeqRow> {
    let times = timeline(rocket, motors, stages);
    let total = stages.len();
    let (Some(lower), Some(upper)) = (n.checked_sub(1).and_then(|i| times.get(i)), times.get(n)) else {
        return Vec::new();
    };
    let config = rocket.active_configuration();
    let (lower_name, upper_name) = (stage_name(n, total), stage_name(n + 1, total));
    let separation = lower.1.map(|b| b + config.separation_delay(n));
    let mut rows = vec![SeqRow { t: Some(0.0), subject: Subject::Motor, text: format!("Launch, {} ignites", lower_name.to_lowercase()) }];
    rows.push(SeqRow { t: lower.1, subject: Subject::Motor, text: format!("{lower_name} burnout") });
    rows.push(SeqRow { t: separation, subject: Subject::Separation, text: "Separation".into() });
    rows.push(SeqRow { t: separation.is_some().then_some(upper.0), subject: Subject::Edited, text: format!("Stage {} ignites", n + 1) });
    rows.push(SeqRow { t: upper.1, subject: Subject::Motor, text: format!("{upper_name} burnout") });
    rows
}

/// The coast between the separation and the next stage's ignition, when it is long enough to warn about.
pub fn long_coast(rows: &[SeqRow]) -> Option<f64> {
    let separation = rows.iter().find(|r| r.subject == Subject::Separation)?.t?;
    let ignition = rows.iter().find(|r| r.subject == Subject::Edited)?.t?;
    let coast = ignition - separation;
    (coast > LONG_COAST).then_some(coast)
}

fn badge<'a>(label: &'static str) -> Element<'a, Message> {
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

/// The title and the station tag of the header: `Separation · stage 1/2` and `870.0`.
pub fn title(n: usize, total: usize) -> String {
    format!("Separation · stage {n}/{total}")
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
    let separates = vec![
        labeled("Separates on", choose(vec![((), format!("{lower_name} burnout"))], &(), |()| Message::Noop)),
        form.num("Delay", config.separation_delay(n), Quantity::Time, Field::SeparationDelay(n)),
    ];

    let mut ignition_rows: Vec<Element<'a, Message>> = Vec::new();
    if let Some(upper) = upper {
        let slot = upper.mounts.iter().find_map(|id| config.slot(*id).filter(|s| s.motor.is_some()));
        let (on, delay) = match slot.map(|s| s.ignition) {
            Some(Ignition::StageSeparation { delay }) => (Ignite::Separation, delay),
            _ => (Ignite::Launch, 0.0),
        };
        let options = vec![(Ignite::Separation, "Separation".to_string()), (Ignite::Launch, "Launch".to_string())];
        ignition_rows.push(labeled("Ignites on", choose(options, &on, Message::IgniteOn)));
        if on == Ignite::Separation {
            ignition_rows.push(form.num("Delay", delay, Quantity::Time, Field::IgnitionDelay));
        }
    }

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
    if let Some(coast) = long_coast(&rows) {
        list.push(chip(Severity::Warn, format!("Long coast · {coast:.1} s (max {LONG_COAST:.0}): check speed and tilt at ignition")).into());
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
    use bowshock_model::{InternalKind, MotorRef, NoseShape, PartKind};

    /// A two-stage rocket: nose, sustainer tube with an H128W mount, booster tube with an I284W mount.
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
        r.set_ignition(config, m2, Ignition::StageSeparation { delay: 1.5 }).unwrap();
        r
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
    fn delays_move_every_later_time_and_a_long_coast_warns() {
        let mut r = two_stage();
        let db = MotorDb::bundled();
        let config = r.active_config_id();
        r.set_stage_separation(config, 1, 2.0).unwrap();
        let stages = r.stages();
        let burn = db.find("I284W").unwrap().burn_time();
        let rows = sequence(&r, &db, &stages, 1);
        assert!((rows[2].t.unwrap() - (burn + 2.0)).abs() < 1e-9);
        assert!((rows[3].t.unwrap() - (burn + 3.5)).abs() < 1e-9);
        // 1.5 s from separation to ignition is fine; 3.5 s is a long coast.
        assert_eq!(long_coast(&rows), None);
        let mount = stages[1].mounts[0];
        r.set_ignition(config, mount, Ignition::StageSeparation { delay: 3.5 }).unwrap();
        assert_eq!(long_coast(&sequence(&r, &db, &r.stages(), 1)), Some(3.5));
        // Lit at launch there is no wait at all.
        r.set_ignition(config, mount, Ignition::Launch).unwrap();
        let rows = sequence(&r, &db, &r.stages(), 1);
        assert_eq!(rows[3].t, Some(0.0));
    }

    #[test]
    fn a_stage_without_a_motor_has_no_times_and_names_follow_the_stage_count() {
        let mut r = two_stage();
        let db = MotorDb::bundled();
        let config = r.active_config_id();
        let stages = r.stages();
        r.set_motor(config, stages[0].mounts[0], None).unwrap();
        let rows = sequence(&r, &db, &r.stages(), 1);
        assert_eq!(rows[1].t, None);
        assert_eq!(rows[2].t, None);
        assert!(sequence(&r, &db, &r.stages(), 5).is_empty());
        assert_eq!(
            (stage_name(1, 2), stage_name(2, 2), stage_name(2, 3), stage_name(1, 1)),
            ("Booster".into(), "Sustainer".into(), "Stage 2".into(), "Stage 1".into())
        );
        assert_eq!(title(1, 2), "Separation · stage 1/2");
    }
}
