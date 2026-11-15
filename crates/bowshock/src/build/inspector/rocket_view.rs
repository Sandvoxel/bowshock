//! The Rocket inspector: what shows when nothing is selected (design/components/Inspector). The rocket's name and designer, the
//! reference diameter and length (derived, so `AUTO`), the stage list with each stage's separation and motors, a summary of the
//! active configuration and the balance.

use bowshock_model::{PartId, Rocket, Stage, StageIssue};
use iced::widget::{column, mouse_area, row, text, text_input, Space};
use iced::{mouse, Element, Fill, Widget};

use super::form::{text_row, value_row, Form};
use super::separation::stage_name;
use super::Message;
use crate::app::{ui, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_2, TEXT_BODY, TEXT_LABEL};
use crate::units::Quantity;
use crate::widgets::button::{button, Weight};
use crate::widgets::chip::{chip, Severity};
use crate::widgets::field::{labeled, section};

fn name_field<'a>(placeholder: &'static str, value: &'a str, on_input: fn(String) -> Message) -> Element<'a, Message> {
    text_input(placeholder, value).on_input(on_input).size(TEXT_BODY).padding([4.0, SPACE_2]).style(theme::input).boxed()
}

/// The designations of the motors the active configuration puts in `stage`, joined by `+` for a cluster, or `No motor`.
pub fn stage_motors(rocket: &Rocket, stage: &Stage) -> String {
    let config = rocket.active_configuration();
    let names: Vec<String> = stage.mounts.iter().filter_map(|id| config.slot(*id)?.motor.as_ref().map(|m| m.designation.clone())).collect();
    if names.is_empty() {
        "No motor".to_string()
    } else {
        names.join(" + ")
    }
}

fn stage_row<'a>(rocket: &Rocket, ctx: &'a Ctx, stage: &Stage, total: usize) -> Element<'a, Message> {
    let units = ctx.units;
    let name = stage_name(stage.index, total);
    let title = if name == stage.name { name } else { format!("{} · {}", stage.name, name) };
    let heading = row![
        text(title).size(TEXT_BODY).style(theme::ink),
        Space::new().width(Fill),
        text(stage_motors(rocket, stage)).font(MONO).size(12.5).style(theme::muted)
    ]
    .spacing(SPACE_2);
    let mut rows: Vec<Element<'a, Message>> = vec![heading.boxed()];
    let span = format!("{} → {}", ui::bare(stage.fore_station, Quantity::Length, units), ui::with_unit(stage.aft_station, Quantity::Length, units));
    rows.push(text(span).font(MONO).size(TEXT_LABEL).style(theme::faint).boxed());
    if let Some(separation) = stage.separation {
        let label = format!("Separation at {}", ui::with_unit(separation.station, Quantity::Length, units));
        let line: Element<'a, Message> = text(label).font(MONO).size(TEXT_LABEL).style(theme::colored(|t| t.accent)).boxed();
        rows.push(mouse_area(line).on_press(Message::SelectSeparation(stage.index)).interaction(mouse::Interaction::Pointer).boxed());
    }
    if let Some(StageIssue::NoJoint { tube, station }) = stage.issue {
        let tube_name = rocket.part(tube).map_or_else(|| "tube".to_string(), |p| p.display_name());
        let at = ui::with_unit(station, Quantity::Length, units);
        rows.push(chip(Severity::Warn, format!("No joint between the two motors · {tube_name}")).into());
        rows.push(text(format!("Split {tube_name} at {at} to fix it")).size(TEXT_LABEL).style(theme::muted).boxed());
        rows.push(button("Split tube here", Weight::Secondary).on_press(Message::SplitTube(tube, station)).into());
    }
    column(rows).spacing(2).boxed()
}

/// The header title and the sections of the Rocket inspector.
pub fn view<'a>(ctx: &'a Ctx) -> (String, Vec<Element<'a, Message>>) {
    let rocket = ctx.doc.rocket();
    let form = Form { ctx, limit: None, first: None };
    let units = ctx.units;
    let title = if rocket.name.is_empty() { "Rocket".to_string() } else { rocket.name.clone() };

    let reference = rocket.reference_diameter().unwrap_or(0.0);
    let identity = vec![
        labeled("Name", name_field("Untitled rocket", &rocket.name, Message::RocketName)),
        labeled("Designer", name_field("Who built it", &rocket.designer, Message::RocketDesigner)),
        form.fixed("Reference Ø", reference, Quantity::Length),
        form.fixed("Length", rocket.length(), Quantity::Length),
        text_row("Parts", format!("{}", rocket.parts().len() + rocket.radials().len() + rocket.internals().len())),
    ];

    let stages = ctx.physics.stages();
    let mut stage_rows: Vec<Element<'a, Message>> = Vec::new();
    // The aft-most stage fires first and is listed first, as the status strip's name reads.
    for stage in stages {
        stage_rows.push(stage_row(rocket, ctx, stage, stages.len()));
    }
    if stage_rows.is_empty() {
        stage_rows.push(text("Add a motor mount to make a stage.").size(TEXT_BODY).style(theme::muted).boxed());
    }

    let config = rocket.active_configuration();
    let mut config_rows: Vec<Element<'a, Message>> = vec![text_row("Active", rocket.configuration_name(config.id).unwrap_or_else(|| config.display_name()))];
    config_rows.push(text_row("Configurations", format!("{}", rocket.configurations().len())));
    for mount in rocket.motor_mounts() {
        let slot = config.slot(mount.id);
        let motor = slot.and_then(|s| s.motor.as_ref()).map_or_else(|| "empty".to_string(), |m| m.designation.clone());
        let delay = match slot {
            Some(s) if s.motor.is_some() && s.is_plugged() => "plugged".to_string(),
            Some(s) if s.motor.is_some() => format!("{:.0} s delay", s.delay),
            _ => String::new(),
        };
        let label = mount_label(ctx.physics.stages(), mount.id);
        config_rows.push(value_row(label, format!("{motor}  {delay}").trim_end().to_string(), None));
    }

    let mut balance_rows: Vec<Element<'a, Message>> = Vec::new();
    match ctx.physics.balance() {
        Some(b) => {
            balance_rows.push(value_row("Mass", ui::with_unit(b.mass, Quantity::Mass, units), None));
            balance_rows.push(value_row("CG", ui::with_unit(b.cg, Quantity::Length, units), None));
            balance_rows.push(value_row("CP", ui::with_unit(b.cp, Quantity::Length, units), None));
            balance_rows.push(value_row("Margin", format!("{:.2} cal", b.margin_cal), None));
        }
        None => balance_rows.push(text("Nothing to weigh yet.").size(TEXT_BODY).style(theme::muted).boxed()),
    }

    let sections = vec![
        section(1, "Rocket", false, identity),
        section(2, "Stages", true, stage_rows),
        section(3, "Configuration", true, config_rows),
        section(4, "Balance", true, balance_rows),
    ];
    (title, sections)
}

/// The row label of a motor mount: the stage it fires in (labels are `'static` in the field column).
fn mount_label(stages: &[Stage], id: PartId) -> &'static str {
    let stage = stages.iter().find(|s| s.mounts.contains(&id)).map_or(1, |s| s.index);
    match stage {
        1 => "Stage 1 motor",
        2 => "Stage 2 motor",
        3 => "Stage 3 motor",
        _ => "Motor",
    }
}
