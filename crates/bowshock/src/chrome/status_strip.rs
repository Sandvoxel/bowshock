//! The status strip (design/components/StatusStrip, ConfigPicker, Staging): CONFIG cell (opens the configuration menu), PHASE
//! switch on a staged rocket, MASS, LEN, Ø REF, CG, CP, the compact stability gauge, then the selection summary and the
//! warnings chip pinned right. Every value comes from the physics cache and is formatted in the app's unit system.

use bowshock_sim::from_model::{Balance, Level, Phase};
use iced::widget::{container, row, text, Space};
use iced::{Center, Element, Fill, Widget};

use super::{Message as ChromeMessage, State};
use crate::app::{ui, Ctx, Message, Mode, Selection};
use crate::theme;
use crate::tokens::{text as type_scale, SPACE_2, SPACE_3, STATUSBAR_H, TEXT_BODY, TEXT_LABEL};
use crate::units::Quantity;
use crate::widgets::chip::{chip, Severity};
use crate::widgets::gauge::Bands;
use crate::widgets::glyph::{self, Mark, Tone};
use crate::widgets::segmented::segmented;
use crate::widgets::status;

/// Labels of the phase entries (Full stack, then one per upper stage).
const STAGE_LABELS: [&str; 6] = ["Stage 1", "Stage 2", "Stage 3", "Stage 4", "Stage 5", "Stage 6"];

const DASH: &str = "—";

/// The strip.
pub fn status_strip<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    let rocket = ctx.doc.rocket();
    let units = ctx.units;
    let balance = ctx.physics.balance();
    let wrap = Message::Chrome;

    let name = rocket.configuration_name(rocket.active_config_id()).unwrap_or_default();
    let mut cells: Vec<Element<'a, Message>> = vec![config_cell(name, state.menu_is_config())];

    if ctx.physics.is_staged() {
        cells.push(phase_cell(ctx));
    }

    let value = |f: fn(&Balance) -> f64, q: Quantity, unit: bool| {
        balance.map_or_else(|| DASH.to_string(), |b| if unit { ui::with_unit(f(b), q, units) } else { ui::bare(f(b), q, units) })
    };
    cells.push(status::readout("MASS", value(|b| b.mass, Quantity::Mass, true), None));
    cells.push(status::readout("LEN", value(|b| b.length, Quantity::Length, true), None));
    cells.push(status::readout("Ø REF", value(|b| b.reference_diameter, Quantity::Length, true), None));
    cells.push(status::cg(value(|b| b.cg, Quantity::Length, false), None));
    cells.push(status::cp(value(|b| b.cp, Quantity::Length, false), None));
    if balance.is_some() {
        cells.push(status::stability(ctx.physics.gauge(), &Bands::default(), Some(Message::Mode(Mode::Analyze))));
    } else {
        cells.push(status::readout("STAB", DASH, None));
    }

    let mut trailing: Vec<Element<'a, Message>> = Vec::new();
    if let Some(notice) = &ctx.notice {
        trailing.push(text(notice.0.as_str()).size(TEXT_BODY).style(theme::colored(|t| t.ok)).boxed());
    }
    if let Some(summary) = selection_summary(ctx) {
        trailing.push(summary);
    }
    let issues: Vec<_> = ctx.physics.issues().collect();
    if !issues.is_empty() {
        let severity = if ctx.physics.worst() == Level::Err { Severity::Err } else { Severity::Warn };
        let word = if ctx.physics.worst() == Level::Err { "error" } else { "warning" };
        let label = format!("{} {word}{}", issues.len(), if issues.len() == 1 { "" } else { "s" });
        trailing.push(chip(severity, label).on_press(wrap(ChromeMessage::ToggleIssues)).into());
    }
    status::strip(cells, trailing)
}

/// `PHASE  [Full stack | Stage 2 …]`: what is flying (Staging).
fn phase_cell<'a>(ctx: &'a Ctx) -> Element<'a, Message> {
    let mut options: Vec<(Phase, &str)> = vec![(Phase::Full, "Full stack")];
    for stage in ctx.physics.stages().iter().skip(1) {
        options.push((Phase::UpperFrom(stage.index), STAGE_LABELS[(stage.index - 1).min(STAGE_LABELS.len() - 1)]));
    }
    let switch = segmented(&options, ctx.physics.phase(), |p| Message::Chrome(ChromeMessage::Phase(p)));
    status::cell(row![ui::label("PHASE"), switch].spacing(SPACE_2).align_y(Center).boxed(), None)
}

/// The CONFIG cell: `CONFIG H128W ▴`, like [`status::config`] but owning its name (the name is computed, not stored). The caret
/// points up because the menu opens upward.
fn config_cell<'a>(name: String, open: bool) -> Element<'a, Message> {
    let tone = if open { Tone::Accent } else { Tone::Muted };
    let content = row![
        ui::label("CONFIG"),
        text(name).font(theme::MONO).size(type_scale::NUM_BODY.size).style(theme::ink),
        Space::new().width(SPACE_2),
        glyph::icon(Mark::CaretUp, tone, 10.0)
    ]
    .spacing(SPACE_2)
    .align_y(Center);
    status::cell(content.boxed(), Some(Message::Chrome(ChromeMessage::ToggleConfigMenu)))
}

/// `Selected: Fin set · 42.6 g` before the warnings, when a part is selected.
fn selection_summary<'a>(ctx: &'a Ctx) -> Option<Element<'a, Message>> {
    let Selection::Part(id) = ctx.selection else {
        return None;
    };
    let rocket = ctx.doc.rocket();
    let name = match (rocket.part(id), rocket.radial(id), rocket.internal(id)) {
        (Some(p), _, _) => p.display_name(),
        (_, Some(r), _) => r.name.clone().unwrap_or_else(|| r.kind.name().to_string()),
        (_, _, Some(i)) => i.kind.name().to_string(),
        _ => return None,
    };
    let mass = rocket.component_mass(id).map(|m| ui::with_unit(m.mass, Quantity::Mass, ctx.units));
    let line = match mass {
        Some(mass) => format!("{name} · {mass}"),
        None => name,
    };
    Some(
        container(row![ui::label("SELECTED"), text(line).font(theme::MONO).size(TEXT_LABEL + 1.5).style(theme::ink)].spacing(SPACE_2).align_y(Center))
            .padding([0.0, SPACE_3])
            .height(Fill)
            .center_y(STATUSBAR_H)
            .boxed(),
    )
}
