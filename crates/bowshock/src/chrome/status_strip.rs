//! The status strip (design/components/StatusStrip, ConfigPicker, Staging): CONFIG cell (opens the configuration menu), PHASE
//! switch on a staged rocket, MASS, LEN, Ø REF, CG, CP, the compact stability gauge, then the selection summary and the
//! warnings chip pinned right. Every value comes from the physics cache and is formatted in the app's unit system.

use bowshock_sim::from_model::{Balance, Level, Phase};
use iced::widget::text::Wrapping;
use iced::widget::{button, container, row, stack, text, tooltip, Space};
use iced::{Center, Element, Fill, Widget};

use super::{Message as ChromeMessage, State};
use crate::app::{ui, Ctx, Message, Mode, Selection};
use crate::theme;
use crate::tokens::{text as type_scale, SPACE_1, SPACE_2, SPACE_3, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::chip::{chip, Severity};
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
    // While a drag changes a value, that readout is semibold until the release (StatusStrip).
    let changing = ctx.changing();
    cells.push(status::readout_live("MASS", value(|b| b.mass, Quantity::Mass, true), None, changing.mass));
    cells.push(status::readout_live("LEN", value(|b| b.length, Quantity::Length, true), None, changing.length));
    cells.push(status::readout_live("Ø REF", value(|b| b.reference_diameter, Quantity::Length, true), None, changing.diameter));
    cells.push(status::cg_live(value(|b| b.cg, Quantity::Length, false), ctx.show_cg, Some(wrap(ChromeMessage::ToggleCg)), changing.cg));
    cells.push(status::cp_live(value(|b| b.cp, Quantity::Length, false), ctx.show_cp, Some(wrap(ChromeMessage::ToggleCp)), changing.cp));
    if balance.is_some() {
        cells.push(status::stability_live(
            ctx.physics.gauge(),
            &ctx.doc.rocket().stability_bands().into(),
            Some(Message::Mode(Mode::Analyze)),
            changing.margin,
        ));
    } else {
        cells.push(status::readout("STAB", DASH, None));
    }

    let mut trailing: Vec<Element<'a, Message>> = Vec::new();
    if let Some(notice) = &ctx.notice {
        trailing.push(single_line(&notice.0, NOTICE_MAX, TEXT_BODY, false, theme::colored(|t| t.ok)));
    }
    if let Some(summary) = selection_summary(ctx) {
        trailing.push(summary);
    }
    let issues: Vec<_> = ctx.physics.issues().collect();
    if !issues.is_empty() {
        let severity = if ctx.physics.worst() == Level::Err { Severity::Err } else { Severity::Warn };
        let word = if ctx.physics.worst() == Level::Err { "error" } else { "warning" };
        let label = format!("{} {word}{}", issues.len(), if issues.len() == 1 { "" } else { "s" });
        // Never wraps: the chip is given the width its words need (icon, gap, text at 12 px, padding).
        let width = 36.0 + label.chars().count() as f32 * 6.8;
        let chip: Element<'a, Message> = chip(severity, label).on_press(wrap(ChromeMessage::ToggleIssues)).into();
        trailing.push(container(chip).width(width).boxed());
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
    // The kit's cell is at least 150 px wide with the caret at its right edge. It keeps that width while its menu is open (the
    // menu is as wide as the bin on its own) so that opening it never squeezes the cells and the warnings chip beside it.
    let natural = name.chars().count() as f32 * 8.0 + 110.0;
    let width = natural.max(150.0);
    let content = row![
        ui::label("CONFIG"),
        text(name).font(theme::MONO).size(type_scale::NUM_BODY.size).wrapping(Wrapping::None).style(theme::ink),
        Space::new().width(Fill),
        glyph::icon(Mark::CaretUp, tone, 10.0)
    ]
    .spacing(SPACE_2)
    .align_y(Center);
    let body = container(content).padding([0.0, SPACE_3]).width(width).height(Fill).center_y(Fill);
    let press = Message::Chrome(ChromeMessage::ToggleConfigMenu);
    let cell: Element<'a, Message> = if open {
        let held = button(body).padding(0).height(Fill).on_press(press).style(|t: &iced::Theme, _| button::Style {
            background: Some(iced::Background::Color(theme::tokens(t).bg_300)),
            text_color: theme::tokens(t).ink,
            ..button::Style::default()
        });
        let edge = container(Space::new())
            .width(Fill)
            .height(2)
            .style(|t: &iced::Theme| container::Style { background: Some(iced::Background::Color(theme::tokens(t).accent)), ..container::Style::default() });
        stack![held, edge].boxed()
    } else {
        button(body).padding(0).height(Fill).style(theme::ghost).on_press(press).boxed()
    };
    row![cell, container(Space::new()).width(1).height(Fill).style(theme::hairline)].height(Fill).boxed()
}

/// Longest notice / selection line shown before it is cut with an ellipsis (characters).
const NOTICE_MAX: usize = 44;
const SELECTION_MAX: usize = 28;

/// `full` on one line, cut to `max` characters with an ellipsis; the cut text has a tooltip with all of it. Never wraps, so the
/// strip keeps its height.
fn single_line<'a>(full: &str, max: usize, size: f32, mono: bool, style: impl Fn(&iced::Theme) -> text::Style + 'a) -> Element<'a, Message> {
    let cut = full.chars().count() > max;
    let shown: String = if cut { full.chars().take(max.saturating_sub(1)).collect::<String>().trim_end().to_string() + "…" } else { full.to_string() };
    let mut line = text(shown).size(size).wrapping(Wrapping::None).style(style);
    if mono {
        line = line.font(theme::MONO);
    }
    if !cut {
        return line.boxed();
    }
    let bubble = container(text(full.to_string()).size(TEXT_BODY)).padding([SPACE_1, SPACE_2]).style(|t: &iced::Theme| theme::tooltip(t));
    tooltip(line, bubble).position(tooltip::Position::Top).gap(SPACE_1).boxed()
}

/// `Selected: Fin set · 42.6 g` before the warnings, when a part is selected.
fn selection_summary<'a>(ctx: &'a Ctx) -> Option<Element<'a, Message>> {
    let Selection::Part(id) = ctx.selection else {
        return None;
    };
    let rocket = ctx.doc.rocket();
    if ctx.is_multi() {
        let parts = ctx.selected_parts();
        let mass: f64 = parts.iter().filter_map(|p| rocket.component_mass(*p)).map(|m| m.mass).sum();
        return Some(summary(format!("{} parts · {}", parts.len(), ui::with_unit(mass, Quantity::Mass, ctx.units))));
    }
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
    Some(summary(line))
}

/// The `SELECTED  <line>` cell: the strip's label/value pair in its own ruled cell (a hairline each side), like MASS or LEN.
fn summary<'a>(line: String) -> Element<'a, Message> {
    let rule = || container(Space::new()).width(1).height(Fill).style(theme::hairline);
    let content = row![ui::label("SELECTED"), single_line(&line, SELECTION_MAX, type_scale::NUM_BODY.size, true, theme::ink)].spacing(SPACE_2).align_y(Center);
    row![rule(), container(content).padding([0.0, SPACE_3]).height(Fill).center_y(Fill), rule()].height(Fill).boxed()
}
