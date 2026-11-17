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
use crate::tokens::{text as type_scale, SPACE_1, SPACE_2, SPACE_3, STATUSBAR_H, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::chip::{chip, Severity};
use crate::widgets::glyph::{self, Mark, Tone};
use crate::widgets::segmented::segmented;
use crate::widgets::status;

/// Labels of the phase entries (Full stack, then one per upper stage).
const STAGE_LABELS: [&str; 6] = ["Stage 1", "Stage 2", "Stage 3", "Stage 4", "Stage 5", "Stage 6"];

const DASH: &str = "—";

/// The strip. It measures its own width so the live deltas of a drag give way before the selection summary does.
pub fn status_strip<'a>(state: &'a State, ctx: &'a Ctx) -> Element<'a, Message> {
    iced::widget::responsive(move |size| strip(state, ctx, size.width)).width(Fill).height(STATUSBAR_H).boxed()
}

fn strip<'a>(state: &'a State, ctx: &'a Ctx, width: f32) -> Element<'a, Message> {
    let rocket = ctx.doc.rocket();
    let units = ctx.units;
    let shown = ctx.shown();
    let balance = shown.now.as_ref();
    let wrap = Message::Chrome;

    let name = rocket.configuration_name(rocket.active_config_id()).unwrap_or_default();
    // What the CONFIG cell takes beyond the 150 px `BASE_W` counts for it (a staged configuration's name is long).
    let config_extra = (name.chars().count() as f32 * 8.0 + 110.0 - 150.0).max(0.0);
    let mut cells: Vec<Element<'a, Message>> = vec![config_cell(name, state.menu_is_config())];

    if ctx.physics.is_staged() {
        cells.push(phase_cell(ctx));
    }

    let value = |f: fn(&Balance) -> f64, q: Quantity, unit: bool| {
        balance.map_or_else(|| DASH.to_string(), |b| if unit { ui::with_unit(f(b), q, units) } else { ui::bare(f(b), q, units) })
    };
    // While a drag changes a value, that readout is semibold until the release (StatusStrip), with its change from the
    // committed value after it (`CP 862.3 (+10.6)`).
    let changing = ctx.changing();
    let delta = |f: fn(&Balance) -> f64, q: Quantity| -> Option<String> {
        let (now, was) = (shown.now?, shown.committed?);
        let d = f(&now) - f(&was);
        // Deltas drop their unit (the readout beside them carries it), to keep the strip short.
        let text = ui::bare(d, q, units);
        if !text.chars().any(|c| c.is_ascii_digit() && c != '0') {
            None
        } else if d > 0.0 {
            Some(format!("+{text}"))
        } else {
            Some(text)
        }
    };
    // The issues chip and the selection take room the deltas must leave them.
    let issues = ctx.physics.issues().count();
    let notes = ctx.import_notes.len();
    let chip_label = (issues + notes > 0).then(|| {
        // The issues set the chip's state; import notes alone make it a neutral `info` chip that opens the same list.
        if issues > 0 {
            let word = if ctx.physics.worst() == Level::Err { "error" } else { "warning" };
            let severity = if ctx.physics.worst() == Level::Err { Severity::Err } else { Severity::Warn };
            (severity, format!("{issues} {word}{}", if issues == 1 { "" } else { "s" }))
        } else {
            (Severity::Info, format!("{notes} import note{}", if notes == 1 { "" } else { "s" }))
        }
    });
    let selection = selection_parts(ctx);
    let live_now = changing.mass || changing.length || changing.diameter || changing.cg || changing.cp || changing.margin;
    let mut extra = if ctx.physics.is_staged() { 240.0 } else { 0.0 };
    if let Some((name, mass)) = &selection {
        let chars = name.chars().count().min(SELECTION_NAME_MAX) + mass.as_ref().map_or(0, |m| m.chars().count() + 3);
        extra += 70.0 + chars as f32 * CHAR_W + 32.0;
    }
    if let Some((_, label)) = &chip_label {
        extra += 36.0 + label.chars().count() as f32 * 6.8 + 16.0;
    }
    let mut deltas = [
        delta(|b| b.mass, Quantity::Mass),
        delta(|b| b.length, Quantity::Length),
        delta(|b| b.reference_diameter, Quantity::Length),
        delta(|b| b.cg, Quantity::Length),
        delta(|b| b.cp, Quantity::Length),
        balance.zip(shown.committed).map(|(b, was)| b.margin_cal - was.margin_cal).filter(|d| d.abs() >= 0.005).map(|d| format!("{d:+.2}")),
    ];
    if live_now {
        let keep = deltas_kept(&deltas, extra, width);
        for (d, k) in deltas.iter_mut().zip(keep) {
            if !k {
                *d = None;
            }
        }
    }
    let [d_mass, d_length, d_diameter, d_cg, d_cp, d_margin] = deltas;
    let live = |changing: bool, delta: Option<String>| status::Live { changing, delta, color: None };
    cells.push(status::readout_live("MASS", value(|b| b.mass, Quantity::Mass, true), None, live(changing.mass, d_mass)));
    // A window too narrow for every cell and the chip gives up the reference diameter, then the length (both are in the
    // inspector): the warnings chip and the balance are what the strip is for.
    // The estimate is a little generous towards itself: the cells go a few px before they would touch the chip.
    let over = BASE_W + config_extra + extra - width + 16.0;
    let (show_ref, show_len) = (over <= 0.0, over <= REF_CELL_W);
    let freed = if show_len { 0.0 } else { LEN_CELL_W } + if show_ref { 0.0 } else { REF_CELL_W };
    if show_len {
        cells.push(status::readout_live("LEN", value(|b| b.length, Quantity::Length, true), None, live(changing.length, d_length)));
    }
    if show_ref {
        cells.push(status::readout_live("Ø REF", value(|b| b.reference_diameter, Quantity::Length, true), None, live(changing.diameter, d_diameter)));
    }
    cells.push(status::cg_live(value(|b| b.cg, Quantity::Length, false), ctx.show_cg, Some(wrap(ChromeMessage::ToggleCg)), live(changing.cg, d_cg)));
    cells.push(status::cp_live(value(|b| b.cp, Quantity::Length, false), ctx.show_cp, Some(wrap(ChromeMessage::ToggleCp)), live(changing.cp, d_cp)));
    if let Some(b) = balance {
        let bands = ctx.doc.rocket().stability_bands();
        // A margin that is changing takes its stability band's colour as well.
        let margin = status::Live {
            changing: changing.margin,
            delta: d_margin,
            color: changing.margin.then(|| super::config_picker::margin_color(ctx.tokens(), bands, b.margin_cal)),
        };
        cells.push(status::stability_live(ctx.physics.gauge(), &bands.into(), Some(Message::Mode(Mode::Analyze)), margin));
    } else {
        cells.push(status::readout("STAB", DASH, None));
    }

    let mut trailing: Vec<Element<'a, Message>> = Vec::new();
    // Priority: SELECTED never clips, so the transient message gives way while a value is live (its deltas need the room).
    // The message takes only the room the readouts, the selection and the chip leave: it is cut to fit, and dropped when even a
    // few words do not, so it can never push the chip off the strip's end.
    let room = ((width + freed - BASE_W - config_extra - extra - 24.0) / NOTICE_CHAR_W).floor().max(0.0) as usize;
    if let Some(notice) = ctx.notice.as_ref().filter(|_| !live_now && room >= NOTICE_MIN) {
        trailing.push(single_line(&notice.0, NOTICE_MAX.min(room), TEXT_BODY, false, theme::colored(|t| t.ok)));
    }
    if let Some((name, mass)) = selection {
        trailing.push(summary(&name, mass));
    }
    if let Some((severity, label)) = chip_label {
        // With every optional cell gone and still no room, the chip keeps its count and drops its word.
        let label = if over > REF_CELL_W + LEN_CELL_W { label.split(' ').next().unwrap_or_default().to_string() } else { label };
        // Never wraps: the chip is given the width its words need (icon, gap, text at 12 px, padding).
        let chip_w = 36.0 + label.chars().count() as f32 * 6.8;
        let chip: Element<'a, Message> = chip(severity, label).on_press(wrap(ChromeMessage::ToggleIssues)).into();
        trailing.push(container(chip).width(chip_w).boxed());
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
/// Fewest characters of a notice worth showing; with less room than that it is left out.
const NOTICE_MIN: usize = 10;
/// Average advance of the notice's 13 px sans text, px (on the generous side, so the estimate errs towards cutting).
const NOTICE_CHAR_W: f32 = 7.2;
/// Widths of the LEN and Ø REF cells, px: what leaving one out frees.
const LEN_CELL_W: f32 = 140.0;
const REF_CELL_W: f32 = 125.0;
/// Longest part name in the SELECTED cell before the ellipsis (the mass after it is never cut).
const SELECTION_NAME_MAX: usize = 22;

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

/// The selection as the strip shows it: the part's name (cut with an ellipsis when long) and its mass, which is never cut.
fn selection_parts(ctx: &Ctx) -> Option<(String, Option<String>)> {
    let Selection::Part(id) = ctx.selection else {
        return None;
    };
    let rocket = ctx.doc.rocket();
    if ctx.is_multi() {
        let parts = ctx.selected_parts();
        let mass: f64 = parts.iter().filter_map(|p| rocket.component_mass(*p)).map(|m| m.mass).sum();
        return Some((format!("{} parts", parts.len()), Some(ui::with_unit(mass, Quantity::Mass, ctx.units))));
    }
    let name = match (rocket.part(id), rocket.radial(id), rocket.internal(id)) {
        (Some(p), _, _) => p.display_name(),
        (_, Some(r), _) => r.name.clone().unwrap_or_else(|| r.kind.name().to_string()),
        (_, _, Some(i)) => i.kind.name().to_string(),
        _ => return None,
    };
    let mass = rocket.component_mass(id).map(|m| ui::with_unit(m.mass, Quantity::Mass, ctx.units));
    Some((name, mass))
}

/// The `SELECTED  <name> · <mass>` cell: the strip's label/value pair in its own ruled cell (a hairline each side), like MASS or
/// LEN. Only the name is ever ellipsised.
fn summary<'a>(name: &str, mass: Option<String>) -> Element<'a, Message> {
    let rule = || container(Space::new()).width(1).height(Fill).style(theme::hairline);
    let size = type_scale::NUM_BODY.size;
    let mut content = row![ui::label("SELECTED"), single_line(name, SELECTION_NAME_MAX, size, true, theme::ink)].spacing(SPACE_2).align_y(Center);
    if let Some(mass) = mass {
        content = content.push(text(format!("· {mass}")).size(size).font(theme::MONO).wrapping(Wrapping::None).style(theme::ink).boxed());
    }
    row![rule(), container(content).padding([0.0, SPACE_3]).height(Fill).center_y(Fill), rule()].height(Fill).boxed()
}

/// Rough width (logical px) of the strip without any live delta, from the cells it always has; the phase cell and the label
/// of the configuration add to it.
const BASE_W: f32 = 880.0;
/// Mono 12–13 px advance, with the gap before a delta.
const CHAR_W: f32 = 7.8;

/// Which of the six live deltas (mass, length, Ø, CG, CP, margin) stay: the least important go first (margin, Ø, length, then
/// CP, CG, mass) until what the strip is estimated to need fits `width`, so the SELECTED cell never loses its mass.
fn deltas_kept(deltas: &[Option<String>; 6], extra: f32, width: f32) -> [bool; 6] {
    let mut keep = [true; 6];
    let need = |keep: &[bool; 6]| {
        extra + deltas.iter().zip(keep).filter(|(_, k)| **k).map(|(d, _)| d.as_ref().map_or(0.0, |d| (d.chars().count() + 3) as f32 * CHAR_W)).sum::<f32>()
    };
    for drop in [5, 2, 1, 4, 3, 0] {
        if need(&keep) + BASE_W <= width {
            break;
        }
        keep[drop] = false;
    }
    keep
}

#[cfg(test)]
mod tests {
    use super::*;

    fn deltas() -> [Option<String>; 6] {
        ["+18.4", "+72.0", "+0.5", "+36.6", "+55.7", "-0.38"].map(|d| Some(d.to_string()))
    }

    #[test]
    fn wide_strips_keep_every_delta_and_narrow_ones_drop_the_least_important_first() {
        assert_eq!(deltas_kept(&deltas(), 400.0, 2000.0), [true; 6]);
        // Just too narrow for all six: the margin goes first.
        let all = BASE_W + 400.0 + deltas().iter().flatten().map(|d| (d.chars().count() + 3) as f32 * CHAR_W).sum::<f32>();
        assert_eq!(deltas_kept(&deltas(), 400.0, all - 1.0), [true, true, true, true, true, false]);
        // Out of room: the mass delta is the last to stay, and none is left when nothing fits.
        assert_eq!(deltas_kept(&deltas(), 400.0, 100.0), [false; 6]);
        let kept = deltas_kept(&deltas(), 400.0, BASE_W + 400.0 + 4.0 * 8.0 * CHAR_W);
        assert!(kept[0] && kept[3] && !kept[5] && !kept[2]);
    }
}
