//! Preferences > Units: the display unit set, the only place it is set. A named set is one choice; below it each quantity can be
//! given its own unit, which makes the set custom (the select then reads `Custom`).

use std::fmt;

use iced::widget::{column, container, row, text};
use iced::{Center, Element, Fill, Widget};

use super::{card, group, setting, Message};
use crate::app::{ui, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::SPACE_3;
use crate::units::{Family, Quantity, UnitSystem};
use crate::widgets::field;
use crate::widgets::segmented::{segmented_dense, Dense};

/// An entry of the unit select: a named set, or the custom mix. The custom entry reads `Custom…` while a named set is current;
/// while a custom mix is current it stays `Custom`. It has nothing to open: the per-quantity strips are on the page.
#[derive(Debug, Clone, Copy, PartialEq)]
enum SystemLabel {
    Preset(UnitSystem),
    Custom(UnitSystem),
}

impl fmt::Display for SystemLabel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SystemLabel::Preset(set) => f.write_str(&set.label()),
            SystemLabel::Custom(set) if set.preset().is_some() => f.write_str("Custom…"),
            SystemLabel::Custom(_) => f.write_str("Custom"),
        }
    }
}

/// The unit system select: the named sets and `Custom`.
fn units_select<'a>(current: UnitSystem) -> Element<'a, Message> {
    let mut systems: Vec<SystemLabel> = UnitSystem::PRESETS.iter().copied().map(SystemLabel::Preset).collect();
    systems.push(SystemLabel::Custom(current));
    let shown = if current.preset().is_some() { SystemLabel::Preset(current) } else { SystemLabel::Custom(current) };
    field::select(systems, Some(shown), move |s| match s {
        SystemLabel::Preset(set) => Message::Units(set),
        SystemLabel::Custom(_) => Message::Nothing,
    })
}

/// One selector per quantity family (`mm cm m in ft`…), a row of a card each, so lengths can be in inches and masses in kilograms
/// at once.
fn unit_rows<'a>(units: UnitSystem) -> Element<'a, Message> {
    let dense = Dense { height: 22.0, pad: 6.0, size: 11.5 };
    let mut rows = Vec::new();
    for family in Family::ALL {
        let options: Vec<(usize, &'static str)> = family.quantity().units().iter().enumerate().map(|(i, u)| (i, u.symbol)).collect();
        let current = family.quantity().unit_index(units.unit(family));
        let strip = segmented_dense(&options, current, |i| Message::Units(units.with(family, family.quantity().units()[i].symbol)), dense, false);
        rows.push(setting(family.label(), None, strip));
    }
    card(rows)
}

/// How values read in the current set: a length, a mass, a speed and a pressure.
fn sample<'a>(units: UnitSystem) -> Element<'a, Message> {
    let values = [(1.142, Quantity::Length), (0.8505, Quantity::Mass), (312.0, Quantity::Velocity), (101_325.0, Quantity::Pressure)]
        .map(|(value, quantity)| ui::with_unit(value, quantity, units))
        .join(" · ");
    let body = row![ui::label("SAMPLE"), text(values).font(MONO).size(12.0).style(theme::muted).width(Fill)].spacing(SPACE_3).align_y(Center);
    container(body).padding(SPACE_3).width(Fill).boxed()
}

/// The page body.
pub fn view(ctx: &Ctx) -> Element<'_, Message> {
    let select = container(units_select(ctx.units)).width(220);
    let set = card(vec![setting("Unit set", Some("Kept for every rocket and every run."), select.boxed()), sample(ctx.units)]);
    column![
        group("UNIT SET", set, None),
        group("PER QUANTITY", unit_rows(ctx.units), Some("Areas and roughness follow length; angles are degrees, time is seconds.")),
    ]
    .spacing(SPACE_3 + 4.0)
    .boxed()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_custom_mix_reads_custom_and_a_named_set_offers_it() {
        let named = UnitSystem::MM_G_S;
        assert_eq!(SystemLabel::Custom(named).to_string(), "Custom…");
        let mix = named.with(Family::Length, "in");
        assert!(mix.preset().is_none());
        assert_eq!(SystemLabel::Custom(mix).to_string(), "Custom");
    }
}
