//! Small helpers every screen shares: section labels and number formatting in the app's unit system.

use bowshock_model::{PartKind, RadialKind};
use iced::widget::text;
use iced::{Element, Theme, Widget};

use crate::theme::{self, MONO};
use crate::tokens::TEXT_LABEL;
use crate::units::{self, Quantity, Unit, UnitSystem};

/// An uppercase mono section label in `ink-faint`.
pub fn label<'a, M: 'a>(s: impl text::IntoFragment<'a>) -> Element<'a, M> {
    text(s).font(MONO).size(TEXT_LABEL).style(|theme: &Theme| text::Style { color: Some(theme::tokens(theme).ink_faint), selection: None }).boxed()
}

/// A length in the display unit of `system`, number only and a decimal short of an input field's, as the viewport tags, the
/// parts bin and the size lines show lengths (`76.2` in mm, `3.00` in in). Pair it with [`len_unit`].
pub fn len(metres: f64, system: UnitSystem) -> String {
    bare(metres, Quantity::Length, system)
}

/// The symbol of the length unit of `system` (`mm`).
pub fn len_unit(system: UnitSystem) -> &'static str {
    Quantity::Length.unit_for(system).symbol
}

/// The size line of a radial part: a fin set's root and tip chord and span, else `Ø9.5 × 50.8`.
pub fn radial_size_label(kind: &RadialKind, u: UnitSystem) -> String {
    match kind {
        RadialKind::FinSet { root_chord, tip_chord, span, .. } => format!("{} → {} × {}", len(*root_chord, u), len(*tip_chord, u), len(*span, u)),
        RadialKind::LaunchLug { length, diameter } => format!("Ø{} × {}", len(*diameter, u), len(*length, u)),
        RadialKind::RailButton { height, diameter } => format!("Ø{} × {}", len(*diameter, u), len(*height, u)),
        RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => format!("{} × {}", len(kind.extent(), u), len(kind.fin_span().unwrap_or(0.0), u)),
        RadialKind::Pod { gap, body } => format!("Ø{} × {} · gap {}", len(body.radius() * 2.0, u), len(body.length(), u), len(*gap, u)),
    }
}

/// The size line of an axial part: `Ø76.2 × 600.0`, or `Ø76.2 → Ø54.0 × 76.2` for a transition (lengths in the unit set's length unit).
pub fn size_label(kind: &PartKind, u: UnitSystem) -> String {
    match kind {
        PartKind::Transition { length, fore_diameter, aft_diameter } => {
            format!("Ø{} → Ø{} × {}", len(*fore_diameter, u), len(*aft_diameter, u), len(*length, u))
        }
        _ => format!("Ø{} × {}", len(kind.fore_diameter().max(kind.aft_diameter()), u), len(kind.length(), u)),
    }
}

/// `value_si` in the display unit of `quantity` in `system`, as the number and its unit symbol. Lengths drop a decimal
/// (`1142.0 mm`, not `1142.00`): the status strip's readouts are for reading, not for editing.
pub fn readout(value_si: f64, quantity: Quantity, system: UnitSystem) -> (String, &'static str) {
    let unit = quantity.unit_for(system);
    let unit = match quantity {
        Quantity::Length if unit.decimals > 1 => Unit { decimals: unit.decimals - 1, ..*unit },
        _ => *unit,
    };
    (units::format_in(value_si, &unit), unit.symbol)
}

/// [`readout`] as one string, `1142.0 mm`.
pub fn with_unit(value_si: f64, quantity: Quantity, system: UnitSystem) -> String {
    let (number, symbol) = readout(value_si, quantity, system);
    format!("{number} {symbol}")
}

/// [`readout`] without the unit symbol, `712.4`.
pub fn bare(value_si: f64, quantity: Quantity, system: UnitSystem) -> String {
    readout(value_si, quantity, system).0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lengths_drop_one_decimal_and_other_quantities_keep_theirs() {
        assert_eq!(with_unit(1.142, Quantity::Length, UnitSystem::MM_G_S), "1142.0 mm");
        assert_eq!(bare(0.7124, Quantity::Length, UnitSystem::MM_G_S), "712.4");
        assert_eq!(with_unit(1.142, Quantity::Length, UnitSystem::M_KG_S), "1.142 m");
        assert_eq!(with_unit(1.284, Quantity::Mass, UnitSystem::M_KG_S), "1.284 kg");
        assert_eq!(with_unit(1.284, Quantity::Mass, UnitSystem::MM_G_S), "1284.0 g");
        assert_eq!(with_unit(0.0762, Quantity::Length, UnitSystem::IN_OZ_S), "3.00 in");
        assert_eq!(with_unit(f64::NAN, Quantity::Length, UnitSystem::MM_G_S), "— mm");
        // A mixed set: lengths in inches and masses in kilograms side by side.
        let mixed = UnitSystem::MM_G_S.with(crate::units::Family::Length, "in").with(crate::units::Family::Mass, "kg");
        assert_eq!(with_unit(0.0762, Quantity::Length, mixed), "3.00 in");
        assert_eq!(with_unit(1.284, Quantity::Mass, mixed), "1.284 kg");
        assert_eq!(len(0.0762, UnitSystem::MM_G_S), "76.2");
        assert_eq!(len(0.0762, mixed), "3.00");
        assert_eq!(len_unit(mixed), "in");
    }
}
