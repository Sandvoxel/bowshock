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

/// Metres as millimetres to one decimal, as the viewport tags and the parts bin show them (`76.2`).
pub fn mm(metres: f64) -> String {
    format!("{:.1}", metres * 1000.0)
}

/// The size line of a radial part: a fin set's root and tip chord and span, else `Ø9.5 × 50.8`.
pub fn radial_size_label(kind: &RadialKind) -> String {
    match kind {
        RadialKind::FinSet { root_chord, tip_chord, span, .. } => format!("{} → {} × {}", mm(*root_chord), mm(*tip_chord), mm(*span)),
        RadialKind::LaunchLug { length, diameter } => format!("Ø{} × {}", mm(*diameter), mm(*length)),
        RadialKind::RailButton { height, diameter } => format!("Ø{} × {}", mm(*diameter), mm(*height)),
        RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => format!("{} × {}", mm(kind.extent()), mm(kind.fin_span().unwrap_or(0.0))),
    }
}

/// The size line of an axial part: `Ø76.2 × 600.0`, or `Ø76.2 → Ø54.0 × 76.2` for a transition.
pub fn size_label(kind: &PartKind) -> String {
    match kind {
        PartKind::Transition { length, fore_diameter, aft_diameter } => format!("Ø{} → Ø{} × {}", mm(*fore_diameter), mm(*aft_diameter), mm(*length)),
        _ => format!("Ø{} × {}", mm(kind.fore_diameter().max(kind.aft_diameter())), mm(kind.length())),
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
        assert_eq!(with_unit(1.142, Quantity::Length, UnitSystem::MmGS), "1142.0 mm");
        assert_eq!(bare(0.7124, Quantity::Length, UnitSystem::MmGS), "712.4");
        assert_eq!(with_unit(1.142, Quantity::Length, UnitSystem::MKgS), "1.142 m");
        assert_eq!(with_unit(1.284, Quantity::Mass, UnitSystem::MKgS), "1.284 kg");
        assert_eq!(with_unit(1.284, Quantity::Mass, UnitSystem::MmGS), "1284.0 g");
        assert_eq!(with_unit(0.0762, Quantity::Length, UnitSystem::InOzS), "3.00 in");
        assert_eq!(with_unit(f64::NAN, Quantity::Length, UnitSystem::MmGS), "— mm");
        assert_eq!(mm(0.0762), "76.2");
    }
}
