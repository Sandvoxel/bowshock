//! Small helpers every screen shares: section labels and number formatting in the app's unit system.

use bowshock_model::{PartKind, RadialKind};
use iced::widget::text;
use iced::{Element, Theme, Widget};

use crate::theme::{self, MONO};
use crate::tokens::TEXT_LABEL;
use crate::units::UnitSystem;
pub use crate::units::{bare, len, len_unit, readout, with_unit};

/// An uppercase mono section label in `ink-faint`.
pub fn label<'a, M: 'a>(s: impl text::IntoFragment<'a>) -> Element<'a, M> {
    text(s).font(MONO).size(TEXT_LABEL).style(|theme: &Theme| text::Style { color: Some(theme::tokens(theme).ink_faint), selection: None }).boxed()
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
