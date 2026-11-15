//! Parts-bin rail icons (design/assets/icons): the seven category icons, embedded as SVG and tinted by state rather than
//! shipped as PNGs. Idle `ink-faint`, hover `ink` on `bg-400`, active `ink` on `bg-200` with a 2 px `accent` bar on the
//! left edge.
//!
//! ```ignore
//! for c in RailIcon::ALL { rail_button(c, c == self.category, Message::Category(c)) }
//! ```

use iced::widget::svg::{self, Svg};
use iced::widget::{button, container, row, tooltip, Space};
use iced::{Element, Fill, Theme, Widget};

use crate::theme::{self, tokens};
use crate::tokens::{RAIL_W, SPACE_1, SPACE_2};

/// The parts-bin categories, top to bottom (keys `1` to `7` switch between them).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RailIcon {
    /// Body tubes, nose cones, transitions.
    Airframe,
    /// Fins, canards, rail buttons and lugs.
    FinsGuides,
    /// Bulkheads, motor mounts, couplers.
    Internal,
    /// Chutes, shock cords, charges.
    Recovery,
    /// Ballast and payload masses.
    Mass,
    /// Motors.
    Motors,
    /// Saved assemblies and vendor parts.
    Presets,
}

impl RailIcon {
    /// All categories in rail order.
    pub const ALL: [RailIcon; 7] =
        [RailIcon::Airframe, RailIcon::FinsGuides, RailIcon::Internal, RailIcon::Recovery, RailIcon::Mass, RailIcon::Motors, RailIcon::Presets];

    /// The category name shown in the tooltip.
    pub fn name(self) -> &'static str {
        match self {
            RailIcon::Airframe => "Airframe",
            RailIcon::FinsGuides => "Fins & guides",
            RailIcon::Internal => "Internal",
            RailIcon::Recovery => "Recovery",
            RailIcon::Mass => "Mass",
            RailIcon::Motors => "Motors",
            RailIcon::Presets => "Presets",
        }
    }

    fn bytes(self) -> &'static [u8] {
        match self {
            RailIcon::Airframe => include_bytes!("../../assets/icons/bs-rail-airframe.svg"),
            RailIcon::FinsGuides => include_bytes!("../../assets/icons/bs-rail-fins-guides.svg"),
            RailIcon::Internal => include_bytes!("../../assets/icons/bs-rail-internal.svg"),
            RailIcon::Recovery => include_bytes!("../../assets/icons/bs-rail-recovery.svg"),
            RailIcon::Mass => include_bytes!("../../assets/icons/bs-rail-mass.svg"),
            RailIcon::Motors => include_bytes!("../../assets/icons/bs-rail-motors.svg"),
            RailIcon::Presets => include_bytes!("../../assets/icons/bs-rail-presets.svg"),
        }
    }
}

/// The bare 24 px icon, tinted `ink` when `lit` and `ink-faint` otherwise.
pub fn rail_icon<'a>(icon: RailIcon, lit: bool) -> Svg<'a, Theme> {
    svg(icon, lit, 24.0)
}

fn svg<'a>(icon: RailIcon, lit: bool, size: f32) -> Svg<'a, Theme> {
    iced::widget::svg(svg::Handle::from_memory(icon.bytes())).width(size).height(size).style(move |theme: &Theme, status| {
        let t = tokens(theme);
        let hovered = status == svg::Status::Hovered;
        svg::Style { color: Some(if lit || hovered { t.ink } else { t.ink_faint }) }
    })
}

/// A rail button: `RAIL_W` square, the active one with a 2 px `accent` bar on its left edge, a tooltip with the category name.
/// `on_press` is published on click.
pub fn rail_button<'a, M: Clone + 'a>(icon: RailIcon, active: bool, on_press: M) -> Element<'a, M> {
    let face = button(container(rail_icon(icon, active)).center(Fill)).width(Fill).height(RAIL_W).padding(0).style(theme::rail(active)).on_press(on_press);
    let bar = container(Space::new()).width(2).height(RAIL_W).style(theme::underline(active));
    let tip = container(iced::widget::text(icon.name()).size(crate::tokens::TEXT_BODY)).padding([SPACE_1, SPACE_2]).style(|t: &Theme| theme::tooltip(t));
    let rail = row![bar, face];
    tooltip(rail, tip).position(tooltip::Position::Right).gap(SPACE_1).boxed()
}
