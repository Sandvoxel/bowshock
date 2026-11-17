//! Preferences > Units: the display unit set, the same setting as the top bar's select. A named set is one choice; below it each
//! quantity can be given its own unit, which makes the set custom (the select then reads `Custom`).

use iced::widget::{column, container};
use iced::{Element, Widget};

use super::{group, Message};
use crate::app::Ctx;
use crate::chrome::{unit_rows, units_select};
use crate::tokens::SPACE_3;

/// The page body.
pub fn view(ctx: &Ctx) -> Element<'_, Message> {
    // The custom entry has nothing to open here: the per-quantity strips are on the page.
    let select = container(units_select(ctx.units, Message::Units, Message::Nothing)).width(220);
    column![
        group("UNIT SET", select.boxed(), Some("The same choice as the unit select in the top bar.")),
        group("PER QUANTITY", unit_rows(ctx.units, Message::Units), Some("Areas and roughness follow length; angles are degrees, time is seconds.")),
    ]
    .spacing(SPACE_3 + 4.0)
    .boxed()
}
