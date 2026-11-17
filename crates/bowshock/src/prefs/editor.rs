//! Preferences > Editor: how a part is put down in the Build screen. Drag and drop (the default) ends a carry when the button is
//! released; click to place keeps the part in hand after you let go and puts it down on the next click (the Kerbal Space Program
//! editor's way). Either applies to every part, from the parts bin or picked up off the rocket, in the viewport and Station Lanes.

use iced::Element;

use super::{card, group, setting, Message};
use crate::app::Ctx;
use crate::widgets::segmented::segmented;

/// What the setting's note says.
pub const PLACING_NOTE: &str = "Click to place keeps a part in hand after you let go; click to put it down, Esc to put it back.";

/// The page body.
pub fn view(ctx: &Ctx) -> Element<'_, Message> {
    let control = segmented(&[(false, "Drag and drop"), (true, "Click to place")], ctx.click_to_place, Message::Placing);
    group("BUILD", card(vec![setting("Placing parts", Some(PLACING_NOTE), control)]), None)
}
