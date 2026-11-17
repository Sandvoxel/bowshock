//! Preferences > Appearance: the theme. The sun / moon button in the top bar changes the same setting.

use iced::widget::container;
use iced::{Element, Widget};

use super::{group, Message};
use crate::app::Ctx;
use crate::widgets::segmented::segmented;

/// The page body.
pub fn view(ctx: &Ctx) -> Element<'_, Message> {
    let theme = container(segmented(&[(true, "Dark"), (false, "Light")], ctx.dark, Message::Theme)).width(220);
    group("THEME", theme.boxed(), Some("Also switched by the sun / moon button in the top bar. Kept for every rocket and every run."))
}
