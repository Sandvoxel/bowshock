//! The shared modal card and full-window scrim.

use iced::widget::{column, container, mouse_area, text, Space, Stack};
use iced::{Background, Color, Element, Fill, Widget};

use crate::theme::{self, SANS_MEDIUM};
use crate::tokens::SPACE_3;

/// A modal card (`title`, `body`, a row of `buttons`) centred over a dimmed backdrop that takes every click and hides the cursor
/// from what lies beneath (so hovering the page does not reach its hover handlers). `on_card_press` is what a click inside the
/// card sends, which keeps it from reaching the backdrop; `on_backdrop_press` is what a click outside sends.
pub fn modal<'a, M: Clone + 'a>(
    title: impl text::IntoFragment<'a>,
    body: Element<'a, M>,
    buttons: Element<'a, M>,
    width: f32,
    on_card_press: M,
    on_backdrop_press: M,
) -> Element<'a, M> {
    let backdrop = container(Space::new().width(Fill).height(Fill))
        .style(|_| container::Style { background: Some(Background::Color(Color::from_rgba(0.0, 0.0, 0.0, 0.55))), ..container::Style::default() });
    let heading = text(title).font(SANS_MEDIUM).size(15);
    let card = container(mouse_area(column![heading, body, buttons].spacing(SPACE_3 + 2.0)).on_press(on_card_press))
        .width(width)
        .padding(SPACE_3 + 8.0)
        .style(theme::overlay);
    let scrim = mouse_area(backdrop).on_press(on_backdrop_press).interaction(iced::mouse::Interaction::Idle);
    Stack::with_children([scrim.boxed(), container(card).center(Fill).boxed()]).boxed()
}

/// An invisible full-window layer that takes every click and hides the cursor from what lies beneath.
pub fn scrim<'a, M: Clone + 'a>(on_press: M) -> Element<'a, M> {
    mouse_area(Space::new().width(Fill).height(Fill)).interaction(iced::mouse::Interaction::Idle).on_press(on_press).boxed()
}
