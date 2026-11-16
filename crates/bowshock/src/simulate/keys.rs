//! Which modifier keys are held, for the Flights list: a button press carries no modifiers, so Shift-click and Cmd/Ctrl-click
//! need the screen to know what is held when the click lands. [`track`] is an invisible zero-size widget that reports every
//! change of the held modifiers (and a lost window focus, which can swallow the release of a key) as a message.

use iced::advanced::layout::{self, Layout};
use iced::advanced::renderer;
use iced::advanced::widget::{Meta, Tree};
use iced::advanced::{Shell, Widget};
use iced::{keyboard, mouse, window, Element, Event, Length, Rectangle, Size, Theme};

/// The invisible tracker: `held` is what the screen believes is held, `on_change` makes the message for a new set.
struct Track<'a, M> {
    held: keyboard::Modifiers,
    on_change: Box<dyn Fn(keyboard::Modifiers) -> M + 'a>,
}

impl<M> Meta for Track<'_, M> {}

impl<M, R: renderer::Renderer> Widget<M, Theme, R> for Track<'_, M> {
    fn size(&self) -> Size<Length> {
        Size::new(Length::Fixed(0.0), Length::Fixed(0.0))
    }

    fn layout(&mut self, tree: &mut Tree, _: &R, limits: &layout::Limits) {
        tree.size = layout::atomic(limits, Length::Fixed(0.0), Length::Fixed(0.0));
    }

    fn draw(&self, _: &Tree, _: &mut R, _: &Theme, _: &renderer::Style, _: Layout, _: mouse::Cursor, _: &Rectangle) {}

    fn update(&mut self, _: &mut Tree, event: &Event, _: Layout, _: mouse::Cursor, _: &R, shell: &mut Shell<'_, M>, _: &Rectangle) {
        let now = match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(m)) => *m,
            Event::Keyboard(keyboard::Event::KeyPressed { modifiers, .. } | keyboard::Event::KeyReleased { modifiers, .. }) => *modifiers,
            Event::Window(window::Event::Unfocused) => keyboard::Modifiers::empty(),
            _ => return,
        };
        if now != self.held {
            shell.publish((self.on_change)(now));
        }
    }
}

/// An invisible element that sends `on_change(modifiers)` whenever the held modifier keys differ from `held`.
pub(super) fn track<'a, M: 'a>(held: keyboard::Modifiers, on_change: impl Fn(keyboard::Modifiers) -> M + 'a) -> Element<'a, M> {
    iced::Widget::boxed(Track { held, on_change: Box::new(on_change) })
}
