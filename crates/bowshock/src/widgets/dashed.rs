//! Dashed 1 px frames. iced borders are solid, so the kit's dashed looks (disabled buttons, `auto` fields, pick targets,
//! linked-sibling brackets) are drawn as rows of small quads, either directly from a widget's `draw` with [`draw`] or as
//! a [`Frame`] layered over another element in a `stack!`.

use iced::advanced::layout;
use iced::advanced::renderer;
use iced::advanced::widget::{Meta, Tree};
use iced::advanced::{Layout, Widget};
use iced::{mouse, Background, Color, Element, Length, Rectangle, Size, Theme};

use crate::theme::tokens;
use crate::tokens::Tokens;

/// Length of a dash and of the gap after it, in logical pixels.
pub const DASH: f32 = 3.0;
/// The gap between dashes.
pub const GAP: f32 = 2.0;

/// Draws a 1 px dashed rectangle outline inside `bounds`.
pub fn draw<R: renderer::Renderer>(renderer: &mut R, bounds: Rectangle, color: Color) {
    let mut quad = |x: f32, y: f32, w: f32, h: f32| {
        renderer
            .fill_quad(renderer::Quad { bounds: Rectangle { x, y, width: w, height: h }, snap: true, ..renderer::Quad::default() }, Background::Color(color));
    };
    let (x0, y0) = (bounds.x, bounds.y);
    let (x1, y1) = (bounds.x + bounds.width, bounds.y + bounds.height);

    let mut x = x0;
    while x < x1 {
        let w = DASH.min(x1 - x);
        quad(x, y0, w, 1.0);
        quad(x, y1 - 1.0, w, 1.0);
        x += DASH + GAP;
    }
    let mut y = y0 + 1.0;
    while y < y1 - 1.0 {
        let h = DASH.min(y1 - 1.0 - y);
        quad(x0, y, 1.0, h);
        quad(x1 - 1.0, y, 1.0, h);
        y += DASH + GAP;
    }
}

/// A transparent element that draws a dashed frame over its whole area; layer it over a control with `stack!`.
///
/// ```ignore
/// stack![button, dashed::frame(|t| t.line_200)]
/// ```
pub struct Frame {
    pick: fn(&Tokens) -> Color,
}

impl Frame {
    /// A dashed frame in the token colour chosen by `pick`.
    pub fn new(pick: fn(&Tokens) -> Color) -> Self {
        Self { pick }
    }
}

impl Meta for Frame {}

impl<M, R: renderer::Renderer> Widget<M, Theme, R> for Frame {
    fn size(&self) -> Size<Length> {
        Size::new(Length::Fill, Length::Fill)
    }

    fn layout(&mut self, tree: &mut Tree, _: &R, limits: &layout::Limits) {
        tree.size = layout::atomic(limits, Length::Fill, Length::Fill);
    }

    fn draw(&self, _: &Tree, renderer: &mut R, theme: &Theme, _: &renderer::Style, layout: Layout, _: mouse::Cursor, _: &Rectangle) {
        draw(renderer, layout.bounds(), (self.pick)(tokens(theme)));
    }
}

/// [`Frame`] as an element.
pub fn frame<'a, M: 'a>(pick: fn(&Tokens) -> Color) -> Element<'a, M> {
    iced::Widget::boxed(Frame::new(pick))
}
