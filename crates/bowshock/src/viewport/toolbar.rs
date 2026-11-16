//! The view toolbar's pieces (design/components/BuilderScreen, SegmentedControl): one framed strip holding the 3D / 2D cells
//! (each with a small drawn icon), the inside views and the snap step as a flat magnet cell, split by hairline rules.

use iced::widget::canvas::{self, Frame, Geometry, Path, Stroke};
use iced::widget::{button, column, container, pick_list, row, text, Space};
use iced::{alignment, mouse, Background, Border, Element, Length, Point, Rectangle, Renderer, Theme, Widget};

use crate::theme::{self, SANS_MEDIUM};
use crate::tokens::{CONTROL_H, SPACE_1, SPACE_3};

/// Label size of a cell (the same as every SegmentedControl).
const TEXT: f32 = 12.0;
/// Side of an icon.
const ICON: f32 = 14.0;

/// The small pictures on the toolbar's cells.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Glyph {
    /// An isometric box: the perspective view.
    Cube,
    /// The rocket's side silhouette: the profile view.
    Profile,
    /// A horseshoe magnet: the snap step.
    Magnet,
}

struct Icon {
    glyph: Glyph,
    /// Drawn in `ink` (the active cell) rather than `ink-muted`.
    active: bool,
}

impl<M> canvas::Program<M> for Icon {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let mut frame = Frame::new(renderer, bounds.size());
        let colour = if self.active { t.ink } else { t.ink_muted };
        let stroke = Stroke::default().with_color(colour).with_width(1.2);
        let p = |x: f32, y: f32| Point::new(x, y);
        match self.glyph {
            Glyph::Cube => {
                // A hexagon outline with the three edges meeting at the near corner.
                let hex = Path::new(|b| {
                    b.move_to(p(7.0, 1.0));
                    b.line_to(p(12.5, 4.0));
                    b.line_to(p(12.5, 10.0));
                    b.line_to(p(7.0, 13.0));
                    b.line_to(p(1.5, 10.0));
                    b.line_to(p(1.5, 4.0));
                    b.close();
                });
                frame.stroke(&hex, stroke);
                frame.stroke(
                    &Path::new(|b| {
                        b.move_to(p(1.5, 4.0));
                        b.line_to(p(7.0, 7.0));
                        b.line_to(p(12.5, 4.0));
                        b.move_to(p(7.0, 7.0));
                        b.line_to(p(7.0, 13.0));
                    }),
                    stroke,
                );
            }
            Glyph::Profile => {
                // A nose cone and a body with one fin, side-on.
                frame.stroke(
                    &Path::new(|b| {
                        b.move_to(p(1.0, 7.0));
                        b.quadratic_curve_to(p(3.5, 4.0), p(6.5, 4.0));
                        b.line_to(p(11.0, 4.0));
                        b.line_to(p(11.0, 10.0));
                        b.line_to(p(6.5, 10.0));
                        b.quadratic_curve_to(p(3.5, 10.0), p(1.0, 7.0));
                        b.move_to(p(9.0, 4.0));
                        b.line_to(p(11.5, 1.5));
                        b.line_to(p(12.8, 1.5));
                        b.line_to(p(12.8, 4.0));
                    }),
                    stroke,
                );
            }
            Glyph::Magnet => {
                // A horseshoe: an arch with two feet, the tips marked.
                frame.stroke(
                    &Path::new(|b| {
                        b.move_to(p(3.0, 12.5));
                        b.line_to(p(3.0, 6.0));
                        b.arc_to(p(3.0, 1.5), p(7.0, 1.5), 4.0);
                        b.arc_to(p(11.0, 1.5), p(11.0, 6.0), 4.0);
                        b.line_to(p(11.0, 12.5));
                    }),
                    Stroke::default().with_color(colour).with_width(2.0),
                );
                frame.stroke(&Path::line(p(2.0, 10.5), p(4.0, 10.5)), Stroke::default().with_color(t.accent).with_width(1.6));
                frame.stroke(&Path::line(p(10.0, 10.5), p(12.0, 10.5)), Stroke::default().with_color(t.accent).with_width(1.6));
            }
        }
        vec![frame.into_geometry()]
    }
}

/// The icon on its own, for cells that are not mutually exclusive options (the snap step).
pub fn icon<'a, M: 'a>(glyph: Glyph, active: bool) -> Element<'a, M> {
    iced::widget::canvas(Icon { glyph, active }).width(ICON).height(ICON).boxed()
}

/// The strip's frame: `bg-100` inside one 1 px `line-200` frame.
pub fn frame<'a, M: 'a>(cells: Vec<Element<'a, M>>) -> Element<'a, M> {
    container(row(cells).align_y(alignment::Vertical::Center)).padding(1).style(|theme: &Theme| theme::segmented_frame(theme)).boxed()
}

/// A hairline rule between two groups of cells.
pub fn rule<'a, M: 'a>() -> Element<'a, M> {
    container(Space::new()).width(1).height(CONTROL_H - 2.0).style(theme::hairline).boxed()
}

/// The cells of one group of mutually exclusive options (each with an optional icon), split by hairlines, the active one with
/// its 2 px accent underline. Not framed: [`frame`] holds the whole strip.
pub fn group<'a, T, M>(options: &[(T, Option<Glyph>, &'a str)], selected: T, on_select: impl Fn(T) -> M) -> Vec<Element<'a, M>>
where
    T: Copy + PartialEq + 'a,
    M: Clone + 'a,
{
    let inner_h = CONTROL_H - 2.0;
    let mut cells: Vec<Element<'a, M>> = Vec::with_capacity(options.len() * 2);
    for (i, &(value, glyph, name)) in options.iter().enumerate() {
        if i > 0 {
            cells.push(container(Space::new()).width(1).height(inner_h).style(theme::hairline).boxed());
        }
        let active = value == selected;
        let label = text(name).size(TEXT).font(SANS_MEDIUM);
        let content: Element<'a, M> = match glyph {
            Some(glyph) => row![icon(glyph, active), label].spacing(SPACE_1 + 2.0).align_y(alignment::Vertical::Center).boxed(),
            None => label.boxed(),
        };
        cells.push(
            column![
                button(content)
                    .width(Length::Shrink)
                    .height(inner_h - 2.0)
                    .padding([0.0, SPACE_3 - 2.0])
                    .style(theme::segment(active))
                    .on_press(on_select(value)),
                container(Space::new()).width(Length::Fill).height(2).style(theme::underline(active)),
            ]
            .width(Length::Shrink)
            .boxed(),
        );
    }
    cells
}

/// The snap step as a flat cell of the strip: the choice and its caret, no frame of its own, hot (`bg-400`) under the cursor.
pub fn flat_select<'a, T, L, M>(options: L, selected: T, on_select: impl Fn(T) -> M + 'a) -> Element<'a, M>
where
    T: ToString + PartialEq + Clone + 'a,
    L: std::borrow::Borrow<[T]> + 'a,
    M: Clone + 'a,
{
    pick_list(Some(selected), options, |item: &T| item.to_string())
        .on_select(on_select)
        .text_size(TEXT)
        .padding([0.0, SPACE_1 + 2.0])
        .style(|theme: &Theme, status| {
            let t = theme::tokens(theme);
            let hot = matches!(status, pick_list::Status::Hovered | pick_list::Status::Opened { .. });
            pick_list::Style {
                text_color: t.ink,
                placeholder_color: t.ink_faint,
                handle_color: t.ink_muted,
                background: Background::Color(if hot { t.bg_400 } else { t.bg_100 }),
                border: Border::default(),
            }
        })
        .menu_style(theme::select_menu)
        .boxed()
}
