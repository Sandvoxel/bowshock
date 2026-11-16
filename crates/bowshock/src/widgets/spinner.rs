//! A small busy indicator: an `accent` arc turning over a faint ring, for a row that waits on the network. It reads the clock
//! when drawn, so whoever shows it keeps `iced::window::frames()` coming meanwhile (the app does while the motor picker is
//! busy).

use std::sync::LazyLock;
use std::time::Instant;

use iced::widget::canvas::{self, path::Arc, Frame, Geometry, Path, Stroke};
use iced::{mouse, Element, Radians, Rectangle, Renderer, Theme, Widget};

use crate::theme::tokens;

/// Turns per second.
const SPEED: f32 = 1.2;
/// Length of the arc, radians.
const SWEEP: f32 = 1.8;

/// When the first spinner was drawn: the clock the angle is read from.
static EPOCH: LazyLock<Instant> = LazyLock::new(Instant::now);

/// A spinner `size` px square.
pub fn spinner<'a, M: 'a>(size: f32) -> Element<'a, M> {
    canvas::Canvas::new(Spinner).width(size).height(size).boxed()
}

struct Spinner;

impl<M> canvas::Program<M> for Spinner {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = tokens(theme);
        let mut frame = Frame::new(renderer, bounds.size());
        let center = frame.center();
        let radius = (bounds.width.min(bounds.height) / 2.0 - 1.5).max(1.0);
        frame.stroke(&Path::circle(center, radius), Stroke::default().with_color(t.line_200).with_width(1.5));
        let start = EPOCH.elapsed().as_secs_f32() * SPEED * std::f32::consts::TAU;
        let arc = Path::new(|b| b.arc(Arc { center, radius, start_angle: Radians(start), end_angle: Radians(start + SWEEP) }));
        frame.stroke(&arc, Stroke::default().with_color(t.accent).with_width(1.5));
        vec![frame.into_geometry()]
    }
}
