//! Scrolls the inspector so a field is in view (a warning that names the field to fix: WarningChip). A widget operation finds
//! where the field sits in the scrolled content, then the inspector's scrollable is moved there.

use iced::advanced::widget::operation::{Animation, Operation, Outcome};
use iced::advanced::widget::{operate, Id};
use iced::widget::operation::scrollable::{scroll_to, AbsoluteOffset};
use iced::{Rectangle, Task};

/// The id of the inspector's scrollable.
pub const SCROLL_ID: &str = "inspector-scroll";

/// Room kept above a revealed field, px.
const MARGIN: f32 = 72.0;

/// Finds a focusable by id and reports its distance from the top of the scrollable's content.
struct Locate {
    target: Id,
    content_top: f32,
    found: Option<f32>,
}

impl Operation<f32> for Locate {
    fn traverse(&mut self, operate: &mut dyn FnMut(&mut dyn Operation<f32>)) {
        operate(self);
    }

    fn container(&mut self, id: Option<&Id>, bounds: Rectangle, _viewport: &Rectangle) {
        if id.is_some_and(|id| *id == Id::new(SCROLL_ID)) {
            self.content_top = bounds.y;
        }
    }

    fn focusable(&mut self, id: Option<&Id>, bounds: Rectangle, _state: &mut dyn iced::advanced::widget::operation::Focusable) {
        if id == Some(&self.target) {
            self.found = Some(bounds.y - self.content_top);
        }
    }

    fn finish(&self) -> Outcome<f32> {
        self.found.map_or(Outcome::None, Outcome::Some)
    }
}

/// A task that scrolls the inspector so the field with widget id `key` sits near the top (nothing if it is not on screen).
pub fn reveal<T: Send + 'static>(key: String) -> Task<T> {
    let find = operate(Locate { target: Id::from(key), content_top: 0.0, found: None });
    find.then(|y| scroll_to(SCROLL_ID, AbsoluteOffset { x: None, y: Some((y - MARGIN).max(0.0)) }, Animation::Instant))
}
