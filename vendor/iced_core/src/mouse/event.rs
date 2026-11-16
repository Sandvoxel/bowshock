use crate::Point;

use super::Button;

/// A mouse event.
///
/// _**Note:** This type is largely incomplete! If you need to track
/// additional events, feel free to [open an issue] and share your use case!_
///
/// [open an issue]: https://github.com/iced-rs/iced/issues
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    /// The mouse cursor entered the window.
    CursorEntered,

    /// The mouse cursor left the window.
    CursorLeft,

    /// The mouse cursor was moved
    CursorMoved {
        /// The new position of the mouse cursor
        position: Point,
    },

    /// A mouse button was pressed.
    ButtonPressed(Button),

    /// A mouse button was released.
    ButtonReleased(Button),

    /// The mouse wheel was scrolled.
    WheelScrolled {
        /// The scroll movement.
        delta: ScrollDelta,
    },

    /// The trackpad was pinched (macOS; forwarded from winit's `PinchGesture`).
    ///
    /// Not part of upstream iced: added by the vendored `iced_core`/`iced_winit` (see `vendor/README.md`). The gesture
    /// belongs to the cursor's position, so widgets read where it is from the [`Cursor`](super::Cursor).
    PinchGesture {
        /// The change in magnification since the last event: positive as the fingers spread (zoom in), negative as they
        /// close. A factor of `1.0 + delta` per event is the usual way to apply it.
        delta: f32,

        /// Where the gesture is in its life.
        phase: GesturePhase,
    },
}

/// The stage of a multi-touch trackpad gesture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GesturePhase {
    /// The fingers went down: the gesture began.
    Started,

    /// The fingers moved.
    Moved,

    /// The fingers lifted (or the system cancelled the gesture): the gesture is over.
    Ended,
}

/// A scroll movement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScrollDelta {
    /// A line-based scroll movement
    Lines {
        /// The number of horizontal lines scrolled
        x: f32,

        /// The number of vertical lines scrolled
        y: f32,
    },
    /// A pixel-based scroll movement
    Pixels {
        /// The number of horizontal pixels scrolled
        x: f32,
        /// The number of vertical pixels scrolled
        y: f32,
    },
}
