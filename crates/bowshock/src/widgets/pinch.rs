//! Trackpad pinch (macOS), shared by every surface that zooms about the cursor: the viewport, Station Lanes, the plots and the
//! ground track.
//!
//! The pinned iced drops winit's `PinchGesture`; the vendored `iced_core`/`iced_winit` (see `vendor/README.md`) forward it as
//! [`iced::mouse::Event::PinchGesture`]. Its `delta` is the change in magnification since the previous event, small and
//! frequent (about every frame), so applying it directly is already smooth: no animation is layered on top, which would only
//! add lag between the fingers and the picture.

/// The stage of a pinch gesture. `iced::mouse` does not re-export it (only `iced::advanced::mouse` does), so the app reaches it
/// from here.
pub use iced::advanced::mouse::GesturePhase as Phase;

/// How much one unit of pinch magnification zooms: the zoom factor of a whole gesture is `exp(GAIN * total delta)`. One keeps
/// the picture under the fingers at the scale the trackpad reports (a pinch that doubles the gap doubles the zoom, to first
/// order), like every other Mac app.
pub const GAIN: f32 = 1.0;

/// The most one event may zoom by, either way. A lost frame or a bad device report must not throw the view across the scale
/// range.
const MAX_STEP: f32 = 0.5;

/// The zoom factor (greater than 1 zooms in) of a pinch event of magnification change `delta`. Factors multiply as deltas
/// add, so the result does not depend on how a gesture was cut into events.
pub fn factor(delta: f32) -> f32 {
    (delta.clamp(-MAX_STEP, MAX_STEP) * GAIN).exp()
}

/// The factor for the view's span instead of its scale (a plot's x-range narrows as the fingers spread).
pub fn span_factor(delta: f32) -> f32 {
    1.0 / factor(delta)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spreading_zooms_in_closing_zooms_out_and_rest_is_neutral() {
        assert!(factor(0.05) > 1.0 && factor(-0.05) < 1.0);
        assert_eq!(factor(0.0), 1.0);
        assert!(span_factor(0.05) < 1.0 && span_factor(-0.05) > 1.0);
    }

    #[test]
    fn a_gesture_zooms_the_same_however_its_events_are_cut() {
        let whole = factor(0.3);
        let in_ten: f32 = (0..10).map(|_| factor(0.03)).product();
        assert!((whole - in_ten).abs() < 1e-5, "{whole} vs {in_ten}");
        // Spread and close again lands where it started.
        assert!((factor(0.2) * factor(-0.2) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn one_event_cannot_zoom_past_the_cap() {
        assert_eq!(factor(40.0), factor(MAX_STEP));
        assert_eq!(factor(-40.0), factor(-MAX_STEP));
        assert!(factor(f32::MAX).is_finite() && factor(f32::MIN) > 0.0);
    }
}
