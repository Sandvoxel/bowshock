//! WarningChip (design/components/WarningChip): a compact outlined status label, always icon + words, 22 px high, square,
//! a 1 px border and text in the state colour on a transparent fill. Messages read "what · by how much (limit)".
//!
//! ```ignore
//! chip(Severity::Err, "Unstable · 0.31 cal (min 1.0)").on_press(Message::SelectOffender)
//! ```

use iced::widget::{container, mouse_area, row, text};
use iced::{mouse, Element, Widget};

use super::glyph::{self, Mark, Tone};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_1, SPACE_2};

/// The state a [`Chip`] reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// A passing check (`ok`, check icon).
    Ok,
    /// A warning (`warn`, triangle icon).
    Warn,
    /// An error (`err`, cross icon).
    Err,
    /// Neutral context such as the flight condition (`ink-muted`, no icon).
    Info,
}

/// Height of a chip.
pub const CHIP_H: f32 = 22.0;

/// An outlined status chip; convert it with `Element::from` / `.into()`.
pub struct Chip<'a, M> {
    severity: Severity,
    label: String,
    on_press: Option<M>,
    mono: bool,
    _marker: std::marker::PhantomData<&'a ()>,
}

/// A chip of `severity` reading `label`.
pub fn chip<'a, M>(severity: Severity, label: impl Into<String>) -> Chip<'a, M> {
    Chip { severity, label: label.into(), on_press: None, mono: false, _marker: std::marker::PhantomData }
}

impl<'a, M> Chip<'a, M> {
    /// Makes the chip clickable (a warning selects the offending part and scrolls the inspector to the field).
    pub fn on_press(mut self, message: M) -> Self {
        self.on_press = Some(message);
        self
    }

    /// Sets the label in the mono face, for chips that are mostly numbers (`Mach 0.3 · AoA 0°`).
    pub fn mono(mut self) -> Self {
        self.mono = true;
        self
    }
}

impl<'a, M: Clone + 'a> From<Chip<'a, M>> for Element<'a, M> {
    fn from(c: Chip<'a, M>) -> Self {
        let (mark, tone, pick): (Option<Mark>, Tone, fn(&crate::tokens::Tokens) -> iced::Color) = match c.severity {
            Severity::Ok => (Some(Mark::Check), Tone::Ok, |t| t.ok),
            Severity::Warn => (Some(Mark::Warning), Tone::Warn, |t| t.warn),
            Severity::Err => (Some(Mark::Error), Tone::Err, |t| t.err),
            Severity::Info => (None, Tone::Muted, |t| t.line_200),
        };
        let mut label = text(c.label).size(12);
        if c.mono {
            label = label.font(MONO);
        }
        // Info chips read as context, so their text is muted while the outline stays a neutral rule.
        label = label.style(theme::colored(if c.severity == Severity::Info { |t| t.ink_muted } else { pick }));

        let mut parts: Vec<Element<'a, M>> = Vec::new();
        if let Some(mark) = mark {
            parts.push(glyph::icon(mark, tone, 12.0));
        }
        parts.push(label.boxed());
        let body = container(row(parts).spacing(SPACE_1 + 1.0).align_y(iced::Center))
            .padding([0.0, SPACE_2 - 1.0])
            .height(CHIP_H)
            .center_y(CHIP_H)
            .style(theme::outlined(pick))
            .boxed();
        match c.on_press {
            Some(message) => mouse_area(body).on_press(message).interaction(mouse::Interaction::Pointer).boxed(),
            None => body,
        }
    }
}
