//! Button (design/components/Button): primary, secondary and ghost weights plus the square icon-only button. 26 px high,
//! square corners, 13 px Plex Sans 500 labels, an optional framed mono key (`F5`). Icon-only buttons always carry a
//! tooltip with their shortcut. A disabled button is never hidden: it drops its fill, gets a dashed border, and its tooltip
//! says why.
//!
//! ```ignore
//! button("Simulate", Weight::Primary).icon(Mark::Play).key("F5").on_press(Message::Simulate)
//! icon_button(Mark::Undo, "Undo").shortcut("Ctrl+Z").on_press(Message::Undo)
//! button("Export…", Weight::Secondary).disabled("Run a simulation first")
//! ```

use iced::widget::{self, container, row, stack, text, tooltip};
use iced::{Element, Length, Theme, Widget};

use super::dashed;
use super::glyph::{self, Mark, Tone};
use crate::theme::{self, MONO, SANS_MEDIUM};
use crate::tokens::{CONTROL_H, SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};

/// Visual weight of a [`Button`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Weight {
    /// `accent` fill: the one commit action of a region.
    Primary,
    /// `bg-300` with a `line-200` border: secondary commands.
    Secondary,
    /// No border until hovered: toolbars, undo/redo, in-panel actions.
    Ghost,
    /// `bg-300` with an `err` label and border: a command that throws work away.
    Destructive,
}

/// A push button. Build it with [`button`] or [`icon_button`], then convert it with `Element::from` / `.into()`.
pub struct Button<'a, M> {
    label: Option<&'a str>,
    mark: Option<Mark>,
    key: Option<&'a str>,
    shortcut: Option<&'a str>,
    tip: Option<std::borrow::Cow<'a, str>>,
    reason: Option<&'a str>,
    weight: Weight,
    on_press: Option<M>,
    width: Length,
    small: bool,
    preview: Option<widget::button::Status>,
}

/// A labelled button of the given `weight`.
pub fn button<'a, M>(label: &'a str, weight: Weight) -> Button<'a, M> {
    Button {
        label: Some(label),
        mark: None,
        key: None,
        shortcut: None,
        tip: None,
        reason: None,
        weight,
        on_press: None,
        width: Length::Shrink,
        small: false,
        preview: None,
    }
}

/// A square icon-only ghost button; `tooltip` names the command (add `.shortcut(..)`).
pub fn icon_button<'a, M>(mark: Mark, tooltip: impl Into<std::borrow::Cow<'a, str>>) -> Button<'a, M> {
    Button {
        label: None,
        mark: Some(mark),
        key: None,
        shortcut: None,
        tip: Some(tooltip.into()),
        reason: None,
        weight: Weight::Ghost,
        on_press: None,
        width: Length::Fixed(CONTROL_H),
        small: false,
        preview: None,
    }
}

impl<'a, M> Button<'a, M> {
    /// The message published on click. Without one the button is inert (and drawn disabled).
    pub fn on_press(mut self, message: M) -> Self {
        self.on_press = Some(message);
        self
    }

    /// An icon before the label.
    pub fn icon(mut self, mark: Mark) -> Self {
        self.mark = Some(mark);
        self
    }

    /// A framed mono key inside the button, after the label (`F5`).
    pub fn key(mut self, key: &'a str) -> Self {
        self.key = Some(key);
        self
    }

    /// The shortcut shown in the tooltip (`Ctrl+Z`).
    pub fn shortcut(mut self, shortcut: &'a str) -> Self {
        self.shortcut = Some(shortcut);
        self
    }

    /// A tooltip for a labelled button.
    pub fn tooltip(mut self, tip: impl Into<std::borrow::Cow<'a, str>>) -> Self {
        self.tip = Some(tip.into());
        self
    }

    /// Disables the button; `reason` is what its tooltip says ("Run a simulation first").
    pub fn disabled(mut self, reason: &'a str) -> Self {
        self.reason = Some(reason);
        self
    }

    /// A compact 22 px button with a 12 px label, for in-panel actions (`Detach`, `Split all`).
    pub fn small(mut self) -> Self {
        self.small = true;
        self
    }

    /// Draws the button in a fixed interaction state whatever the cursor does: the kit's static *Hover* sample, and what a
    /// headless screenshot (which has no cursor) can show.
    pub fn preview(mut self, status: widget::button::Status) -> Self {
        self.preview = Some(status);
        self
    }

    /// The button's width (default: shrink to its content; icon buttons are `control-h` square).
    pub fn width(mut self, width: impl Into<Length>) -> Self {
        self.width = width.into();
        self
    }
}

impl<'a, M: Clone + 'a> From<Button<'a, M>> for Element<'a, M> {
    fn from(b: Button<'a, M>) -> Self {
        let disabled = b.reason.is_some() || b.on_press.is_none();
        let tone = match (disabled, b.weight) {
            (true, _) => Tone::Faint,
            (false, Weight::Primary) => Tone::AccentInk,
            (false, Weight::Secondary) => Tone::Ink,
            (false, Weight::Ghost) => Tone::Muted,
            (false, Weight::Destructive) => Tone::Err,
        };

        let mut content: Vec<Element<'a, M>> = Vec::new();
        if let Some(mark) = b.mark {
            content.push(glyph::icon(mark, tone, if b.label.is_some() { 12.0 } else { 14.0 }));
        }
        if let Some(label) = b.label {
            content.push(text(label).font(SANS_MEDIUM).size(if b.small { 12.0 } else { TEXT_BODY }).wrapping(text::Wrapping::None).boxed());
        }
        if let Some(key) = b.key {
            let style: fn(&Theme) -> iced::widget::container::Style =
                if b.weight == Weight::Primary && !disabled { theme::keycap_on_accent } else { theme::keycap };
            content.push(container(text(key).font(MONO).size(TEXT_LABEL)).padding([0.0, SPACE_1]).style(style).boxed());
        }
        let height = if b.small { 22.0 } else { CONTROL_H };
        let inner = container(row(content).spacing(SPACE_2).align_y(iced::Center)).center_y(height);
        let padding = if b.label.is_some() { [0.0, if b.small { SPACE_3 - 2.0 } else { SPACE_3 }] } else { [0.0, 0.0] };
        let inner = if b.label.is_none() { container(inner).center_x(Length::Fill).boxed() } else { inner.boxed() };

        let mut press = widget::button(inner).width(b.width).height(height).padding(padding);
        let preview = b.preview;
        press = match b.weight {
            Weight::Primary => press.style(move |t, s| theme::primary(t, preview.unwrap_or(s))),
            Weight::Secondary => press.style(move |t, s| theme::secondary(t, preview.unwrap_or(s))),
            Weight::Ghost => press.style(move |t, s| theme::ghost(t, preview.unwrap_or(s))),
            Weight::Destructive => press.style(move |t, s| theme::destructive(t, preview.unwrap_or(s))),
        };
        if !disabled {
            press = press.on_press_maybe(b.on_press);
        }

        let mut body: Element<'a, M> = if disabled && b.label.is_some() { stack![press, dashed::frame(|t| t.line_200)].boxed() } else { press.boxed() };

        let tip = if let Some(reason) = b.reason { Some((reason.into(), None)) } else { b.tip.map(|tip| (tip, b.shortcut)) };
        if let Some((tip, shortcut)) = tip {
            let mut parts: Vec<Element<'a, M>> = vec![text(tip).size(TEXT_BODY).boxed()];
            if let Some(shortcut) = shortcut {
                parts.push(container(text(shortcut).font(MONO).size(TEXT_LABEL)).padding([0.0, SPACE_1]).style(theme::keycap).boxed());
            }
            let bubble = container(row(parts).spacing(SPACE_2).align_y(iced::Center)).padding([SPACE_1, SPACE_2]).style(|t: &Theme| theme::tooltip(t));
            body = tooltip(body, bubble).position(tooltip::Position::Bottom).gap(SPACE_1).boxed();
        }
        body
    }
}
