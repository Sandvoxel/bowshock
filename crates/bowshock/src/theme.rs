//! Bowshock themes and widget styles. Every style reads its colours from [`tokens`] of the active theme; corners are square
//! and nothing has a shadow (design/README.md, "Hard rules").

use iced::theme::palette::Seed;
use iced::widget::{button, checkbox, container, pane_grid, text_input};
use iced::{font, Background, Border, Color, Font, Theme};

use crate::tokens::{self, Tokens};

pub const SANS: Font = Font::new("IBM Plex Sans");
pub const MONO: Font = Font::new("IBM Plex Mono");
pub const SANS_SEMIBOLD: Font = SANS.weight(font::Weight::Semibold);
pub const SANS_MEDIUM: Font = SANS.weight(font::Weight::Medium);

pub const FONTS: [&[u8]; 6] = [
    include_bytes!("../assets/fonts/IBMPlexSans-Regular.ttf"),
    include_bytes!("../assets/fonts/IBMPlexSans-Medium.ttf"),
    include_bytes!("../assets/fonts/IBMPlexSans-SemiBold.ttf"),
    include_bytes!("../assets/fonts/IBMPlexMono-Regular.ttf"),
    include_bytes!("../assets/fonts/IBMPlexMono-Medium.ttf"),
    include_bytes!("../assets/fonts/IBMPlexMono-SemiBold.ttf"),
];

fn seed(t: &Tokens) -> Seed {
    Seed { background: t.bg_100, text: t.ink, primary: t.accent, success: t.ok, warning: t.warn, danger: t.err }
}

pub fn dark() -> Theme {
    Theme::custom("Bowshock Dark", seed(&tokens::DARK))
}

pub fn light() -> Theme {
    Theme::custom("Bowshock Light", seed(&tokens::LIGHT))
}

/// The tokens of `theme`: light for a light palette, dark otherwise.
pub fn tokens(theme: &Theme) -> &'static Tokens {
    if theme.palette().is_dark {
        &tokens::DARK
    } else {
        &tokens::LIGHT
    }
}

fn frame(color: Color) -> Border {
    Border { color, width: 1.0, radius: 0.0.into() }
}

fn filled(bg: Color, border: Color) -> container::Style {
    container::Style { background: Some(Background::Color(bg)), border: frame(border), ..container::Style::default() }
}

/// Top bar and status strip: `bg-100`, framed by `line-100`.
pub fn bar(theme: &Theme) -> container::Style {
    let t = tokens(theme);
    container::Style { text_color: Some(t.ink), ..filled(t.bg_100, t.line_100) }
}

/// Side panels (parts bin, inspector): `bg-200`.
pub fn panel(theme: &Theme) -> container::Style {
    let t = tokens(theme);
    container::Style { text_color: Some(t.ink), ..filled(t.bg_200, t.line_100) }
}

/// The viewport ground and Station Lanes: `bg-000`, the deepest surface.
pub fn ground(theme: &Theme) -> container::Style {
    let t = tokens(theme);
    container::Style { text_color: Some(t.ink), ..filled(t.bg_000, t.line_100) }
}

/// A floating viewport overlay: `bg-200` in a 1 px `line-200` frame.
pub fn overlay(theme: &Theme) -> container::Style {
    let t = tokens(theme);
    container::Style { text_color: Some(t.ink), ..filled(t.bg_200, t.line_200) }
}

/// One ruled cell of the status strip (a `line-100` rule on its right is drawn by the frame).
pub fn cell(theme: &Theme) -> container::Style {
    let t = tokens(theme);
    container::Style { border: frame(t.line_100), ..container::Style::default() }
}

pub fn panes(theme: &Theme) -> pane_grid::Style {
    let t = tokens(theme);
    let line = pane_grid::Line { color: t.accent, width: 1.0 };
    pane_grid::Style {
        hovered_region: pane_grid::Highlight { background: Background::Color(t.accent_soft), border: frame(t.accent) },
        picked_split: line,
        hovered_split: line,
    }
}

/// The one commit action per region: `accent` fill, `accent-ink` text.
pub fn primary(theme: &Theme, status: button::Status) -> button::Style {
    let t = tokens(theme);
    let style = button::Style { background: Some(Background::Color(t.accent)), text_color: t.accent_ink, border: frame(t.accent), ..button::Style::default() };
    match status {
        button::Status::Hovered | button::Status::Pressed => button::Style { background: Some(Background::Color(t.accent.scale_alpha(0.85))), ..style },
        button::Status::Disabled => disabled(t),
        button::Status::Active => style,
    }
}

/// Toolbar and in-panel actions: no border until hovered.
pub fn ghost(theme: &Theme, status: button::Status) -> button::Style {
    let t = tokens(theme);
    match status {
        button::Status::Active => button::Style { text_color: t.ink_muted, ..button::Style::default() },
        button::Status::Hovered | button::Status::Pressed => {
            button::Style { background: Some(Background::Color(t.bg_400)), text_color: t.ink, border: frame(t.line_200), ..button::Style::default() }
        }
        button::Status::Disabled => disabled(t),
    }
}

fn disabled(t: &Tokens) -> button::Style {
    // The design asks for a dashed border; iced borders are solid, so a faint frame stands in.
    button::Style { text_color: t.ink_faint, border: frame(t.line_100), ..button::Style::default() }
}

/// One segment of a segmented control. The active one is `bg-300` (its accent underline is drawn by the caller).
pub fn segment(active: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |theme, status| {
        let t = tokens(theme);
        let bg = match (active, status) {
            (true, _) => t.bg_300,
            (false, button::Status::Hovered | button::Status::Pressed) => t.bg_400,
            (false, _) => t.bg_100,
        };
        button::Style {
            background: Some(Background::Color(bg)),
            text_color: if active { t.ink } else { t.ink_muted },
            border: Border { radius: 0.0.into(), ..Border::default() },
            ..button::Style::default()
        }
    }
}

/// A segmented control's strip: `bg-100` inside a 1 px `line-200` frame.
pub fn segmented_frame(theme: &Theme) -> container::Style {
    let t = tokens(theme);
    filled(t.bg_100, t.line_200)
}

/// The 2 px `accent` underline under the active segment (transparent under the others).
pub fn underline(active: bool) -> impl Fn(&Theme) -> container::Style {
    move |theme| {
        let t = tokens(theme);
        let c = if active { t.accent } else { Color::TRANSPARENT };
        container::Style { background: Some(Background::Color(c)), ..container::Style::default() }
    }
}

/// A text field: `bg-300` in a 1 px `line-100` frame, `accent` when focused.
pub fn input(theme: &Theme, status: text_input::Status) -> text_input::Style {
    let t = tokens(theme);
    let border = match status {
        text_input::Status::Focused { .. } => t.accent,
        text_input::Status::Hovered => t.line_200,
        text_input::Status::Active | text_input::Status::Disabled => t.line_100,
    };
    text_input::Style { background: Background::Color(t.bg_300), border: frame(border), placeholder: t.ink_faint, value: t.ink, selection: t.accent_soft }
}

/// A checkbox: a square frame, filled `accent` with an `accent-ink` tick when on.
pub fn check(theme: &Theme, status: checkbox::Status) -> checkbox::Style {
    let t = tokens(theme);
    let (checked, hovered) = match status {
        checkbox::Status::Active { is_checked } | checkbox::Status::Disabled { is_checked } => (is_checked, false),
        checkbox::Status::Hovered { is_checked } => (is_checked, true),
    };
    let fill = if checked { t.accent } else { t.bg_300 };
    let edge = if checked {
        t.accent
    } else if hovered {
        t.ink_muted
    } else {
        t.line_200
    };
    checkbox::Style { background: Background::Color(fill), icon_color: t.accent_ink, border: frame(edge), text_color: Some(t.ink_muted) }
}

pub const PAD_PANEL: f32 = tokens::SPACE_3;
