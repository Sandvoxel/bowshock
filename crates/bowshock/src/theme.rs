//! Bowshock themes and widget styles. Every style reads its colours from [`tokens`] of the active theme; corners are square
//! and nothing has a shadow (design/README.md, "Hard rules").

use iced::theme::palette::Seed;
use iced::overlay::menu;
use iced::widget::{button, checkbox, container, pane_grid, pick_list, text, text_input};
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

/// A 1 px square border.
pub fn frame(color: Color) -> Border {
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

/// Text in a token colour: `text(..).style(theme::colored(|t| t.ink_muted))`.
pub fn colored(pick: fn(&Tokens) -> Color) -> impl Fn(&Theme) -> text::Style {
    move |theme| text::Style { color: Some(pick(tokens(theme))), selection: None }
}

/// `ink-muted` text: field labels, secondary text, units.
pub fn muted(theme: &Theme) -> text::Style {
    text::Style { color: Some(tokens(theme).ink_muted), selection: None }
}

/// `ink-faint` text: section heads, status keys, placeholders.
pub fn faint(theme: &Theme) -> text::Style {
    text::Style { color: Some(tokens(theme).ink_faint), selection: None }
}

/// `ink` text: values.
pub fn ink(theme: &Theme) -> text::Style {
    text::Style { color: Some(tokens(theme).ink), selection: None }
}

/// Default weight: `bg-300` with a `line-200` border; `bg-400` on hover.
pub fn secondary(theme: &Theme, status: button::Status) -> button::Style {
    let t = tokens(theme);
    let style = button::Style { background: Some(Background::Color(t.bg_300)), text_color: t.ink, border: frame(t.line_200), ..button::Style::default() };
    match status {
        button::Status::Hovered | button::Status::Pressed => button::Style { background: Some(Background::Color(t.bg_400)), ..style },
        button::Status::Disabled => disabled(t),
        button::Status::Active => style,
    }
}

/// A rail button (parts bin category): `ink-faint` icon, `ink` on `bg-400` when hovered, `ink` on `bg-200` when active.
pub fn rail(active: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |theme, status| {
        let t = tokens(theme);
        let (bg, fg) = match (active, status) {
            (_, button::Status::Hovered | button::Status::Pressed) => (Some(t.bg_400), t.ink),
            (true, _) => (Some(t.bg_200), t.ink),
            (false, _) => (None, t.ink_faint),
        };
        button::Style { background: bg.map(Background::Color), text_color: fg, border: Border::default(), ..button::Style::default() }
    }
}

/// A tooltip bubble: an overlay frame (`bg-200`, `line-200`).
pub fn tooltip(theme: &Theme) -> container::Style {
    overlay(theme)
}

/// A 1 px `line-100` rule: give it a width or height of 1.
pub fn hairline(theme: &Theme) -> container::Style {
    container::Style { background: Some(Background::Color(tokens(theme).line_100)), ..container::Style::default() }
}

/// A transparent, outlined label in a token colour (the warning chip).
pub fn outlined(pick: fn(&Tokens) -> Color) -> impl Fn(&Theme) -> container::Style {
    move |theme| {
        let c = pick(tokens(theme));
        container::Style { text_color: Some(c), border: frame(c), ..container::Style::default() }
    }
}

/// The framed mono keycap inside a button or tooltip (`F5`, `Ctrl+Z`).
pub fn keycap(theme: &Theme) -> container::Style {
    let t = tokens(theme);
    container::Style { text_color: Some(t.ink_muted), border: frame(t.line_200), ..container::Style::default() }
}

/// A select (`pick_list`): the text-field look with an `ink-muted` handle.
pub fn select(theme: &Theme, status: pick_list::Status) -> pick_list::Style {
    let t = tokens(theme);
    let border = match status {
        pick_list::Status::Active | pick_list::Status::Disabled => t.line_200,
        pick_list::Status::Hovered => t.ink_muted,
        pick_list::Status::Opened { .. } => t.accent,
    };
    pick_list::Style { text_color: t.ink, placeholder_color: t.ink_faint, handle_color: t.ink_muted, background: Background::Color(t.bg_300), border: frame(border) }
}

/// The dropdown of a select: `bg-200` in a `line-200` frame, the selected row `bg-400`.
pub fn select_menu(theme: &Theme) -> menu::Style {
    let t = tokens(theme);
    menu::Style {
        background: Background::Color(t.bg_200),
        border: frame(t.line_200),
        text_color: t.ink,
        selected_text_color: t.ink,
        selected_background: Background::Color(t.bg_400),
        shadow: iced::Shadow::default(),
    }
}

/// The keycap inside a primary button: `accent-ink` on the accent fill.
pub fn keycap_on_accent(theme: &Theme) -> container::Style {
    let t = tokens(theme);
    container::Style { text_color: Some(t.accent_ink), border: frame(t.accent_ink), ..container::Style::default() }
}
