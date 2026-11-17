//! The top bar (design/components/Navigation, BuilderScreen): logo and name, the File / Edit / View menu labels, the document's
//! title with its dirty marker, undo and redo, the Build / Simulate / Analyze switch in the centre, the Preferences button and the
//! primary Simulate button. The theme and the units are set in Preferences only.

use iced::widget::{canvas, container, row, stack, text, Space};
use iced::{Background, Center, Element, Fill, Theme, Widget};

use super::menu_bar::{self, BAR_PAD, BAR_SPACING, LOGO_W, RULE_W};
use super::Message as ChromeMessage;
use crate::app::document::EXTENSION;
use crate::app::{Ctx, Message, Mode};
use crate::logo;
use crate::prefs;
use crate::theme::{self, MONO, SANS_SEMIBOLD};
use crate::tokens::{CONTROL_H, SPACE_1, SPACE_2, TEXT_BODY, TEXT_LABEL, TOPBAR_H};
use crate::widgets::button::{button, icon_button, Weight};
use crate::widgets::glyph::Mark;
use crate::widgets::segmented::segmented;

/// Undo's shortcut as the platform writes it.
const UNDO_TIP: &str = if cfg!(target_os = "macos") { "⌘Z" } else { "Ctrl+Z" };
/// Redo's shortcut as the platform writes it.
const REDO_TIP: &str = if cfg!(target_os = "macos") { "⇧⌘Z" } else { "Ctrl+Shift+Z" };

/// The bar.
pub fn top_bar(ctx: &Ctx) -> Element<'_, Message> {
    let mark = row![canvas(logo::Mark).width(20).height(20), text("BOWSHOCK").font(SANS_SEMIBOLD).size(TEXT_BODY)].spacing(SPACE_2).align_y(Center);
    let logo = container(mark).width(LOGO_W);

    let dirty: Element<'_, Message> = if ctx.doc.is_dirty() {
        container(Space::new())
            .width(7)
            .height(7)
            .style(|t: &Theme| container::Style { background: Some(Background::Color(theme::tokens(t).accent)), ..container::Style::default() })
            .boxed()
    } else {
        Space::new().width(7).boxed()
    };
    let ext: Element<'_, Message> =
        if ctx.doc.path().is_some() { text(format!(".{EXTENSION}")).font(MONO).size(TEXT_BODY).style(theme::faint).boxed() } else { Space::new().boxed() };
    // One line, cut with an ellipsis: an imported design can carry a sentence for a name, and the title has what is left of the
    // bar's left half (the container below clips anything longer in a narrow window).
    const TITLE_MAX: usize = 24;
    let full = ctx.doc.title();
    let shown =
        if full.chars().count() > TITLE_MAX { format!("{}…", full.chars().take(TITLE_MAX - 1).collect::<String>().trim_end()) } else { full.to_string() };
    let name = text(shown).font(MONO).size(TEXT_BODY).style(theme::ink).wrapping(iced::widget::text::Wrapping::None);
    let title = row![name, ext, dirty].spacing(SPACE_1 + 2.0).align_y(Center);

    let step = |verb: &'static str, label: Option<&str>| -> std::borrow::Cow<'static, str> {
        label.map_or(std::borrow::Cow::Borrowed(verb), |s| std::borrow::Cow::Owned(format!("{verb} {}", s.to_lowercase())))
    };
    let undo = icon_button(Mark::Undo, step("Undo", ctx.doc.undo_label())).shortcut(UNDO_TIP);
    let redo = icon_button(Mark::Redo, step("Redo", ctx.doc.redo_label())).shortcut(REDO_TIP);
    let undo: Element<'_, Message> = if ctx.doc.can_undo() { undo.on_press(Message::Undo).into() } else { undo.into() };
    let redo: Element<'_, Message> = if ctx.doc.can_redo() { redo.on_press(Message::Redo).into() } else { redo.into() };
    let (undo, redo) = (with_word(undo, "Undo"), with_word(redo, "Redo"));

    let modes = segmented(&[(Mode::Build, "Build"), (Mode::Simulate, "Simulate"), (Mode::Analyze, "Analyze")], ctx.mode, Message::Mode);

    let simulate: Element<'_, Message> = if ctx.run_pending() {
        button("Running…", Weight::Primary).icon(Mark::Play).key("F5").disabled("Flights are flying").into()
    } else {
        button("Simulate", Weight::Primary).icon(Mark::Play).key("F5").on_press(Message::Run).into()
    };
    let menu: Element<'_, Message> =
        button("", Weight::Primary).icon(Mark::CaretDown).tooltip("Run options").on_press(Message::Chrome(ChromeMessage::ToggleRunMenu)).into();
    // The caret is its own cell, divided from Simulate by a hairline of the accent's shadow.
    let divider = container(Space::new())
        .width(1)
        .height(26)
        .style(|t: &Theme| container::Style { background: Some(Background::Color(theme::tokens(t).bg_000.scale_alpha(0.45))), ..container::Style::default() });
    let simulate = row![simulate, divider, menu].spacing(0);

    // The two sides take equal shares of the bar, whatever they hold, so the mode switch sits on the window's centre line. Undo
    // and redo sit on the right, where there is room, so the title never pushes them out of the left half.
    let left = row![logo, rule(), menu_bar::labels(ctx.menu), rule(), title].spacing(BAR_SPACING).align_y(Center);
    let right = row![undo, redo, rule(), prefs_button(), simulate.boxed()].spacing(BAR_SPACING).align_y(Center);
    container(
        row![container(left).width(Fill).clip(true), modes, container(right).width(Fill).align_right(Fill)]
            .spacing(BAR_SPACING)
            .padding([0.0, BAR_PAD])
            .align_y(Center),
    )
    .height(TOPBAR_H)
    .center_y(TOPBAR_H)
    .style(theme::bar)
    .boxed()
}

/// A vertical hairline between regions of the bar.
fn rule<'a>() -> Element<'a, Message> {
    container(Space::new()).width(RULE_W).height(20).style(theme::hairline).boxed()
}

/// `body` with `word` laid over it, fully transparent, so a drive script can `click "<word>"`.
fn with_word<'a>(body: Element<'a, Message>, word: &'static str) -> Element<'a, Message> {
    let hidden = container(text(word).size(8).style(|_: &Theme| text::Style { color: Some(iced::Color::TRANSPARENT), selection: None })).center(Fill);
    stack![body, hidden].width(CONTROL_H).height(CONTROL_H).boxed()
}

/// The Preferences button (⌘, / Ctrl+,): a sliders glyph. Its (transparent) word is also in the button so a script can
/// `click "Prefs"`.
fn prefs_button<'a>() -> Element<'a, Message> {
    let glyph = canvas(SlidersGlyph).width(CONTROL_H).height(CONTROL_H);
    let body = with_word(glyph.boxed(), "Prefs");
    let button = iced::widget::button(body).padding(0).style(theme::ghost).on_press(Message::Prefs(prefs::Message::Show));
    let tip = if cfg!(target_os = "macos") { "Preferences  ⌘," } else { "Preferences  Ctrl+," };
    iced::widget::tooltip(button, container(text(tip).size(TEXT_LABEL + 1.0)).padding([SPACE_1, SPACE_2]).style(theme::tooltip))
        .position(iced::widget::tooltip::Position::Bottom)
        .boxed()
}

/// Three horizontal lines, each with a knob on it, in the button's text colour.
struct SlidersGlyph;

impl canvas::Program<Message> for SlidersGlyph {
    type State = ();

    fn draw(&self, _: &(), renderer: &iced::Renderer, theme: &Theme, bounds: iced::Rectangle, cursor: iced::mouse::Cursor) -> Vec<canvas::Geometry> {
        let t = theme::tokens(theme);
        let hovered = cursor.is_over(bounds);
        let color = if hovered { t.ink } else { t.ink_muted };
        let surface = if hovered { t.bg_400 } else { t.bg_100 };
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let c = iced::Point::new(bounds.width / 2.0, bounds.height / 2.0);
        let stroke = canvas::Stroke::default().with_color(color).with_width(1.4);
        let lines = [(-5.0, -3.0), (0.0, 3.5), (5.0, -0.5)];
        for (dy, _) in lines {
            frame.stroke(&canvas::Path::line(iced::Point::new(c.x - 7.0, c.y + dy), iced::Point::new(c.x + 7.0, c.y + dy)), stroke);
        }
        for (dy, dx) in lines {
            let knob = canvas::Path::circle(iced::Point::new(c.x + dx, c.y + dy), 2.2);
            frame.fill(&knob, surface);
            frame.stroke(&knob, stroke);
        }
        vec![frame.into_geometry()]
    }
}
