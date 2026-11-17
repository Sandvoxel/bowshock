//! The File / Edit / View menu bar: the three labels in the top bar and the dropdown that hangs under the open one.
//!
//! The dropdown is an overlay over the whole window: a scrim that closes it, the panel under its label, and the label row drawn
//! again above the scrim at exactly the place it has in the top bar, so the pointer can move onto another label (which switches
//! menus) or click the open one (which closes it). Every x position comes from the constants below, which the top bar uses too.

use iced::widget::{button, column, container, mouse_area, row, text, Space, Stack};
use iced::{Background, Center, Element, Fill, Padding, Theme, Widget};

use super::Message as ChromeMessage;
use crate::app::{ui, Ctx, Message, Mode, Selection};
use crate::theme::{self, MONO};
use crate::tokens::{CONTROL_H, SPACE_3, TEXT_BODY, TOPBAR_H};
use crate::viewport::{self, InsideView, Projection};
use crate::widgets::glyph::{icon, Mark, Tone};

/// Width of the logo container at the left of the bar (the mark and `BOWSHOCK`).
pub const LOGO_W: f32 = 112.0;
/// Width of each menu label.
pub const LABEL_W: f32 = 52.0;
/// The bar's side padding and the spacing between its regions.
pub const BAR_PAD: f32 = SPACE_3;
pub const BAR_SPACING: f32 = SPACE_3;
/// Width of a hairline rule in the bar.
pub const RULE_W: f32 = 1.0;
/// Left edge of the first label.
const LABELS_X: f32 = BAR_PAD + LOGO_W + BAR_SPACING + RULE_W + BAR_SPACING;

/// Width of a menu panel.
const MENU_W: f32 = 300.0;
/// Height of a menu row.
const ROW_H: f32 = 26.0;
/// The leading gutter of a menu that holds check marks.
const GUTTER_W: f32 = 18.0;

/// The modifier a shortcut hint is written with.
const MOD: &str = if cfg!(target_os = "macos") { "⌘" } else { "Ctrl+" };
/// Shift's glyph in a hint.
const SHIFT: &str = if cfg!(target_os = "macos") { "⇧" } else { "Shift+" };

/// The menus of the bar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopMenu {
    File,
    Edit,
    View,
}

impl TopMenu {
    /// The menus in bar order.
    pub const ALL: [TopMenu; 3] = [TopMenu::File, TopMenu::Edit, TopMenu::View];

    /// The label in the bar.
    pub fn label(self) -> &'static str {
        match self {
            TopMenu::File => "File",
            TopMenu::Edit => "Edit",
            TopMenu::View => "View",
        }
    }

    /// Left edge of this menu's label, and of its panel.
    fn x(self) -> f32 {
        LABELS_X + Self::ALL.iter().position(|m| *m == self).unwrap_or(0) as f32 * LABEL_W
    }
}

/// What the viewport currently shows, for the View menu's check marks.
#[derive(Debug, Clone, Copy)]
pub struct ViewChecks {
    pub projection: Projection,
    pub inside: InsideView,
}

/// One selectable row.
#[derive(Debug, Clone)]
pub struct Item {
    pub label: String,
    pub hint: String,
    /// What a click sends; `None` is a greyed row.
    pub message: Option<Message>,
    /// The row is the current choice / switched on: it gets the accent check mark.
    pub checked: bool,
}

/// One line of a menu.
#[derive(Debug, Clone)]
pub enum Row {
    Item(Box<Item>),
    Separator,
    /// A small caps heading (`RECENT`).
    Section(&'static str),
    /// A faint line of text that does nothing.
    Note(&'static str),
}

fn item(label: &str, hint: impl Into<String>, message: Message) -> Row {
    Row::Item(Box::new(Item { label: label.to_string(), hint: hint.into(), message: Some(message), checked: false }))
}

fn item_if(enabled: bool, label: impl Into<String>, hint: impl Into<String>, message: Message) -> Row {
    Row::Item(Box::new(Item { label: label.into(), hint: hint.into(), message: enabled.then_some(message), checked: false }))
}

fn choice(enabled: bool, checked: bool, label: &str, hint: &str, message: Message) -> Row {
    Row::Item(Box::new(Item { label: label.to_string(), hint: hint.to_string(), message: enabled.then_some(message), checked }))
}

/// The rows of `menu`.
pub fn rows(menu: TopMenu, ctx: &Ctx, view: ViewChecks) -> Vec<Row> {
    match menu {
        TopMenu::File => file_rows(ctx),
        TopMenu::Edit => edit_rows(ctx),
        TopMenu::View => view_rows(ctx, view),
    }
}

fn file_rows(ctx: &Ctx) -> Vec<Row> {
    use crate::export::{self, Kind};
    let mut rows = vec![
        item("New", format!("{MOD}N"), Message::New),
        item("Open…", format!("{MOD}O"), Message::Open),
        item("Open example", format!("{MOD}E"), Message::Example),
        Row::Separator,
        item("Save", format!("{MOD}S"), Message::Save),
        item("Save as…", format!("{SHIFT}{MOD}S"), Message::SaveAs),
        Row::Separator,
        item("Import OpenRocket (.ork)…", "", Message::ImportOrk),
        item("Export 3D model…", "", Message::Export(export::Message::Open(Kind::Model))),
        item("Export image…", "", Message::Export(export::Message::Open(Kind::Image))),
        item("Print / export PDF…", format!("{MOD}P"), Message::Print(crate::print::Message::Open)),
        Row::Separator,
        Row::Section("RECENT"),
    ];
    if ctx.recent.paths().is_empty() {
        rows.push(Row::Note("No recent files"));
    }
    for p in ctx.recent.paths() {
        let name = p.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| p.display().to_string());
        let dir = p.parent().and_then(|d| d.file_name()).map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        rows.push(Row::Item(Box::new(Item { label: name, hint: dir, message: p.exists().then(|| Message::OpenRecent(p.clone())), checked: false })));
    }
    rows
}

fn edit_rows(ctx: &Ctx) -> Vec<Row> {
    let step = |verb: &str, label: Option<&str>| label.map_or(verb.to_string(), |s| format!("{verb} {}", s.to_lowercase()));
    let deletable = ctx.mode == Mode::Build && matches!(ctx.selection, Selection::Part(_));
    vec![
        item_if(ctx.doc.can_undo(), step("Undo", ctx.doc.undo_label()), format!("{MOD}Z"), Message::Undo),
        item_if(ctx.doc.can_redo(), step("Redo", ctx.doc.redo_label()), format!("{SHIFT}{MOD}Z"), Message::Redo),
        Row::Separator,
        item("Rename", "F2", Message::Rename),
        item_if(deletable, "Delete", "", Message::Delete),
        Row::Separator,
        item("Preferences…", format!("{MOD},"), Message::Prefs(crate::prefs::Message::Show)),
    ]
}

fn view_rows(ctx: &Ctx, view: ViewChecks) -> Vec<Row> {
    let build = ctx.mode == Mode::Build;
    let mode = |m: Mode, label: &str| choice(true, ctx.mode == m, label, "", Message::Mode(m));
    let projection = |p: Projection, label: &str| choice(build, view.projection == p, label, "", Message::Viewport(viewport::Message::Projection(p)));
    let inside = |v: InsideView, label: &str, key: &str| choice(build, view.inside == v, label, key, Message::Viewport(viewport::Message::InsideView(v)));
    let toggle = |on: bool, label: &str, m: ChromeMessage| choice(true, on, label, "", Message::Chrome(m));
    vec![
        mode(Mode::Build, "Build"),
        mode(Mode::Simulate, "Simulate"),
        mode(Mode::Analyze, "Analyze"),
        Row::Separator,
        projection(Projection::ThreeD, "3D"),
        projection(Projection::Profile, "2D"),
        Row::Separator,
        inside(InsideView::Solid, "Solid", "1"),
        inside(InsideView::Cutaway, "Cutaway", "2"),
        inside(InsideView::XRay, "X-ray", "3"),
        inside(InsideView::Exploded, "Exploded", "4"),
        Row::Separator,
        toggle(ctx.show_cg, "Show CG marker", ChromeMessage::ToggleCg),
        toggle(ctx.show_cp, "Show CP marker", ChromeMessage::ToggleCp),
        toggle(ctx.compare_configs, "Compare configurations in lanes", ChromeMessage::ToggleCompare),
    ]
}

/// The three labels as they sit in the top bar: `open` is the menu whose panel is showing. A click opens that menu, or closes it
/// when it is the open one.
pub fn labels<'a>(open: Option<TopMenu>) -> Element<'a, Message> {
    label_row(open, false)
}

fn label_row<'a>(open: Option<TopMenu>, hover_switches: bool) -> Element<'a, Message> {
    let mut cells = row![].spacing(0);
    for menu in TopMenu::ALL {
        let is_open = open == Some(menu);
        let style = move |theme: &Theme, status: button::Status| {
            if is_open {
                let t = theme::tokens(theme);
                button::Style { background: Some(Background::Color(t.bg_400)), text_color: t.ink, ..button::Style::default() }
            } else {
                theme::ghost(theme, status)
            }
        };
        let body = text(menu.label()).size(TEXT_BODY).width(Fill).align_x(Center);
        let click = Message::Menu(if is_open { None } else { Some(menu) });
        let b = button(body).width(LABEL_W).height(CONTROL_H).padding([4.0, 0.0]).style(style).on_press(click);
        // Only the other labels switch on hover: a fresh `mouse_area` reports an enter on the next event of any kind, so one on
        // the open label would re-open its menu right after Escape closed it.
        let cell: Element<'a, Message> = if hover_switches && !is_open { mouse_area(b).on_enter(Message::Menu(Some(menu))).boxed() } else { b.boxed() };
        cells = cells.push(cell);
    }
    cells.boxed()
}

/// The open menu over the window; `None` while all are closed.
pub fn overlay<'a>(ctx: &'a Ctx, view: ViewChecks) -> Option<Element<'a, Message>> {
    let menu = ctx.menu?;
    let rows = rows(menu, ctx, view);
    // Every row of a menu with check marks leaves room for them.
    let gutter = menu == TopMenu::View;

    let mut col = column![];
    for r in rows {
        col = col.push(row_view(r, gutter));
    }
    let panel = container(mouse_area(col).on_press(Message::Chrome(ChromeMessage::Nothing))).width(MENU_W).style(theme::overlay);
    let scrim = crate::widgets::modal::scrim(Message::Menu(None));
    let placed = container(panel).width(Fill).height(Fill).padding(Padding { top: TOPBAR_H, left: menu.x(), ..Padding::ZERO });
    let bar = container(label_row(Some(menu), true)).width(Fill).height(TOPBAR_H).padding(Padding { left: LABELS_X, ..Padding::ZERO }).align_y(Center);
    Some(Stack::with_children([scrim, placed.boxed(), bar.boxed()]).boxed())
}

fn row_view<'a>(row_: Row, gutter: bool) -> Element<'a, Message> {
    match row_ {
        Row::Separator => container(Space::new()).width(Fill).height(1).style(theme::hairline).boxed(),
        Row::Section(name) => container(ui::label(name)).padding([6.0, 10.0]).boxed(),
        Row::Note(note) => container(text(note).size(12.5).style(theme::faint)).padding([4.0, 10.0]).boxed(),
        Row::Item(it) => {
            let enabled = it.message.is_some();
            let mark: Element<'a, Message> = if it.checked { icon(Mark::Check, Tone::Accent, 13.0) } else { Space::new().boxed() };
            let lead = container(mark).width(GUTTER_W);
            let hint = text(it.hint).font(MONO).size(11.5).style(if enabled { theme::muted } else { theme::faint });
            let body = row![text(it.label).size(12.5), Space::new().width(Fill), hint].align_y(Center);
            let line = if gutter { row![lead, body].align_y(Center) } else { row![body] };
            let style = |theme: &Theme, status: button::Status| {
                let t = theme::tokens(theme);
                let hovered = matches!(status, button::Status::Hovered | button::Status::Pressed);
                let background = hovered.then_some(Background::Color(t.bg_400));
                let text_color = if matches!(status, button::Status::Disabled) { t.ink_faint } else { t.ink };
                button::Style { background, text_color, border: iced::Border::default(), ..button::Style::default() }
            };
            let mut b = button(line).width(Fill).height(ROW_H).padding([0.0, 10.0]).style(style);
            if let Some(message) = it.message {
                b = b.on_press(message);
            }
            b.boxed()
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    fn view() -> ViewChecks {
        ViewChecks { projection: Projection::ThreeD, inside: InsideView::Solid }
    }

    fn labels_of(rows: &[Row]) -> Vec<String> {
        rows.iter()
            .map(|r| match r {
                Row::Item(i) => i.label.clone(),
                Row::Separator => "|".to_string(),
                Row::Section(s) => s.to_string(),
                Row::Note(n) => n.to_string(),
            })
            .collect()
    }

    fn message(rows: &[Row], label: &str) -> Option<Message> {
        rows.iter().find_map(|r| match r {
            Row::Item(i) if i.label == label => Some(i.message.clone()),
            _ => None,
        })?
    }

    /// The File menu's rows, in order, route to the right commands.
    #[test]
    fn the_file_menu_routes_to_commands() {
        let ctx = Ctx::new();
        let rows = rows(TopMenu::File, &ctx, view());
        assert_eq!(
            labels_of(&rows),
            [
                "New",
                "Open…",
                "Open example",
                "|",
                "Save",
                "Save as…",
                "|",
                "Import OpenRocket (.ork)…",
                "Export 3D model…",
                "Export image…",
                "Print / export PDF…",
                "|",
                "RECENT",
                "No recent files"
            ]
        );
        assert!(matches!(message(&rows, "New"), Some(Message::New)));
        assert!(matches!(message(&rows, "Open…"), Some(Message::Open)));
        assert!(matches!(message(&rows, "Open example"), Some(Message::Example)));
        assert!(matches!(message(&rows, "Save"), Some(Message::Save)));
        assert!(matches!(message(&rows, "Save as…"), Some(Message::SaveAs)));
        assert!(matches!(message(&rows, "Import OpenRocket (.ork)…"), Some(Message::ImportOrk)));
        assert!(matches!(message(&rows, "Export 3D model…"), Some(Message::Export(crate::export::Message::Open(crate::export::Kind::Model)))));
        assert!(matches!(message(&rows, "Export image…"), Some(Message::Export(crate::export::Message::Open(crate::export::Kind::Image)))));
        assert!(matches!(message(&rows, "Print / export PDF…"), Some(Message::Print(crate::print::Message::Open))));
        let Some(Row::Item(save)) = rows.iter().find(|r| matches!(r, Row::Item(i) if i.label == "Save")) else { panic!() };
        assert_eq!(save.hint, format!("{MOD}S"));
    }

    /// A recent file that is gone is greyed (no message); one that exists opens.
    #[test]
    fn missing_recent_files_are_greyed() {
        let temp = |name: &str| -> PathBuf { std::env::temp_dir().join(format!("bowshock-{}-{name}", std::process::id())) };
        let real = temp("real.bowshock");
        std::fs::write(&real, "{}").unwrap();
        let mut ctx = Ctx::new();
        ctx.recent.push(&temp("gone.bowshock"));
        ctx.recent.push(&real);
        let all = rows(TopMenu::File, &ctx, view());
        std::fs::remove_file(&real).ok();
        let recents: Vec<&Item> =
            all.iter().skip_while(|r| !matches!(r, Row::Section("RECENT"))).filter_map(|r| if let Row::Item(i) = r { Some(&**i) } else { None }).collect();
        assert_eq!(recents.len(), 2);
        assert!(matches!(recents[0].message, Some(Message::OpenRecent(_))));
        assert!(recents[1].message.is_none(), "a missing file is greyed");
    }

    /// The Edit menu: undo and redo are greyed with nothing to undo, Delete needs a selected part in Build mode.
    #[test]
    fn the_edit_menu_greys_what_cannot_run() {
        let mut ctx = Ctx::new();
        let rows_now = |ctx: &Ctx| rows(TopMenu::Edit, ctx, view());
        let r = rows_now(&ctx);
        assert_eq!(labels_of(&r).iter().filter(|l| *l != "|").count(), 5);
        assert!(message(&r, "Delete").is_none());
        assert!(message(&r, "Rename").is_some());
        assert!(matches!(message(&r, "Preferences…"), Some(Message::Prefs(crate::prefs::Message::Show))));
        ctx.doc.edit("Rename rocket", |rk| rk.name = "x".to_string());
        let r = rows_now(&ctx);
        assert!(matches!(message(&r, "Undo rename rocket"), Some(Message::Undo)));
        assert!(message(&r, "Redo").is_none());
    }

    /// The View menu: checks follow the mode, projection and inside view; the last two groups grey out outside Build.
    #[test]
    fn the_view_menu_checks_the_current_choices() {
        let mut ctx = Ctx::new();
        let checked = |rows: &[Row]| -> Vec<String> {
            rows.iter().filter_map(|r| if let Row::Item(i) = r { i.checked.then(|| i.label.clone()) } else { None }).collect()
        };
        let r = rows(TopMenu::View, &ctx, ViewChecks { projection: Projection::Profile, inside: InsideView::XRay });
        assert_eq!(checked(&r), ["Build", "2D", "X-ray", "Show CG marker", "Show CP marker"]);
        assert!(matches!(message(&r, "Cutaway"), Some(Message::Viewport(viewport::Message::InsideView(InsideView::Cutaway)))));
        ctx.mode = Mode::Simulate;
        let r = rows(TopMenu::View, &ctx, view());
        assert!(message(&r, "Cutaway").is_none() && message(&r, "3D").is_none());
        assert!(matches!(message(&r, "Analyze"), Some(Message::Mode(Mode::Analyze))));
    }
}
