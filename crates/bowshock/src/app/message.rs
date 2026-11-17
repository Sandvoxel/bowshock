//! The top-level [`Message`], the key press it carries, and the global keyboard map.

use std::path::PathBuf;
use std::time::Instant;

use bowshock_model::PartId;
use iced::{keyboard, Point};

use super::{part_menu, scale, Mode};
use crate::build::{inspector, lanes, parts_bin};
use crate::units::UnitSystem;
use crate::viewport;
use crate::{analyze, chrome, export, prefs, print, simulate};

/// A key press, as the subscription reports it.
#[derive(Debug, Clone)]
pub struct KeyPress {
    /// The key.
    pub key: keyboard::Key,
    /// Modifier keys held.
    pub modifiers: keyboard::Modifiers,
    /// A widget (a focused text field, say) already handled it: plain-character shortcuts must not fire.
    pub captured: bool,
}

/// The top-level message: shared actions plus one variant per area, each wrapping that area's own `Message`.
#[derive(Debug, Clone)]
pub enum Message {
    /// A pane divider of the Build screen moved.
    Resized(iced::widget::pane_grid::ResizeEvent),
    /// The top bar's Build / Simulate / Analyze switch.
    Mode(Mode),
    /// Light / dark.
    ToggleTheme,
    /// The unit system select.
    Units(UnitSystem),
    /// Animation / notice clock tick.
    Frame(Instant),
    /// The cursor moved (window coordinates).
    Cursor(Point),
    /// Left button released anywhere nothing took it.
    Released,
    /// Escape: abandon the drag or press in progress and close menus.
    Cancel,
    /// F2: rename the selected part in the Inspector, else the active configuration.
    Rename,
    /// Right-click on a part (the viewport and the lanes have selected it): open its context menu at the cursor.
    OpenPartMenu(PartId),
    /// An entry of the part context menu (or a click outside it).
    PartMenu(part_menu::Action),
    /// The Scale overlay: open it, edit it, apply it.
    Scale(scale::Message),
    /// Any key press (the app maps global shortcuts and forwards the rest to the active area).
    Key(KeyPress),
    /// Delete / Backspace: remove the selected part.
    Delete,
    /// Undo the last document step.
    Undo,
    /// Redo the last undone step.
    Redo,
    /// New document.
    New,
    /// Open a document (shows the file dialog).
    Open,
    /// The open dialog finished.
    OpenPath(Option<PathBuf>),
    /// Save (to the document's path, or Save As when it has none).
    Save,
    /// Save As (shows the file dialog).
    SaveAs,
    /// The save dialog finished.
    SavePath(Option<PathBuf>),
    /// Load the built-in example rocket.
    Example,
    /// The top bar's file name: open or close the File menu.
    ToggleFileMenu,
    /// File menu > Import OpenRocket (.ork)…: shows the import dialog.
    ImportOrk,
    /// File menu > a recent file.
    OpenRecent(PathBuf),
    /// The "Save changes?" modal's *Save* (Enter): save, then run the held command.
    UnsavedSave,
    /// The modal's *Don't save*: run the held command and lose the changes.
    UnsavedDiscard,
    /// The modal's *Cancel* (Escape): keep the document.
    UnsavedCancel,
    /// The Simulate button (F5): switch to Simulate and run the current configuration's flight.
    Run,
    /// The Simulate button's menu: switch to Simulate and fly every configuration (one flight each).
    RunAll,
    /// The parts bin.
    PartsBin(parts_bin::Message),
    /// The viewport.
    Viewport(viewport::Message),
    /// Station Lanes.
    Lanes(lanes::Message),
    /// The inspector.
    Inspector(inspector::Message),
    /// The Simulate screen.
    Simulate(simulate::Message),
    /// The Analyze screen.
    Analyze(analyze::Message),
    /// The top bar and status strip: configuration menu, phase, issues, motor picker.
    Chrome(chrome::Message),
    /// The window's close button, Cmd+Q or Alt+F4: exits at once when nothing is unsaved, else asks first.
    CloseRequested,
    /// The export 3D model and export image dialogs.
    Export(export::Message),
    /// The print / PDF dialog.
    Print(print::Message),
    /// The Preferences window.
    Prefs(prefs::Message),
}

/// What a click means while a field is in pick mode (the viewport and the lanes are pickable): `Some(Some(part))` opens the part's
/// value menu, `Some(None)` is swallowed (a click on nothing, a grab, a drag of a bar), `None` is not a pick at all.
pub(super) fn pick_intercept(message: &Message) -> Option<Option<PartId>> {
    use crate::build::lanes::Message as Lanes;
    use crate::viewport::Message as Viewport;
    match message {
        Message::Viewport(Viewport::Select(Some(pick)) | Viewport::Toggle(pick)) => Some(Some(pick.id())),
        Message::Viewport(Viewport::SelectPart(Some(id))) => Some(Some(*id)),
        Message::Viewport(Viewport::Select(None) | Viewport::SelectPart(None) | Viewport::SelectSeparation(_) | Viewport::Grab(_) | Viewport::Focus(_)) => {
            Some(None)
        }
        Message::Lanes(Lanes::Select(Some(key)) | Lanes::Toggle(key)) => Some(Some(key.part())),
        Message::Viewport(Viewport::PartMenu(_)) | Message::Lanes(Lanes::PartMenu(_)) => Some(None),
        Message::Lanes(
            Lanes::Select(None) | Lanes::SelectSeparation(_) | Lanes::Focus(_) | Lanes::GrabAxial(_) | Lanes::MoveTo { .. } | Lanes::ResizeTo { .. },
        ) => Some(None),
        _ => None,
    }
}

/// The global keyboard map: shortcuts that work in every mode, whatever has focus.
pub(super) fn shortcut(press: &KeyPress) -> Option<Message> {
    use keyboard::key::Named;
    let mods = press.modifiers;
    match &press.key {
        keyboard::Key::Named(Named::Escape) => Some(Message::Cancel),
        keyboard::Key::Named(Named::F5) => Some(Message::Run),
        keyboard::Key::Named(Named::F2) => Some(Message::Rename),
        keyboard::Key::Named(Named::Delete | Named::Backspace) if !press.captured && !mods.command() => Some(Message::Delete),
        keyboard::Key::Character(c) if mods.command() => {
            let c = c.as_str().to_lowercase();
            match (c.as_str(), mods.shift()) {
                ("z", false) => Some(Message::Undo),
                ("z", true) | ("y", false) => Some(Message::Redo),
                ("n", false) => Some(Message::New),
                ("o", false) => Some(Message::Open),
                ("s", false) => Some(Message::Save),
                ("s", true) => Some(Message::SaveAs),
                ("e", false) => Some(Message::Example),
                ("l", false) => Some(Message::Chrome(chrome::Message::NewConfiguration)),
                ("p", false) => Some(Message::Print(print::Message::Open)),
                (",", false) => Some(Message::Prefs(prefs::Message::Show)),
                _ => None,
            }
        }
        _ => None,
    }
}
