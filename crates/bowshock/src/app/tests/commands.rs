use super::*;

/// Hovering the parts bin (rows, focus flag) while the File menu is open must not close it; a click elsewhere still does.
#[test]
fn hovering_the_parts_bin_keeps_the_file_menu_open() {
    let (mut app, _) = app();
    app.update(Message::ToggleFileMenu);
    assert!(app.ctx.file_menu);
    let row = parts_bin::RowId::new(parts_bin::Category::Fins, 0);
    app.update(Message::PartsBin(parts_bin::Message::Hover(true)));
    app.update(Message::PartsBin(parts_bin::Message::RowHover(Some(row))));
    app.update(Message::PartsBin(parts_bin::Message::RowHover(None)));
    app.update(Message::PartsBin(parts_bin::Message::Hover(false)));
    assert!(app.ctx.file_menu, "hover must not dismiss the menu");
    app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
    assert!(!app.ctx.file_menu, "a real action closes it");
}

#[test]
fn undo_redo_shortcuts_and_the_dirty_flag() {
    let (mut app, _) = app();
    assert!(app.ctx.doc.is_dirty());
    let press = |c: &str, shift: bool| {
        let modifiers = if shift { keyboard::Modifiers::COMMAND | keyboard::Modifiers::SHIFT } else { keyboard::Modifiers::COMMAND };
        Message::Key(KeyPress { key: keyboard::Key::Character(c.into()), modifiers, captured: false })
    };
    app.update(press("z", false));
    assert_eq!(app.ctx.doc.rocket().parts().len(), 2);
    app.update(press("z", true));
    assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
    app.update(press("z", false));
    app.update(press("y", false));
    assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
}

#[test]
fn the_example_loads_and_a_clean_document_is_replaced_without_asking() {
    let mut app = App::new();
    app.update(Message::Example);
    assert!(!app.ctx.doc.rocket().parts().is_empty() && !app.ctx.doc.is_dirty());
    assert!(app.ctx.physics.balance().is_some());
    app.update(Message::New);
    assert!(app.ctx.unsaved.is_none(), "no modal when nothing is unsaved");
    assert!(app.ctx.doc.rocket().parts().is_empty());
}

#[test]
fn f2_renames_the_selected_part_and_otherwise_the_configuration() {
    let mut app = App::new();
    app.update(Message::Example);
    let id = app.ctx.doc.rocket().parts()[0].id;
    app.ctx.selection = Selection::Part(id);
    let f2 = KeyPress { key: keyboard::Key::Named(keyboard::key::Named::F2), modifiers: keyboard::Modifiers::empty(), captured: false };
    assert!(matches!(shortcut(&f2), Some(Message::Rename)));
    app.update(Message::Rename);
    app.update(Message::Inspector(inspector::Message::RenameInput("Cone".to_string())));
    app.update(Message::Inspector(inspector::Message::RenameCommit));
    assert_eq!(app.ctx.doc.rocket().part(id).unwrap().display_name(), "Cone");
    // Nothing selected: the configuration's name is edited instead, the part keeps its own.
    app.ctx.selection = Selection::None;
    app.update(Message::Rename);
    app.update(Message::Chrome(chrome::Message::RenameInput("Windy".to_string())));
    app.update(Message::Chrome(chrome::Message::CommitRename));
    assert_eq!(app.ctx.doc.rocket().part(id).unwrap().display_name(), "Cone");
    let rocket = app.ctx.doc.rocket();
    assert_eq!(rocket.configuration_name(rocket.active_config_id()).as_deref(), Some("Windy"));
}

#[test]
fn new_with_unsaved_edits_opens_the_modal_and_blocks_other_commands() {
    let mut app = dirty_app();
    assert_eq!(app.ctx.unsaved, Some(Pending::New));
    assert!(!app.ctx.doc.rocket().parts().is_empty(), "nothing is replaced yet");
    app.update(Message::Delete);
    app.update(Message::Example);
    assert_eq!(app.ctx.unsaved, Some(Pending::New), "other commands are swallowed while it is open");
    assert!(app.ctx.doc.is_dirty());
}

#[test]
fn dont_save_proceeds_and_cancel_keeps_the_document() {
    let mut app = dirty_app();
    app.update(Message::UnsavedCancel);
    assert!(app.ctx.unsaved.is_none() && app.ctx.doc.is_dirty() && !app.ctx.doc.rocket().parts().is_empty());
    app.update(Message::New);
    app.update(Message::Cancel);
    assert!(app.ctx.unsaved.is_none() && app.ctx.doc.is_dirty(), "Escape cancels");
    app.update(Message::New);
    app.update(Message::UnsavedDiscard);
    assert!(app.ctx.unsaved.is_none() && !app.ctx.doc.is_dirty() && app.ctx.doc.rocket().parts().is_empty());
}

#[test]
fn save_writes_the_file_then_proceeds() {
    let path = std::env::temp_dir().join(format!("bowshock-{}-unsaved-modal.bowshock", std::process::id()));
    let mut app = App::new();
    app.update(Message::Example);
    app.ctx.doc.save_as(&path).unwrap();
    app.ctx.doc.edit("Rename", |r| r.name = "changed".to_string());
    app.update(Message::New);
    assert!(app.ctx.unsaved.is_some());
    app.update(Message::Key(KeyPress { key: keyboard::Key::Named(keyboard::key::Named::Enter), modifiers: keyboard::Modifiers::empty(), captured: false }));
    let saved = bowshock_model::Rocket::from_bytes(&std::fs::read(&path).unwrap()).unwrap();
    std::fs::remove_file(&path).ok();
    assert_eq!(saved.name, "changed", "Enter saves");
    assert!(app.ctx.unsaved.is_none() && app.ctx.doc.rocket().parts().is_empty() && !app.ctx.doc.is_dirty());
}

#[test]
fn closing_the_window_with_unsaved_edits_asks_first() {
    let mut app = App::new();
    app.update(Message::Example);
    app.update(Message::CloseRequested);
    assert!(app.ctx.unsaved.is_none(), "a clean document closes without asking");
    app.ctx.doc.edit("Rename", |r| r.name = "changed".to_string());
    app.update(Message::CloseRequested);
    assert_eq!(app.ctx.unsaved, Some(Pending::Exit));
    app.update(Message::UnsavedCancel);
    assert!(app.ctx.unsaved.is_none() && app.ctx.doc.is_dirty(), "Cancel stays");
    app.update(Message::CloseRequested);
    app.update(Message::UnsavedSave);
    assert_eq!(app.ctx.saving_for, Some(Pending::Exit), "an untitled document asks where to save, then exits");
    app.update(Message::SavePath(None));
    assert!(app.ctx.saving_for.is_none() && app.ctx.doc.is_dirty(), "a cancelled Save As keeps the window open");
}

#[test]
fn save_as_from_the_modal_proceeds_when_answered_and_aborts_when_cancelled() {
    let path = std::env::temp_dir().join(format!("bowshock-{}-unsaved-as.bowshock", std::process::id()));
    let mut app = dirty_app();
    app.update(Message::UnsavedSave);
    assert!(app.ctx.unsaved.is_none() && app.ctx.saving_for == Some(Pending::New), "an untitled document asks where to save");
    app.update(Message::SavePath(None));
    assert!(app.ctx.saving_for.is_none() && app.ctx.doc.is_dirty() && !app.ctx.doc.rocket().parts().is_empty(), "cancelled: nothing happens");
    app.update(Message::New);
    app.update(Message::UnsavedSave);
    app.update(Message::SavePath(Some(path.clone())));
    let written = path.exists();
    std::fs::remove_file(&path).ok();
    assert!(written && app.ctx.doc.rocket().parts().is_empty() && !app.ctx.doc.is_dirty());
}

#[test]
fn configuration_commands_undo_and_the_status_values_follow_the_document() {
    use crate::chrome::Message as Chrome;
    let mut app = App::new();
    app.update(Message::Example);
    let first = app.ctx.doc.rocket().active_config_id();
    let margin = app.ctx.physics.balance().unwrap().margin_cal;
    assert!(margin > 1.0 && app.ctx.physics.config_margin(first) == Some(margin));

    // Duplicate makes the copy current; Undo brings the first back.
    app.update(Message::Chrome(Chrome::DuplicateConfig));
    let copy = app.ctx.doc.rocket().active_config_id();
    assert_ne!(copy, first);
    assert_eq!(app.ctx.doc.rocket().configurations().len(), 2);
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket().active_config_id(), first);

    // Rename through the menu's field.
    app.update(Message::Chrome(Chrome::DuplicateConfig));
    app.update(Message::Chrome(Chrome::BeginRename));
    app.update(Message::Chrome(Chrome::RenameInput("Windy day".to_string())));
    app.update(Message::Chrome(Chrome::CommitRename));
    let rocket = app.ctx.doc.rocket();
    assert_eq!(rocket.configuration_name(rocket.active_config_id()).as_deref(), Some("Windy day"));

    // Delete: one is removed, the last one stays (with a message).
    let active = rocket.active_config_id();
    app.update(Message::Chrome(Chrome::DeleteConfig(active)));
    assert_eq!(app.ctx.doc.rocket().configurations().len(), 1);
    let last = app.ctx.doc.rocket().active_config_id();
    app.update(Message::Chrome(Chrome::DeleteConfig(last)));
    assert_eq!(app.ctx.doc.rocket().configurations().len(), 1);
    assert!(app.ctx.notice.is_some());
}
