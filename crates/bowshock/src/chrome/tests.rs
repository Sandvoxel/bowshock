//! The chrome's behaviour through the whole app: marker toggles, the stability limits, issue clicks and the Simulate menu.

use bowshock_sim::from_model::Level;

use super::Message as Chrome;
use crate::app::{App, Message, Mode, Selection};
use crate::build::inspector::{Band, Message as Insp};

#[test]
fn an_openrocket_import_lists_its_notes_until_the_next_load() {
    let mut app = App::with_example();
    let fixture = concat!(env!("CARGO_MANIFEST_DIR"), "/../bowshock-model/tests/data/mini_two_stage.ork");
    let _ = app.update(Message::OpenPath(Some(fixture.into())));
    assert!(!app.ctx().import_notes.is_empty(), "the fixture has notes");
    let _ = app.update(Message::New);
    assert!(app.ctx().import_notes.is_empty());
}

#[test]
fn cg_and_cp_cells_toggle_their_markers() {
    let mut app = App::with_example();
    assert!(app.ctx().show_cg && app.ctx().show_cp);
    let _ = app.update(Message::Chrome(Chrome::ToggleCg));
    assert!(!app.ctx().show_cg && app.ctx().show_cp);
    let _ = app.update(Message::Chrome(Chrome::ToggleCp));
    let _ = app.update(Message::Chrome(Chrome::ToggleCg));
    assert!(app.ctx().show_cg && !app.ctx().show_cp);
}

#[test]
fn the_documents_limits_drive_the_health_list() {
    let mut app = App::with_example();
    let stable = |app: &App| app.ctx().physics.warnings().iter().any(|w| w.level == Level::Ok && w.message.starts_with("Stable"));
    assert!(stable(&app), "{:?}", app.ctx().physics.warnings());
    // Asking for three calibers makes the example's 1.9 marginal, and undoing the edit takes that back.
    let _ = app.update(Message::Inspector(Insp::Band(Band::Stable, 3.0)));
    assert_eq!(app.ctx().doc.rocket().stability_bands().stable, 3.0);
    let marginal: Vec<_> = app.ctx().physics.warnings().iter().filter(|w| w.message.starts_with("Marginal")).collect();
    assert_eq!(marginal.len(), 1, "{:?}", app.ctx().physics.warnings());
    assert_eq!(marginal[0].field, Some(bowshock_model::Dim::Span));
    let _ = app.update(Message::Undo);
    assert!(stable(&app));
}

#[test]
fn an_issue_with_a_field_selects_its_part() {
    let mut app = App::with_example();
    let _ = app.update(Message::Inspector(Insp::Band(Band::Stable, 3.0)));
    let issue = app.ctx().physics.issues().find(|w| w.field.is_some()).cloned().expect("a marginal warning");
    assert!(app.ctx().selection == Selection::None);
    let _ = app.update(Message::Chrome(Chrome::SelectIssue(issue.part, issue.field)));
    assert_eq!(app.ctx().selection, Selection::Part(issue.part.unwrap()));
}

#[test]
fn the_simulate_menu_runs_one_flight_per_configuration() {
    let mut app = App::with_example();
    let _ = app.update(Message::Chrome(Chrome::DuplicateConfig));
    assert_eq!(app.ctx().doc.rocket().configurations().len(), 2);
    let _ = app.update(Message::Chrome(Chrome::ToggleRunMenu));
    let _ = app.update(Message::Chrome(Chrome::RunAll));
    let _ = app.update(Message::RunAll);
    assert_eq!(app.ctx().mode, Mode::Simulate);
    assert!(app.ctx().doc.rocket().flights().len() >= 2, "{}", app.ctx().doc.rocket().flights().len());
}
