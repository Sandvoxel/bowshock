#![allow(unused_must_use)] // The tests drive `update` directly and have no runtime to hand its tasks to.

use std::path::PathBuf;

use iced::keyboard;

use super::*;
use crate::app::settings::Settings;
use crate::app::{App, KeyPress};
use crate::units::Family;

fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("bowshock_prefs_{}_{name}.json", std::process::id()))
}

fn open_app(name: &str) -> (App, PathBuf) {
    let file = scratch(name);
    let _ = std::fs::remove_file(&file);
    let mut app = App::new();
    app.ctx_mut().settings_file = Some(file.clone());
    (app, file)
}

fn key(app: &mut App, key: keyboard::Key, modifiers: keyboard::Modifiers) {
    app.update(app::Message::Key(KeyPress { key, modifiers, captured: false }));
}

#[test]
fn the_list_has_the_four_sections_in_order() {
    let labels: Vec<_> = Section::ALL.iter().map(|s| s.label()).collect();
    assert_eq!(labels, ["Appearance", "Units", "Compute", "Storage"]);
}

#[test]
fn navigation_opens_on_a_page_and_remembers_it() {
    let mut ctx = Ctx::new();
    let mut state = State::new();
    assert!(!state.is_open());
    state.update(Message::Show, &mut ctx);
    assert!(state.is_open());
    assert_eq!(state.section(), Section::Appearance, "the first page opens first");
    state.update(Message::Select(Section::Units), &mut ctx);
    assert_eq!(state.section(), Section::Units);
    state.update(Message::Close, &mut ctx);
    assert!(!state.is_open());
    state.update(Message::Show, &mut ctx);
    assert_eq!(state.section(), Section::Units, "it comes back to the page it was left on");
    state.update(Message::Open(Section::Compute), &mut ctx);
    assert!(state.is_open());
    assert_eq!(state.section(), Section::Compute, "Open goes to the named page");
    state.update(Message::Nothing, &mut ctx);
    assert_eq!(state.section(), Section::Compute);
}

#[test]
fn leaving_storage_drops_an_unanswered_clear() {
    let mut ctx = Ctx::new();
    let mut state = State::new();
    state.update(Message::Open(Section::Storage), &mut ctx);
    state.update(Message::Storage(storage::Message::Ask(storage::Kind::Gpu)), &mut ctx);
    assert!(state.view(&ctx).is_some());
    state.update(Message::Select(Section::Units), &mut ctx);
    state.update(Message::Select(Section::Storage), &mut ctx);
    assert!(state.storage_confirm().is_none(), "the question does not wait on the page");
}

#[test]
fn the_theme_is_kept() {
    let (mut app, file) = open_app("theme");
    assert!(app.ctx().dark);
    app.update(app::Message::Prefs(Message::Show));
    app.update(app::Message::Prefs(Message::Theme(false)));
    assert!(!app.ctx().dark);
    assert!(!Settings::load(Some(&file)).dark, "Light is in the settings file");
    app.update(app::Message::Prefs(Message::Theme(true)));
    assert!(app.ctx().dark);
    assert!(Settings::load(Some(&file)).dark, "Dark is kept too");
    app.update(app::Message::Prefs(Message::Close));
    let _ = std::fs::remove_file(&file);
}

#[test]
fn the_unit_set_and_the_solver_are_kept() {
    let (mut app, file) = open_app("units");
    let mix = UnitSystem::MM_G_S.with(Family::Length, "in").with(Family::Mass, "kg");
    app.update(app::Message::Prefs(Message::Show));
    app.update(app::Message::Prefs(Message::Units(mix)));
    app.update(app::Message::Prefs(Message::Solver(LocalSolver::Cpu)));
    assert_eq!(app.ctx().units, mix);
    let saved = Settings::load(Some(&file));
    assert_eq!(saved.units, mix);
    assert_eq!(saved.local_solver, LocalSolver::Cpu);
    assert!(saved.dark, "what was not touched is kept as it was");
    let _ = std::fs::remove_file(&file);
}

#[test]
fn the_window_takes_the_screen_until_it_closes() {
    let (mut app, file) = open_app("modal");
    key(&mut app, keyboard::Key::Character(",".into()), keyboard::Modifiers::COMMAND);
    assert!(app.prefs().is_open(), "Cmd/Ctrl+, opens it");
    // Commands of the page behind are blocked, typing is not a shortcut, Escape closes.
    let revision = app.ctx().doc.revision();
    app.update(app::Message::New);
    app.update(app::Message::Mode(crate::app::Mode::Analyze));
    assert_eq!(app.ctx().mode, crate::app::Mode::Build);
    assert_eq!(app.ctx().doc.revision(), revision);
    app.update(app::Message::Key(KeyPress {
        key: keyboard::Key::Named(keyboard::key::Named::Escape),
        modifiers: keyboard::Modifiers::empty(),
        captured: false,
    }));
    assert!(!app.prefs().is_open());
    app.update(app::Message::Mode(crate::app::Mode::Analyze));
    assert_eq!(app.ctx().mode, crate::app::Mode::Analyze, "the page answers again once it is closed");
    let _ = std::fs::remove_file(&file);
}

#[test]
fn every_page_draws() {
    let mut ctx = Ctx::new();
    let mut state = State::new();
    for section in Section::ALL {
        state.update(Message::Open(section), &mut ctx);
        assert!(state.view(&ctx).is_some(), "{section:?}");
    }
}
