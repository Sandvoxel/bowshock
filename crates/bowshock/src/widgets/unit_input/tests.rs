use super::cells::{cells_from, cells_text, open_brace};
use super::logic::scrub;
use super::*;

#[test]
fn a_mixed_value_shows_no_number_and_focusing_starts_empty() {
    let m = Memory::default();
    let plain: UnitInput<'_, ()> = unit_input("Length", 0.1, Quantity::Length, &UnitSystem::MM_G_S);
    assert_eq!(plain.display(&m), "100.00");
    let mixed: UnitInput<'_, ()> = unit_input("Length", 0.1, Quantity::Length, &UnitSystem::MM_G_S).mixed(true);
    // The text input shows its `—` placeholder over an empty value; an edit starts from nothing.
    assert_eq!(mixed.display(&m), "");
}

#[test]
fn scrub_moves_one_display_step_per_pixel() {
    // Section 4 of design/README.md: mm shows two decimals, so one pixel is 0.01 mm.
    let step = units::Quantity::Length.unit_for(units::UnitSystem::MM_G_S).step();
    let (_, value) = scrub(100.0, 5.0, step, keyboard::Modifiers::default());
    assert!((value - 100.05).abs() < 1e-9);
    let (_, value) = scrub(100.0, -3.0, step, keyboard::Modifiers::default());
    assert!((value - 99.97).abs() < 1e-9);
}

#[test]
fn shift_is_a_tenth_and_ctrl_is_ten_times() {
    let (_, fine) = scrub(0.0, 10.0, 0.01, keyboard::Modifiers::SHIFT);
    assert!((fine - 0.01).abs() < 1e-9, "{fine}");
    let (_, coarse) = scrub(0.0, 10.0, 0.01, keyboard::Modifiers::CTRL);
    assert!((coarse - 1.0).abs() < 1e-9, "{coarse}");
}

#[test]
fn fine_scrub_accumulates_below_the_display_step() {
    // Ten Shift-pixels are one step; the unquantised value must carry the fraction between events.
    let mut shown = 0.0;
    for _ in 0..10 {
        shown = scrub(shown, 1.0, 0.01, keyboard::Modifiers::SHIFT).0;
    }
    assert!((shown - 0.01).abs() < 1e-9);
}

#[test]
fn text_with_references_becomes_cells_and_back() {
    let text = "0.5 * {body_tube#3.od} + 2";
    let cells = cells_from(text);
    assert_eq!(cells.iter().filter(|c| matches!(c, Cell::Chip(t) if t == "body_tube#3.od")).count(), 1);
    assert_eq!(cells.len(), "0.5 * ".len() + 1 + " + 2".len());
    assert_eq!(cells_text(&cells), text);
    // An unclosed brace stays a character.
    assert!(cells_from("2 * {bo").iter().all(|c| matches!(c, Cell::Char(_))));
    assert_eq!(cells_text(&cells_from("{a} + {b")), "{a} + {b");
}

#[test]
fn an_open_brace_is_pick_mode() {
    let open = |s: &str| open_brace(&cells_from(s));
    assert_eq!(open("0.5 * {"), Some(6));
    assert_eq!(open("{a#1.od} + {"), Some(4));
    assert_eq!(open("{a#1.od} + 2"), None);
    assert_eq!(open("2*38"), None);
    // A slot is where a chip waits to be re-picked, wherever it sits.
    let cells = vec![Cell::Char('2'), Cell::Slot(Some("a#1.od".into())), Cell::Char('+')];
    assert_eq!(open_brace(&cells), Some(1));
    assert_eq!(cells_text(&cells), "2{+");
}

#[test]
fn badges_read_as_the_kit_labels_them() {
    assert_eq!(Badge::Refs(2).label(), "→ 2");
    assert_eq!(Badge::PerConfig.label(), "PER CONFIG");
    assert_eq!(Badge::Override.label(), "OVR");
}
