//! The inspector's update step against a real app: scrubbing, limits, anchors, detach, overrides, renames, staging timing.

use bowshock_model::{DeployEvent, Dim, Ignition, InternalKind, InternalPart, PartId, Rocket};

use super::edit::{Anchor, Field, Ignite, Trigger};
use super::{part, separation, Message, State};
use crate::app::{App, Message as AppMessage, Selection};

/// An app with the example loaded and an inspector state of its own (the app's is private to it).
struct Fx {
    app: App,
    state: State,
}

impl Fx {
    fn new() -> Fx {
        Fx { app: App::with_example(), state: State::new() }
    }

    fn send(&mut self, message: Message) {
        let _ = self.state.update(message, self.app.ctx_mut());
        self.app.sync();
    }

    fn undo(&mut self) {
        let _ = self.app.update(AppMessage::Undo);
    }

    fn redo(&mut self) {
        let _ = self.app.update(AppMessage::Redo);
    }

    fn rocket(&self) -> &Rocket {
        self.app.ctx().doc.rocket()
    }

    fn select(&mut self, id: PartId) {
        self.app.ctx_mut().selection = Selection::Part(id);
    }

    fn fins(&self) -> PartId {
        self.rocket().radials().iter().find(|r| r.kind.is_fin()).map(|r| r.id).unwrap()
    }

    fn internal(&self, f: impl Fn(&InternalPart) -> bool) -> PartId {
        self.rocket().internals().iter().find(|p| f(p)).map(|p| p.id).unwrap()
    }
}

#[test]
fn a_scrubbed_field_is_one_undo_step_and_commits_the_final_value() {
    let mut fx = Fx::new();
    let fin = fx.fins();
    fx.select(fin);
    let before = fx.rocket().dim(fin, Dim::RootChord).unwrap();
    for chord in [0.121, 0.123, 0.126, 0.13] {
        fx.send(Message::Edit(Field::Dim(Dim::RootChord), chord, true));
    }
    fx.send(Message::Edit(Field::Dim(Dim::RootChord), 0.13, false));
    assert_eq!(fx.rocket().dim(fin, Dim::RootChord), Some(0.13));
    fx.undo();
    assert_eq!(fx.rocket().dim(fin, Dim::RootChord), Some(before), "the whole scrub is one step");
    fx.redo();
    assert_eq!(fx.rocket().dim(fin, Dim::RootChord), Some(0.13));
}

#[test]
fn a_limited_value_is_kept_with_its_reason() {
    let mut fx = Fx::new();
    let tube = fx.rocket().parts()[1].id;
    fx.select(tube);
    fx.send(Message::Edit(Field::Dim(Dim::Length), 0.0001, false));
    assert_eq!(fx.rocket().dim(tube, Dim::Length), Some(0.002));
    assert_eq!(fx.state.limit, Some((tube, Dim::Length, "MINIMUM LENGTH")));
    // The next edit of something else forgets it.
    fx.send(Message::Edit(Field::Dim(Dim::Wall), 0.002, false));
    assert_eq!(fx.state.limit, None);
}

#[test]
fn the_offset_is_measured_from_the_chosen_anchor() {
    let mut fx = Fx::new();
    let fin = fx.fins();
    fx.select(fin);
    let (parent_length, extent, offset) = {
        let r = fx.rocket();
        let f = r.radial(fin).unwrap();
        (r.part(f.parent).unwrap().kind.length(), f.kind.extent(), f.offset)
    };
    // A fin is offered from the bottom: flush with the end of the tube it reads 0, and 5 mm forward it reads -5.
    assert_eq!(fx.state.anchor(fx.rocket(), fin), Anchor::Bottom);
    fx.send(Message::Edit(Field::Dim(Dim::Position), -0.005, false));
    let moved = fx.rocket().radial(fin).unwrap().offset;
    assert!((moved - (parent_length - extent - 0.005)).abs() < 1e-9 && moved < offset);
    fx.send(Message::Anchor(Anchor::Top));
    fx.send(Message::Edit(Field::Dim(Dim::Position), 0.2, false));
    assert!((fx.rocket().radial(fin).unwrap().offset - 0.2).abs() < 1e-9);
}

#[test]
fn detach_and_split_all_are_single_undo_steps_and_select_the_free_fin() {
    let mut fx = Fx::new();
    let fin = fx.fins();
    fx.select(fin);
    fx.send(Message::NextInstance);
    fx.send(Message::Detach);
    let free = fx.app.ctx().selection.part().unwrap();
    assert_ne!(free, fin);
    let r = fx.rocket();
    assert_eq!((r.radial(fin).unwrap().count, r.radial(free).unwrap().count), (2, 1));
    assert!(r.is_free_fin(free));
    fx.undo();
    assert_eq!(fx.rocket().radial(fin).unwrap().count, 3);
    fx.select(fin);
    fx.send(Message::SplitAll);
    assert!(fx.rocket().radials().iter().filter(|r| r.kind.is_fin()).all(|r| r.count == 1));
    fx.undo();
    assert_eq!(fx.rocket().radial(fin).unwrap().count, 3);
}

#[test]
fn overrides_start_from_the_computed_value_and_clear() {
    let mut fx = Fx::new();
    let tube = fx.rocket().parts()[1].id;
    fx.select(tube);
    let computed = fx.rocket().component_mass(tube).unwrap().mass;
    fx.send(Message::StartOverride(Field::MassOverride));
    assert_eq!(fx.rocket().part(tube).unwrap().mass_override, Some(computed));
    fx.send(Message::Edit(Field::MassOverride, 0.5, false));
    assert_eq!(fx.rocket().component_mass(tube).unwrap().mass, 0.5);
    fx.send(Message::ClearOverride(Field::MassOverride));
    assert_eq!(fx.rocket().part(tube).unwrap().mass_override, None);
    fx.send(Message::StartOverride(Field::CgOverride));
    let cg = fx.rocket().part(tube).unwrap().cg_override.unwrap();
    assert!(cg > 0.0 && cg < fx.rocket().part(tube).unwrap().kind.length());
}

#[test]
fn auto_diameters_follow_until_typed_and_reset_with_a_right_click() {
    let mut fx = Fx::new();
    let ring = fx.internal(|p| matches!(p.kind, InternalKind::CenteringRing { .. }));
    fx.select(ring);
    assert!(fx.rocket().resolved(ring).unwrap().outer_auto);
    fx.send(Message::Edit(Field::AutoDiameter, 0.05, false));
    assert!(!fx.rocket().resolved(ring).unwrap().outer_auto);
    fx.send(Message::ResetAuto(Field::AutoDiameter));
    assert!(fx.rocket().resolved(ring).unwrap().outer_auto);
}

#[test]
fn notes_and_renames_go_through_the_document() {
    let mut fx = Fx::new();
    let chute = fx.internal(|p| matches!(p.kind, InternalKind::Parachute { .. }));
    fx.select(chute);
    fx.send(Message::Note("Pack with the nomex".into()));
    assert_eq!(fx.rocket().note(chute), "Pack with the nomex");
    fx.send(Message::StartRename);
    fx.send(Message::RenameInput("Main chute".into()));
    fx.send(Message::RenameCommit);
    assert_eq!(fx.rocket().internal(chute).unwrap().name.as_deref(), Some("Main chute"));
    // Naming it as its kind returns it to the kind's name.
    fx.send(Message::StartRename);
    fx.send(Message::RenameInput("Parachute".into()));
    fx.send(Message::RenameCommit);
    assert_eq!(fx.rocket().internal(chute).unwrap().name, None);
    // Nothing selected: the rocket's own name.
    fx.app.ctx_mut().selection = Selection::None;
    fx.send(Message::StartRename);
    fx.send(Message::RenameInput("Starship".into()));
    fx.send(Message::RenameCommit);
    assert_eq!(fx.rocket().name, "Starship");
}

#[test]
fn recovery_and_motor_controls_edit_the_active_configuration_and_the_part() {
    let mut fx = Fx::new();
    let chute = fx.internal(|p| matches!(p.kind, InternalKind::Parachute { .. }));
    fx.select(chute);
    fx.send(Message::Trigger(Trigger::Altitude));
    fx.send(Message::Edit(Field::DeployAltitude, 120.0, false));
    let InternalKind::Parachute { deploy, .. } = fx.rocket().internal(chute).unwrap().kind else { unreachable!() };
    assert_eq!(deploy.event, DeployEvent::Altitude(120.0));
    let mount = fx.internal(|p| p.kind.is_motor_mount());
    fx.select(mount);
    fx.send(Message::Edit(Field::EjectionDelay, 10.0, false));
    let slot_delay = |fx: &Fx| fx.rocket().active_configuration().slot(mount).unwrap().delay;
    assert_eq!(slot_delay(&fx), 10.0);
    fx.send(Message::Plug(true));
    assert!(slot_delay(&fx).is_infinite());
    fx.send(Message::Plug(false));
    assert_eq!(slot_delay(&fx), 0.0);
}

#[test]
fn a_selected_separation_edits_the_configuration_timing() {
    let mut fx = Fx::new();
    fx.app.ctx_mut().doc.replace(separation::tests::two_stage(), None);
    fx.app.sync();
    fx.app.ctx_mut().selection = Selection::Separation(1);
    fx.app.sync();
    assert_eq!(fx.app.ctx().selection, Selection::Separation(1));
    assert!(separation::view(fx.app.ctx(), 1).is_some() && separation::view(fx.app.ctx(), 2).is_none());
    fx.send(Message::Edit(Field::SeparationDelay(1), 1.0, false));
    fx.send(Message::IgniteOn(Ignite::Separation));
    fx.send(Message::Edit(Field::IgnitionDelay, 2.5, true));
    assert_eq!(fx.rocket().active_configuration().separation_delay(1), 1.0);
    let upper = fx.app.ctx().physics.stages()[1].mounts[0];
    let ignition = |fx: &Fx| fx.rocket().active_configuration().slot(upper).unwrap().ignition;
    assert_eq!(ignition(&fx), Ignition::StageSeparation { delay: 2.5 });
    // One scrub, one undo step: back to the ignition `IgniteOn` made.
    fx.undo();
    assert_eq!(ignition(&fx), Ignition::StageSeparation { delay: 1.5 });
}

#[test]
fn every_part_has_a_first_field_and_every_view_builds() {
    let mut fx = Fx::new();
    let ids = fx.rocket().ids();
    for id in ids {
        assert!(part::first_field(fx.rocket(), id).is_some(), "part {id} has no first field");
        fx.select(id);
        // Building the element exercises every section builder (a panic here is a bug in a field).
        let _ = fx.state.view(fx.app.ctx());
    }
    assert_eq!(part::first_field(fx.rocket(), fx.rocket().parts()[0].id), Some(Field::Dim(Dim::Length)));
    assert_eq!(part::first_field(fx.rocket(), 9999), None);
    fx.app.ctx_mut().selection = Selection::None;
    let _ = fx.state.view(fx.app.ctx());
}

#[test]
fn edits_are_ignored_while_a_drag_has_the_rocket_open() {
    let mut fx = Fx::new();
    let fin = fx.fins();
    fx.select(fin);
    fx.app.ctx_mut().doc.begin_drag("Drag");
    fx.send(Message::Edit(Field::Dim(Dim::RootChord), 0.2, false));
    assert_ne!(fx.rocket().dim(fin, Dim::RootChord), Some(0.2));
    fx.app.ctx_mut().doc.end_drag(false);
}
