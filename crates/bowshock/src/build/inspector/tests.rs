//! The inspector's update step against a real app: scrubbing, limits, anchors, detach, overrides, renames, staging timing.

use bowshock_model::{DeployEvent, Dim, EventKind, InternalKind, InternalPart, PartId, Rocket, StageTrigger};

use super::edit::{Anchor, Field, StageOn, Trigger};
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
    assert_eq!(fx.rocket().active_deployment(chute).unwrap().event, DeployEvent::Altitude(120.0));
    // The configuration holds it; the part keeps its own value as the default for every other configuration.
    let InternalKind::Parachute { deploy, .. } = fx.rocket().internal(chute).unwrap().kind else { unreachable!() };
    assert_ne!(deploy.event, DeployEvent::Altitude(120.0));
    let active = fx.rocket().active_config_id();
    assert_eq!(fx.rocket().active_configuration().deployment(chute).map(|d| d.event), Some(DeployEvent::Altitude(120.0)));
    let other = fx.app.ctx_mut().doc.edit("Add configuration", |r| r.add_configuration("Other"));
    assert_eq!(fx.rocket().deployment(other, chute).unwrap().event, deploy.event);
    assert_ne!(other, active);
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
    let sep = |fx: &Fx| fx.rocket().active_configuration().event(1, EventKind::Separation);
    let ign = |fx: &Fx| fx.rocket().active_configuration().event(2, EventKind::Ignition);
    fx.send(Message::Edit(Field::StageDelay(1, EventKind::Separation), 1.0, false));
    fx.send(Message::StageOn(2, EventKind::Ignition, StageOn::Burnout));
    fx.send(Message::Edit(Field::StageDelay(2, EventKind::Ignition), 2.5, true));
    assert_eq!((sep(&fx).trigger, sep(&fx).delay), (StageTrigger::Burnout, 1.0));
    assert_eq!((ign(&fx).trigger, ign(&fx).delay), (StageTrigger::Burnout, 2.5));
    // One scrub, one undo step: back to the ignition `StageOn` made (the fixture's 1.5 s delay is kept by the switch).
    fx.undo();
    assert_eq!((ign(&fx).trigger, ign(&fx).delay), (StageTrigger::Burnout, 1.5));
    // Every trigger kind can be chosen for both events and every inspector view builds with it.
    for on in [StageOn::Launch, StageOn::Altitude, StageOn::Burnout] {
        fx.send(Message::StageOn(1, EventKind::Separation, on));
        fx.send(Message::StageOn(2, EventKind::Ignition, on));
        assert_eq!(StageOn::of(sep(&fx).trigger), on);
        assert_eq!(StageOn::of(ign(&fx).trigger), on);
        let _ = separation::view(fx.app.ctx(), 1);
        let _ = fx.state.view(fx.app.ctx());
    }
    fx.send(Message::Edit(Field::StageAltitude(1, EventKind::Separation), 80.0, false));
    assert_eq!(sep(&fx).trigger, StageTrigger::Altitude(80.0));
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

/// The example with a two-copy pod standing on its first body tube, selected.
fn with_pod() -> (Fx, PartId) {
    let mut fx = Fx::new();
    let tube = fx.rocket().parts().iter().find(|p| matches!(p.kind, bowshock_model::PartKind::BodyTube { .. })).map(|p| p.id).unwrap();
    let pod = fx
        .app
        .ctx_mut()
        .doc
        .edit("Add pod", |r| {
            let kind = r.default_pod();
            r.attach_radial(tube, 0.02, 0.0, 2, kind).ok()
        })
        .unwrap();
    fx.app.sync();
    fx.select(pod);
    (fx, pod)
}

#[test]
fn a_pod_set_edits_its_count_gap_and_position_and_has_no_symmetry_actions() {
    let (mut fx, pod) = with_pod();
    assert_eq!(part::first_field(fx.rocket(), pod), Some(Field::Dim(Dim::Gap)));
    fx.send(Message::Count(3));
    assert_eq!(fx.rocket().radial(pod).unwrap().count, 3);
    fx.send(Message::Edit(Field::Dim(Dim::Gap), 0.01, false));
    assert_eq!(fx.rocket().dim(pod, Dim::Gap), Some(0.01));
    fx.send(Message::Edit(Field::Dim(Dim::Angle), 0.5, false));
    assert!((fx.rocket().radial(pod).unwrap().angle - 0.5).abs() < 1e-9);
    // Its copies cannot be detached or split: the bar has nothing to show, and the messages change nothing.
    assert!(super::symmetry::bar(&fx.state, fx.app.ctx(), pod).is_none());
    let before = fx.rocket().clone();
    fx.send(Message::Detach);
    fx.send(Message::SplitAll);
    assert_eq!(fx.rocket().radials().len(), before.radials().len());
    // The parts list selects a part of the pod.
    let inside = fx.rocket().radial(pod).and_then(|r| r.kind.pod_body()).map(|b| b.parts()[1].id).unwrap();
    fx.send(Message::SelectPart(inside));
    assert_eq!(fx.app.ctx().selection, Selection::Part(inside));
    // The mass of the set is the total of its parts, every copy.
    assert!(fx.rocket().component_mass(pod).unwrap().mass > 0.0);
}

#[test]
fn the_parts_of_a_pod_label_their_station_order_and_pod() {
    let (mut fx, pod) = with_pod();
    let parts: Vec<PartId> = fx.rocket().radial(pod).and_then(|r| r.kind.pod_body()).map(|b| b.parts().iter().map(|p| p.id).collect()).unwrap();
    assert_eq!(part::order_label(fx.rocket(), parts[0]), "First in the pod");
    let nose_name = fx.rocket().part(parts[0]).unwrap().display_name();
    assert_eq!(part::order_label(fx.rocket(), parts[1]), format!("After {nose_name}"));
    // Core parts keep their own labels.
    let core = fx.rocket().parts()[0].id;
    assert_eq!(part::order_label(fx.rocket(), core), "First (nose)");
    // Every part inside the pod (and the pod) builds a view without panicking, with an anchor and an override.
    for id in fx.rocket().ids() {
        fx.select(id);
        let _ = fx.state.view(fx.app.ctx());
    }
    fx.select(parts[1]);
    fx.send(Message::StartOverride(Field::MassOverride));
    let one = fx.rocket().part(parts[1]).unwrap().mass_override.unwrap();
    let all = fx.rocket().component_mass(parts[1]).unwrap().mass;
    assert!((all - 2.0 * one).abs() < 1e-9 * all.max(1.0), "the override is per copy: {all} vs 2 x {one}");
    // Duplicating a tube of the pod puts the copy in the pod's stack.
    fx.send(Message::Duplicate);
    assert_eq!(fx.rocket().radial(pod).and_then(|r| r.kind.pod_body()).map(|b| b.parts().len()), Some(3));
}

#[test]
fn a_pod_count_change_reads_set_pod_count_and_a_fin_count_set_fin_count() {
    let (mut fx, pod) = with_pod();
    fx.send(Message::Count(3));
    assert_eq!(fx.app.ctx().doc.undo_label(), Some("Set pod count"));
    let fin = fx.fins();
    fx.select(fin);
    let count = fx.rocket().radial(fin).unwrap().count;
    fx.send(Message::Count(if count == 4 { 3 } else { 4 }));
    assert_eq!(fx.app.ctx().doc.undo_label(), Some("Set fin count"));
    assert_ne!(fin, pod);
}

#[test]
fn a_motor_mount_in_a_pod_belongs_to_the_stage_the_pod_stands_on() {
    let (mut fx, pod) = with_pod();
    let tube = fx.rocket().radial(pod).and_then(|r| r.kind.pod_body()).map(|b| b.parts()[1].id).unwrap();
    let kind = InternalKind::InnerTube { outer_diameter: 0.03, thickness: 0.002, length: 0.2, motor_mount: true, overhang: 0.0 };
    let mount = fx.app.ctx_mut().doc.edit("Add mount", |r| r.attach_internal(tube, 0.02, kind).ok()).unwrap();
    fx.app.sync();
    let found = fx.rocket().motor_mounts().into_iter().find(|m| m.id == mount).unwrap();
    assert_eq!((found.pod, found.copies), (Some(pod), 2));
    assert!(fx.rocket().stages().iter().all(|s| !s.mounts.contains(&mount)), "a pod's mount is not a stage's");
    assert_eq!(fx.state.stage_of_mount(fx.rocket(), mount), fx.rocket().stage_of(mount).unwrap());
    // Without and with a motor the inspector builds, and the check names the empty mount / the fit.
    fx.select(mount);
    let _ = fx.state.view(fx.app.ctx());
    let config = fx.rocket().active_config_id();
    let motor = bowshock_model::MotorRef::new("H128W", "Aerotech");
    fx.app.ctx_mut().doc.edit("Set motor", |r| r.set_motor(config, mount, Some(motor)).ok());
    fx.app.sync();
    let _ = fx.state.view(fx.app.ctx());
    let ctx = fx.app.ctx();
    let checks = super::health::motor(fx.rocket(), mount, ctx.physics.motors(), ctx.physics.warnings(), ctx);
    assert!(!checks.is_empty());
}

#[test]
fn the_rocket_inspector_sets_and_clears_the_active_configurations_ballast() {
    let mut fx = Fx::new();
    fx.send(Message::BallastMass(0.05));
    let ballast = fx.rocket().active_configuration().ballast.expect("a mass makes a ballast");
    assert_eq!(ballast.mass, 0.05);
    fx.send(Message::BallastStation(0.4));
    assert_eq!(fx.rocket().active_configuration().ballast.map(|b| b.station), Some(0.4));
    fx.send(Message::BallastMass(0.0));
    assert!(fx.rocket().active_configuration().ballast.is_none(), "zero removes it");
}

#[test]
fn a_fillet_stays_closed_until_it_is_opened_and_follows_the_user_after() {
    let mut fx = Fx::new();
    let fin = fx.fins();
    fx.select(fin);
    assert!(fx.state.fillet_open.is_none());
    fx.send(Message::FilletOpen(true));
    assert_eq!(fx.state.fillet_open, Some((fin, true)));
}
