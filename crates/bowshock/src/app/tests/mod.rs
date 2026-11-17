#![allow(unused_must_use)] // The tests drive `update` directly and have no runtime to hand its tasks to.

mod carry;
mod commands;
mod edit;
mod placing;
mod pods;
mod preview;
mod view;

use super::message::shortcut;
use super::{drag, live, part_menu, scale, settings, ui};
use super::{App, KeyPress, Message, Pending, Selection};
use crate::build::{inspector, parts_bin};
use crate::chrome;
use crate::units::UnitSystem;
use crate::viewport::manipulate::Edit;
use crate::viewport::pick::{Pick, SurfaceTarget};
use crate::viewport::Message as Vp;
use bowshock_model::{Dim, NoseShape, PartId, PartKind, RadialKind};
use iced::{keyboard, mouse, Point};

/// An app holding a nose, a tube and a transition (fore to aft).
pub(super) fn app() -> (App, [PartId; 3]) {
    let mut app = App::new();
    let ctx = &mut app.ctx;
    drag::place(ctx, 0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1, tip_radius: 0.0 });
    drag::place(ctx, 1, PartKind::BodyTube { length: 0.5, diameter: 0.1 });
    drag::place(ctx, 2, PartKind::Transition { length: 0.1, fore_diameter: 0.1, aft_diameter: 0.06 });
    app.after_update();
    let ids = [0, 1, 2].map(|i| app.ctx.doc.rocket().parts()[i].id);
    (app, ids)
}

pub(super) fn vp(app: &mut App, message: Vp) {
    app.update(Message::Viewport(message));
}

/// The app with the viewport fitted to 900 × 560 and its animation landed.
pub(super) fn fitted() -> (App, [PartId; 3]) {
    let (mut app, ids) = app();
    land(&mut app, Vp::Fit(iced::Size::new(900.0, 560.0)));
    (app, ids)
}

/// Sends `message` to the viewport and runs any camera animation it started to its end.
pub(super) fn land(app: &mut App, message: Vp) {
    vp(app, message);
    app.viewport.tick(std::time::Instant::now() + std::time::Duration::from_secs(5));
}

/// Lengthens the nose cone from 0.2 m to 1.5 m, which pushes the rocket's far end out of a view fitted to the short one
/// (stations run from the tip, so a longer nose moves everything behind it aft).
pub(super) fn lengthen_nose(app: &mut App, nose: PartId) {
    app.ctx.doc.edit("Lengthen nose", |r| r.set_dim(nose, Dim::Length, 1.5));
    app.after_update();
}

pub(super) fn parts(app: &App) -> Vec<bowshock_model::Part> {
    app.ctx.doc.rocket().parts().to_vec()
}

pub(super) fn target(parent: PartId, offset: f64, angle: f64) -> SurfaceTarget {
    SurfaceTarget { parent, offset, angle, copy: 0 }
}

/// A pod of one bare tube (Ø30 × 300 mm), three copies, on core part `tube`. Returns the pod set and its tube.
pub(super) fn bare_pod(app: &mut App, tube: PartId) -> (PartId, PartId) {
    let body = bowshock_model::PodBody::single(bowshock_model::Part::new(0, PartKind::BodyTube { length: 0.3, diameter: 0.03 }), vec![], vec![]);
    let pod = app.ctx.doc.edit("Pod", |r| r.attach_radial(tube, 0.1, 0.0, 3, RadialKind::Pod { gap: 0.0, body }).unwrap());
    let part = app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().parts()[0].id;
    (pod, part)
}

pub(super) fn pod_kinds(app: &App, pod: PartId) -> Vec<PartKind> {
    app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().parts().iter().map(|p| p.kind.clone()).collect()
}

/// Why each of the bin's nose rows (every nose variant) is unavailable, `None` for an available one.
pub(super) fn nose_reasons(app: &App) -> Vec<Option<String>> {
    (0..5).map(|i| app.parts_bin.row_unavailable(parts_bin::RowId::new(parts_bin::Category::Airframe, i), &app.ctx)).collect()
}

/// An example with an unsaved edit, New pressed: the modal is up holding New.
pub(super) fn dirty_app() -> App {
    let mut app = App::new();
    app.update(Message::Example);
    app.ctx.doc.edit("Rename", |r| r.name = "changed".to_string());
    app.update(Message::New);
    app
}

/// The tube with a fin set on it at `offset`, and the fins' id.
pub(super) fn finned(app: &mut App, tube: PartId, offset: f64) -> PartId {
    let id = app.ctx.doc.edit("Fins", |r| {
        r.attach_radial(tube, offset, 0.0, 3, RadialKind::FinSet { root_chord: 0.08, tip_chord: 0.04, span: 0.06, sweep: 0.03, thickness: 0.003 }).unwrap()
    });
    app.after_update();
    id
}

pub(super) fn committed(app: &App) -> bowshock_sim::from_model::Balance {
    *app.ctx.physics.balance().expect("the fixture has a balance")
}

/// A motor mount that was put in the nose cone by mistake (placed there by the model; the nose's bore is too narrow for it).
pub(super) fn mount_in_the_nose(app: &mut App, [nose, tube, _]: [PartId; 3]) -> PartId {
    let kind = bowshock_model::InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.002, length: 0.15, motor_mount: true, overhang: 0.0 };
    let mount = app.ctx.doc.edit("Mount", |r| {
        let id = r.attach_internal(tube, 0.0, kind).unwrap();
        r.move_internal(id, nose, 0.03).unwrap();
        id
    });
    app.after_update();
    mount
}
