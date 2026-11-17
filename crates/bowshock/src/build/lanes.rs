//! Station Lanes: the canvas under the viewport that lines every part up with the model by station (design/components/
//! StationLanes). One lane per system, every part a bar at its real position, CG/CP through all of them.
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`] and [`State::on_key`] with the shared [`Ctx`]. Layout of the module:
//!
//! * `scene`: the rocket as bars in stations (sub-rows, stages, mass profile, drag-free data, unit-tested);
//! * `geometry`: stations to pixels (fit, zoom/pan), ruler ticks, hit-testing;
//! * `edit`: the document edits (move, resize, landing targets, snapping);
//! * `canvas`: drawing and mouse input;
//! * `pods`: the `Pod ×N` brackets of a pod's parts and their tags (never dropped: beside the ends, along the bracket, in another
//!   row, or in a row the lane gains);
//! * `menu`: the "Add here…" menu.
//!
//! Edits go through `ctx.doc` like every other surface's: moves and resizes merge into one undo step per drag, a part from the
//! bin or an airframe part picked up in the lanes is a `Drags` drag finished with the shared `app::drag` functions.

mod canvas;
mod edit;
mod geometry;
mod menu;
mod pods;
mod scene;
pub(crate) use scene::remaining_copies;

use std::cell::RefCell;

use bowshock_model::{PartId, System};
use bowshock_sim::from_model::{self, Phase};
use iced::widget::{canvas as canvas_widget, container, stack};
use iced::{keyboard, Element, Fill, Point, Task, Widget};

use crate::app::drag;
use crate::app::physics::BALANCE_MACH;
use crate::app::symmetry::Rejoin;
use crate::app::{self, Ctx, Selection};
use crate::chrome;
use crate::tokens::LANE_H;
use crate::viewport;
use crate::viewport::pick::{Pick, SurfaceTarget};
use canvas::{Carry, Lanes};
use edit::End;
use geometry::{Zoom, RULER_H};
use scene::{BarKey, Inputs, Marks, OtherCg, Scene};

/// Height of the lanes pane: fixed, whatever is in it ([`LANES_H`](crate::tokens::LANES_H)), or just the ruler when collapsed.
pub fn height(collapsed: bool) -> f32 {
    if collapsed {
        RULER_H
    } else {
        crate::tokens::LANES_H
    }
}

/// Station Lanes' messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// A click: select the part of a bar (a motor selects its mount), or nothing.
    Select(Option<BarKey>),
    /// A Shift-click or Cmd/Ctrl-click on a bar: add its part to the selection, or take it out.
    Toggle(BarKey),
    /// A click on a separation line.
    SelectSeparation(usize),
    /// A separation line dragged: it now sits at the joint just aft of this airframe part. The first one opens the drag's one
    /// undo step.
    SeparationTo(usize, Option<PartId>),
    /// The separation line was released (`true`: keep it as one undo step) or Esc was pressed (`false`: put it back).
    SeparationEnd(bool),
    /// Right-click on a bar: select its part (unless it is already selected) and open the part's context menu.
    PartMenu(BarKey),
    /// A click on the current configuration's motor bar: open the configuration menu.
    OpenConfigMenu,
    /// A double click: select and focus the viewport on the part.
    Focus(PartId),
    /// A bar dragged along the length: its fore edge is now at `start` (station, metres).
    MoveTo {
        /// The part.
        id: PartId,
        /// Station of its fore edge.
        start: f64,
    },
    /// One end of a bar dragged to `station`.
    ResizeTo {
        /// The part.
        id: PartId,
        /// Which end.
        end: End,
        /// Where it is now.
        station: f64,
    },
    /// An airframe part picked up by dragging its bar.
    GrabAxial(PartId),
    /// A carried part (from the bin, or picked up here) is over the lanes with its fore edge at this station.
    CarryAt(f64),
    /// The button was released: lands a carried part, or ends a move or resize.
    EditEnd,
    /// A free part was released on a symmetry group's bar: offer to rejoin it (the inspector asks whose settings win).
    DropOnGroup {
        /// The free part.
        part: PartId,
        /// The group whose bar it was dropped on.
        group: PartId,
    },
    /// The user zoomed or panned the lanes (the new zoom of the whole-rocket fit).
    SetZoom(Zoom),
    /// Back to the whole rocket.
    ResetView,
    /// Right click on empty lane space: open the add menu there.
    Menu {
        /// Where, in the pane.
        at: Point,
        /// The lane.
        system: System,
        /// The station clicked.
        station: f64,
        /// Width of the pane, so the menu stays inside it.
        width: f32,
    },
    /// An entry of the add menu.
    Add(usize),
    /// `L`: collapse to the ruler / expand.
    ToggleCollapsed,
    /// `M`: pin the mass shading.
    TogglePinned,
}

/// The cached CGs of the other configurations.
#[derive(Debug, Default)]
struct OtherCache {
    key: Option<(u64, Phase, u64)>,
    cgs: Vec<OtherCg>,
}

/// Station Lanes' state.
#[derive(Debug, Default)]
pub struct State {
    collapsed: bool,
    pinned: bool,
    /// The user's zoom and pan of the whole-rocket fit.
    zoom: Zoom,
    menu: Option<menu::Open>,
    others: RefCell<OtherCache>,
}

impl State {
    /// An empty lanes state: the whole rocket, expanded.
    pub fn new() -> Self {
        State::default()
    }

    /// Handles one of the lanes' messages. Edits go through `ctx.doc`; selection through `ctx.selection`.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        if !matches!(message, Message::Menu { .. } | Message::Add(_)) {
            self.menu = None;
        }
        match message {
            Message::Select(key) => ctx.select_one(key.map(BarKey::part)),
            Message::Toggle(key) => ctx.toggle_part(key.part()),
            Message::SelectSeparation(stage) => ctx.selection = Selection::Separation(stage),
            Message::OpenConfigMenu => return Task::done(app::Message::Chrome(chrome::Message::ToggleConfigMenu)),
            // A double click selects the part and turns the viewport square-on to it.
            Message::Focus(id) => return Task::done(app::Message::Viewport(viewport::Message::Focus(id))),
            Message::MoveTo { id, start } => match edit::refusal(ctx.doc.rocket(), id, start) {
                Some(reason) => ctx.notify(reason),
                None => {
                    ctx.doc.edit_merging("Move part", &format!("move:{id}"), |r| edit::move_part(r, id, start));
                }
            },
            Message::ResizeTo { id, end, station } => {
                ctx.doc.edit_merging("Resize part", &format!("resize:{id}"), |r| edit::resize_part(r, id, end, station));
            }
            Message::GrabAxial(id) => drag::grab(ctx, Pick::Axial(id)),
            Message::CarryAt(start) => carry_at(ctx, start),
            Message::SeparationTo(stage, joint) => drag::move_separation(ctx, stage, joint),
            Message::SeparationEnd(commit) => drag::finish_separation(ctx, commit),
            Message::EditEnd => {
                drag::finish_axial(ctx, true);
                drag::finish_radial(ctx, true);
                drag::finish_internal(ctx, true);
                ctx.doc.seal();
            }
            Message::DropOnGroup { part, group } => {
                // The move ended on a group's bar: seal it, then ask (in the inspector) whose settings win.
                ctx.doc.seal();
                if ctx.doc.rocket().can_rejoin(part, group) {
                    ctx.selection = Selection::Part(part);
                    ctx.symmetry.instance = None;
                    ctx.symmetry.rejoin = Some(Rejoin { part, group });
                }
            }
            Message::SetZoom(zoom) => self.zoom = zoom,
            Message::ResetView => self.zoom = Zoom::default(),
            Message::PartMenu(key) => {
                let part = key.part();
                if !ctx.selected_parts().contains(&part) {
                    ctx.select_one(Some(part));
                    ctx.symmetry.instance = None;
                }
                return Task::done(app::Message::OpenPartMenu(part));
            }
            Message::Menu { at, system, station, width } => self.menu = Some(menu::Open { at, system, station, width }),
            Message::Add(index) => {
                if let Some(open) = self.menu.take() {
                    menu::add(ctx, open.system, open.station, index);
                }
            }
            Message::ToggleCollapsed => self.collapsed = !self.collapsed,
            Message::TogglePinned => self.pinned = !self.pinned,
        }
        Task::none()
    }

    /// The shortcut `key` maps to while the lanes are on screen, if any. Only called for keys no widget handled.
    pub fn on_key(&self, key: &keyboard::Key, modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        if !modifiers.is_empty() {
            return None;
        }
        let keyboard::Key::Character(c) = key else { return None };
        match c.as_str() {
            "l" | "L" => Some(Message::ToggleCollapsed),
            "m" | "M" => Some(Message::TogglePinned),
            _ => None,
        }
    }

    /// The CG of every configuration but the current one, from the cache (recomputed when the document or phase changed).
    fn other_cgs(&self, ctx: &Ctx) -> Vec<OtherCg> {
        let rocket = ctx.doc.rocket();
        let key = (ctx.doc.revision(), ctx.physics.phase(), rocket.active_config_id());
        let mut cache = self.others.borrow_mut();
        if cache.key != Some(key) {
            cache.cgs = rocket
                .configurations()
                .iter()
                .filter(|c| c.id != rocket.active_config_id())
                .filter_map(|c| {
                    let mut copy = rocket.clone();
                    copy.select_configuration(c.id);
                    let balance = from_model::balance(&copy, ctx.physics.motors(), ctx.physics.phase(), BALANCE_MACH, 0.0, true).ok()?;
                    Some(OtherCg { name: rocket.configuration_name(c.id).unwrap_or_else(|| c.display_name()), cg: balance.cg })
                })
                .collect();
            cache.key = Some(key);
        }
        cache.cgs.clone()
    }

    /// The scene on show: the document's bars with the physics cache's marks.
    fn scene(&self, ctx: &Ctx, carry: Option<&Carry>) -> Scene {
        let rocket = ctx.doc.rocket();
        let mut scene = Scene::build(&Inputs {
            rocket,
            stages: ctx.physics.stages(),
            phase: ctx.physics.phase(),
            motors: ctx.physics.motors(),
            units: ctx.units,
            compare: ctx.compare_configs,
        });
        let shown = ctx.shown();
        scene.marks = shown.now.map(|b| Marks { cg: b.cg, cp: b.cp, margin: b.margin_cal });
        scene.was = shown.committed.map(|b| (b.cg, b.cp));
        if ctx.compare_configs {
            scene.other_cgs = self.other_cgs(ctx);
        }
        if let Some(carry) = carry {
            scene.ensure_lane(carry.system);
        }
        scene
    }

    /// The lanes pane, [`height`] high. It always shows the whole rocket (nose tip to aft end) until the user zooms or pans it.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let carry = carry_of(ctx);
        let scene = self.scene(ctx, carry.as_ref());
        let h = height(self.collapsed);
        let lanes = Lanes {
            scene,
            rocket: ctx.doc.rocket(),
            zoom: self.zoom,
            units: ctx.units,
            selection: ctx.selection,
            parts: ctx.selected_parts(),
            flash: ctx.flash_levels().map(|(id, level, _)| (id, level)),
            show: (ctx.show_cg, ctx.show_cp),
            carry,
            pinned: self.pinned,
            collapsed: self.collapsed,
            lane_h: LANE_H,
            drive_modifiers: ctx.env.drive_modifiers,
        };
        let drawing = canvas_widget(lanes).width(Fill).height(h);
        let body: Element<'a, Message> = match (&self.menu, self.collapsed) {
            (Some(open), false) => stack![drawing, menu::view(open, ctx)].boxed(),
            _ => drawing.boxed(),
        };
        container(body).width(Fill).height(h).boxed()
    }
}

/// The part carried over the lanes (from the parts bin, or picked up here), as the ghost needs it.
fn carry_of(ctx: &Ctx) -> Option<Carry> {
    let rocket = ctx.doc.rocket();
    let drag = &ctx.drag;
    // The margin of the rocket as it would be if the part were released here.
    let margin = ctx.shown().now.map(|b| b.margin_cal).filter(|_| drag.landing().is_some());
    if let Some(d) = &drag.axial {
        let landing = d.snap.and_then(|n| rocket.stations().get(n).copied());
        return Some(Carry { name: d.name.to_string(), system: d.kind.system(), extent: d.kind.length(), landing, margin });
    }
    if let Some(d) = &drag.radial {
        let landing = d.target.and_then(|t| Some(rocket.part_station(t.parent)? + t.offset));
        return Some(Carry { name: d.kind.name().to_string(), system: d.kind.system(), extent: d.kind.extent(), landing, margin });
    }
    if let Some(d) = &drag.internal {
        let landing = d.target.and_then(|t| Some(rocket.part_station(t.parent)? + t.offset));
        return Some(Carry { name: d.name.to_string(), system: d.kind.system(), extent: d.kind.extent(), landing, margin });
    }
    None
}

/// A carried part is over the lanes with its fore edge at `start`: sets where it would land.
fn carry_at(ctx: &mut Ctx, start: f64) {
    let rocket = ctx.doc.rocket();
    if let Some(d) = &ctx.drag.axial {
        let node = edit::nearest_node(rocket, &d.kind, start).map(|(i, _)| i);
        if let Some(d) = ctx.drag.axial.as_mut() {
            d.snap = node;
        }
    } else if let Some(d) = &ctx.drag.radial {
        let angle = d.origin.as_ref().map(|o| o.angle).or(d.target.map(|t| t.angle)).unwrap_or(0.0);
        let target = edit::radial_landing(rocket, &d.kind, start).map(|(parent, offset)| SurfaceTarget { parent, offset, angle, copy: 0 });
        if let Some(d) = ctx.drag.radial.as_mut() {
            d.target = target;
        }
    } else if let Some(d) = &ctx.drag.internal {
        let target = edit::internal_landing(rocket, d.kind.extent(), start).map(|(parent, offset)| drag::InternalTarget { parent, offset });
        if let Some(d) = ctx.drag.internal.as_mut() {
            d.target = target;
        }
    }
}

#[cfg(test)]
#[allow(unused_must_use)] // The tests drive `update` directly and have no runtime to hand its tasks to.
mod tests {
    use super::*;
    use crate::app::{App, Message as App_};
    use bowshock_model::{InternalKind, MassRole, PartKind};
    use geometry::Axis;

    fn lanes(app: &mut App, message: Message) {
        app.update(App_::Lanes(message));
    }

    /// The example with the first ballast / mass component's id.
    fn example() -> (App, PartId) {
        let app = App::with_example();
        let id = app
            .ctx()
            .doc
            .rocket()
            .internals()
            .iter()
            .find(|p| matches!(p.kind, InternalKind::MassComponent { role: MassRole::Ballast, .. }))
            .expect("the example has ballast")
            .id;
        (app, id)
    }

    #[test]
    fn a_click_selects_a_part_a_motor_selects_its_mount_and_empty_space_deselects() {
        let (mut app, ballast) = example();
        lanes(&mut app, Message::Select(Some(BarKey::Part(ballast))));
        assert_eq!(app.ctx().selection, Selection::Part(ballast));
        let mount = app.ctx().doc.rocket().motor_mounts()[0].id;
        let config = app.ctx().doc.rocket().active_config_id();
        lanes(&mut app, Message::Select(Some(BarKey::Motor { mount, config })));
        assert_eq!(app.ctx().selection, Selection::Part(mount));
        lanes(&mut app, Message::SelectSeparation(1));
        // No separation in a one-stage rocket: the selection is dropped by the app.
        assert_eq!(app.ctx().selection, Selection::None);
        lanes(&mut app, Message::Select(None));
        assert_eq!(app.ctx().selection, Selection::None);
    }

    #[test]
    fn moving_a_bar_is_one_undo_step_and_stops_at_the_tube_walls() {
        let (mut app, ballast) = example();
        let before = app.ctx().doc.rocket().clone();
        for start in [0.30, 0.35, 0.40] {
            lanes(&mut app, Message::MoveTo { id: ballast, start });
        }
        assert!((app.ctx().doc.rocket().part_station(ballast).unwrap() - 0.40).abs() < 1e-9);
        // Far past the tube's aft wall: it stops there.
        lanes(&mut app, Message::MoveTo { id: ballast, start: 5.0 });
        let rocket = app.ctx().doc.rocket();
        let (parent, extent) = (rocket.internal(ballast).unwrap().parent, rocket.internal(ballast).unwrap().kind.extent());
        assert!((rocket.part_span(ballast).unwrap().1 - rocket.part_span(parent).unwrap().1).abs() < 1e-9 && extent > 0.0);
        lanes(&mut app, Message::EditEnd);
        app.update(App_::Undo);
        assert_eq!(app.ctx().doc.rocket(), &before);
    }

    #[test]
    fn dragging_an_end_changes_the_length_in_one_step() {
        let (mut app, ballast) = example();
        let before = app.ctx().doc.rocket().clone();
        let (fore, aft) = app.ctx().doc.rocket().part_span(ballast).unwrap();
        for station in [aft + 0.01, aft + 0.02, aft + 0.03] {
            lanes(&mut app, Message::ResizeTo { id: ballast, end: edit::End::Aft, station });
        }
        lanes(&mut app, Message::EditEnd);
        let (f, a) = app.ctx().doc.rocket().part_span(ballast).unwrap();
        assert!((f - fore).abs() < 1e-9 && (a - (aft + 0.03)).abs() < 1e-9);
        app.update(App_::Undo);
        assert_eq!(app.ctx().doc.rocket(), &before);
    }

    #[test]
    fn a_right_click_on_a_bar_selects_its_part_unless_it_is_already_selected() {
        let (mut app, ballast) = example();
        lanes(&mut app, Message::PartMenu(BarKey::Part(ballast)));
        assert_eq!(app.ctx().selection, Selection::Part(ballast));
        // Right-clicking another bar moves the selection to it.
        let mount = app.ctx().doc.rocket().motor_mounts()[0].id;
        lanes(&mut app, Message::PartMenu(BarKey::Part(mount)));
        assert_eq!(app.ctx().selection, Selection::Part(mount));
    }

    #[test]
    fn the_add_menu_adds_that_systems_part_at_the_station_clicked_and_selects_it() {
        let (mut app, _) = example();
        let before = app.ctx().doc.rocket().internals().len();
        let entries = menu::entries(System::Avionics);
        assert!(!entries.is_empty() && entries.iter().all(|e| matches!(e, menu::Entry::Internal(_, k) if k.system() == System::Avionics)));
        let battery = entries.iter().position(|e| e.name() == "Battery").unwrap();
        lanes(&mut app, Message::Menu { at: Point::new(300.0, 100.0), system: System::Avionics, station: 0.4, width: 700.0 });
        lanes(&mut app, Message::Add(battery));
        let rocket = app.ctx().doc.rocket();
        assert_eq!(rocket.internals().len(), before + 1);
        let added = rocket.internals().last().unwrap();
        assert_eq!(app.ctx().selection, Selection::Part(added.id));
        // Centred on the station clicked.
        let (fore, aft) = rocket.part_span(added.id).unwrap();
        assert!(((fore + aft) / 2.0 - 0.4).abs() < 1e-9);
        // The structure lane offers airframe parts too, the aero lane fins.
        assert!(menu::entries(System::Structure).iter().any(|e| matches!(e, menu::Entry::Axial(..))));
        assert!(menu::entries(System::Aero).iter().any(|e| matches!(e, menu::Entry::Radial(..))));
    }

    #[test]
    fn a_part_carried_from_the_bin_lands_where_the_lanes_say_and_goes_back_with_no_landing() {
        let (mut app, _) = example();
        let rings = app.ctx().doc.rocket().internals().len();
        let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
        drag::start_internal(app.ctx_mut(), Some(0), "Centering ring", ring);
        lanes(&mut app, Message::CarryAt(0.45));
        let carry = carry_of(app.ctx()).expect("a carried part");
        assert_eq!((carry.system, carry.landing), (System::Structure, Some(0.45)));
        lanes(&mut app, Message::EditEnd);
        assert_eq!(app.ctx().doc.rocket().internals().len(), rings + 1);
        assert!(app.ctx().drag.internal.is_none());

        // A fin set carried over the lanes lands on the tube under its station.
        let radials = app.ctx().doc.rocket().radials().len();
        let fins = bowshock_model::RadialKind::LaunchLug { length: 0.05, diameter: 0.01 };
        drag::start_radial(app.ctx_mut(), 0, fins);
        lanes(&mut app, Message::CarryAt(0.5));
        lanes(&mut app, Message::EditEnd);
        assert_eq!(app.ctx().doc.rocket().radials().len(), radials + 1);

        // Released over the lanes before it ever had a landing: nothing is added.
        let before = app.ctx().doc.rocket().clone();
        let ring = InternalKind::Bulkhead { diameter: None, thickness: 0.006 };
        drag::start_internal(app.ctx_mut(), Some(1), "Bulkhead", ring);
        lanes(&mut app, Message::EditEnd);
        assert_eq!(app.ctx().doc.rocket(), &before);
    }

    #[test]
    fn an_airframe_part_picked_up_by_its_bar_reorders_through_the_viewport_drag_path() {
        let mut app = App::new();
        for (i, kind) in [
            PartKind::NoseCone { shape: bowshock_model::NoseShape::TangentOgive, length: 0.2, diameter: 0.1, tip_radius: 0.0 },
            PartKind::BodyTube { length: 0.5, diameter: 0.1 },
            PartKind::Transition { length: 0.1, fore_diameter: 0.1, aft_diameter: 0.06 },
        ]
        .into_iter()
        .enumerate()
        {
            drag::place(app.ctx_mut(), i, kind);
        }
        app.sync();
        let [nose, tube, transition] = [0, 1, 2].map(|i| app.ctx().doc.rocket().parts()[i].id);
        let _ = nose;
        lanes(&mut app, Message::GrabAxial(transition));
        assert!(app.ctx().drag.axial.is_some() && app.ctx().doc.rocket().part(transition).is_none());
        // Carried to the fore end of the tube (station 0.2): the node between nose and tube.
        lanes(&mut app, Message::CarryAt(0.2));
        lanes(&mut app, Message::EditEnd);
        let rocket = app.ctx().doc.rocket();
        assert_eq!((rocket.index_of(transition), rocket.index_of(tube)), (Some(1), Some(2)));
        // One undo step puts it back.
        app.update(App_::Undo);
        assert_eq!(app.ctx().doc.rocket().index_of(transition), Some(2));
    }

    #[test]
    fn the_lanes_show_the_whole_rocket_whatever_the_viewport_does_and_a_reset_returns_to_it() {
        let mut state = State::new();
        let mut app = App::with_example();
        // Whatever the viewport camera is doing, the lanes' axis is the whole rocket's fit.
        let scene = state.scene(app.ctx(), None);
        let fit = Axis::fit(700.0, scene.length);
        assert!(scene.length >= app.ctx().doc.rocket().length());
        let axis_of = |state: &State| state.zoom.apply(fit);
        assert_eq!(axis_of(&state), fit);
        let zoomed = Zoom::between(fit, fit.zoomed(2.0, 300.0, fit.ppm, fit.ppm * 40.0));
        state.update(Message::SetZoom(zoomed), app.ctx_mut());
        assert_eq!(state.zoom, zoomed);
        assert!(axis_of(&state).ppm > fit.ppm);
        state.update(Message::ResetView, app.ctx_mut());
        assert_eq!(state.zoom, Zoom::default());
        assert_eq!(axis_of(&state), fit);
        state.update(Message::ToggleCollapsed, app.ctx_mut());
        assert!(state.collapsed && height(true) < height(false));
        state.update(Message::TogglePinned, app.ctx_mut());
        assert!(state.pinned);
    }

    /// The example with a pod set of two on its first body tube, 0.1 m along it: the app, the pod, its tube and its fins.
    fn example_with_pod() -> (App, PartId, PartId, PartId) {
        let mut app = App::with_example();
        let (pod, tube, fins) = {
            let doc = &mut app.ctx_mut().doc;
            let host = doc.rocket().parts().iter().find(|p| matches!(p.kind, PartKind::BodyTube { .. })).expect("the example has a tube").id;
            let kind = doc.rocket().default_pod();
            let pod = doc.edit("Add pod", |r| r.attach_radial(host, 0.1, 0.0, 2, kind).ok()).expect("a pod");
            let tube = doc.rocket().pods().find(|p| p.id == pod).and_then(|p| p.kind.pod_body()).and_then(|b| b.parts().last()).map(|p| p.id).unwrap();
            let fin = bowshock_model::RadialKind::FinSet { root_chord: 0.06, tip_chord: 0.03, span: 0.03, sweep: 0.02, thickness: 0.002 };
            let fins = doc.edit("Add fins", |r| r.attach_radial(tube, 0.05, 0.0, 3, fin).ok()).expect("fins on the pod");
            (pod, tube, fins)
        };
        (app, pod, tube, fins)
    }

    #[test]
    fn pod_parts_are_selected_and_edited_from_their_bars_and_a_drag_out_of_the_pod_is_refused() {
        let (mut app, pod, tube, fins) = example_with_pod();
        app.sync();
        // A click on the pod set's bar, or on a member's, selects that id.
        for id in [pod, tube, fins] {
            lanes(&mut app, Message::Select(Some(BarKey::Part(id))));
            assert_eq!(app.ctx().selection, Selection::Part(id));
        }
        // The pod set's bar moves it along its parent in one undo step.
        let before = app.ctx().doc.rocket().clone();
        let station = app.ctx().doc.rocket().part_station(pod).unwrap();
        for start in [station + 0.02, station + 0.04] {
            lanes(&mut app, Message::MoveTo { id: pod, start });
        }
        lanes(&mut app, Message::EditEnd);
        assert!((app.ctx().doc.rocket().part_station(pod).unwrap() - (station + 0.04)).abs() < 1e-9);
        app.update(App_::Undo);
        assert_eq!(app.ctx().doc.rocket(), &before);
        // A fin of the pod moves along the pod, and the pod's tube cannot be moved along the axis: refused with a notice.
        lanes(&mut app, Message::MoveTo { id: fins, start: station + 0.3 });
        assert!((app.ctx().doc.rocket().part_station(fins).unwrap() - (station + 0.3)).abs() < 1e-9);
        lanes(&mut app, Message::EditEnd);
        let before = app.ctx().doc.rocket().clone();
        app.ctx_mut().notice = None;
        lanes(&mut app, Message::MoveTo { id: tube, start: 0.0 });
        assert_eq!(app.ctx().doc.rocket(), &before);
        assert!(app.ctx().notice.is_some(), "the refusal is said in the status strip");
        // So is carrying a fin out of the pod.
        app.ctx_mut().notice = None;
        let aft = app.ctx().doc.rocket().part_span(pod).unwrap().1;
        lanes(&mut app, Message::MoveTo { id: fins, start: aft + 0.2 });
        assert_eq!(app.ctx().doc.rocket(), &before);
        assert!(app.ctx().notice.is_some());
        // The pod's tube resizes by its aft end.
        let (fore, tube_aft) = app.ctx().doc.rocket().part_span(tube).unwrap();
        lanes(&mut app, Message::ResizeTo { id: tube, end: End::Aft, station: tube_aft + 0.05 });
        lanes(&mut app, Message::EditEnd);
        let (f, a) = app.ctx().doc.rocket().part_span(tube).unwrap();
        assert!((f - fore).abs() < 1e-9 && (a - (tube_aft + 0.05)).abs() < 1e-9);
    }
}
