use super::super::assembly::tag;
use super::super::carry::Carry;
use super::super::labels::Request;
use super::super::pick::{self, Pick};
use super::super::Message;
use super::paint::{brackets, dashed, drag_verb, draw_note, handle_shape, label, label_in};
use super::{Carried, Drag, Edit, Hover, Live, Overlay, Press, Selection, State, DOUBLE, SLOP};
use crate::theme;
use crate::tokens::TEXT_LABEL;
use bowshock_model::Dim;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, Stroke};
use iced::{keyboard, mouse, Point, Rectangle, Renderer, Theme};
use std::time::Instant;

impl canvas::Program<Message> for Overlay<'_> {
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        let size = bounds.size();
        // A carry the app has put back (Esc, or the parts bin) is forgotten.
        if let Some(carried) = state.carry.as_mut() {
            if self.hand.lifted == Some(carried.carry.id) {
                carried.confirmed = true;
            } else if carried.confirmed {
                state.carry = None;
            }
        }
        let edit = |e: Edit| Some(Action::publish(Message::Edit(e)).and_capture());
        // The right button orbits (the scene below sees it too); pressed and released in place on a part it opens that part's
        // context menu instead, so the event is never captured here.
        match event {
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => {
                state.right = cursor
                    .position_in(bounds)
                    .filter(|_| state.drag.is_none() && state.carry.is_none())
                    .map(|at| Press { at, target: pick::pick_inside(self.rocket, &self.camera, at, size, self.shown_inside()) });
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                if let (Some(press), Some(at)) = (&state.right, cursor.position_in(bounds)) {
                    if at.distance(press.at) > SLOP {
                        state.right = None;
                    }
                }
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right)) => {
                if let Some(pick) = state.right.take().and_then(|p| p.target) {
                    return Some(Action::publish(Message::PartMenu(pick)));
                }
            }
            _ => {}
        }
        match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) => {
                state.modifiers = Some(*modifiers);
                // Alt and Shift change what shows; re-run a live drag so the snap follows the key.
                if let Some(mut drag) = state.drag.take() {
                    let at = cursor.position().map(|p| Point::new(p.x - bounds.x, p.y - bounds.y));
                    let out = at.and_then(|at| self.drag_to(&mut drag, at, *modifiers));
                    state.drag = Some(drag);
                    return Some(out.map_or_else(Action::request_redraw, |e| Action::publish(Message::Edit(e))));
                }
                Some(Action::request_redraw())
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key: keyboard::Key::Named(keyboard::key::Named::Escape), .. }) if state.carry.is_some() => {
                state.carry = None;
                edit(Edit::Cancel)
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key, text, .. }) if state.drag.is_some() => {
                use keyboard::key::Named;
                let drag = state.drag.as_mut()?;
                match key {
                    keyboard::Key::Named(Named::Escape) => {
                        state.drag = None;
                        edit(Edit::Cancel)
                    }
                    keyboard::Key::Named(Named::Enter) => {
                        let (id, dim) = (drag.id, drag.placed.spec.dim);
                        let value = self.parse(dim, &drag.typed);
                        state.drag = None;
                        match value {
                            Some(v) => edit(Edit::Commit(id, dim, v)),
                            None => edit(Edit::End),
                        }
                    }
                    keyboard::Key::Named(Named::Backspace) => {
                        drag.typed.pop();
                        Some(Action::request_redraw().and_capture())
                    }
                    _ => {
                        let typed = text.as_ref().map(|t| t.as_str()).unwrap_or("");
                        if !matches!(drag.placed.spec.dim, Dim::Vertex(_))
                            && typed.chars().all(|c| c.is_ascii_digit() || ".-+*/ ".contains(c))
                            && !typed.is_empty()
                        {
                            drag.typed.push_str(typed);
                            return Some(Action::request_redraw().and_capture());
                        }
                        None
                    }
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let at = cursor.position_in(bounds)?;
                // A part held with no button down (click to place) is put down where it is by a left press.
                if self.hand.click && state.carry.take().is_some() {
                    return edit(Edit::End);
                }
                let vis = self.visible(size, state.modifiers.unwrap_or(self.drive_modifiers));
                if let Some(h) = self.handle_at(at, size, state.modifiers.unwrap_or(self.drive_modifiers)) {
                    let id = self.selected?;
                    let now = Instant::now();
                    let dim = h.spec.dim;
                    if state.last_handle.is_some_and(|(t, i, d)| i == id && d == dim && now.duration_since(t) < DOUBLE) {
                        state.last_handle = None;
                        state.drag = None;
                        return match dim {
                            Dim::Vertex(i) => edit(Edit::RemoveVertex(id, i)),
                            _ => edit(Edit::Focus(dim)),
                        };
                    }
                    state.last_handle = Some((now, id, dim));
                    let base = self.rocket.instance_dim(id, self.instance, dim).unwrap_or(0.0);
                    let base_vertex = if let Dim::Vertex(i) = dim { self.rocket.vertex(id, i).unwrap_or((0.0, 0.0)) } else { (0.0, 0.0) };
                    state.drag = Some(Drag {
                        id,
                        placed: h,
                        base,
                        base_vertex,
                        from: at,
                        targets: self.rocket.snap_targets(id, dim),
                        typed: String::new(),
                        last: None,
                        live: Live { value: base, caught: None, limit: None },
                    });
                    let nose = self.rocket.part(id).is_some_and(|p| p.kind.name() == "Nose cone");
                    return edit(Edit::Begin(id, drag_verb(dim, nose)));
                }
                // A double-click on an edge of the selected freeform fin adds a vertex there.
                if let Some((id, after, (z, h))) = self.edge_at(at, &vis) {
                    let now = Instant::now();
                    if state.last_edge.is_some_and(|(t, i, a)| i == id && a == after && now.duration_since(t) < DOUBLE) {
                        state.last_edge = None;
                        return edit(Edit::AddVertex(id, after, z, h));
                    }
                    state.last_edge = Some((now, id, after));
                }
                if let Some((pod, chip, _)) = self.pod_chip(size) {
                    if chip.contains(at) {
                        return Some(Action::publish(Message::Select(Some(Pick::Radial { id: pod, instance: 0 }))).and_capture());
                    }
                }
                let target = pick::pick_inside(self.rocket, &self.camera, at, size, self.shown_inside());
                state.press = Some(Press { at, target });
                // An airframe part of the core pressed here is held where it was pressed if the press becomes a pick-up.
                let held = match target {
                    Some(Pick::Axial(id)) if self.rocket.pod_of(id).is_none() => {
                        pick::axis_station(&self.camera, at, size).zip(self.rocket.part_station(id)).map(|(z, fore)| (id, z - fore))
                    }
                    _ => None,
                };
                Some(match held {
                    Some((id, hold)) => Action::publish(Message::Held(id, hold)).and_capture(),
                    None => Action::capture(),
                })
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                let at = cursor.position().map(|p| Point::new(p.x - bounds.x, p.y - bounds.y))?;
                if let Some(mut drag) = state.drag.take() {
                    let out = if drag.typed.is_empty() { self.drag_to(&mut drag, at, state.modifiers.unwrap_or(self.drive_modifiers)) } else { None };
                    state.drag = Some(drag);
                    return Some(out.map_or_else(|| Action::request_redraw().and_capture(), |e| Action::publish(Message::Edit(e)).and_capture()));
                }
                if let Some(carried) = state.carry.as_mut() {
                    let step = pick::step(state.modifiers.unwrap_or(self.drive_modifiers), self.snap_step);
                    let landing = pick::axis_station(&self.camera, at, size).and_then(|z| carried.carry.landing(self.rocket, z, step));
                    let place = landing.filter(|l| l.fits).map(|l| (carried.carry.id, l.parent, l.offset)).filter(|p| carried.placed != Some(*p));
                    carried.landing = landing;
                    return Some(match place {
                        Some(p) => {
                            carried.placed = Some(p);
                            Action::publish(Message::Edit(Edit::Place(p.0, p.1, p.2)))
                        }
                        None => Action::request_redraw(),
                    });
                }
                if let Some(press) = &state.press {
                    if at.distance(press.at) <= SLOP {
                        return None;
                    }
                    // Beyond the slop it is a drag: on a part that picks it up; on empty space it is not a click either.
                    let (target, from) = (press.target, press.at);
                    state.press = None;
                    return match target? {
                        // An internal part is carried where it stands: along the rocket, into the next body part past the end.
                        Pick::Internal(id) => {
                            let carry = Carry::new(self.rocket, id, pick::axis_station(&self.camera, from, size)?)?;
                            state.carry = Some(Carried { carry, landing: None, placed: None, confirmed: false });
                            Some(Action::publish(Message::Edit(Edit::Carry(id))))
                        }
                        pick => Some(Action::publish(Message::Grab(pick)).and_capture()),
                    };
                }
                let hover = cursor.position_in(bounds).map_or(Hover::None, |p| self.hover_at(p, size, state.modifiers.unwrap_or(self.drive_modifiers)));
                (hover != self.hover).then(|| Action::publish(Message::Hover(hover)))
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                if state.drag.take().is_some() {
                    return edit(Edit::End);
                }
                if state.carry.is_some() {
                    // Placing by clicking, the part stays in hand after the release: the next press puts it down.
                    if self.hand.click {
                        return None;
                    }
                    // The edit is over: the document records its one undo step.
                    state.carry = None;
                    return edit(Edit::End);
                }
                let press = state.press.take()?;
                // Shift-click or Cmd/Ctrl-click on a part adds it to the selection or takes it out; on empty space it clears.
                let message = match press.target {
                    Some(pick)
                        if state.modifiers.unwrap_or(self.drive_modifiers).shift()
                            || state.modifiers.unwrap_or(self.drive_modifiers).command()
                            || state.modifiers.unwrap_or(self.drive_modifiers).control() =>
                    {
                        Message::Toggle(pick)
                    }
                    target => Message::Select(target),
                };
                Some(Action::publish(message).and_capture())
            }
            _ => None,
        }
    }

    fn draw(&self, state: &State, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let cursor =
            self.cursor.map(|c| Point::new(c.x - bounds.x, c.y - bounds.y)).filter(|c| (0.0..=size.width).contains(&c.x) && (0.0..=size.height).contains(&c.y));
        let all = state.modifiers.unwrap_or(self.drive_modifiers).alt() || self.profile;
        if let (Some(carried), Some(at)) = (&state.carry, cursor) {
            self.draw_carry(&mut frame, t, carried, at);
        }

        // The selection: brackets (solid on the part, or on the clicked copy of a group and dashed on its siblings) with a tag.
        let selection = self.selection();
        let Some(id) = self.selected else {
            self.draw_hover(&mut frame, t, cursor);
            return vec![frame.into_geometry()];
        };
        if !self.also.is_empty() {
            self.draw_multi(&mut frame, t, size);
            self.draw_hover(&mut frame, t, cursor);
            return vec![frame.into_geometry()];
        }
        let tag_text = self.tag_text(id, &selection);
        if let Some(Selection::Radial { radial, .. }) = &selection {
            let (rocket, camera) = self.selected.and_then(|id| self.frame_of(id)).unwrap_or((self.rocket, &self.camera));
            let own = self.instance.min(radial.angles().len() - 1);
            for i in (0..radial.angles().len()).filter(|i| *i != own) {
                if let Some(sibling) = pick::placement(rocket, radial, i) {
                    brackets(&mut frame, pick::radial_bounds(camera, size, &radial.effective(i).kind, sibling), t.accent.scale_alpha(0.7), true);
                }
            }
        }
        // A part of a pod stands in every copy of the pod: dashed brackets on the others, and the tag that selects the set.
        if let Some(view) = self.pod.as_ref().filter(|v| self.rocket.pod_of(id).is_some_and(|p| p.id == v.pod)) {
            for k in 1..view.cameras.len() {
                if let Some(b) = self.bounds_in(&view.sub, &view.cameras[k], id, size) {
                    brackets(&mut frame, b, t.accent.scale_alpha(0.7), true);
                }
            }
            if let Some((_, chip, text)) = self.pod_chip(size) {
                frame.fill_rectangle(chip.position(), chip.size(), t.bg_000);
                frame.stroke_rectangle(chip.position(), chip.size(), Stroke::default().with_color(t.accent).with_width(1.0));
                label(&mut frame, &text, Point::new(chip.x + 6.0, chip.y + 4.0 + TEXT_LABEL), t.accent, t.bg_000);
            }
        }
        let Some(b) = self.bounds_of_selection(size) else {
            return vec![frame.into_geometry()];
        };
        brackets(&mut frame, b, t.accent, false);
        // Carried along the rocket the part has no handles: only its brackets and the tag at the cursor.
        if state.carry.is_some() {
            return vec![frame.into_geometry()];
        }

        // Where the tag and the handle labels go: placed together so none overprints another or the inset (see `labels`).
        let vis = self.visible(size, state.modifiers.unwrap_or(self.drive_modifiers));
        let plan = self.plan(id, &tag_text, b, size, &vis, all && state.drag.is_none(), state.modifiers.unwrap_or(self.drive_modifiers).alt());
        label_in(&mut frame, t, &tag_text, plan.tag, t.accent);
        if let Some(ins) = &vis.inset {
            self.draw_inset(&mut frame, t, id, ins, &vis, &plan.inset);
        }
        let active = state.drag.as_ref().map(|d| d.placed.spec.dim);
        for p in &vis.placed {
            let live = active == Some(p.spec.dim);
            handle_shape(&mut frame, t, p, live, self.hover == Hover::Handle(p.spec.dim));
        }
        for l in &plan.side {
            if let Some((from, to)) = Request::leader(vis.placed[l.handle].at, l.rect) {
                dashed(&mut frame, from, to, t.line_200, true);
            }
            label_in(&mut frame, t, &l.text, l.rect, t.ink_muted);
        }
        // Without Alt a fin set's cant, thickness and angle handles are in the aft-view inset, which nothing on screen mentions.
        if state.drag.is_none() {
            if let Some(note) = plan.note(vis.crowded) {
                draw_note(&mut frame, t, size, &note, "Zoom in for more");
            } else if !all && self.rocket.radial(id).is_some_and(|r| r.kind.is_fin()) {
                draw_note(&mut frame, t, size, "Hold Alt for the aft view · fin cant", "Hold Alt for aft view");
            }
        }

        if let Some(drag) = &state.drag {
            self.draw_drag(&mut frame, t, drag, &vis, state.modifiers.unwrap_or(self.drive_modifiers));
        } else if let (Some(Hover::Handle(dim)), Some(c)) = (Some(self.hover), cursor) {
            // A handle whose label is already shown needs no second tag at the cursor (it would sit over that label and the
            // selection tag).
            let labelled = plan.side.iter().any(|l| vis.placed[l.handle].spec.dim == dim);
            if let (Some(v), true, false) = (self.rocket.instance_dim(id, self.instance, dim), vis.placed.iter().any(|p| p.spec.dim == dim), labelled) {
                let name = self.rocket.dim_label(id, dim);
                tag(&mut frame, c, 0.0, &format!("{name} · {}", self.with_unit(dim, v)), t.ink, t.line_200, t.bg_200);
            }
        } else {
            self.draw_hover(&mut frame, t, cursor);
        }
        vec![frame.into_geometry()]
    }

    // No `mouse_interaction`: a layer that reports a cursor hides the mouse from the layers below it (see
    // `scene::Scene::interaction`), and the model underneath needs the middle button and the wheel.
}
