use super::super::handles::{self, Axis, Inset, Placed};
use super::super::labels::{self, Request};
use super::super::markers;
use super::super::pick::{self, Inside};
use super::super::snap;
use super::paint::{arrow, brackets, dashed, label, label_in, label_size, text, Labelled, Plan, BOTTOM_STRIP, CUBE_MARGIN, HANDLE_BLOCK, TOOLBAR_X, TOOLBAR_Y};
use super::{Carried, Drag, Hover, Overlay, Selection, Visible, BRACKET_GAP, GLYPH_EM};
use crate::app::ui::{radial_size_label, size_label};
use crate::tokens::{Tokens, TEXT_LABEL};
use crate::viewport::assembly::{self, tag};
use crate::viewport::{InsideView, CUBE_SIZE, TRIAD_GUTTER};
use bowshock_model::{Dim, PartId, PartRef};
use glam::Vec3;
use iced::widget::canvas::{Frame, LineDash, Path, Stroke};
use iced::{alignment, keyboard, Point, Rectangle, Size, Vector};

impl Overlay<'_> {
    /// Which internal parts a press can land on: all of them in Cutaway and X-ray, in Solid only the selected internal part
    /// (the shell is ghosted around it).
    pub(super) fn shown_inside(&self) -> Inside {
        match self.inside {
            InsideView::Cutaway | InsideView::XRay => Inside::All,
            _ => match self.selected {
                Some(id) if self.rocket.internals().iter().any(|p| p.id == id) => Inside::Only(id),
                _ => Inside::Hidden,
            },
        }
    }

    /// The tag at the cursor of an internal part being carried: where it would be (`Motor mount → Booster @ 612 mm`), or a ⊘
    /// with why it cannot go where the cursor is.
    pub(super) fn draw_carry(&self, frame: &mut Frame, t: &Tokens, carried: &Carried, cursor: Point) {
        let name = self.rocket.kind_of(carried.carry.id).map_or_else(|| "Part".to_string(), |p| p.name());
        let Some(landing) = carried.landing else {
            return;
        };
        let parent = self.rocket.part(landing.parent).map_or_else(String::new, |p| p.display_name());
        if landing.fits {
            let station = self.rocket.part_station(landing.parent).unwrap_or(0.0) + landing.offset;
            let margin = self.balance.as_ref().map(|b| b.margin);
            let suffix = assembly::margin_suffix(self.rocket, t, margin);
            assembly::tag_with(
                frame,
                cursor,
                0.0,
                &format!("{name} → {parent} @ {} {}", crate::app::ui::len(station, self.units), crate::app::ui::len_unit(self.units)),
                suffix,
                t.ink,
                t.line_200,
                t.bg_200,
            );
        } else {
            assembly::no_entry(frame, cursor + Vector::new(14.0, 14.0), t.err);
            tag(frame, cursor, 18.0, &format!("{name} does not fit the {parent}"), t.ink, t.line_200, t.bg_200);
        }
    }

    /// The tag over the selection: what it is, and for a linked group how many copies and whether they follow.
    pub(super) fn tag_text(&self, id: PartId, selection: &Option<Selection<'_>>) -> String {
        match selection {
            Some(Selection::Axial { kind, .. }) => kind.name().to_uppercase(),
            Some(Selection::Radial { radial, .. }) if radial.kind.is_pod() => {
                if radial.count > 1 {
                    format!("POD ×{} · LINKED", radial.count)
                } else {
                    "POD".to_string()
                }
            }
            Some(Selection::Radial { radial, .. }) => {
                if radial.count > 1 {
                    let tag = if radial.has_overrides() { format!("{} OVR", radial.overridden_copies().len()) } else { "ALL FOLLOW".to_string() };
                    format!("{} ×{} · LINKED · {tag}", radial.kind.name().to_uppercase(), radial.count)
                } else {
                    radial.kind.name().to_uppercase()
                }
            }
            Some(Selection::Internal) => self.rocket.kind_of(id).map(|p| p.kind_name().to_uppercase()).unwrap_or_default(),
            None => String::new(),
        }
    }

    /// The text beside handle `dim` of part `id` (`SWEEP 60.0`, `Ø 76.2`); empty for a freeform fin's vertices.
    pub(super) fn label_text(&self, id: PartId, dim: Dim) -> String {
        let end = self.length_end(id, dim).map(|e| e.trim_end_matches(" end").to_uppercase());
        let text = match (dim, self.rocket.instance_dim(id, self.instance, dim)) {
            (Dim::Vertex(_), _) => String::new(),
            (_, Some(v)) => self.handle_label(id, dim, v),
            _ => self.rocket.dim_label(id, dim),
        };
        match end {
            Some(end) => format!("{text} · {end}"),
            None => text,
        }
    }

    /// Which end of the part a length handle drags (`fore end`, `aft end`); `None` for a part with a single length handle.
    pub(super) fn length_end(&self, id: PartId, dim: Dim) -> Option<&'static str> {
        match dim {
            Dim::ForeLength => Some("fore end"),
            Dim::Length if self.rocket.handle_specs(id).iter().any(|h| h.dim == Dim::ForeLength) => Some("aft end"),
            _ => None,
        }
    }

    /// Places the selection tag, the handle labels and the inset's labels so that none overprints another, the inset, the
    /// toolbar, the ViewCube or a handle (see [`labels`]). Labels are most important first: the tag, the primary dimensions,
    /// the others. A label with no free spot is dropped and named in `Plan::hidden`.
    pub(super) fn plan(&self, id: PartId, tag_text: &str, b: Rectangle, size: Size, vis: &Visible, with_labels: bool, alt: bool) -> Plan {
        let area = Rectangle::new(Point::ORIGIN, size);
        let mut blocked = vec![
            Rectangle::new(Point::ORIGIN, Size::new(TOOLBAR_X.min(size.width), TOOLBAR_Y)),
            Rectangle::new(Point::new(0.0, (size.height - BOTTOM_STRIP).max(0.0)), Size::new(size.width, BOTTOM_STRIP)),
        ];
        if !self.profile {
            let w = TRIAD_GUTTER + CUBE_SIZE;
            blocked.push(Rectangle::new(Point::new(size.width - CUBE_MARGIN - w, CUBE_MARGIN), Size::new(w, CUBE_SIZE)));
        }
        if let Some(ins) = &vis.inset {
            blocked.push(ins.panel);
        }
        blocked.extend(vis.placed.iter().map(|p| labels::around(p.at, Size::new(HANDLE_BLOCK, HANDLE_BLOCK))));
        if let Some(marks) = &self.balance {
            blocked.extend(markers::balance_label_rects(self.rocket, &self.camera, size, marks, self.units));
        }
        blocked.extend(self.cut_label(size));
        if let Some((_, chip, _)) = self.pod_chip(size) {
            blocked.push(chip);
        }

        // The tag: above the part's left corner, else below it, else against the right side.
        let tag_size = label_size(tag_text);
        let (g, h) = (BRACKET_GAP, tag_size.height);
        let (left, right) = (b.x - g, b.x + b.width + g - tag_size.width);
        let (above, below) = (b.y - g - 4.0 - h, b.y + b.height + g + 4.0);
        let (lside, rside) = (b.x - g - 4.0 - tag_size.width, b.x + b.width + g + 4.0);
        let spots = vec![
            Point::new(left, above),
            Point::new(right, above),
            Point::new(left, above - h - 2.0),
            Point::new(left, below),
            Point::new(right, below),
            Point::new(lside, b.y),
            Point::new(rside, b.y),
            Point::new(right, above - h - 2.0),
        ];
        let mut requests = vec![labels::Request { size: tag_size, priority: 0, spots }];
        let mut wanted = Vec::new();
        if with_labels {
            // Without Alt the 2D profile labels at most two primary dimensions (the selection tag and a couple of names).
            let cap = if self.profile && !alt { 2 } else { usize::MAX };
            for (i, p) in vis.placed.iter().enumerate().filter(|(_, p)| !handles::in_inset(&p.spec) && (alt || p.spec.primary)).take(cap) {
                let text = self.label_text(id, p.spec.dim);
                if !text.is_empty() {
                    requests.push(Request::at_handle(p.at, label_size(&text), if p.spec.primary { 1 } else { 2 }));
                    wanted.push((i, text));
                }
            }
        }
        let rects = labels::layout(&requests, &blocked, &self.silhouettes(id, size), area);
        let tag = rects[0].unwrap_or_else(|| Rectangle::new(requests[0].spots[0], tag_size));
        let mut side = Vec::new();
        let mut hidden = Vec::new();
        for ((handle, text), rect) in wanted.into_iter().zip(&rects[1..]) {
            match rect {
                Some(rect) => side.push(Labelled { handle, text, rect: *rect }),
                None => hidden.push(self.rocket.dim_label(id, vis.placed[handle].spec.dim).to_uppercase()),
            }
        }

        // The inset's labels are placed inside its panel, in the bands above and below the picture (never on it).
        let mut inset = Vec::new();
        if let Some(ins) = &vis.inset {
            let title = Rectangle::new(Point::new(ins.panel.x + 8.0, ins.panel.y + 6.0), Size::new(8.0 * TEXT_LABEL * GLYPH_EM, TEXT_LABEL));
            let r = ins.base + ins.reach.max(handles::CANT_LEVER) + 6.0;
            let mut blocked = vec![title, Rectangle::new(ins.centre - Vector::new(r, r), Size::new(2.0 * r, 2.0 * r))];
            let members: Vec<(usize, &Placed)> = vis.placed.iter().enumerate().filter(|(_, p)| handles::in_inset(&p.spec)).collect();
            blocked.extend(members.iter().map(|(_, p)| labels::around(p.at, Size::new(HANDLE_BLOCK, HANDLE_BLOCK))));
            let texts: Vec<String> = members.iter().map(|(_, p)| self.label_text(id, p.spec.dim)).collect();
            let panel = Rectangle::new(ins.panel.position() + Vector::new(2.0, 2.0), ins.panel.size() - Size::new(4.0, 4.0));
            let requests: Vec<Request> =
                members.iter().zip(&texts).map(|((_, p), s)| Request::at_handle(p.at, label_size(s), 1).with_bands(p.at, panel, 20.0)).collect();
            for (((handle, p), text), rect) in members.iter().zip(texts).zip(labels::layout(&requests, &blocked, &[], panel)) {
                match rect {
                    Some(rect) => inset.push(Labelled { handle: *handle, text, rect }),
                    None => hidden.push(self.rocket.dim_label(id, p.spec.dim).to_uppercase()),
                }
            }
        }
        Plan { tag, side, inset, hidden }
    }

    /// Screen bounds of every other part of the rocket (the airframe and the radial parts that are not the selection): labels
    /// prefer not to sit over them, so they stay in free sky.
    pub(super) fn silhouettes(&self, id: PartId, size: Size) -> Vec<Rectangle> {
        let (rocket, camera) = (self.rocket, &self.camera);
        let mut out: Vec<Rectangle> = rocket
            .parts()
            .iter()
            .enumerate()
            .filter(|(_, p)| p.id != id)
            .map(|(i, p)| pick::screen_bounds(camera, size, rocket.stations()[i], &p.kind))
            .collect();
        for radial in rocket.radials().iter().filter(|r| r.id != id) {
            for i in 0..radial.angles().len() {
                if let Some(at) = pick::placement(rocket, radial, i) {
                    out.push(pick::radial_bounds(camera, size, &radial.effective(i).kind, at));
                }
            }
        }
        out
    }

    /// A multi-selection: accent brackets on every selected part and one tag (`3 PARTS`) over the group. No handles. A linked
    /// radial group (a fin set) is one part with brackets on every copy, dashed on the copies beside the first, as when it is
    /// selected alone.
    pub(super) fn draw_multi(&self, frame: &mut Frame, t: &Tokens, size: Size) {
        let parts = self.selected.iter().chain(&self.also);
        let mut union: Option<Rectangle> = None;
        let mut count = 0;
        for id in parts {
            let Some(b) = self.bounds_of(*id, size) else { continue };
            brackets(frame, b, t.accent, false);
            union = Some(union.map_or(b, |u| u.union(&b)));
            count += 1;
            if let Some(radial) = self.rocket.radial(*id) {
                for i in 0..radial.angles().len() {
                    let Some(at) = pick::placement(self.rocket, radial, i) else { continue };
                    let copy = pick::radial_bounds(&self.camera, size, &radial.effective(i).kind, at);
                    // The copy `bounds_of` already framed is not framed twice.
                    if (copy.x - b.x).abs() + (copy.y - b.y).abs() + (copy.width - b.width).abs() + (copy.height - b.height).abs() > 0.5 {
                        brackets(frame, copy, t.accent, true);
                        union = union.map(|u| u.union(&copy));
                    }
                }
            }
        }
        let Some(b) = union else { return };
        let caption_y = if b.y - BRACKET_GAP - 4.0 < TOOLBAR_Y && b.x < TOOLBAR_X { b.y + b.height + BRACKET_GAP + 22.0 } else { b.y - BRACKET_GAP - 4.0 };
        label(frame, &format!("{count} PARTS"), Point::new(b.x - BRACKET_GAP, caption_y), t.accent, t.bg_000);
    }

    pub(super) fn draw_hover(&self, frame: &mut Frame, t: &Tokens, cursor: Option<Point>) {
        let (Hover::Part(id), Some(cursor)) = (self.hover, cursor) else {
            return;
        };
        // The selected part already carries its own tag over the brackets (the kit shows one, not two).
        if self.selected == Some(id) {
            return;
        }
        let label = match (self.rocket.part(id), self.rocket.radial(id)) {
            (Some(part), _) => Some(format!("{} · {}", part.kind.name(), size_label(&part.kind, self.units))),
            (_, Some(radial)) => {
                let times = if radial.count > 1 { format!(" ×{}", radial.count) } else { String::new() };
                Some(format!("{}{times} · {}", radial.kind.name(), radial_size_label(&radial.kind, self.units)))
            }
            _ => None,
        };
        if let Some(label) = label {
            tag(frame, cursor, 0.0, &label, t.ink, t.line_200, t.bg_200);
        }
    }

    /// The aft-view inset of a radial part: the parent in section, every copy, and the first copy's handles with names.
    pub(super) fn draw_inset(&self, frame: &mut Frame, t: &Tokens, id: PartId, ins: &Inset, vis: &Visible, names: &[Labelled]) {
        let Some(radial) = self.rocket.radial(id) else {
            return;
        };
        frame.fill_rectangle(ins.panel.position(), ins.panel.size(), t.bg_000);
        frame.stroke_rectangle(ins.panel.position(), ins.panel.size(), Stroke::default().with_color(t.line_200).with_width(1.0));
        text(
            frame,
            "AFT VIEW",
            Point::new(ins.panel.x + 8.0, ins.panel.y + 6.0),
            t.ink_muted,
            TEXT_LABEL,
            alignment::Horizontal::Left,
            alignment::Vertical::Top,
        );
        frame.fill(&Path::circle(ins.centre, ins.base), t.part_body);
        let thickness = radial.kind.thickness().unwrap_or(0.002);
        let width = (thickness as f32 * handles::THICKNESS_SCALE).clamp(2.0, 12.0);
        for (k, a) in radial.angles().into_iter().enumerate() {
            if let bowshock_model::RadialKind::Pod { gap, body } = &radial.kind {
                // A pod copy is a circle of its own radius, `gap` out from the core's surface.
                let (g, r) = (*gap as f32 * ins.scale, body.radius() as f32 * ins.scale);
                let c = ins.centre + handles::arc_dir(a) * (ins.base + g + r);
                frame.fill(&Path::circle(c, r), t.part_fin);
                frame.stroke(&Path::circle(c, r), Stroke::default().with_color(if k == 0 { t.accent } else { t.line_200 }).with_width(1.0));
                continue;
            }
            let (from, to) = (ins.centre + handles::arc_dir(a) * ins.base, ins.centre + handles::arc_dir(a) * (ins.base + ins.reach.max(4.0)));
            let colour = if k == 0 { t.accent } else { t.part_fin };
            if radial.kind.is_fin() {
                // A canted fin seen end-on is a thin plate leaning to one side (its own cant, the group's or its override).
                let own = radial.effective(k);
                match handles::canted_outline(ins, &own.kind, a, own.cant).filter(|_| own.cant.abs() > 1e-9) {
                    Some(plate) => {
                        let outline = Path::new(|p| {
                            p.move_to(plate[0]);
                            for q in &plate[1..] {
                                p.line_to(*q);
                            }
                            p.close();
                        });
                        frame.fill(&outline, t.part_fin);
                        frame.stroke(&outline, Stroke::default().with_color(t.ink_muted).with_width(0.75));
                    }
                    None => frame.stroke(&Path::line(from, to), Stroke::default().with_color(t.part_fin).with_width(width)),
                }
                if k == 0 {
                    frame.stroke(&Path::line(from, to), Stroke::default().with_color(colour).with_width(1.0));
                }
            } else {
                frame.fill(&Path::circle(to, 3.0), colour);
            }
        }
        for l in names {
            if let Some((from, to)) = Request::leader(vis.placed[l.handle].at, l.rect) {
                dashed(frame, from, to, t.line_200, true);
            }
            label_in(frame, t, &l.text, l.rect, t.ink_muted);
        }
        // The way the rocket spins seen from behind, when any fin is canted: a curved arrow and CW / CCW at the left of the picture.
        if let Some(clockwise) = handles::spin_sense(radial) {
            let at = Point::new(ins.panel.x + 20.0, ins.centre.y);
            let (arc, head) = handles::spin_arrow(at, 8.0, clockwise);
            let path = Path::new(|p| {
                p.move_to(arc[0]);
                for q in &arc[1..] {
                    p.line_to(*q);
                }
            });
            frame.stroke(&path, Stroke::default().with_color(t.accent).with_width(1.5));
            let tip = Path::new(|p| {
                p.move_to(head[0]);
                p.line_to(head[1]);
                p.line_to(head[2]);
                p.close();
            });
            frame.fill(&tip, t.accent);
            text(
                frame,
                if clockwise { "CW" } else { "CCW" },
                Point::new(at.x, at.y + 14.0),
                t.accent,
                TEXT_LABEL,
                alignment::Horizontal::Center,
                alignment::Vertical::Top,
            );
        }
        // The cant lever as a dotted arc from the fin's own direction to its tilt.
        if let Some(cant) = vis.placed.iter().find(|p| p.spec.dim == Dim::Cant) {
            let Axis::Arc { centre } = cant.axis else {
                return;
            };
            let lever = cant.at.distance(centre).max(1.0);
            let a0 = handles::copy_angle(radial, self.instance);
            let cant_now = radial.effective(self.instance).cant;
            let arc = Path::new(|p| {
                for i in 0..=12 {
                    let a = a0 + cant_now * f64::from(i) / 12.0;
                    let pt = centre + handles::arc_dir(a) * lever;
                    if i == 0 {
                        p.move_to(pt);
                    } else {
                        p.line_to(pt);
                    }
                }
            });
            frame.stroke(&arc, Stroke { line_dash: LineDash { segments: &[1.5, 3.0], offset: 0 }, ..Stroke::default().with_color(t.accent).with_width(1.0) });
        }
    }

    /// The live feedback of a drag: the old-size line, the direction arrow, the snap ticks and guide, and the framed tag.
    pub(super) fn draw_drag(&self, frame: &mut Frame, t: &Tokens, drag: &Drag, vis: &Visible, modifiers: keyboard::Modifiers) {
        let dim = drag.placed.spec.dim;
        let Some(now) = vis.placed.iter().find(|p| p.spec.dim == dim) else {
            return;
        };
        let size = frame.size();
        let value = self.rocket.instance_dim(drag.id, self.instance, dim).unwrap_or(drag.live.value);
        let moved = value - drag.base;
        let warn = drag.live.limit.is_some();
        let tone = if warn { t.warn } else { t.accent };

        // Where the edge was (dashed `ink-muted`) and which way it is going.
        match drag.placed.axis {
            Axis::Line { dir, per_unit } => {
                let start = drag.placed.at;
                let perp = Vector::new(-dir.y, dir.x);
                let perp = if perp.y < 0.0 || (perp.y == 0.0 && perp.x < 0.0) { perp * -1.0 } else { perp };
                dashed(frame, start + perp * 36.0, start - perp * 36.0, t.ink_muted, false);
                if moved.abs() > 1e-9 {
                    let was = format!("WAS {}", self.num(dim, drag.base));
                    let at = if dir.x.abs() > dir.y.abs() {
                        start + perp * 46.0
                    } else {
                        Point::new(start.x - 40.0 - was.chars().count() as f32 * TEXT_LABEL * GLYPH_EM, start.y)
                    };
                    // While the snap ticks are out their labels own the space under the path (the kit shows no `WAS` then).
                    if !modifiers.shift() {
                        label(frame, &was, at, t.ink_muted, t.bg_000);
                    }
                    let towards = (now.at - start).x * dir.x + (now.at - start).y * dir.y;
                    let heading = if towards >= 0.0 { dir } else { dir * -1.0 };
                    arrow(frame, now.at + heading * 8.0, heading, 18.0, t.accent);
                }
                // Snap ticks with Shift: one per nearby target along the path, the caught one in `ok`.
                if modifiers.shift() {
                    // The caught tick first, then the nearest; a tick closer than 14 px to one already drawn is skipped.
                    let mut order = snap::ticks(&drag.targets, value, per_unit, 8);
                    order.sort_by_key(|&i| drag.live.caught != Some(i));
                    let mut drawn: Vec<f32> = Vec::new();
                    let mut rows: Vec<(usize, f32, f32)> = Vec::new();
                    for i in order {
                        let target = &drag.targets[i];
                        let along = (target.value - value) as f32 * per_unit;
                        if drawn.iter().any(|d| (d - along).abs() < 14.0) {
                            continue;
                        }
                        // Along a horizontal path labels share one row beside their ticks; one that would overprint a neighbour
                        // drops to the next row, on a longer stem.
                        let x0 = now.at.x + dir.x * along + 2.0;
                        let span = (x0, x0 + 6.0 + target.label.chars().count() as f32 * TEXT_LABEL * GLYPH_EM);
                        let row = if dir.x.abs() > dir.y.abs() {
                            let free = (0..).find(|r| !rows.iter().any(|(rr, a, b)| rr == r && span.0 < *b && *a < span.1)).unwrap_or(0);
                            rows.push((free, span.0, span.1));
                            free as f32 + 1.0
                        } else {
                            1.0
                        };
                        drawn.push(along);
                        let at = now.at + dir * along;
                        let caught = drag.live.caught == Some(i);
                        let (colour, width) = if caught { (t.ok, 2.5) } else { (t.ink_muted, 1.0) };
                        let reach = 12.0 + 16.0 * (row - 1.0);
                        frame.stroke(&Path::line(at - perp * 8.0, at + perp * (reach + 2.0)), Stroke::default().with_color(colour).with_width(width));
                        let text_colour = if caught { t.ink } else { t.ink_faint };
                        label(frame, &target.label, at + perp * (reach + 6.0) + dir * 4.0, text_colour, t.bg_000);
                        if caught {
                            if let Some(src) = target.source.and_then(|s| self.anchor(s, size)) {
                                dashed(frame, at, src, t.ok, true);
                            }
                        }
                    }
                }
            }
            Axis::Arc { centre } => {
                dashed(frame, centre, drag.placed.at, t.ink_muted, false);
                if modifiers.shift() {
                    let lever = now.at.distance(centre).max(1.0);
                    for i in snap::ticks(&drag.targets, value, lever, 6) {
                        let target = &drag.targets[i];
                        let a = handles::angle_about(centre, now.at) + (target.value - value);
                        let at = centre + handles::arc_dir(a) * lever;
                        let caught = drag.live.caught == Some(i);
                        let (colour, width) = if caught { (t.ok, 2.5) } else { (t.ink_muted, 1.0) };
                        frame.stroke(&Path::line(at, at + handles::arc_dir(a) * 8.0), Stroke::default().with_color(colour).with_width(width));
                        label(frame, &target.label, at + handles::arc_dir(a) * 12.0, if caught { t.ink } else { t.ink_faint }, t.bg_000);
                    }
                }
            }
            Axis::Plane { .. } => {}
        }

        // The framed tag: title in `ink-muted`, value in the drag colour, the change at the right, then the match or the reason.
        let end = self.length_end(drag.id, dim);
        let title = match (dim, end) {
            (Dim::Vertex(_), _) => dim.label(),
            (_, Some(end)) => format!("{} · {end}", self.rocket.dim_label(drag.id, dim)),
            _ => self.rocket.dim_label(drag.id, dim),
        };
        let typing = !drag.typed.is_empty();
        let value_text = if typing {
            format!("{}▏", drag.typed)
        } else if let Dim::Vertex(i) = dim {
            let (z, h) = self.rocket.vertex(drag.id, i).unwrap_or((0.0, 0.0));
            format!("{} , {}", self.num(Dim::Length, z), self.num(Dim::Length, h))
        } else if end.is_some() && moved.abs() > 1e-9 {
            format!("{} → {}", self.num(dim, drag.base), self.with_unit(dim, value))
        } else {
            self.with_unit(dim, value)
        };
        let delta = if typing || matches!(dim, Dim::Vertex(_)) { String::new() } else { self.delta(dim, moved) };
        let second = match (drag.live.limit, drag.live.caught.and_then(|i| drag.targets.get(i))) {
            (Some(reason), _) => Some((t.warn, reason.to_string())),
            (None, Some(target)) => Some((t.ok, if target.label.starts_with("flush") { target.label.clone() } else { format!("= {}", target.label) })),
            _ => None,
        };
        let chars = |s: &str| s.chars().count() as f32;
        let inner_w = (chars(&value_text) * 13.0 * GLYPH_EM + chars(&delta) * TEXT_LABEL * GLYPH_EM + 18.0)
            .max(chars(&title) * TEXT_LABEL * GLYPH_EM)
            .max(second.as_ref().map_or(0.0, |s| chars(&s.1) * TEXT_LABEL * GLYPH_EM + 14.0))
            .max(96.0);
        let (w, h) = (inner_w + 16.0, if second.is_some() { 62.0 } else { 46.0 });
        let tag_left = modifiers.shift() && matches!(drag.placed.axis, Axis::Line { dir, .. } if dir.x.abs() <= dir.y.abs());
        let anchor = now.at + Vector::new(if tag_left { -(w + 20.0) } else { 20.0 }, -(h + 16.0));
        let origin = Point::new(anchor.x.clamp(6.0, (size.width - w - 6.0).max(6.0)), anchor.y.clamp(48.0, (size.height - h - 6.0).max(48.0)));
        frame.fill_rectangle(origin, Size::new(w, h), t.bg_200);
        frame.stroke_rectangle(origin, Size::new(w, h), Stroke::default().with_color(tone).with_width(1.0));
        let (x, y) = (origin.x + 8.0, origin.y + 8.0);
        text(frame, title, Point::new(x, y), t.ink_muted, TEXT_LABEL - 1.0, alignment::Horizontal::Left, alignment::Vertical::Top);
        text(frame, value_text, Point::new(x, y + 14.0), tone, 13.0, alignment::Horizontal::Left, alignment::Vertical::Top);
        if !delta.is_empty() {
            text(frame, delta, Point::new(origin.x + w - 8.0, y + 17.0), t.ink_muted, TEXT_LABEL, alignment::Horizontal::Right, alignment::Vertical::Top);
        }
        if let Some((colour, line)) = second {
            frame.fill_rectangle(Point::new(x, y + 36.0), Size::new(6.0, 6.0), colour);
            text(
                frame,
                line,
                Point::new(x + 12.0, y + 39.0),
                if colour == t.warn { t.warn } else { t.ink },
                TEXT_LABEL,
                alignment::Horizontal::Left,
                alignment::Vertical::Center,
            );
        }
    }

    /// A point on screen that stands for part `id` (the middle of its top edge), where a snap guide runs to.
    pub(super) fn anchor(&self, id: PartId, size: Size) -> Option<Point> {
        let (rocket, camera) = self.frame_of(id)?;
        let (a, b) = rocket.part_span(id)?;
        let up = handles::radial_up(camera);
        let r = match rocket.kind_of(id)? {
            PartRef::Axial(p) => p.kind.fore_diameter().max(p.kind.aft_diameter()) / 2.0,
            PartRef::Internal(_) => rocket.resolved(id)?.outer_diameter / 2.0,
            PartRef::Radial(_) => 0.0,
        };
        Some(camera.project(up * r as f32 + Vec3::Z * ((a + b) / 2.0) as f32, size))
    }
}
