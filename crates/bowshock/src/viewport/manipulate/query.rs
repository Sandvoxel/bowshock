use super::super::handles::{self, Axis, Context, Placed};
use super::super::markers;
use super::super::pick;
use super::super::snap;
use super::super::{Camera, InsideView};
use super::paint::bounds;
use super::{visible, Drag, Edit, Hover, Live, Overlay, Selection, Visible, GLYPH_EM, HIT};
use crate::tokens::TEXT_LABEL;
use crate::units::{self, Quantity};
use bowshock_model::{Dim, PartId, Rocket};
use glam::Vec3;
use iced::{keyboard, Point, Rectangle, Size};

impl Overlay<'_> {
    pub(super) fn selection(&self) -> Option<Selection<'_>> {
        self.selection_of(self.selected?)
    }

    /// The clicked copy of group `id`: the one `instance` names for the primary part, the first for any other.
    pub(super) fn copy_of(&self, id: PartId) -> usize {
        if self.selected == Some(id) {
            self.instance
        } else {
            0
        }
    }

    /// The rocket and camera part `id` is seen through: the document's, or for a part inside the selected part's pod, that
    /// pod's own (its first copy; the others are bracketed in dashes). `None` for a part of some other pod.
    pub(super) fn frame_of(&self, id: PartId) -> Option<(&Rocket, &Camera)> {
        match self.rocket.pod_of(id) {
            None => Some((self.rocket, &self.camera)),
            Some(pod) => self.pod.as_ref().filter(|v| v.pod == pod.id).map(|v| (&v.sub, &v.cameras[0])),
        }
    }

    pub(super) fn selection_of(&self, id: PartId) -> Option<Selection<'_>> {
        let (rocket, _) = self.frame_of(id)?;
        self.selection_in(rocket, id)
    }

    pub(super) fn selection_in<'r>(&self, rocket: &'r Rocket, id: PartId) -> Option<Selection<'r>> {
        if let Some(index) = rocket.index_of(id) {
            return Some(Selection::Axial { z0: rocket.stations()[index], kind: &rocket.parts()[index].kind });
        }
        if let Some(radial) = rocket.radial(id) {
            let instance = self.copy_of(id).min(radial.angles().len().saturating_sub(1));
            return Some(Selection::Radial { radial, at: pick::placement(rocket, radial, instance)? });
        }
        rocket.internal(id).map(|_| Selection::Internal)
    }

    pub(super) fn instance(&self) -> usize {
        self.instance
    }

    /// The screen bounds of the selected part.
    pub(super) fn bounds_of_selection(&self, size: Size) -> Option<Rectangle> {
        self.bounds_of(self.selected?, size)
    }

    /// The screen bounds of part `id`.
    pub(super) fn bounds_of(&self, id: PartId, size: Size) -> Option<Rectangle> {
        let (rocket, camera) = self.frame_of(id)?;
        self.bounds_in(rocket, camera, id, size)
    }

    /// The screen bounds of part `id` of `rocket` seen through `camera` (a pod's part through one copy's camera).
    pub(super) fn bounds_in(&self, rocket: &Rocket, camera: &Camera, id: PartId, size: Size) -> Option<Rectangle> {
        Some(match self.selection_in(rocket, id)? {
            Selection::Axial { z0, kind } => pick::screen_bounds(camera, size, z0, kind),
            Selection::Radial { radial, at } => {
                let kind = radial.effective(self.copy_of(id).min(radial.angles().len().saturating_sub(1))).kind;
                pick::radial_bounds(camera, size, &kind, at)
            }
            Selection::Internal => {
                let res = rocket.resolved(id)?;
                let z0 = rocket.part_station(id)?;
                let up = handles::radial_up(camera);
                let r = (res.outer_diameter / 2.0) as f32;
                let zs = [z0 as f32, (z0 + res.length) as f32];
                let pts = zs.into_iter().flat_map(|z| [Vec3::Z * z + up * r, Vec3::Z * z - up * r]);
                let pts: Vec<Point> = pts.map(|p| camera.project(p, size)).collect();
                bounds(&pts)
            }
        })
    }

    /// The cutaway's CUT label and dial, a rectangle the inset and the tags keep off (`None` outside the cutaway).
    pub(super) fn cut_label(&self, size: Size) -> Option<Rectangle> {
        (self.inside == InsideView::Cutaway).then(|| {
            let p = self.camera.project(Vec3::ZERO, size);
            Rectangle::new(Point::new(p.x - 72.0, p.y - 32.0), Size::new(84.0, 44.0))
        })
    }

    pub(super) fn context(&self, size: Size, bounds: Rectangle) -> Context<'_> {
        let (rocket, camera) = self.selected.and_then(|id| self.frame_of(id)).unwrap_or((self.rocket, &self.camera));
        let bounds = self.cut_label(size).map_or(bounds, |c| bounds.union(&c));
        let avoid = self.balance.as_ref().map(|marks| markers::balance_label_rects(self.rocket, &self.camera, size, marks, self.units)).unwrap_or_default();
        Context { rocket, camera, size, instance: self.instance(), bounds, cube: !self.profile, avoid }
    }

    /// The tag that selects the pod set while a part inside it is selected: its place (below the pod's copies on screen), the
    /// pod and its text (`POD ×2`).
    pub(super) fn pod_chip(&self, size: Size) -> Option<(PartId, Rectangle, String)> {
        let view = self.pod.as_ref().filter(|v| self.selected.is_some_and(|id| self.rocket.pod_of(id).is_some_and(|p| p.id == v.pod)))?;
        let pod = self.rocket.radial(view.pod)?;
        let union = (0..pod.angles().len())
            .filter_map(|i| pick::placement(self.rocket, pod, i))
            .map(|at| pick::radial_bounds(&self.camera, size, &pod.kind, at))
            .reduce(|a, b| a.union(&b))?;
        let text = if pod.count > 1 { format!("POD ×{}", pod.count) } else { "POD".to_string() };
        let width = text.chars().count() as f32 * TEXT_LABEL * GLYPH_EM + 12.0;
        let mut chip = Rectangle::new(Point::new(union.x, union.y + union.height + 6.0), Size::new(width, TEXT_LABEL + 8.0));
        // Keep off the CG/CP glyphs and their labels: step down until clear.
        if let Some(marks) = &self.balance {
            let rects = markers::balance_label_rects(self.rocket, &self.camera, size, marks, self.units);
            for _ in 0..8 {
                if !rects.iter().any(|r| r.intersects(&chip)) {
                    break;
                }
                chip.y += chip.height + 4.0;
            }
        }
        Some((pod.id, chip, text))
    }

    pub(super) fn visible(&self, size: Size, modifiers: keyboard::Modifiers) -> Visible {
        // Handles edit one part's dimension; with several parts selected there is no such part, so none show.
        if !self.also.is_empty() {
            return Visible::default();
        }
        let (Some(id), Some(b)) = (self.selected, self.bounds_of_selection(size)) else {
            return Visible::default();
        };
        visible(&self.context(size, b), id, modifiers.alt() || self.profile, self.inside)
    }

    pub(super) fn handle_at(&self, at: Point, size: Size, modifiers: keyboard::Modifiers) -> Option<Placed> {
        self.visible(size, modifiers)
            .placed
            .into_iter()
            .filter(|p| (p.at.x - at.x).abs() <= HIT && (p.at.y - at.y).abs() <= HIT)
            .min_by(|a, b| a.at.distance(at).total_cmp(&b.at.distance(at)))
    }

    pub(super) fn hover_at(&self, at: Point, size: Size, modifiers: keyboard::Modifiers) -> Hover {
        if let Some(h) = self.handle_at(at, size, modifiers) {
            return Hover::Handle(h.spec.dim);
        }
        pick::pick_inside(self.rocket, &self.camera, at, size, self.shown_inside()).map_or(Hover::None, |p| Hover::Part(p.id()))
    }

    pub(super) fn quantity(dim: Dim) -> Quantity {
        match dim {
            Dim::Cant | Dim::Angle => Quantity::Angle,
            Dim::Shape => Quantity::Scalar,
            _ => Quantity::Length,
        }
    }

    /// `value` as a number in the display unit.
    pub(super) fn num(&self, dim: Dim, value: f64) -> String {
        units::format_in(value, Self::quantity(dim).unit_for(self.units))
    }

    /// `value` with its unit symbol, `76.20 mm`.
    pub(super) fn with_unit(&self, dim: Dim, value: f64) -> String {
        let unit = Self::quantity(dim).unit_for(self.units);
        if unit.symbol.is_empty() {
            units::format_in(value, unit)
        } else {
            format!("{} {}", units::format_in(value, unit), unit.symbol)
        }
    }

    /// A signed change, `+58.0`.
    pub(super) fn delta(&self, dim: Dim, delta: f64) -> String {
        let text = self.num(dim, delta);
        if delta > 0.0 && !text.starts_with('-') && text.chars().any(|c| c.is_ascii_digit() && c != '0') {
            format!("+{text}")
        } else {
            text
        }
    }

    /// The handle's own label: `SWEEP 60.0`, `Ø 76.2`.
    pub(super) fn handle_label(&self, id: PartId, dim: Dim, value: f64) -> String {
        match dim {
            Dim::Diameter | Dim::ForeDiameter | Dim::InnerDiameter => format!("Ø {}", self.num(dim, value)),
            _ => format!("{} {}", self.rocket.dim_label(id, dim), self.num(dim, value)),
        }
    }

    /// Parses a typed value in the display unit of `dim`.
    pub(super) fn parse(&self, dim: Dim, text: &str) -> Option<f64> {
        let q = Self::quantity(dim);
        units::parse_quantity(text, q, q.unit_for(self.units), &|_| None).ok()
    }

    /// Screen px per unit of value for catching targets: along the line, or the lever of an arc.
    pub(super) fn px_per_unit(placed: &Placed) -> f32 {
        match placed.axis {
            Axis::Line { per_unit, .. } => per_unit,
            Axis::Arc { centre } => placed.at.distance(centre).max(1.0),
            Axis::Plane { .. } => 1.0,
        }
    }

    /// The drag at `at` with `modifiers`: updates its live state and returns the value to set, if it changed.
    pub(super) fn drag_to(&self, drag: &mut Drag, at: Point, modifiers: keyboard::Modifiers) -> Option<Edit> {
        let (id, dim) = (drag.id, drag.placed.spec.dim);
        let coarse = modifiers.control() || modifiers.logo();
        if let (Dim::Vertex(i), Axis::Plane { .. }) = (dim, drag.placed.axis) {
            let (z, h) = handles::vertex_after(&drag.placed, drag.base_vertex, drag.from, at);
            let step = snap::step(Dim::Length, self.snap_step, coarse);
            let (z, h) = (snap::quantize(z, step), snap::quantize(h, step).max(0.0));
            drag.live = Live { value: z, caught: None, limit: None };
            return (drag.last != Some((z, h))).then(|| {
                drag.last = Some((z, h));
                Edit::Vertex(id, i, z, h)
            });
        }
        let raw = handles::value_after(&drag.placed, drag.base, drag.from, at);
        let ppu = Self::px_per_unit(&drag.placed);
        let snapped = snap::snap(dim, raw, &drag.targets, ppu, modifiers.shift(), coarse, self.snap_step);
        let (applied, limit) = self.rocket.clamp_dim(id, dim, snapped.value).unwrap_or((snapped.value, None));
        drag.live = Live { value: applied, caught: snapped.caught.filter(|_| limit.is_none()), limit };
        (drag.last != Some((snapped.value, 0.0))).then(|| {
            drag.last = Some((snapped.value, 0.0));
            Edit::Dim(id, dim, snapped.value)
        })
    }

    /// The edge of a selected freeform fin under `at` (the index of its first vertex) with the `(z, h)` there.
    pub(super) fn edge_at(&self, at: Point, vis: &Visible) -> Option<(PartId, usize, (f64, f64))> {
        let id = self.selected?;
        let verts: Vec<&Placed> = {
            let mut v: Vec<&Placed> = vis.placed.iter().filter(|p| matches!(p.spec.dim, Dim::Vertex(_))).collect();
            v.sort_by_key(|p| match p.spec.dim {
                Dim::Vertex(i) => i,
                _ => 0,
            });
            v
        };
        (0..verts.len()).find_map(|i| {
            let (a, b) = (verts[i], verts[(i + 1) % verts.len()]);
            let ab = b.at - a.at;
            let len2 = ab.x * ab.x + ab.y * ab.y;
            if len2 < 1.0 {
                return None;
            }
            let t = (((at.x - a.at.x) * ab.x + (at.y - a.at.y) * ab.y) / len2).clamp(0.0, 1.0);
            let foot = a.at + ab * t;
            (foot.distance(at) <= 6.0 && i + 1 < verts.len()).then(|| {
                let Dim::Vertex(ia) = a.spec.dim else { return None };
                let base = self.rocket.vertex(id, ia)?;
                Some((id, ia, handles::vertex_after(a, base, a.at, foot)))
            })?
        })
    }
}
