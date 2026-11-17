use roxmltree::Node;

use super::super::xml::*;
use super::super::{Builder, Parent};
use super::shoulder;
use crate::geometry::{NoseShape, PartKind};
use crate::material::MaterialKind;
use crate::rocket::Rocket;
use crate::PartId;

impl<'w> Builder<'w> {
    /// Resolves the radii of a run of axial components the way OpenRocket does: an `auto` radius follows the neighbour (a
    /// tube's and a transition's fore radius the aft radius of the part ahead, a nose cone's or transition's aft radius the
    /// fore radius of the part behind). The number stored next to `auto` can be stale, so it is only the last resort.
    pub(in crate::ork) fn plan_radii(&mut self, nodes: &[Node]) {
        #[derive(Clone, Copy)]
        enum Spec {
            Fixed(f64),
            Auto(Option<f64>),
        }
        let spec = |n: Node, tag: &str| match dim(n, tag) {
            Some(Dim::Val(v)) => Spec::Fixed(v),
            Some(Dim::Auto(v)) => Spec::Auto(v),
            None => Spec::Auto(None),
        };
        let mut items: Vec<(Spec, Spec, bool)> = Vec::new(); // (fore, aft, tube: one radius)
        for n in nodes {
            items.push(match n.tag_name().name() {
                "nosecone" if text(*n, "isflipped") == Some("true") => (spec(*n, "aftradius"), Spec::Fixed(0.0), false),
                "nosecone" => (Spec::Fixed(0.0), spec(*n, "aftradius"), false),
                "bodytube" => (spec(*n, "radius"), spec(*n, "radius"), true),
                _ => (spec(*n, "foreradius"), spec(*n, "aftradius"), false),
            });
        }
        let value = |s: Spec| if let Spec::Fixed(v) = s { Some(v) } else { None };
        let mut r: Vec<(Option<f64>, Option<f64>)> = items.iter().map(|i| (value(i.0), value(i.1))).collect();
        let k = items.len();
        loop {
            let mut changed = true;
            while changed {
                changed = false;
                for i in 0..k {
                    let from_prev = if i > 0 { r[i - 1].1 } else { None };
                    let from_next = if i + 1 < k { r[i + 1].0 } else { None };
                    if items[i].2 {
                        if r[i].0.is_none() {
                            if let Some(v) = from_prev.or(from_next) {
                                r[i] = (Some(v), Some(v));
                                changed = true;
                            }
                        }
                        continue;
                    }
                    if r[i].0.is_none() {
                        if let Some(v) = from_prev {
                            r[i].0 = Some(v);
                            changed = true;
                        }
                    }
                    if r[i].1.is_none() {
                        if let Some(v) = from_next {
                            r[i].1 = Some(v);
                            changed = true;
                        }
                    }
                }
            }
            // Anything still open is cut by its stored number (or a default) and the neighbours follow.
            let Some(i) = (0..k).find(|&i| r[i].0.is_none() || r[i].1.is_none()) else {
                break;
            };
            let stale = |s: Spec| if let Spec::Auto(Some(v)) = s { v } else { 0.0125 };
            if r[i].0.is_none() {
                r[i].0 = Some(stale(items[i].0));
            }
            if r[i].1.is_none() {
                r[i].1 = Some(if items[i].2 { r[i].0.unwrap_or(0.0125) } else { stale(items[i].1) });
            }
        }
        for (n, (fore, aft)) in nodes.iter().zip(r) {
            self.radii.insert(n.id(), (fore.unwrap_or(0.0), aft.unwrap_or(0.0)));
        }
    }

    /// Adds every body component of a `<subcomponents>` (a stage, or a pod body) to the stack.
    pub(in crate::ork) fn stack(&mut self, subs: Node) {
        for c in subs.children().filter(Node::is_element) {
            match c.tag_name().name() {
                "nosecone" | "bodytube" | "transition" => self.axial(c),
                other => {
                    let name = name_of(c).unwrap_or_else(|| other.to_string());
                    self.warn(format!("{name}: a {other} directly in a stage is not supported"));
                }
            }
        }
    }

    pub(in crate::ork) fn axial(&mut self, n: Node) {
        let tag = n.tag_name().name();
        let name = name_of(n);
        let label = name.clone().unwrap_or_else(|| tag.to_string());
        let length = num(n, "length").unwrap_or(0.0).max(0.0);
        let (rf, ra) = self.radii.get(&n.id()).copied().unwrap_or((0.0125, 0.0125));
        let (kind, shoulders) = match tag {
            "nosecone" if text(n, "isflipped") == Some("true") => {
                self.warn(format!("{label}: a flipped nose cone (tail cone) is imported as a conical transition to a point"));
                (PartKind::Transition { length, fore_diameter: 2.0 * rf, aft_diameter: 0.0 }, [None, None])
            }
            "nosecone" => {
                let shape = self.nose_shape(&label, text(n, "shape").unwrap_or("ogive"), num(n, "shapeparameter").unwrap_or(1.0), length, 2.0 * ra);
                (PartKind::NoseCone { shape, length, diameter: 2.0 * ra, tip_radius: 0.0 }, [None, shoulder(n, "aft")])
            }
            "bodytube" => (PartKind::BodyTube { length, diameter: 2.0 * rf }, [None, None]),
            _ => {
                if text(n, "shape").is_some_and(|s| s != "conical") {
                    self.warn(format!("{label}: only conical transitions are modelled ({} imported as conical)", text(n, "shape").unwrap_or("")));
                }
                (PartKind::Transition { length, fore_diameter: 2.0 * rf, aft_diameter: 2.0 * ra }, [shoulder(n, "fore"), shoulder(n, "aft")])
            }
        };
        let at = self.r.parts.len();
        let id = match self.r.insert(at, kind.clone()) {
            Ok(id) => id,
            Err(e) => {
                self.warn(format!("{label}: cannot be placed in the stack ({e}); skipped with its contents"));
                return;
            }
        };
        self.r.set_kind(id, kind.clone());
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, kind.default_material());
        let thickness = match text(n, "thickness") {
            Some("filled") => f64::INFINITY,
            _ => num(n, "thickness").unwrap_or(0.002),
        };
        if let Some(p) = self.r.parts.iter_mut().find(|p| p.id == id) {
            p.name = name;
            p.material = material;
            p.finish = finish_of(n);
            if let Some(s) = shoulders[0] {
                p.shoulder_fore = s;
            }
            if let Some(s) = shoulders[1] {
                p.shoulder_aft = s;
            }
        }
        self.r.set_thickness(id, thickness);
        let station = self.x;
        self.x += length;
        // Children are placed relative to this part.
        let bore = self.r.parts.iter().find(|p| p.id == id).map_or(0.0, |p| p.inner_diameter_at(length / 2.0));
        let parent = Parent { axial: id, axial_station: station, top: station, len: length, bore, internal: false };
        let first = self.r.next_id;
        self.children(n, parent);
        self.zero_children(n, first);
        if tag == "bodytube" {
            if let Some(m) = child(n, "motormount") {
                self.body_tube_mount(&label, &parent, m);
            }
        }
        // A capped shoulder has a disc closing its far end (the model's shoulder is an open tube): its mass and position are
        // added to the part's by overriding both.
        let density = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(0.0);
        let mut caps: Vec<(f64, f64)> = Vec::new(); // (mass, position behind the part's fore face)
        for which in ["fore", "aft"] {
            if text(n, &format!("{which}shouldercapped")) != Some("true") {
                continue;
            }
            let r = num(n, &format!("{which}shoulderradius")).unwrap_or(0.0);
            let (t, l) = (num(n, &format!("{which}shoulderthickness")).unwrap_or(0.0), num(n, &format!("{which}shoulderlength")).unwrap_or(0.0));
            let at = if which == "aft" { length + l - t / 2.0 } else { -l + t / 2.0 };
            let inner = (r - t).max(0.0);
            caps.push((std::f64::consts::PI * inner * inner * t.min(r) * density, at));
        }
        let cap: f64 = caps.iter().map(|c| c.0).sum();
        if cap > 0.0 && num(n, "overridemass").is_none() && num(n, "overridecg").is_none() {
            if let Some(m) = self.r.component_mass(id) {
                let (mass, cg) = (m.mass * scale, m.cg - station);
                let total = mass + cap;
                self.r.set_mass_override(id, Some(total));
                self.r.set_cg_override(id, Some((mass * cg + caps.iter().map(|c| c.0 * c.1).sum::<f64>()) / total));
            }
        } else {
            self.settle_mass(id, n, scale, None);
        }
    }

    /// With "override subcomponents mass" the part's override already includes everything inside it: the parts created since
    /// `first` weigh nothing of their own.
    pub(in crate::ork) fn zero_children(&mut self, n: Node, first: PartId) {
        if text(n, "overridesubcomponentsmass") == Some("true") {
            for id in first..self.r.next_id {
                self.r.set_mass_override(id, Some(0.0));
            }
        }
    }

    /// A mass override on a whole `<stage>` that includes its subcomponents sets the stage's total: every part made since
    /// `first` is scaled by one factor so they add up to it (their CGs, and so the stage's, are kept).
    pub(in crate::ork) fn stage_override(&mut self, stage: Node, first: PartId) {
        let Some(total) = num(stage, "overridemass") else {
            return;
        };
        let name = name_of(stage).unwrap_or_else(|| "stage".to_string());
        if text(stage, "overridesubcomponentsmass") != Some("true") {
            self.warn(format!("{name}: a mass override of the stage itself (without its subcomponents) is not modelled"));
            return;
        }
        let ids: Vec<PartId> = (first..self.r.next_id).filter(|&id| !self.r.is_pod(id)).collect();
        let copies = |r: &Rocket, id: PartId| r.pod_of(id).map_or(1.0, |p| f64::from(p.count.max(1)));
        let sum: f64 = ids.iter().filter_map(|&id| self.r.component_mass(id)).map(|m| m.mass).sum();
        if sum <= 0.0 {
            return;
        }
        let k = total / sum;
        for id in ids {
            if let Some(m) = self.r.component_mass(id) {
                let one = m.mass / copies(&self.r, id);
                self.r.set_mass_override(id, Some(one * k));
            }
        }
        if num(stage, "overridecg").is_some() {
            self.warn(format!("{name}: a CG override of the whole stage is not modelled"));
        }
    }

    pub(in crate::ork) fn nose_shape(&mut self, label: &str, shape: &str, param: f64, length: f64, diameter: f64) -> NoseShape {
        match shape {
            "conical" => NoseShape::Conical,
            "ogive" => {
                if (param - 1.0).abs() > 1e-6 {
                    self.warn(format!("{label}: secant ogive (shape parameter {param}) imported as a tangent ogive"));
                }
                NoseShape::TangentOgive
            }
            "ellipsoid" => NoseShape::Elliptical,
            "power" => NoseShape::PowerSeries(param.clamp(0.01, 1.0)),
            "haack" => {
                if param.abs() < 1e-9 {
                    NoseShape::VonKarman
                } else {
                    NoseShape::Haack(param)
                }
            }
            "parabolic" => {
                if (param - 1.0).abs() < 1e-6 {
                    NoseShape::Parabolic
                } else {
                    // r/R = (2t - K t^2)/(2 - K): the closest power series r/R = t^n.
                    let k = param.clamp(0.0, 1.0);
                    let err =
                        |n: f64| -> f64 { (1..=32).map(|i| f64::from(i) / 32.0).map(|t: f64| ((2.0 * t - k * t * t) / (2.0 - k) - t.powf(n)).powi(2)).sum() };
                    let n = (30..=100).map(|i| f64::from(i) / 100.0).min_by(|a, b| err(*a).total_cmp(&err(*b))).unwrap_or(0.5);
                    self.warn(format!("{label}: parabolic nose with K = {param} imported as the power series n = {n:.2}"));
                    let _ = (length, diameter);
                    NoseShape::PowerSeries(n)
                }
            }
            other => {
                self.warn(format!("{label}: unknown nose shape \"{other}\" imported as a tangent ogive"));
                NoseShape::TangentOgive
            }
        }
    }
}
