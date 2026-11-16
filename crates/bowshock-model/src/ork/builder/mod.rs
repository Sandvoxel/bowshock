mod axial;
mod internal;
mod radial;

use roxmltree::Node;

use super::xml::{axial_offset, child, family_of, name_of, num};
use super::{Builder, Parent};
use crate::geometry::Shoulder;
use crate::material::{materials_of_kind, MaterialId, MaterialKind};
use crate::rocket::Rocket;
use crate::PartId;
use std::collections::HashMap;

impl<'w> Builder<'w> {
    pub(in crate::ork) fn new(w: &'w mut Vec<String>) -> Builder<'w> {
        Builder { r: Rocket::default(), w, mounts: Vec::new(), recovery: Vec::new(), stage: 0, x: 0.0, radii: HashMap::new(), reported: Vec::new() }
    }

    pub(in crate::ork) fn warn(&mut self, message: impl Into<String>) {
        let message = message.into();
        if !self.w.contains(&message) {
            self.w.push(message);
        }
    }

    pub(in crate::ork) fn fresh(&mut self) -> PartId {
        let id = self.r.next_id;
        self.r.next_id += 1;
        id
    }

    /// The model material of `<material type density>Name</material>` of `n` (the nearest density of the same kind) and the
    /// factor from the model material's density to OpenRocket's.
    pub(in crate::ork) fn material(&mut self, n: Node, tag: &str, kind: MaterialKind, default: MaterialId) -> (MaterialId, f64) {
        let Some(m) = child(n, tag) else {
            return (default, 1.0);
        };
        let given = m.attribute("type").unwrap_or("bulk");
        let want = match kind {
            MaterialKind::Bulk => "bulk",
            MaterialKind::Surface => "surface",
            MaterialKind::Line => "line",
        };
        let name = m.text().unwrap_or("").trim().to_string();
        let Some(density) = m.attribute("density").and_then(|d| d.parse::<f64>().ok()).filter(|d| *d > 0.0) else {
            return (default, 1.0);
        };
        if given != want {
            self.warn(format!("material \"{name}\" is {given} stock where {want} stock is needed; using the default material"));
            return (default, 1.0);
        }
        // The family the name gives (plywood, fiberglass, ...) first, so a part keeps its stiffness (shear modulus) with its name;
        // only without a recognisable name does the nearest density decide.
        let family = if kind == MaterialKind::Bulk { family_of(&name) } else { None };
        let nearest = |ids: Option<&[MaterialId]>| {
            materials_of_kind(kind)
                .filter(|m| ids.is_none_or(|ids| ids.contains(&m.id)))
                .min_by(|a, b| (density / a.density).ln().abs().total_cmp(&(density / b.density).ln().abs()))
        };
        let best = nearest(family).or_else(|| nearest(None));
        let Some(best) = best else {
            return (default, 1.0);
        };
        let ratio = density / best.density;
        if (ratio - 1.0).abs() > 0.02 && !self.reported.contains(&name) {
            self.reported.push(name.clone());
            self.w.push(format!(
                "material \"{name}\" ({density} {}) is not in the model's table: using {} ({}) and scaling the part masses to match",
                kind.density_unit(),
                best.name,
                best.density
            ));
        }
        (best.id, ratio)
    }

    /// Applies OpenRocket's mass, CG and drag overrides of `n` to part `id`; otherwise, when the material density differed from
    /// the model's by `scale` (or the caller knows the OpenRocket mass, `expected`), overrides the mass so it matches.
    pub(in crate::ork) fn settle_mass(&mut self, id: PartId, n: Node, scale: f64, expected: Option<f64>) {
        if let Some(m) = num(n, "overridemass") {
            self.r.set_mass_override(id, Some(m));
        } else if let Some(e) = expected {
            let model = self.r.component_mass(id).map_or(0.0, |m| m.mass);
            if model <= 0.0 || (e / model - 1.0).abs() > 0.002 {
                self.r.set_mass_override(id, Some(e));
            }
        } else if (scale - 1.0).abs() > 0.002 {
            if let Some(m) = self.r.component_mass(id) {
                self.r.set_mass_override(id, Some(m.mass * scale));
            }
        }
        if let Some(c) = num(n, "overridecg") {
            self.r.set_cg_override(id, Some(c));
        }
        if child(n, "overridecd").is_some() {
            let applied = num(n, "overridecd").is_some_and(|cd| self.r.set_cd_override(id, Some(cd)));
            if !applied {
                let name = name_of(n).unwrap_or_else(|| n.tag_name().name().to_string());
                self.warn(format!("{name}: drag override is not modelled"));
            }
        }
    }

    /// Where a component of length `len` with the placement of `n` starts, given its parent and the end of the previous sibling.
    pub(in crate::ork) fn place(&self, n: Node, p: &Parent, len: f64, after: f64) -> f64 {
        let (method, v) = axial_offset(n);
        match method.as_str() {
            "middle" => p.top + p.len / 2.0 - len / 2.0 + v,
            "bottom" => p.top + p.len - len + v,
            "absolute" => v,
            "after" => after + v,
            _ => p.top + v,
        }
    }
}

pub(in crate::ork) fn shoulder(n: Node, which: &str) -> Option<Shoulder> {
    let length = num(n, &format!("{which}shoulderlength")).unwrap_or(0.0);
    if length <= 0.0 {
        return None;
    }
    Some(Shoulder {
        length,
        diameter: 2.0 * num(n, &format!("{which}shoulderradius")).unwrap_or(0.0),
        thickness: num(n, &format!("{which}shoulderthickness")).unwrap_or(0.002),
    })
}
