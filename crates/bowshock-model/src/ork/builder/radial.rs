use roxmltree::Node;

use super::super::xml::*;
use super::super::{Builder, MountRec, Parent, RecoveryRec};
use crate::geometry::{FinSection, Part, PartKind, PodBody, RadialKind, RadialPart};
use crate::material::{MaterialId, MaterialKind};
use crate::PartId;

impl<'w> Builder<'w> {
    pub(in crate::ork) fn push_radial(&mut self, n: Node, p: &Parent, top: f64, angle: f64, count: u8, kind: RadialKind, material: MaterialId) -> PartId {
        let id = self.fresh();
        let mut part = RadialPart::new(id, kind, p.axial, top - p.axial_station, angle.rem_euclid(std::f64::consts::TAU), count);
        part.material = material;
        part.name = name_of(n);
        part.finish = finish_of(n);
        self.r.radials.push(part);
        id
    }

    pub(in crate::ork) fn angle_of(n: Node) -> f64 {
        deg(num(n, "rotation").or_else(|| child(n, "angleoffset").and_then(|c| c.text()).and_then(|t| t.trim().parse().ok())).unwrap_or(0.0))
    }

    pub(in crate::ork) fn fin_set(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let label = name_of(n).unwrap_or_else(|| "Fin set".to_string());
        let thickness = num(n, "thickness").unwrap_or(0.003);
        let (kind, root) = match n.tag_name().name() {
            "trapezoidfinset" => {
                let root = num(n, "rootchord").unwrap_or(0.1);
                (
                    RadialKind::FinSet {
                        root_chord: root,
                        tip_chord: num(n, "tipchord").unwrap_or(0.05),
                        span: num(n, "height").unwrap_or(0.05),
                        sweep: num(n, "sweeplength").unwrap_or(0.0),
                        thickness,
                    },
                    root,
                )
            }
            "ellipticalfinset" => {
                let root = num(n, "rootchord").unwrap_or(0.1);
                (RadialKind::EllipticalFin { root_chord: root, span: num(n, "height").unwrap_or(0.05), thickness }, root)
            }
            _ => {
                let mut points: Vec<(f64, f64)> = child(n, "finpoints")
                    .map(|f| children(f, "point").filter_map(|c| Some((c.attribute("x")?.parse().ok()?, c.attribute("y")?.parse().ok()?))).collect())
                    .unwrap_or_default();
                let x0 = points.first().map_or(0.0, |q| q.0);
                for q in &mut points {
                    q.0 -= x0;
                }
                let root = points.last().map_or(0.1, |q| q.0.abs());
                (RadialKind::FreeformFin { points, thickness }, root)
            }
        };
        let top = self.place(n, p, root, after);
        let count = num(n, "fincount").map_or(3.0, |v| v.max(1.0));
        if count > 16.0 {
            self.warn(format!("{label}: {count} fins exceed the model's 16; imported with 16"));
        }
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::BIRCH_PLYWOOD);
        // The fin's own stock sets its shear modulus for the flutter estimate: say so when only its density picked the table entry.
        if let Some(m) = child(n, "material").filter(|m| m.attribute("type").is_none_or(|t| t == "bulk")) {
            let name = m.text().unwrap_or("").trim();
            if family_of(name).is_none() {
                let g = material.shear_modulus() / 1.0e9;
                self.warn(format!(
                    "{label}: fin material \"{name}\" is not recognised, so its shear modulus is unknown; the flutter estimate uses {} ({g:.2} GPa) by density (set the fin's own value to override)",
                    material.name()
                ));
            }
        }
        let id = self.push_radial(n, p, top, Self::angle_of(n), count.min(16.0) as u8, kind, material);
        let section = match text(n, "crosssection") {
            Some("rounded") => FinSection::Rounded,
            Some("airfoil") => FinSection::Airfoil,
            _ => FinSection::Square,
        };
        let fillet = num(n, "filletradius").unwrap_or(0.0);
        let (fillet_material, fillet_scale) = self.material(n, "filletmaterial", MaterialKind::Bulk, MaterialId::EPOXY);
        // A tab (the part of the root that goes through the airframe wall) adds the mass of its plate; it has no aerodynamics.
        let tab = fin_tab_of(n, root);
        if let Some(r) = self.r.radials.iter_mut().find(|r| r.id == id) {
            r.cross_section = section;
            r.cant = deg(num(n, "cant").unwrap_or(0.0));
            r.fillet_radius = fillet.max(0.0);
            r.fillet_material = fillet_material;
            r.tab = tab;
        }
        let _ = fillet_scale;
        self.settle_mass(id, n, scale, None);
        top + root
    }

    pub(in crate::ork) fn lug_or_button(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let button = n.tag_name().name() == "railbutton";
        let label = name_of(n).unwrap_or_else(|| if button { "Rail button" } else { "Launch lug" }.to_string());
        let (kind, len) = if button {
            let d = num(n, "outerdiameter").unwrap_or(0.01);
            (RadialKind::RailButton { height: num(n, "height").unwrap_or(0.01), diameter: d }, 0.0)
        } else {
            let len = num(n, "length").unwrap_or(0.03);
            (RadialKind::LaunchLug { length: len, diameter: 2.0 * num(n, "radius").unwrap_or(0.004) }, len)
        };
        let top = self.place(n, p, len, after);
        let instances = num(n, "instancecount").map_or(1, |v| v.max(1.0) as usize);
        let separation = num(n, "instanceseparation").unwrap_or(0.0);
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, if button { MaterialId::ALUMINUM } else { MaterialId::CARDBOARD });
        let ids: Vec<PartId> = (0..instances)
            .map(|k| {
                // A rail button sits at its centre: the model's offset is its fore edge.
                let fore = top + separation * k as f64 - if button { num(n, "outerdiameter").unwrap_or(0.01) / 2.0 } else { 0.0 };
                self.push_radial(n, p, fore, Self::angle_of(n), 1, kind.clone(), material)
            })
            .collect();
        let per = num(n, "overridemass").map(|m| m / instances as f64);
        for &id in &ids {
            match per {
                Some(m) => {
                    self.r.set_mass_override(id, Some(m));
                }
                None => self.settle_mass(id, n, scale, None),
            }
        }
        if let Some(c) = num(n, "overridecg") {
            for &id in &ids {
                self.r.set_cg_override(id, Some(c));
            }
        }
        // OpenRocket's drag override is for the whole component, shared out over the instances.
        if let Some(cd) = num(n, "overridecd") {
            for &id in &ids {
                self.r.set_cd_override(id, Some(cd / instances as f64));
            }
        }
        let _ = label;
        top + len
    }

    /// A tube fin set is a pod of one open tube (the sim flies an open pod as tube fins).
    pub(in crate::ork) fn tube_fins(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let label = name_of(n).unwrap_or_else(|| "Tube fin set".to_string());
        let len = num(n, "length").unwrap_or(0.05);
        let top = self.place(n, p, len, after);
        // An `auto` tube radius is the outer radius of the tube it stands on (OpenRocket's TubeFinSet).
        let parent_radius = self.r.parts.iter().find(|x| x.id == p.axial).map_or(0.0125, |x| x.kind.aft_diameter() / 2.0);
        let radius = dim(n, "radius").and_then(Dim::explicit).unwrap_or(parent_radius);
        let _ = &label;
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::CARDBOARD);
        let kind = PartKind::BodyTube { length: len, diameter: 2.0 * radius };
        let mut part = Part::new(0, kind);
        part.material = material;
        part.finish = finish_of(n);
        part.thickness = num(n, "thickness").unwrap_or(0.001).min(radius);
        part.name = name_of(n);
        // Mass: OpenRocket's tube is an annulus of the stock's density.
        if num(n, "overridemass").is_none() && (scale - 1.0).abs() > 0.002 {
            let ro = radius;
            let ri = (radius - part.thickness).max(0.0);
            part.mass_override = Some(std::f64::consts::PI * (ro * ro - ri * ri) * len * material.density() * scale);
        } else {
            part.mass_override = num(n, "overridemass").map(|m| m / num(n, "fincount").unwrap_or(1.0).max(1.0));
        }
        let count = num(n, "fincount").map_or(3.0, |v| v.max(1.0)).min(16.0) as u8;
        let body = PodBody { parts: vec![part], radials: Vec::new(), internals: Vec::new() };
        self.attach_pod(n, p, top, Self::angle_of(n), count, 0.0, body, Vec::new(), Vec::new());
        top + len
    }

    // ---- pods and parallel stages ----

    pub(in crate::ork) fn pod(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let parallel = n.tag_name().name() == "parallelstage";
        let label = name_of(n).unwrap_or_else(|| "Pod".to_string());
        if parallel {
            self.warn(format!("{label}: a parallel stage is imported as a pod that burns and flies with its stage (separation events are not modelled)"));
        }
        let mut inner = Builder::new(self.w);
        inner.stage = self.stage;
        if let Some(subs) = child(n, "subcomponents") {
            let axial: Vec<Node> =
                subs.children().filter(|c| c.is_element() && matches!(c.tag_name().name(), "nosecone" | "bodytube" | "transition")).collect();
            inner.plan_radii(&axial);
            inner.stack(subs);
        }
        let length = inner.x;
        let Builder { r, mounts, recovery, .. } = inner;
        let body = PodBody { parts: r.parts, radials: r.radials, internals: r.internals };
        let gap = match child(n, "radiusoffset").and_then(|c| c.attribute("method")) {
            Some("surface") | Some("relative") | None => num(n, "radiusoffset").unwrap_or(0.0).max(0.0),
            Some(other) => {
                self.warn(format!("{label}: radius offset method \"{other}\" is not modelled; the pod stands on the surface"));
                0.0
            }
        };
        let top = self.place(n, p, length, after);
        let count = num(n, "instancecount").map_or(1.0, |v| v.max(1.0)).min(16.0) as u8;
        self.attach_pod(n, p, top, Self::angle_of(n), count, gap, body, mounts, recovery);
        top + length
    }

    /// Attaches `body` as a pod set and carries the mounts and recovery devices found inside it over to the final part ids.
    pub(in crate::ork) fn attach_pod(
        &mut self,
        n: Node,
        p: &Parent,
        top: f64,
        angle: f64,
        count: u8,
        gap: f64,
        mut body: PodBody,
        mounts: Vec<MountRec>,
        recovery: Vec<RecoveryRec>,
    ) {
        let old: Vec<PartId> = body.ids().collect();
        let id = self.fresh();
        self.r.next_id = body.reid(self.r.next_id);
        let new: Vec<PartId> = body.ids().collect();
        let remap = |part: PartId| old.iter().position(|o| *o == part).and_then(|i| new.get(i).copied());
        for mut m in mounts {
            if let Some(part) = remap(m.part) {
                m.part = part;
                m.stage = self.stage;
                self.mounts.push(m);
            }
        }
        for mut rec in recovery {
            if let Some(part) = remap(rec.part) {
                rec.part = part;
                self.recovery.push(rec);
            }
        }
        let mut set = RadialPart::new(id, RadialKind::Pod { gap, body }, p.axial, top - p.axial_station, angle.rem_euclid(std::f64::consts::TAU), count);
        set.name = name_of(n);
        self.r.radials.push(set);
    }
}
