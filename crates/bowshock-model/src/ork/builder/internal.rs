use roxmltree::Node;

use super::super::xml::*;
use super::super::{Builder, IgnitionSpec, MotorSpec, MountRec, Parent, RecoveryRec};
use crate::config::MotorRef;
use crate::internal::{DeployEvent, Deployment, InternalKind, InternalPart, MassRole};
use crate::material::{MaterialId, MaterialKind};
use crate::PartId;

impl<'w> Builder<'w> {
    /// Adds the components inside `n`'s `<subcomponents>` placed against `p`.
    pub(in crate::ork) fn children(&mut self, n: Node, p: Parent) {
        let Some(subs) = child(n, "subcomponents") else {
            return;
        };
        let mut after = p.top;
        for c in subs.children().filter(Node::is_element) {
            let end = match c.tag_name().name() {
                "innertube" => self.inner_tube(c, &p, after),
                "tubecoupler" => self.coupler(c, &p, after),
                "bulkhead" => self.disc(c, &p, after, false),
                "centeringring" => self.disc(c, &p, after, true),
                "engineblock" => self.engine_block(c, &p, after),
                "parachute" => self.parachute(c, &p, after),
                "streamer" => self.streamer(c, &p, after),
                "shockcord" => self.shock_cord(c, &p, after),
                "masscomponent" => self.mass_component(c, &p, after),
                "trapezoidfinset" | "ellipticalfinset" | "freeformfinset" => self.fin_set(c, &p, after),
                "launchlug" | "railbutton" => self.lug_or_button(c, &p, after),
                "tubefinset" => self.tube_fins(c, &p, after),
                "podset" | "parallelstage" => self.pod(c, &p, after),
                "nosecone" | "bodytube" | "transition" => {
                    let name = name_of(c).unwrap_or_else(|| c.tag_name().name().to_string());
                    self.warn(format!("{name}: a body component inside another component is not supported"));
                    after
                }
                other => {
                    let name = name_of(c).unwrap_or_else(|| other.to_string());
                    self.warn(format!("{name}: {other} is not supported and was left out"));
                    after
                }
            };
            after = end;
        }
    }

    pub(in crate::ork) fn push_internal(&mut self, n: Node, p: &Parent, top: f64, kind: InternalKind, material: MaterialId) -> PartId {
        let id = self.fresh();
        let mut part = InternalPart::new(id, kind, p.axial, top - p.axial_station);
        part.material = material;
        part.name = name_of(n);
        self.r.internals.push(part);
        id
    }

    pub(in crate::ork) fn inner_tube(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "length").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let outer = dim(n, "outerradius").and_then(Dim::value).unwrap_or(0.02);
        let thickness = num(n, "thickness").unwrap_or(0.002).min(outer);
        let mount = child(n, "motormount");
        let overhang = mount.and_then(|m| num(m, "overhang")).unwrap_or(0.0);
        let label = name_of(n).unwrap_or_else(|| "Inner tube".to_string());
        let cluster = text(n, "clusterconfiguration").unwrap_or("single");
        if cluster != "single" {
            self.warn(format!("{label}: cluster \"{cluster}\" imported as a single tube (only one motor flies)"));
        }
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::CARDBOARD);
        let kind = InternalKind::InnerTube { outer_diameter: 2.0 * outer, thickness, length: len, motor_mount: mount.is_some(), overhang };
        let id = self.push_internal(n, p, top, kind, material);
        if let Some(m) = mount {
            self.record_mount(id, m);
        }
        let inner = Parent { top, len, bore: 2.0 * (outer - thickness), internal: true, ..*p };
        let first = self.r.next_id;
        self.children(n, inner);
        self.zero_children(n, first);
        self.settle_mass(id, n, scale, None);
        top + len
    }

    /// A body tube that is its own motor mount. The model keeps motors in inner tubes, so the mount becomes an inner tube of
    /// the body tube's bore and length, wall thickness 0 and no mass (the body tube already carries the wall), with the
    /// mount's overhang and motors.
    pub(in crate::ork) fn body_tube_mount(&mut self, label: &str, p: &Parent, m: Node) {
        let overhang = num(m, "overhang").unwrap_or(0.0);
        let kind = InternalKind::InnerTube { outer_diameter: p.bore, thickness: 0.0, length: p.len, motor_mount: true, overhang };
        let id = self.fresh();
        let mut part = InternalPart::new(id, kind, p.axial, p.top - p.axial_station);
        part.name = Some(format!("{label} motor mount"));
        self.r.internals.push(part);
        self.r.set_mass_override(id, Some(0.0));
        self.record_mount(id, m);
        self.warn(format!("{label}: the body tube is its own motor mount; imported as a massless inner tube of its bore ({:.1} mm)", p.bore * 1e3));
    }

    pub(in crate::ork) fn record_mount(&mut self, part: PartId, m: Node) {
        let mut rec = MountRec { part, stage: self.stage, motors: Vec::new(), ignition: Vec::new() };
        if let Some(event) = text(m, "ignitionevent") {
            rec.ignition.push(IgnitionSpec { config: String::new(), event: event.to_string(), delay: num(m, "ignitiondelay").unwrap_or(0.0) });
        }
        for c in m.children().filter(Node::is_element) {
            let config = c.attribute("configid").unwrap_or("").to_string();
            match c.tag_name().name() {
                "motor" => {
                    let designation = text(c, "designation").unwrap_or("").to_string();
                    if designation.is_empty() {
                        continue;
                    }
                    let delay = match text(c, "delay") {
                        Some("none") | None => f64::INFINITY,
                        Some(d) => d.parse().unwrap_or(f64::INFINITY),
                    };
                    let manufacturer = text(c, "manufacturer").unwrap_or("").to_string();
                    let digest = text(c, "digest").unwrap_or("").to_string();
                    rec.motors.push(MotorSpec { config, motor: MotorRef::new(designation, manufacturer), digest, delay });
                }
                "ignitionconfiguration" => rec.ignition.push(IgnitionSpec {
                    config,
                    event: text(c, "ignitionevent").unwrap_or("automatic").to_string(),
                    delay: num(c, "ignitiondelay").unwrap_or(0.0),
                }),
                _ => {}
            }
        }
        self.mounts.push(rec);
    }

    pub(in crate::ork) fn coupler(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "length").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let outer = dim(n, "outerradius").and_then(Dim::explicit).or_else(|| dim(n, "outerradius").and_then(Dim::value));
        let outer_d = outer.map(|r| 2.0 * r);
        let thickness = num(n, "thickness").unwrap_or(0.002);
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::CARDBOARD);
        let kind = InternalKind::Coupler { outer_diameter: outer_d, thickness, length: len };
        let id = self.push_internal(n, p, top, kind, material);
        let inner = Parent { top, len, bore: outer_d.unwrap_or(p.bore) - 2.0 * thickness, internal: true, ..*p };
        let first = self.r.next_id;
        self.children(n, inner);
        self.zero_children(n, first);
        self.settle_mass(id, n, scale, None);
        top + len
    }

    pub(in crate::ork) fn disc(&mut self, n: Node, p: &Parent, after: f64, ring: bool) -> f64 {
        let len = num(n, "length").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        // `auto` follows the OpenRocket parent: an airframe tube's bore, or (inside an inner tube or coupler) that tube's bore.
        let outer = match dim(n, "outerradius") {
            Some(Dim::Auto(_)) | None if p.internal => Some(p.bore),
            d => d.and_then(Dim::value).map(|r| 2.0 * r),
        };
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::BIRCH_PLYWOOD);
        let kind = if ring {
            // An automatic inner radius is the outer radius of a sibling inner tube; inside an inner tube there is none, so the
            // ring is a solid disc.
            let inner = match dim(n, "innerradius") {
                Some(Dim::Auto(_)) | None if p.internal => Some(0.0),
                d => d.and_then(Dim::value).map(|r| 2.0 * r),
            };
            InternalKind::CenteringRing { outer_diameter: outer, inner_diameter: inner, thickness: len }
        } else {
            InternalKind::Bulkhead { diameter: outer, thickness: len }
        };
        let id = self.push_internal(n, p, top, kind, material);
        self.settle_mass(id, n, scale, None);
        top + len
    }

    pub(in crate::ork) fn engine_block(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "length").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let thickness = num(n, "thickness").unwrap_or(0.002);
        let outer = dim(n, "outerradius").and_then(Dim::value).map(|r| 2.0 * r);
        let inner = (outer.unwrap_or(p.bore) - 2.0 * thickness).max(0.0);
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::BIRCH_PLYWOOD);
        // An engine block is only a ring of mass in the simulation: import it as a centering ring.
        let kind = InternalKind::CenteringRing { outer_diameter: outer, inner_diameter: Some(inner), thickness: len };
        let id = self.push_internal(n, p, top, kind, material);
        self.settle_mass(id, n, scale, None);
        top + len
    }

    pub(in crate::ork) fn deployment(&mut self, label: &str, n: Node) -> Deployment {
        let delay = num(n, "deploydelay").unwrap_or(0.0).max(0.0);
        let event = match text(n, "deployevent").unwrap_or("apogee") {
            "ejection" => DeployEvent::Ejection,
            "altitude" => DeployEvent::Altitude(num(n, "deployaltitude").unwrap_or(0.0)),
            "launch" => return Deployment { event: DeployEvent::TimeFromLaunch, delay },
            "apogee" => DeployEvent::Apogee,
            "never" => {
                self.warn(format!("{label}: deployment \"never\" is not modelled; it deploys 1e6 s after launch"));
                return Deployment { event: DeployEvent::TimeFromLaunch, delay: 1.0e6 };
            }
            other => {
                self.warn(format!("{label}: deployment event \"{other}\" is not modelled; deploying at apogee instead"));
                DeployEvent::Apogee
            }
        };
        Deployment { event, delay }
    }

    pub(in crate::ork) fn recovery_rec(&mut self, id: PartId, label: &str, n: Node) {
        let per_config =
            children(n, "deploymentconfiguration").map(|c| (c.attribute("configid").unwrap_or("").to_string(), self.deployment(label, c))).collect::<Vec<_>>();
        self.recovery.push(RecoveryRec { part: id, per_config });
    }

    pub(in crate::ork) fn parachute(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let label = name_of(n).unwrap_or_else(|| "Parachute".to_string());
        let len = num(n, "packedlength").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let diameter = num(n, "diameter").unwrap_or(0.3);
        let cd = dim(n, "cd").and_then(Dim::explicit).unwrap_or(0.8);
        let lines = num(n, "linecount").map_or(6, |v| v.max(0.0) as u32);
        let line_length = num(n, "linelength").unwrap_or(0.3);
        let (material, _) = self.material(n, "material", MaterialKind::Surface, MaterialId::RIPSTOP_NYLON);
        let (line_material, _) = self.material(n, "linematerial", MaterialKind::Line, MaterialId::BRAIDED_NYLON_2MM);
        let surface = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(material.density());
        let line_density = child(n, "linematerial").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(line_material.density());
        let deploy = self.deployment(&label, n);
        let kind = InternalKind::Parachute {
            diameter,
            cd,
            line_count: lines,
            line_length,
            line_material,
            packed_length: len,
            packed_diameter: num(n, "packedradius").map(|r| 2.0 * r),
            deploy,
        };
        let id = self.push_internal(n, p, top, kind, material);
        let expected = std::f64::consts::FRAC_PI_4 * diameter * diameter * surface + f64::from(lines) * line_length * line_density;
        self.settle_mass(id, n, 1.0, Some(expected));
        self.recovery_rec(id, &label, n);
        top + len
    }

    pub(in crate::ork) fn streamer(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let label = name_of(n).unwrap_or_else(|| "Streamer".to_string());
        let len = num(n, "packedlength").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let (length, width) = (num(n, "striplength").unwrap_or(0.5), num(n, "stripwidth").unwrap_or(0.05));
        let cd = dim(n, "cd").and_then(Dim::explicit).unwrap_or(0.8);
        let (material, _) = self.material(n, "material", MaterialKind::Surface, MaterialId::MYLAR);
        let surface = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(material.density());
        let deploy = self.deployment(&label, n);
        let id = self.push_internal(n, p, top, InternalKind::Streamer { length, width, cd, deploy }, material);
        self.settle_mass(id, n, 1.0, Some(length * width * surface));
        self.recovery_rec(id, &label, n);
        top + len
    }

    pub(in crate::ork) fn shock_cord(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "packedlength").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let length = num(n, "cordlength").unwrap_or(0.0);
        let (material, _) = self.material(n, "material", MaterialKind::Line, MaterialId::BRAIDED_NYLON_3MM);
        let density = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(material.density());
        let id = self.push_internal(n, p, top, InternalKind::ShockCord { length, packed_length: len }, material);
        self.settle_mass(id, n, 1.0, Some(length * density));
        top + len
    }

    pub(in crate::ork) fn mass_component(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "packedlength").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let role = match text(n, "masscomponenttype") {
            Some("altimeter") | Some("flightcomputer") => MassRole::Altimeter,
            Some("battery") => MassRole::Battery,
            Some("payload") => MassRole::Payload,
            Some("deploymentcharge") | Some("recoveryhardware") | Some("masscomponent") | None => MassRole::Generic,
            Some(_) => MassRole::Generic,
        };
        let kind =
            InternalKind::MassComponent { mass: num(n, "mass").unwrap_or(0.0).max(0.0), length: len, diameter: num(n, "packedradius").map(|r| 2.0 * r), role };
        let id = self.push_internal(n, p, top, kind.clone(), kind.default_material());
        // A mass component's own `mass` is its weight; an override on it replaces that.
        self.settle_mass(id, n, 1.0, None);
        top + len
    }
}
