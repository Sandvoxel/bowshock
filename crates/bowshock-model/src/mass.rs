//! Mass, centre of gravity and moments of inertia of the parts and of the dry rocket (no motor).
//!
//! Method (standard solid-body mass, centre-of-gravity and inertia formulas; Niskanen 2009, section 4.2.3 "Mass and moment of inertia calculations"):
//! shells of revolution (body tubes, transitions, nose cones) are integrated as a stack of thin annular discs whose outer radius
//! follows the profile and whose wall is the part's thickness; tubes, rings, bulkheads and couplers are annular cylinders; fins
//! are planform area x thickness x section factor x density; a parachute is canopy area x surface density plus lines x line
//! density, packed as a cylinder. Mass and CG overrides replace the computed values (inertias scale with a mass override).
//!
//! Inertias are about the part's own CG: `i_long` about a transverse axis through the CG, `i_rot` about the long axis.

use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

use crate::geometry::{PartKind, RadialKind, RadialPart};
use crate::internal::{InternalKind, InternalPart};
use crate::material::MaterialId;
use crate::rocket::Rocket;
use crate::PartId;

/// Thin discs a curved shell is integrated in. The midpoint rule on a smooth profile has error of order 1/N^2.
const SLICES: usize = 256;

/// A launch lug's wall: a tenth of its diameter, at most a millimetre.
const LUG_WALL_MAX: f64 = 0.001;

/// Mass, centre of gravity and inertias of a part or a group of parts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MassProps {
    /// Mass, kg.
    pub mass: f64,
    /// Centre of gravity as a station: metres behind the rocket's nose tip.
    pub cg: f64,
    /// Moment of inertia about a transverse axis through the CG, kg m^2.
    pub i_long: f64,
    /// Moment of inertia about the long axis, kg m^2.
    pub i_rot: f64,
}

impl MassProps {
    /// Nothing: zero mass.
    pub const ZERO: MassProps = MassProps { mass: 0.0, cg: 0.0, i_long: 0.0, i_rot: 0.0 };

    /// A point mass at station `cg`.
    pub fn point(mass: f64, cg: f64) -> MassProps {
        MassProps { mass, cg, i_long: 0.0, i_rot: 0.0 }
    }

    /// The same body moved `dz` aft.
    pub fn shifted(self, dz: f64) -> MassProps {
        MassProps { cg: self.cg + dz, ..self }
    }

    /// The combined body: masses add, the CG is the mass-weighted mean, `i_rot` adds (the long axis passes through every
    /// CG) and `i_long` follows the parallel-axis theorem. Zero-mass items are ignored; no items give [`MassProps::ZERO`].
    pub fn combine(items: impl IntoIterator<Item = MassProps>) -> MassProps {
        let items: Vec<MassProps> = items.into_iter().filter(|p| p.mass > 0.0).collect();
        let mass: f64 = items.iter().map(|p| p.mass).sum();
        if mass <= 0.0 {
            return MassProps::ZERO;
        }
        let cg = items.iter().map(|p| p.mass * p.cg).sum::<f64>() / mass;
        MassProps { mass, cg, i_long: items.iter().map(|p| p.i_long + p.mass * (p.cg - cg).powi(2)).sum(), i_rot: items.iter().map(|p| p.i_rot).sum() }
    }
}

/// A hollow cylinder of density `rho`, radii `ro` and `ri`, length `len`, CG at its middle (relative to its fore face).
fn annulus(rho: f64, ro: f64, ri: f64, len: f64) -> MassProps {
    let (ro, ri) = (ro.max(0.0), ri.clamp(0.0, ro.max(0.0)));
    let mass = rho * PI * (ro * ro - ri * ri) * len.max(0.0);
    cylinder_lump(mass, len, ro, ri)
}

/// A uniform body of `mass` shaped as a hollow cylinder `len` long.
fn cylinder_lump(mass: f64, len: f64, ro: f64, ri: f64) -> MassProps {
    let sq = ro * ro + ri * ri;
    MassProps { mass, cg: len / 2.0, i_long: mass / 12.0 * (3.0 * sq + len * len), i_rot: mass / 2.0 * sq }
}

/// The shell of axial part `kind` with wall `wall`, integrated in thin discs; CG relative to the fore face.
fn shell(kind: &PartKind, wall: f64, rho: f64) -> MassProps {
    let len = kind.length().max(0.0);
    let n = if matches!(kind, PartKind::BodyTube { .. }) { 1 } else { SLICES };
    let dz = len / n as f64;
    MassProps::combine((0..n).map(|i| {
        let z = (i as f64 + 0.5) * dz;
        let ro = kind.radius_at(z);
        annulus(rho, ro, (ro - wall).max(0.0), dz).shifted(z - dz / 2.0)
    }))
}

/// A mass override scales the inertias with it; a CG override replaces the CG (given from the part's fore edge). The result
/// is moved to `station`, the part's fore edge in the rocket.
fn finish(mut p: MassProps, mass_override: Option<f64>, cg_override: Option<f64>, station: f64) -> MassProps {
    if let Some(m) = mass_override {
        let k = if p.mass > 0.0 { m / p.mass } else { 0.0 };
        (p.mass, p.i_long, p.i_rot) = (m, p.i_long * k, p.i_rot * k);
    }
    if let Some(c) = cg_override {
        p.cg = c;
    }
    p.shifted(station)
}

impl Rocket {
    /// Mass, CG station and inertias of one part, including every copy of a radial group, with any overrides applied. `None`
    /// if `id` is not in the rocket. Motors are not part of the model.
    pub fn component_mass(&self, id: PartId) -> Option<MassProps> {
        if let Some(i) = self.pod_index(id) {
            // A part of a pod: every copy, each standing `axis_radius` off the core axis. Each copy's own inertias come from the
            // pod's rocket; moving it off the axis adds m r^2 about the long axis and, averaged over the copies' directions,
            // m r^2 / 2 about a transverse axis (the parallel-axis theorem).
            let pod = &self.radials[i];
            let (z0, sub) = self.scratch_of(id)?;
            let one = sub.component_mass(id)?;
            let (n, r) = (f64::from(pod.count.max(1)), self.pod_frame(pod)?.axis_radius);
            return Some(MassProps {
                mass: n * one.mass,
                cg: z0 + one.cg,
                i_long: n * (one.i_long + one.mass * r * r / 2.0),
                i_rot: n * (one.i_rot + one.mass * r * r),
            });
        }
        if let Some(body) = self.radial(id).and_then(|r| r.kind.pod_body()) {
            return Some(MassProps::combine(body.ids().filter_map(|child| self.component_mass(child))));
        }
        if let Some(index) = self.index_of(id) {
            let p = &self.parts[index];
            let rho = p.material.density_as(crate::material::MaterialKind::Bulk, MaterialId::CARDBOARD);
            let shoulders = [false, true].into_iter().filter_map(|aft| self.shoulder_props(p.id, aft, rho));
            let body = MassProps::combine(std::iter::once(shell(&p.kind, p.thickness, rho)).chain(shoulders));
            return Some(finish(body, p.mass_override, p.cg_override, self.stations()[index]));
        }
        if let Some(r) = self.radial(id) {
            return Some(finish(self.radial_props(r), r.mass_override, r.cg_override, self.radial_station(r)?));
        }
        let p = self.internal(id)?;
        let station = self.part_station(id)?;
        Some(finish(self.internal_props(p)?, p.mass_override, p.cg_override, station))
    }

    /// A shoulder as a tube of the part's material, CG relative to the part's fore face (a fore shoulder lies ahead of it, an
    /// aft shoulder behind the aft face). Solid when its wall reaches the axis. `None` without a shoulder.
    fn shoulder_props(&self, id: PartId, aft: bool, rho: f64) -> Option<MassProps> {
        let s = self.shoulder(id, aft).filter(|s| s.is_some())?;
        let ro = self.shoulder_diameter(id, aft)? / 2.0;
        let body = annulus(rho, ro, (ro - s.thickness).max(0.0), s.length);
        let z0 = if aft { self.part(id)?.kind.length() } else { -s.length };
        Some(body.shifted(z0))
    }

    /// Every part's [`component_mass`](Self::component_mass), in [`ids`](Self::ids) order.
    pub fn component_masses(&self) -> Vec<(PartId, MassProps)> {
        // A pod set is its parts: counting both would weigh it twice.
        self.ids().into_iter().filter(|&id| !self.is_pod(id)).filter_map(|id| self.component_mass(id).map(|m| (id, m))).collect()
    }

    /// The whole rocket without a motor: every part combined (see [`MassProps::combine`]) with the active configuration's
    /// ballast, a point mass at its station. Station of the CG is from the nose tip.
    pub fn dry_mass(&self) -> MassProps {
        let ballast = self.active_configuration().ballast.map(|b| MassProps::point(b.mass, b.station));
        MassProps::combine(self.component_masses().into_iter().map(|(_, m)| m).chain(ballast))
    }

    /// A radial group (all copies), CG relative to the group's fore edge. Copies that have unlinked a field (see
    /// [`crate::symmetry`]) weigh by their own values.
    fn radial_props(&self, r: &RadialPart) -> MassProps {
        if !r.has_overrides() {
            return self.radial_props_of(r, &r.kind, r.count.max(1));
        }
        MassProps::combine((0..usize::from(r.count.max(1))).map(|i| self.radial_props_of(r, &r.effective(i).kind, 1)))
    }

    /// `copies` identical copies of `kind` standing as radial part `r` stands (its position, material and section).
    fn radial_props_of(&self, r: &RadialPart, kind: &RadialKind, copies: u8) -> MassProps {
        let n = f64::from(copies);
        let base = self.surface_radius(r.parent, r.offset).zip(self.surface_radius(r.parent, r.offset + kind.extent())).map_or(0.0, |(a, b)| a.min(b));
        let bulk = |m: MaterialId| m.density_as(crate::material::MaterialKind::Bulk, MaterialId::BIRCH_PLYWOOD);
        match kind {
            RadialKind::FinSet { .. } | RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => {
                let (area, thickness) = (kind.fin_area().unwrap_or(0.0), kind.thickness().unwrap_or(0.0));
                let (cz, ch) = kind.fin_centroid().unwrap_or((0.0, 0.0));
                let mass = bulk(r.material) * area * thickness * r.cross_section.volume_factor();
                let radius = base + ch;
                let chord = kind.extent();
                // Each plate is a point at the centroid for the long axis' spin; across the axis it averages r^2/2 about the
                // diameter it lies on plus its own chord-wise spread, treated as a thin rod.
                let fin =
                    MassProps { mass: n * mass, cg: cz, i_long: n * mass * (radius * radius / 2.0 + chord * chord / 12.0), i_rot: n * mass * radius * radius };
                // The fillet: two concave strips of cross-section r^2 (1 - pi/4) along the root, each a
                // point-like mass at the root with the fin's centroid station and the surface radius.
                if r.fillet_radius > 0.0 {
                    let m = n * 2.0 * r.fillet_radius * r.fillet_radius * (1.0 - PI / 4.0) * chord * bulk(r.fillet_material);
                    let strip = MassProps { mass: m, cg: chord / 2.0, i_long: m * (base * base / 2.0 + chord * chord / 12.0), i_rot: m * base * base };
                    return MassProps::combine([fin, strip]);
                }
                fin
            }
            RadialKind::LaunchLug { length, diameter } => {
                let (ro, wall) = (diameter / 2.0, (diameter / 10.0).min(LUG_WALL_MAX));
                let one = annulus(bulk(r.material), ro, ro - wall, *length);
                let radius = base + ro;
                MassProps {
                    mass: n * one.mass,
                    cg: one.cg,
                    i_long: n * (one.i_long + one.mass * radius * radius / 2.0),
                    i_rot: n * (one.i_rot + one.mass * radius * radius),
                }
            }
            RadialKind::RailButton { height, diameter } => {
                // A stem 0.6 of the head's diameter over 0.7 of the height, and the head.
                let volume = PI / 4.0 * (0.36 * diameter * diameter * 0.7 * height + diameter * diameter * 0.3 * height);
                let mass = bulk(r.material) * volume;
                let radius = base + height / 2.0;
                MassProps { mass: n * mass, cg: diameter / 2.0, i_long: n * mass * radius * radius / 2.0, i_rot: n * mass * radius * radius }
            }
            // A pod weighs its parts ([`Rocket::component_mass`]); the set itself has no plate or tube of its own.
            RadialKind::Pod { .. } => MassProps::ZERO,
        }
    }

    /// One internal part, CG relative to its fore edge.
    fn internal_props(&self, p: &InternalPart) -> Option<MassProps> {
        use crate::material::MaterialKind::{Bulk, Line, Surface};
        let size = self.resolved(p.id)?;
        let (ro, ri) = (size.outer_diameter / 2.0, size.inner_diameter / 2.0);
        let bulk = p.material.density_as(Bulk, p.kind.default_material());
        Some(match &p.kind {
            InternalKind::InnerTube { length, .. } | InternalKind::Coupler { length, .. } => annulus(bulk, ro, ri, *length),
            InternalKind::CenteringRing { thickness, .. } | InternalKind::Bulkhead { thickness, .. } | InternalKind::EngineBlock { thickness, .. } => {
                annulus(bulk, ro, ri, *thickness)
            }
            InternalKind::Parachute { diameter, line_count, line_length, line_material, packed_length, .. } => {
                let canopy = PI / 4.0 * diameter * diameter * p.material.density_as(Surface, MaterialId::RIPSTOP_NYLON);
                let lines = f64::from(*line_count) * line_length * line_material.density_as(Line, MaterialId::BRAIDED_NYLON_2MM);
                cylinder_lump(canopy + lines, *packed_length, ro, 0.0)
            }
            InternalKind::Streamer { length, width, .. } => cylinder_lump(length * width * p.material.density_as(Surface, MaterialId::MYLAR), *width, ro, 0.0),
            InternalKind::ShockCord { length, packed_length } => {
                cylinder_lump(length * p.material.density_as(Line, MaterialId::BRAIDED_NYLON_3MM), *packed_length, ro, 0.0)
            }
            InternalKind::MassComponent { mass, length, .. } => cylinder_lump(*mass, *length, ro, 0.0),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::NoseShape;

    fn close(a: f64, b: f64, rel: f64) -> bool {
        (a - b).abs() <= rel * b.abs().max(1e-12)
    }

    /// A 76.2 mm x 600 mm cardboard tube with a 1.6 mm wall, as the first part.
    fn tube_rocket() -> (Rocket, PartId) {
        let mut r = Rocket::default();
        let id = r.insert(0, PartKind::BodyTube { length: 0.6, diameter: 0.0762 }).unwrap();
        (r, id)
    }

    #[test]
    fn cardboard_tube_mass_matches_hand_calculation() {
        let (r, id) = tube_rocket();
        let m = r.component_mass(id).unwrap();
        // rho * pi/4 * (D^2 - d^2) * L with d = D - 2 t.
        let (d_out, d_in) = (0.0762_f64, 0.0762 - 2.0 * 0.0016);
        let hand = 680.0 * PI / 4.0 * (d_out * d_out - d_in * d_in) * 0.6;
        assert!(close(m.mass, hand, 0.01), "{} vs {hand}", m.mass);
        // About 0.153 kg: a 3 in cardboard airframe tube weighs roughly a quarter kilogram per metre.
        assert!((0.14..0.17).contains(&m.mass));
        assert!(close(m.cg, 0.3, 1e-9));
        // Thin-wall cylinder: i_rot is about m r^2, i_long m (3 r^2 + L^2) / 12.
        let r2 = (d_out / 2.0).powi(2) / 2.0 + (d_in / 2.0).powi(2) / 2.0;
        assert!(close(m.i_rot, m.mass * r2, 1e-9));
        assert!(close(m.i_long, m.mass / 12.0 * (3.0 * 2.0 * r2 + 0.36), 1e-9));
    }

    #[test]
    fn cg_is_in_station_coordinates() {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.3, diameter: 0.0762 }).unwrap();
        let t = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: 0.0762 }).unwrap();
        assert!(close(r.component_mass(t).unwrap().cg, 0.6, 1e-9));
        assert!(r.component_mass(99).is_none());
    }

    #[test]
    fn overrides_win() {
        let (mut r, id) = tube_rocket();
        let natural = r.component_mass(id).unwrap();
        assert!(r.set_mass_override(id, Some(0.5)));
        let m = r.component_mass(id).unwrap();
        assert_eq!(m.mass, 0.5);
        assert!(close(m.i_long, natural.i_long * 0.5 / natural.mass, 1e-9));
        assert!(r.set_cg_override(id, Some(0.1)));
        assert!(close(r.component_mass(id).unwrap().cg, 0.1, 1e-12));
        assert!(r.set_mass_override(id, None) && r.set_cg_override(id, None));
        assert_eq!(r.component_mass(id), Some(natural));
        assert!(!r.set_mass_override(99, Some(1.0)));
    }

    #[test]
    fn solid_cone_matches_closed_form() {
        let mut r = Rocket::default();
        let n = r.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.3, diameter: 0.1 }).unwrap();
        assert!(r.set_thickness(n, 1.0)); // limited to the radius: a solid cone
        assert!(close(r.part(n).unwrap().thickness, 0.05, 1e-12));
        let rho = r.part(n).unwrap().material.density();
        let m = r.component_mass(n).unwrap();
        assert!(close(m.mass, rho * PI * 0.05 * 0.05 * 0.3 / 3.0, 1e-3), "{}", m.mass);
        assert!(close(m.cg, 0.3 * 0.75, 1e-3));
        // Solid cone about its axis: 3/10 m R^2.
        assert!(close(m.i_rot, 0.3 * m.mass * 0.05 * 0.05, 1e-3));
    }

    #[test]
    fn hollow_ogive_is_lighter_than_solid_and_heavier_for_thicker_walls() {
        let mut r = Rocket::default();
        let n = r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.0762 }).unwrap();
        let thin = r.component_mass(n).unwrap().mass;
        r.set_thickness(n, 0.004);
        let thick = r.component_mass(n).unwrap().mass;
        r.set_thickness(n, 1.0);
        let solid = r.component_mass(n).unwrap().mass;
        assert!(0.0 < thin && thin < thick && thick < solid);
        // A tangent ogive is fuller than a cone: more than a third of the circumscribing cylinder.
        assert!(solid / (r.part(n).unwrap().material.density() * PI * 0.0381 * 0.0381 * 0.2) > 1.0 / 3.0);
    }

    #[test]
    fn transition_shell_matches_frustum_difference() {
        let mut r = Rocket::default();
        let t = r.insert(0, PartKind::Transition { length: 0.1, fore_diameter: 0.0762, aft_diameter: 0.0508 }).unwrap();
        let wall = r.part(t).unwrap().thickness;
        let frustum = |d1: f64, d2: f64| PI * 0.1 / 12.0 * (d1 * d1 + d1 * d2 + d2 * d2);
        let hand = 680.0 * (frustum(0.0762, 0.0508) - frustum(0.0762 - 2.0 * wall, 0.0508 - 2.0 * wall));
        assert!(close(r.component_mass(t).unwrap().mass, hand, 1e-3));
    }

    #[test]
    fn fin_mass_uses_planform_area() {
        let (mut r, tube) = tube_rocket();
        let fin = RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: 0.06, thickness: 0.003 };
        let id = r.attach_radial(tube, 0.4, 0.0, 3, fin.clone()).unwrap();
        let rho = r.radial(id).unwrap().material.density();
        let area = (0.12 + 0.05) / 2.0 * 0.07;
        let m = r.component_mass(id).unwrap();
        assert!(close(m.mass, 3.0 * rho * area * 0.003, 1e-9));
        // Trapezoid centroid along the chord: between the root's middle (0.06) and the tip's (0.085), about 0.0708.
        let (cz, _) = fin.fin_centroid().unwrap();
        assert!(close(m.cg, 0.4 + cz, 1e-9));
        assert!(0.06 < cz && cz < 0.085);
        // Inertia of three plates about the axis: n m r^2 at the centroid radius.
        assert!(m.i_rot > 0.0 && m.i_long > 0.0);
        // Airfoil sections hold less material; ellipse area is pi/4 c s.
        r.set_fin_section(id, crate::geometry::FinSection::Airfoil);
        assert!(r.component_mass(id).unwrap().mass < m.mass);
        let ell = RadialKind::EllipticalFin { root_chord: 0.12, span: 0.07, thickness: 0.003 };
        assert!(close(ell.fin_area().unwrap(), PI / 4.0 * 0.12 * 0.07, 1e-12));
        assert!(close(ell.fin_centroid().unwrap().0, 0.06, 1e-12));
        // The sampled outline agrees with the exact area to within the polygon's error.
        let poly = crate::geometry::polygon_area_centroid(&ell.planform().unwrap());
        assert!(close(poly.0, ell.fin_area().unwrap(), 0.01));
        assert!(close(poly.2, ell.fin_centroid().unwrap().1, 0.01));
    }

    #[test]
    fn internal_parts_mass_and_auto_sizes() {
        let (mut r, tube) = tube_rocket();
        let inner = r
            .attach_internal(tube, 0.4, InternalKind::InnerTube { outer_diameter: 0.054, thickness: 0.0016, length: 0.2, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let ring = r.attach_internal(tube, 0.45, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 }).unwrap();
        let res = r.resolved(ring).unwrap();
        // The tube wall is 1.6 mm: bore 72.8 mm. The ring sits on the inner tube, so its hole is 54 mm.
        assert!(close(res.outer_diameter, 0.0762 - 0.0032, 1e-9) && res.outer_auto);
        assert!(close(res.inner_diameter, 0.054, 1e-9) && res.inner_auto);
        let hand = 630.0 * PI / 4.0 * (res.outer_diameter.powi(2) - 0.054_f64.powi(2)) * 0.006;
        assert!(close(r.component_mass(ring).unwrap().mass, hand, 1e-9));
        assert!(close(r.component_mass(ring).unwrap().cg, 0.45 + 0.003, 1e-9));
        // Off the inner tube it is a solid disc.
        r.move_internal(ring, tube, 0.1).unwrap();
        assert_eq!(r.resolved(ring).unwrap().inner_diameter, 0.0);
        // The inner tube is an annulus.
        let tube_mass = 680.0 * PI / 4.0 * (0.054_f64.powi(2) - 0.0508_f64.powi(2)) * 0.2;
        assert!(close(r.component_mass(inner).unwrap().mass, tube_mass, 1e-9));
        // The bulkhead is a solid disc of the bore, and an explicit diameter is not auto.
        let bh = r.attach_internal(tube, 0.0, InternalKind::Bulkhead { diameter: Some(0.05), thickness: 0.006 }).unwrap();
        assert!(!r.resolved(bh).unwrap().outer_auto);
        assert!(close(r.component_mass(bh).unwrap().mass, 630.0 * PI / 4.0 * 0.0025 * 0.006, 1e-9));
        // Auto sizes follow the parent's wall.
        let thin = r.resolved(ring).unwrap().outer_diameter;
        r.set_thickness(tube, 0.004);
        assert!(r.resolved(ring).unwrap().outer_diameter < thin);
    }

    #[test]
    fn parachute_is_canopy_plus_lines() {
        let (mut r, tube) = tube_rocket();
        let chute = InternalKind::Parachute {
            diameter: 0.457,
            cd: 0.8,
            line_count: 6,
            line_length: 0.457,
            line_material: MaterialId::BRAIDED_NYLON_2MM,
            packed_length: 0.1,
            packed_diameter: None,
            deploy: crate::internal::Deployment::default(),
        };
        let id = r.attach_internal(tube, 0.05, chute).unwrap();
        let hand = PI / 4.0 * 0.457 * 0.457 * 0.067 + 6.0 * 0.457 * 0.001;
        let m = r.component_mass(id).unwrap();
        assert!(close(m.mass, hand, 1e-9));
        assert!(close(m.cg, 0.05 + 0.05, 1e-9));
        // A canopy made of a bulk material falls back to the default fabric rather than weighing tonnes.
        r.internals.iter_mut().find(|p| p.id == id).unwrap().material = MaterialId::ALUMINUM;
        assert!(close(r.component_mass(id).unwrap().mass, hand, 1e-9));
        // Streamer and shock cord.
        let s = r.attach_internal(tube, 0.2, InternalKind::Streamer { length: 1.0, width: 0.05, cd: 0.4, deploy: Default::default() }).unwrap();
        assert!(close(r.component_mass(s).unwrap().mass, 1.0 * 0.05 * 0.021, 1e-9));
        let c = r.attach_internal(tube, 0.3, InternalKind::ShockCord { length: 2.0, packed_length: 0.05 }).unwrap();
        assert!(close(r.component_mass(c).unwrap().mass, 2.0 * 0.0035, 1e-9));
    }

    #[test]
    fn combine_uses_parallel_axis() {
        let a = MassProps { mass: 1.0, cg: 0.0, i_long: 0.1, i_rot: 0.01 };
        let b = MassProps { mass: 1.0, cg: 1.0, i_long: 0.1, i_rot: 0.01 };
        let c = MassProps::combine([a, b, MassProps::ZERO]);
        assert_eq!((c.mass, c.cg, c.i_rot), (2.0, 0.5, 0.02));
        assert!(close(c.i_long, 0.2 + 2.0 * 0.25, 1e-12));
        assert_eq!(MassProps::combine([]), MassProps::ZERO);
        assert_eq!(MassProps::point(2.0, 3.0).shifted(1.0).cg, 4.0);
    }

    #[test]
    fn dry_mass_is_the_sum_of_parts() {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.0762 }).unwrap();
        let t = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: 0.0762 }).unwrap();
        r.attach_radial(t, 0.45, 0.0, 4, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: 0.06, thickness: 0.003 }).unwrap();
        r.attach_internal(t, 0.1, InternalKind::MassComponent { mass: 0.15, length: 0.08, diameter: None, role: crate::internal::MassRole::Altimeter })
            .unwrap();
        let parts = r.component_masses();
        assert_eq!(parts.len(), 4);
        let total: f64 = parts.iter().map(|(_, m)| m.mass).sum();
        let dry = r.dry_mass();
        assert!(close(dry.mass, total, 1e-12));
        let weighted = parts.iter().map(|(_, m)| m.mass * m.cg).sum::<f64>() / total;
        assert!(close(dry.cg, weighted, 1e-12));
        assert!((0.0..r.length()).contains(&dry.cg));
        assert!(Rocket::default().dry_mass() == MassProps::ZERO);
    }
}

#[cfg(test)]
mod shoulder_tests {
    use super::*;
    use crate::geometry::NoseShape;
    use crate::handles::Dim;

    fn close(a: f64, b: f64, rel: f64) -> bool {
        (a - b).abs() <= rel * b.abs().max(1e-12)
    }

    #[test]
    fn nose_shoulder_adds_a_tube_into_the_airframe() {
        let mut r = Rocket::default();
        let nose = r.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.2, diameter: 0.0762 }).unwrap();
        r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let before = r.component_mass(nose).unwrap();
        assert!(r.set_dim(nose, Dim::ShoulderLength, 0.05).is_some());
        // AUTO follows the tube's inner diameter: 76.2 mm less two 1.6 mm cardboard walls.
        let d_in = 0.0762 - 2.0 * 0.0016;
        assert!(close(r.dim(nose, Dim::ShoulderDiameter).unwrap(), d_in, 1e-12));
        let after = r.component_mass(nose).unwrap();
        // rho pi/4 (D^2 - (D - 2 t)^2) L, polystyrene 1050 kg/m^3, 2 mm wall, 50 mm long.
        let hand = 1050.0 * PI / 4.0 * (d_in * d_in - (d_in - 0.004).powi(2)) * 0.05;
        assert!(close(after.mass - before.mass, hand, 1e-9), "{} vs {hand}", after.mass - before.mass);
        // The shoulder's own CG is 25 mm behind the nose base (station 0.2).
        let added = (after.mass * after.cg - before.mass * before.cg) / (after.mass - before.mass);
        assert!(close(added, 0.225, 1e-9), "{added}");
        // Setting the diameter fixes it; resetting puts it back on AUTO.
        r.set_dim(nose, Dim::ShoulderDiameter, 0.06).unwrap();
        assert!(close(r.dim(nose, Dim::ShoulderDiameter).unwrap(), 0.06, 1e-12));
        assert!(r.reset_shoulder_diameter(nose, true));
        assert!(close(r.dim(nose, Dim::ShoulderDiameter).unwrap(), d_in, 1e-12));
        // A nose cone has no fore shoulder.
        assert!(r.dim(nose, Dim::ForeShoulderLength).is_none());
    }

    #[test]
    fn transition_has_both_shoulders_ahead_and_behind() {
        let mut r = Rocket::default();
        r.insert(0, PartKind::BodyTube { length: 0.3, diameter: 0.0762 }).unwrap();
        let t = r.insert(1, PartKind::Transition { length: 0.1, fore_diameter: 0.0762, aft_diameter: 0.054 }).unwrap();
        r.insert(2, PartKind::BodyTube { length: 0.3, diameter: 0.054 }).unwrap();
        let before = r.component_mass(t).unwrap();
        r.set_dim(t, Dim::ForeShoulderLength, 0.04).unwrap();
        r.set_dim(t, Dim::ShoulderLength, 0.03).unwrap();
        let after = r.component_mass(t).unwrap();
        assert!(after.mass > before.mass);
        let fore_in = 0.0762 - 2.0 * 0.0016;
        let aft_in = 0.054 - 2.0 * 0.0016;
        assert!(close(r.dim(t, Dim::ForeShoulderDiameter).unwrap(), fore_in, 1e-12));
        assert!(close(r.dim(t, Dim::ShoulderDiameter).unwrap(), aft_in, 1e-12));
        let rho = 680.0;
        let hand = rho * PI / 4.0 * ((fore_in.powi(2) - (fore_in - 0.004).powi(2)) * 0.04 + (aft_in.powi(2) - (aft_in - 0.004).powi(2)) * 0.03);
        assert!(close(after.mass - before.mass, hand, 1e-9));
        // Both shoulders are clamped to the tube beside them.
        assert_eq!(r.set_dim(t, Dim::ShoulderLength, 5.0).unwrap().applied, 0.3);
    }

    #[test]
    fn fin_fillet_mass_matches_hand_calculation() {
        let mut r = Rocket::default();
        let tube = r.insert(0, PartKind::BodyTube { length: 0.6, diameter: 0.0762 }).unwrap();
        let fin =
            r.attach_radial(tube, 0.45, 0.0, 4, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: 0.06, thickness: 0.003 }).unwrap();
        let before = r.component_mass(fin).unwrap();
        assert!(r.set_dim(fin, Dim::FilletRadius, 0.005).is_some());
        let after = r.component_mass(fin).unwrap();
        // Two strips per fin of cross-section r^2 (1 - pi/4) along the 120 mm root, epoxy 1100 kg/m^3, four fins.
        let hand = 4.0 * 2.0 * 0.005_f64.powi(2) * (1.0 - PI / 4.0) * 0.12 * 1100.0;
        assert!(close(after.mass - before.mass, hand, 1e-9), "{} vs {hand}", after.mass - before.mass);
        assert!(r.set_fillet_radius(fin, 0.0));
        assert_eq!(r.component_mass(fin).unwrap(), before);
        assert!(!r.set_fillet_radius(tube, 0.01));
    }

    #[test]
    fn old_documents_load_without_shoulders_or_fillets() {
        let mut r = Rocket::default();
        let nose = r.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.2, diameter: 0.0762 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let fin =
            r.attach_radial(tube, 0.3, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.06, sweep: 0.03, thickness: 0.003 }).unwrap();
        // A document saved without the new keys carries none of them.
        let json = r.to_json();
        assert!(!json.contains("shoulder") && !json.contains("fillet"));
        let back = Rocket::from_json(&json).unwrap();
        assert_eq!(back.shoulder(nose, true).unwrap().length, 0.0);
        assert_eq!(back.radial(fin).unwrap().fillet_radius, 0.0);
        // With them set they survive a round trip.
        r.set_dim(nose, Dim::ShoulderLength, 0.04).unwrap();
        r.set_dim(fin, Dim::FilletRadius, 0.004).unwrap();
        let back = Rocket::from_json(&r.to_json()).unwrap();
        assert_eq!(back.shoulder(nose, true).unwrap().length, 0.04);
        assert_eq!(back.radial(fin).unwrap().fillet_radius, 0.004);
        assert_eq!(back, r);
    }
}
