//! Mass, centre of gravity and moments of inertia of the parts and of the dry rocket (no motor).
//!
//! Method (after OpenRocket, Niskanen 2013, section 3.2 "Mass and centre of gravity", and the standard solid-body formulas):
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
        if let Some(index) = self.index_of(id) {
            let p = &self.parts[index];
            let rho = p.material.density_as(crate::material::MaterialKind::Bulk, MaterialId::CARDBOARD);
            return Some(finish(shell(&p.kind, p.thickness, rho), p.mass_override, p.cg_override, self.stations()[index]));
        }
        if let Some(r) = self.radial(id) {
            return Some(finish(self.radial_props(r), r.mass_override, r.cg_override, self.radial_station(r)?));
        }
        let p = self.internal(id)?;
        let station = self.part_station(id)?;
        Some(finish(self.internal_props(p)?, p.mass_override, p.cg_override, station))
    }

    /// Every part's [`component_mass`](Self::component_mass), in [`ids`](Self::ids) order.
    pub fn component_masses(&self) -> Vec<(PartId, MassProps)> {
        self.ids().into_iter().filter_map(|id| self.component_mass(id).map(|m| (id, m))).collect()
    }

    /// The whole rocket without a motor: every part combined (see [`MassProps::combine`]). Station of the CG is from the
    /// nose tip.
    pub fn dry_mass(&self) -> MassProps {
        MassProps::combine(self.component_masses().into_iter().map(|(_, m)| m))
    }

    /// A radial group (all copies), CG relative to the group's fore edge.
    fn radial_props(&self, r: &RadialPart) -> MassProps {
        let n = f64::from(r.count.max(1));
        let base = self.radial_base_radius(r).unwrap_or(0.0);
        let bulk = |m: MaterialId| m.density_as(crate::material::MaterialKind::Bulk, MaterialId::BIRCH_PLYWOOD);
        match &r.kind {
            RadialKind::FinSet { .. } | RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => {
                let (area, thickness) = (r.kind.fin_area().unwrap_or(0.0), r.kind.thickness().unwrap_or(0.0));
                let (cz, ch) = r.kind.fin_centroid().unwrap_or((0.0, 0.0));
                let mass = bulk(r.material) * area * thickness * r.cross_section.volume_factor();
                let radius = base + ch;
                let chord = r.kind.extent();
                // Each plate is a point at the centroid for the long axis' spin; across the axis it averages r^2/2 about the
                // diameter it lies on plus its own chord-wise spread, treated as a thin rod.
                MassProps { mass: n * mass, cg: cz, i_long: n * mass * (radius * radius / 2.0 + chord * chord / 12.0), i_rot: n * mass * radius * radius }
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
