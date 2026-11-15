//! Snap targets for direct edits (design kit, Snapping): the values that matter for a dimension, in priority order.
//!
//! 1 other parts (faces, neighbouring diameters, sibling angles); 2 matching (same as a sibling); 3 stock (tube lengths and
//! sizes, motor diameters, sheet thicknesses); 4 calibres. Priority 5 (round numbers on the coarse step) is the caller's
//! grid, not a list.
//!
//! Sizes follow common hobby stock: Estes/Apogee BT series outer diameters, 38 mm-class motor tubes (Cesaroni/AeroTech
//! designations 24, 29, 38, 54, 75, 98 mm), and 1.6/3.2/4.8 mm (1/16, 1/8, 3/16 in) fin sheet.

use crate::geometry::{NoseShape, PartKind, RadialKind};
use crate::handles::Dim;
use crate::internal::{InternalKind, PartRef};
use crate::{PartId, Rocket};

/// A value a drag can catch.
#[derive(Clone, Debug, PartialEq)]
pub struct Target {
    /// The dimension value, SI (metres, radians; for a shape parameter the parameter).
    pub value: f64,
    /// What it is, shown on the tick (`Ø98.0 · booster tube`, `STOCK 24 in`).
    pub label: String,
    /// The part it matches, if it is another part (the dotted guide runs to it).
    pub source: Option<PartId>,
    /// 1 (other parts) to 4 (calibres); lower wins ties.
    pub priority: u8,
}

/// Motor diameters, mm.
pub const MOTOR_DIAMETERS_MM: [f64; 6] = [24.0, 29.0, 38.0, 54.0, 75.0, 98.0];
/// Fin sheet thicknesses, mm.
pub const FIN_STOCK_MM: [f64; 3] = [1.6, 3.2, 4.8];
/// Stock tube lengths, inches.
pub const TUBE_LENGTHS_IN: [f64; 6] = [6.0, 12.0, 18.0, 24.0, 36.0, 48.0];
/// Stock airframe tube outer diameters: name and metres.
pub const TUBE_SIZES: [(&str, f64); 8] =
    [("BT-20", 0.0187), ("BT-50", 0.0248), ("BT-55", 0.0330), ("BT-60", 0.0416), ("BT-70", 0.0561), ("BT-80", 0.0660), ("3 in", 0.0762), ("4 in", 0.1016)];

fn mm(v: f64) -> String {
    format!("{:.1}", v * 1000.0)
}

fn deg(v: f64) -> String {
    format!("{:.0}°", v.to_degrees())
}

struct Out {
    list: Vec<Target>,
}

impl Out {
    fn add(&mut self, value: f64, label: impl Into<String>, source: Option<PartId>, priority: u8) {
        if !value.is_finite() {
            return;
        }
        if let Some(t) = self.list.iter_mut().find(|t| (t.value - value).abs() < 1e-9) {
            if priority < t.priority {
                *t = Target { value, label: label.into(), source, priority };
            }
            return;
        }
        self.list.push(Target { value, label: label.into(), source, priority });
    }
}

impl Rocket {
    /// The targets a drag of `dim` of part `id` can catch, nearest-priority first (then by value). Empty if the part has no
    /// such dimension. Computed once at drag start.
    pub fn snap_targets(&self, id: PartId, dim: Dim) -> Vec<Target> {
        let mut out = Out { list: Vec::new() };
        let Some(part) = self.kind_of(id) else {
            return Vec::new();
        };
        let name_of = |p: PartId| self.kind_of(p).map(|p| p.name()).unwrap_or_default().to_lowercase();
        let reference = self.reference_diameter().unwrap_or(0.0);
        match (dim, part) {
            (Dim::Length | Dim::ForeLength, PartRef::Axial(p)) => {
                // Dragging the fore end moves every face ahead of it too, so only the aft end has faces to catch.
                if dim == Dim::Length {
                    let z0 = self.part_station(id).unwrap_or(0.0);
                    self.faces(id, false, |station, label, src| out.add(station - z0, label, src, 1));
                }
                for other in self.parts().iter().filter(|o| o.id != id && std::mem::discriminant(&o.kind) == std::mem::discriminant(&p.kind)) {
                    out.add(other.kind.length(), format!("= {} length", other.display_name().to_lowercase()), Some(other.id), 2);
                }
                if matches!(p.kind, PartKind::BodyTube { .. }) {
                    for n in TUBE_LENGTHS_IN {
                        out.add(n * 0.0254, format!("STOCK {n:.0} in · {:.1}", n * 25.4), None, 3);
                    }
                }
                if matches!(p.kind, PartKind::NoseCone { .. }) {
                    for n in [3.0, 4.0, 5.0] {
                        out.add(n * p.kind.aft_diameter(), format!("{n:.0} CAL"), None, 4);
                    }
                }
            }
            (Dim::Length | Dim::ForeLength | Dim::PackedLength, PartRef::Internal(_) | PartRef::Radial(_)) => {
                let fore = self.part_station(id).unwrap_or(0.0);
                let aft = fore + self.part_span(id).map_or(0.0, |(a, b)| b - a);
                self.faces(id, true, |station, label, src| {
                    let length = if dim == Dim::ForeLength { aft - station } else { station - fore };
                    if length > 0.0 {
                        out.add(length, label, src, 1);
                    }
                });
                if let PartRef::Internal(p) = part {
                    for o in self.internals_of(p.parent).filter(|o| o.id != id && std::mem::discriminant(&o.kind) == std::mem::discriminant(&p.kind)) {
                        out.add(o.kind.extent(), format!("= {} length", o.display_name().to_lowercase()), Some(o.id), 2);
                    }
                    if matches!(p.kind, InternalKind::InnerTube { .. } | InternalKind::Coupler { .. }) {
                        for n in TUBE_LENGTHS_IN {
                            out.add(n * 0.0254, format!("STOCK {n:.0} in · {:.1}", n * 25.4), None, 3);
                        }
                    }
                }
            }
            (Dim::Position, PartRef::Radial(_) | PartRef::Internal(_)) => {
                let parent = self.parent_of(id).unwrap_or(0);
                let base = self.part_station(parent).unwrap_or(0.0);
                let extent = self.part_span(id).map_or(0.0, |(a, b)| b - a);
                self.faces(id, true, |station, label, src| {
                    out.add(station - base, label.clone(), src, 1);
                    out.add(station - base - extent, label, src, 1);
                });
            }
            (Dim::Diameter | Dim::ForeDiameter, PartRef::Axial(_)) => {
                let i = self.index_of(id).unwrap_or(0);
                let before = i.checked_sub(1).and_then(|j| self.parts().get(j));
                let after = self.parts().get(i + 1);
                for (n, d) in [before.map(|b| (b, b.kind.aft_diameter())), after.map(|a| (a, a.kind.fore_diameter()))].into_iter().flatten() {
                    if d > 0.0 {
                        out.add(d, format!("Ø{} · {}", mm(d), n.display_name().to_lowercase()), Some(n.id), 1);
                    }
                }
                for o in self.parts().iter().filter(|o| o.id != id) {
                    for d in [o.kind.fore_diameter(), o.kind.aft_diameter()] {
                        if d > 0.0 {
                            out.add(d, format!("Ø{} · {}", mm(d), o.display_name().to_lowercase()), Some(o.id), 2);
                        }
                    }
                }
                for (n, d) in TUBE_SIZES {
                    out.add(d, format!("Ø{} · {n}", mm(d)), None, 3);
                }
            }
            (Dim::Diameter, PartRef::Internal(p)) => {
                let mid = p.offset + p.kind.extent() / 2.0;
                if let Some(bore) = self.inner_diameter_at(p.parent, mid) {
                    out.add(bore, format!("Ø{} · {} bore", mm(bore), name_of(p.parent)), Some(p.parent), 1);
                }
                for o in self.internals_of(p.parent).filter(|o| o.id != id) {
                    if let Some(r) = self.resolved(o.id) {
                        if matches!(o.kind, InternalKind::InnerTube { .. } | InternalKind::Coupler { .. } | InternalKind::Bulkhead { .. }) {
                            out.add(r.outer_diameter, format!("Ø{} · {}", mm(r.outer_diameter), name_of(o.id)), Some(o.id), 2);
                        }
                    }
                }
                if let InternalKind::InnerTube { thickness, motor_mount: true, .. } = p.kind {
                    for d in MOTOR_DIAMETERS_MM {
                        out.add(d / 1000.0 + 2.0 * thickness, format!("{d:.0} mm MOTOR"), None, 3);
                    }
                }
                for (n, d) in TUBE_SIZES {
                    out.add(d, format!("Ø{} · {n}", mm(d)), None, 3);
                }
            }
            (Dim::InnerDiameter, PartRef::Internal(p)) => {
                for o in self.internals_of(p.parent).filter(|o| matches!(o.kind, InternalKind::InnerTube { .. })) {
                    if let InternalKind::InnerTube { outer_diameter, .. } = o.kind {
                        out.add(outer_diameter, format!("Ø{} · {}", mm(outer_diameter), name_of(o.id)), Some(o.id), 1);
                    }
                }
            }
            (Dim::Thickness, PartRef::Radial(r)) => {
                for o in self.radials().iter().filter(|o| o.id != id) {
                    if let Some(t) = o.kind.thickness() {
                        out.add(t, format!("= {} thickness", name_of(o.id)), Some(o.id), 2);
                    }
                }
                if r.kind.is_fin() {
                    for t in FIN_STOCK_MM {
                        out.add(t / 1000.0, format!("STOCK {t:.1} mm"), None, 3);
                    }
                }
            }
            (Dim::Thickness, PartRef::Internal(p)) => {
                for o in self.internals_of(p.parent).filter(|o| o.id != id && std::mem::discriminant(&o.kind) == std::mem::discriminant(&p.kind)) {
                    out.add(o.kind.extent(), format!("= {} thickness", name_of(o.id)), Some(o.id), 2);
                }
                for t in [3.0, 6.0, 9.0, 12.0] {
                    out.add(t / 1000.0, format!("STOCK {t:.0} mm PLY"), None, 3);
                }
            }
            (Dim::Wall, _) => {
                for o in self.parts().iter().filter(|o| o.id != id && o.thickness > 0.0) {
                    out.add(o.thickness, format!("= {} wall", o.display_name().to_lowercase()), Some(o.id), 2);
                }
            }
            (Dim::Angle, PartRef::Radial(r)) => {
                for o in self.radials().iter().filter(|o| o.id != id && o.parent == r.parent) {
                    for a in o.angles() {
                        out.add(a, format!("{} · {}", deg(a), name_of(o.id)), Some(o.id), 1);
                    }
                }
                for k in 0..r.count.max(1) {
                    let a = std::f64::consts::TAU * f64::from(k) / f64::from(r.count.max(1));
                    out.add(a, format!("{} EVEN", deg(a)), None, 2);
                }
                out.add(0.0, "0° TOP", None, 2);
            }
            (Dim::Gap, PartRef::Radial(_)) => {
                out.add(0.0, "TOUCHING", None, 1);
            }
            (Dim::Cant, PartRef::Radial(_)) => {
                out.add(0.0, "0° CANT", None, 1);
            }
            (Dim::Span, PartRef::Radial(_)) => {
                for o in self.radials().iter().filter(|o| o.id != id) {
                    if let Some(s) = o.kind.fin_span() {
                        out.add(s, format!("= {} height", name_of(o.id)), Some(o.id), 2);
                    }
                }
                for n in [1.0, 1.5, 2.0] {
                    out.add(n * reference, format!("{n} CAL"), None, 4);
                }
            }
            (Dim::RootChord, PartRef::Radial(r)) => {
                if let Some(len) = self.part(r.parent).map(|p| p.kind.length()) {
                    out.add(len - r.offset, "tube aft".to_string(), Some(r.parent), 1);
                }
                for o in self.radials().iter().filter(|o| o.id != id && o.kind.is_fin()) {
                    out.add(o.kind.extent(), format!("= {} root", name_of(o.id)), Some(o.id), 2);
                }
            }
            (Dim::TipChord, PartRef::Radial(r)) => {
                out.add(0.0, "POINTED TIP", None, 1);
                if let RadialKind::FinSet { root_chord, .. } = r.kind {
                    out.add(root_chord, "= root chord", None, 2);
                }
            }
            (Dim::Sweep, PartRef::Radial(r)) => {
                out.add(0.0, "NO SWEEP", None, 1);
                if let RadialKind::FinSet { root_chord, tip_chord, .. } = r.kind {
                    out.add(root_chord - tip_chord, "STRAIGHT TRAILING EDGE", None, 2);
                }
            }
            (Dim::Shape, PartRef::Axial(p)) => match p.kind {
                PartKind::NoseCone { shape: NoseShape::Haack(_), .. } => {
                    out.add(0.0, "C 0 · LD-HAACK (VON KÁRMÁN)", None, 3);
                    out.add(1.0 / 3.0, "C 1/3 · LV-HAACK", None, 3);
                    out.add(2.0 / 3.0, "C 2/3 · TANGENT-LIKE", None, 3);
                }
                PartKind::NoseCone { shape: NoseShape::PowerSeries(_), .. } => {
                    out.add(0.5, "n 1/2 · PARABOLA", None, 3);
                    out.add(0.75, "n 3/4", None, 3);
                    out.add(1.0 / 3.0, "n 1/3", None, 3);
                }
                _ => {}
            },
            (Dim::ShoulderDiameter | Dim::ForeShoulderDiameter, PartRef::Axial(_)) => {
                // A shoulder slides into the tube beside it: its outer diameter catches that tube's inner diameter.
                let aft = dim == Dim::ShoulderDiameter;
                let i = self.index_of(id).unwrap_or(0);
                let neighbour = if aft { self.parts().get(i + 1) } else { i.checked_sub(1).and_then(|j| self.parts().get(j)) };
                if let Some(n) = neighbour {
                    let d = n.inner_diameter_at(if aft { 0.0 } else { n.kind.length() });
                    if d > 0.0 {
                        out.add(d, format!("Ø{} · {} inner", mm(d), n.display_name().to_lowercase()), Some(n.id), 1);
                    }
                }
            }
            (Dim::ShoulderLength | Dim::ForeShoulderLength, PartRef::Axial(_)) => {
                let aft = dim == Dim::ShoulderLength;
                let i = self.index_of(id).unwrap_or(0);
                let neighbour = if aft { self.parts().get(i + 1) } else { i.checked_sub(1).and_then(|j| self.parts().get(j)) };
                let own = self.shoulder_diameter(id, aft).unwrap_or(0.0);
                if own > 0.0 {
                    out.add(own, "1 CAL", None, 4);
                }
                if let Some(n) = neighbour {
                    out.add(n.kind.length(), format!("= {} length", n.display_name().to_lowercase()), Some(n.id), 2);
                }
            }
            (Dim::Overhang, PartRef::Internal(_)) => {
                out.add(0.0, "flush with mount aft", None, 1);
            }
            _ => {}
        }
        let mut list = out.list;
        list.sort_by(|a, b| a.priority.cmp(&b.priority).then(a.value.total_cmp(&b.value)));
        list
    }

    /// Every face of every other part that can be aligned to: `(station, label, part)`. With `inside_only` the axial parts'
    /// faces are left out unless they are the parent's (for a part that lives on or in one).
    fn faces(&self, id: PartId, inside_only: bool, mut f: impl FnMut(f64, String, Option<PartId>)) {
        let parent = self.parent_of(id);
        for other in self.ids().into_iter().filter(|&o| o != id) {
            let axial = self.part(other).is_some();
            if axial && !(inside_only && Some(other) == parent) {
                continue;
            }
            if !axial && inside_only && self.parent_of(other) != parent {
                continue;
            }
            let Some((a, b)) = self.part_span(other) else {
                continue;
            };
            let name = self.kind_of(other).map(|p| p.name()).unwrap_or_default().to_lowercase();
            f(a, format!("flush with {name} fore"), Some(other));
            f(b, format!("flush with {name} aft"), Some(other));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::NoseShape;

    fn rocket() -> (Rocket, PartId, PartId, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::Haack(1.0 / 3.0), length: 0.3, diameter: 0.1 }).unwrap();
        let booster = r.insert(1, PartKind::BodyTube { length: 0.4, diameter: 0.0762 }).unwrap();
        let payload = r.insert(1, PartKind::BodyTube { length: 0.3, diameter: 0.0762 }).unwrap();
        (r, booster, payload, 0)
    }

    #[test]
    fn diameter_targets_prefer_neighbours_then_stock() {
        let (r, booster, payload, _) = rocket();
        let t = r.snap_targets(booster, Dim::Diameter);
        assert_eq!(t[0].priority, 1);
        assert!(t.iter().any(|t| t.source == Some(payload) && t.priority == 1 && t.label.contains("body tube")));
        assert!(t.iter().any(|t| t.label.contains("BT-60") && t.priority == 3));
        // Sorted by priority.
        assert!(t.windows(2).all(|w| w[0].priority <= w[1].priority));
    }

    #[test]
    fn length_targets_include_stock_and_flush_faces() {
        let (mut r, booster, _, _) = rocket();
        r.attach_internal(booster, 0.1, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 }).unwrap();
        let t = r.snap_targets(booster, Dim::Length);
        // The ring's aft face is 0.106 into the tube.
        assert!(t.iter().any(|t| (t.value - 0.106).abs() < 1e-9 && t.priority == 1));
        assert!(t.iter().any(|t| (t.value - 12.0 * 0.0254).abs() < 1e-9 && t.priority == 3));
        let nose = r.parts()[0].id;
        let t = r.snap_targets(nose, Dim::Length);
        assert!(t.iter().any(|t| (t.value - 0.3).abs() < 1e-9 && t.label == "3 CAL" && t.priority == 4));
    }

    #[test]
    fn fin_targets() {
        let (mut r, booster, _, _) = rocket();
        let fin =
            r.attach_radial(booster, 0.2, 0.0, 4, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.08, sweep: 0.04, thickness: 0.003 }).unwrap();
        let t = r.snap_targets(fin, Dim::Thickness);
        assert!(t.iter().any(|t| (t.value - 0.0032).abs() < 1e-12 && t.priority == 3));
        let t = r.snap_targets(fin, Dim::Angle);
        assert!(t.iter().any(|t| (t.value - std::f64::consts::FRAC_PI_2).abs() < 1e-9));
        assert_eq!(r.snap_targets(fin, Dim::Cant)[0].value, 0.0);
        let t = r.snap_targets(fin, Dim::Span);
        assert!(t.iter().any(|t| t.priority == 4 && (t.value - 0.15).abs() < 1e-9));
    }
}
