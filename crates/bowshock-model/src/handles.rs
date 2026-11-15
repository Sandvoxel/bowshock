//! Direct-edit dimensions: every geometric dimension of every part type as a [`Dim`], read with [`Rocket::dim`], written
//! with [`Rocket::set_dim`] (limited, with the reason when it stops), and listed per part as [`HandleSpec`]s for the
//! viewport's handles (design kit, DirectEdit). All values SI: metres and radians.

use crate::geometry::{NoseShape, PartKind, RadialKind, Shoulder};
use crate::internal::{InternalKind, PartRef};
use crate::{PartId, Rocket};

/// One geometric dimension of a part.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Dim {
    /// Length along the axis (axial part, inner tube, coupler, mass component, launch lug).
    Length,
    /// The part's (aft) outer diameter: nose base, tube, transition aft, inner tube or coupler outer, lug diameter.
    Diameter,
    /// A transition's fore diameter.
    ForeDiameter,
    /// Shell wall thickness (axial parts, inner tubes, couplers).
    Wall,
    /// Nose shape parameter (Haack `C`, power-series `n`).
    Shape,
    /// Offset of the fore edge from the parent's fore face (radial and internal parts).
    Position,
    /// Fin span (the height above the surface).
    Span,
    /// Fin root chord.
    RootChord,
    /// Fin tip chord.
    TipChord,
    /// Fin sweep.
    Sweep,
    /// Plate, ring, bulkhead or engine-block thickness.
    Thickness,
    /// Fin cant angle.
    Cant,
    /// Angle of the first copy around the axis.
    Angle,
    /// Clear distance from the parent's surface to a pod.
    Gap,
    /// Rail button height.
    Height,
    /// Hole diameter of a ring or engine block.
    InnerDiameter,
    /// How far a motor mount's motor sticks out behind it.
    Overhang,
    /// Packed length of recovery gear.
    PackedLength,
    /// Length of the aft shoulder (nose cones and transitions).
    ShoulderLength,
    /// Outer diameter of the aft shoulder (follows the tube it enters until set).
    ShoulderDiameter,
    /// Wall thickness of the aft shoulder.
    ShoulderThickness,
    /// Length of a transition's fore shoulder.
    ForeShoulderLength,
    /// Outer diameter of a transition's fore shoulder.
    ForeShoulderDiameter,
    /// Wall thickness of a transition's fore shoulder.
    ForeShoulderThickness,
    /// Radius of the fillet along a fin's root.
    FilletRadius,
    /// A freeform fin's vertex (moved with [`Rocket::set_vertex`], not [`Rocket::set_dim`]).
    Vertex(usize),
}

impl Dim {
    /// The name shown on handle labels and tags (upper case).
    pub fn label(self) -> String {
        match self {
            Dim::Length => "LENGTH".into(),
            Dim::Diameter => "DIAMETER".into(),
            Dim::ForeDiameter => "FORE DIAMETER".into(),
            Dim::Wall => "WALL".into(),
            Dim::Shape => "SHAPE".into(),
            Dim::Position => "POSITION".into(),
            Dim::Span => "HEIGHT".into(),
            Dim::RootChord => "ROOT CHORD".into(),
            Dim::TipChord => "TIP CHORD".into(),
            Dim::Sweep => "SWEEP".into(),
            Dim::Thickness => "THICKNESS".into(),
            Dim::Cant => "CANT".into(),
            Dim::Angle => "ANGLE".into(),
            Dim::Gap => "GAP".into(),
            Dim::Height => "HEIGHT".into(),
            Dim::InnerDiameter => "INNER DIAMETER".into(),
            Dim::Overhang => "OVERHANG".into(),
            Dim::PackedLength => "PACKED LENGTH".into(),
            Dim::ShoulderLength => "SHOULDER LENGTH".into(),
            Dim::ShoulderDiameter => "SHOULDER DIAMETER".into(),
            Dim::ShoulderThickness => "SHOULDER THICKNESS".into(),
            Dim::ForeShoulderLength => "FORE SHOULDER LENGTH".into(),
            Dim::ForeShoulderDiameter => "FORE SHOULDER DIAMETER".into(),
            Dim::ForeShoulderThickness => "FORE SHOULDER THICKNESS".into(),
            Dim::FilletRadius => "FILLET RADIUS".into(),
            Dim::Vertex(i) => format!("VERTEX {}", i + 1),
        }
    }

    /// Whether the value is an angle (radians).
    pub fn is_angle(self) -> bool {
        matches!(self, Dim::Cant | Dim::Angle)
    }
}

/// The shape of a handle: a square moves in a straight line, a circle along an arc or a curve.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HandleKind {
    /// A length, diameter, thickness or position.
    Linear,
    /// An angle around the axis.
    Angular,
    /// A shape setting, slid along the profile.
    Curve,
}

/// Where a handle is seen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HandleView {
    /// On the part, side-on.
    Side,
    /// In the aft-view inset (thickness, angles around the axis).
    Aft,
    /// Inside the shell: shown in Cutaway.
    Inside,
}

/// One handle of a part.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HandleSpec {
    /// The dimension it edits.
    pub dim: Dim,
    /// Square or circle.
    pub kind: HandleKind,
    /// Shown on select (the rest need `Alt` or the 2D profile).
    pub primary: bool,
    /// Where it is drawn.
    pub view: HandleView,
}

const fn spec(dim: Dim, primary: bool, view: HandleView) -> HandleSpec {
    let kind = match dim {
        Dim::Cant | Dim::Angle => HandleKind::Angular,
        Dim::Shape | Dim::FilletRadius => HandleKind::Curve,
        _ => HandleKind::Linear,
    };
    HandleSpec { dim, kind, primary, view }
}

/// The shoulder a dimension belongs to: `(aft, field)` with field 0 length, 1 outer diameter, 2 wall.
fn shoulder_dim(dim: Dim) -> Option<(bool, u8)> {
    Some(match dim {
        Dim::ShoulderLength => (true, 0),
        Dim::ShoulderDiameter => (true, 1),
        Dim::ShoulderThickness => (true, 2),
        Dim::ForeShoulderLength => (false, 0),
        Dim::ForeShoulderDiameter => (false, 1),
        Dim::ForeShoulderThickness => (false, 2),
        _ => return None,
    })
}

/// The result of an applied edit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DimEdit {
    /// The value that was set (the request, limited).
    pub applied: f64,
    /// Why the request was limited, if it was.
    pub limited: Option<&'static str>,
}

const MIN_LEN: f64 = 0.002;
const MAX_LEN: f64 = 3.0;
const MIN_DIA: f64 = 0.004;
const MAX_DIA: f64 = 0.6;
const MAX_CANT: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// `(min, max, reason at min, reason at max)`.
type Limits = (f64, f64, &'static str, &'static str);

fn open() -> Limits {
    (f64::NEG_INFINITY, f64::INFINITY, "", "")
}

impl Rocket {
    /// The handles of part `id`, primary ones first as listed in the DirectEdit table. Shoulder length and diameter and a
    /// fin's fillet radius are secondary: shown in Cutaway or with `Alt`.
    pub fn handle_specs(&self, id: PartId) -> Vec<HandleSpec> {
        use HandleView::{Aft, Inside, Side};
        if let Some((_, sub)) = self.scratch_of(id) {
            return sub.handle_specs(id);
        }
        let Some(part) = self.kind_of(id) else {
            return Vec::new();
        };
        match part {
            PartRef::Axial(p) => match &p.kind {
                PartKind::NoseCone { shape, .. } => {
                    let mut v = vec![
                        spec(Dim::Length, true, Side),
                        spec(Dim::Diameter, true, Side),
                        spec(Dim::Wall, false, Inside),
                        spec(Dim::ShoulderLength, false, Inside),
                        spec(Dim::ShoulderDiameter, false, Inside),
                    ];
                    if shape.parameter().is_some() {
                        v.push(spec(Dim::Shape, false, Side));
                    }
                    v
                }
                PartKind::BodyTube { .. } => vec![spec(Dim::Length, true, Side), spec(Dim::Diameter, true, Side), spec(Dim::Wall, false, Inside)],
                PartKind::Transition { .. } => vec![
                    spec(Dim::ForeDiameter, true, Side),
                    spec(Dim::Diameter, true, Side),
                    spec(Dim::Length, true, Side),
                    spec(Dim::Wall, false, Inside),
                    spec(Dim::ForeShoulderLength, false, Inside),
                    spec(Dim::ForeShoulderDiameter, false, Inside),
                    spec(Dim::ShoulderLength, false, Inside),
                    spec(Dim::ShoulderDiameter, false, Inside),
                ],
            },
            PartRef::Radial(r) => {
                let aft = [spec(Dim::Thickness, false, Aft), spec(Dim::Cant, false, Aft), spec(Dim::Angle, false, Aft)];
                match &r.kind {
                    RadialKind::FinSet { .. } => {
                        let mut v = vec![
                            spec(Dim::Span, true, Side),
                            spec(Dim::RootChord, true, Side),
                            spec(Dim::TipChord, true, Side),
                            spec(Dim::Sweep, true, Side),
                            spec(Dim::Position, true, Side),
                        ];
                        v.extend(aft);
                        v.push(spec(Dim::FilletRadius, false, Side));
                        v
                    }
                    RadialKind::EllipticalFin { .. } => {
                        let mut v = vec![spec(Dim::Span, true, Side), spec(Dim::RootChord, true, Side), spec(Dim::Position, true, Side)];
                        v.extend(aft);
                        v.push(spec(Dim::FilletRadius, false, Side));
                        v
                    }
                    RadialKind::FreeformFin { points, .. } => {
                        let mut v: Vec<HandleSpec> = (0..points.len()).map(|i| spec(Dim::Vertex(i), true, Side)).collect();
                        v.push(spec(Dim::Position, true, Side));
                        v.extend(aft);
                        v.push(spec(Dim::FilletRadius, false, Side));
                        v
                    }
                    RadialKind::LaunchLug { .. } => vec![spec(Dim::Position, true, Side), spec(Dim::Length, true, Side), spec(Dim::Angle, false, Aft)],
                    RadialKind::RailButton { .. } => vec![spec(Dim::Position, true, Side), spec(Dim::Height, false, Side), spec(Dim::Angle, false, Aft)],
                    RadialKind::Pod { .. } => vec![spec(Dim::Position, true, Side), spec(Dim::Gap, true, Side), spec(Dim::Angle, true, Aft)],
                }
            }
            PartRef::Internal(i) => match &i.kind {
                InternalKind::InnerTube { motor_mount, .. } => {
                    let mut v = vec![spec(Dim::Length, true, Inside), spec(Dim::Position, true, Inside)];
                    if *motor_mount {
                        v.push(spec(Dim::Overhang, true, Inside));
                    }
                    v.extend([spec(Dim::Diameter, true, Inside), spec(Dim::Wall, false, Inside)]);
                    v
                }
                InternalKind::Coupler { .. } => vec![spec(Dim::Length, true, Inside), spec(Dim::Position, true, Inside), spec(Dim::Wall, false, Inside)],
                InternalKind::CenteringRing { .. } | InternalKind::EngineBlock { .. } => {
                    vec![spec(Dim::Position, true, Inside), spec(Dim::Thickness, true, Inside), spec(Dim::InnerDiameter, false, Inside)]
                }
                InternalKind::Bulkhead { .. } => vec![spec(Dim::Position, true, Inside), spec(Dim::Thickness, true, Inside)],
                InternalKind::Parachute { .. } | InternalKind::ShockCord { .. } => {
                    vec![spec(Dim::Position, true, Inside), spec(Dim::PackedLength, false, Inside)]
                }
                InternalKind::Streamer { .. } => vec![spec(Dim::Position, true, Inside)],
                InternalKind::MassComponent { .. } => vec![spec(Dim::Position, true, Inside), spec(Dim::Length, false, Inside)],
            },
        }
    }

    /// The label of `dim` on part `id` ("AFT DIAMETER" on a transition).
    pub fn dim_label(&self, id: PartId, dim: Dim) -> String {
        match (self.part(id).map(|p| &p.kind), dim) {
            (Some(PartKind::Transition { .. }), Dim::Diameter) => "AFT DIAMETER".into(),
            (Some(PartKind::NoseCone { .. }), Dim::Diameter) => "BASE DIAMETER".into(),
            _ => dim.label(),
        }
    }

    /// The current value of `dim` on part `id`; `None` if the part has no such dimension. Auto internal diameters resolve.
    pub fn dim(&self, id: PartId, dim: Dim) -> Option<f64> {
        if let Some((_, sub)) = self.scratch_of(id) {
            return sub.dim(id, dim);
        }
        match self.kind_of(id)? {
            PartRef::Axial(p) => match dim {
                Dim::Length => Some(p.kind.length()),
                Dim::Diameter => Some(p.kind.aft_diameter()),
                Dim::ForeDiameter => matches!(p.kind, PartKind::Transition { .. }).then(|| p.kind.fore_diameter()),
                Dim::Wall => Some(p.thickness),
                Dim::InnerDiameter => Some((p.kind.fore_diameter().max(p.kind.aft_diameter()) - 2.0 * p.thickness).max(0.0)),
                Dim::Shape => match &p.kind {
                    PartKind::NoseCone { shape, .. } => shape.parameter(),
                    _ => None,
                },
                d if shoulder_dim(d).is_some() => {
                    let (aft, field) = shoulder_dim(d)?;
                    let s = self.shoulder(id, aft)?;
                    match field {
                        0 => Some(s.length),
                        1 => self.shoulder_diameter(id, aft),
                        _ => Some(s.thickness),
                    }
                }
                _ => None,
            },
            PartRef::Radial(r) => match (dim, &r.kind) {
                (Dim::Position, _) => Some(r.offset),
                (Dim::Gap, RadialKind::Pod { gap, .. }) => Some(*gap),
                (Dim::Angle, _) => Some(r.angle),
                (Dim::Cant, k) if k.is_fin() => Some(r.cant),
                (Dim::Span, k) => k.fin_span(),
                (Dim::RootChord, RadialKind::FinSet { root_chord, .. } | RadialKind::EllipticalFin { root_chord, .. }) => Some(*root_chord),
                (Dim::TipChord, RadialKind::FinSet { tip_chord, .. }) => Some(*tip_chord),
                (Dim::Sweep, RadialKind::FinSet { sweep, .. }) => Some(*sweep),
                (Dim::Thickness, k) => k.thickness(),
                (Dim::FilletRadius, k) if k.is_fin() => Some(r.fillet_radius),
                (Dim::Length, RadialKind::LaunchLug { length, .. }) => Some(*length),
                (Dim::Diameter, RadialKind::LaunchLug { diameter, .. } | RadialKind::RailButton { diameter, .. }) => Some(*diameter),
                (Dim::Height, RadialKind::RailButton { height, .. }) => Some(*height),
                _ => None,
            },
            PartRef::Internal(p) => {
                let res = self.resolved(id)?;
                match (dim, &p.kind) {
                    (Dim::Position, _) => Some(p.offset),
                    (
                        Dim::Length,
                        InternalKind::InnerTube { length, .. } | InternalKind::Coupler { length, .. } | InternalKind::MassComponent { length, .. },
                    ) => Some(*length),
                    (Dim::Diameter, InternalKind::InnerTube { .. } | InternalKind::Coupler { .. } | InternalKind::Bulkhead { .. }) => Some(res.outer_diameter),
                    (Dim::Wall, InternalKind::InnerTube { thickness, .. } | InternalKind::Coupler { thickness, .. }) => Some(*thickness),
                    (
                        Dim::Thickness,
                        InternalKind::CenteringRing { thickness, .. } | InternalKind::Bulkhead { thickness, .. } | InternalKind::EngineBlock { thickness, .. },
                    ) => Some(*thickness),
                    (Dim::InnerDiameter, InternalKind::CenteringRing { .. } | InternalKind::EngineBlock { .. }) => Some(res.inner_diameter),
                    (Dim::Overhang, InternalKind::InnerTube { overhang, .. }) => Some(*overhang),
                    (Dim::PackedLength, InternalKind::Parachute { packed_length, .. } | InternalKind::ShockCord { packed_length, .. }) => Some(*packed_length),
                    (Dim::InnerDiameter, InternalKind::InnerTube { thickness, .. } | InternalKind::Coupler { thickness, .. }) => {
                        Some((res.outer_diameter - 2.0 * thickness).max(0.0))
                    }
                    _ => None,
                }
            }
        }
    }

    fn dim_limits(&self, id: PartId, dim: Dim) -> Option<Limits> {
        let part = self.kind_of(id)?;
        let length_of_parent = |parent: PartId| self.part(parent).map_or(MAX_LEN, |p| p.kind.length());
        Some(match (part, dim) {
            (PartRef::Axial(_), Dim::Length) => (MIN_LEN, MAX_LEN, "MINIMUM LENGTH", "MAXIMUM LENGTH"),
            (PartRef::Axial(p), Dim::Diameter | Dim::ForeDiameter) => (MIN_DIA.max(2.0 * p.thickness), MAX_DIA, "WALL FILLS THE TUBE", "MAXIMUM DIAMETER"),
            (PartRef::Axial(p), Dim::Wall) => (0.0, p.kind.fore_diameter().max(p.kind.aft_diameter()) / 2.0, "NO WALL", "WALL = RADIUS"),
            (PartRef::Axial(p), d) if shoulder_dim(d).is_some() => {
                let (aft, field) = shoulder_dim(d).unwrap_or((true, 0));
                let i = self.index_of(id)?;
                let neighbour = if aft { self.parts.get(i + 1) } else { i.checked_sub(1).and_then(|j| self.parts.get(j)) };
                match field {
                    0 => (0.0, neighbour.map_or(0.5, |n| n.kind.length()).max(MIN_LEN), "NO SHOULDER", "TUBE LENGTH"),
                    1 => (MIN_DIA, p.kind.fore_diameter().max(p.kind.aft_diameter()).max(MIN_DIA), "MINIMUM DIAMETER", "PART DIAMETER"),
                    _ => {
                        let d = self.shoulder_diameter(id, aft).unwrap_or(0.0);
                        (0.0003, (d / 2.0).max(0.0003), "MINIMUM WALL", "SOLID")
                    }
                }
            }
            (PartRef::Axial(p), Dim::Shape) => match &p.kind {
                PartKind::NoseCone { shape: NoseShape::PowerSeries(_), .. } => (0.01, 1.0, "n AT MINIMUM", "n = 1 IS A CONE"),
                _ => (0.0, 1.0, "C AT MINIMUM", "C AT MAXIMUM"),
            },
            (PartRef::Radial(r), Dim::Position) => (0.0, (length_of_parent(r.parent) - r.kind.min_attached()).max(0.0), "AT TUBE FORE", "AT TUBE AFT"),
            (PartRef::Radial(_), Dim::Angle) => open(),
            (PartRef::Radial(_), Dim::Gap) => (0.0, 0.5, "TOUCHING THE SURFACE", "MAXIMUM GAP"),
            (PartRef::Radial(_), Dim::Cant) => (-MAX_CANT, MAX_CANT, "CANT LIMIT 30°", "CANT LIMIT 30°"),
            (PartRef::Radial(_), Dim::Span) => (0.005, 1.0, "MINIMUM HEIGHT", "MAXIMUM HEIGHT"),
            (PartRef::Radial(r), Dim::RootChord) => (0.005, length_of_parent(r.parent).max(0.005), "MINIMUM CHORD", "LONGER THAN THE TUBE"),
            (PartRef::Radial(_), Dim::TipChord) => (0.0, 2.0, "POINTED TIP", "MAXIMUM CHORD"),
            (PartRef::Radial(r), Dim::Sweep) => (-r.offset, 1.0, "TIP AT TUBE FORE", "MAXIMUM SWEEP"),
            (PartRef::Radial(_), Dim::Thickness) => (0.0003, 0.02, "MINIMUM THICKNESS", "MAXIMUM THICKNESS"),
            (PartRef::Radial(_), Dim::Height) => (0.002, 0.1, "MINIMUM HEIGHT", "MAXIMUM HEIGHT"),
            (PartRef::Radial(r), Dim::FilletRadius) if r.kind.is_fin() => (0.0, 0.05, "NO FILLET", "MAXIMUM FILLET"),
            (PartRef::Radial(_), Dim::Length) => (0.005, 1.0, "MINIMUM LENGTH", "MAXIMUM LENGTH"),
            (PartRef::Radial(_), Dim::Diameter) => (0.002, 0.1, "MINIMUM DIAMETER", "MAXIMUM DIAMETER"),
            (PartRef::Internal(p), _) => {
                let room = (length_of_parent(p.parent) - p.offset).max(MIN_LEN);
                let bore = self.inner_diameter_at(p.parent, p.offset + p.kind.extent() / 2.0).unwrap_or(MAX_DIA);
                let res = self.resolved(id)?;
                match dim {
                    Dim::Position => (0.0, (length_of_parent(p.parent) - p.kind.extent()).max(0.0), "AT TUBE FORE", "AT TUBE AFT"),
                    Dim::Length | Dim::PackedLength | Dim::Thickness => (0.0005, room, "MINIMUM", "TUBE END"),
                    Dim::Diameter => {
                        let wall = match p.kind {
                            InternalKind::InnerTube { thickness, .. } | InternalKind::Coupler { thickness, .. } => thickness,
                            _ => 0.0,
                        };
                        (MIN_DIA.max(2.0 * wall + 0.002), bore.max(MIN_DIA), "WALL FILLS THE TUBE", "TUBE BORE")
                    }
                    Dim::Wall => (0.0005, res.outer_diameter / 2.0, "MINIMUM WALL", "WALL = RADIUS"),
                    Dim::InnerDiameter => (0.0, res.outer_diameter, "SOLID DISC", "RING FILLS THE TUBE"),
                    Dim::Overhang => (-p.kind.extent(), 0.5, "FULLY RECESSED", "MAXIMUM OVERHANG"),
                    _ => return None,
                }
            }
            _ => return None,
        })
    }

    /// `value` limited to what `dim` of part `id` may be, with the reason when it was limited. `None` if the part has no
    /// such dimension.
    pub fn clamp_dim(&self, id: PartId, dim: Dim, value: f64) -> Option<(f64, Option<&'static str>)> {
        if let Some((_, sub)) = self.scratch_of(id) {
            return sub.clamp_dim(id, dim, value);
        }
        self.dim(id, dim)?;
        let (lo, hi, rlo, rhi) = self.dim_limits(id, dim)?;
        Some(if value < lo - 1e-12 {
            (lo, Some(rlo))
        } else if value > hi + 1e-12 {
            (hi, Some(rhi))
        } else {
            (value, None)
        })
    }

    /// Sets `dim` of part `id` to `value`, limited by [`clamp_dim`](Self::clamp_dim). A diameter change carries to the
    /// neighbouring faces that were equal to it (a nose's base follows its tube, a transition's face follows the tube it
    /// joins); parts aft of a length change move with the stack. `None` if the part has no such dimension.
    pub fn set_dim(&mut self, id: PartId, dim: Dim, value: f64) -> Option<DimEdit> {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_dim(id, dim, value)).flatten();
        }
        let (v, limited) = self.clamp_dim(id, dim, value)?;
        let ok = match self.kind_of(id)? {
            PartRef::Axial(p) => {
                let (kind, old_fore, old_aft) = (p.kind.clone(), p.kind.fore_diameter(), p.kind.aft_diameter());
                match dim {
                    Dim::Length => self.set_kind(id, kind.with_length(v)),
                    Dim::Diameter => {
                        let new = kind.with_aft_diameter(v);
                        let new = if matches!(new, PartKind::BodyTube { .. }) { new.with_fore_diameter(v) } else { new };
                        let done = self.set_kind(id, new);
                        if done {
                            let i = self.index_of(id)?;
                            self.follow(i, true, old_aft, v);
                            if matches!(kind, PartKind::BodyTube { .. }) {
                                self.follow(i, false, old_fore, v);
                            }
                        }
                        done
                    }
                    Dim::ForeDiameter => {
                        let done = self.set_kind(id, kind.with_fore_diameter(v));
                        if done {
                            let i = self.index_of(id)?;
                            self.follow(i, false, old_fore, v);
                        }
                        done
                    }
                    Dim::Wall => self.set_thickness(id, v),
                    d if shoulder_dim(d).is_some() => {
                        let (aft, field) = shoulder_dim(d).unwrap_or((true, 0));
                        self.set_shoulder_field(id, aft, field, v)
                    }
                    Dim::Shape => match kind {
                        PartKind::NoseCone { shape, length, diameter } => {
                            let shape = match shape {
                                NoseShape::PowerSeries(_) => NoseShape::PowerSeries(v),
                                NoseShape::Haack(_) => NoseShape::Haack(v),
                                other => other,
                            };
                            self.set_kind(id, PartKind::NoseCone { shape, length, diameter })
                        }
                        _ => false,
                    },
                    _ => false,
                }
            }
            PartRef::Radial(r) => {
                let (parent, offset, angle, count, mut kind) = (r.parent, r.offset, r.angle, r.count, r.kind.clone());
                match dim {
                    Dim::Position => self.move_radial(id, parent, v, angle).is_ok(),
                    Dim::Angle => self.move_radial(id, parent, offset, v).is_ok(),
                    Dim::Cant => self.set_cant(id, v),
                    Dim::FilletRadius => self.set_fillet_radius(id, v),
                    _ => {
                        match (&mut kind, dim) {
                            (RadialKind::FinSet { span, .. } | RadialKind::EllipticalFin { span, .. }, Dim::Span) => *span = v,
                            (RadialKind::FinSet { root_chord, .. } | RadialKind::EllipticalFin { root_chord, .. }, Dim::RootChord) => *root_chord = v,
                            (RadialKind::FinSet { tip_chord, .. }, Dim::TipChord) => *tip_chord = v,
                            (RadialKind::FinSet { sweep, .. }, Dim::Sweep) => *sweep = v,
                            (
                                RadialKind::FinSet { thickness, .. } | RadialKind::EllipticalFin { thickness, .. } | RadialKind::FreeformFin { thickness, .. },
                                Dim::Thickness,
                            ) => *thickness = v,
                            (RadialKind::LaunchLug { length, .. }, Dim::Length) => *length = v,
                            (RadialKind::LaunchLug { diameter, .. } | RadialKind::RailButton { diameter, .. }, Dim::Diameter) => *diameter = v,
                            (RadialKind::RailButton { height, .. }, Dim::Height) => *height = v,
                            (RadialKind::Pod { gap, .. }, Dim::Gap) => *gap = v,
                            _ => return None,
                        }
                        self.set_radial(id, kind, count)
                    }
                }
            }
            PartRef::Internal(p) => {
                let (parent, mut kind) = (p.parent, p.kind.clone());
                if dim == Dim::Position {
                    self.move_internal(id, parent, v).is_ok()
                } else {
                    match (&mut kind, dim) {
                        (
                            InternalKind::InnerTube { length, .. } | InternalKind::Coupler { length, .. } | InternalKind::MassComponent { length, .. },
                            Dim::Length,
                        ) => *length = v,
                        (InternalKind::InnerTube { outer_diameter, .. }, Dim::Diameter) => *outer_diameter = v,
                        (InternalKind::Coupler { outer_diameter, .. }, Dim::Diameter) => *outer_diameter = Some(v),
                        (InternalKind::Bulkhead { diameter, .. }, Dim::Diameter) => *diameter = Some(v),
                        (InternalKind::InnerTube { thickness, .. } | InternalKind::Coupler { thickness, .. }, Dim::Wall) => *thickness = v,
                        (
                            InternalKind::CenteringRing { thickness, .. }
                            | InternalKind::Bulkhead { thickness, .. }
                            | InternalKind::EngineBlock { thickness, .. },
                            Dim::Thickness,
                        ) => *thickness = v,
                        (InternalKind::CenteringRing { inner_diameter, .. }, Dim::InnerDiameter) => *inner_diameter = Some(v),
                        (InternalKind::EngineBlock { inner, .. }, Dim::InnerDiameter) => *inner = v,
                        (InternalKind::InnerTube { overhang, .. }, Dim::Overhang) => *overhang = v,
                        (InternalKind::Parachute { packed_length, .. } | InternalKind::ShockCord { packed_length, .. }, Dim::PackedLength) => {
                            *packed_length = v
                        }
                        _ => return None,
                    }
                    self.set_internal_kind(id, kind)
                }
            }
        };
        ok.then_some(DimEdit { applied: v, limited })
    }

    /// Carries a diameter change from part `i` to its neighbour on the aft (`aft`) or fore side: a face equal to `old` takes
    /// `new`; a body tube passes it on, a nose cone or transition stops it.
    fn follow(&mut self, mut i: usize, aft: bool, old: f64, new: f64) {
        loop {
            let j = if aft {
                i + 1
            } else {
                match i.checked_sub(1) {
                    Some(j) => j,
                    None => return,
                }
            };
            let Some(neighbour) = self.parts.get(j) else {
                return;
            };
            let face = if aft { neighbour.kind.fore_diameter() } else { neighbour.kind.aft_diameter() };
            if (face - old).abs() > 1e-9 {
                return;
            }
            let (id, kind) = (neighbour.id, neighbour.kind.clone());
            let changed = match (&kind, aft) {
                (PartKind::BodyTube { .. }, _) => kind.with_aft_diameter(new),
                (PartKind::Transition { .. }, true) => kind.with_fore_diameter(new),
                (PartKind::Transition { .. } | PartKind::NoseCone { .. }, false) => kind.with_aft_diameter(new),
                (PartKind::NoseCone { .. }, true) => return,
            };
            self.set_kind(id, changed);
            if !matches!(kind, PartKind::BodyTube { .. }) {
                return;
            }
            i = j;
        }
    }

    /// The shoulder of axial part `id` at its aft face (`aft`) or fore face; `None` for parts that have none (body tubes, and
    /// a nose cone's fore end).
    pub fn shoulder(&self, id: PartId, aft: bool) -> Option<Shoulder> {
        let p = self.part(id)?;
        match (&p.kind, aft) {
            (PartKind::NoseCone { .. } | PartKind::Transition { .. }, true) => Some(p.shoulder_aft),
            (PartKind::Transition { .. }, false) => Some(p.shoulder_fore),
            _ => None,
        }
    }

    /// The outer diameter of a shoulder with `AUTO` resolved: the inner diameter of the tube it enters (the neighbour on that
    /// side), or the part's own inner diameter at that face with no neighbour.
    pub fn shoulder_diameter(&self, id: PartId, aft: bool) -> Option<f64> {
        if let Some((_, sub)) = self.scratch_of(id) {
            return sub.shoulder_diameter(id, aft);
        }
        let s = self.shoulder(id, aft)?;
        if s.diameter > 0.0 {
            return Some(s.diameter);
        }
        let i = self.index_of(id)?;
        let neighbour = if aft { self.parts.get(i + 1) } else { i.checked_sub(1).and_then(|j| self.parts.get(j)) };
        Some(match neighbour {
            Some(n) => n.inner_diameter_at(if aft { 0.0 } else { n.kind.length() }),
            None => {
                let p = &self.parts[i];
                p.inner_diameter_at(if aft { p.kind.length() } else { 0.0 })
            }
        })
    }

    /// Puts a shoulder's outer diameter back on `AUTO` (following the tube it enters). Returns whether the part has that
    /// shoulder.
    pub fn reset_shoulder_diameter(&mut self, id: PartId, aft: bool) -> bool {
        self.set_shoulder_field(id, aft, 1, 0.0)
    }

    /// Sets a shoulder field (`field`: 0 length, 1 outer diameter, 2 wall).
    fn set_shoulder_field(&mut self, id: PartId, aft: bool, field: u8, v: f64) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_shoulder_field(id, aft, field, v)).unwrap_or(false);
        }
        if self.shoulder(id, aft).is_none() {
            return false;
        }
        let Some(p) = self.parts.iter_mut().find(|p| p.id == id) else {
            return false;
        };
        let s = if aft { &mut p.shoulder_aft } else { &mut p.shoulder_fore };
        match field {
            0 => s.length = v,
            1 => s.diameter = v,
            _ => s.thickness = v,
        }
        true
    }

    /// Sets the fillet radius of fin group `id` (0 removes the fillet). Returns whether `id` is a fin.
    pub fn set_fillet_radius(&mut self, id: PartId, radius: f64) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_fillet_radius(id, radius)).unwrap_or(false);
        }
        match self.radials.iter_mut().find(|r| r.id == id && r.kind.is_fin()) {
            Some(r) => {
                r.fillet_radius = radius.max(0.0);
                true
            }
            None => false,
        }
    }

    /// Sets what the fillet of fin group `id` is made of (a bulk material). Returns whether it was applied.
    pub fn set_fillet_material(&mut self, id: PartId, material: crate::MaterialId) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_fillet_material(id, material)).unwrap_or(false);
        }
        if material.kind() != crate::MaterialKind::Bulk {
            return false;
        }
        match self.radials.iter_mut().find(|r| r.id == id && r.kind.is_fin()) {
            Some(r) => {
                r.fillet_material = material;
                true
            }
            None => false,
        }
    }

    /// The `(z, height)` vertex `index` of freeform fin `id`.
    pub fn vertex(&self, id: PartId, index: usize) -> Option<(f64, f64)> {
        match &self.radial(id)?.kind {
            RadialKind::FreeformFin { points, .. } => points.get(index).copied(),
            _ => None,
        }
    }

    /// Moves vertex `index` of freeform fin `id` to `(z, height)`; the two root vertices stay on the surface (height 0) and no
    /// vertex goes ahead of the parent's fore face. Returns whether it was applied.
    pub fn set_vertex(&mut self, id: PartId, index: usize, z: f64, height: f64) -> bool {
        self.edit_points(id, |points, offset| {
            let last = points.len() - 1;
            let Some(p) = points.get_mut(index) else {
                return false;
            };
            *p = (z.max(-offset), if index == 0 || index == last { 0.0 } else { height.max(0.0) });
            true
        })
    }

    /// Adds a vertex to freeform fin `id` after vertex `after`, at `(z, height)`.
    pub fn add_vertex(&mut self, id: PartId, after: usize, z: f64, height: f64) -> bool {
        self.edit_points(id, |points, _| {
            if after >= points.len() {
                return false;
            }
            points.insert(after + 1, (z, height.max(0.0)));
            true
        })
    }

    /// Removes vertex `index` from freeform fin `id` (a fin keeps at least three).
    pub fn remove_vertex(&mut self, id: PartId, index: usize) -> bool {
        self.edit_points(id, |points, _| {
            if points.len() <= 3 || index >= points.len() {
                return false;
            }
            points.remove(index);
            true
        })
    }

    fn edit_points(&mut self, id: PartId, f: impl FnOnce(&mut Vec<(f64, f64)>, f64) -> bool) -> bool {
        let Some(r) = self.radial(id) else {
            return false;
        };
        let (offset, count, mut kind) = (r.offset, r.count, r.kind.clone());
        let RadialKind::FreeformFin { points, .. } = &mut kind else {
            return false;
        };
        f(points, offset) && self.set_radial(id, kind, count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::NoseShape;

    fn rocket() -> (Rocket, PartId, PartId, PartId) {
        let mut r = Rocket::default();
        let nose = r.insert(0, PartKind::NoseCone { shape: NoseShape::Haack(1.0 / 3.0), length: 0.2, diameter: 0.1 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let tail = r.insert(2, PartKind::BodyTube { length: 0.3, diameter: 0.1 }).unwrap();
        (r, nose, tube, tail)
    }

    #[test]
    fn diameter_follows_neighbours_that_matched() {
        let (mut r, nose, tube, tail) = rocket();
        let e = r.set_dim(tube, Dim::Diameter, 0.12).unwrap();
        assert_eq!(e.limited, None);
        assert_eq!(r.dim(nose, Dim::Diameter), Some(0.12));
        assert_eq!(r.dim(tail, Dim::Diameter), Some(0.12));
    }

    #[test]
    fn a_transition_face_follows_but_stops_the_chain() {
        let (mut r, _, tube, tail) = rocket();
        let t = r.insert(2, PartKind::Transition { length: 0.1, fore_diameter: 0.1, aft_diameter: 0.06 }).unwrap();
        // Inserted between two 0.1 tubes the transition matched both; narrowing its aft face takes the tail with it.
        r.set_dim(t, Dim::Diameter, 0.06).unwrap();
        assert_eq!(r.dim(tail, Dim::Diameter), Some(0.06));
        r.set_dim(tube, Dim::Diameter, 0.08).unwrap();
        assert_eq!(r.dim(t, Dim::ForeDiameter), Some(0.08));
        assert_eq!(r.dim(t, Dim::Diameter), Some(0.06));
        assert_eq!(r.dim(tail, Dim::Diameter), Some(0.06));
    }

    #[test]
    fn limits_stop_with_a_reason() {
        let (mut r, _, tube, _) = rocket();
        let e = r.set_dim(tube, Dim::Length, -1.0).unwrap();
        assert_eq!((e.applied, e.limited), (MIN_LEN, Some("MINIMUM LENGTH")));
        r.set_thickness(tube, 0.004);
        let e = r.set_dim(tube, Dim::Diameter, 0.004).unwrap();
        assert_eq!((e.applied, e.limited), (0.008, Some("WALL FILLS THE TUBE")));
        let e = r.set_dim(tube, Dim::Wall, 1.0).unwrap();
        assert_eq!(e.limited, Some("WALL = RADIUS"));
        assert!(r.set_dim(tube, Dim::Span, 0.1).is_none());
    }

    #[test]
    fn fins_and_internals_edit_through_set_dim() {
        let (mut r, _, tube, _) = rocket();
        let fin =
            r.attach_radial(tube, 0.3, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.08, sweep: 0.04, thickness: 0.003 }).unwrap();
        r.set_dim(fin, Dim::Span, 0.1).unwrap();
        assert_eq!(r.dim(fin, Dim::Span), Some(0.1));
        let e = r.set_dim(fin, Dim::Position, 5.0).unwrap();
        assert_eq!(e.limited, Some("AT TUBE AFT"));
        assert!((r.dim(fin, Dim::Position).unwrap() - (0.5 - 0.025)).abs() < 1e-9);
        let e = r.set_dim(fin, Dim::Cant, 1.0).unwrap();
        assert_eq!(e.limited, Some("CANT LIMIT 30°"));
        let ring = r.attach_internal(tube, 0.1, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 }).unwrap();
        r.set_dim(ring, Dim::InnerDiameter, 0.03).unwrap();
        assert!((r.dim(ring, Dim::InnerDiameter).unwrap() - 0.03).abs() < 1e-12);
        let e = r.set_dim(ring, Dim::Position, 9.0).unwrap();
        assert_eq!(e.limited, Some("AT TUBE AFT"));
    }

    #[test]
    fn freeform_vertices_move_and_root_stays_on_the_surface() {
        let (mut r, _, tube, _) = rocket();
        let fin = r
            .attach_radial(tube, 0.2, 0.0, 1, RadialKind::FreeformFin { points: vec![(0.0, 0.0), (0.02, 0.05), (0.08, 0.05), (0.1, 0.0)], thickness: 0.003 })
            .unwrap();
        assert!(r.set_vertex(fin, 1, 0.03, 0.07));
        assert_eq!(r.vertex(fin, 1), Some((0.03, 0.07)));
        assert!(r.set_vertex(fin, 0, 0.0, 0.3));
        assert_eq!(r.vertex(fin, 0), Some((0.0, 0.0)));
        assert!(r.add_vertex(fin, 1, 0.05, 0.09));
        assert!(r.remove_vertex(fin, 2));
        assert!(r.remove_vertex(fin, 1));
        assert!(!r.remove_vertex(fin, 1));
    }

    #[test]
    fn every_listed_handle_has_a_value() {
        let (mut r, nose, tube, _) = rocket();
        let fin =
            r.attach_radial(tube, 0.3, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.08, sweep: 0.04, thickness: 0.003 }).unwrap();
        let mount = r
            .attach_internal(tube, 0.0, InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.002, length: 0.3, motor_mount: true, overhang: 0.0 })
            .unwrap();
        for id in [nose, tube, fin, mount] {
            for h in r.handle_specs(id) {
                assert!(r.dim(id, h.dim).is_some(), "{id} {:?}", h.dim);
            }
        }
    }
}
