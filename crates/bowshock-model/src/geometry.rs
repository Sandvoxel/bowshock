//! Airframe geometry: nose cone profiles, axial parts (nose, tube, transition) and radial parts (fins, lugs, rail buttons).
//!
//! Nose profiles follow Crowell's families as tabulated in *OpenRocket technical documentation* (Niskanen 2013, section 3.1.1)
//! and Gary Crowell, "The Descriptive Geometry of Nose Cones" (1996).

use serde::{Deserialize, Serialize};

use crate::material::{Finish, MaterialId};
use crate::PartId;

/// Nose cone profile family.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum NoseShape {
    /// Straight cone.
    Conical,
    /// Tangent ogive.
    TangentOgive,
    /// Von Karman (Haack LD) minimum-drag shape. Equal to `Haack(0.0)`.
    VonKarman,
    /// Half ellipse.
    Elliptical,
    /// Parabolic with K = 1.
    Parabolic,
    /// Power series `r = R (x/L)^n`, `0 < n <= 1` (`n = 1` is a cone, 0.5 a half-power "parabola").
    PowerSeries(f64),
    /// Haack series with shape constant `C`: `0` is the LD-Haack (Von Karman), 1/3 the LV-Haack, 2/3 the tangent-ogive-like
    /// "Haack tangent".
    Haack(f64),
}

impl NoseShape {
    /// One of each family, parametric ones with their conventional default.
    pub const LIST: [NoseShape; 7] = [
        NoseShape::Conical,
        NoseShape::TangentOgive,
        NoseShape::VonKarman,
        NoseShape::Elliptical,
        NoseShape::Parabolic,
        NoseShape::PowerSeries(0.5),
        NoseShape::Haack(1.0 / 3.0),
    ];

    /// Display name.
    pub fn name(&self) -> &'static str {
        match self {
            NoseShape::Conical => "Conical",
            NoseShape::TangentOgive => "Tangent ogive",
            NoseShape::VonKarman => "Von Kármán",
            NoseShape::Elliptical => "Elliptical",
            NoseShape::Parabolic => "Parabolic",
            NoseShape::PowerSeries(_) => "Power series",
            NoseShape::Haack(_) => "Haack series",
        }
    }

    /// The shape parameter (`n` or `C`) of a parametric family.
    pub fn parameter(&self) -> Option<f64> {
        match *self {
            NoseShape::PowerSeries(n) | NoseShape::Haack(n) => Some(n),
            _ => None,
        }
    }
}

/// Geometry of one airframe part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PartKind {
    /// Nose cone ending at `diameter`.
    NoseCone {
        /// Profile family.
        shape: NoseShape,
        /// Length along the axis.
        length: f64,
        /// Aft (base) diameter.
        diameter: f64,
    },
    /// Cylindrical body tube.
    BodyTube {
        /// Length along the axis.
        length: f64,
        /// Outer diameter.
        diameter: f64,
    },
    /// Conical transition (a boattail when it contracts).
    Transition {
        /// Length along the axis.
        length: f64,
        /// Diameter at the fore face.
        fore_diameter: f64,
        /// Diameter at the aft face.
        aft_diameter: f64,
    },
}

const NOSE_POINTS: usize = 32;

impl PartKind {
    /// Display name: "Nose cone", "Body tube", "Transition", or "Boattail" when aft < fore.
    pub fn name(&self) -> &'static str {
        match self {
            PartKind::NoseCone { .. } => "Nose cone",
            PartKind::BodyTube { .. } => "Body tube",
            PartKind::Transition { fore_diameter, aft_diameter, .. } => {
                if aft_diameter < fore_diameter {
                    "Boattail"
                } else {
                    "Transition"
                }
            }
        }
    }

    /// Length along the axis.
    pub fn length(&self) -> f64 {
        match *self {
            PartKind::NoseCone { length, .. } | PartKind::BodyTube { length, .. } | PartKind::Transition { length, .. } => length,
        }
    }

    /// Diameter at the fore face (0 for a nose cone).
    pub fn fore_diameter(&self) -> f64 {
        match *self {
            PartKind::NoseCone { .. } => 0.0,
            PartKind::BodyTube { diameter, .. } => diameter,
            PartKind::Transition { fore_diameter, .. } => fore_diameter,
        }
    }

    /// Diameter at the aft face.
    pub fn aft_diameter(&self) -> f64 {
        match *self {
            PartKind::NoseCone { diameter, .. } | PartKind::BodyTube { diameter, .. } => diameter,
            PartKind::Transition { aft_diameter, .. } => aft_diameter,
        }
    }

    /// This kind with its length replaced.
    pub fn with_length(&self, length: f64) -> PartKind {
        let mut out = self.clone();
        match &mut out {
            PartKind::NoseCone { length: l, .. } | PartKind::BodyTube { length: l, .. } | PartKind::Transition { length: l, .. } => *l = length,
        }
        out
    }

    /// This kind with its aft diameter replaced (a nose cone's base diameter, a tube's only diameter).
    pub fn with_aft_diameter(&self, diameter: f64) -> PartKind {
        let mut out = self.clone();
        match &mut out {
            PartKind::NoseCone { diameter: d, .. } | PartKind::BodyTube { diameter: d, .. } | PartKind::Transition { aft_diameter: d, .. } => *d = diameter,
        }
        out
    }

    /// This kind with its fore diameter replaced. A nose cone has no fore diameter (always a point) and a body tube has one
    /// diameter, so for those this is the same as [`with_aft_diameter`](Self::with_aft_diameter) / unchanged respectively.
    pub fn with_fore_diameter(&self, diameter: f64) -> PartKind {
        match self {
            PartKind::NoseCone { .. } => self.clone(),
            PartKind::BodyTube { .. } => self.with_aft_diameter(diameter),
            PartKind::Transition { length, aft_diameter, .. } => PartKind::Transition { length: *length, fore_diameter: diameter, aft_diameter: *aft_diameter },
        }
    }

    /// Outer radius at `z` from the part's fore face (clamped to the part).
    pub fn radius_at(&self, z: f64) -> f64 {
        match *self {
            PartKind::NoseCone { shape, length, diameter } => {
                if length <= 0.0 {
                    diameter / 2.0
                } else {
                    nose_radius(shape, length, diameter / 2.0, z.clamp(0.0, length))
                }
            }
            PartKind::BodyTube { diameter, .. } => diameter / 2.0,
            PartKind::Transition { length, fore_diameter, aft_diameter } => {
                let t = if length > 0.0 { (z / length).clamp(0.0, 1.0) } else { 1.0 };
                (fore_diameter + (aft_diameter - fore_diameter) * t) / 2.0
            }
        }
    }

    /// Outer profile as `(z, radius)` with z measured from the part's fore face, fore to aft.
    /// Noses are sampled with 33 points (32 segments); tubes and transitions use two.
    pub fn profile(&self) -> Vec<(f64, f64)> {
        match *self {
            PartKind::NoseCone { shape, length, diameter } => {
                let r_max = diameter / 2.0;
                (0..=NOSE_POINTS)
                    .map(|i| {
                        if i == 0 {
                            return (0.0, 0.0);
                        }
                        if i == NOSE_POINTS {
                            return (length, r_max);
                        }
                        let x = length * i as f64 / NOSE_POINTS as f64;
                        (x, nose_radius(shape, length, r_max, x))
                    })
                    .collect()
            }
            PartKind::BodyTube { length, diameter } => {
                vec![(0.0, diameter / 2.0), (length, diameter / 2.0)]
            }
            PartKind::Transition { length, fore_diameter, aft_diameter } => vec![(0.0, fore_diameter / 2.0), (length, aft_diameter / 2.0)],
        }
    }

    /// The stock a new part of this kind is made of.
    pub fn default_material(&self) -> MaterialId {
        match self {
            PartKind::NoseCone { .. } => MaterialId::POLYSTYRENE,
            PartKind::BodyTube { .. } | PartKind::Transition { .. } => MaterialId::CARDBOARD,
        }
    }
}

pub(crate) fn nose_radius(shape: NoseShape, l: f64, r: f64, x: f64) -> f64 {
    let t = x / l;
    let v = match shape {
        NoseShape::Conical => r * t,
        NoseShape::TangentOgive => {
            let rho = (r * r + l * l) / (2.0 * r);
            ((rho * rho - (l - x) * (l - x)).max(0.0)).sqrt() + r - rho
        }
        NoseShape::VonKarman => haack(r, t, 0.0),
        NoseShape::Elliptical => r * (1.0 - (1.0 - t) * (1.0 - t)).max(0.0).sqrt(),
        NoseShape::Parabolic => r * (2.0 * t - t * t),
        NoseShape::PowerSeries(n) => r * t.powf(n.clamp(0.01, 1.0)),
        NoseShape::Haack(c) => haack(r, t, c),
    };
    v.clamp(0.0, r)
}

/// Haack series radius at fraction `t` of the length: `R/sqrt(pi) * sqrt(theta - sin(2 theta)/2 + C sin^3 theta)`.
fn haack(r: f64, t: f64, c: f64) -> f64 {
    let theta = (1.0 - 2.0 * t).clamp(-1.0, 1.0).acos();
    r * ((theta - (2.0 * theta).sin() / 2.0 + c * theta.sin().powi(3)) / std::f64::consts::PI).max(0.0).sqrt()
}

/// One placed airframe part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Part {
    /// Unique, stable id.
    pub id: PartId,
    /// Geometry.
    pub kind: PartKind,
    /// User-given name, `None` for the kind's name.
    #[serde(default)]
    pub name: Option<String>,
    /// What the shell is made of.
    pub material: MaterialId,
    /// Outside finish (sets skin-friction roughness).
    pub finish: Finish,
    /// Shell wall thickness, metres. A thickness of at least the radius makes the part solid.
    pub thickness: f64,
    /// Mass that replaces the computed one, kg.
    #[serde(default)]
    pub mass_override: Option<f64>,
    /// CG that replaces the computed one, metres behind the part's fore face.
    #[serde(default)]
    pub cg_override: Option<f64>,
}

impl Part {
    /// A part of `kind` with the default material (and its usual wall), regular paint and no overrides.
    pub fn new(id: PartId, kind: PartKind) -> Part {
        let material = kind.default_material();
        Part { id, kind, name: None, material, finish: Finish::default(), thickness: material.material().default_wall, mass_override: None, cg_override: None }
    }

    /// The user's name, or the kind's ("Body tube").
    pub fn display_name(&self) -> String {
        self.name.clone().unwrap_or_else(|| self.kind.name().to_string())
    }

    /// Inner diameter of the shell at `z` from the fore face: the outer diameter less twice the wall, never negative.
    pub fn inner_diameter_at(&self, z: f64) -> f64 {
        (2.0 * (self.kind.radius_at(z) - self.thickness)).max(0.0)
    }
}

/// Section of a fin's plate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FinSection {
    /// Flat plate with square edges.
    #[default]
    Square,
    /// Edges rounded over.
    Rounded,
    /// Symmetric airfoil.
    Airfoil,
}

impl FinSection {
    /// Every section.
    pub const ALL: [FinSection; 3] = [FinSection::Square, FinSection::Rounded, FinSection::Airfoil];

    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            FinSection::Square => "Square",
            FinSection::Rounded => "Rounded",
            FinSection::Airfoil => "Airfoil",
        }
    }

    /// Plate volume as a fraction of planform area x thickness. Square is 1; an airfoil takes the area of a symmetric
    /// four-digit NACA section (0.685 of chord x thickness); rounded edges are an estimate (0.9).
    pub fn volume_factor(self) -> f64 {
        match self {
            FinSection::Square => 1.0,
            FinSection::Rounded => 0.9,
            FinSection::Airfoil => 0.685,
        }
    }
}

/// Planform family of a fin set.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FinPlanform {
    /// Trapezoid: [`RadialKind::FinSet`].
    Trapezoidal,
    /// Half ellipse on the root: [`RadialKind::EllipticalFin`].
    Elliptical,
    /// Arbitrary polygon: [`RadialKind::FreeformFin`].
    Freeform,
}

impl FinPlanform {
    /// Every planform.
    pub const ALL: [FinPlanform; 3] = [FinPlanform::Trapezoidal, FinPlanform::Elliptical, FinPlanform::Freeform];

    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            FinPlanform::Trapezoidal => "Trapezoidal",
            FinPlanform::Elliptical => "Elliptical",
            FinPlanform::Freeform => "Freeform",
        }
    }
}

/// Segments in the polygon that stands for an elliptical fin's outline.
const ELLIPSE_SEGMENTS: usize = 24;

/// Geometry of a part attached to the side of an axial part (a "radial" part), repeated around the axis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum RadialKind {
    /// A trapezoidal fin: root on the parent, tip `span` out, the tip's leading edge `sweep` aft of the root's.
    FinSet {
        /// Chord along the parent at the root.
        root_chord: f64,
        /// Chord at the tip.
        tip_chord: f64,
        /// Distance from the parent surface to the tip.
        span: f64,
        /// How far the tip's leading edge is behind the root's, along the axis.
        sweep: f64,
        /// Plate thickness.
        thickness: f64,
    },
    /// An elliptical fin: a half ellipse standing on the root, semi-axes `root_chord / 2` and `span`.
    EllipticalFin {
        /// Chord along the parent at the root (the ellipse's major axis).
        root_chord: f64,
        /// Distance from the parent surface to the tip.
        span: f64,
        /// Plate thickness.
        thickness: f64,
    },
    /// A fin of any outline: a closed polygon of `(z, height)` points, `z` along the axis from the fore end of the root and
    /// `height` above the parent surface. The root is the edge between the first and last point, so those two have height 0.
    FreeformFin {
        /// Polygon vertices in order (at least three).
        points: Vec<(f64, f64)>,
        /// Plate thickness.
        thickness: f64,
    },
    /// A tube parallel to the axis, standing off the surface.
    LaunchLug {
        /// Length along the axis.
        length: f64,
        /// Outer diameter.
        diameter: f64,
    },
    /// A rail button: a stem out from the surface with a wider head.
    RailButton {
        /// Height above the surface.
        height: f64,
        /// Head diameter.
        diameter: f64,
    },
}

impl RadialKind {
    /// Display name.
    pub fn name(&self) -> &'static str {
        match self {
            RadialKind::FinSet { .. } | RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => "Fin set",
            RadialKind::LaunchLug { .. } => "Launch lug",
            RadialKind::RailButton { .. } => "Rail button",
        }
    }

    /// Whether this is a fin of any planform.
    pub fn is_fin(&self) -> bool {
        self.fin_planform().is_some()
    }

    /// The planform family of a fin; `None` for lugs and buttons.
    pub fn fin_planform(&self) -> Option<FinPlanform> {
        match self {
            RadialKind::FinSet { .. } => Some(FinPlanform::Trapezoidal),
            RadialKind::EllipticalFin { .. } => Some(FinPlanform::Elliptical),
            RadialKind::FreeformFin { .. } => Some(FinPlanform::Freeform),
            _ => None,
        }
    }

    /// Plate thickness of a fin.
    pub fn thickness(&self) -> Option<f64> {
        match *self {
            RadialKind::FinSet { thickness, .. } | RadialKind::EllipticalFin { thickness, .. } | RadialKind::FreeformFin { thickness, .. } => Some(thickness),
            _ => None,
        }
    }

    /// How much of the parent's length it occupies along the axis.
    pub fn extent(&self) -> f64 {
        match self {
            RadialKind::FinSet { root_chord, .. } | RadialKind::EllipticalFin { root_chord, .. } => *root_chord,
            RadialKind::FreeformFin { points, .. } => points.iter().map(|p| p.0).fold(0.0, f64::max),
            RadialKind::LaunchLug { length, .. } => *length,
            RadialKind::RailButton { diameter, .. } => *diameter,
        }
    }

    /// This kind placed with its fore edge at `offset` on its parent: a fin may sweep forward (negative sweep), but its tip's
    /// leading edge may not go ahead of the parent's fore face, so the sweep is limited to `-offset`.
    pub fn limited_at(&self, offset: f64) -> RadialKind {
        match self {
            RadialKind::FinSet { root_chord, tip_chord, span, sweep, thickness } => {
                RadialKind::FinSet { root_chord: *root_chord, tip_chord: *tip_chord, span: *span, sweep: sweep.max(-offset), thickness: *thickness }
            }
            RadialKind::FreeformFin { points, thickness } => {
                RadialKind::FreeformFin { points: points.iter().map(|&(z, h)| (z.max(-offset), h)).collect(), thickness: *thickness }
            }
            _ => self.clone(),
        }
    }

    /// The least length of the part that must lie on its parent. A fin may trail past the parent's aft end as long as a
    /// quarter of its root chord stays on it; lugs and buttons must lie wholly on the parent.
    pub fn min_attached(&self) -> f64 {
        if self.is_fin() {
            self.extent() / 4.0
        } else {
            self.extent()
        }
    }

    /// The trapezoidal fin outline as four `(z, height)` points, `z` along the axis from the fore end of the root and `height`
    /// above the parent surface: root leading edge, tip leading edge, tip trailing edge, root trailing edge. `None` for
    /// every other kind (see [`planform`](Self::planform) for any fin).
    pub fn fin_outline(&self) -> Option<[(f64, f64); 4]> {
        match *self {
            RadialKind::FinSet { root_chord, tip_chord, span, sweep, .. } => Some([(0.0, 0.0), (sweep, span), (sweep + tip_chord, span), (root_chord, 0.0)]),
            _ => None,
        }
    }

    /// The outline of any fin as a closed polygon of `(z, height)` points (the last joins back to the first): the trapezoid's
    /// four corners, an ellipse sampled in 24 segments, or the freeform points. `None` for lugs and buttons.
    pub fn planform(&self) -> Option<Vec<(f64, f64)>> {
        match self {
            RadialKind::FinSet { .. } => self.fin_outline().map(|o| o.to_vec()),
            RadialKind::EllipticalFin { root_chord, span, .. } => Some(
                (0..=ELLIPSE_SEGMENTS)
                    .map(|i| {
                        let phi = std::f64::consts::PI * i as f64 / ELLIPSE_SEGMENTS as f64;
                        (root_chord / 2.0 * (1.0 - phi.cos()), if i == 0 || i == ELLIPSE_SEGMENTS { 0.0 } else { span * phi.sin() })
                    })
                    .collect(),
            ),
            RadialKind::FreeformFin { points, .. } => Some(points.clone()),
            _ => None,
        }
    }

    /// Planform area of one fin, m^2.
    pub fn fin_area(&self) -> Option<f64> {
        match self {
            RadialKind::FinSet { root_chord, tip_chord, span, .. } => Some((root_chord + tip_chord) / 2.0 * span),
            RadialKind::EllipticalFin { root_chord, span, .. } => Some(std::f64::consts::FRAC_PI_4 * root_chord * span),
            RadialKind::FreeformFin { points, .. } => Some(polygon_area_centroid(points).0),
            _ => None,
        }
    }

    /// Centroid of one fin's planform as `(z, height)`, `z` from the fore end of the root.
    pub fn fin_centroid(&self) -> Option<(f64, f64)> {
        match self {
            RadialKind::EllipticalFin { root_chord, span, .. } => Some((root_chord / 2.0, 4.0 * span / (3.0 * std::f64::consts::PI))),
            RadialKind::FinSet { .. } | RadialKind::FreeformFin { .. } => {
                let (_, z, h) = polygon_area_centroid(&self.planform()?);
                Some((z, h))
            }
            _ => None,
        }
    }

    /// Distance from the parent surface to the fin tip.
    pub fn fin_span(&self) -> Option<f64> {
        match self {
            RadialKind::FinSet { span, .. } | RadialKind::EllipticalFin { span, .. } => Some(*span),
            RadialKind::FreeformFin { points, .. } => Some(points.iter().map(|p| p.1).fold(0.0, f64::max)),
            _ => None,
        }
    }

    /// This fin with another planform, keeping what carries over: root chord, span and thickness. A trapezoid made from an
    /// ellipse or freeform fin gets a tip chord of 0.4 root chord and a sweep of 0.3 root chord; a freeform fin starts as the
    /// current outline. Lugs and buttons, and a fin already of that planform, are returned unchanged.
    pub fn with_planform(&self, planform: FinPlanform) -> RadialKind {
        let (Some(current), Some(thickness)) = (self.fin_planform(), self.thickness()) else {
            return self.clone();
        };
        if current == planform {
            return self.clone();
        }
        let (root_chord, span) = (self.extent(), self.fin_span().unwrap_or(0.0));
        match planform {
            FinPlanform::Trapezoidal => RadialKind::FinSet { root_chord, tip_chord: 0.4 * root_chord, span, sweep: 0.3 * root_chord, thickness },
            FinPlanform::Elliptical => RadialKind::EllipticalFin { root_chord, span, thickness },
            FinPlanform::Freeform => RadialKind::FreeformFin { points: self.planform().unwrap_or_default(), thickness },
        }
    }
}

/// Signed-area-free `(area, centroid z, centroid h)` of a polygon by the shoelace formula (the area is absolute). A degenerate
/// polygon gets the average of its vertices.
pub(crate) fn polygon_area_centroid(points: &[(f64, f64)]) -> (f64, f64, f64) {
    let n = points.len();
    if n == 0 {
        return (0.0, 0.0, 0.0);
    }
    let (mut a2, mut cz, mut ch) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let (p, q) = (points[i], points[(i + 1) % n]);
        let cross = p.0 * q.1 - q.0 * p.1;
        a2 += cross;
        cz += (p.0 + q.0) * cross;
        ch += (p.1 + q.1) * cross;
    }
    if a2.abs() < 1e-18 {
        let k = n as f64;
        return (0.0, points.iter().map(|p| p.0).sum::<f64>() / k, points.iter().map(|p| p.1).sum::<f64>() / k);
    }
    (a2.abs() / 2.0, cz / (3.0 * a2), ch / (3.0 * a2))
}

/// A radial part placed on an axial part: `count` linked copies spaced evenly around it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RadialPart {
    /// Unique id, from the same sequence as axial part ids.
    pub id: PartId,
    /// Geometry shared by every copy.
    pub kind: RadialKind,
    /// The axial part it sits on.
    pub parent: PartId,
    /// Distance from the parent's fore face to the fore edge of the part.
    pub offset: f64,
    /// Angle of the first copy, radians: 0 is +Y, increasing towards +X.
    pub angle: f64,
    /// Number of copies, evenly spaced.
    pub count: u8,
    /// User-given name, `None` for the kind's name.
    #[serde(default)]
    pub name: Option<String>,
    /// What each copy is made of.
    pub material: MaterialId,
    /// Outside finish.
    pub finish: Finish,
    /// Section of a fin's plate (ignored by lugs and buttons).
    #[serde(default)]
    pub cross_section: FinSection,
    /// Fin cant: rotation about the fin's own span axis, radians (positive spins the rocket the same way as increasing angle).
    #[serde(default)]
    pub cant: f64,
    /// Mass of the whole group (all `count` copies) that replaces the computed one, kg.
    #[serde(default)]
    pub mass_override: Option<f64>,
    /// CG that replaces the computed one, metres behind the group's fore edge.
    #[serde(default)]
    pub cg_override: Option<f64>,
}

impl RadialPart {
    /// A group with the default material for its kind, regular paint, a square section, no cant and no overrides.
    pub fn new(id: PartId, kind: RadialKind, parent: PartId, offset: f64, angle: f64, count: u8) -> RadialPart {
        let material = Self::default_material(&kind);
        RadialPart {
            id,
            kind,
            parent,
            offset,
            angle,
            count,
            name: None,
            material,
            finish: Finish::default(),
            cross_section: FinSection::default(),
            cant: 0.0,
            mass_override: None,
            cg_override: None,
        }
    }

    /// The stock a new part of this kind is made of: plywood fins, cardboard lugs, aluminum rail buttons.
    pub fn default_material(kind: &RadialKind) -> MaterialId {
        match kind {
            RadialKind::LaunchLug { .. } => MaterialId::CARDBOARD,
            RadialKind::RailButton { .. } => MaterialId::ALUMINUM,
            _ => MaterialId::BIRCH_PLYWOOD,
        }
    }

    /// The user's name, or the kind's ("Fin set").
    pub fn display_name(&self) -> String {
        self.name.clone().unwrap_or_else(|| self.kind.name().to_string())
    }

    /// The angle of each copy.
    pub fn angles(&self) -> Vec<f64> {
        let n = f64::from(self.count.max(1));
        (0..self.count.max(1)).map(|i| (self.angle + std::f64::consts::TAU * f64::from(i) / n).rem_euclid(std::f64::consts::TAU)).collect()
    }
}

/// Most copies a radial part may have.
pub const MAX_COUNT: u8 = 16;
