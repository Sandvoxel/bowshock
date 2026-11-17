//! Uniform scaling of a rocket or of chosen parts: every length is multiplied by one factor (geometric similarity), angles, counts,
//! materials, motors and drag coefficients are kept. Mass overrides follow the cube of the factor and centre-of-gravity overrides
//! the factor itself when asked, because a similar body of the same material has mass proportional to volume.

use std::collections::HashSet;

use crate::config::Ballast;
use crate::expr::{dim_kind, Kind};
use crate::geometry::{FinTab, Part, PartKind, PodBody, RadialKind, RadialPart, Shoulder};
use crate::internal::{InternalKind, InternalPart};
use crate::symmetry::RadialField;
use crate::{PartId, Rocket};

/// Smallest and largest factor [`Rocket::scale`] accepts.
const FACTOR_RANGE: (f64, f64) = (0.01, 100.0);

/// What [`Rocket::scale`] scales.
#[derive(Clone, Debug, PartialEq)]
pub enum ScaleScope {
    /// The whole rocket: every length, including where parts sit, so the design stays geometrically similar.
    Whole,
    /// Only these parts' own dimensions; their offsets stay, and the parts behind a scaled axial part move with the stack.
    Parts(Vec<PartId>),
}

/// How one scaling pass treats the quantities it meets.
#[derive(Clone, Copy)]
struct Pass {
    factor: f64,
    masses: bool,
    /// Whether positions (offsets) scale too.
    offsets: bool,
}

impl Pass {
    fn mass(self, m: Option<f64>) -> Option<f64> {
        if self.masses {
            m.map(|v| v * self.factor.powi(3))
        } else {
            m
        }
    }

    fn cg(self, c: Option<f64>) -> Option<f64> {
        if self.masses {
            c.map(|v| v * self.factor)
        } else {
            c
        }
    }

    fn offset(self, o: f64) -> f64 {
        if self.offsets {
            o * self.factor
        } else {
            o
        }
    }
}

fn scale_opt(v: Option<f64>, f: f64) -> Option<f64> {
    v.map(|x| x * f)
}

fn part_kind(k: &PartKind, f: f64) -> PartKind {
    match k {
        PartKind::NoseCone { shape, length, diameter, tip_radius } => {
            // A sphere scales to a sphere, so the tangent point and the proportions are those of the original.
            PartKind::NoseCone { shape: *shape, length: length * f, diameter: diameter * f, tip_radius: tip_radius * f }
        }
        PartKind::BodyTube { length, diameter } => PartKind::BodyTube { length: length * f, diameter: diameter * f },
        PartKind::Transition { length, fore_diameter, aft_diameter } => {
            PartKind::Transition { length: length * f, fore_diameter: fore_diameter * f, aft_diameter: aft_diameter * f }
        }
    }
}

fn shoulder(s: Shoulder, f: f64) -> Shoulder {
    Shoulder { length: s.length * f, diameter: s.diameter * f, thickness: s.thickness * f }
}

/// A part's own dimensions, except the stack-defining kind (which the caller sets).
fn part_fields(p: &mut Part, pass: Pass) {
    p.thickness *= pass.factor;
    p.shoulder_fore = shoulder(p.shoulder_fore, pass.factor);
    p.shoulder_aft = shoulder(p.shoulder_aft, pass.factor);
    p.mass_override = pass.mass(p.mass_override);
    p.cg_override = pass.cg(p.cg_override);
}

fn radial_kind(k: &RadialKind, pass: Pass) -> RadialKind {
    let f = pass.factor;
    match k {
        RadialKind::FinSet { root_chord, tip_chord, span, sweep, thickness } => {
            RadialKind::FinSet { root_chord: root_chord * f, tip_chord: tip_chord * f, span: span * f, sweep: sweep * f, thickness: thickness * f }
        }
        RadialKind::EllipticalFin { root_chord, span, thickness } => {
            RadialKind::EllipticalFin { root_chord: root_chord * f, span: span * f, thickness: thickness * f }
        }
        RadialKind::FreeformFin { points, thickness } => {
            RadialKind::FreeformFin { points: points.iter().map(|&(z, h)| (z * f, h * f)).collect(), thickness: thickness * f }
        }
        RadialKind::LaunchLug { length, diameter } => RadialKind::LaunchLug { length: length * f, diameter: diameter * f },
        RadialKind::RailButton { height, diameter } => RadialKind::RailButton { height: height * f, diameter: diameter * f },
        RadialKind::Pod { gap, body } => {
            let mut body = body.clone();
            // A pod is a rocket of its own: all of it scales, positions inside it included.
            scale_body(&mut body, Pass { offsets: true, ..pass });
            RadialKind::Pod { gap: gap * f, body }
        }
    }
}

/// A radial part's own dimensions and overrides.
fn radial_fields(r: &mut RadialPart, pass: Pass) {
    let f = pass.factor;
    r.kind = radial_kind(&r.kind, pass);
    r.fillet_radius *= f;
    r.tab = r.tab.map(|t| FinTab { height: t.height * f, length: t.length * f, position: t.position * f });
    for o in &mut r.overrides {
        if o.field != RadialField::Cant {
            o.value *= f;
        }
    }
    r.mass_override = pass.mass(r.mass_override);
    r.cg_override = pass.cg(r.cg_override);
}

fn internal_kind(k: &InternalKind, pass: Pass) -> InternalKind {
    let f = pass.factor;
    match k {
        InternalKind::InnerTube { outer_diameter, thickness, length, motor_mount, overhang } => InternalKind::InnerTube {
            outer_diameter: outer_diameter * f,
            thickness: thickness * f,
            length: length * f,
            motor_mount: *motor_mount,
            overhang: overhang * f,
        },
        InternalKind::CenteringRing { outer_diameter, inner_diameter, thickness } => InternalKind::CenteringRing {
            outer_diameter: scale_opt(*outer_diameter, f),
            inner_diameter: scale_opt(*inner_diameter, f),
            thickness: thickness * f,
        },
        InternalKind::Bulkhead { diameter, thickness } => InternalKind::Bulkhead { diameter: scale_opt(*diameter, f), thickness: thickness * f },
        InternalKind::Coupler { outer_diameter, thickness, length } => {
            InternalKind::Coupler { outer_diameter: scale_opt(*outer_diameter, f), thickness: thickness * f, length: length * f }
        }
        InternalKind::Parachute { diameter, cd, line_count, line_length, line_material, packed_length, packed_diameter, deploy } => InternalKind::Parachute {
            diameter: diameter * f,
            cd: *cd,
            line_count: *line_count,
            line_length: line_length * f,
            line_material: *line_material,
            packed_length: packed_length * f,
            packed_diameter: scale_opt(*packed_diameter, f),
            deploy: *deploy,
        },
        InternalKind::Streamer { length, width, cd, deploy } => InternalKind::Streamer { length: length * f, width: width * f, cd: *cd, deploy: *deploy },
        InternalKind::ShockCord { length, packed_length } => InternalKind::ShockCord { length: length * f, packed_length: packed_length * f },
        InternalKind::MassComponent { mass, length, diameter, role } => InternalKind::MassComponent {
            mass: if pass.masses { mass * f.powi(3) } else { *mass },
            length: length * f,
            diameter: scale_opt(*diameter, f),
            role: *role,
        },
    }
}

fn internal_fields(i: &mut InternalPart, pass: Pass) {
    i.kind = internal_kind(&i.kind, pass);
    i.mass_override = pass.mass(i.mass_override);
    i.cg_override = pass.cg(i.cg_override);
}

fn scale_body(body: &mut PodBody, pass: Pass) {
    for p in &mut body.parts {
        p.kind = part_kind(&p.kind, pass.factor);
        part_fields(p, pass);
    }
    for r in &mut body.radials {
        r.offset = pass.offset(r.offset);
        radial_fields(r, pass);
    }
    for i in &mut body.internals {
        i.offset = pass.offset(i.offset);
        internal_fields(i, pass);
    }
}

impl Rocket {
    /// Scales the rocket or some parts by `factor` (clamped to 0.01..=100), see [`ScaleScope`]. With `masses`, mass overrides
    /// (and the mass of mass components and ballast) multiply by `factor^3` and centre-of-gravity overrides by `factor`.
    /// Formulas for lengths have their constants scaled, then every formula is re-evaluated. Returns how many parts changed.
    pub fn scale(&mut self, factor: f64, scope: &ScaleScope, masses: bool) -> usize {
        if !factor.is_finite() || factor <= 0.0 {
            return 0;
        }
        let factor = factor.clamp(FACTOR_RANGE.0, FACTOR_RANGE.1);
        let changed = match scope {
            ScaleScope::Whole => self.scale_whole(Pass { factor, masses, offsets: true }),
            ScaleScope::Parts(ids) => {
                let ids: HashSet<PartId> = ids.iter().copied().collect();
                let n = self.scale_ids(&ids, Pass { factor, masses, offsets: false });
                self.scale_formulas(factor, |part| ids.contains(&part));
                n
            }
        };
        self.apply_formulas();
        changed
    }

    fn scale_formulas(&mut self, factor: f64, which: impl Fn(PartId) -> bool) {
        for formula in &mut self.formulas {
            if which(formula.target.part) && dim_kind(formula.target.dim) == Kind::Length {
                if let Ok(scaled) = formula.expr.scaled(Kind::Length, factor) {
                    formula.expr = scaled;
                }
            }
        }
    }

    fn scale_whole(&mut self, pass: Pass) -> usize {
        let mut body =
            PodBody { parts: std::mem::take(&mut self.parts), radials: std::mem::take(&mut self.radials), internals: std::mem::take(&mut self.internals) };
        scale_body(&mut body, pass);
        let count = count_body(&body);
        self.parts = body.parts;
        self.radials = body.radials;
        self.internals = body.internals;
        for c in &mut self.configs {
            if let Some(b) = &mut c.ballast {
                *b = Ballast { mass: if pass.masses { b.mass * pass.factor.powi(3) } else { b.mass }, station: b.station * pass.factor };
            }
        }
        self.scale_formulas(pass.factor, |_| true);
        count
    }

    fn scale_ids(&mut self, ids: &HashSet<PartId>, pass: Pass) -> usize {
        let mut count = 0;
        let axial: Vec<(PartId, PartKind)> = self.parts.iter().filter(|p| ids.contains(&p.id)).map(|p| (p.id, part_kind(&p.kind, pass.factor))).collect();
        for (id, kind) in axial {
            if let Some(p) = self.parts.iter_mut().find(|p| p.id == id) {
                part_fields(p, pass);
            }
            self.set_kind(id, kind);
            count += 1;
        }
        let mut radial_parents = Vec::new();
        for r in self.radials.iter_mut().filter(|r| ids.contains(&r.id)) {
            radial_fields(r, pass);
            radial_parents.push(r.parent);
            count += 1;
        }
        let mut internal_parents = Vec::new();
        for i in self.internals.iter_mut().filter(|i| ids.contains(&i.id)) {
            internal_fields(i, pass);
            internal_parents.push(i.parent);
            count += 1;
        }
        radial_parents.sort_unstable();
        radial_parents.dedup();
        for p in radial_parents {
            self.clamp_radials(p);
        }
        internal_parents.sort_unstable();
        internal_parents.dedup();
        for p in internal_parents {
            self.clamp_internals(p);
        }
        let pods: Vec<PartId> = self.radials.iter().filter(|r| r.kind.is_pod() && !ids.contains(&r.id)).map(|r| r.id).collect();
        for pod in pods {
            count += self.with_pod_set(pod, |s| s.scale_ids(ids, pass)).unwrap_or(0);
        }
        count
    }
}

fn count_body(body: &PodBody) -> usize {
    body.parts.len() + body.internals.len() + body.radials.iter().map(|r| 1 + r.kind.pod_body().map_or(0, count_body)).sum::<usize>()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::expr::{Expr, FieldRef, Op};
    use crate::{Dim, NoseShape};

    /// Nose, tube and a 3-fin set on the tube.
    fn example() -> (Rocket, PartId, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.06, tip_radius: 0.0 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.06 }).unwrap();
        let fin =
            r.attach_radial(tube, 0.3, 0.0, 3, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.08, sweep: 0.04, thickness: 0.003 }).unwrap();
        (r, tube, fin)
    }

    #[test]
    fn whole_scale_doubles_lengths_fins_and_mass_overrides() {
        let (mut r, tube, fin) = example();
        r.set_mass_override(tube, Some(0.3));
        let (len, dia) = (r.length(), r.reference_diameter().unwrap());
        let n = r.scale(2.0, &ScaleScope::Whole, true);
        assert_eq!(n, 3);
        assert!((r.length() - 2.0 * len).abs() < 1e-12);
        assert!((r.reference_diameter().unwrap() - 2.0 * dia).abs() < 1e-12);
        match &r.radial(fin).unwrap().kind {
            RadialKind::FinSet { root_chord, tip_chord, span, sweep, thickness } => {
                for (got, want) in [(root_chord, 0.24), (tip_chord, 0.10), (span, 0.16), (sweep, 0.08), (thickness, 0.006)] {
                    assert!((got - want).abs() < 1e-12, "{got} vs {want}");
                }
            }
            other => panic!("{other:?}"),
        }
        assert!((r.radial(fin).unwrap().offset - 0.6).abs() < 1e-12);
        assert!((r.part(tube).unwrap().mass_override.unwrap() - 2.4).abs() < 1e-12);

        let (mut r, tube, _) = example();
        r.set_mass_override(tube, Some(0.3));
        r.scale(2.0, &ScaleScope::Whole, false);
        assert!((r.part(tube).unwrap().mass_override.unwrap() - 0.3).abs() < 1e-12);
    }

    #[test]
    fn formulas_still_evaluate_after_scaling() {
        let (mut r, tube, fin) = example();
        let mul = |a: Expr, b: Expr| Expr::Bin(Box::new(a), Op::Mul, Box::new(b));
        r.set_formula(fin, Dim::RootChord, mul(Expr::Num(2.0), Expr::Ref(FieldRef::new(tube, Dim::Diameter)))).unwrap();
        r.set_formula(fin, Dim::Span, Expr::Num(0.08)).unwrap();
        r.scale(2.0, &ScaleScope::Whole, false);
        assert!((r.dim(fin, Dim::RootChord).unwrap() - 0.24).abs() < 1e-12);
        assert!((r.dim(fin, Dim::Span).unwrap() - 0.16).abs() < 1e-12, "a literal length scales with the rocket");
    }

    #[test]
    fn part_scope_scales_own_dimensions_and_the_stack_follows() {
        let (mut r, tube, fin) = example();
        let (len, off) = (r.length(), r.radial(fin).unwrap().offset);
        assert_eq!(r.scale(2.0, &ScaleScope::Parts(vec![tube]), false), 1);
        assert!((r.length() - (len + 0.5)).abs() < 1e-12);
        assert!((r.part(tube).unwrap().kind.aft_diameter() - 0.12).abs() < 1e-12);
        assert!((r.radial(fin).unwrap().offset - off).abs() < 1e-12, "offsets stay");
        assert_eq!(r.scale(f64::NAN, &ScaleScope::Whole, false), 0);
    }
}
