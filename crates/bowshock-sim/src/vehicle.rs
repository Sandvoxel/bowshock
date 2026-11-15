//! Vehicle description consumed by the aerodynamics and flight code.
//!
//! All lengths are metres, stations are measured from the nose tip increasing aft.

/// Nose cone profile family.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoseShape {
    /// Straight cone.
    Conical,
    /// Tangent ogive.
    TangentOgive,
    /// Von Karman (Haack LD) minimum-wave-drag shape.
    VonKarman,
    /// Half ellipse (rounded tip).
    Elliptical,
    /// Parabolic series with K = 0.5.
    Parabolic,
}

/// What a body segment is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyKind {
    /// Nose cone with the given profile; `fore_radius` is the (usually zero) tip radius.
    Nose(NoseShape),
    /// Cylindrical or constant-radius tube.
    Tube,
    /// Conical transition (shoulder or boat-tail).
    Transition,
}

/// One axisymmetric body segment.
#[derive(Clone, Debug, PartialEq)]
pub struct BodySegment {
    /// Station of the fore face from the nose tip (m).
    pub station: f64,
    /// Segment length (m).
    pub length: f64,
    /// Outer radius at the fore face (m).
    pub fore_radius: f64,
    /// Outer radius at the aft face (m).
    pub aft_radius: f64,
    /// Segment type.
    pub kind: BodyKind,
    /// Surface roughness height (m); 0 is polished.
    pub roughness: f64,
}

impl BodySegment {
    /// Aft station (m).
    pub fn aft_station(&self) -> f64 {
        self.station + self.length
    }

    /// Outer radius at distance `x` (0..=length) from the fore face.
    pub fn radius_at(&self, x: f64) -> f64 {
        let l = self.length.max(1e-12);
        let u = (x / l).clamp(0.0, 1.0);
        let s = match self.kind {
            BodyKind::Nose(shape) => nose_profile(shape, u, self.fore_radius, self.aft_radius, self.length),
            _ => u,
        };
        self.fore_radius + (self.aft_radius - self.fore_radius) * s
    }
}

/// Normalised profile `s(u)` in 0..1 for `u = x/L` from the tip. For the ogive the exact
/// tangent-ogive radius depends on the blunt radius ratio, hence the extra arguments.
fn nose_profile(shape: NoseShape, u: f64, r0: f64, r1: f64, len: f64) -> f64 {
    let dr = (r1 - r0).abs().max(1e-12);
    match shape {
        NoseShape::Conical => u,
        NoseShape::TangentOgive => {
            let rho = (dr * dr + len * len) / (2.0 * dr);
            let x = u * len;
            (((rho * rho - (len - x) * (len - x)).max(0.0)).sqrt() + dr - rho) / dr
        }
        NoseShape::VonKarman => {
            let th = (1.0 - 2.0 * u).clamp(-1.0, 1.0).acos();
            let th1 = std::f64::consts::PI;
            let f = |t: f64| (t - 0.5 * (2.0 * t).sin()).max(0.0).sqrt();
            f(th) / f(th1)
        }
        NoseShape::Elliptical => (1.0 - (1.0 - u) * (1.0 - u)).max(0.0).sqrt(),
        NoseShape::Parabolic => (2.0 * u - 0.5 * u * u) / 1.5,
    }
}

/// Fin planform family.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FinPlanform {
    /// Straight-edged trapezoid defined by root/tip chord, span and sweep.
    Trapezoidal,
    /// Semi-elliptical fin (`root_chord` x `span`; tip chord and sweep ignored).
    Elliptical,
}

/// Fin edge cross-section.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FinCrossSection {
    /// Flat slab with square edges.
    Square,
    /// Slab with rounded leading/trailing edges.
    Rounded,
    /// Symmetric airfoil (sharp trailing edge).
    Airfoil,
}

/// A set of identical fins evenly spaced around the body.
#[derive(Clone, Debug, PartialEq)]
pub struct FinSet {
    /// Number of fins.
    pub count: u32,
    /// Station of the root leading edge from the tip (m).
    pub station: f64,
    /// Body radius under the fins (m).
    pub body_radius: f64,
    /// Root chord (m).
    pub root_chord: f64,
    /// Tip chord (m).
    pub tip_chord: f64,
    /// Exposed span, root to tip (m).
    pub span: f64,
    /// Leading-edge offset of the tip relative to the root leading edge (m); may be negative.
    pub sweep: f64,
    /// Fin thickness (m).
    pub thickness: f64,
    /// Edge cross-section.
    pub cross_section: FinCrossSection,
    /// Cant angle (rad): each fin turned about its span axis, positive for a positive (right-handed about the nose direction)
    /// roll moment. Drives the roll of the rigid-body flight model (see [`crate::flight`]); the point-mass model ignores it.
    pub cant: f64,
    /// Surface roughness height (m).
    pub roughness: f64,
    /// Planform family.
    pub planform: FinPlanform,
    /// Span-averaged cosine of the mid-chord sweep, when the planform is not the trapezoid itself (a free-form fin flown as
    /// its equivalent trapezoid): the subsonic lift slope uses it. `None` takes it from the trapezoid.
    pub mid_chord_cos: Option<f64>,
    /// Stations (m) of the real root chord's ends when they differ from the trapezoid's (a free-form fin): fin-fin
    /// interference compares root chords. `None` is `station .. station + root_chord`.
    pub root_extent: Option<(f64, f64)>,
    /// Roll angle of each fin about the rocket axis (rad): 0 is +Y, increasing towards +X (plus the pod's angle for fins on
    /// a pod). Empty means `count` fins evenly spaced from 0. The normal force of 1- and 2-fin sets depends on it (see
    /// [`crate::aero::CROSSFLOW_ROLL`]).
    pub angles: Vec<f64>,
    /// Identifier of the body the fins stand on. Fin sets on the same body whose root chords overlap count together for
    /// fin-fin interference (5 or more fins lose lift, [`crate::aero::fin_fin_factor`]); sets with different ids never
    /// interfere.
    pub parent: u64,
    /// Number of identical bodies the set is spread over (the copies of a pod; 1 on the core): `count / bodies` fins stand on
    /// each of them.
    pub bodies: u32,
}

impl FinSet {
    /// Area of one fin (m^2).
    pub fn fin_area(&self) -> f64 {
        match self.planform {
            FinPlanform::Trapezoidal => 0.5 * (self.root_chord + self.tip_chord) * self.span,
            FinPlanform::Elliptical => std::f64::consts::FRAC_PI_4 * self.root_chord * self.span,
        }
    }
}

/// A set of open tubes standing against the body with air flowing through them (see [`crate::tube_fins`]): the
/// aerodynamics of an empty body tube on the outside of a rocket.
#[derive(Clone, Debug, PartialEq)]
pub struct TubeFinSet {
    /// Number of tubes (the copies of the pod).
    pub count: u32,
    /// Station of the tubes' fore ends from the tip (m).
    pub station: f64,
    /// Tube length, the "chord" of the ring airfoil (m).
    pub length: f64,
    /// Outer radius of a tube (m).
    pub outer_radius: f64,
    /// Inner radius of a tube (m); the wall is `outer_radius - inner_radius`.
    pub inner_radius: f64,
    /// Radius of the body surface the tubes stand against (m).
    pub body_radius: f64,
    /// Clear distance from that surface to the tubes' outer surface (m); 0 for tubes touching the body.
    pub gap: f64,
    /// Surface roughness height (m).
    pub roughness: f64,
}

/// Surface protuberance that adds drag but no lift.
#[derive(Clone, Debug, PartialEq)]
pub enum Protuberance {
    /// Launch lug (a short tube along the body).
    LaunchLug {
        /// Station of the forward end (m).
        station: f64,
        /// Length (m).
        length: f64,
        /// Outer diameter (m).
        outer_diameter: f64,
        /// Inner diameter (m).
        inner_diameter: f64,
    },
    /// Rail button (a short cylinder standing off the body).
    RailButton {
        /// Station (m).
        station: f64,
        /// Height above the body (m).
        height: f64,
        /// Diameter (m).
        diameter: f64,
    },
}

impl Protuberance {
    /// Station of the protuberance (m).
    pub fn station(&self) -> f64 {
        match *self {
            Protuberance::LaunchLug { station, .. } | Protuberance::RailButton { station, .. } => station,
        }
    }
}

/// Mass, centre of gravity and moments of inertia of a lump.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MassProps {
    /// Mass (kg).
    pub mass: f64,
    /// Centre of gravity station from the tip (m).
    pub cg: f64,
    /// Longitudinal (pitch/yaw) moment of inertia about a transverse axis through the lump's own CG (kg m^2). If the whole
    /// flying stack has none (every lump zero) the rigid-body flight model assumes a uniform rod over the stack's length.
    pub i_long: f64,
    /// Rotational (roll) moment of inertia about the long axis (kg m^2). If the whole flying stack has none the rigid-body flight
    /// model assumes a solid cylinder of the reference diameter.
    pub i_rot: f64,
}

/// What a staging event (a motor ignition or a stage separation) waits for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Trigger {
    /// The burnout of the stage the event belongs to (separation) or of the stage below it (ignition).
    Burnout,
    /// The separation of the stage below (ignition only; on a separation it means [`Trigger::Burnout`]).
    Separation,
    /// Launch: the delay is the time from launch.
    Launch,
    /// The rocket climbing through this altitude above ground (m). It is armed once the stage (separation) or the stage below
    /// (ignition) has lit, and it only fires on the way up: a rocket that never reaches the altitude never fires it.
    Altitude(f64),
}

/// A [`Trigger`] and a delay after it (s).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Timing {
    /// What is waited for.
    pub trigger: Trigger,
    /// Seconds after the trigger.
    pub delay: f64,
}

impl Timing {
    /// At launch with no delay.
    pub const LAUNCH: Timing = Timing { trigger: Trigger::Launch, delay: 0.0 };
    /// At the burnout of the stage in question with no delay: the default separation.
    pub const BURNOUT: Timing = Timing { trigger: Trigger::Burnout, delay: 0.0 };

    /// `trigger` plus `delay` seconds.
    pub const fn new(trigger: Trigger, delay: f64) -> Timing {
        Timing { trigger, delay }
    }
}

/// Motor data: a thrust curve plus the masses/dimensions needed for mass and CG.
#[derive(Clone, Debug, PartialEq)]
pub struct ThrustCurve {
    /// `(time since ignition s, thrust N)` points, sorted by time; thrust is linearly
    /// interpolated and zero outside the table.
    pub points: Vec<(f64, f64)>,
    /// Propellant mass (kg).
    pub propellant_mass: f64,
    /// Loaded motor mass (kg).
    pub total_mass: f64,
    /// Motor length (m).
    pub length: f64,
    /// Motor diameter (m).
    pub diameter: f64,
}

impl ThrustCurve {
    /// Thrust at `t` seconds after ignition (N).
    pub fn thrust(&self, t: f64) -> f64 {
        let p = &self.points;
        if p.len() < 2 || t < p[0].0 || t > p[p.len() - 1].0 {
            return 0.0;
        }
        let i = p.partition_point(|q| q.0 <= t).clamp(1, p.len() - 1);
        let (t0, f0) = p[i - 1];
        let (t1, f1) = p[i];
        if t1 <= t0 {
            return f1;
        }
        f0 + (f1 - f0) * (t - t0) / (t1 - t0)
    }

    /// Burn time (s): last point of the table.
    pub fn burn_time(&self) -> f64 {
        self.points.last().map_or(0.0, |p| p.0)
    }

    /// Total impulse (N s) by trapezoidal integration.
    pub fn total_impulse(&self) -> f64 {
        self.impulse_until(f64::INFINITY)
    }

    /// Impulse delivered up to `t` seconds after ignition (N s).
    pub fn impulse_until(&self, t: f64) -> f64 {
        let mut sum = 0.0;
        for w in self.points.windows(2) {
            let (t0, f0) = w[0];
            let (t1, f1) = w[1];
            if t <= t0 {
                break;
            }
            let te = t.min(t1);
            let fe = if t1 > t0 { f0 + (f1 - f0) * (te - t0) / (t1 - t0) } else { f1 };
            sum += 0.5 * (f0 + fe) * (te - t0);
        }
        sum
    }

    /// Motor mass `t` seconds after ignition (kg): propellant burns in proportion to impulse delivered.
    pub fn mass(&self, t: f64) -> f64 {
        let total = self.total_impulse();
        if total <= 0.0 || t <= 0.0 {
            return self.total_mass;
        }
        let burned = (self.impulse_until(t) / total).clamp(0.0, 1.0);
        self.total_mass - self.propellant_mass * burned
    }

    /// Distance of the motor CG forward of its aft end (m): the motor's middle.
    pub fn cg(&self) -> f64 {
        0.5 * self.length
    }
}

/// Motor installed in a stage.
#[derive(Clone, Debug, PartialEq)]
pub struct StageMotor {
    /// Thrust curve and masses.
    pub curve: ThrustCurve,
    /// Station of the motor's aft end (nozzle exit plane) from the tip (m).
    pub mount_aft_station: f64,
    /// When the motor ignites. For the first stage that flies, `Burnout` and `Separation` mean launch.
    pub ignition: Timing,
    /// Ejection-charge delay after burnout (s), `None` for no ejection charge.
    pub ejection_delay: Option<f64>,
}

impl StageMotor {
    /// Station of the motor CG from the tip (m).
    pub fn cg_station(&self) -> f64 {
        self.mount_aft_station - self.curve.cg()
    }
}

/// One stage of the vehicle. `Vehicle::stages` is in **firing order**: `stages[0]` is the lowest
/// stage (ignites at launch, separates first) and the last element is the sustainer/top stage.
#[derive(Clone, Debug, PartialEq)]
pub struct StageMass {
    /// Dry mass of everything in this stage except its motor.
    pub dry: MassProps,
    /// Forward station of the stage (m).
    pub fore_station: f64,
    /// Aft station of the stage (m).
    pub aft_station: f64,
    /// Motor, if any.
    pub motor: Option<StageMotor>,
    /// When this stage separates from the stage above it (ignored for the top stage).
    pub separation: Timing,
}

/// Complete vehicle for simulation.
#[derive(Clone, Debug, PartialEq)]
pub struct Vehicle {
    /// Reference diameter for coefficients (m), usually the largest body diameter.
    pub reference_diameter: f64,
    /// Body segments, sorted fore to aft.
    pub body: Vec<BodySegment>,
    /// Fin sets.
    pub fin_sets: Vec<FinSet>,
    /// Tube fin sets (open tubes standing beside the body, see [`TubeFinSet`]).
    pub tube_fins: Vec<TubeFinSet>,
    /// Lugs and rail buttons.
    pub protuberances: Vec<Protuberance>,
    /// Stages in firing order (see [`StageMass`]). Empty means a massless, motorless vehicle.
    pub stages: Vec<StageMass>,
}

impl Vehicle {
    /// Reference area (m^2).
    pub fn reference_area(&self) -> f64 {
        std::f64::consts::FRAC_PI_4 * self.reference_diameter * self.reference_diameter
    }

    /// Total length from the tip to the aft end of the last body segment (m).
    pub fn length(&self) -> f64 {
        self.body.iter().map(BodySegment::aft_station).fold(0.0, f64::max)
    }

    /// Copy of the vehicle with everything aft of `station` removed (used after stage separation).
    /// The segment straddling the cut is truncated; fin sets and protuberances starting at or
    /// aft of the cut are dropped.
    pub fn truncated(&self, station: f64) -> Vehicle {
        let mut v = self.clone();
        v.body.retain(|b| b.station < station - 1e-9);
        for b in &mut v.body {
            if b.aft_station() > station {
                let new_len = station - b.station;
                // Cut at the new station; the aft radius becomes the profile radius there.
                b.aft_radius = b.radius_at(new_len);
                b.length = new_len;
            }
        }
        v.fin_sets.retain(|f| f.station < station - 1e-9);
        v.tube_fins.retain(|t| t.station < station - 1e-9);
        v.protuberances.retain(|p| p.station() < station - 1e-9);
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn curve() -> ThrustCurve {
        ThrustCurve { points: vec![(0.0, 0.0), (0.1, 100.0), (1.1, 100.0), (1.2, 0.0)], propellant_mass: 0.1, total_mass: 0.25, length: 0.2, diameter: 0.029 }
    }

    #[test]
    fn impulse_and_mass() {
        let c = curve();
        assert!((c.total_impulse() - 110.0).abs() < 1e-9);
        assert!((c.burn_time() - 1.2).abs() < 1e-12);
        assert!((c.thrust(0.6) - 100.0).abs() < 1e-9);
        assert_eq!(c.thrust(2.0), 0.0);
        assert!((c.mass(-1.0) - 0.25).abs() < 1e-12);
        assert!((c.mass(5.0) - 0.15).abs() < 1e-12);
        assert!(c.mass(0.6) < 0.25 && c.mass(0.6) > 0.15);
    }

    #[test]
    fn nose_profiles_hit_endpoints() {
        for shape in [NoseShape::Conical, NoseShape::TangentOgive, NoseShape::VonKarman, NoseShape::Elliptical, NoseShape::Parabolic] {
            let s = BodySegment { station: 0.0, length: 0.2, fore_radius: 0.0, aft_radius: 0.03, kind: BodyKind::Nose(shape), roughness: 0.0 };
            assert!(s.radius_at(0.0).abs() < 1e-9, "{shape:?}");
            assert!((s.radius_at(0.2) - 0.03).abs() < 1e-9, "{shape:?}");
            assert!(s.radius_at(0.1) > 0.0);
        }
    }
}
