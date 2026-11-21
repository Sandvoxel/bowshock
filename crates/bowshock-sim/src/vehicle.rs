//! Vehicle description consumed by the aerodynamics and flight code.
//!
//! All lengths are metres, stations are measured from the nose tip increasing aft.

use bowshock_reference::blunt_nose::{self, BluntNose};

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
    /// Nose cone with the given profile; `fore_radius` is the (usually zero) radius of the fore face, and the segment's
    /// `tip_radius` the radius of a spherical tip.
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
    /// Radius of the spherical tip of a nose (0 is a point): the sharp profile of [`BodyKind::Nose`]'s family with its tip
    /// replaced by a sphere tangent to it, and `length` the length of the nose as built, sphere apex to base (see
    /// [`bowshock_reference::blunt_nose`]). A tip radius that has no tangent sphere on the profile gives the sharp nose.
    pub tip_radius: f64,
    /// Surface roughness height (m); 0 is polished.
    pub roughness: f64,
}

impl BodySegment {
    /// Aft station (m).
    pub fn aft_station(&self) -> f64 {
        self.station + self.length
    }

    /// The tip sphere of a blunted nose and the sharp nose it sits on; `None` for a sharp nose and every other kind of segment.
    pub fn blunt_nose(&self) -> Option<BluntNose> {
        let BodyKind::Nose(shape) = self.kind else { return None };
        let r = self.aft_radius;
        if !(self.tip_radius > 0.0 && self.fore_radius == 0.0 && r > 0.0 && self.length > 0.0) {
            return None;
        }
        // The model's nose cones are memoised under their own tags 0..=6: the sim's five families approximate those, so they get
        // theirs.
        let key = [100 + shape as u64, 0, self.length.to_bits(), r.to_bits(), self.tip_radius.to_bits()];
        let sharp = |l: f64, x: f64| r * nose_profile(shape, x / l, 0.0, r, l);
        let limit = blunt_nose::MAX_TIP_FRACTION * r * (1.0 + 1.0e-9);
        (self.tip_radius <= limit)
            .then(|| blunt_nose::solve_cached(key, &sharp, self.length, self.tip_radius, (shape == NoseShape::Conical).then_some(r)))
            .flatten()
    }

    /// Length of the sharp nose a blunted nose sits on; the segment's length otherwise.
    pub fn sharp_length(&self) -> f64 {
        self.blunt_nose().map_or(self.length, |b| b.sharp_length)
    }

    /// Outer radius at distance `x` (0..=length) from the fore face.
    pub fn radius_at(&self, x: f64) -> f64 {
        let l = self.length.max(1e-12);
        if let Some(b) = self.blunt_nose() {
            let r = self.aft_radius;
            let BodyKind::Nose(shape) = self.kind else { return r };
            return b.radius(|s| r * nose_profile(shape, s / b.sharp_length, 0.0, r, b.sharp_length), x.clamp(0.0, l)).clamp(0.0, r);
        }
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FinCrossSection {
    /// Flat slab with square edges.
    Square,
    /// Slab with rounded leading/trailing edges.
    Rounded,
    /// Symmetric airfoil (sharp trailing edge).
    Airfoil,
    /// Sharp-edged double wedge (supersonic wave drag after Ackeret).
    DoubleWedge,
    /// Sharp-edged hexagonal section: a wedge of this fraction of the chord (`0 < f <= 0.5`) at the leading and at the
    /// trailing edge with a flat mid-section (supersonic wave drag after Ackeret).
    Hexagonal(f64),
    /// Sharp-edged biconvex (circular-arc) section (supersonic wave drag after Ackeret).
    Biconvex,
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
    /// Cant angle (rad): each fin turned about its span axis so that its chord swings towards the direction its roll angle
    /// decreases; a positive cant turns the rocket the way the fins' roll angle increases (right-handed about the nose direction,
    /// clockwise seen from behind). Drives the roll of the rigid-body flight model (see [`crate::flight`]); the point-mass model
    /// ignores it. Reference: Barrowman 1967 and Niskanen 2009, section 3.2.3 (roll forcing and damping of canted fins).
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
    /// Roll angle of each fin about the rocket axis (rad): 0 is +Y, increasing clockwise seen from behind, towards +X (plus the
    /// pod's angle for fins on a pod). Empty means `count` fins evenly spaced from 0. The normal force of 1- and 2-fin sets
    /// depends on it (see [`crate::aero::CROSSFLOW_ROLL`]).
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

    /// Estimated diameter of the nozzle exit (m); 0 for a curve without thrust or case diameter. The motor files (RASP, RockSim) carry no nozzle dimensions, so the exit is
    /// derived: the static thrust is `F = p_e A_e (1 + gamma M_e^2) - p_sl A_e` and a hobby nozzle is tuned to sea level (`p_e = p_sl`), so
    /// `A_e = F / (gamma M_e^2 p_sl)` at the average thrust (impulse over burn time) and the exit Mach number [`NOZZLE_EXIT_MACH`], kept between [`NOZZLE_EXIT_FRACTION`]
    /// of the case diameter. This is the area the exhaust covers at the base (the rest of the aft closure is base area the wake acts on, not the whole case).
    pub fn nozzle_exit_diameter(&self) -> f64 {
        const P_SEA_LEVEL: f64 = 101_325.0;
        const GAMMA: f64 = 1.4;
        let burn = self.burn_time();
        let thrust = if burn > 0.0 { self.total_impulse() / burn } else { 0.0 };
        if !(thrust.is_finite() && thrust > 0.0 && self.diameter > 0.0) {
            return 0.0;
        }
        let matched = (4.0 * thrust / (std::f64::consts::PI * GAMMA * NOZZLE_EXIT_MACH * NOZZLE_EXIT_MACH * P_SEA_LEVEL)).sqrt();
        matched.clamp(NOZZLE_EXIT_FRACTION.0 * self.diameter, NOZZLE_EXIT_FRACTION.1 * self.diameter)
    }
}

/// Mach number of the exhaust at the nozzle exit of a hobby motor: its nozzle expands the exhaust to about 2.5 (expansion ratio 6-8, `gamma` 1.2 in the real gas).
pub const NOZZLE_EXIT_MACH: f64 = 2.5;
/// The nozzle exits of the commercial 29-98 mm motor cases are 0.5-0.8 of the case diameter (the nozzle exit diameters RASAero II's example rockets give: 0.55 of a 75 mm case, 0.56 of a
/// 54 mm one, 0.64 of a 98 mm one).
pub const NOZZLE_EXIT_FRACTION: (f64, f64) = (0.5, 0.8);

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
    /// Tilt from vertical (rad) beyond which the motor does not light when its ignition comes: the stage then never fires.
    /// `None` lights whatever the attitude.
    pub tilt_lockout: Option<f64>,
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

/// What a vehicle component came from: a display name, the model part, and per-part overrides.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Label {
    /// Display name of the source part.
    pub name: String,
    /// Model part the component was built from, if any.
    pub part: Option<bowshock_model::PartId>,
    /// Drag coefficient (on the vehicle reference area) that replaces this component's computed drag.
    pub cd_override: Option<f64>,
}

/// Labels for the vehicle's components, by index: `body[i]` labels `Vehicle::body[i]`, and likewise for the
/// other lists. A list shorter than its component list labels the missing entries with [`Label::default`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Labels {
    /// Labels of the body segments.
    pub body: Vec<Label>,
    /// Labels of the fin sets.
    pub fin_sets: Vec<Label>,
    /// Labels of the tube fin sets.
    pub tube_fins: Vec<Label>,
    /// Labels of the protuberances.
    pub protuberances: Vec<Label>,
}

/// Label `i` of `list`, or a default one when the list is short.
fn label_at(list: &[Label], i: usize) -> Label {
    list.get(i).cloned().unwrap_or_default()
}

impl Labels {
    /// Label of body segment `i` (a generated name when missing).
    pub fn body(&self, i: usize) -> Label {
        named(label_at(&self.body, i), || format!("Body {}", i + 1))
    }

    /// Label of fin set `i` (a generated name when missing).
    pub fn fin_set(&self, i: usize) -> Label {
        named(label_at(&self.fin_sets, i), || format!("Fins {}", i + 1))
    }

    /// Label of tube fin set `i` (a generated name when missing).
    pub fn tube_fin(&self, i: usize) -> Label {
        named(label_at(&self.tube_fins, i), || format!("Tube fins {}", i + 1))
    }

    /// Label of protuberance `i` (a generated name when missing).
    pub fn protuberance(&self, i: usize) -> Label {
        named(label_at(&self.protuberances, i), || format!("Protuberance {}", i + 1))
    }

    /// Whether any component carries a drag override.
    pub fn any_cd_override(&self) -> bool {
        [&self.body, &self.fin_sets, &self.tube_fins, &self.protuberances].iter().any(|l| l.iter().any(|l| l.cd_override.is_some()))
    }

    /// Copy with every drag override removed.
    pub fn without_cd_overrides(&self) -> Labels {
        let mut l = self.clone();
        for list in [&mut l.body, &mut l.fin_sets, &mut l.tube_fins, &mut l.protuberances] {
            for label in list {
                label.cd_override = None;
            }
        }
        l
    }
}

fn named(mut label: Label, name: impl FnOnce() -> String) -> Label {
    if label.name.is_empty() {
        label.name = name();
    }
    label
}

/// Keeps the entries of `items` (and the matching entries of `labels`) for which `keep` holds.
fn retain_labelled<T>(items: &mut Vec<T>, labels: &mut Vec<Label>, keep: impl Fn(&T) -> bool) {
    let mut kept_labels = Vec::with_capacity(labels.len());
    let mut i = 0;
    items.retain(|item| {
        let k = keep(item);
        if k {
            if let Some(l) = labels.get(i) {
                kept_labels.push(l.clone());
            }
        }
        i += 1;
        k
    });
    *labels = kept_labels;
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
    /// Names, source parts and overrides of the components (see [`Labels`]).
    pub labels: Labels,
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
        let l = &mut v.labels;
        retain_labelled(&mut v.body, &mut l.body, |b| b.station < station - 1e-9);
        for b in &mut v.body {
            if b.aft_station() > station {
                let new_len = station - b.station;
                // Cut at the new station; the aft radius becomes the profile radius there.
                b.aft_radius = b.radius_at(new_len);
                b.length = new_len;
            }
        }
        retain_labelled(&mut v.fin_sets, &mut l.fin_sets, |f| f.station < station - 1e-9);
        retain_labelled(&mut v.tube_fins, &mut l.tube_fins, |t| t.station < station - 1e-9);
        retain_labelled(&mut v.protuberances, &mut l.protuberances, |p| p.station() < station - 1e-9);
        v
    }

    /// Stage `k` (firing order) as a vehicle of its own, the way it flies once it has separated from the stage above: everything
    /// from the joint it separates at (the aft end of stage `k + 1`) to its own aft end (the end of the vehicle for stage 0; for a
    /// later stage the joint the stage below it left at). Stations are measured from the joint, so the vehicle begins at station 0
    /// with an open face, not a nose. The segment the joint cuts is cut there (a nose cut this way becomes a transition); fin
    /// sets, tube fins and protuberances that start forward of the joint belong to the stage above and are dropped. The one stage
    /// of the result is stage `k` with its dry mass, motor and stations moved to the new origin; the reference diameter is the
    /// whole vehicle's, so coefficients keep their reference area. `None` for the top stage (it separates from nothing) and for
    /// a stage that does not exist.
    pub fn dropped_stage(&self, k: usize) -> Option<Vehicle> {
        let joint = self.stages.get(k + 1)?.aft_station;
        let stage = &self.stages[k];
        let mut v = if k == 0 { self.clone() } else { self.truncated(stage.aft_station) };
        let l = &mut v.labels;
        retain_labelled(&mut v.body, &mut l.body, |b| b.aft_station() > joint + 1e-9);
        for b in &mut v.body {
            if b.station < joint {
                let cut = joint - b.station;
                b.fore_radius = b.radius_at(cut);
                b.length -= cut;
                b.station = joint;
                if matches!(b.kind, BodyKind::Nose(_)) {
                    b.kind = BodyKind::Transition;
                    b.tip_radius = 0.0;
                }
            }
            b.station -= joint;
        }
        retain_labelled(&mut v.fin_sets, &mut l.fin_sets, |f| f.station >= joint - 1e-9);
        for f in &mut v.fin_sets {
            f.station -= joint;
            if let Some((a, b)) = f.root_extent.as_mut() {
                *a -= joint;
                *b -= joint;
            }
        }
        retain_labelled(&mut v.tube_fins, &mut l.tube_fins, |t| t.station >= joint - 1e-9);
        for t in &mut v.tube_fins {
            t.station -= joint;
        }
        retain_labelled(&mut v.protuberances, &mut l.protuberances, |p| p.station() >= joint - 1e-9);
        for p in &mut v.protuberances {
            match p {
                Protuberance::LaunchLug { station, .. } | Protuberance::RailButton { station, .. } => *station -= joint,
            }
        }
        v.stages = vec![StageMass {
            dry: MassProps { cg: stage.dry.cg - joint, ..stage.dry },
            fore_station: 0.0,
            aft_station: stage.aft_station - joint,
            motor: stage.motor.clone().map(|m| StageMotor { mount_aft_station: m.mount_aft_station - joint, ..m }),
            separation: stage.separation,
        }];
        Some(v)
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
            let s = BodySegment { station: 0.0, length: 0.2, fore_radius: 0.0, aft_radius: 0.03, kind: BodyKind::Nose(shape), tip_radius: 0.0, roughness: 0.0 };
            assert!(s.radius_at(0.0).abs() < 1e-9, "{shape:?}");
            assert!((s.radius_at(0.2) - 0.03).abs() < 1e-9, "{shape:?}");
            assert!(s.radius_at(0.1) > 0.0);
        }
    }

    /// Without a tip radius the segment is the legacy profile bit for bit (the fast path never touches the blunting).
    #[test]
    fn a_sharp_nose_is_the_legacy_profile_exactly() {
        for shape in [NoseShape::Conical, NoseShape::TangentOgive, NoseShape::VonKarman, NoseShape::Elliptical, NoseShape::Parabolic] {
            let s = BodySegment { station: 0.0, length: 0.2, fore_radius: 0.0, aft_radius: 0.03, kind: BodyKind::Nose(shape), tip_radius: 0.0, roughness: 0.0 };
            assert!(s.blunt_nose().is_none() && s.sharp_length() == 0.2);
            for i in 0..=20 {
                let x = 0.2 * f64::from(i) / 20.0;
                let legacy = 0.0 + (0.03 - 0.0) * nose_profile(shape, (x / 0.2).clamp(0.0, 1.0), 0.0, 0.03, 0.2);
                assert_eq!(s.radius_at(x), legacy, "{shape:?} {x}");
            }
        }
    }

    /// A blunted segment starts at a point, ends at the base radius, is as long as built and its sharp nose is longer.
    #[test]
    fn blunted_noses_hit_endpoints_and_keep_the_built_length() {
        for shape in [NoseShape::Conical, NoseShape::TangentOgive, NoseShape::VonKarman, NoseShape::Parabolic] {
            let s =
                BodySegment { station: 0.0, length: 0.2, fore_radius: 0.0, aft_radius: 0.03, kind: BodyKind::Nose(shape), tip_radius: 0.01, roughness: 0.0 };
            let b = s.blunt_nose().unwrap_or_else(|| panic!("{shape:?} takes a tip sphere"));
            assert!((b.length() - 0.2).abs() < 1e-11 && s.sharp_length() > 0.2, "{shape:?}");
            assert_eq!(s.radius_at(0.0), 0.0);
            assert!((s.radius_at(0.2) - 0.03).abs() < 1e-12, "{shape:?}");
            // a round tip: the radius leaves the axis faster than the sharp profile does
            assert!(s.radius_at(0.002) > 0.0 && s.radius_at(0.002) < 0.015);
            // and a sphere larger than 0.9 of the base radius is refused (the nose stays sharp)
            let big = BodySegment { tip_radius: 0.03, ..s.clone() };
            assert!(big.blunt_nose().is_none());
        }
    }
}
