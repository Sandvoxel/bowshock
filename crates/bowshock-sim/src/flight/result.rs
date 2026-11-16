use crate::roll::RollResonance;

/// Flight event kinds.
#[derive(Clone, Debug, PartialEq)]
pub enum EventKind {
    /// A motor ignited (see [`Event::stage`]).
    Ignition,
    /// A motor's ignition came while the rocket leaned past its tilt lockout: it never lights (see [`Event::stage`]).
    IgnitionInhibited,
    /// The rocket left the rail.
    RailExit,
    /// A motor burned out.
    Burnout,
    /// A stage separated (the dropped stage is [`Event::stage`]).
    Separation,
    /// Apogee.
    Apogee,
    /// A recovery device deployed (its name).
    Deploy(String),
    /// Ground contact.
    Landing,
}

/// A timed flight event.
#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    /// Time (s).
    pub t: f64,
    /// What happened.
    pub kind: EventKind,
    /// Altitude above ground (m).
    pub altitude: f64,
    /// Stage involved for ignition/burnout/separation.
    pub stage: Option<usize>,
}

/// One recorded instant of the flight. All values are SI (m, s, kg, N, Pa, K, rad); stations are from the nose tip.
///
/// The flight data set: position and velocity, aerodynamics (Mach, dynamic pressure, Reynolds
/// number, the drag-coefficient build-up), forces, mass and balance, the atmosphere and the recovery device state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sample {
    /// Time (s).
    pub t: f64,
    /// Position (m): x east, y north, z up above the pad.
    pub position: [f64; 3],
    /// Ground-relative velocity (m/s).
    pub velocity: [f64; 3],
    /// Altitude above ground (m).
    pub altitude: f64,
    /// Altitude above sea level (m): the ground altitude of the site plus [`altitude`](Self::altitude).
    pub altitude_msl: f64,
    /// Vertical velocity (m/s).
    pub vertical_velocity: f64,
    /// Horizontal ground speed (m/s).
    pub horizontal_velocity: f64,
    /// Ground speed magnitude (m/s).
    pub speed: f64,
    /// Speed relative to the air, wind and gusts included (m/s).
    pub air_speed: f64,
    /// Distance from the pad measured along the ground (m).
    pub ground_distance: f64,
    /// Flight-path angle above the horizontal (rad): `+pi/2` straight up, `-pi/2` straight down.
    pub flight_path_angle: f64,
    /// Mach number relative to the air.
    pub mach: f64,
    /// Magnitude of the kinematic acceleration (m/s^2).
    pub acceleration: f64,
    /// Signed vertical acceleration (m/s^2).
    pub vertical_acceleration: f64,
    /// Thrust (N).
    pub thrust: f64,
    /// Drag force (N).
    pub drag: f64,
    /// Total airframe drag coefficient (reference-area based): the sum of the parts below, faded out as canopies fill. The
    /// recovery devices' share is [`cd_recovery`](Self::cd_recovery), kept apart because it is hundreds of times larger.
    pub cd: f64,
    /// Airframe skin-friction drag coefficient.
    pub cd_friction: f64,
    /// Airframe pressure (wave) drag coefficient.
    pub cd_pressure: f64,
    /// Airframe base drag coefficient.
    pub cd_base: f64,
    /// Fin drag coefficient.
    pub cd_fins: f64,
    /// Launch lug and rail button drag coefficient.
    pub cd_protuberances: f64,
    /// Share of the drag coefficient carried by deployed recovery devices (`Cd A / A_ref`).
    pub cd_recovery: f64,
    /// Drag area `Cd A` of the deployed recovery devices, canopy inflation included (m^2).
    pub recovery_cd_area: f64,
    /// Total mass (kg).
    pub mass: f64,
    /// Propellant mass remaining in the motors of the stages still flying (kg).
    pub propellant_mass: f64,
    /// Centre of gravity station (m).
    pub cg: f64,
    /// Centre of pressure station at the current Mach and zero angle of attack (m).
    pub cp: f64,
    /// Static margin in calibers at the current Mach, zero angle of attack and current CG.
    pub stability_margin: f64,
    /// Angle between the rocket's axis and the air velocity (rad), 0 flying nose first, `pi` tail first. In the rigid-body model it
    /// is the real angle of attack (wind, gusts and the turn of the flight path make it non-zero). In the point-mass model the
    /// rocket weathercocks perfectly, so it is zero in free flight and non-zero only on the rail (the rail fixes the axis) and
    /// below 1 m/s of air speed.
    pub angle_of_attack: f64,
    /// Horizontal wind speed at the rocket, gusts included (m/s).
    pub wind_speed: f64,
    /// Dynamic pressure `q = 1/2 rho v^2` of the air-relative flow (Pa).
    pub dynamic_pressure: f64,
    /// Air density (kg/m^3).
    pub air_density: f64,
    /// Air temperature (K).
    pub air_temperature: f64,
    /// Static air pressure (Pa).
    pub air_pressure: f64,
    /// Speed of sound (m/s).
    pub speed_of_sound: f64,
    /// Reynolds number based on the length of the flying stack (`rho v L / mu`).
    pub reynolds: f64,
    /// Rate of turn of the rocket's axis, the magnitude of the pitch and yaw rates (rad/s). Zero in the point-mass model, on the
    /// rail and under a parachute.
    pub pitch_rate: f64,
    /// Roll rate about the axis (rad/s), right-handed about the nose direction: positive is clockwise seen from behind, the way a
    /// fin's roll angle increases and positive fin cant turns the rocket. Zero without cant, in the point-mass model, on the rail
    /// and under a parachute.
    pub roll_rate: f64,
    /// Elevation of the rocket's axis above the horizontal (rad): `+pi/2` nose straight up, `-pi/2` nose straight down. It lags the
    /// flight-path angle by the angle of attack.
    pub attitude_angle: f64,
    /// Natural frequency of the pitch oscillation at this instant (rad/s), `sqrt(q A_ref CNalpha (x_cp - x_cg) / I)` (see
    /// [`crate::roll::pitch_frequency`]); zero where the rocket does not oscillate: in the point-mass model, on the rail, under a
    /// parachute, or when the CP is not aft of the CG. The spin rate [`roll_rate`](Self::roll_rate) is compared against it
    /// (roll-pitch resonance, [`Summary::roll_resonance`]).
    pub pitch_frequency: f64,
    /// Cant of the first fin group under active roll control (rad; see [`crate::vehicle::RollControl`]); 0 without one.
    pub control_cant: f64,
}

/// Recorded flight.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Trajectory {
    /// Decimated samples (about 2000), always including the start, end and event instants.
    pub samples: Vec<Sample>,
    /// Events in time order.
    pub events: Vec<Event>,
}

/// The least static margin of a flight (see [`Summary::min_margin`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MinMargin {
    /// Static margin (calibers) at zero angle of attack with the CG of that moment.
    pub cal: f64,
    /// Mach number where it occurs.
    pub mach: f64,
    /// Time of flight where it occurs (s).
    pub time: f64,
}

/// Flight summary values.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Summary {
    /// Apogee altitude above ground (m).
    pub apogee: f64,
    /// Time of apogee (s).
    pub time_to_apogee: f64,
    /// Apogee time minus final burnout time (s): the ideal ejection delay.
    pub optimum_delay: f64,
    /// Maximum ground speed before apogee (m/s).
    pub max_velocity: f64,
    /// Maximum Mach number before apogee.
    pub max_mach: f64,
    /// Maximum kinematic acceleration before apogee (in g).
    pub max_acceleration_g: f64,
    /// Speed at rail exit (m/s); 0 if the rocket never left the rail.
    pub rail_exit_velocity: f64,
    /// Static margin at rail exit (cal), at the angle of attack the rocket has there in the rigid-body model (the CP moves forward
    /// with it, so wind lowers it) and at zero in the point-mass model.
    pub stability_off_rail_cal: f64,
    /// `(device name, descent rate m/s)`: vertical speed just before the next device deploys or the ground is hit.
    pub descent_rates: Vec<(String, f64)>,
    /// Speed at ground contact (m/s).
    pub ground_hit_velocity: f64,
    /// Time from ignition to landing (s).
    pub flight_time: f64,
    /// `(distance m, bearing deg clockwise from north)` of the landing point from the pad.
    pub landing: (f64, f64),
    /// Landing point `[east, north]` (m).
    pub landing_position: [f64; 2],
    /// Roll rate of the largest magnitude over the flight (rad/s), positive clockwise seen from behind, the way positive fin cant
    /// turns the rocket; 0 for a rocket without canted fins or in the point-mass model.
    pub max_roll_rate: f64,
    /// Roll rate at the last burnout (rad/s, same sense).
    pub roll_rate_at_burnout: f64,
    /// Roll rate at apogee (rad/s, same sense).
    pub roll_rate_at_apogee: f64,
    /// Where the spin rate meets the pitch natural frequency, if it does (see [`crate::roll`]); `None` when it keeps clear.
    pub roll_resonance: Option<RollResonance>,
    /// Whether a 3D stability solve ([`LiftTable`](crate::lift_table::LiftTable)) set the normal force and CP in some stage of the
    /// flight, so the margins above and below are the solve's rather than Extended's.
    pub stability_from_lift: bool,
    /// The least static margin between rail exit and apogee, `None` if the rocket never left the rail.
    pub min_margin: Option<MinMargin>,
}

/// Result of [`simulate`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlightResult {
    /// Time history and events.
    pub trajectory: Trajectory,
    /// Headline numbers.
    pub summary: Summary,
}
