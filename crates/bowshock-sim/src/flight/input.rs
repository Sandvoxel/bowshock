use crate::drag_table::DragTable;
use crate::vehicle::Vehicle;

/// Gust intensity setting (standard deviation as a fraction of the mean wind speed).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Turbulence {
    /// No gusts.
    #[default]
    Off,
    /// 5 % of mean wind.
    Low,
    /// 10 % of mean wind.
    Med,
    /// 20 % of mean wind.
    High,
}

impl Turbulence {
    /// Gust standard deviation as a fraction of mean wind speed.
    pub fn intensity(self) -> f64 {
        match self {
            Turbulence::Off => 0.0,
            Turbulence::Low => 0.05,
            Turbulence::Med => 0.10,
            Turbulence::High => 0.20,
        }
    }
}

/// One point of a wind profile (see [`Launch::wind_levels`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindPoint {
    /// Altitude above the site (m).
    pub altitude: f64,
    /// Wind speed (m/s).
    pub speed: f64,
    /// Direction the wind blows *from*, clockwise from north (rad).
    pub from: f64,
}

/// Launch site and conditions.
#[derive(Clone, Debug, PartialEq)]
pub struct Launch {
    /// Launch rail length (m).
    pub rail_length: f64,
    /// Rail tilt from vertical (rad).
    pub angle_from_vertical: f64,
    /// Azimuth the rail leans towards, clockwise from north (rad).
    pub direction: f64,
    /// Site altitude above sea level (m).
    pub site_altitude: f64,
    /// Ground temperature at the site (K).
    pub ground_temperature: f64,
    /// Mean wind speed at the ground (m/s).
    pub wind_speed: f64,
    /// Direction the ground wind blows *from*, clockwise from north (rad), meteorological convention.
    pub wind_from: f64,
    /// The wind above the ground, ascending in altitude: the ground wind is the point at 0 m, the wind's east and north
    /// components vary linearly between points and hold above the top one (see [`Launch::wind_at`]). Empty for a wind that
    /// is the same at every altitude.
    pub wind_levels: Vec<WindPoint>,
    /// Gust intensity.
    pub turbulence: Turbulence,
    /// Seed for the gust generator.
    pub seed: u64,
    /// Geodetic latitude of the site (rad): with one, gravity follows the latitude and the Coriolis acceleration of the
    /// rotating Earth acts (see the module docs); `None` is a flat, non-rotating Earth with standard gravity.
    pub latitude: Option<f64>,
    /// Start in the air instead of on the rail: `(altitude above the site m, speed m/s along the launch direction)`.
    pub air_start: Option<(f64, f64)>,
}

impl Default for Launch {
    fn default() -> Self {
        Self {
            rail_length: 1.5,
            angle_from_vertical: 0.0,
            direction: 0.0,
            site_altitude: 0.0,
            ground_temperature: 288.15,
            wind_speed: 0.0,
            wind_from: 0.0,
            wind_levels: Vec::new(),
            turbulence: Turbulence::Off,
            seed: 1,
            latitude: None,
            air_start: None,
        }
    }
}

/// The wind vector `[east, north]` (m/s) of `speed` blowing from `from` (rad clockwise from north).
fn wind_vector(speed: f64, from: f64) -> [f64; 2] {
    [-speed * from.sin(), -speed * from.cos()]
}

/// The wind of `profile` (`(altitude, [east, north])`, ascending, the ground first) at altitude `h`: linear between points,
/// held below the first and above the last.
pub(super) fn wind_in(profile: &[(f64, [f64; 2])], h: f64) -> [f64; 2] {
    let Some(&(_, first)) = profile.first() else { return [0.0; 2] };
    let i = profile.partition_point(|p| p.0 <= h);
    if i == 0 {
        return first;
    }
    if i >= profile.len() {
        return profile[profile.len() - 1].1;
    }
    let ((h0, a), (h1, b)) = (profile[i - 1], profile[i]);
    let f = if h1 > h0 { (h - h0) / (h1 - h0) } else { 1.0 };
    [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])]
}

impl Launch {
    /// The wind profile as `(altitude, [east, north])` points: the ground wind at 0 m, then the levels above it.
    pub(super) fn wind_profile(&self) -> Vec<(f64, [f64; 2])> {
        let mut out = vec![(0.0, wind_vector(self.wind_speed, self.wind_from))];
        out.extend(self.wind_levels.iter().filter(|l| l.altitude > 0.0).map(|l| (l.altitude, wind_vector(l.speed, l.from))));
        out.sort_by(|a, b| a.0.total_cmp(&b.0));
        out
    }

    /// The mean wind `[east, north]` (m/s) at `altitude_agl` above the site (see [`Launch::wind_levels`]).
    pub fn wind_at(&self, altitude_agl: f64) -> [f64; 2] {
        wind_in(&self.wind_profile(), altitude_agl)
    }
}

/// When a recovery device deploys.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Deploy {
    /// At apogee.
    Apogee,
    /// Descending through this altitude above ground (m).
    Altitude(f64),
    /// At the ejection charge of the motor in the device's stage.
    Ejection,
    /// `delay` seconds after launch (the delay field of the device is the time from launch).
    Launch,
    /// On descent (after apogee), once the air speed is below this (m/s).
    SpeedBelow(f64),
    /// Once the rocket tilts more than this from vertical after leaving the rail (rad): the body axis in the rigid-body model,
    /// the velocity in the point-mass model.
    TiltAbove(f64),
}

/// Parachute or other recovery device.
#[derive(Clone, Debug, PartialEq)]
pub struct RecoveryDevice {
    /// Name used for the deploy event and descent-rate summary.
    pub name: String,
    /// Stage index (firing order) that carries the device.
    pub stage: usize,
    /// Drag coefficient times area (m^2).
    pub cd_area: f64,
    /// Deployment trigger.
    pub deploy: Deploy,
    /// Delay after the trigger (s).
    pub delay: f64,
}

/// Which equations of motion fly the rocket.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FlightModel {
    /// Three degrees of freedom: a point mass whose axis always points into the relative wind (perfect weathercocking, no angle
    /// of attack), Niskanen (2009) section 4.2. Kept for regression and fast sweeps.
    PointMass,
    /// Rigid body with quaternion attitude: pitch, yaw and roll dynamics under the normal force at the centre of pressure
    /// about the moving centre of gravity, aerodynamic, body-crossflow and jet damping, thrust along the body axis, and the
    /// moments of inertia of the parts and of the burning propellant. Wind and gusts produce angle of attack and the rocket
    /// weathercocks, oscillating about the relative wind with the damping of the design. Under a deployed parachute the descent
    /// reverts to the point-mass model.
    #[default]
    RigidBody,
}

/// Everything needed to fly a vehicle.
#[derive(Clone, Debug, PartialEq)]
pub struct SimInput {
    /// The vehicle.
    pub vehicle: Vehicle,
    /// Launch conditions.
    pub launch: Launch,
    /// Recovery devices.
    pub recovery: Vec<RecoveryDevice>,
    /// Fine time step used on the rail and under thrust (s); default 0.001. The rigid-body model shortens it whenever the
    /// pitch dynamics need it.
    pub dt: f64,
    /// Hard stop (s); default 600.
    pub max_time: f64,
    /// Equations of motion; default [`FlightModel::RigidBody`].
    pub model: FlightModel,
    /// CFD drag tables, index `k` for the stack with the lowest `k` stages dropped; missing or `None` flies the build-up.
    pub drag_tables: Vec<Option<DragTable>>,
}

impl SimInput {
    /// Input with default step (1 ms), `max_time` (600 s) and the rigid-body model.
    pub fn new(vehicle: Vehicle, launch: Launch, recovery: Vec<RecoveryDevice>) -> Self {
        Self { vehicle, launch, recovery, dt: 0.001, max_time: 600.0, model: FlightModel::default(), drag_tables: Vec::new() }
    }
}
