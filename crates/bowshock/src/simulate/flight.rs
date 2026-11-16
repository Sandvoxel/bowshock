//! A flight of the Simulate screen: a named set of launch conditions plus a motor choice, and the result of its last run.
//!
//! The definition of a flight (name, [`Spec`]) is stored in the document ([`bowshock_model::FlightDef`], so it saves, loads and
//! undoes with the rocket); the screen's [`State`](super::State) mirrors it and adds what is never saved: the last run, the compare
//! slot, a dispersion study. A flight never owns a copy of the rocket: [`prepare`] builds the simulation input from the document
//! *at run time*, with the flight's motor and delay swapped into a clone, so a flight always flies the rocket as it is now. A run
//! remembers the [`Spec`] and the document revision it flew; when either differs from the flight's current one the result is
//! *stale* (the screen draws its values in `ink-faint` until it is run again).

use std::sync::Arc;
use std::time::Duration;

use bowshock_model::{AirStart, Conditions, ConfigId, DragSource, FlightDef, FlightMotor, Location, MotorMount, MotorRef, Rocket, WindLevel};
use bowshock_sim::from_model::{self, Phase};
use bowshock_sim::{EventKind, FlightResult, Launch, SimInput, Turbulence, WindPoint};

use crate::app::Ctx;
use crate::widgets::chip::Severity;

/// Identifies a flight within the screen.
pub type FlightId = u64;

/// Rail exit speed (m/s) a flight should have to be stable off the rail (the kit's `ok` chip, "≥ 15").
pub const MIN_RAIL_EXIT: f64 = 15.0;
/// Ground-hit speed (m/s) a landing must stay under (20 ft/s, the kit's "< 6.1").
pub const MAX_GROUND_HIT: f64 = 6.1;
/// Static margin (calibers) off the rail that counts as stable.
pub const MIN_OFF_RAIL_CAL: f64 = 1.0;
/// Static margin (calibers) the whole ascent should keep at subsonic Mach numbers (RASAero II recommends 1.0 calibers subsonic).
pub const MIN_MARGIN_SUBSONIC_CAL: f64 = 1.0;
/// Static margin (calibers) the whole ascent should keep from [`TRANSONIC_MACH`] up (RASAero II recommends 2.0 calibers at
/// transonic and supersonic Mach numbers).
pub const MIN_MARGIN_SUPERSONIC_CAL: f64 = 2.0;
/// The Mach number from which the supersonic margin guidance applies (the start of the transonic range).
pub const TRANSONIC_MACH: f64 = 0.8;
/// What a result or channel label reads when the 3D stability solve, not Extended, gave the stability.
pub const CFD_3D_TAG: &str = "· CFD (3D)";

/// The static margin (calibers) the ascent should keep at `mach`: [`MIN_MARGIN_SUBSONIC_CAL`] below [`TRANSONIC_MACH`],
/// [`MIN_MARGIN_SUPERSONIC_CAL`] from it up. This is the guidance of the RASAero II manual ("Marginal Stability and Unstable Rocket
/// Warnings": a recommended stability margin of 1.0 calibers at subsonic Mach numbers and 2.0 calibers at transonic and supersonic
/// ones, checked below 5° angle of attack), <https://www.rasaero.com/dloads/RASAero%20II%20Users%20Manual.pdf>.
pub fn required_margin_cal(mach: f64) -> f64 {
    if mach >= TRANSONIC_MACH {
        MIN_MARGIN_SUPERSONIC_CAL
    } else {
        MIN_MARGIN_SUBSONIC_CAL
    }
}

/// How far the ejection delay may be from the optimum before the screen warns (s).
pub const DELAY_TOLERANCE: f64 = 0.5;

/// What the flight burns.
#[derive(Debug, Clone, PartialEq)]
pub enum MotorChoice {
    /// Whatever the document's active configuration holds, now.
    Active,
    /// A named configuration of the document.
    Config(ConfigId),
    /// This motor in the first mount of the first stage, everything else as the active configuration has it.
    Motor(MotorRef),
}

impl From<&FlightMotor> for MotorChoice {
    fn from(motor: &FlightMotor) -> Self {
        match motor {
            FlightMotor::Active => MotorChoice::Active,
            FlightMotor::Config(id) => MotorChoice::Config(*id),
            FlightMotor::Motor(m) => MotorChoice::Motor(m.clone()),
        }
    }
}

impl From<&MotorChoice> for FlightMotor {
    fn from(choice: &MotorChoice) -> Self {
        match choice {
            MotorChoice::Active => FlightMotor::Active,
            MotorChoice::Config(id) => FlightMotor::Config(*id),
            MotorChoice::Motor(m) => FlightMotor::Motor(m.clone()),
        }
    }
}

/// The stored gust setting of `t`.
fn stored_turbulence(t: Turbulence) -> bowshock_model::Turbulence {
    match t {
        Turbulence::Off => bowshock_model::Turbulence::Off,
        Turbulence::Low => bowshock_model::Turbulence::Low,
        Turbulence::Med => bowshock_model::Turbulence::Medium,
        Turbulence::High => bowshock_model::Turbulence::High,
    }
}

/// The simulator's gust setting of `t`.
fn sim_turbulence(t: bowshock_model::Turbulence) -> Turbulence {
    match t {
        bowshock_model::Turbulence::Off => Turbulence::Off,
        bowshock_model::Turbulence::Low => Turbulence::Low,
        bowshock_model::Turbulence::Medium => Turbulence::Med,
        bowshock_model::Turbulence::High => Turbulence::High,
    }
}

/// A flight's conditions: everything that, when changed, makes its result stale. SI throughout (angles in radians,
/// temperature in kelvin).
#[derive(Debug, Clone, PartialEq)]
pub struct Spec {
    /// The motor.
    pub motor: MotorChoice,
    /// Ejection delay (s); `None` takes the configuration's own.
    pub delay: Option<f64>,
    /// Launch rail length (m).
    pub rail_length: f64,
    /// Rail tilt from vertical (rad).
    pub launch_angle: f64,
    /// Mean wind speed (m/s).
    pub wind_speed: f64,
    /// Direction the wind blows from, clockwise from north (rad).
    pub wind_from: f64,
    /// Gust intensity.
    pub turbulence: Turbulence,
    /// Site altitude above sea level (m).
    pub site_altitude: f64,
    /// Ground temperature (K).
    pub temperature: f64,
    /// The distance from the pad the rocket has to land within (m).
    pub field_radius: f64,
    /// Where the drag comes from.
    pub drag: DragSource,
    /// The wind above the ground (see [`Conditions::wind_levels`]).
    pub wind_levels: Vec<WindLevel>,
    /// The launch site's latitude and longitude.
    pub location: Option<Location>,
    /// A start in the air instead of on the pad.
    pub air_start: Option<AirStart>,
}

impl Default for Spec {
    /// The stored default flight's conditions ([`Conditions::default`]): 5° rail, 2.2 m/s breeze from 250°, low turbulence, 120 m
    /// site at 15 °C, 300 m field.
    fn default() -> Self {
        Spec::of(&FlightDef::nominal(0))
    }
}

impl Spec {
    /// The conditions stored in `def`.
    pub fn of(def: &FlightDef) -> Spec {
        let c = &def.conditions;
        Spec {
            motor: MotorChoice::from(&def.motor),
            delay: def.delay,
            rail_length: c.rail_length,
            launch_angle: c.launch_angle,
            wind_speed: c.wind_speed,
            wind_from: c.wind_from,
            turbulence: sim_turbulence(c.turbulence),
            site_altitude: c.site_altitude,
            temperature: c.temperature,
            field_radius: c.field_radius,
            drag: def.drag,
            wind_levels: c.wind_levels.clone(),
            location: c.location,
            air_start: c.air_start,
        }
    }

    /// The stored form of these conditions for flight `id` called `name`.
    pub fn stored(&self, id: FlightId, name: &str) -> FlightDef {
        FlightDef {
            id,
            name: name.to_string(),
            motor: FlightMotor::from(&self.motor),
            delay: self.delay,
            conditions: Conditions {
                rail_length: self.rail_length,
                launch_angle: self.launch_angle,
                wind_speed: self.wind_speed,
                wind_from: self.wind_from,
                turbulence: stored_turbulence(self.turbulence),
                site_altitude: self.site_altitude,
                temperature: self.temperature,
                field_radius: self.field_radius,
                wind_levels: self.wind_levels.clone(),
                location: self.location,
                air_start: self.air_start,
            },
            drag: self.drag,
        }
    }
}

/// The times of the flight a result number points at (clicking the number moves the plot crosshair there).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Times {
    /// Rail exit.
    pub rail_exit: f64,
    /// Last burnout.
    pub burnout: f64,
    /// Apogee.
    pub apogee: f64,
    /// Peak ground speed on the way up.
    pub max_speed: f64,
    /// Peak Mach number on the way up.
    pub max_mach: f64,
    /// Peak acceleration on the way up.
    pub max_accel: f64,
    /// Peak spin rate on the way up (the largest roll rate of either sense).
    pub max_roll: f64,
    /// First recovery deployment, if any.
    pub deploy: Option<f64>,
    /// Ground contact.
    pub landing: f64,
}

impl Times {
    /// Reads the event times and the peak instants out of `result`.
    pub fn of(result: &FlightResult) -> Times {
        let mut t = Times { landing: result.summary.flight_time, apogee: result.summary.time_to_apogee, ..Times::default() };
        for e in &result.trajectory.events {
            match e.kind {
                EventKind::RailExit => t.rail_exit = e.t,
                EventKind::Burnout => t.burnout = t.burnout.max(e.t),
                EventKind::Apogee => t.apogee = e.t,
                EventKind::Deploy(_) => t.deploy = Some(t.deploy.map_or(e.t, |d: f64| d.min(e.t))),
                EventKind::Landing => t.landing = e.t,
                _ => {}
            }
        }
        let up = result.trajectory.samples.iter().take_while(|s| s.t <= t.apogee);
        let (mut v, mut m, mut a, mut r) = (f64::MIN, f64::MIN, f64::MIN, 0.0);
        for s in up {
            if s.speed > v {
                (v, t.max_speed) = (s.speed, s.t);
            }
            if s.mach > m {
                (m, t.max_mach) = (s.mach, s.t);
            }
            if s.acceleration > a {
                (a, t.max_accel) = (s.acceleration, s.t);
            }
            if s.roll_rate.abs() > r {
                (r, t.max_roll) = (s.roll_rate.abs(), s.t);
            }
        }
        t
    }
}

/// One finished run of a flight.
#[derive(Debug, Clone)]
pub struct Run {
    /// The conditions it flew.
    pub spec: Spec,
    /// The document revision it flew.
    pub revision: u64,
    /// The trajectory and summary.
    pub result: Arc<FlightResult>,
    /// Wall-clock time the simulation took.
    pub took: Duration,
    /// The simulation's fine time step (s).
    pub dt: f64,
    /// The ejection delay it flew (s); infinite for a plugged motor.
    pub delay: f64,
    /// Instants of the headline numbers.
    pub times: Times,
}

impl Run {
    /// The worst thing wrong with this flight, `Ok` when nothing: unstable off the rail is an error; a slow rail exit, a hard
    /// landing or a landing outside the field is a warning.
    pub fn severity(&self) -> Severity {
        let s = &self.result.summary;
        if s.stability_off_rail_cal < MIN_OFF_RAIL_CAL {
            Severity::Err
        } else if s.rail_exit_velocity < MIN_RAIL_EXIT || s.ground_hit_velocity > MAX_GROUND_HIT || !self.in_field() {
            Severity::Warn
        } else {
            Severity::Ok
        }
    }

    /// Whether it lands within the field radius it was flown with.
    pub fn in_field(&self) -> bool {
        self.result.summary.landing.0 <= self.spec.field_radius
    }
}

/// One flight of the list.
#[derive(Debug, Clone)]
pub struct Flight {
    /// Stable id.
    pub id: FlightId,
    /// Name in the list.
    pub name: String,
    /// What it flies.
    pub spec: Spec,
    /// The compare colour slot (1–4, `series-1…4`) while ticked; stays with the flight.
    pub slot: Option<u8>,
    /// The last finished run.
    pub run: Option<Run>,
    /// A run is in flight on a worker thread.
    pub running: bool,
    /// Why the last attempt could not run (the rocket has no motor, ...).
    pub error: Option<String>,
    /// Bumped on every launch so a superseded run's result is ignored.
    pub ticket: u64,
    /// The last dispersion study.
    pub dispersion: Option<Dispersed>,
    /// A dispersion study is in flight on a worker thread.
    pub dispersing: bool,
    /// Bumped on every dispersion launch so a superseded study is ignored.
    pub dispersion_ticket: u64,
}

/// Landings of a dispersion study (GroundTrack, "Dispersion"): the flight flown many times with its conditions varied within
/// tolerances.
#[derive(Debug, Clone)]
pub struct Dispersed {
    /// The conditions the study varied around.
    pub spec: Spec,
    /// The document revision it flew.
    pub revision: u64,
    /// Where each run landed, `(east, north)` metres from the pad.
    pub landings: Vec<(f64, f64)>,
}

impl Dispersed {
    /// How many runs landed farther than `radius` from the pad.
    pub fn outside(&self, radius: f64) -> usize {
        self.landings.iter().filter(|(e, n)| e.hypot(*n) > radius).count()
    }
}

/// Runs of a dispersion study.
pub const DISPERSION_RUNS: usize = 100;

/// The tolerances a dispersion study varies `spec` within (1σ): wind speed a quarter of the mean plus 0.3 m/s of gustiness, wind
/// direction 10°, rail angle 1°, motor impulse 3 %, drag 5 % and dry mass 2 %.
pub fn tolerances(spec: &Spec) -> bowshock_sim::Dispersion {
    bowshock_sim::Dispersion {
        wind_speed: 0.25 * spec.wind_speed + 0.3,
        wind_dir: 10f64.to_radians(),
        launch_angle: 1f64.to_radians(),
        thrust_scale: 0.03,
        cd_scale: 0.05,
        mass: 0.02,
    }
}

impl Flight {
    /// A flight that has not run.
    pub fn new(id: FlightId, name: impl Into<String>, spec: Spec) -> Flight {
        Flight {
            id,
            name: name.into(),
            spec,
            slot: None,
            run: None,
            running: false,
            error: None,
            ticket: 0,
            dispersion: None,
            dispersing: false,
            dispersion_ticket: 0,
        }
    }

    /// The dispersion study of the current run, if there is one and it still matches the run.
    pub fn dispersion_of_run(&self) -> Option<&Dispersed> {
        let (run, d) = (self.run.as_ref()?, self.dispersion.as_ref()?);
        (d.spec == run.spec && d.revision == run.revision).then_some(d)
    }

    /// Whether its result no longer matches its conditions or the document.
    pub fn is_stale(&self, revision: u64) -> bool {
        self.run.as_ref().is_some_and(|r| r.spec != self.spec || r.revision != revision)
    }

    /// A flight that has not run, with the conditions stored in `def`.
    pub fn from_def(def: &FlightDef) -> Flight {
        Flight::new(def.id, def.name.clone(), Spec::of(def))
    }

    /// The stored definition of this flight.
    pub fn def(&self) -> FlightDef {
        self.spec.stored(self.id, &self.name)
    }
}

/// The value every item of `values` has, or `None` when they differ (or there are none).
fn agree<T: PartialEq>(mut values: impl Iterator<Item = T>) -> Option<T> {
    let first = values.next()?;
    values.all(|v| v == first).then_some(first)
}

/// What the conditions of several flights have in common (the CONDITIONS panel of a multi-selection): `Some(value)` where every
/// flight agrees, `None` where they differ, which the panel shows as the kit's mixed `—` until the field is edited.
#[derive(Debug, Clone, PartialEq)]
pub struct Shared {
    /// The motor.
    pub motor: Option<MotorChoice>,
    /// The ejection delay each flight flies (s), see [`effective_delay`].
    pub delay: Option<f64>,
    /// Launch rail length (m).
    pub rail_length: Option<f64>,
    /// Rail tilt from vertical (rad).
    pub launch_angle: Option<f64>,
    /// Mean wind speed (m/s).
    pub wind_speed: Option<f64>,
    /// Direction the wind blows from (rad).
    pub wind_from: Option<f64>,
    /// Gust intensity.
    pub turbulence: Option<Turbulence>,
    /// Site altitude (m).
    pub site_altitude: Option<f64>,
    /// Ground temperature (K).
    pub temperature: Option<f64>,
    /// Field radius (m).
    pub field_radius: Option<f64>,
    /// The wind profile.
    pub wind_levels: Option<Vec<WindLevel>>,
    /// The launch location (`Some(None)`: none of the flights has one).
    pub location: Option<Option<Location>>,
    /// The air start (`Some(None)`: every flight starts on the pad).
    pub air_start: Option<Option<AirStart>>,
}

impl Shared {
    /// What `flights` have in common. An empty set agrees on nothing.
    pub fn of(ctx: &Ctx, flights: &[&Flight]) -> Shared {
        let specs = || flights.iter().map(|f| &f.spec);
        Shared {
            motor: agree(specs().map(|s| s.motor.clone())),
            delay: agree(specs().map(|s| effective_delay(ctx, s))),
            rail_length: agree(specs().map(|s| s.rail_length)),
            launch_angle: agree(specs().map(|s| s.launch_angle)),
            wind_speed: agree(specs().map(|s| s.wind_speed)),
            wind_from: agree(specs().map(|s| s.wind_from)),
            turbulence: agree(specs().map(|s| s.turbulence)),
            site_altitude: agree(specs().map(|s| s.site_altitude)),
            temperature: agree(specs().map(|s| s.temperature)),
            field_radius: agree(specs().map(|s| s.field_radius)),
            wind_levels: agree(specs().map(|s| s.wind_levels.clone())),
            location: agree(specs().map(|s| s.location)),
            air_start: agree(specs().map(|s| s.air_start)),
        }
    }
}

/// The first motor mount of the first stage (a cluster flies its first motor), else any mount.
pub fn first_mount(ctx: &Ctx) -> Option<MotorMount> {
    let rocket = ctx.doc.rocket();
    ctx.physics.mounts_of_stage(rocket, 1).into_iter().next().or_else(|| rocket.motor_mounts().into_iter().next())
}

/// The ejection delay (s) `spec` flies, without building the run: its own, else the one the configuration has in the first
/// mount (0 with no mount or no slot).
pub fn effective_delay(ctx: &Ctx, spec: &Spec) -> f64 {
    if let Some(delay) = spec.delay {
        return delay;
    }
    let rocket = ctx.doc.rocket();
    let config = match &spec.motor {
        MotorChoice::Config(id) => rocket.configuration(*id),
        _ => Some(rocket.active_configuration()),
    };
    first_mount(ctx).and_then(|m| config?.slot(m.id)).map_or(0.0, |s| s.delay)
}

/// The designations the flight burns, in firing order (`H128W`, `I284W → H128W`), or `No motor`.
pub fn motor_label(ctx: &Ctx, spec: &Spec) -> String {
    let rocket = ctx.doc.rocket();
    match &spec.motor {
        MotorChoice::Motor(m) => m.designation.clone(),
        MotorChoice::Active => designations(rocket, rocket.active_config_id()),
        MotorChoice::Config(id) => designations(rocket, *id),
    }
}

fn designations(rocket: &Rocket, id: ConfigId) -> String {
    let Some(config) = rocket.configuration(id) else { return "—".to_string() };
    let mounts = rocket.motor_mounts();
    let mut motors: Vec<(f64, &str)> = config
        .motors
        .iter()
        .filter_map(|s| {
            let d = s.motor.as_ref()?.designation.as_str();
            Some((mounts.iter().find(|m| m.id == s.mount).map_or(0.0, |m| m.fore_station), d))
        })
        .collect();
    motors.sort_by(|a, b| b.0.total_cmp(&a.0));
    if motors.is_empty() {
        "No motor".to_string()
    } else {
        motors.iter().map(|m| m.1).collect::<Vec<_>>().join(" → ")
    }
}

/// The launch conditions of `spec`. The rail leans into the wind (the usual practice, and why a track hooks back on itself).
pub fn launch_of(spec: &Spec) -> Launch {
    Launch {
        rail_length: spec.rail_length,
        angle_from_vertical: spec.launch_angle,
        direction: spec.wind_from,
        site_altitude: spec.site_altitude,
        ground_temperature: spec.temperature,
        wind_speed: spec.wind_speed,
        wind_from: spec.wind_from,
        wind_levels: spec.wind_levels.iter().map(|l| WindPoint { altitude: l.altitude, speed: l.speed, from: l.from }).collect(),
        turbulence: spec.turbulence,
        seed: 1,
        latitude: spec.location.map(|l| l.latitude.to_radians()),
        air_start: spec.air_start.map(|a| (a.altitude, a.speed)),
    }
}

/// The first motor mount of `rocket`'s first stage, else its first mount (see [`first_mount`]).
fn first_mount_of(rocket: &Rocket) -> Option<MotorMount> {
    let mounts = rocket.motor_mounts();
    let first = rocket.stages().into_iter().find(|s| s.index == 1);
    let of_stage = first.and_then(|s| mounts.iter().find(|m| s.mounts.contains(&m.id)).cloned());
    of_stage.or_else(|| mounts.into_iter().next())
}

/// Builds the simulation input for `spec` from the document and returns it with the ejection delay it flies. The document is
/// cloned, never touched: the flight's motor and delay are swapped into the clone.
pub fn prepare(ctx: &Ctx, spec: &Spec) -> Result<(SimInput, f64), String> {
    let rocket = ctx.doc.rocket().clone();
    let (mut input, delay) = prepare_rocket(rocket.clone(), ctx.physics.motors(), spec)?;
    if spec.drag == DragSource::CfdTable {
        let stacks = 0..input.vehicle.stages.len();
        input.drag_tables = stacks.clone().map(|k| ctx.physics.drag_table(&rocket, k + 1).ok()).collect();
        input.lift_tables = stacks.map(|k| ctx.physics.lift_table(&rocket, k + 1).ok()).collect();
    }
    Ok((input, delay))
}

/// The simulation input of `spec` flown by `rocket` with `motors` (no CFD tables: those live with the session), and the ejection
/// delay it flies: the flight's motor and delay swapped into `rocket`. What [`prepare`] builds on, and what a worker thread
/// that has no [`Ctx`] calls.
pub fn prepare_rocket(mut rocket: Rocket, motors: &bowshock_motors::MotorDb, spec: &Spec) -> Result<(SimInput, f64), String> {
    let mount = first_mount_of(&rocket);
    match &spec.motor {
        MotorChoice::Active => {}
        MotorChoice::Config(id) => {
            if !rocket.select_configuration(*id) {
                return Err("that configuration no longer exists".to_string());
            }
        }
        MotorChoice::Motor(motor) => {
            let mount = mount.as_ref().ok_or("the rocket has no motor mount")?;
            rocket.set_motor(rocket.active_config_id(), mount.id, Some(motor.clone())).map_err(|e| e.to_string())?;
        }
    }
    if let (Some(delay), Some(mount)) = (spec.delay, mount.as_ref()) {
        rocket.set_motor_delay(rocket.active_config_id(), mount.id, delay).map_err(|e| e.to_string())?;
    }
    let delay = mount.and_then(|m| rocket.active_configuration().slot(m.id).map(|s| s.delay)).unwrap_or(0.0);
    let input = from_model::sim_input(&rocket, motors, Phase::Full, launch_of(spec)).map_err(|e| format!("Cannot fly: {e}"))?;
    if input.vehicle.stages.iter().all(|s| s.motor.is_none()) {
        return Err("Cannot fly: no motor in the configuration".to_string());
    }
    Ok((input, delay))
}

/// What to tell about `spec`'s drag source: why a stack of the rocket flies on the build-up although the CFD table was asked for,
/// when the drag table is fine why it flies on Extended's stability (no or a stale 3D solve), and that the parts' drag overrides do
/// not apply to the table.
pub fn drag_note(ctx: &Ctx, spec: &Spec) -> Option<String> {
    if spec.drag != DragSource::CfdTable {
        return None;
    }
    let rocket = ctx.doc.rocket();
    let stages = 1..=rocket.stages().len().max(1);
    let fallback = stages.clone().find_map(|stage| ctx.physics.drag_table(rocket, stage).err());
    fallback
        .or_else(|| stages.into_iter().find_map(|stage| ctx.physics.lift_table(rocket, stage).err()))
        .or_else(|| has_cd_override(rocket).then(|| "Cd overrides apply to Extended drag only.".to_string()))
}

/// Whether any part of `rocket` (a pod's included) has a drag override.
fn has_cd_override(rocket: &Rocket) -> bool {
    let radial = |r: &bowshock_model::RadialPart| {
        r.cd_override.is_some()
            || r.kind.pod_body().is_some_and(|b| b.parts().iter().any(|p| p.cd_override.is_some()) || b.radials().iter().any(|p| p.cd_override.is_some()))
    };
    rocket.parts().iter().any(|p| p.cd_override.is_some()) || rocket.radials().iter().any(radial)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_kit_flights_are_nominal_windy_hot_and_bigger() {
        let flights: Vec<Flight> = bowshock_model::example_flights().iter().map(Flight::from_def).collect();
        let names: Vec<&str> = flights.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["Nominal", "Windy", "Hot day", "Bigger motor"]);
        assert_eq!(flights[1].spec.wind_speed, 7.0);
        assert!((flights[2].spec.temperature - 308.15).abs() < 1e-9);
        assert_eq!(flights[3].spec.motor, MotorChoice::Motor(MotorRef::new("I284W", "AeroTech")));
        // Ids are unique.
        assert!(flights.windows(2).all(|w| w[0].id != w[1].id));
        // A flight survives the trip through its stored form.
        assert!(flights.iter().all(|f| Flight::from_def(&f.def()).spec == f.spec));
    }

    #[test]
    fn launch_leans_into_the_wind_and_carries_the_site() {
        let launch = launch_of(&Spec::default());
        assert!((launch.angle_from_vertical - 5f64.to_radians()).abs() < 1e-12);
        assert_eq!(launch.direction, launch.wind_from);
        assert_eq!((launch.site_altitude, launch.ground_temperature, launch.wind_speed), (120.0, 288.15, 2.2));
    }

    #[test]
    fn a_result_is_stale_when_the_spec_or_the_document_changes() {
        let mut f = Flight::new(1, "x", Spec::default());
        assert!(!f.is_stale(0), "no run, nothing to be stale");
        f.run = Some(Run {
            spec: f.spec.clone(),
            revision: 3,
            result: Arc::new(FlightResult::default()),
            took: Duration::ZERO,
            dt: 0.001,
            delay: 10.0,
            times: Times::default(),
        });
        assert!(!f.is_stale(3));
        assert!(f.is_stale(4));
        f.spec.wind_speed = 3.0;
        assert!(f.is_stale(3));
    }

    #[test]
    fn physics_caches_cfd_plan_across_views() {
        let mut app = crate::app::App::with_example();
        let (rocket, revision, units) = (app.ctx().doc.rocket().clone(), app.ctx().doc.revision(), app.ctx().units);
        app.ctx_mut().physics.sync(&rocket, revision, units);
        let first = app.ctx().physics.cfd_plan(1).expect("plan") as *const _;
        let spec = Spec { drag: DragSource::CfdTable, ..Spec::default() };
        for _ in 0..3 {
            assert!(drag_note(app.ctx(), &spec).is_some());
        }
        assert!(!app.ctx_mut().physics.sync(&rocket, revision, units), "nothing changed, nothing recomputes");
        let second = app.ctx().physics.cfd_plan(1).expect("plan") as *const _;
        assert!(std::ptr::eq(first, second));
    }
}
