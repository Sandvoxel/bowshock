//! Motor configurations: one rocket, several ways to fly it. A configuration names what goes in each motor mount and how the
//! flight is timed; the airframe is shared. The model stores only motor *references*: thrust curves and masses come from the
//! motor database.

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::internal::{DeployEvent, Deployment, InternalKind};
use crate::rocket::Rocket;
use crate::PartId;

/// Stable identifier of a [`Configuration`] within a [`Rocket`].
pub type ConfigId = u64;

/// A motor in the motor database, by name.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct MotorRef {
    /// Designation, e.g. `"H128W"`.
    pub designation: String,
    /// Manufacturer, e.g. `"Aerotech"`.
    pub manufacturer: String,
}

impl MotorRef {
    /// A reference to `designation` by `manufacturer`.
    pub fn new(designation: impl Into<String>, manufacturer: impl Into<String>) -> MotorRef {
        MotorRef { designation: designation.into(), manufacturer: manufacturer.into() }
    }
}

/// What a staging event waits for.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum StageTrigger {
    /// The burnout of the stage the event belongs to (a separation), or of the stage below it (an ignition).
    Burnout,
    /// The separation of the stage below (an ignition only).
    Separation,
    /// Launch: the event's delay is the time from launch.
    TimeFromLaunch,
    /// The rocket climbing through this altitude above the launch site, metres, on the way up.
    Altitude(f64),
}

impl StageTrigger {
    /// Whether the trigger can be used for an event of `kind`: a separation cannot wait for itself.
    pub fn valid_for(self, kind: EventKind) -> bool {
        !(self == StageTrigger::Separation && kind == EventKind::Separation)
    }
}

/// Which staging event of a stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EventKind {
    /// The stage separates from the one above it.
    Separation,
    /// The stage's motor ignites.
    Ignition,
}

/// A timed staging event of one stage: a trigger and a delay after it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StageEvent {
    /// The stage (`Stage::index`, from 1, the aft-most first to fire); a separation is the stage's separation from the one above.
    pub stage: usize,
    /// Which event.
    pub kind: EventKind,
    /// What it waits for.
    pub trigger: StageTrigger,
    /// Seconds after the trigger.
    pub delay: f64,
}

impl StageEvent {
    /// What an event does when the configuration does not say: a stage separates at its burnout; the first stage lights at
    /// launch and every upper stage when the stage below separates.
    pub fn default_of(stage: usize, kind: EventKind) -> StageEvent {
        let trigger = match (kind, stage) {
            (EventKind::Separation, _) => StageTrigger::Burnout,
            (EventKind::Ignition, 0 | 1) => StageTrigger::TimeFromLaunch,
            (EventKind::Ignition, _) => StageTrigger::Separation,
        };
        StageEvent { stage, kind, trigger, delay: 0.0 }
    }
}

/// Ballast a configuration adds to rebalance for its motor: a point mass at a station.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ballast {
    /// Mass in kg.
    pub mass: f64,
    /// Station from the rocket's tip, metres.
    pub station: f64,
}

/// The deployment of one recovery device in one configuration, which replaces the part's own.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecoveryTiming {
    /// The parachute or streamer.
    pub part: PartId,
    /// When it deploys in this configuration.
    pub deploy: Deployment,
}

/// What goes in one motor mount in one configuration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotorSlot {
    /// The inner tube (an [`InternalKind::InnerTube`] with `motor_mount`).
    pub mount: PartId,
    /// The motor in it; `None` leaves the mount empty.
    pub motor: Option<MotorRef>,
    /// Ejection charge delay in seconds after burnout; `f64::INFINITY` is a plugged motor (no ejection charge).
    #[serde(with = "plugged_delay")]
    pub delay: f64,
}

impl MotorSlot {
    /// An empty slot on `mount` with no delay.
    pub fn empty(mount: PartId) -> MotorSlot {
        MotorSlot { mount, motor: None, delay: 0.0 }
    }

    /// Whether the ejection charge is blocked.
    pub fn is_plugged(&self) -> bool {
        self.delay.is_infinite()
    }
}

/// A named choice of motors and timing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Configuration {
    /// Unique, stable id.
    pub id: ConfigId,
    /// Name; empty means "named after the motors" (see [`display_name`](Self::display_name)).
    pub name: String,
    /// One entry per motor mount that has been given a motor or timing.
    pub motors: Vec<MotorSlot>,
    /// Staging events that differ from the defaults (see [`StageEvent::default_of`]): when each stage separates and when each
    /// upper stage lights. Keyed by stage and kind.
    #[serde(default)]
    pub events: Vec<StageEvent>,
    /// Recovery devices whose deployment differs from the part's own in this configuration.
    #[serde(default)]
    pub recovery: Vec<RecoveryTiming>,
    /// Ballast this configuration carries, if any.
    #[serde(default)]
    pub ballast: Option<Ballast>,
}

impl Configuration {
    /// An empty configuration.
    pub fn new(id: ConfigId, name: impl Into<String>) -> Configuration {
        Configuration { id, name: name.into(), motors: Vec::new(), events: Vec::new(), recovery: Vec::new(), ballast: None }
    }

    /// The slot for `mount`.
    pub fn slot(&self, mount: PartId) -> Option<&MotorSlot> {
        self.motors.iter().find(|s| s.mount == mount)
    }

    /// The name, or when empty the designations of the motors joined by ` → ` in slot order (use
    /// [`Rocket::configuration_name`] for firing order), or `"No motor"`.
    pub fn display_name(&self) -> String {
        if !self.name.is_empty() {
            return self.name.clone();
        }
        let motors: Vec<&str> = self.motors.iter().filter_map(|s| s.motor.as_ref().map(|m| m.designation.as_str())).collect();
        if motors.is_empty() {
            "No motor".to_string()
        } else {
            motors.join(" → ")
        }
    }

    /// The staging event `kind` of `stage` as this configuration has it (the default when it says nothing).
    pub fn event(&self, stage: usize, kind: EventKind) -> StageEvent {
        self.events.iter().find(|e| e.stage == stage && e.kind == kind).copied().unwrap_or_else(|| StageEvent::default_of(stage, kind))
    }

    /// The deployment this configuration gives recovery device `part`, if it overrides the part's own (see
    /// [`Rocket::deployment`]).
    pub fn deployment(&self, part: PartId) -> Option<Deployment> {
        self.recovery.iter().find(|r| r.part == part).map(|r| r.deploy)
    }
}

/// A motor mount of the rocket and where its motor sits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMount {
    /// The inner tube's id.
    pub id: PartId,
    /// The axial part it is in.
    pub parent: PartId,
    /// Station of the tube's fore end, from the rocket's tip.
    pub fore_station: f64,
    /// Station of the tube's aft end.
    pub aft_station: f64,
    /// How far the motor's aft end sticks out behind the tube (negative: recessed).
    pub overhang: f64,
    /// Station of the motor's aft end: `aft_station + overhang`.
    pub motor_aft_station: f64,
    /// Bore of the tube: the largest motor diameter that fits.
    pub inner_diameter: f64,
    /// Length of the tube.
    pub length: f64,
    /// The pod set the mount is in; `None` for a mount of the core. A pod's mount holds one motor per copy of the pod: the
    /// motors fly with the stage of the core part the pod stands on, lit with it (strap-on boosters).
    pub pod: Option<PartId>,
    /// How many motors the mount stands for: the copies of its pod, 1 for the core.
    pub copies: u8,
}

/// Why a configuration change was refused.
#[derive(Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// No configuration has this id.
    NoSuchConfig,
    /// The part is not a motor mount.
    NotAMotorMount,
    /// The last configuration cannot be removed.
    LastConfiguration,
    /// The trigger cannot be used for that event.
    InvalidTrigger,
    /// The part is not a parachute or streamer.
    NotARecoveryDevice,
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ConfigError::NoSuchConfig => "no such configuration",
            ConfigError::NotAMotorMount => "the part is not a motor mount",
            ConfigError::LastConfiguration => "the last configuration cannot be removed",
            ConfigError::InvalidTrigger => "that trigger cannot be used for this event",
            ConfigError::NotARecoveryDevice => "the part is not a recovery device",
        })
    }
}

impl std::error::Error for ConfigError {}

impl Rocket {
    /// Every configuration, in the order they were made. There is always at least one.
    pub fn configurations(&self) -> &[Configuration] {
        &self.configs
    }

    /// The configuration with `id`.
    pub fn configuration(&self, id: ConfigId) -> Option<&Configuration> {
        self.configs.iter().find(|c| c.id == id)
    }

    /// Id of the active configuration (the one being flown and shown in the status strip).
    pub fn active_config_id(&self) -> ConfigId {
        self.active_config
    }

    /// The active configuration.
    pub fn active_configuration(&self) -> &Configuration {
        self.configs.iter().find(|c| c.id == self.active_config).unwrap_or(&self.configs[0])
    }

    /// Adds an empty configuration (no motors) named `name` (empty: named after its motors) and returns its id. It does not
    /// become active; see [`select_configuration`](Self::select_configuration).
    pub fn add_configuration(&mut self, name: impl Into<String>) -> ConfigId {
        let id = self.next_config_id;
        self.next_config_id += 1;
        self.configs.push(Configuration::new(id, name));
        id
    }

    /// Copies configuration `id` (motors, delays, staging and recovery timing, ballast) and returns the copy's id; a named
    /// configuration gets `" copy"` appended to its name. The copy does not become active.
    pub fn duplicate_configuration(&mut self, id: ConfigId) -> Option<ConfigId> {
        let mut copy = self.configuration(id)?.clone();
        copy.id = self.next_config_id;
        self.next_config_id += 1;
        if !copy.name.is_empty() {
            copy.name.push_str(" copy");
        }
        let id = copy.id;
        self.configs.push(copy);
        Some(id)
    }

    /// Removes configuration `id`, returning it. The last configuration cannot be removed. Removing the active one activates
    /// its neighbour.
    pub fn remove_configuration(&mut self, id: ConfigId) -> Result<Configuration, ConfigError> {
        let i = self.configs.iter().position(|c| c.id == id).ok_or(ConfigError::NoSuchConfig)?;
        if self.configs.len() == 1 {
            return Err(ConfigError::LastConfiguration);
        }
        let removed = self.configs.remove(i);
        if self.active_config == id {
            self.active_config = self.configs[i.min(self.configs.len() - 1)].id;
        }
        Ok(removed)
    }

    /// Renames configuration `id` (an empty name means "named after the motors"). Returns whether it exists.
    pub fn rename_configuration(&mut self, id: ConfigId, name: impl Into<String>) -> bool {
        match self.configs.iter_mut().find(|c| c.id == id) {
            Some(c) => {
                c.name = name.into();
                true
            }
            None => false,
        }
    }

    /// Makes `id` the active configuration. Returns whether it exists.
    pub fn select_configuration(&mut self, id: ConfigId) -> bool {
        let exists = self.configuration(id).is_some();
        if exists {
            self.active_config = id;
        }
        exists
    }

    /// The slot of configuration `config` for `mount`, created empty if missing.
    fn slot_mut(&mut self, config: ConfigId, mount: PartId) -> Result<&mut MotorSlot, ConfigError> {
        if !self.motor_mounts().iter().any(|m| m.id == mount) {
            return Err(ConfigError::NotAMotorMount);
        }
        let c = self.configs.iter_mut().find(|c| c.id == config).ok_or(ConfigError::NoSuchConfig)?;
        let i = match c.motors.iter().position(|s| s.mount == mount) {
            Some(i) => i,
            None => {
                c.motors.push(MotorSlot::empty(mount));
                c.motors.len() - 1
            }
        };
        Ok(&mut c.motors[i])
    }

    /// Puts `motor` (or none) in `mount` for configuration `config`.
    pub fn set_motor(&mut self, config: ConfigId, mount: PartId, motor: Option<MotorRef>) -> Result<(), ConfigError> {
        self.slot_mut(config, mount)?.motor = motor;
        Ok(())
    }

    /// Sets the ejection delay of the motor in `mount` (`f64::INFINITY` plugs it).
    pub fn set_motor_delay(&mut self, config: ConfigId, mount: PartId, delay: f64) -> Result<(), ConfigError> {
        self.slot_mut(config, mount)?.delay = delay;
        Ok(())
    }

    /// Sets a staging event of configuration `config`: the trigger and delay of the separation or ignition of `event.stage`. A
    /// negative or non-finite delay (or altitude) becomes 0. Refused with [`ConfigError::InvalidTrigger`] for a separation that
    /// waits for a separation.
    pub fn set_stage_event(&mut self, config: ConfigId, event: StageEvent) -> Result<(), ConfigError> {
        if !event.trigger.valid_for(event.kind) {
            return Err(ConfigError::InvalidTrigger);
        }
        let clean = |x: f64| if x.is_finite() { x.max(0.0) } else { 0.0 };
        let event = StageEvent {
            delay: clean(event.delay),
            trigger: match event.trigger {
                StageTrigger::Altitude(h) => StageTrigger::Altitude(clean(h)),
                other => other,
            },
            ..event
        };
        let c = self.configs.iter_mut().find(|c| c.id == config).ok_or(ConfigError::NoSuchConfig)?;
        match c.events.iter_mut().find(|e| e.stage == event.stage && e.kind == event.kind) {
            Some(entry) => *entry = event,
            None => c.events.push(event),
        }
        Ok(())
    }

    /// The deployment of recovery device `part` in configuration `config`: the configuration's own if it has one, else the
    /// part's. `None` if either does not exist or the part is not a parachute or streamer.
    pub fn deployment(&self, config: ConfigId, part: PartId) -> Option<Deployment> {
        let own = match self.internal(part)?.kind {
            InternalKind::Parachute { deploy, .. } | InternalKind::Streamer { deploy, .. } => deploy,
            _ => return None,
        };
        Some(self.configuration(config)?.deployment(part).unwrap_or(own))
    }

    /// [`deployment`](Self::deployment) in the active configuration.
    pub fn active_deployment(&self, part: PartId) -> Option<Deployment> {
        self.deployment(self.active_config, part)
    }

    /// Gives recovery device `part` the deployment `deploy` in configuration `config` only (the part's own value is the default
    /// of the others). A negative or non-finite delay (or altitude) becomes 0.
    pub fn set_deployment(&mut self, config: ConfigId, part: PartId, deploy: Deployment) -> Result<(), ConfigError> {
        if !matches!(self.internal(part).map(|p| &p.kind), Some(InternalKind::Parachute { .. } | InternalKind::Streamer { .. })) {
            return Err(ConfigError::NotARecoveryDevice);
        }
        let clean = |x: f64| if x.is_finite() { x.max(0.0) } else { 0.0 };
        let deploy = Deployment {
            delay: clean(deploy.delay),
            event: match deploy.event {
                DeployEvent::Altitude(h) => DeployEvent::Altitude(clean(h)),
                other => other,
            },
        };
        let c = self.configs.iter_mut().find(|c| c.id == config).ok_or(ConfigError::NoSuchConfig)?;
        match c.recovery.iter_mut().find(|r| r.part == part) {
            Some(entry) => entry.deploy = deploy,
            None => c.recovery.push(RecoveryTiming { part, deploy }),
        }
        Ok(())
    }

    /// Sets (or with `None` removes) the ballast of configuration `config`. A mass that is not positive removes it; the
    /// station is kept within the rocket.
    pub fn set_ballast(&mut self, config: ConfigId, ballast: Option<Ballast>) -> Result<(), ConfigError> {
        let length = self.length();
        let c = self.configs.iter_mut().find(|c| c.id == config).ok_or(ConfigError::NoSuchConfig)?;
        c.ballast = ballast
            .filter(|b| b.mass.is_finite() && b.mass > 0.0)
            .map(|b| Ballast { mass: b.mass, station: if b.station.is_finite() { b.station.clamp(0.0, length) } else { 0.0 } });
        Ok(())
    }

    /// Display name of configuration `id`: its name, or when empty the motor designations in firing order (aft-most mount
    /// first), `"I284W → H128W"`; `"No motor"` if it has none.
    pub fn configuration_name(&self, id: ConfigId) -> Option<String> {
        let c = self.configuration(id)?;
        if !c.name.is_empty() {
            return Some(c.name.clone());
        }
        let mounts = self.motor_mounts();
        let mut motors: Vec<(f64, &str)> = c
            .motors
            .iter()
            .filter_map(|s| {
                let designation = s.motor.as_ref()?.designation.as_str();
                Some((mounts.iter().find(|m| m.id == s.mount).map_or(0.0, |m| m.fore_station), designation))
            })
            .collect();
        motors.sort_by(|a, b| b.0.total_cmp(&a.0));
        Some(if motors.is_empty() { "No motor".to_string() } else { motors.iter().map(|m| m.1).collect::<Vec<_>>().join(" → ") })
    }

    /// The motor mounts (inner tubes with `motor_mount`) of the core and of pods, fore to aft by their fore end. A mount in a pod
    /// stands for one motor per copy of the pod ([`MotorMount::copies`]) and is left out of the stages.
    pub fn motor_mounts(&self) -> Vec<MotorMount> {
        let mut mounts: Vec<MotorMount> = self
            .internals
            .iter()
            .chain(self.pod_internals())
            .filter_map(|p| {
                let InternalKind::InnerTube { outer_diameter, thickness, length, motor_mount: true, overhang } = p.kind else {
                    return None;
                };
                let fore = self.part_station(p.parent)? + p.offset;
                let pod = self.pod_of(p.parent);
                Some(MotorMount {
                    id: p.id,
                    parent: p.parent,
                    fore_station: fore,
                    aft_station: fore + length,
                    overhang,
                    motor_aft_station: fore + length + overhang,
                    inner_diameter: (outer_diameter - 2.0 * thickness).max(0.0),
                    length,
                    pod: pod.map(|p| p.id),
                    copies: pod.map_or(1, |p| p.count.max(1)),
                })
            })
            .collect();
        mounts.sort_by(|a, b| a.fore_station.total_cmp(&b.fore_station).then(a.id.cmp(&b.id)));
        mounts
    }

    /// Drops motor slots whose mount is no longer a motor mount.
    pub(crate) fn prune_configs(&mut self) {
        let mounts: Vec<PartId> = self.internals.iter().chain(self.pod_internals()).filter(|p| p.kind.is_motor_mount()).map(|p| p.id).collect();
        for c in &mut self.configs {
            c.motors.retain(|s| mounts.contains(&s.mount));
        }
        let devices: Vec<PartId> = self
            .internals
            .iter()
            .chain(self.pod_internals())
            .filter(|p| matches!(p.kind, InternalKind::Parachute { .. } | InternalKind::Streamer { .. }))
            .map(|p| p.id)
            .collect();
        for c in &mut self.configs {
            c.recovery.retain(|r| devices.contains(&r.part));
        }
    }
}

/// Serde for a delay where infinity (a plugged motor) is `null` in JSON, which cannot hold non-finite numbers.
mod plugged_delay {
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(delay: &f64, s: S) -> Result<S::Ok, S::Error> {
        if delay.is_finite() {
            s.serialize_f64(*delay)
        } else {
            s.serialize_none()
        }
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
        Ok(Option::<f64>::deserialize(d)?.unwrap_or(f64::INFINITY))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::PartKind;

    fn rocket_with_mount() -> (Rocket, PartId) {
        let mut r = Rocket::default();
        let tube = r.insert(0, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let mount = r
            .attach_internal(tube, 0.3, InternalKind::InnerTube { outer_diameter: 0.054, thickness: 0.0016, length: 0.2, motor_mount: true, overhang: 0.01 })
            .unwrap();
        (r, mount)
    }

    #[test]
    fn default_rocket_has_one_configuration() {
        let r = Rocket::default();
        assert_eq!(r.configurations().len(), 1);
        assert_eq!(r.active_configuration().display_name(), "No motor");
    }

    #[test]
    fn mounts_report_stations() {
        let (r, mount) = rocket_with_mount();
        let m = r.motor_mounts();
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].id, mount);
        assert!((m[0].fore_station - 0.3).abs() < 1e-12);
        assert!((m[0].aft_station - 0.5).abs() < 1e-12);
        assert!((m[0].motor_aft_station - 0.51).abs() < 1e-12);
        assert!((m[0].inner_diameter - 0.0508).abs() < 1e-12);
    }

    #[test]
    fn add_duplicate_rename_select_remove() {
        let (mut r, mount) = rocket_with_mount();
        let first = r.active_config_id();
        r.set_motor(first, mount, Some(MotorRef::new("H128W", "Aerotech"))).unwrap();
        r.set_motor_delay(first, mount, 8.0).unwrap();
        assert_eq!(r.active_configuration().display_name(), "H128W");

        let copy = r.duplicate_configuration(first).unwrap();
        assert_ne!(copy, first);
        assert_eq!(r.configuration(copy).unwrap().motors, r.configuration(first).unwrap().motors);
        assert!(r.rename_configuration(copy, "Light"));
        assert_eq!(r.configuration(copy).unwrap().display_name(), "Light");
        let again = r.duplicate_configuration(copy).unwrap();
        assert_eq!(r.configuration(again).unwrap().name, "Light copy");

        let fresh = r.add_configuration("");
        assert!(r.configuration(fresh).unwrap().motors.is_empty());
        assert!(r.select_configuration(copy));
        assert_eq!(r.active_config_id(), copy);
        assert!(!r.select_configuration(999));

        // Removing the active configuration activates a neighbour; the last one stays.
        r.remove_configuration(copy).unwrap();
        assert_ne!(r.active_config_id(), copy);
        assert!(r.configuration(r.active_config_id()).is_some());
        while r.configurations().len() > 1 {
            let id = r.configurations()[0].id;
            r.remove_configuration(id).unwrap();
        }
        let last = r.configurations()[0].id;
        assert_eq!(r.remove_configuration(last), Err(ConfigError::LastConfiguration));
        assert_eq!(r.remove_configuration(999), Err(ConfigError::NoSuchConfig));
    }

    #[test]
    fn slots_only_for_motor_mounts_and_plug() {
        let (mut r, mount) = rocket_with_mount();
        let id = r.active_config_id();
        assert_eq!(r.set_motor(id, 12345, None), Err(ConfigError::NotAMotorMount));
        assert_eq!(r.set_motor(77, mount, None), Err(ConfigError::NoSuchConfig));
        r.set_motor_delay(id, mount, f64::INFINITY).unwrap();
        assert!(r.active_configuration().slot(mount).unwrap().is_plugged());
        r.set_stage_event(id, StageEvent { stage: 2, kind: EventKind::Ignition, trigger: StageTrigger::Separation, delay: 0.5 }).unwrap();
        let sep = |delay| StageEvent { stage: 1, kind: EventKind::Separation, trigger: StageTrigger::Burnout, delay };
        r.set_stage_event(id, sep(0.25)).unwrap();
        r.set_stage_event(id, sep(0.5)).unwrap();
        assert_eq!(r.active_configuration().events.len(), 2, "one entry per stage and kind");
        assert_eq!(r.active_configuration().event(1, EventKind::Separation).delay, 0.5);
        assert_eq!(r.active_configuration().event(2, EventKind::Ignition).delay, 0.5);
        // Defaults: separate at burnout; the first stage lights at launch, upper stages on separation.
        assert_eq!(r.active_configuration().event(2, EventKind::Separation), StageEvent::default_of(2, EventKind::Separation));
        assert_eq!(StageEvent::default_of(1, EventKind::Ignition).trigger, StageTrigger::TimeFromLaunch);
        assert_eq!(StageEvent::default_of(3, EventKind::Ignition).trigger, StageTrigger::Separation);
        // A separation cannot wait for a separation; bad numbers become 0.
        let bad = StageEvent { trigger: StageTrigger::Separation, ..sep(0.0) };
        assert_eq!(r.set_stage_event(id, bad), Err(ConfigError::InvalidTrigger));
        r.set_stage_event(id, StageEvent { trigger: StageTrigger::Altitude(-5.0), ..sep(f64::NAN) }).unwrap();
        let e = r.active_configuration().event(1, EventKind::Separation);
        assert_eq!((e.trigger, e.delay), (StageTrigger::Altitude(0.0), 0.0));
        assert_eq!(r.set_stage_event(77, sep(0.0)), Err(ConfigError::NoSuchConfig));
    }

    #[test]
    fn recovery_timing_and_ballast_are_per_configuration_and_copied_by_duplicate() {
        let (mut r, _) = rocket_with_mount();
        let tube = r.parts()[0].id;
        let chute = r
            .attach_internal(
                tube,
                0.1,
                InternalKind::Parachute {
                    diameter: 0.5,
                    cd: 0.8,
                    line_count: 6,
                    line_length: 0.5,
                    line_material: crate::material::MaterialId::KEVLAR_LINE,
                    packed_length: 0.1,
                    packed_diameter: None,
                    deploy: Deployment { event: DeployEvent::Apogee, delay: 0.0 },
                },
            )
            .unwrap();
        let first = r.active_config_id();
        let second = r.add_configuration("Windy");
        // The part's own deployment is the default of every configuration.
        assert_eq!(r.deployment(second, chute), Some(Deployment { event: DeployEvent::Apogee, delay: 0.0 }));
        r.set_deployment(second, chute, Deployment { event: DeployEvent::Altitude(120.0), delay: 1.0 }).unwrap();
        assert_eq!(r.deployment(first, chute).unwrap().event, DeployEvent::Apogee);
        assert_eq!(r.deployment(second, chute).unwrap().event, DeployEvent::Altitude(120.0));
        assert_eq!(r.set_deployment(second, 4242, Deployment::default()), Err(ConfigError::NotARecoveryDevice));

        r.set_ballast(second, Some(Ballast { mass: 0.05, station: 99.0 })).unwrap();
        assert!(r.configuration(second).unwrap().ballast.unwrap().station <= r.length(), "kept within the rocket");
        assert!(r.configuration(first).unwrap().ballast.is_none());
        let copy = r.duplicate_configuration(second).unwrap();
        assert_eq!(r.configuration(copy).unwrap().recovery, r.configuration(second).unwrap().recovery);
        assert_eq!(r.configuration(copy).unwrap().ballast, r.configuration(second).unwrap().ballast);
        r.set_ballast(second, Some(Ballast { mass: 0.0, station: 0.1 })).unwrap();
        assert!(r.configuration(second).unwrap().ballast.is_none(), "no mass, no ballast");

        // Removing the part drops the overrides.
        r.remove_internal(chute).unwrap();
        assert!(r.configuration(copy).unwrap().recovery.is_empty());
        assert_eq!(r.deployment(copy, chute), None);
    }

    #[test]
    fn removing_a_mount_drops_its_motors() {
        let (mut r, mount) = rocket_with_mount();
        let id = r.active_config_id();
        r.set_motor(id, mount, Some(MotorRef::new("H128W", "Aerotech"))).unwrap();
        // Carrying the mount with its parent keeps the motor...
        let parent = r.internal(mount).unwrap().parent;
        let carried = r.take_internals(parent);
        assert_eq!(r.configuration(id).unwrap().motors.len(), 1);
        r.restore_internals(carried);
        assert_eq!(r.motor_mounts().len(), 1);
        // ...removing it drops it, and so does turning it into a plain tube.
        let kind = r.internal(mount).unwrap().kind.clone();
        let InternalKind::InnerTube { outer_diameter, thickness, length, overhang, .. } = kind else { unreachable!() };
        assert!(r.set_internal_kind(mount, InternalKind::InnerTube { outer_diameter, thickness, length, motor_mount: false, overhang }));
        assert!(r.configuration(id).unwrap().motors.is_empty());
        r.set_internal_kind(mount, InternalKind::InnerTube { outer_diameter, thickness, length, motor_mount: true, overhang });
        r.set_motor(id, mount, Some(MotorRef::new("H128W", "Aerotech"))).unwrap();
        r.remove_internal(mount).unwrap();
        assert!(r.configuration(id).unwrap().motors.is_empty());
    }
}
