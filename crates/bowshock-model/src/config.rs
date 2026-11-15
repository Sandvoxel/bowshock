//! Motor configurations: one rocket, several ways to fly it. A configuration names what goes in each motor mount and how the
//! flight is timed; the airframe is shared. The model stores only motor *references*: thrust curves and masses come from the
//! motor database.

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::internal::InternalKind;
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

/// When a motor lights.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Ignition {
    /// At launch.
    Launch,
    /// After the stage below separates, plus `delay` seconds.
    StageSeparation {
        /// Seconds after separation.
        delay: f64,
    },
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
    /// When it lights.
    pub ignition: Ignition,
}

impl MotorSlot {
    /// An empty slot on `mount`, lit at launch, with no delay.
    pub fn empty(mount: PartId) -> MotorSlot {
        MotorSlot { mount, motor: None, delay: 0.0, ignition: Ignition::Launch }
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
    /// `(stage index, seconds after the stage's burnout)` at which the stage separates, for stages that have an entry
    /// (see [`Rocket::stages`](crate::Rocket::stages); a missing entry separates at burnout).
    pub stage_separation: Vec<(usize, f64)>,
}

impl Configuration {
    /// An empty configuration.
    pub fn new(id: ConfigId, name: impl Into<String>) -> Configuration {
        Configuration { id, name: name.into(), motors: Vec::new(), stage_separation: Vec::new() }
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

    /// Delay after burnout at which stage `stage` separates (0 when not set).
    pub fn separation_delay(&self, stage: usize) -> f64 {
        self.stage_separation.iter().find(|(s, _)| *s == stage).map_or(0.0, |(_, d)| *d)
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
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ConfigError::NoSuchConfig => "no such configuration",
            ConfigError::NotAMotorMount => "the part is not a motor mount",
            ConfigError::LastConfiguration => "the last configuration cannot be removed",
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

    /// Copies configuration `id` (motors, delays, ignition and separation timing) and returns the copy's id; a named
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

    /// Sets when the motor in `mount` lights.
    pub fn set_ignition(&mut self, config: ConfigId, mount: PartId, ignition: Ignition) -> Result<(), ConfigError> {
        self.slot_mut(config, mount)?.ignition = ignition;
        Ok(())
    }

    /// Sets the delay after burnout at which stage `stage` separates in configuration `config`.
    pub fn set_stage_separation(&mut self, config: ConfigId, stage: usize, delay: f64) -> Result<(), ConfigError> {
        let c = self.configs.iter_mut().find(|c| c.id == config).ok_or(ConfigError::NoSuchConfig)?;
        match c.stage_separation.iter_mut().find(|(s, _)| *s == stage) {
            Some(entry) => entry.1 = delay,
            None => c.stage_separation.push((stage, delay)),
        }
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

    /// The motor mounts (inner tubes with `motor_mount`), fore to aft by their fore end.
    pub fn motor_mounts(&self) -> Vec<MotorMount> {
        let stations = self.stations();
        let mut mounts: Vec<MotorMount> = self
            .internals
            .iter()
            .filter_map(|p| {
                let InternalKind::InnerTube { outer_diameter, thickness, length, motor_mount: true, overhang } = p.kind else {
                    return None;
                };
                let fore = stations[self.index_of(p.parent)?] + p.offset;
                Some(MotorMount {
                    id: p.id,
                    parent: p.parent,
                    fore_station: fore,
                    aft_station: fore + length,
                    overhang,
                    motor_aft_station: fore + length + overhang,
                    inner_diameter: (outer_diameter - 2.0 * thickness).max(0.0),
                    length,
                })
            })
            .collect();
        mounts.sort_by(|a, b| a.fore_station.total_cmp(&b.fore_station).then(a.id.cmp(&b.id)));
        mounts
    }

    /// Drops motor slots whose mount is no longer a motor mount.
    pub(crate) fn prune_configs(&mut self) {
        let mounts: Vec<PartId> = self.internals.iter().filter(|p| p.kind.is_motor_mount()).map(|p| p.id).collect();
        for c in &mut self.configs {
            c.motors.retain(|s| mounts.contains(&s.mount));
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
        r.set_ignition(id, mount, Ignition::StageSeparation { delay: 0.5 }).unwrap();
        r.set_stage_separation(id, 1, 0.25).unwrap();
        r.set_stage_separation(id, 1, 0.5).unwrap();
        assert_eq!(r.active_configuration().separation_delay(1), 0.5);
        assert_eq!(r.active_configuration().separation_delay(2), 0.0);
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
