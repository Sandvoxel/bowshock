//! Saving and loading a [`Rocket`] as pretty JSON.
//!
//! The file is `{ "format_version": N, "rocket": { ... } }`. Loading refuses newer versions and documents whose ids, parents or
//! ordering are inconsistent, so a loaded rocket always obeys the rules the editing API keeps.
//!
//! # Versions
//!
//! * **2** (current): configurations hold stage events (`events`), per-configuration recovery timing (`recovery`) and ballast.
//! * **1**: a motor slot carried its own `ignition` and a configuration its `stage_separation` delays. They are migrated on
//!   load: a separation delay `d` of stage `n` becomes the separation event of stage `n` (burnout + `d`); a slot lit by
//!   separation becomes the ignition event of its stage (separation + the slot's delay, which is the same instant the version-1
//!   sequence lit it), and a stage lit at launch (including every upper stage without a slot) an ignition at time 0 after launch.

use std::collections::HashSet;
use std::fmt;

use serde::{Deserialize, Serialize};

use crate::config::{ConfigId, Configuration, EventKind, StageEvent, StageTrigger};
use crate::geometry::{PartKind, MAX_COUNT};
use crate::rocket::Rocket;
use crate::PartId;

/// The version [`Rocket::to_json`] writes; [`Rocket::from_json`] reads it and every earlier one (see the module docs).
pub const FORMAT_VERSION: u32 = 2;

#[derive(Serialize)]
struct SaveFile<'a> {
    format_version: u32,
    rocket: &'a Rocket,
}

#[derive(Deserialize)]
struct LoadFile {
    format_version: u32,
    rocket: serde_json::Value,
}

/// What version 1 stored about staging per configuration, taken out of the document before it is read as version 2.
struct LegacyStaging {
    config: ConfigId,
    /// `(stage, seconds after burnout)`.
    separation: Vec<(usize, f64)>,
    /// `(mount, Some(delay after separation))` for a slot lit by separation, `None` for one lit at launch.
    ignition: Vec<(PartId, Option<f64>)>,
}

/// Removes the version-1 fields (`stage_separation` and each slot's `ignition`) from every configuration of `rocket`.
fn take_v1_staging(rocket: &mut serde_json::Value) -> Vec<LegacyStaging> {
    let Some(configs) = rocket.get_mut("configs").and_then(serde_json::Value::as_array_mut) else {
        return Vec::new();
    };
    configs
        .iter_mut()
        .filter_map(|c| {
            let object = c.as_object_mut()?;
            let config = object.get("id")?.as_u64()?;
            let separation = object
                .remove("stage_separation")
                .and_then(|v| v.as_array().cloned())
                .unwrap_or_default()
                .iter()
                .filter_map(|e| Some((usize::try_from(e.get(0)?.as_u64()?).ok()?, e.get(1)?.as_f64()?)))
                .collect();
            let mut ignition = Vec::new();
            for slot in object.get_mut("motors").and_then(serde_json::Value::as_array_mut).into_iter().flatten() {
                let Some(slot) = slot.as_object_mut() else {
                    continue;
                };
                let (Some(mount), Some(rule)) = (slot.get("mount").and_then(serde_json::Value::as_u64), slot.remove("ignition")) else {
                    continue;
                };
                let delay = rule.get("StageSeparation").and_then(|s| s.get("delay")).and_then(serde_json::Value::as_f64);
                ignition.push((mount, delay));
            }
            Some(LegacyStaging { config, separation, ignition })
        })
        .collect()
}

/// Why a document could not be loaded.
#[derive(Debug)]
pub enum JsonError {
    /// Not valid JSON, or not the shape of a rocket document.
    Parse(serde_json::Error),
    /// Written by a newer (or unknown) version of the format.
    UnsupportedVersion(u32),
    /// Well-formed but inconsistent, with what is wrong.
    Invalid(String),
}

impl fmt::Display for JsonError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JsonError::Parse(e) => write!(f, "not a rocket document: {e}"),
            JsonError::UnsupportedVersion(v) => write!(f, "unsupported document version {v} (this build reads version {FORMAT_VERSION})"),
            JsonError::Invalid(why) => write!(f, "inconsistent rocket document: {why}"),
        }
    }
}

impl std::error::Error for JsonError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            JsonError::Parse(e) => Some(e),
            _ => None,
        }
    }
}

impl From<serde_json::Error> for JsonError {
    fn from(e: serde_json::Error) -> Self {
        JsonError::Parse(e)
    }
}

impl Rocket {
    /// The document as pretty-printed JSON with a `format_version` field. A plugged motor's infinite delay is written `null`.
    pub fn to_json(&self) -> String {
        // Every value is a plain number, string or enum; serialising cannot fail (non-finite floats are written as null).
        serde_json::to_string_pretty(&SaveFile { format_version: FORMAT_VERSION, rocket: self }).expect("rocket documents always serialise")
    }

    /// Loads a document written by [`to_json`](Self::to_json) (any version, see the module docs) and checks it: unique ids,
    /// parents that exist, one nose cone at the fore end only, symmetry counts in range. Next-id counters are repaired if
    /// behind, configuration lists are made non-empty with a valid active entry, and motors for mounts that do not exist are
    /// dropped.
    pub fn from_json(text: &str) -> Result<Rocket, JsonError> {
        let mut file: LoadFile = serde_json::from_str(text)?;
        if file.format_version == 0 || file.format_version > FORMAT_VERSION {
            return Err(JsonError::UnsupportedVersion(file.format_version));
        }
        let legacy = if file.format_version < 2 { take_v1_staging(&mut file.rocket) } else { Vec::new() };
        let mut rocket: Rocket = serde_json::from_value(file.rocket)?;
        rocket.repair_and_check().map_err(JsonError::Invalid)?;
        rocket.migrate_v1_staging(&legacy);
        Ok(rocket)
    }

    /// Turns what version 1 stored about staging into stage events (see the module docs). Runs on the checked rocket, since
    /// finding a slot's stage needs the stages.
    fn migrate_v1_staging(&mut self, legacy: &[LegacyStaging]) {
        let stages = self.stages();
        for old in legacy {
            let Some(config) = self.configs.iter().position(|c| c.id == old.config) else {
                continue;
            };
            let mut events: Vec<StageEvent> =
                old.separation.iter().map(|&(stage, delay)| StageEvent { stage, kind: EventKind::Separation, trigger: StageTrigger::Burnout, delay }).collect();
            for stage in stages.iter().filter(|s| s.index > 1) {
                // The slot that flies in the stage is the first mount that holds a motor.
                let slot = stage.mounts.iter().find_map(|id| self.configs[config].slot(*id).filter(|s| s.motor.is_some()).map(|_| *id));
                let rule = slot.and_then(|id| old.ignition.iter().find(|(mount, _)| *mount == id)).and_then(|(_, rule)| *rule);
                let (trigger, delay) = match rule {
                    Some(delay) => (StageTrigger::Separation, delay),
                    None => (StageTrigger::TimeFromLaunch, 0.0),
                };
                events.push(StageEvent { stage: stage.index, kind: EventKind::Ignition, trigger, delay });
            }
            self.configs[config].events = events;
        }
    }

    fn repair_and_check(&mut self) -> Result<(), String> {
        let mut seen: HashSet<PartId> = HashSet::new();
        let ids = self.parts.iter().map(|p| p.id).chain(self.radials.iter().map(|r| r.id)).chain(self.internals.iter().map(|i| i.id));
        for id in ids {
            if !seen.insert(id) {
                return Err(format!("part id {id} is used twice"));
            }
        }
        self.next_id = self.next_id.max(seen.iter().max().map_or(0, |m| m + 1));

        for (i, p) in self.parts.iter().enumerate() {
            let sizes = match p.kind {
                PartKind::NoseCone { length, diameter, .. } | PartKind::BodyTube { length, diameter } => [length, diameter, 0.0],
                PartKind::Transition { length, fore_diameter, aft_diameter } => [length, fore_diameter, aft_diameter],
            };
            if sizes.iter().any(|s| !s.is_finite() || *s < 0.0) || !p.thickness.is_finite() || p.thickness < 0.0 {
                return Err(format!("part {} has a negative or non-finite size", p.id));
            }
            if matches!(p.kind, PartKind::NoseCone { .. }) && i != 0 {
                return Err(format!("nose cone {} is not at the fore end", p.id));
            }
        }
        if self.parts.len() > 1 && self.parts[1..].iter().any(|p| matches!(p.kind, PartKind::NoseCone { .. })) {
            return Err("more than one nose cone".to_string());
        }
        for r in &self.radials {
            if self.part(r.parent).is_none() {
                return Err(format!("radial part {} is on {}, which is not an airframe part", r.id, r.parent));
            }
            if r.count == 0 || r.count > MAX_COUNT {
                return Err(format!("radial part {} has {} copies", r.id, r.count));
            }
        }
        // Per-copy overrides that name a copy or field that does not exist are dropped.
        for r in &mut self.radials {
            r.prune_overrides();
        }
        for i in &self.internals {
            if self.part(i.parent).is_none() {
                return Err(format!("internal part {} is in {}, which is not an airframe part", i.id, i.parent));
            }
        }

        if self.configs.is_empty() {
            self.configs.push(Configuration::new(self.next_config_id, ""));
        }
        let mut config_ids: HashSet<u64> = HashSet::new();
        for c in &self.configs {
            if !config_ids.insert(c.id) {
                return Err(format!("configuration id {} is used twice", c.id));
            }
        }
        self.next_config_id = self.next_config_id.max(config_ids.iter().max().map_or(0, |m| m + 1));
        if !config_ids.contains(&self.active_config) {
            self.active_config = self.configs[0].id;
        }
        self.prune_configs();
        self.repair_flights();
        self.repair_formulas();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Ballast, EventKind, MotorRef, StageEvent, StageTrigger};
    use crate::geometry::{FinSection, NoseShape, RadialKind};
    use crate::internal::{DeployEvent, Deployment, InternalKind, MassRole};
    use crate::material::{Finish, MaterialId};

    /// A rocket using every kind of part and every optional field.
    fn full_rocket() -> Rocket {
        let mut r = Rocket { name: "Test rocket".to_string(), designer: "Bowshock".to_string(), ..Rocket::default() };
        let nose = r.insert(0, PartKind::NoseCone { shape: NoseShape::Haack(1.0 / 3.0), length: 0.3, diameter: 0.0762 }).unwrap();
        let upper = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        r.insert(2, PartKind::Transition { length: 0.1, fore_diameter: 0.0762, aft_diameter: 0.054 }).unwrap();
        let booster = r.insert(3, PartKind::BodyTube { length: 0.4, diameter: 0.054 }).unwrap();
        r.set_name(nose, Some("Nose".to_string()));
        r.set_finish(upper, Finish::SmoothPaint);
        r.set_material(upper, MaterialId::FIBERGLASS);
        r.set_thickness(upper, 0.002);
        r.set_mass_override(upper, Some(0.5));
        r.set_cg_override(upper, Some(0.2));

        let fins = r.attach_radial(booster, 0.2, 0.3, 3, RadialKind::EllipticalFin { root_chord: 0.1, span: 0.06, thickness: 0.003 }).unwrap();
        r.set_fin_section(fins, FinSection::Airfoil);
        r.set_cant(fins, 0.02);
        r.attach_radial(upper, 0.1, 0.0, 2, RadialKind::FreeformFin { points: vec![(0.0, 0.0), (0.04, 0.05), (0.1, 0.0)], thickness: 0.003 }).unwrap();
        r.attach_radial(upper, 0.1, 1.0, 1, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap();
        r.attach_radial(upper, 0.3, 1.0, 2, RadialKind::RailButton { height: 0.008, diameter: 0.011 }).unwrap();

        let m_up = r
            .attach_internal(upper, 0.3, InternalKind::InnerTube { outer_diameter: 0.054, thickness: 0.0016, length: 0.2, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let m_low = r
            .attach_internal(booster, 0.2, InternalKind::InnerTube { outer_diameter: 0.038, thickness: 0.0016, length: 0.2, motor_mount: true, overhang: 0.01 })
            .unwrap();
        r.attach_internal(upper, 0.45, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 }).unwrap();
        r.attach_internal(upper, 0.0, InternalKind::Bulkhead { diameter: Some(0.07), thickness: 0.006 }).unwrap();
        r.attach_internal(upper, 0.05, InternalKind::Coupler { outer_diameter: None, thickness: 0.002, length: 0.1 }).unwrap();
        r.attach_internal(booster, 0.0, InternalKind::EngineBlock { outer: None, inner: 0.02, thickness: 0.006 }).unwrap();
        r.attach_internal(
            upper,
            0.1,
            InternalKind::Parachute {
                diameter: 0.5,
                cd: 0.8,
                line_count: 6,
                line_length: 0.5,
                line_material: MaterialId::KEVLAR_LINE,
                packed_length: 0.1,
                packed_diameter: Some(0.05),
                deploy: Deployment { event: DeployEvent::Altitude(150.0), delay: 1.0 },
            },
        )
        .unwrap();
        r.attach_internal(
            upper,
            0.2,
            InternalKind::Streamer { length: 1.0, width: 0.05, cd: 0.4, deploy: Deployment { event: DeployEvent::Ejection, delay: 0.0 } },
        )
        .unwrap();
        r.attach_internal(upper, 0.25, InternalKind::ShockCord { length: 2.0, packed_length: 0.04 }).unwrap();
        let altimeter =
            r.attach_internal(upper, 0.05, InternalKind::MassComponent { mass: 0.05, length: 0.08, diameter: None, role: MassRole::Altimeter }).unwrap();
        r.set_material(altimeter, MaterialId::BRASS);

        let first = r.active_config_id();
        r.set_motor(first, m_up, Some(MotorRef::new("H128W", "Aerotech"))).unwrap();
        r.set_motor(first, m_low, Some(MotorRef::new("I284W", "Aerotech"))).unwrap();
        r.set_motor_delay(first, m_up, f64::INFINITY).unwrap();
        let event = |stage, kind, trigger, delay| StageEvent { stage, kind, trigger, delay };
        r.set_stage_event(first, event(2, EventKind::Ignition, StageTrigger::Separation, 1.5)).unwrap();
        r.set_stage_event(first, event(1, EventKind::Separation, StageTrigger::Altitude(120.0), 0.5)).unwrap();
        let chute = r.internals().iter().find(|p| matches!(p.kind, InternalKind::Parachute { .. })).unwrap().id;
        r.set_deployment(first, chute, Deployment { event: DeployEvent::TimeFromLaunch, delay: 12.0 }).unwrap();
        r.set_ballast(first, Some(Ballast { mass: 0.04, station: 0.6 })).unwrap();
        let second = r.duplicate_configuration(first).unwrap();
        r.rename_configuration(second, "Light");
        r.select_configuration(second);
        r
    }

    #[test]
    fn round_trips_a_full_document() {
        let r = full_rocket();
        let json = r.to_json();
        assert!(json.contains(&format!("\"format_version\": {FORMAT_VERSION}")));
        assert!(json.contains('\n'), "pretty printed");
        assert!(json.contains("null"), "plugged delay is null");
        let back = Rocket::from_json(&json).unwrap();
        assert_eq!(back, r);
        assert_eq!(back.active_configuration().name, "Light");
        assert!(back.configuration(back.configurations()[0].id).unwrap().slot(back.motor_mounts()[0].id).unwrap().is_plugged());
        // The loaded document keeps working: new ids do not collide.
        let mut back = back;
        let fresh = back.insert(4, PartKind::BodyTube { length: 0.1, diameter: 0.054 }).unwrap();
        assert!(!r.ids().contains(&fresh));
        // And its derived quantities are the same.
        assert_eq!(back.stages().len(), 2);
        assert_eq!(Rocket::from_json(&r.to_json()).unwrap().dry_mass(), r.dry_mass());
    }

    #[test]
    fn default_rocket_round_trips() {
        let r = Rocket::default();
        assert_eq!(Rocket::from_json(&r.to_json()).unwrap(), r);
    }

    #[test]
    fn rejects_bad_documents() {
        assert!(matches!(Rocket::from_json("not json"), Err(JsonError::Parse(_))));
        assert!(matches!(Rocket::from_json("{}"), Err(JsonError::Parse(_))));
        let json = full_rocket().to_json();
        let future = json.replacen(&format!("\"format_version\": {FORMAT_VERSION}"), "\"format_version\": 99", 1);
        assert!(matches!(Rocket::from_json(&future), Err(JsonError::UnsupportedVersion(99))));
        let unknown_material = json.replace("\"fiberglass\"", "\"unobtainium\"");
        assert!(matches!(Rocket::from_json(&unknown_material), Err(JsonError::Parse(_))));

        let mut r = full_rocket();
        let id = r.parts()[1].id;
        r.parts[2].id = id;
        assert!(matches!(Rocket::from_json(&r.to_json()), Err(JsonError::Invalid(why)) if why.contains("twice")));
        let mut r = full_rocket();
        r.radials[0].parent = 9999;
        assert!(matches!(Rocket::from_json(&r.to_json()), Err(JsonError::Invalid(_))));
        let mut r = full_rocket();
        r.radials[0].count = 0;
        assert!(matches!(Rocket::from_json(&r.to_json()), Err(JsonError::Invalid(_))));
        let mut r = full_rocket();
        r.parts.swap(0, 1);
        assert!(matches!(Rocket::from_json(&r.to_json()), Err(JsonError::Invalid(_))));
    }

    #[test]
    fn repairs_counters_and_configurations() {
        let mut r = full_rocket();
        r.next_id = 0;
        r.next_config_id = 0;
        r.configs[0].motors.push(crate::config::MotorSlot::empty(12345));
        let back = Rocket::from_json(&r.to_json()).unwrap();
        assert!(back.next_id > *back.ids().iter().max().unwrap());
        assert!(back.next_config_id > back.configurations().iter().map(|c| c.id).max().unwrap());
        assert!(back.configurations()[0].slot(12345).is_none());
        let mut r = full_rocket();
        r.configs.clear();
        assert_eq!(Rocket::from_json(&r.to_json()).unwrap().configurations().len(), 1);
        let mut r = full_rocket();
        r.active_config = 4242;
        let back = Rocket::from_json(&r.to_json()).unwrap();
        assert_eq!(back.active_config_id(), back.configurations()[0].id);
    }
}

#[cfg(test)]
mod migration_tests {
    use super::*;
    use crate::config::MotorRef;
    use crate::internal::InternalKind;
    use serde_json::{json, Value};

    /// A two-stage rocket with motors in both mounts: `(rocket, upper mount, lower mount)`.
    fn two_stage() -> (Rocket, PartId, PartId) {
        let mut r = Rocket::default();
        let upper = r.insert(0, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let lower = r.insert(1, PartKind::BodyTube { length: 0.4, diameter: 0.0762 }).unwrap();
        let tube = |length| InternalKind::InnerTube { outer_diameter: 0.054, thickness: 0.0016, length, motor_mount: true, overhang: 0.0 };
        let m_up = r.attach_internal(upper, 0.2, tube(0.2)).unwrap();
        let m_low = r.attach_internal(lower, 0.1, tube(0.2)).unwrap();
        let id = r.active_config_id();
        r.set_motor(id, m_up, Some(MotorRef::new("H128W", "Aerotech"))).unwrap();
        r.set_motor(id, m_low, Some(MotorRef::new("I284W", "Aerotech"))).unwrap();
        assert_eq!(r.stages().len(), 2);
        (r, m_up, m_low)
    }

    /// The document `r` would have been saved as by version 1: no events, recovery or ballast, `stage_separation` delays and an
    /// `ignition` per slot.
    fn as_v1(r: &Rocket, separation: Value, ignitions: &[(PartId, Value)]) -> String {
        let mut doc: Value = serde_json::from_str(&r.to_json()).unwrap();
        doc["format_version"] = json!(1);
        for config in doc["rocket"]["configs"].as_array_mut().unwrap() {
            let config = config.as_object_mut().unwrap();
            for key in ["events", "recovery", "ballast"] {
                config.remove(key);
            }
            config.insert("stage_separation".into(), separation.clone());
            for slot in config["motors"].as_array_mut().unwrap() {
                let mount = slot["mount"].as_u64().unwrap();
                slot["ignition"] = ignitions.iter().find(|(m, _)| *m == mount).map_or(json!("Launch"), |(_, v)| v.clone());
            }
        }
        serde_json::to_string(&doc).unwrap()
    }

    #[test]
    fn current_documents_say_version_2() {
        assert_eq!(FORMAT_VERSION, 2);
        assert!(Rocket::default().to_json().contains("\"format_version\": 2"));
    }

    #[test]
    fn version_1_separation_delays_and_separation_ignition_become_stage_events() {
        let (r, m_up, _) = two_stage();
        let v1 = as_v1(&r, json!([[1, 0.5]]), &[(m_up, json!({ "StageSeparation": { "delay": 1.5 } }))]);
        let back = Rocket::from_json(&v1).unwrap();
        let config = back.active_configuration();
        let separation = config.event(1, EventKind::Separation);
        assert_eq!((separation.trigger, separation.delay), (StageTrigger::Burnout, 0.5));
        let ignition = config.event(2, EventKind::Ignition);
        assert_eq!((ignition.trigger, ignition.delay), (StageTrigger::Separation, 1.5));
        assert!(config.ballast.is_none() && config.recovery.is_empty());
        // Saved again it is a version-2 document that reads back the same.
        let again = Rocket::from_json(&back.to_json()).unwrap();
        assert_eq!(again, back);
    }

    #[test]
    fn version_1_upper_stage_lit_at_launch_stays_lit_at_launch() {
        let (r, _, _) = two_stage();
        let back = Rocket::from_json(&as_v1(&r, json!([]), &[])).unwrap();
        let ignition = back.active_configuration().event(2, EventKind::Ignition);
        assert_eq!((ignition.trigger, ignition.delay), (StageTrigger::TimeFromLaunch, 0.0));
        // The default would have been to light on separation: the migration keeps what the old document meant.
        assert_ne!(ignition, StageEvent::default_of(2, EventKind::Ignition));
        assert_eq!(back.active_configuration().event(1, EventKind::Separation), StageEvent::default_of(1, EventKind::Separation));
    }

    #[test]
    fn rejects_version_0_and_newer_than_this_build() {
        let json = Rocket::default().to_json();
        for bad in ["0", "3"] {
            let text = json.replacen(&format!("\"format_version\": {FORMAT_VERSION}"), &format!("\"format_version\": {bad}"), 1);
            assert!(matches!(Rocket::from_json(&text), Err(JsonError::UnsupportedVersion(_))), "{bad}");
        }
    }
}
