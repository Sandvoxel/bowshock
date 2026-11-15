//! Saving and loading a [`Rocket`] as pretty JSON.
//!
//! The file is `{ "format_version": N, "rocket": { ... } }`. Loading refuses newer versions and documents whose ids, parents or
//! ordering are inconsistent, so a loaded rocket always obeys the rules the editing API keeps.

use std::collections::HashSet;
use std::fmt;

use serde::{Deserialize, Serialize};

use crate::config::Configuration;
use crate::geometry::{PartKind, MAX_COUNT};
use crate::rocket::Rocket;
use crate::PartId;

/// The version [`Rocket::to_json`] writes and [`Rocket::from_json`] reads.
pub const FORMAT_VERSION: u32 = 1;

#[derive(Serialize)]
struct SaveFile<'a> {
    format_version: u32,
    rocket: &'a Rocket,
}

#[derive(Deserialize)]
struct LoadFile {
    format_version: u32,
    rocket: Rocket,
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

    /// Loads a document written by [`to_json`](Self::to_json) and checks it: unique ids, parents that exist, one nose cone at
    /// the fore end only, symmetry counts in range. Next-id counters are repaired if behind, configuration lists are made
    /// non-empty with a valid active entry, and motors for mounts that do not exist are dropped.
    pub fn from_json(text: &str) -> Result<Rocket, JsonError> {
        let file: LoadFile = serde_json::from_str(text)?;
        if file.format_version != FORMAT_VERSION {
            return Err(JsonError::UnsupportedVersion(file.format_version));
        }
        let mut rocket = file.rocket;
        rocket.repair_and_check().map_err(JsonError::Invalid)?;
        Ok(rocket)
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
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Ignition, MotorRef};
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
        r.set_ignition(first, m_up, Ignition::StageSeparation { delay: 1.5 }).unwrap();
        r.set_stage_separation(first, 1, 0.5).unwrap();
        let second = r.duplicate_configuration(first).unwrap();
        r.rename_configuration(second, "Light");
        r.select_configuration(second);
        r
    }

    #[test]
    fn round_trips_a_full_document() {
        let r = full_rocket();
        let json = r.to_json();
        assert!(json.contains("\"format_version\": 1"));
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
        let future = json.replacen("\"format_version\": 1", "\"format_version\": 99", 1);
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
