//! The `.bowshock` file: a [`Rocket`] in a small binary container.
//!
//! # Layout
//!
//! | bytes | content |
//! |-------|---------|
//! | 0..8  | the magic `BOWSHOCK` |
//! | 8..10 | the format version, `u16` little-endian ([`FORMAT_VERSION`]) |
//! | 10..  | the document: the [`Rocket`] serialised as CBOR (RFC 8949), then compressed as a raw deflate stream |
//!
//! Loading refuses a file with another magic or an unknown version, a damaged stream, and documents whose ids, parents or
//! ordering are inconsistent, so a loaded rocket always obeys the rules the editing API keeps. There is no reader for any
//! earlier text format.
//!
//! # Versions
//!
//! * **1** (current): the first release of the format.
//!
//! # Changing the format
//!
//! Saved files outlive the build that wrote them, so every change keeps them loading:
//!
//! * **Adding a field** needs no new version: give it `#[serde(default)]` (and `skip_serializing_if` when the default should stay
//!   out of the file) so older files read as before. Do not add `deny_unknown_fields` to a document type.
//! * **Renaming, removing, retyping or restructuring** a field or enum variant breaks the stored shape. Bump [`FORMAT_VERSION`]
//!   (it is the header's `u16`), keep reading the old shape by migrating it in [`Rocket::from_bytes`] (decode the old version
//!   into a `ciborium::Value`, rewrite it, then deserialise), and add a golden file for the new version under `tests/data/`.
//! * `tests/data/v1_example.bowshock` is the committed version-1 golden file; a test in this module loads it, so an accidental
//!   break of the shape fails the build.

use std::collections::HashSet;
use std::fmt;
use std::io::{Read, Write};

use flate2::read::DeflateDecoder;
use flate2::write::DeflateEncoder;
use flate2::Compression;
use serde::Serialize;

use crate::config::Configuration;
use crate::geometry::{PartKind, MAX_COUNT};
use crate::rocket::Rocket;
use crate::PartId;

/// The version [`Rocket::to_bytes`] writes and [`Rocket::from_bytes`] reads.
pub const FORMAT_VERSION: u32 = 1;

/// The first eight bytes of every `.bowshock` file.
pub const MAGIC: &[u8; 8] = b"BOWSHOCK";

/// Bytes before the compressed document: the magic and the `u16` version.
const HEADER_LEN: usize = MAGIC.len() + 2;

/// The largest document a load will inflate (a guard against a decompression bomb, far above any real document).
const MAX_DOCUMENT_BYTES: u64 = 1 << 30;

/// Why a document could not be loaded.
#[derive(Debug)]
pub enum FormatError {
    /// The bytes do not start with the Bowshock magic (or are too short to).
    NotBowshock,
    /// A Bowshock file of a version this build does not read.
    UnsupportedVersion(u32),
    /// The compressed stream or the CBOR inside it is damaged, or is not the shape of a rocket document.
    Corrupt(String),
    /// Well-formed but inconsistent, with what is wrong.
    Invalid(String),
}

impl fmt::Display for FormatError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FormatError::NotBowshock => write!(f, "Not a Bowshock document"),
            FormatError::UnsupportedVersion(v) => {
                write!(f, "Not a Bowshock document this build can read (format version {v}; this build reads version {FORMAT_VERSION})")
            }
            FormatError::Corrupt(why) => write!(f, "damaged Bowshock document: {why}"),
            FormatError::Invalid(why) => write!(f, "inconsistent rocket document: {why}"),
        }
    }
}

impl std::error::Error for FormatError {}

/// Header, then `value` as deflated CBOR.
pub(crate) fn pack<T: Serialize>(value: &T) -> Vec<u8> {
    let mut cbor = Vec::new();
    // Every value is a plain number, string or enum and the sink is a Vec; serialising cannot fail.
    ciborium::into_writer(value, &mut cbor).expect("rocket documents always serialise");
    let mut out = Vec::with_capacity(HEADER_LEN + cbor.len() / 4);
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&(FORMAT_VERSION as u16).to_le_bytes());
    let mut deflate = DeflateEncoder::new(out, Compression::best());
    deflate.write_all(&cbor).expect("writing to a Vec cannot fail");
    deflate.finish().expect("writing to a Vec cannot fail")
}

/// Checks the header and returns the inflated CBOR that follows it.
fn unpack(bytes: &[u8]) -> Result<Vec<u8>, FormatError> {
    if bytes.len() < HEADER_LEN || &bytes[..MAGIC.len()] != MAGIC {
        return Err(FormatError::NotBowshock);
    }
    let version = u32::from(u16::from_le_bytes([bytes[MAGIC.len()], bytes[MAGIC.len() + 1]]));
    if version == 0 || version > FORMAT_VERSION {
        return Err(FormatError::UnsupportedVersion(version));
    }
    let mut cbor = Vec::new();
    let mut inflate = DeflateDecoder::new(&bytes[HEADER_LEN..]).take(MAX_DOCUMENT_BYTES + 1);
    inflate.read_to_end(&mut cbor).map_err(|e| FormatError::Corrupt(e.to_string()))?;
    if cbor.len() as u64 > MAX_DOCUMENT_BYTES {
        return Err(FormatError::Corrupt("the document is implausibly large".to_string()));
    }
    Ok(cbor)
}

impl Rocket {
    /// The document as a `.bowshock` file (see the module docs for the layout). A plugged motor's infinite delay is stored as
    /// the CBOR float it is.
    pub fn to_bytes(&self) -> Vec<u8> {
        pack(self)
    }

    /// Loads a document written by [`to_bytes`](Self::to_bytes) and checks it: unique ids, parents that exist, one nose cone at
    /// the fore end only, symmetry counts in range. Next-id counters are repaired if behind, configuration lists are made
    /// non-empty with a valid active entry, and motors for mounts that do not exist are dropped.
    ///
    /// # Errors
    /// [`FormatError::NotBowshock`] for anything without the magic (including plain JSON), [`FormatError::UnsupportedVersion`],
    /// [`FormatError::Corrupt`] for a damaged file and [`FormatError::Invalid`] for an inconsistent document.
    pub fn from_bytes(bytes: &[u8]) -> Result<Rocket, FormatError> {
        let cbor = unpack(bytes)?;
        let mut rocket: Rocket = ciborium::from_reader(cbor.as_slice()).map_err(|e| FormatError::Corrupt(e.to_string()))?;
        rocket.repair_and_check().map_err(FormatError::Invalid)?;
        Ok(rocket)
    }

    fn repair_and_check(&mut self) -> Result<(), String> {
        let mut seen: HashSet<PartId> = HashSet::new();
        let ids = self.ids();
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
            if self.index_of(r.parent).is_none() {
                return Err(format!("radial part {} is on {}, which is not an airframe part", r.id, r.parent));
            }
            if r.count == 0 || r.count > MAX_COUNT {
                return Err(format!("radial part {} has {} copies", r.id, r.count));
            }
        }
        // Per-copy overrides that name a copy or field that does not exist are dropped.
        self.check_pods()?;
        for r in &mut self.radials {
            r.prune_overrides();
        }
        for i in &self.internals {
            if self.index_of(i.parent).is_none() {
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
        self.repair_plots();
        self.repair_cfd();
        self.repair_formulas();
        Ok(())
    }
}

/// The uncompressed CBOR of `rocket` as lossy text: field names and strings are readable, so tests can check what a document
/// does (not) mention.
#[cfg(test)]
pub(crate) fn cbor_text(rocket: &Rocket) -> String {
    let mut cbor = Vec::new();
    ciborium::into_writer(rocket, &mut cbor).unwrap();
    String::from_utf8_lossy(&cbor).into_owned()
}

/// `rocket` as a loose CBOR value tree, for tests that edit a document the way an older or hand-edited file differs.
#[cfg(test)]
pub(crate) fn to_value(rocket: &Rocket) -> ciborium::Value {
    ciborium::Value::serialized(rocket).unwrap()
}

/// Loads a (possibly edited) value tree as if it were a file.
#[cfg(test)]
pub(crate) fn from_value(value: &ciborium::Value) -> Result<Rocket, FormatError> {
    Rocket::from_bytes(&pack(value))
}

/// The map entry `key` of a CBOR map value, for tests that edit a document value.
#[cfg(test)]
pub(crate) fn entry<'a>(map: &'a mut ciborium::Value, key: &str) -> Option<&'a mut ciborium::Value> {
    map.as_map_mut()?.iter_mut().find(|(k, _)| k.as_text() == Some(key)).map(|(_, v)| v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Ballast, EventKind, MotorRef, StageEvent, StageTrigger};
    use crate::geometry::{FinSection, NoseShape, RadialKind};
    use crate::internal::{DeployEvent, Deployment, InternalKind, MassRole};
    use crate::material::{Finish, MaterialId};

    /// A version-1 file as it was first written (`tests/data/v1_example.bowshock`, the app's `hpr_l1_cert` example). It must
    /// load for as long as this build reads version 1: if this fails, a change broke existing files. Never regenerate it to make
    /// a failure go away; change the format by bumping [`FORMAT_VERSION`] with a migration (see the module docs) and add a new
    /// golden file for the new version beside this one.
    #[test]
    fn the_version_1_golden_file_still_loads() {
        const GOLDEN: &[u8] = include_bytes!("../tests/data/v1_example.bowshock");
        assert_eq!(&GOLDEN[..8], MAGIC);
        assert_eq!(u16::from_le_bytes([GOLDEN[8], GOLDEN[9]]), 1);
        let r = Rocket::from_bytes(GOLDEN).expect("the golden version-1 file loads");
        assert_eq!(r.parts().len(), 3);
        assert_eq!(r.flights().len(), 4);
        assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap(), r);
    }

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
        assert!(r.set_fin_shear_modulus(fins, Some(3.5e9)));
        assert!(r.set_fin_tip_to_tip(fins, true));
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
        let bytes = r.to_bytes();
        assert_eq!(&bytes[..8], MAGIC);
        assert_eq!(u16::from_le_bytes([bytes[8], bytes[9]]) as u32, FORMAT_VERSION);
        let back = Rocket::from_bytes(&bytes).unwrap();
        assert_eq!(back, r);
        assert_eq!(back.active_configuration().name, "Light");
        assert!(back.configuration(back.configurations()[0].id).unwrap().slot(back.motor_mounts()[0].id).unwrap().is_plugged());
        // The loaded document keeps working: new ids do not collide.
        let mut back = back;
        let fresh = back.insert(4, PartKind::BodyTube { length: 0.1, diameter: 0.054 }).unwrap();
        assert!(!r.ids().contains(&fresh));
        // And its derived quantities are the same.
        assert_eq!(back.stages().len(), 2);
        assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap().dry_mass(), r.dry_mass());
    }

    #[test]
    fn default_rocket_round_trips() {
        let r = Rocket::default();
        assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap(), r);
    }

    #[test]
    fn the_file_is_compressed_binary_not_text() {
        let bytes = full_rocket().to_bytes();
        assert!(std::str::from_utf8(&bytes).is_err() || !bytes.contains(&b'{'));
        let cbor = unpack(&bytes).unwrap();
        assert!(bytes.len() < cbor.len(), "deflate shrinks the document ({} < {})", bytes.len(), cbor.len());
    }

    #[test]
    fn rejects_foreign_and_damaged_files() {
        let good = full_rocket().to_bytes();
        // Not ours: empty, plain JSON (the old format), a wrong magic.
        assert!(matches!(Rocket::from_bytes(&[]), Err(FormatError::NotBowshock)));
        assert!(matches!(Rocket::from_bytes(b"{\"format_version\":2,\"rocket\":{}}"), Err(FormatError::NotBowshock)));
        let mut wrong_magic = good.clone();
        wrong_magic[0] = b'X';
        assert!(matches!(Rocket::from_bytes(&wrong_magic), Err(FormatError::NotBowshock)));
        assert_eq!(FormatError::NotBowshock.to_string(), "Not a Bowshock document");
        // Unknown versions.
        for version in [0u16, FORMAT_VERSION as u16 + 1] {
            let mut bytes = good.clone();
            bytes[8..10].copy_from_slice(&version.to_le_bytes());
            assert!(matches!(Rocket::from_bytes(&bytes), Err(FormatError::UnsupportedVersion(v)) if v == u32::from(version)));
        }
        // Right header, damaged body: truncated, garbage, valid deflate of non-CBOR, valid CBOR of the wrong shape.
        assert!(matches!(Rocket::from_bytes(&good[..good.len() / 2]), Err(FormatError::Corrupt(_))));
        let mut garbage = good[..10].to_vec();
        garbage.extend_from_slice(&[0xff; 64]);
        assert!(matches!(Rocket::from_bytes(&garbage), Err(FormatError::Corrupt(_))));
        assert!(matches!(Rocket::from_bytes(&pack(&"just a string")), Err(FormatError::Corrupt(_))));
        assert!(matches!(Rocket::from_bytes(&good[..10]), Err(FormatError::Corrupt(_))));
    }

    #[test]
    fn rejects_inconsistent_documents() {
        let mut r = full_rocket();
        let id = r.parts()[1].id;
        r.parts[2].id = id;
        assert!(matches!(Rocket::from_bytes(&r.to_bytes()), Err(FormatError::Invalid(why)) if why.contains("twice")));
        let mut r = full_rocket();
        r.radials[0].parent = 9999;
        assert!(matches!(Rocket::from_bytes(&r.to_bytes()), Err(FormatError::Invalid(_))));
        let mut r = full_rocket();
        r.radials[0].count = 0;
        assert!(matches!(Rocket::from_bytes(&r.to_bytes()), Err(FormatError::Invalid(_))));
        let mut r = full_rocket();
        r.parts.swap(0, 1);
        assert!(matches!(Rocket::from_bytes(&r.to_bytes()), Err(FormatError::Invalid(_))));
    }

    #[test]
    fn repairs_counters_and_configurations() {
        let mut r = full_rocket();
        r.next_id = 0;
        r.next_config_id = 0;
        r.configs[0].motors.push(crate::config::MotorSlot::empty(12345));
        let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
        assert!(back.next_id > *back.ids().iter().max().unwrap());
        assert!(back.next_config_id > back.configurations().iter().map(|c| c.id).max().unwrap());
        assert!(back.configurations()[0].slot(12345).is_none());
        let mut r = full_rocket();
        r.configs.clear();
        assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap().configurations().len(), 1);
        let mut r = full_rocket();
        r.active_config = 4242;
        let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
        assert_eq!(back.active_config_id(), back.configurations()[0].id);
    }
}
