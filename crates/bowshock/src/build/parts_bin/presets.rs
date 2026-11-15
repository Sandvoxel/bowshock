//! Presets: saved assemblies (a fin can, a mount with its rings) that the bin adds in one click.
//!
//! A preset is a small JSON document. Built-in ones live in `assets/presets/` and are embedded; the user's are files in
//! [`user_dir`] (`$BOWSHOCK_PRESETS_DIR`, else `~/.config/bowshock/presets`). A preset with no axial parts is a set of radial
//! and internal parts for whatever tube it is added to; one with axial parts adds them at the aft end of the stack.

use std::path::{Path, PathBuf};
use std::{env, fs, io};

use bowshock_model::{Finish, InternalKind, MaterialId, PartId, PartKind, RadialKind, Rocket};
use serde::{Deserialize, Serialize};

use super::items::{self, Zone};
use crate::app::{Ctx, Selection};

/// An axial part of a preset.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AxialSpec {
    /// Geometry.
    pub kind: PartKind,
    /// User-given name.
    #[serde(default)]
    pub name: Option<String>,
    /// Material.
    pub material: MaterialId,
    /// Outside finish.
    pub finish: Finish,
    /// Wall thickness, metres.
    pub thickness: f64,
}

/// A radial group of a preset.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RadialSpec {
    /// The axial part it sits on: an index into the preset's axial parts (0 when it has none: the tube it is added to).
    #[serde(default)]
    pub host: usize,
    /// Geometry.
    pub kind: RadialKind,
    /// Metres from the host's fore face (from its aft face when `from_aft`: the aft end of the part at that distance
    /// from the aft end of the host).
    pub offset: f64,
    /// Measure `offset` from the host's aft end instead of its fore end.
    #[serde(default)]
    pub from_aft: bool,
    /// Angle of the first copy, radians.
    #[serde(default)]
    pub angle: f64,
    /// Copies.
    pub count: u8,
    /// Material.
    pub material: MaterialId,
}

/// An internal part of a preset.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InternalSpec {
    /// The axial part it sits in: an index into the preset's axial parts (0 when it has none: the tube it is added to).
    #[serde(default)]
    pub host: usize,
    /// Geometry.
    pub kind: InternalKind,
    /// Metres from the host's fore face; from the host's aft end, to the part's aft edge, when `from_aft`.
    pub offset: f64,
    /// Measure `offset` from the host's aft end instead of its fore end.
    #[serde(default)]
    pub from_aft: bool,
    /// User-given name.
    #[serde(default)]
    pub name: Option<String>,
    /// Material.
    pub material: MaterialId,
}

/// A saved assembly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Preset {
    /// The name shown in the bin.
    pub name: String,
    /// Axial parts, fore to aft.
    #[serde(default)]
    pub axial: Vec<AxialSpec>,
    /// Radial groups.
    #[serde(default)]
    pub radials: Vec<RadialSpec>,
    /// Internal parts.
    #[serde(default)]
    pub internals: Vec<InternalSpec>,
}

/// A preset as listed in the bin.
#[derive(Debug, Clone)]
pub struct PresetEntry {
    /// The preset.
    pub preset: Preset,
    /// Shipped with the app (cannot be deleted).
    pub builtin: bool,
    /// The file of a user preset.
    pub file: Option<PathBuf>,
}

impl Preset {
    /// How many parts it adds.
    pub fn part_count(&self) -> usize {
        self.axial.len() + self.radials.len() + self.internals.len()
    }

    /// The size line: `3 parts`.
    pub fn summary(&self) -> String {
        let n = self.part_count();
        format!("{n} part{}", if n == 1 { "" } else { "s" })
    }

    /// Whether it carries fins (drawn as a fin can).
    pub fn has_fins(&self) -> bool {
        self.radials.iter().any(|r| r.kind.is_fin())
    }

    /// Whether it carries a motor mount.
    pub fn has_mount(&self) -> bool {
        self.internals.iter().any(|i| i.kind.is_motor_mount())
    }

    /// Captures part `id` with everything it carries: an axial part with its radial and internal parts, or a radial or internal
    /// part alone (to be added to whatever tube is selected). `None` if `id` is not in the rocket.
    pub fn capture(rocket: &Rocket, id: PartId) -> Option<Preset> {
        if let Some(part) = rocket.part(id) {
            let radials = rocket.radials().iter().filter(|r| r.parent == id);
            let internals = rocket.internals_of(id);
            return Some(Preset {
                name: part.display_name(),
                axial: vec![AxialSpec {
                    kind: part.kind.clone(),
                    name: part.name.clone(),
                    material: part.material,
                    finish: part.finish,
                    thickness: part.thickness,
                }],
                radials: radials
                    .map(|r| RadialSpec {
                        host: 0,
                        kind: r.kind.clone(),
                        offset: r.offset,
                        from_aft: false,
                        angle: r.angle,
                        count: r.count,
                        material: r.material,
                    })
                    .collect(),
                internals: internals
                    .map(|i| InternalSpec { host: 0, kind: i.kind.clone(), offset: i.offset, from_aft: false, name: i.name.clone(), material: i.material })
                    .collect(),
            });
        }
        if let Some(r) = rocket.radial(id) {
            let spec = RadialSpec { host: 0, kind: r.kind.clone(), offset: r.offset, from_aft: false, angle: r.angle, count: r.count, material: r.material };
            return Some(Preset { name: r.display_name(), axial: vec![], radials: vec![spec], internals: vec![] });
        }
        let i = rocket.internal(id)?;
        let spec = InternalSpec { host: 0, kind: i.kind.clone(), offset: i.offset, from_aft: false, name: i.name.clone(), material: i.material };
        Some(Preset { name: i.display_name(), axial: vec![], radials: vec![], internals: vec![spec] })
    }

    /// Why this preset cannot be added to `rocket` right now, if it cannot.
    pub fn unavailable(&self, rocket: &Rocket, selection: Selection) -> Option<&'static str> {
        if self.axial.is_empty() {
            let zone = self.internals.first().map_or(Zone::Aft, |i| items::zone_of(&i.kind));
            return items::host_for(rocket, selection, zone).is_none().then_some("Needs a body tube to go on");
        }
        let mut probe = rocket.clone();
        self.axial.iter().all(|a| Self::insert_axial(&mut probe, a).is_some()).then_some(()).map_or(Some("The airframe has no room for it"), |()| None)
    }

    fn insert_axial(rocket: &mut Rocket, spec: &AxialSpec) -> Option<PartId> {
        let node = rocket.nodes_for(&spec.kind).last().map(|(i, _)| *i)?;
        let kind = rocket.matched(node, &spec.kind);
        let id = rocket.insert(node, kind).ok()?;
        rocket.set_name(id, spec.name.clone());
        rocket.set_material(id, spec.material);
        rocket.set_finish(id, spec.finish);
        rocket.set_thickness(id, spec.thickness);
        Some(id)
    }

    /// Adds the preset to `rocket`: its axial parts at the aft end, everything else on `host` when the preset has no axial
    /// parts. Returns the part to select (the first axial part, else the first part added).
    pub fn apply_to(&self, r: &mut Rocket, host: Option<PartId>) -> Option<PartId> {
        let hosts: Vec<PartId> =
            if self.axial.is_empty() { vec![host?] } else { self.axial.iter().map(|a| Self::insert_axial(r, a)).collect::<Option<_>>()? };
        let mut head = hosts.first().copied().filter(|_| !self.axial.is_empty());
        for spec in &self.radials {
            let host = *hosts.get(spec.host)?;
            let len = r.part(host)?.kind.length();
            let offset = if spec.from_aft { len - spec.kind.extent() - spec.offset } else { spec.offset };
            if let Ok(id) = r.attach_radial(host, offset, spec.angle, spec.count, spec.kind.clone()) {
                r.set_material(id, spec.material);
                head = head.or(Some(id));
            }
        }
        for spec in &self.internals {
            let host = *hosts.get(spec.host)?;
            let len = r.part(host)?.kind.length();
            let kind = items::fitted(&spec.kind, items::host_inner(r, host)?);
            let offset = if spec.from_aft { len - kind.extent() - spec.offset } else { spec.offset };
            if let Ok(id) = r.attach_internal(host, offset, kind) {
                r.set_name(id, spec.name.clone());
                r.set_material(id, spec.material);
                head = head.or(Some(id));
            }
        }
        head
    }

    /// Adds the preset as one undo step (see [`apply_to`](Self::apply_to)) and selects what it added first.
    pub fn apply(&self, ctx: &mut Ctx) -> Option<PartId> {
        let zone = self.internals.first().map_or(Zone::Aft, |i| items::zone_of(&i.kind));
        let host = items::host_for(ctx.doc.rocket(), ctx.selection, zone);
        let first = ctx.doc.edit(&format!("Add {}", self.name.to_lowercase()), |r| self.apply_to(r, host))?;
        ctx.selection = Selection::Part(first);
        ctx.camera_fit |= !self.axial.is_empty();
        Some(first)
    }
}

/// The presets shipped with the app.
pub fn builtin() -> Vec<Preset> {
    [include_str!("../../../assets/presets/three_fin_can.json"), include_str!("../../../assets/presets/mount_38mm.json")]
        .into_iter()
        .map(|text| serde_json::from_str(text).expect("a built-in preset is valid JSON"))
        .collect()
}

/// Where the user's presets live: `$BOWSHOCK_PRESETS_DIR`, else `~/.config/bowshock/presets`.
pub fn user_dir() -> Option<PathBuf> {
    env::var_os("BOWSHOCK_PRESETS_DIR").map(PathBuf::from).or_else(|| env::var_os("HOME").map(|h| Path::new(&h).join(".config/bowshock/presets")))
}

/// The built-in presets, then the user's (by file name). Unreadable files are skipped.
pub fn load_all() -> Vec<PresetEntry> {
    let mut entries: Vec<PresetEntry> = builtin().into_iter().map(|preset| PresetEntry { preset, builtin: true, file: None }).collect();
    if let Some(dir) = user_dir() {
        let mut files: Vec<PathBuf> =
            fs::read_dir(dir).into_iter().flatten().flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|e| e == "json")).collect();
        files.sort();
        for file in files {
            if let Some(preset) = fs::read_to_string(&file).ok().and_then(|t| serde_json::from_str::<Preset>(&t).ok()) {
                entries.push(PresetEntry { preset, builtin: false, file: Some(file) });
            }
        }
    }
    entries
}

/// Writes `preset` into [`user_dir`] under a file name made from its name (numbered if taken). Returns the file.
pub fn save(preset: &Preset) -> io::Result<PathBuf> {
    let dir = user_dir().ok_or_else(|| io::Error::other("no home directory for presets"))?;
    fs::create_dir_all(&dir)?;
    let slug: String = preset.name.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '_' }).collect();
    let slug = if slug.trim_matches('_').is_empty() { "preset".to_string() } else { slug };
    let mut file = dir.join(format!("{slug}.json"));
    for n in 2.. {
        if !file.exists() {
            break;
        }
        file = dir.join(format!("{slug}_{n}.json"));
    }
    let text = serde_json::to_string_pretty(preset).map_err(io::Error::other)?;
    fs::write(&file, text)?;
    Ok(file)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rocket() -> (Rocket, PartId) {
        let mut r = Rocket::default();
        r.insert(0, items::axial_catalogue()[0].1.clone()).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: 0.0762 }).unwrap();
        (r, tube)
    }

    #[test]
    fn built_in_presets_parse() {
        let all = builtin();
        assert_eq!(all.len(), 2);
        assert!(all[0].has_fins() && !all[0].axial.is_empty());
        assert!(all[1].has_mount() && all[1].axial.is_empty());
    }

    #[test]
    fn a_tube_with_its_children_round_trips_through_json_and_applies() {
        let (mut r, tube) = rocket();
        r.attach_radial(tube, 0.48, 0.0, 3, items::radial_catalogue()[0].1.clone()).unwrap();
        r.attach_internal(tube, 0.4, items::structure_catalogue()[0].1.clone()).unwrap();
        let preset = Preset::capture(&r, tube).unwrap();
        assert_eq!((preset.axial.len(), preset.radials.len(), preset.internals.len()), (1, 1, 1));
        let back: Preset = serde_json::from_str(&serde_json::to_string(&preset).unwrap()).unwrap();
        assert_eq!(back, preset);

        let mut fresh = Rocket::default();
        back.apply_to(&mut fresh, None).unwrap();
        assert_eq!((fresh.parts().len(), fresh.radials().len(), fresh.internals().len()), (1, 1, 1));
        assert_eq!(fresh.radials()[0].count, 3);
    }

    #[test]
    fn the_mount_preset_fills_the_aft_tube() {
        let (mut r, tube) = rocket();
        let mount = &builtin()[1];
        assert_eq!(mount.unavailable(&r, Selection::None), None);
        mount.apply_to(&mut r, Some(tube)).unwrap();
        assert_eq!(r.internals().len(), mount.internals.len());
        assert!(r.internals().iter().all(|i| i.parent == tube));
        // The mount is flush with the aft end.
        let m = r.internals().iter().find(|i| i.kind.is_motor_mount()).unwrap();
        assert!((m.offset + m.kind.extent() - 0.6).abs() < 1e-9);
        // Nothing to add it to.
        assert!(mount.unavailable(&Rocket::default(), Selection::None).is_some());
    }

    #[test]
    fn the_fin_can_adds_a_tube_with_three_fins() {
        let (mut r, _) = rocket();
        let can = &builtin()[0];
        assert_eq!(can.unavailable(&r, Selection::None), None);
        can.apply_to(&mut r, None).unwrap();
        assert_eq!((r.parts().len(), r.radials().len()), (3, 1));
        assert_eq!(r.radials()[0].count, 3);
    }
}
