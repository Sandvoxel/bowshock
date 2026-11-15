//! Import of OpenRocket `.ork` designs into a [`Rocket`] document.
//!
//! An `.ork` is XML (`<openrocket><rocket>…</rocket><simulations>…</simulations></openrocket>`), normally stored as the
//! `rocket.ork` entry of a ZIP archive; older files are a bare XML or gzip stream. [`import`] reads all three. Everything the
//! document model can express is mapped; everything else becomes an entry of [`OrkImport::warnings`] (nothing is dropped
//! silently).
//!
//! # Mapping
//!
//! * **Stages.** OpenRocket stacks `<stage>`s, each a list of body components. The model has one stack and derives its stages
//!   from the motor mounts, so every stage's nose cone, body tubes and transitions are concatenated, top stage first. Stage
//!   *separation* and motor *ignition* events become the configurations' [`StageEvent`]s: `burnout` / `ejection` (burnout plus
//!   the motor's ejection delay) / `launch` / `automatic`; `upperignition` separation is approximated by burnout.
//! * **Body.** Nose cones (ogive with parameter 1, conical, ellipsoid, power, parabolic, Haack), body tubes, conical
//!   transitions, with wall thickness (`filled` is solid), shoulders and finish (`normal` is regular paint, `rough`, `unfinished`,
//!   `smooth`, `polished`).
//! * **Radial parts.** Trapezoidal, elliptical and freeform fin sets (count, cant, cross-section, thickness, fillet, position),
//!   launch lugs and rail buttons (linear `instancecount`/`instanceseparation` instances become one group each), pod sets and
//!   parallel stages (as pods), tube fin sets (as a pod of one open tube).
//! * **Internal parts.** Inner tubes (motor mounts with their overhang; a body tube that is its own motor mount gets a massless
//!   inner tube of its bore and length to hold the motor, with a warning), centering rings, bulkheads, tube couplers, engine
//!   blocks, parachutes, streamers, shock cord and mass components, placed exactly where OpenRocket puts them (the part's OpenRocket
//!   parent keeps it, even when it overhangs a joint).
//! * **Materials.** The model's material table is fixed. A part takes the table entry of the same kind nearest in density; when
//!   the densities differ the part's mass is *overridden* with OpenRocket's mass (`density × the model's own volume`), so mass
//!   and CG match. Components with an OpenRocket mass override carry it (a group's override is for the whole group; with
//!   "override subcomponents" the children are zeroed). CG overrides carry over too.
//! * **Configurations.** Each OpenRocket flight configuration becomes a [`Configuration`](crate::Configuration) (motor
//!   designation and manufacturer, ejection delay, `none` is a plugged motor, ignition and separation events, per-configuration
//!   recovery deployment). Thrust curves are not in an `.ork`; the motors are listed in [`OrkImport::motors`] to be resolved
//!   against a motor database (e.g. RASP files exported by `tools/openrocket-harness`).
//! * **Simulations.** Each stored simulation becomes a [`FlightDef`] (rail, wind, site, turbulence) and an [`OrkSimulation`] with
//!   the values the model has no field for, plus OpenRocket's stored result summary.
//!
//! Not modelled (reported as warnings): clustered motors beyond the first tube, secant ogives, non-conical transitions,
//! fin tabs (mass only), motor mounts in body tubes, drag overrides, custom reference lengths, "never" deployment/ignition.
//!
//! References: the OpenRocket file format (`net.sf.openrocket.file.openrocket`), Niskanen (2013) *OpenRocket technical
//! documentation* for the component definitions.

use std::collections::HashMap;
use std::fmt;
use std::io::Read;
use std::path::Path;

use roxmltree::{Document, Node};

use crate::config::{ConfigId, EventKind, MotorRef, StageEvent, StageTrigger};
use crate::flights::{Conditions, FlightDef, FlightMotor, Turbulence};
use crate::geometry::{FinSection, NoseShape, Part, PartKind, PodBody, RadialKind, RadialPart, Shoulder};
use crate::internal::{DeployEvent, Deployment, InternalKind, InternalPart, MassRole};
use crate::material::{materials_of_kind, Finish, MaterialId, MaterialKind};
use crate::rocket::Rocket;
use crate::PartId;

/// Why a file could not be imported.
#[derive(Debug)]
pub enum OrkError {
    /// The file could not be read.
    Io(String),
    /// The ZIP or gzip container is damaged or unsupported.
    Archive(String),
    /// The XML is malformed.
    Xml(String),
    /// The XML is not an OpenRocket document, or has no `<rocket>`.
    NotOpenRocket,
}

impl fmt::Display for OrkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OrkError::Io(e) => write!(f, "cannot read the file: {e}"),
            OrkError::Archive(e) => write!(f, "damaged .ork archive: {e}"),
            OrkError::Xml(e) => write!(f, "malformed .ork XML: {e}"),
            OrkError::NotOpenRocket => f.write_str("not an OpenRocket design"),
        }
    }
}

impl std::error::Error for OrkError {}

/// OpenRocket's stored result summary of a simulation (the `<flightdata>` attributes), SI.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct OrkStored {
    /// Apogee above the launch site, m.
    pub max_altitude: Option<f64>,
    /// Peak speed, m/s.
    pub max_velocity: Option<f64>,
    /// Peak acceleration, m/s².
    pub max_acceleration: Option<f64>,
    /// Peak Mach number.
    pub max_mach: Option<f64>,
    /// Time to apogee, s.
    pub time_to_apogee: Option<f64>,
    /// Total flight time, s.
    pub flight_time: Option<f64>,
    /// Speed at ground hit, m/s.
    pub ground_hit_velocity: Option<f64>,
    /// Speed when leaving the launch rod, m/s.
    pub launch_rod_velocity: Option<f64>,
}

/// One stored OpenRocket simulation: its conditions in full.
#[derive(Clone, Debug, PartialEq)]
pub struct OrkSimulation {
    /// Name in OpenRocket.
    pub name: String,
    /// The model configuration it flies (`None` if the file's configuration id is unknown).
    pub config: Option<ConfigId>,
    /// Launch rod length, m.
    pub rod_length: f64,
    /// Rod tilt from vertical, rad.
    pub rod_angle: f64,
    /// Rod azimuth, rad (OpenRocket launches into the wind by default, as the app does).
    pub rod_direction: f64,
    /// Mean wind speed, m/s.
    pub wind_average: f64,
    /// Turbulence intensity (standard deviation as a fraction of the mean wind).
    pub wind_turbulence: f64,
    /// Launch site altitude above sea level, m.
    pub altitude: f64,
    /// Launch latitude, degrees.
    pub latitude: f64,
    /// Whether the atmosphere is the standard one (`isa`) rather than the extended one with a given temperature and pressure.
    pub isa: bool,
    /// Ground temperature, K (the ISA value at the site for the standard atmosphere).
    pub temperature: f64,
    /// Ground pressure, Pa (`None` for the standard atmosphere).
    pub pressure: Option<f64>,
    /// Integration time step, s.
    pub time_step: f64,
    /// OpenRocket's own result, if the file stored one.
    pub stored: Option<OrkStored>,
}

/// The result of an import.
#[derive(Clone, Debug)]
pub struct OrkImport {
    /// The imported document. Its flights are the file's simulations (or the default flight if it has none).
    pub rocket: Rocket,
    /// Everything that could not be carried over exactly, one line each.
    pub warnings: Vec<String>,
    /// Every motor the configurations use, without duplicates.
    pub motors: Vec<MotorRef>,
    /// The stored simulations, in file order, parallel to the rocket's flights when the file has any.
    pub simulations: Vec<OrkSimulation>,
    /// OpenRocket configuration ids (`configid` attributes) in the order of the rocket's configurations.
    pub config_ids: Vec<String>,
}

/// Imports the `.ork` file at `path`.
pub fn import_file(path: impl AsRef<Path>) -> Result<OrkImport, OrkError> {
    let bytes = std::fs::read(path.as_ref()).map_err(|e| OrkError::Io(e.to_string()))?;
    import(&bytes)
}

/// Imports an `.ork` held in memory: a ZIP archive with a `rocket.ork` entry, a gzip stream or bare XML.
pub fn import(bytes: &[u8]) -> Result<OrkImport, OrkError> {
    let xml = unpack(bytes)?;
    let doc = Document::parse(&xml).map_err(|e| OrkError::Xml(e.to_string()))?;
    let root = doc.root_element();
    if root.tag_name().name() != "openrocket" {
        return Err(OrkError::NotOpenRocket);
    }
    let rocket_node = child(root, "rocket").ok_or(OrkError::NotOpenRocket)?;
    let mut warnings = Vec::new();
    let mut ctx = Import::new(&mut warnings);
    ctx.rocket_node(rocket_node);
    ctx.simulations(child(root, "simulations"));
    let Import { b, config_ids, simulations, motors, .. } = ctx;
    let mut rocket = b.r;
    rocket.repair_flights();
    Ok(OrkImport { rocket, warnings, motors, simulations, config_ids })
}

// ---- containers ---------------------------------------------------------------------------------------------------------

fn unpack(bytes: &[u8]) -> Result<String, OrkError> {
    let raw = if bytes.starts_with(b"PK") {
        unzip(bytes)?
    } else if bytes.starts_with(&[0x1f, 0x8b]) {
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(bytes).read_to_end(&mut out).map_err(|e| OrkError::Archive(e.to_string()))?;
        out
    } else {
        bytes.to_vec()
    };
    String::from_utf8(raw).map_err(|e| OrkError::Xml(format!("not UTF-8: {e}")))
}

fn le16(b: &[u8], at: usize) -> Option<usize> {
    Some(usize::from(u16::from_le_bytes(b.get(at..at + 2)?.try_into().ok()?)))
}

fn le32(b: &[u8], at: usize) -> Option<usize> {
    Some(u32::from_le_bytes(b.get(at..at + 4)?.try_into().ok()?) as usize)
}

/// Extracts the design entry of a ZIP: `rocket.ork`, else the first `*.ork` or `*.xml`. Stored and deflated entries.
fn unzip(b: &[u8]) -> Result<Vec<u8>, OrkError> {
    let bad = || OrkError::Archive("truncated ZIP".to_string());
    let last = b.len().checked_sub(22).ok_or_else(bad)?;
    let eocd = (last.saturating_sub(65_535)..=last).rev().find(|&i| b[i..i + 4] == [0x50, 0x4b, 0x05, 0x06]).ok_or_else(bad)?;
    let entries = le16(b, eocd + 10).ok_or_else(bad)?;
    let mut p = le32(b, eocd + 16).ok_or_else(bad)?;
    let mut found: Vec<(String, usize, usize, usize, usize)> = Vec::new();
    for _ in 0..entries {
        if b.get(p..p + 4) != Some(&[0x50, 0x4b, 0x01, 0x02]) {
            return Err(OrkError::Archive("bad central directory".to_string()));
        }
        let method = le16(b, p + 10).ok_or_else(bad)?;
        let csize = le32(b, p + 20).ok_or_else(bad)?;
        let usize_ = le32(b, p + 24).ok_or_else(bad)?;
        let (n, m, k) = (le16(b, p + 28).ok_or_else(bad)?, le16(b, p + 30).ok_or_else(bad)?, le16(b, p + 32).ok_or_else(bad)?);
        let local = le32(b, p + 42).ok_or_else(bad)?;
        let name = String::from_utf8_lossy(b.get(p + 46..p + 46 + n).ok_or_else(bad)?).into_owned();
        found.push((name, method, csize, usize_, local));
        p += 46 + n + m + k;
    }
    let pick = found
        .iter()
        .find(|e| e.0 == "rocket.ork")
        .or_else(|| found.iter().find(|e| e.0.ends_with(".ork")))
        .or_else(|| found.iter().find(|e| e.0.ends_with(".xml")))
        .ok_or_else(|| OrkError::Archive("no rocket.ork in the archive".to_string()))?;
    let (_, method, csize, usize_, local) = pick;
    if b.get(*local..*local + 4) != Some(&[0x50, 0x4b, 0x03, 0x04]) {
        return Err(OrkError::Archive("bad local header".to_string()));
    }
    let start = local + 30 + le16(b, local + 26).ok_or_else(bad)? + le16(b, local + 28).ok_or_else(bad)?;
    let data = b.get(start..start + csize).ok_or_else(bad)?;
    match method {
        0 => Ok(data.to_vec()),
        8 => {
            let mut out = Vec::with_capacity(*usize_);
            flate2::read::DeflateDecoder::new(data).read_to_end(&mut out).map_err(|e| OrkError::Archive(e.to_string()))?;
            Ok(out)
        }
        m => Err(OrkError::Archive(format!("unsupported ZIP compression method {m}"))),
    }
}

// ---- XML helpers --------------------------------------------------------------------------------------------------------

fn child<'a, 'i>(n: Node<'a, 'i>, tag: &str) -> Option<Node<'a, 'i>> {
    n.children().find(|c| c.is_element() && c.tag_name().name() == tag)
}

fn children<'a, 'i>(n: Node<'a, 'i>, tag: &'a str) -> impl Iterator<Item = Node<'a, 'i>> {
    n.children().filter(move |c| c.is_element() && c.tag_name().name() == tag)
}

fn text<'a>(n: Node<'a, '_>, tag: &str) -> Option<&'a str> {
    child(n, tag).and_then(|c| c.text()).map(str::trim)
}

fn num(n: Node, tag: &str) -> Option<f64> {
    text(n, tag).and_then(|t| t.parse::<f64>().ok()).filter(|v| v.is_finite())
}

fn name_of(n: Node) -> Option<String> {
    text(n, "name").filter(|s| !s.is_empty()).map(str::to_string)
}

/// A dimension that may be `auto` (OpenRocket writes `auto <computed value>`).
#[derive(Clone, Copy, Debug, PartialEq)]
enum Dim {
    Auto(Option<f64>),
    Val(f64),
}

impl Dim {
    /// The number if there is one (an explicit value, or the value OpenRocket computed for `auto`).
    fn value(self) -> Option<f64> {
        match self {
            Dim::Auto(v) => v,
            Dim::Val(v) => Some(v),
        }
    }

    /// The explicit value, or `None` for `auto`.
    fn explicit(self) -> Option<f64> {
        match self {
            Dim::Val(v) => Some(v),
            Dim::Auto(_) => None,
        }
    }
}

fn dim(n: Node, tag: &str) -> Option<Dim> {
    let t = text(n, tag)?;
    if let Some(rest) = t.strip_prefix("auto") {
        return Some(Dim::Auto(rest.trim().parse::<f64>().ok().filter(|v| v.is_finite())));
    }
    t.parse::<f64>().ok().filter(|v| v.is_finite()).map(Dim::Val)
}

/// `(method, value)` of an axial offset: `<axialoffset method="top">` (or the older `<position type=…>`).
fn axial_offset(n: Node) -> (String, f64) {
    for tag in ["axialoffset", "position"] {
        if let Some(c) = child(n, tag) {
            let method = c.attribute("method").or_else(|| c.attribute("type")).unwrap_or("top").to_string();
            return (method, c.text().and_then(|t| t.trim().parse().ok()).unwrap_or(0.0));
        }
    }
    ("top".to_string(), 0.0)
}

fn finish_of(n: Node) -> Finish {
    match text(n, "finish") {
        Some("rough") => Finish::Rough,
        Some("unfinished") => Finish::Unfinished,
        Some("smooth") => Finish::SmoothPaint,
        Some("polished") => Finish::Polished,
        _ => Finish::RegularPaint,
    }
}

// ---- builder ------------------------------------------------------------------------------------------------------------

/// Where a component's children are placed: the axial part that holds them in the model, and the span of the OpenRocket parent.
#[derive(Clone, Copy, Debug)]
struct Parent {
    /// The axial part the model parts are attached to.
    axial: PartId,
    /// Station of that part's fore face.
    axial_station: f64,
    /// Station of the OpenRocket parent's top.
    top: f64,
    /// Length of the OpenRocket parent.
    len: f64,
    /// Inner diameter of the OpenRocket parent (what `auto` diameters follow).
    bore: f64,
    /// Whether the OpenRocket parent is itself an internal part (an inner tube or coupler) rather than an airframe part.
    internal: bool,
}

/// A motor in a mount for one OpenRocket configuration.
#[derive(Clone, Debug)]
struct MotorSpec {
    config: String,
    motor: MotorRef,
    /// Ejection delay (infinite: plugged).
    delay: f64,
}

/// An ignition event of a mount for one configuration (`config` empty: the mount's default).
#[derive(Clone, Debug)]
struct IgnitionSpec {
    config: String,
    event: String,
    delay: f64,
}

/// A motor mount found in the file.
#[derive(Clone, Debug)]
struct MountRec {
    part: PartId,
    /// OpenRocket stage index (from the top, 0) of the stage it is in.
    stage: usize,
    motors: Vec<MotorSpec>,
    ignition: Vec<IgnitionSpec>,
}

/// A recovery device with the deployment each configuration gives it.
#[derive(Clone, Debug)]
struct RecoveryRec {
    part: PartId,
    per_config: Vec<(String, Deployment)>,
}

/// Builds one stack (the rocket, or one pod's body) of model parts.
struct Builder<'w> {
    r: Rocket,
    w: &'w mut Vec<String>,
    mounts: Vec<MountRec>,
    recovery: Vec<RecoveryRec>,
    /// OpenRocket stage index of what is being added.
    stage: usize,
    /// Station of the next axial part's fore face.
    x: f64,
    /// Resolved `(fore, aft)` radii of the axial components, by XML node (see [`Builder::plan_radii`]).
    radii: HashMap<roxmltree::NodeId, (f64, f64)>,
    /// Materials already reported as approximated.
    reported: Vec<String>,
}

/// Per-simulation context of an import.
struct Import<'w> {
    b: Builder<'w>,
    /// OpenRocket configuration ids in file order.
    config_ids: Vec<String>,
    config_map: HashMap<String, ConfigId>,
    simulations: Vec<OrkSimulation>,
    motors: Vec<MotorRef>,
    /// `(stage separation default, per-config)` of every OpenRocket stage, by stage index.
    separations: Vec<SeparationRec>,
}

#[derive(Clone, Debug, Default)]
struct SeparationRec {
    default: Option<(String, f64)>,
    per_config: Vec<(String, String, f64)>,
}

fn deg(v: f64) -> f64 {
    v.to_radians()
}

impl<'w> Builder<'w> {
    fn new(w: &'w mut Vec<String>) -> Builder<'w> {
        Builder { r: Rocket::default(), w, mounts: Vec::new(), recovery: Vec::new(), stage: 0, x: 0.0, radii: HashMap::new(), reported: Vec::new() }
    }

    fn warn(&mut self, message: impl Into<String>) {
        let message = message.into();
        if !self.w.contains(&message) {
            self.w.push(message);
        }
    }

    fn fresh(&mut self) -> PartId {
        let id = self.r.next_id;
        self.r.next_id += 1;
        id
    }

    /// The model material of `<material type density>Name</material>` of `n` (the nearest density of the same kind) and the
    /// factor from the model material's density to OpenRocket's.
    fn material(&mut self, n: Node, tag: &str, kind: MaterialKind, default: MaterialId) -> (MaterialId, f64) {
        let Some(m) = child(n, tag) else {
            return (default, 1.0);
        };
        let given = m.attribute("type").unwrap_or("bulk");
        let want = match kind {
            MaterialKind::Bulk => "bulk",
            MaterialKind::Surface => "surface",
            MaterialKind::Line => "line",
        };
        let name = m.text().unwrap_or("").trim().to_string();
        let Some(density) = m.attribute("density").and_then(|d| d.parse::<f64>().ok()).filter(|d| *d > 0.0) else {
            return (default, 1.0);
        };
        if given != want {
            self.warn(format!("material \"{name}\" is {given} stock where {want} stock is needed; using the default material"));
            return (default, 1.0);
        }
        let best = materials_of_kind(kind).min_by(|a, b| (density / a.density).ln().abs().total_cmp(&(density / b.density).ln().abs()));
        let Some(best) = best else {
            return (default, 1.0);
        };
        let ratio = density / best.density;
        if (ratio - 1.0).abs() > 0.02 && !self.reported.contains(&name) {
            self.reported.push(name.clone());
            self.w.push(format!(
                "material \"{name}\" ({density} {}) is not in the model's table: using {} ({}) and scaling the part masses to match",
                kind.density_unit(),
                best.name,
                best.density
            ));
        }
        (best.id, ratio)
    }

    /// Applies OpenRocket's mass and CG overrides of `n` to part `id`; otherwise, when the material density differed from the
    /// model's by `scale` (or the caller knows the OpenRocket mass, `expected`), overrides the mass so it matches.
    fn settle_mass(&mut self, id: PartId, n: Node, scale: f64, expected: Option<f64>) {
        if let Some(m) = num(n, "overridemass") {
            self.r.set_mass_override(id, Some(m));
        } else if let Some(e) = expected {
            let model = self.r.component_mass(id).map_or(0.0, |m| m.mass);
            if model <= 0.0 || (e / model - 1.0).abs() > 0.002 {
                self.r.set_mass_override(id, Some(e));
            }
        } else if (scale - 1.0).abs() > 0.002 {
            if let Some(m) = self.r.component_mass(id) {
                self.r.set_mass_override(id, Some(m.mass * scale));
            }
        }
        if let Some(c) = num(n, "overridecg") {
            self.r.set_cg_override(id, Some(c));
        }
        if child(n, "overridecd").is_some() {
            let name = name_of(n).unwrap_or_else(|| n.tag_name().name().to_string());
            self.warn(format!("{name}: drag override is not modelled"));
        }
    }

    /// Where a component of length `len` with the placement of `n` starts, given its parent and the end of the previous sibling.
    fn place(&self, n: Node, p: &Parent, len: f64, after: f64) -> f64 {
        let (method, v) = axial_offset(n);
        match method.as_str() {
            "middle" => p.top + p.len / 2.0 - len / 2.0 + v,
            "bottom" => p.top + p.len - len + v,
            "absolute" => v,
            "after" => after + v,
            _ => p.top + v,
        }
    }

    // ---- axial stack ----

    /// Resolves the radii of a run of axial components the way OpenRocket does: an `auto` radius follows the neighbour (a
    /// tube's and a transition's fore radius the aft radius of the part ahead, a nose cone's or transition's aft radius the
    /// fore radius of the part behind). The number stored next to `auto` can be stale, so it is only the last resort.
    fn plan_radii(&mut self, nodes: &[Node]) {
        #[derive(Clone, Copy)]
        enum Spec {
            Fixed(f64),
            Auto(Option<f64>),
        }
        let spec = |n: Node, tag: &str| match dim(n, tag) {
            Some(Dim::Val(v)) => Spec::Fixed(v),
            Some(Dim::Auto(v)) => Spec::Auto(v),
            None => Spec::Auto(None),
        };
        let mut items: Vec<(Spec, Spec, bool)> = Vec::new(); // (fore, aft, tube: one radius)
        for n in nodes {
            items.push(match n.tag_name().name() {
                "nosecone" if text(*n, "isflipped") == Some("true") => (spec(*n, "aftradius"), Spec::Fixed(0.0), false),
                "nosecone" => (Spec::Fixed(0.0), spec(*n, "aftradius"), false),
                "bodytube" => (spec(*n, "radius"), spec(*n, "radius"), true),
                _ => (spec(*n, "foreradius"), spec(*n, "aftradius"), false),
            });
        }
        let value = |s: Spec| if let Spec::Fixed(v) = s { Some(v) } else { None };
        let mut r: Vec<(Option<f64>, Option<f64>)> = items.iter().map(|i| (value(i.0), value(i.1))).collect();
        let k = items.len();
        loop {
            let mut changed = true;
            while changed {
                changed = false;
                for i in 0..k {
                    let from_prev = if i > 0 { r[i - 1].1 } else { None };
                    let from_next = if i + 1 < k { r[i + 1].0 } else { None };
                    if items[i].2 {
                        if r[i].0.is_none() {
                            if let Some(v) = from_prev.or(from_next) {
                                r[i] = (Some(v), Some(v));
                                changed = true;
                            }
                        }
                        continue;
                    }
                    if r[i].0.is_none() {
                        if let Some(v) = from_prev {
                            r[i].0 = Some(v);
                            changed = true;
                        }
                    }
                    if r[i].1.is_none() {
                        if let Some(v) = from_next {
                            r[i].1 = Some(v);
                            changed = true;
                        }
                    }
                }
            }
            // Anything still open is cut by its stored number (or a default) and the neighbours follow.
            let Some(i) = (0..k).find(|&i| r[i].0.is_none() || r[i].1.is_none()) else {
                break;
            };
            let stale = |s: Spec| if let Spec::Auto(Some(v)) = s { v } else { 0.0125 };
            if r[i].0.is_none() {
                r[i].0 = Some(stale(items[i].0));
            }
            if r[i].1.is_none() {
                r[i].1 = Some(if items[i].2 { r[i].0.unwrap_or(0.0125) } else { stale(items[i].1) });
            }
        }
        for (n, (fore, aft)) in nodes.iter().zip(r) {
            self.radii.insert(n.id(), (fore.unwrap_or(0.0), aft.unwrap_or(0.0)));
        }
    }

    /// Adds every body component of a `<subcomponents>` (a stage, or a pod body) to the stack.
    fn stack(&mut self, subs: Node) {
        for c in subs.children().filter(Node::is_element) {
            match c.tag_name().name() {
                "nosecone" | "bodytube" | "transition" => self.axial(c),
                other => {
                    let name = name_of(c).unwrap_or_else(|| other.to_string());
                    self.warn(format!("{name}: a {other} directly in a stage is not supported"));
                }
            }
        }
    }

    fn axial(&mut self, n: Node) {
        let tag = n.tag_name().name();
        let name = name_of(n);
        let label = name.clone().unwrap_or_else(|| tag.to_string());
        let length = num(n, "length").unwrap_or(0.0).max(0.0);
        let (rf, ra) = self.radii.get(&n.id()).copied().unwrap_or((0.0125, 0.0125));
        let (kind, shoulders) = match tag {
            "nosecone" if text(n, "isflipped") == Some("true") => {
                self.warn(format!("{label}: a flipped nose cone (tail cone) is imported as a conical transition to a point"));
                (PartKind::Transition { length, fore_diameter: 2.0 * rf, aft_diameter: 0.0 }, [None, None])
            }
            "nosecone" => {
                let shape = self.nose_shape(&label, text(n, "shape").unwrap_or("ogive"), num(n, "shapeparameter").unwrap_or(1.0), length, 2.0 * ra);
                (PartKind::NoseCone { shape, length, diameter: 2.0 * ra }, [None, shoulder(n, "aft")])
            }
            "bodytube" => (PartKind::BodyTube { length, diameter: 2.0 * rf }, [None, None]),
            _ => {
                if text(n, "shape").is_some_and(|s| s != "conical") {
                    self.warn(format!("{label}: only conical transitions are modelled ({} imported as conical)", text(n, "shape").unwrap_or("")));
                }
                (PartKind::Transition { length, fore_diameter: 2.0 * rf, aft_diameter: 2.0 * ra }, [shoulder(n, "fore"), shoulder(n, "aft")])
            }
        };
        let at = self.r.parts.len();
        let id = match self.r.insert(at, kind.clone()) {
            Ok(id) => id,
            Err(e) => {
                self.warn(format!("{label}: cannot be placed in the stack ({e}); skipped with its contents"));
                return;
            }
        };
        self.r.set_kind(id, kind.clone());
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, kind.default_material());
        let thickness = match text(n, "thickness") {
            Some("filled") => f64::INFINITY,
            _ => num(n, "thickness").unwrap_or(0.002),
        };
        if let Some(p) = self.r.parts.iter_mut().find(|p| p.id == id) {
            p.name = name;
            p.material = material;
            p.finish = finish_of(n);
            if let Some(s) = shoulders[0] {
                p.shoulder_fore = s;
            }
            if let Some(s) = shoulders[1] {
                p.shoulder_aft = s;
            }
        }
        self.r.set_thickness(id, thickness);
        let station = self.x;
        self.x += length;
        // Children are placed relative to this part.
        let bore = self.r.parts.iter().find(|p| p.id == id).map_or(0.0, |p| p.inner_diameter_at(length / 2.0));
        let parent = Parent { axial: id, axial_station: station, top: station, len: length, bore, internal: false };
        let first = self.r.next_id;
        self.children(n, parent);
        self.zero_children(n, first);
        if tag == "bodytube" {
            if let Some(m) = child(n, "motormount") {
                self.body_tube_mount(&label, &parent, m);
            }
        }
        // A capped shoulder has a disc closing its far end (the model's shoulder is an open tube): its mass and position are
        // added to the part's by overriding both.
        let density = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(0.0);
        let mut caps: Vec<(f64, f64)> = Vec::new(); // (mass, position behind the part's fore face)
        for which in ["fore", "aft"] {
            if text(n, &format!("{which}shouldercapped")) != Some("true") {
                continue;
            }
            let r = num(n, &format!("{which}shoulderradius")).unwrap_or(0.0);
            let (t, l) = (num(n, &format!("{which}shoulderthickness")).unwrap_or(0.0), num(n, &format!("{which}shoulderlength")).unwrap_or(0.0));
            let at = if which == "aft" { length + l - t / 2.0 } else { -l + t / 2.0 };
            let inner = (r - t).max(0.0);
            caps.push((std::f64::consts::PI * inner * inner * t.min(r) * density, at));
        }
        let cap: f64 = caps.iter().map(|c| c.0).sum();
        if cap > 0.0 && num(n, "overridemass").is_none() && num(n, "overridecg").is_none() {
            if let Some(m) = self.r.component_mass(id) {
                let (mass, cg) = (m.mass * scale, m.cg - station);
                let total = mass + cap;
                self.r.set_mass_override(id, Some(total));
                self.r.set_cg_override(id, Some((mass * cg + caps.iter().map(|c| c.0 * c.1).sum::<f64>()) / total));
            }
        } else {
            self.settle_mass(id, n, scale, None);
        }
    }

    /// With "override subcomponents mass" the part's override already includes everything inside it: the parts created since
    /// `first` weigh nothing of their own.
    fn zero_children(&mut self, n: Node, first: PartId) {
        if text(n, "overridesubcomponentsmass") == Some("true") {
            for id in first..self.r.next_id {
                self.r.set_mass_override(id, Some(0.0));
            }
        }
    }

    /// A mass override on a whole `<stage>` that includes its subcomponents sets the stage's total: every part made since
    /// `first` is scaled by one factor so they add up to it (their CGs, and so the stage's, are kept).
    fn stage_override(&mut self, stage: Node, first: PartId) {
        let Some(total) = num(stage, "overridemass") else {
            return;
        };
        let name = name_of(stage).unwrap_or_else(|| "stage".to_string());
        if text(stage, "overridesubcomponentsmass") != Some("true") {
            self.warn(format!("{name}: a mass override of the stage itself (without its subcomponents) is not modelled"));
            return;
        }
        let ids: Vec<PartId> = (first..self.r.next_id).filter(|&id| !self.r.is_pod(id)).collect();
        let copies = |r: &Rocket, id: PartId| r.pod_of(id).map_or(1.0, |p| f64::from(p.count.max(1)));
        let sum: f64 = ids.iter().filter_map(|&id| self.r.component_mass(id)).map(|m| m.mass).sum();
        if sum <= 0.0 {
            return;
        }
        let k = total / sum;
        for id in ids {
            if let Some(m) = self.r.component_mass(id) {
                let one = m.mass / copies(&self.r, id);
                self.r.set_mass_override(id, Some(one * k));
            }
        }
        if num(stage, "overridecg").is_some() {
            self.warn(format!("{name}: a CG override of the whole stage is not modelled"));
        }
    }

    fn nose_shape(&mut self, label: &str, shape: &str, param: f64, length: f64, diameter: f64) -> NoseShape {
        match shape {
            "conical" => NoseShape::Conical,
            "ogive" => {
                if (param - 1.0).abs() > 1e-6 {
                    self.warn(format!("{label}: secant ogive (shape parameter {param}) imported as a tangent ogive"));
                }
                NoseShape::TangentOgive
            }
            "ellipsoid" => NoseShape::Elliptical,
            "power" => NoseShape::PowerSeries(param.clamp(0.01, 1.0)),
            "haack" => {
                if param.abs() < 1e-9 {
                    NoseShape::VonKarman
                } else {
                    NoseShape::Haack(param)
                }
            }
            "parabolic" => {
                if (param - 1.0).abs() < 1e-6 {
                    NoseShape::Parabolic
                } else {
                    // r/R = (2t - K t^2)/(2 - K): the closest power series r/R = t^n.
                    let k = param.clamp(0.0, 1.0);
                    let err =
                        |n: f64| -> f64 { (1..=32).map(|i| f64::from(i) / 32.0).map(|t: f64| ((2.0 * t - k * t * t) / (2.0 - k) - t.powf(n)).powi(2)).sum() };
                    let n = (30..=100).map(|i| f64::from(i) / 100.0).min_by(|a, b| err(*a).total_cmp(&err(*b))).unwrap_or(0.5);
                    self.warn(format!("{label}: parabolic nose with K = {param} imported as the power series n = {n:.2}"));
                    let _ = (length, diameter);
                    NoseShape::PowerSeries(n)
                }
            }
            other => {
                self.warn(format!("{label}: unknown nose shape \"{other}\" imported as a tangent ogive"));
                NoseShape::TangentOgive
            }
        }
    }

    // ---- children ----

    /// Adds the components inside `n`'s `<subcomponents>` placed against `p`.
    fn children(&mut self, n: Node, p: Parent) {
        let Some(subs) = child(n, "subcomponents") else {
            return;
        };
        let mut after = p.top;
        for c in subs.children().filter(Node::is_element) {
            let end = match c.tag_name().name() {
                "innertube" => self.inner_tube(c, &p, after),
                "tubecoupler" => self.coupler(c, &p, after),
                "bulkhead" => self.disc(c, &p, after, false),
                "centeringring" => self.disc(c, &p, after, true),
                "engineblock" => self.engine_block(c, &p, after),
                "parachute" => self.parachute(c, &p, after),
                "streamer" => self.streamer(c, &p, after),
                "shockcord" => self.shock_cord(c, &p, after),
                "masscomponent" => self.mass_component(c, &p, after),
                "trapezoidfinset" | "ellipticalfinset" | "freeformfinset" => self.fin_set(c, &p, after),
                "launchlug" | "railbutton" => self.lug_or_button(c, &p, after),
                "tubefinset" => self.tube_fins(c, &p, after),
                "podset" | "parallelstage" => self.pod(c, &p, after),
                "nosecone" | "bodytube" | "transition" => {
                    let name = name_of(c).unwrap_or_else(|| c.tag_name().name().to_string());
                    self.warn(format!("{name}: a body component inside another component is not supported"));
                    after
                }
                other => {
                    let name = name_of(c).unwrap_or_else(|| other.to_string());
                    self.warn(format!("{name}: {other} is not supported and was left out"));
                    after
                }
            };
            after = end;
        }
    }

    fn push_internal(&mut self, n: Node, p: &Parent, top: f64, kind: InternalKind, material: MaterialId) -> PartId {
        let id = self.fresh();
        let mut part = InternalPart::new(id, kind, p.axial, top - p.axial_station);
        part.material = material;
        part.name = name_of(n);
        self.r.internals.push(part);
        id
    }

    fn inner_tube(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "length").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let outer = dim(n, "outerradius").and_then(Dim::value).unwrap_or(0.02);
        let thickness = num(n, "thickness").unwrap_or(0.002).min(outer);
        let mount = child(n, "motormount");
        let overhang = mount.and_then(|m| num(m, "overhang")).unwrap_or(0.0);
        let label = name_of(n).unwrap_or_else(|| "Inner tube".to_string());
        let cluster = text(n, "clusterconfiguration").unwrap_or("single");
        if cluster != "single" {
            self.warn(format!("{label}: cluster \"{cluster}\" imported as a single tube (only one motor flies)"));
        }
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::CARDBOARD);
        let kind = InternalKind::InnerTube { outer_diameter: 2.0 * outer, thickness, length: len, motor_mount: mount.is_some(), overhang };
        let id = self.push_internal(n, p, top, kind, material);
        if let Some(m) = mount {
            self.record_mount(id, m);
        }
        let inner = Parent { top, len, bore: 2.0 * (outer - thickness), internal: true, ..*p };
        let first = self.r.next_id;
        self.children(n, inner);
        self.zero_children(n, first);
        self.settle_mass(id, n, scale, None);
        top + len
    }

    /// A body tube that is its own motor mount. The model keeps motors in inner tubes, so the mount becomes an inner tube of
    /// the body tube's bore and length, wall thickness 0 and no mass (the body tube already carries the wall), with the
    /// mount's overhang and motors.
    fn body_tube_mount(&mut self, label: &str, p: &Parent, m: Node) {
        let overhang = num(m, "overhang").unwrap_or(0.0);
        let kind = InternalKind::InnerTube { outer_diameter: p.bore, thickness: 0.0, length: p.len, motor_mount: true, overhang };
        let id = self.fresh();
        let mut part = InternalPart::new(id, kind, p.axial, p.top - p.axial_station);
        part.name = Some(format!("{label} motor mount"));
        self.r.internals.push(part);
        self.r.set_mass_override(id, Some(0.0));
        self.record_mount(id, m);
        self.warn(format!("{label}: the body tube is its own motor mount; imported as a massless inner tube of its bore ({:.1} mm)", p.bore * 1e3));
    }

    fn record_mount(&mut self, part: PartId, m: Node) {
        let mut rec = MountRec { part, stage: self.stage, motors: Vec::new(), ignition: Vec::new() };
        if let Some(event) = text(m, "ignitionevent") {
            rec.ignition.push(IgnitionSpec { config: String::new(), event: event.to_string(), delay: num(m, "ignitiondelay").unwrap_or(0.0) });
        }
        for c in m.children().filter(Node::is_element) {
            let config = c.attribute("configid").unwrap_or("").to_string();
            match c.tag_name().name() {
                "motor" => {
                    let designation = text(c, "designation").unwrap_or("").to_string();
                    if designation.is_empty() {
                        continue;
                    }
                    let delay = match text(c, "delay") {
                        Some("none") | None => f64::INFINITY,
                        Some(d) => d.parse().unwrap_or(f64::INFINITY),
                    };
                    let manufacturer = text(c, "manufacturer").unwrap_or("").to_string();
                    rec.motors.push(MotorSpec { config, motor: MotorRef::new(designation, manufacturer), delay });
                }
                "ignitionconfiguration" => rec.ignition.push(IgnitionSpec {
                    config,
                    event: text(c, "ignitionevent").unwrap_or("automatic").to_string(),
                    delay: num(c, "ignitiondelay").unwrap_or(0.0),
                }),
                _ => {}
            }
        }
        self.mounts.push(rec);
    }

    fn coupler(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "length").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let outer = dim(n, "outerradius").and_then(Dim::explicit).or_else(|| dim(n, "outerradius").and_then(Dim::value));
        let outer_d = outer.map(|r| 2.0 * r);
        let thickness = num(n, "thickness").unwrap_or(0.002);
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::CARDBOARD);
        let kind = InternalKind::Coupler { outer_diameter: outer_d, thickness, length: len };
        let id = self.push_internal(n, p, top, kind, material);
        let inner = Parent { top, len, bore: outer_d.unwrap_or(p.bore) - 2.0 * thickness, internal: true, ..*p };
        let first = self.r.next_id;
        self.children(n, inner);
        self.zero_children(n, first);
        self.settle_mass(id, n, scale, None);
        top + len
    }

    fn disc(&mut self, n: Node, p: &Parent, after: f64, ring: bool) -> f64 {
        let len = num(n, "length").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        // `auto` follows the OpenRocket parent: an airframe tube's bore, or (inside an inner tube or coupler) that tube's bore.
        let outer = match dim(n, "outerradius") {
            Some(Dim::Auto(_)) | None if p.internal => Some(p.bore),
            d => d.and_then(Dim::value).map(|r| 2.0 * r),
        };
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::BIRCH_PLYWOOD);
        let kind = if ring {
            // An automatic inner radius is the outer radius of a sibling inner tube; inside an inner tube there is none, so the
            // ring is a solid disc.
            let inner = match dim(n, "innerradius") {
                Some(Dim::Auto(_)) | None if p.internal => Some(0.0),
                d => d.and_then(Dim::value).map(|r| 2.0 * r),
            };
            InternalKind::CenteringRing { outer_diameter: outer, inner_diameter: inner, thickness: len }
        } else {
            InternalKind::Bulkhead { diameter: outer, thickness: len }
        };
        let id = self.push_internal(n, p, top, kind, material);
        self.settle_mass(id, n, scale, None);
        top + len
    }

    fn engine_block(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "length").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let thickness = num(n, "thickness").unwrap_or(0.002);
        let outer = dim(n, "outerradius").and_then(Dim::value).map(|r| 2.0 * r);
        let inner = (outer.unwrap_or(p.bore) - 2.0 * thickness).max(0.0);
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::BIRCH_PLYWOOD);
        let kind = InternalKind::EngineBlock { outer, inner, thickness: len };
        let id = self.push_internal(n, p, top, kind, material);
        self.settle_mass(id, n, scale, None);
        top + len
    }

    fn deployment(&mut self, label: &str, n: Node) -> Deployment {
        let delay = num(n, "deploydelay").unwrap_or(0.0).max(0.0);
        let event = match text(n, "deployevent").unwrap_or("apogee") {
            "ejection" => DeployEvent::Ejection,
            "altitude" => DeployEvent::Altitude(num(n, "deployaltitude").unwrap_or(0.0)),
            "launch" => return Deployment { event: DeployEvent::TimeFromLaunch, delay },
            "apogee" => DeployEvent::Apogee,
            "never" => {
                self.warn(format!("{label}: deployment \"never\" is not modelled; it deploys 1e6 s after launch"));
                return Deployment { event: DeployEvent::TimeFromLaunch, delay: 1.0e6 };
            }
            other => {
                self.warn(format!("{label}: deployment event \"{other}\" is not modelled; deploying at apogee instead"));
                DeployEvent::Apogee
            }
        };
        Deployment { event, delay }
    }

    fn recovery_rec(&mut self, id: PartId, label: &str, n: Node) {
        let per_config =
            children(n, "deploymentconfiguration").map(|c| (c.attribute("configid").unwrap_or("").to_string(), self.deployment(label, c))).collect::<Vec<_>>();
        self.recovery.push(RecoveryRec { part: id, per_config });
    }

    fn parachute(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let label = name_of(n).unwrap_or_else(|| "Parachute".to_string());
        let len = num(n, "packedlength").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let diameter = num(n, "diameter").unwrap_or(0.3);
        let cd = dim(n, "cd").and_then(Dim::explicit).unwrap_or(0.8);
        let lines = num(n, "linecount").map_or(6, |v| v.max(0.0) as u32);
        let line_length = num(n, "linelength").unwrap_or(0.3);
        let (material, _) = self.material(n, "material", MaterialKind::Surface, MaterialId::RIPSTOP_NYLON);
        let (line_material, _) = self.material(n, "linematerial", MaterialKind::Line, MaterialId::BRAIDED_NYLON_2MM);
        let surface = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(material.density());
        let line_density = child(n, "linematerial").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(line_material.density());
        let deploy = self.deployment(&label, n);
        let kind = InternalKind::Parachute {
            diameter,
            cd,
            line_count: lines,
            line_length,
            line_material,
            packed_length: len,
            packed_diameter: num(n, "packedradius").map(|r| 2.0 * r),
            deploy,
        };
        let id = self.push_internal(n, p, top, kind, material);
        let expected = std::f64::consts::FRAC_PI_4 * diameter * diameter * surface + f64::from(lines) * line_length * line_density;
        self.settle_mass(id, n, 1.0, Some(expected));
        self.recovery_rec(id, &label, n);
        top + len
    }

    fn streamer(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let label = name_of(n).unwrap_or_else(|| "Streamer".to_string());
        let len = num(n, "packedlength").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let (length, width) = (num(n, "striplength").unwrap_or(0.5), num(n, "stripwidth").unwrap_or(0.05));
        let cd = dim(n, "cd").and_then(Dim::explicit).unwrap_or(0.8);
        let (material, _) = self.material(n, "material", MaterialKind::Surface, MaterialId::MYLAR);
        let surface = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(material.density());
        let deploy = self.deployment(&label, n);
        let id = self.push_internal(n, p, top, InternalKind::Streamer { length, width, cd, deploy }, material);
        self.settle_mass(id, n, 1.0, Some(length * width * surface));
        self.recovery_rec(id, &label, n);
        top + len
    }

    fn shock_cord(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "packedlength").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let length = num(n, "cordlength").unwrap_or(0.0);
        let (material, _) = self.material(n, "material", MaterialKind::Line, MaterialId::BRAIDED_NYLON_3MM);
        let density = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(material.density());
        let id = self.push_internal(n, p, top, InternalKind::ShockCord { length, packed_length: len }, material);
        self.settle_mass(id, n, 1.0, Some(length * density));
        top + len
    }

    fn mass_component(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let len = num(n, "packedlength").unwrap_or(0.0);
        let top = self.place(n, p, len, after);
        let role = match text(n, "masscomponenttype") {
            Some("altimeter") | Some("flightcomputer") => MassRole::Altimeter,
            Some("battery") => MassRole::Battery,
            Some("payload") => MassRole::Payload,
            Some("deploymentcharge") | Some("recoveryhardware") | Some("masscomponent") | None => MassRole::Generic,
            Some(_) => MassRole::Generic,
        };
        let kind =
            InternalKind::MassComponent { mass: num(n, "mass").unwrap_or(0.0).max(0.0), length: len, diameter: num(n, "packedradius").map(|r| 2.0 * r), role };
        let id = self.push_internal(n, p, top, kind.clone(), kind.default_material());
        // A mass component's own `mass` is its weight; an override on it replaces that.
        self.settle_mass(id, n, 1.0, None);
        top + len
    }

    // ---- radial parts ----

    fn push_radial(&mut self, n: Node, p: &Parent, top: f64, angle: f64, count: u8, kind: RadialKind, material: MaterialId) -> PartId {
        let id = self.fresh();
        let mut part = RadialPart::new(id, kind, p.axial, top - p.axial_station, angle.rem_euclid(std::f64::consts::TAU), count);
        part.material = material;
        part.name = name_of(n);
        part.finish = finish_of(n);
        self.r.radials.push(part);
        id
    }

    fn angle_of(n: Node) -> f64 {
        deg(num(n, "rotation").or_else(|| child(n, "angleoffset").and_then(|c| c.text()).and_then(|t| t.trim().parse().ok())).unwrap_or(0.0))
    }

    fn fin_set(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let label = name_of(n).unwrap_or_else(|| "Fin set".to_string());
        let thickness = num(n, "thickness").unwrap_or(0.003);
        let (kind, root) = match n.tag_name().name() {
            "trapezoidfinset" => {
                let root = num(n, "rootchord").unwrap_or(0.1);
                (
                    RadialKind::FinSet {
                        root_chord: root,
                        tip_chord: num(n, "tipchord").unwrap_or(0.05),
                        span: num(n, "height").unwrap_or(0.05),
                        sweep: num(n, "sweeplength").unwrap_or(0.0),
                        thickness,
                    },
                    root,
                )
            }
            "ellipticalfinset" => {
                let root = num(n, "rootchord").unwrap_or(0.1);
                (RadialKind::EllipticalFin { root_chord: root, span: num(n, "height").unwrap_or(0.05), thickness }, root)
            }
            _ => {
                let mut points: Vec<(f64, f64)> = child(n, "finpoints")
                    .map(|f| children(f, "point").filter_map(|c| Some((c.attribute("x")?.parse().ok()?, c.attribute("y")?.parse().ok()?))).collect())
                    .unwrap_or_default();
                let x0 = points.first().map_or(0.0, |q| q.0);
                for q in &mut points {
                    q.0 -= x0;
                }
                let root = points.last().map_or(0.1, |q| q.0.abs());
                (RadialKind::FreeformFin { points, thickness }, root)
            }
        };
        let top = self.place(n, p, root, after);
        let count = num(n, "fincount").map_or(3.0, |v| v.max(1.0));
        if count > 16.0 {
            self.warn(format!("{label}: {count} fins exceed the model's 16; imported with 16"));
        }
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::BIRCH_PLYWOOD);
        let id = self.push_radial(n, p, top, Self::angle_of(n), count.min(16.0) as u8, kind, material);
        let section = match text(n, "crosssection") {
            Some("rounded") => FinSection::Rounded,
            Some("airfoil") => FinSection::Airfoil,
            _ => FinSection::Square,
        };
        let fillet = num(n, "filletradius").unwrap_or(0.0);
        let (fillet_material, fillet_scale) = self.material(n, "filletmaterial", MaterialKind::Bulk, MaterialId::EPOXY);
        if let Some(r) = self.r.radials.iter_mut().find(|r| r.id == id) {
            r.cross_section = section;
            r.cant = deg(num(n, "cant").unwrap_or(0.0));
            r.fillet_radius = fillet.max(0.0);
            r.fillet_material = fillet_material;
        }
        let _ = fillet_scale;
        // Fin tabs (the part of the root that goes through the airframe wall) add mass only: a plate of the fin's thickness.
        let tab = num(n, "tabheight").unwrap_or(0.0) * num(n, "tablength").unwrap_or(0.0);
        let expected = (tab > 0.0 && num(n, "overridemass").is_none()).then(|| {
            let thickness = num(n, "thickness").unwrap_or(0.0);
            let density = child(n, "material").and_then(|m| m.attribute("density")).and_then(|d| d.parse::<f64>().ok()).unwrap_or(0.0);
            let model = self.r.component_mass(id).map_or(0.0, |m| m.mass) * scale;
            model + tab * thickness * density * count.min(16.0)
        });
        self.settle_mass(id, n, scale, expected);
        top + root
    }

    fn lug_or_button(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let button = n.tag_name().name() == "railbutton";
        let label = name_of(n).unwrap_or_else(|| if button { "Rail button" } else { "Launch lug" }.to_string());
        let (kind, len) = if button {
            let d = num(n, "outerdiameter").unwrap_or(0.01);
            (RadialKind::RailButton { height: num(n, "height").unwrap_or(0.01), diameter: d }, 0.0)
        } else {
            let len = num(n, "length").unwrap_or(0.03);
            (RadialKind::LaunchLug { length: len, diameter: 2.0 * num(n, "radius").unwrap_or(0.004) }, len)
        };
        let top = self.place(n, p, len, after);
        let instances = num(n, "instancecount").map_or(1, |v| v.max(1.0) as usize);
        let separation = num(n, "instanceseparation").unwrap_or(0.0);
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, if button { MaterialId::ALUMINUM } else { MaterialId::CARDBOARD });
        let ids: Vec<PartId> = (0..instances)
            .map(|k| {
                // A rail button sits at its centre: the model's offset is its fore edge.
                let fore = top + separation * k as f64 - if button { num(n, "outerdiameter").unwrap_or(0.01) / 2.0 } else { 0.0 };
                self.push_radial(n, p, fore, Self::angle_of(n), 1, kind.clone(), material)
            })
            .collect();
        let per = num(n, "overridemass").map(|m| m / instances as f64);
        for &id in &ids {
            match per {
                Some(m) => {
                    self.r.set_mass_override(id, Some(m));
                }
                None => self.settle_mass(id, n, scale, None),
            }
        }
        if let Some(c) = num(n, "overridecg") {
            for &id in &ids {
                self.r.set_cg_override(id, Some(c));
            }
        }
        let _ = label;
        top + len
    }

    /// A tube fin set is a pod of one open tube (the sim flies an open pod as tube fins).
    fn tube_fins(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let label = name_of(n).unwrap_or_else(|| "Tube fin set".to_string());
        let len = num(n, "length").unwrap_or(0.05);
        let top = self.place(n, p, len, after);
        // An `auto` tube radius is the outer radius of the tube it stands on (OpenRocket's TubeFinSet).
        let parent_radius = self.r.parts.iter().find(|x| x.id == p.axial).map_or(0.0125, |x| x.kind.aft_diameter() / 2.0);
        let radius = dim(n, "radius").and_then(Dim::explicit).unwrap_or(parent_radius);
        let _ = &label;
        let (material, scale) = self.material(n, "material", MaterialKind::Bulk, MaterialId::CARDBOARD);
        let kind = PartKind::BodyTube { length: len, diameter: 2.0 * radius };
        let mut part = Part::new(0, kind);
        part.material = material;
        part.finish = finish_of(n);
        part.thickness = num(n, "thickness").unwrap_or(0.001).min(radius);
        part.name = name_of(n);
        // Mass: OpenRocket's tube is an annulus of the stock's density.
        if num(n, "overridemass").is_none() && (scale - 1.0).abs() > 0.002 {
            let ro = radius;
            let ri = (radius - part.thickness).max(0.0);
            part.mass_override = Some(std::f64::consts::PI * (ro * ro - ri * ri) * len * material.density() * scale);
        } else {
            part.mass_override = num(n, "overridemass").map(|m| m / num(n, "fincount").unwrap_or(1.0).max(1.0));
        }
        let count = num(n, "fincount").map_or(3.0, |v| v.max(1.0)).min(16.0) as u8;
        let body = PodBody { parts: vec![part], radials: Vec::new(), internals: Vec::new() };
        self.attach_pod(n, p, top, Self::angle_of(n), count, 0.0, body, Vec::new(), Vec::new());
        top + len
    }

    // ---- pods and parallel stages ----

    fn pod(&mut self, n: Node, p: &Parent, after: f64) -> f64 {
        let parallel = n.tag_name().name() == "parallelstage";
        let label = name_of(n).unwrap_or_else(|| "Pod".to_string());
        if parallel {
            self.warn(format!("{label}: a parallel stage is imported as a pod that burns and flies with its stage (separation events are not modelled)"));
        }
        let mut inner = Builder::new(self.w);
        inner.stage = self.stage;
        if let Some(subs) = child(n, "subcomponents") {
            let axial: Vec<Node> =
                subs.children().filter(|c| c.is_element() && matches!(c.tag_name().name(), "nosecone" | "bodytube" | "transition")).collect();
            inner.plan_radii(&axial);
            inner.stack(subs);
        }
        let length = inner.x;
        let Builder { r, mounts, recovery, .. } = inner;
        let body = PodBody { parts: r.parts, radials: r.radials, internals: r.internals };
        let gap = match child(n, "radiusoffset").and_then(|c| c.attribute("method")) {
            Some("surface") | Some("relative") | None => num(n, "radiusoffset").unwrap_or(0.0).max(0.0),
            Some(other) => {
                self.warn(format!("{label}: radius offset method \"{other}\" is not modelled; the pod stands on the surface"));
                0.0
            }
        };
        let top = self.place(n, p, length, after);
        let count = num(n, "instancecount").map_or(1.0, |v| v.max(1.0)).min(16.0) as u8;
        self.attach_pod(n, p, top, Self::angle_of(n), count, gap, body, mounts, recovery);
        top + length
    }

    /// Attaches `body` as a pod set and carries the mounts and recovery devices found inside it over to the final part ids.
    fn attach_pod(
        &mut self,
        n: Node,
        p: &Parent,
        top: f64,
        angle: f64,
        count: u8,
        gap: f64,
        mut body: PodBody,
        mounts: Vec<MountRec>,
        recovery: Vec<RecoveryRec>,
    ) {
        let old: Vec<PartId> = body.ids().collect();
        let id = self.fresh();
        self.r.next_id = body.reid(self.r.next_id);
        let new: Vec<PartId> = body.ids().collect();
        let remap = |part: PartId| old.iter().position(|o| *o == part).and_then(|i| new.get(i).copied());
        for mut m in mounts {
            if let Some(part) = remap(m.part) {
                m.part = part;
                m.stage = self.stage;
                self.mounts.push(m);
            }
        }
        for mut rec in recovery {
            if let Some(part) = remap(rec.part) {
                rec.part = part;
                self.recovery.push(rec);
            }
        }
        let mut set = RadialPart::new(id, RadialKind::Pod { gap, body }, p.axial, top - p.axial_station, angle.rem_euclid(std::f64::consts::TAU), count);
        set.name = name_of(n);
        self.r.radials.push(set);
    }
}

fn shoulder(n: Node, which: &str) -> Option<Shoulder> {
    let length = num(n, &format!("{which}shoulderlength")).unwrap_or(0.0);
    if length <= 0.0 {
        return None;
    }
    Some(Shoulder {
        length,
        diameter: 2.0 * num(n, &format!("{which}shoulderradius")).unwrap_or(0.0),
        thickness: num(n, &format!("{which}shoulderthickness")).unwrap_or(0.002),
    })
}

// ---- document -----------------------------------------------------------------------------------------------------------

impl<'w> Import<'w> {
    fn new(w: &'w mut Vec<String>) -> Import<'w> {
        Import { b: Builder::new(w), config_ids: Vec::new(), config_map: HashMap::new(), simulations: Vec::new(), motors: Vec::new(), separations: Vec::new() }
    }

    fn rocket_node(&mut self, n: Node) {
        self.b.r.name = name_of(n).unwrap_or_default();
        self.b.r.designer = text(n, "designer").unwrap_or("").to_string();
        match text(n, "referencetype") {
            None | Some("maximum") => {}
            Some(other) => self.b.warn(format!("reference type \"{other}\" is not modelled; the largest body diameter is the reference")),
        }
        // Configurations, in file order; the default one is active.
        let mut default = None;
        for (i, c) in children(n, "motorconfiguration").enumerate() {
            let Some(id) = c.attribute("configid") else {
                continue;
            };
            let cid = if i == 0 { self.b.r.configs[0].id } else { self.b.r.add_configuration("") };
            self.b.r.rename_configuration(cid, text(c, "name").unwrap_or(""));
            self.config_ids.push(id.to_string());
            self.config_map.insert(id.to_string(), cid);
            if c.attribute("default") == Some("true") {
                default = Some(cid);
            }
        }
        if let Some(d) = default {
            self.b.r.select_configuration(d);
        }
        // The stack, top stage first.
        if let Some(subs) = child(n, "subcomponents") {
            let axial: Vec<Node> = children(subs, "stage")
                .flat_map(|s| child(s, "subcomponents").into_iter().flat_map(|c| c.children().filter(Node::is_element)))
                .filter(|c| matches!(c.tag_name().name(), "nosecone" | "bodytube" | "transition"))
                .collect();
            self.b.plan_radii(&axial);
            for (index, stage) in children(subs, "stage").enumerate() {
                self.b.stage = index;
                let mut rec = SeparationRec::default();
                if let Some(e) = text(stage, "separationevent") {
                    rec.default = Some((e.to_string(), num(stage, "separationdelay").unwrap_or(0.0)));
                }
                for c in children(stage, "separationconfiguration") {
                    rec.per_config.push((
                        c.attribute("configid").unwrap_or("").to_string(),
                        text(c, "separationevent").unwrap_or("burnout").to_string(),
                        num(c, "separationdelay").unwrap_or(0.0),
                    ));
                }
                self.separations.push(rec);
                let first = self.b.r.next_id;
                if let Some(s) = child(stage, "subcomponents") {
                    self.b.stack(s);
                }
                self.b.stage_override(stage, first);
            }
        }
        self.apply_configurations();
    }

    /// An upper stage's motor may stick out behind its airframe by a few millimetres (OpenRocket's default overhang is 3 mm, and
    /// the motor simply reaches into the next stage). The model wants the stage joint at or aft of the motor's aft end, so such a
    /// motor is pulled in (the mount's overhang shortened) until the joint is at its aft end; the shift is reported.
    fn tuck_in_overhangs(&mut self) {
        const REACH: f64 = 0.02;
        for _ in 0..self.b.r.internals.len() {
            let Some(station) = self
                .b
                .r
                .stage_issues()
                .into_iter()
                .map(|(_, i)| match i {
                    crate::StageIssue::NoJoint { station, .. } => station,
                })
                .next()
            else {
                return;
            };
            let stations = self.b.r.stations();
            let joint = stations[1..stations.len().saturating_sub(1)]
                .iter()
                .copied()
                .filter(|j| *j < station && station - j <= REACH)
                .fold(f64::NEG_INFINITY, f64::max);
            let mount = self.b.r.motor_mounts().into_iter().filter(|m| m.pod.is_none() && (m.motor_aft_station - station).abs() < 1e-9).map(|m| m.id).next();
            let (true, Some(mount)) = (joint.is_finite(), mount) else {
                return;
            };
            if let Some(part) = self.b.r.internals.iter_mut().find(|p| p.id == mount) {
                if let InternalKind::InnerTube { overhang, .. } = &mut part.kind {
                    *overhang -= station - joint;
                }
            }
            let name = self.b.r.internal(mount).map(InternalPart::display_name).unwrap_or_default();
            self.b.warn(format!(
                "{name}: its motor reaches {:.1} mm past a stage joint; the motor was moved that far forward so the stage separates at the joint",
                (station - joint) * 1e3
            ));
        }
    }

    /// Motors, ignition, separation and recovery timing of every configuration.
    fn apply_configurations(&mut self) {
        self.tuck_in_overhangs();
        let mounts = self.b.mounts.clone();
        let recovery = self.b.recovery.clone();
        // OpenRocket stages with a motor mount of the core, top first, are the model's stages from the last (index 1 = aft-most).
        let core_mounts: Vec<&MountRec> = mounts.iter().filter(|m| self.b.r.internals.iter().any(|i| i.id == m.part)).collect();
        let mut stages: Vec<usize> = core_mounts.iter().map(|m| m.stage).collect();
        stages.sort_unstable();
        stages.dedup();
        let model_stage = |or_stage: usize| stages.iter().position(|s| *s == or_stage).map(|i| stages.len() - i);

        for (or_id, &cid) in self.config_ids.clone().iter().zip(self.config_ids.clone().iter().map(|k| self.config_map[k]).collect::<Vec<_>>().iter()) {
            for m in &mounts {
                let Some(spec) = m.motors.iter().find(|s| &s.config == or_id) else {
                    continue;
                };
                let ignition = m.ignition.iter().find(|i| &i.config == or_id).or_else(|| m.ignition.iter().find(|i| i.config.is_empty()));
                let in_core = self.b.r.internals.iter().any(|i| i.id == m.part);
                let label = spec.motor.designation.clone();
                if matches!(ignition, Some(i) if i.event == "never") {
                    self.b.warn(format!("motor {label}: ignition \"never\" is not modelled; the mount is left empty in that configuration"));
                    continue;
                }
                if self.b.r.set_motor(cid, m.part, Some(spec.motor.clone())).is_err() {
                    self.b.warn(format!("motor {label}: its mount is not a motor mount of the model; left out"));
                    continue;
                }
                let _ = self.b.r.set_motor_delay(cid, m.part, spec.delay);
                if !self.motors.contains(&spec.motor) {
                    self.motors.push(spec.motor.clone());
                }
                if in_core {
                    if let (Some(stage), Some(ign)) = (model_stage(m.stage), ignition) {
                        self.ignition_event(cid, stage, ign, &label);
                    }
                }
            }
            for (index, rec) in self.separations.clone().iter().enumerate() {
                let Some(stage) = model_stage(index) else {
                    continue;
                };
                if stage == stages.len() {
                    continue; // the top stage has nothing above it to separate from
                }
                let event = rec.per_config.iter().find(|c| &c.0 == or_id).map(|c| (c.1.clone(), c.2)).or_else(|| rec.default.clone());
                if let Some((event, delay)) = event {
                    self.separation_event(cid, stage, &event, delay, &mounts, or_id, index);
                }
            }
            for rec in &recovery {
                if let Some((_, d)) = rec.per_config.iter().find(|c| &c.0 == or_id) {
                    let _ = self.b.r.set_deployment(cid, rec.part, *d);
                }
            }
        }
    }

    fn ignition_event(&mut self, cid: ConfigId, stage: usize, ign: &IgnitionSpec, label: &str) {
        let (trigger, delay) = match ign.event.as_str() {
            "launch" => (StageTrigger::TimeFromLaunch, ign.delay),
            "burnout" if stage > 1 => (StageTrigger::Burnout, ign.delay),
            "burnout" => (StageTrigger::TimeFromLaunch, ign.delay),
            "ejection" if stage > 1 => {
                self.b.warn(format!("motor {label}: ignition at the ejection charge of the stage below is approximated by that stage's burnout"));
                (StageTrigger::Burnout, ign.delay)
            }
            // `automatic`: launch for the first stage, the separation of the stage below for the others (the model's defaults).
            _ if stage > 1 => (StageTrigger::Separation, ign.delay),
            _ => (StageTrigger::TimeFromLaunch, ign.delay),
        };
        let _ = self.b.r.set_stage_event(cid, StageEvent { stage, kind: EventKind::Ignition, trigger, delay });
    }

    fn separation_event(&mut self, cid: ConfigId, stage: usize, event: &str, delay: f64, mounts: &[MountRec], or_id: &str, or_stage: usize) {
        let ejection_delay = mounts
            .iter()
            .filter(|m| m.stage == or_stage)
            .find_map(|m| m.motors.iter().find(|s| s.config == or_id).map(|s| s.delay))
            .filter(|d| d.is_finite())
            .unwrap_or(0.0);
        let (trigger, delay) = match event {
            "launch" => (StageTrigger::TimeFromLaunch, delay),
            "burnout" => (StageTrigger::Burnout, delay),
            "ejection" => (StageTrigger::Burnout, delay + ejection_delay),
            "upperignition" => {
                self.b.warn(format!("stage {stage} separation at the ignition of the upper stage is approximated by burnout"));
                (StageTrigger::Burnout, delay)
            }
            "never" => {
                self.b.warn(format!("stage {stage} separation \"never\" is not modelled; it separates at burnout"));
                (StageTrigger::Burnout, 0.0)
            }
            other => {
                self.b.warn(format!("stage {stage} separation event \"{other}\" is not modelled; it separates at burnout"));
                (StageTrigger::Burnout, 0.0)
            }
        };
        let _ = self.b.r.set_stage_event(cid, StageEvent { stage, kind: EventKind::Separation, trigger, delay });
    }

    fn simulations(&mut self, sims: Option<Node>) {
        let Some(sims) = sims else {
            return;
        };
        let mut flights = Vec::new();
        for (i, s) in children(sims, "simulation").enumerate() {
            let Some(c) = child(s, "conditions") else {
                continue;
            };
            let config = c.children().find(|x| x.tag_name().name() == "configid").and_then(|x| x.text()).and_then(|t| self.config_map.get(t.trim()).copied());
            let altitude = num(c, "launchaltitude").unwrap_or(0.0);
            let atmosphere = child(c, "atmosphere");
            let isa = atmosphere.and_then(|a| a.attribute("model")).is_none_or(|m| m == "isa");
            let temperature = match (isa, atmosphere.and_then(|a| a.attribute("temperature")).and_then(|t| t.parse::<f64>().ok())) {
                (false, Some(t)) => t,
                _ => 288.15 - 0.0065 * altitude,
            };
            let name = name_of(s).unwrap_or_else(|| format!("Simulation {}", i + 1));
            let sim = OrkSimulation {
                name: name.clone(),
                config,
                rod_length: num(c, "launchrodlength").unwrap_or(1.0),
                rod_angle: deg(num(c, "launchrodangle").unwrap_or(0.0)),
                rod_direction: deg(num(c, "launchroddirection").unwrap_or(90.0)),
                wind_average: num(c, "windaverage").unwrap_or(2.0),
                wind_turbulence: num(c, "windturbulence").unwrap_or(0.1),
                altitude,
                latitude: num(c, "launchlatitude").unwrap_or(45.0),
                isa,
                temperature,
                pressure: atmosphere.and_then(|a| a.attribute("pressure")).and_then(|t| t.parse().ok()),
                time_step: num(c, "timestep").unwrap_or(0.05),
                stored: child(s, "flightdata").map(stored),
            };
            let turbulence = match sim.wind_turbulence {
                t if t < 0.025 => Turbulence::Off,
                t if t < 0.075 => Turbulence::Low,
                t if t < 0.15 => Turbulence::Medium,
                _ => Turbulence::High,
            };
            let conditions = Conditions {
                rail_length: sim.rod_length,
                launch_angle: sim.rod_angle,
                wind_speed: sim.wind_average,
                // OpenRocket's default direction (and the app's launch) is into a wind from 90 degrees.
                wind_from: deg(num(c, "winddirection").unwrap_or(90.0)),
                turbulence,
                site_altitude: altitude,
                temperature,
                field_radius: 300.0,
            };
            flights.push(FlightDef {
                id: flights.len() as u64 + 1,
                name,
                motor: config.map_or(FlightMotor::Active, FlightMotor::Config),
                delay: None,
                conditions,
            });
            self.simulations.push(sim);
        }
        if !flights.is_empty() {
            self.b.r.set_flights(flights);
        }
    }
}

fn stored(f: Node) -> OrkStored {
    let at = |name: &str| f.attribute(name).and_then(|v| v.parse::<f64>().ok());
    OrkStored {
        max_altitude: at("maxaltitude"),
        max_velocity: at("maxvelocity"),
        max_acceleration: at("maxacceleration"),
        max_mach: at("maxmach"),
        time_to_apogee: at("timetoapogee"),
        flight_time: at("flighttime"),
        ground_hit_velocity: at("groundhitvelocity"),
        launch_rod_velocity: at("launchrodvelocity"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{EventKind, StageTrigger};
    use crate::internal::{DeployEvent, InternalKind};

    const MINI: &[u8] = include_bytes!("../tests/data/mini_two_stage.ork");

    fn mini() -> OrkImport {
        import(MINI).expect("the fixture imports")
    }

    fn part_id(r: &Rocket, name: &str) -> PartId {
        r.ids()
            .into_iter()
            .find(|&id| match r.kind_of(id) {
                Some(crate::PartRef::Axial(p)) => p.display_name() == name,
                Some(crate::PartRef::Radial(p)) => p.display_name() == name,
                Some(crate::PartRef::Internal(p)) => p.display_name() == name,
                None => false,
            })
            .unwrap_or_else(|| panic!("no part named {name}"))
    }

    fn close(a: f64, b: f64, rel: f64) {
        assert!((a - b).abs() <= rel * b.abs().max(1e-12), "{a} vs {b}");
    }

    /// A one-stage document around `body`, for tests of single features.
    fn doc(body: &str) -> OrkImport {
        let xml = format!(
            "<openrocket version=\"1.8\"><rocket><name>t</name><subcomponents><stage><name>s</name><subcomponents>{body}</subcomponents></stage></subcomponents></rocket></openrocket>"
        );
        import(xml.as_bytes()).expect("a small document imports")
    }

    const NOSE: &str = "<nosecone><name>n</name><material type=\"bulk\" density=\"1050\">p</material><length>0.2</length><thickness>0.002</thickness><shape>ogive</shape><shapeparameter>1.0</shapeparameter><aftradius>0.025</aftradius></nosecone>";

    #[test]
    fn auto_radii_follow_the_neighbours_and_ignore_the_stale_number() {
        let imp = mini();
        let r = &imp.rocket;
        let d: Vec<(f64, f64)> = r.parts().iter().map(|p| (p.kind.fore_diameter(), p.kind.aft_diameter())).collect();
        // Nose: `auto` follows the tube behind it (0.027 m radius). Boattail: fore follows the tube ahead, aft is given.
        // The sustainer tube says `auto 0.1` but follows the boattail's aft radius; the booster tube follows it too.
        assert_eq!(d, vec![(0.0, 0.054), (0.054, 0.054), (0.054, 0.045), (0.045, 0.045), (0.045, 0.045)]);
        let stations = r.stations();
        close(stations[4], 0.2 + 0.4 + 0.05 + 0.25, 1e-12);
        close(r.length(), 1.4, 1e-12);
        assert_eq!(r.name, "Mini two-stage");
        assert_eq!(r.parts()[0].finish, crate::Finish::SmoothPaint);
        assert_eq!(r.parts()[4].finish, crate::Finish::Rough);
        assert!(r.parts()[0].shoulder_aft.is_some() && r.parts()[0].shoulder_aft.diameter == 0.049);
    }

    #[test]
    fn masses_follow_openrocket_density_override_and_formulas() {
        let imp = mini();
        let r = &imp.rocket;
        // A tube of 1000 kg/m^3 stock (the table has no such entry): mass = pi (ro^2 - ri^2) L rho, within the override scaling.
        let tube = r.component_mass(part_id(r, "Upper tube")).unwrap().mass;
        close(tube, std::f64::consts::PI * (0.027f64.powi(2) - 0.025f64.powi(2)) * 0.4 * 1000.0, 5e-3);
        // A mass override replaces the computed mass of a whole fin group.
        close(r.component_mass(part_id(r, "Sustainer fins")).unwrap().mass, 0.2, 1e-12);
        // Parachute: canopy area x surface density + lines x line density (OpenRocket's formula), whatever the table says.
        close(r.component_mass(part_id(r, "Main")).unwrap().mass, std::f64::consts::FRAC_PI_4 * 0.36 * 0.067 + 6.0 * 0.5 * 0.001, 5e-3);
        close(r.component_mass(part_id(r, "Nose weight")).unwrap().mass, 0.1, 1e-12);
        close(r.component_mass(part_id(r, "Cord")).unwrap().mass, 1.0 * 0.0035, 5e-3);
        assert!(imp.warnings.iter().any(|w| w.contains("Odd cardboard")), "{:?}", imp.warnings);
    }

    #[test]
    fn rail_button_instances_are_separate_groups_at_their_spacing() {
        let imp = mini();
        let r = &imp.rocket;
        let buttons: Vec<&crate::RadialPart> = r.radials().iter().filter(|p| p.name.as_deref() == Some("Buttons")).collect();
        assert_eq!(buttons.len(), 2);
        let (a, b) = (r.radial_station(buttons[0]).unwrap(), r.radial_station(buttons[1]).unwrap());
        close(b - a, 0.3, 1e-9);
        // The model's offset is the button's fore edge: its centre sits where OpenRocket puts the component.
        let top = r.stations()[4];
        close(a + 0.0097 / 2.0, top + 0.1, 1e-9);
        assert!((buttons[0].angle - std::f64::consts::PI).abs() < 1e-9);
        assert!(matches!(buttons[0].kind, RadialKind::RailButton { height, diameter } if height == 0.0097 && diameter == 0.0097));
    }

    #[test]
    fn fins_carry_count_cant_cross_section_and_position() {
        let imp = mini();
        let r = &imp.rocket;
        let fins = r.radial(part_id(r, "Sustainer fins")).unwrap();
        assert_eq!(fins.count, 3);
        assert_eq!(fins.cross_section, crate::FinSection::Airfoil);
        close(fins.cant, 1.5f64.to_radians(), 1e-12);
        close(fins.angle, 10f64.to_radians(), 1e-12);
        assert_eq!(fins.kind, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.08, sweep: 0.07, thickness: 0.003 });
        // `bottom` 0: the root's trailing edge is at the aft end of the tube (stations 0.65 to 0.9).
        close(r.radial_station(fins).unwrap(), 0.9 - 0.12, 1e-9);
        let booster = r.radial(part_id(r, "Booster fins")).unwrap();
        assert_eq!(booster.count, 4);
        close(r.radial_station(booster).unwrap(), 1.4 - 0.15, 1e-9);
    }

    #[test]
    fn internal_parts_sit_where_openrocket_puts_them() {
        let imp = mini();
        let r = &imp.rocket;
        let st = |name: &str| r.part_station(part_id(r, name)).unwrap();
        close(st("Main"), 0.2 + 0.02, 1e-12);
        // `bottom` 0.05: the coupler's aft end is 0.05 above the aft end of its tube (stations 0.2 to 0.6).
        close(st("Coupler"), 0.6 + 0.05 - 0.12, 1e-12);
        // The sustainer mount: bottom of the tube (0.65 to 0.9), 0.15 long.
        close(st("Sustainer mount"), 0.9 - 0.15, 1e-12);
        // A ring inside an inner tube has no sibling tube to follow: a solid disc, flush with the tube's bore.
        let ring = r.internal(part_id(r, "Retainer")).unwrap();
        assert!(matches!(ring.kind, InternalKind::CenteringRing { inner_diameter: Some(d), .. } if d == 0.0));
        // Auto outer diameters of a bulkhead in a coupler follow the coupler's bore.
        let bulkhead = r.internal(part_id(r, "Coupler bulkhead")).unwrap();
        assert!(matches!(bulkhead.kind, InternalKind::Bulkhead { diameter: Some(d), .. } if (d - (0.054 - 0.004 - 0.004)).abs() < 1e-9));
    }

    #[test]
    fn configurations_carry_motors_delays_and_staging() {
        let imp = mini();
        let r = &imp.rocket;
        assert_eq!(r.configurations().len(), 2);
        assert_eq!(imp.config_ids.len(), 2);
        assert_eq!(r.active_config_id(), r.configurations()[0].id, "the default configuration is active");
        let mounts = r.motor_mounts();
        assert_eq!(mounts.len(), 2);
        let (sustainer, booster) = (mounts[0].id, mounts[1].id);
        let (c0, c1) = (&r.configurations()[0], &r.configurations()[1]);
        let slot = |c: &crate::Configuration, m: PartId| c.slot(m).cloned().unwrap();
        assert_eq!(slot(c0, sustainer).motor, Some(MotorRef::new("G64W", "AeroTech")));
        assert!(slot(c0, sustainer).is_plugged(), "delay `none` is a plugged motor");
        assert_eq!(slot(c1, sustainer).motor, Some(MotorRef::new("G80T", "AeroTech")));
        assert_eq!(slot(c1, sustainer).delay, 7.0);
        assert_eq!(slot(c0, booster).motor, Some(MotorRef::new("H128W", "AeroTech")));
        assert_eq!(imp.motors.len(), 3);
        assert_eq!((mounts[0].overhang, mounts[1].overhang), (0.0, 0.01));

        // Stage 2 (sustainer) lights 2 s after the booster burns out in configuration 0, at burnout in configuration 1.
        let ignite0 = c0.event(2, EventKind::Ignition);
        assert_eq!((ignite0.trigger, ignite0.delay), (StageTrigger::Burnout, 2.0));
        assert_eq!(c1.event(2, EventKind::Ignition).delay, 0.0);
        // Stage 1 (booster) lights at launch. Its separation: the stage's `ejection` is burnout plus the motor's delay (a plugged
        // motor adds none); configuration 1 says burnout + 1.5 s.
        assert_eq!(c0.event(1, EventKind::Ignition).trigger, StageTrigger::TimeFromLaunch);
        let sep0 = c0.event(1, EventKind::Separation);
        assert_eq!((sep0.trigger, sep0.delay), (StageTrigger::Burnout, 0.5));
        let sep1 = c1.event(1, EventKind::Separation);
        assert_eq!((sep1.trigger, sep1.delay), (StageTrigger::Burnout, 1.5));
        assert_eq!(r.stages().len(), 2);
        assert!(r.stage_issues().is_empty());
    }

    #[test]
    fn recovery_deployment_is_per_configuration() {
        let imp = mini();
        let r = &imp.rocket;
        let chute = part_id(r, "Main");
        let (c0, c1) = (r.configurations()[0].id, r.configurations()[1].id);
        assert_eq!(r.deployment(c0, chute).map(|d| d.event), Some(DeployEvent::Ejection));
        let d = r.deployment(c1, chute).unwrap();
        assert_eq!((d.event, d.delay), (DeployEvent::Altitude(150.0), 1.0));
    }

    #[test]
    fn simulations_become_flights() {
        let imp = mini();
        let flight = &imp.rocket.flights()[0];
        assert_eq!(flight.name, "Windy pad");
        assert_eq!(flight.motor, crate::FlightMotor::Config(imp.rocket.configurations()[1].id));
        let c = &flight.conditions;
        close(c.rail_length, 2.4384, 1e-12);
        close(c.launch_angle, 5f64.to_radians(), 1e-12);
        assert_eq!((c.wind_speed, c.turbulence, c.site_altitude), (4.0, crate::Turbulence::High, 1400.0));
        close(c.temperature, 288.15 - 0.0065 * 1400.0, 1e-9);
        let sim = &imp.simulations[0];
        assert_eq!(sim.config, Some(imp.rocket.configurations()[1].id));
        assert_eq!(sim.stored.as_ref().and_then(|s| s.max_altitude), Some(1234.5));
        close(sim.latitude, 32.9, 1e-12);
    }

    #[test]
    fn the_imported_document_saves_and_loads() {
        let imp = mini();
        let back = Rocket::from_json(&imp.rocket.to_json()).expect("a saved import loads");
        assert_eq!(back, imp.rocket);
    }

    #[test]
    fn unsupported_things_are_reported_not_dropped() {
        let body = "<nosecone><name>secant</name><length>0.2</length><shape>ogive</shape><shapeparameter>0.6</shapeparameter><aftradius>0.025</aftradius></nosecone>\
             <bodytube><name>t</name><length>0.5</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>\
               <innertube><name>cluster</name><length>0.1</length><outerradius>0.01</outerradius><thickness>0.001</thickness><clusterconfiguration>3-ring</clusterconfiguration></innertube>\
               <parachute><name>chute</name><packedlength>0.05</packedlength><diameter>0.3</diameter><deployevent>never</deployevent></parachute>\
               <thingamajig><name>gizmo</name></thingamajig>\
             </subcomponents></bodytube>\
             <transition><name>flare</name><length>0.05</length><shape>ogive</shape><foreradius>auto</foreradius><aftradius>0.03</aftradius></transition>".to_string();
        let imp = doc(&body);
        let has = |needle: &str| imp.warnings.iter().any(|w| w.contains(needle));
        assert!(has("secant ogive"), "{:?}", imp.warnings);
        assert!(has("cluster \"3-ring\""), "{:?}", imp.warnings);
        assert!(has("deployment \"never\""), "{:?}", imp.warnings);
        assert!(has("gizmo") && has("thingamajig"), "{:?}", imp.warnings);
        assert!(has("only conical transitions"), "{:?}", imp.warnings);
        // What could be imported still was.
        assert_eq!(imp.rocket.parts().len(), 3);
    }

    #[test]
    fn a_stage_override_with_subcomponents_sets_the_stage_total() {
        let xml = format!(
            "<openrocket><rocket><name>t</name><subcomponents><stage><name>s</name><overridemass>1.5</overridemass><overridesubcomponentsmass>true</overridesubcomponentsmass><subcomponents>{NOSE}\
             <bodytube><name>t</name><material type=\"bulk\" density=\"1850\">g</material><length>0.5</length><radius>auto</radius><thickness>0.002</thickness>\
             <subcomponents><masscomponent><name>m</name><packedlength>0.05</packedlength><mass>0.3</mass></masscomponent></subcomponents></bodytube></subcomponents></stage></subcomponents></rocket></openrocket>"
        );
        let imp = import(xml.as_bytes()).unwrap();
        close(imp.rocket.dry_mass().mass, 1.5, 1e-9);
    }

    #[test]
    fn a_capped_shoulder_adds_its_disc() {
        let capped = |c: &str| {
            let nose = format!(
                "<nosecone><name>n</name><material type=\"bulk\" density=\"1050\">p</material><length>0.1</length><thickness>0.002</thickness><shape>ogive</shape><aftradius>0.0125</aftradius>\
                 <aftshoulderradius>0.0115</aftshoulderradius><aftshoulderlength>0.02</aftshoulderlength><aftshoulderthickness>0.002</aftshoulderthickness><aftshouldercapped>{c}</aftshouldercapped></nosecone>"
            );
            let imp = doc(&nose);
            imp.rocket.component_mass(imp.rocket.parts()[0].id).unwrap()
        };
        let (open, closed) = (capped("false"), capped("true"));
        close(closed.mass - open.mass, std::f64::consts::PI * 0.0095f64.powi(2) * 0.002 * 1050.0, 1e-6);
        assert!(closed.cg > open.cg, "the disc is at the far end of the shoulder");
    }

    #[test]
    fn nose_shapes_map_to_the_models_families() {
        let shape = |s: &str, p: f64| {
            let nose = format!(
                "<nosecone><name>n</name><length>0.2</length><shape>{s}</shape><shapeparameter>{p}</shapeparameter><aftradius>0.025</aftradius></nosecone>"
            );
            let imp = doc(&nose);
            match imp.rocket.parts()[0].kind {
                PartKind::NoseCone { shape, .. } => shape,
                _ => unreachable!(),
            }
        };
        assert_eq!(shape("conical", 0.0), NoseShape::Conical);
        assert_eq!(shape("ellipsoid", 0.0), NoseShape::Elliptical);
        assert_eq!(shape("power", 0.5), NoseShape::PowerSeries(0.5));
        assert_eq!(shape("haack", 0.0), NoseShape::VonKarman);
        assert_eq!(shape("haack", 1.0 / 3.0), NoseShape::Haack(1.0 / 3.0));
        assert_eq!(shape("parabolic", 1.0), NoseShape::Parabolic);
        assert!(matches!(shape("parabolic", 0.5), NoseShape::PowerSeries(n) if (0.3..1.0).contains(&n)));
    }

    #[test]
    fn freeform_and_elliptical_fins() {
        let body = format!(
            "{NOSE}<bodytube><name>t</name><length>0.5</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>\
             <freeformfinset><name>ff</name><fincount>3</fincount><axialoffset method=\"bottom\">0.0</axialoffset><thickness>0.002</thickness>\
               <finpoints><point x=\"0.0\" y=\"0.0\"/><point x=\"0.025\" y=\"0.03\"/><point x=\"0.105\" y=\"0.04\"/><point x=\"0.08\" y=\"0.0\"/></finpoints></freeformfinset>\
             <ellipticalfinset><name>ef</name><fincount>4</fincount><axialoffset method=\"bottom\">0.0</axialoffset><thickness>0.002</thickness><rootchord>0.1</rootchord><height>0.05</height></ellipticalfinset>\
             </subcomponents></bodytube>"
        );
        let imp = doc(&body);
        let r = &imp.rocket;
        let ff = r.radial(part_id(r, "ff")).unwrap();
        assert!(matches!(&ff.kind, RadialKind::FreeformFin { points, .. } if points.len() == 4 && points[3] == (0.08, 0.0)));
        let ef = r.radial(part_id(r, "ef")).unwrap();
        assert_eq!((ef.count, &ef.kind), (4, &RadialKind::EllipticalFin { root_chord: 0.1, span: 0.05, thickness: 0.002 }));
    }

    #[test]
    fn a_pod_set_becomes_a_pod_with_its_parts() {
        let body = format!(
            "{NOSE}<bodytube><name>t</name><length>0.5</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>\
             <podset><name>pods</name><instancecount>2</instancecount><radiusoffset method=\"relative\">0.003</radiusoffset><angleoffset method=\"relative\">90.0</angleoffset><axialoffset method=\"top\">0.1</axialoffset>\
               <subcomponents><nosecone><name>pn</name><length>0.05</length><shape>conical</shape><aftradius>0.01</aftradius></nosecone>\
               <bodytube><name>pt</name><length>0.2</length><radius>auto</radius><thickness>0.001</thickness></bodytube></subcomponents></podset>\
             </subcomponents></bodytube>"
        );
        let imp = doc(&body);
        let r = &imp.rocket;
        let pod = r.pods().next().expect("a pod set");
        assert_eq!(pod.count, 2);
        assert_eq!(pod.kind.pod_gap(), Some(0.003));
        assert_eq!(pod.kind.pod_body().map(|b| b.parts().len()), Some(2));
        close(pod.angle, std::f64::consts::FRAC_PI_2, 1e-12);
        close(pod.offset, 0.1, 1e-12);
    }

    #[test]
    fn containers_zip_gzip_and_plain_xml_all_import() {
        use flate2::write::{DeflateEncoder, GzEncoder};
        use flate2::Compression;
        use std::io::Write;

        let plain = import(MINI).unwrap();
        let mut gz = GzEncoder::new(Vec::new(), Compression::default());
        gz.write_all(MINI).unwrap();
        assert_eq!(import(&gz.finish().unwrap()).unwrap().rocket, plain.rocket);

        for deflate in [true, false] {
            let payload = if deflate {
                let mut e = DeflateEncoder::new(Vec::new(), Compression::default());
                e.write_all(MINI).unwrap();
                e.finish().unwrap()
            } else {
                MINI.to_vec()
            };
            let name = b"rocket.ork";
            let method: u16 = if deflate { 8 } else { 0 };
            let mut zip = Vec::new();
            // A decoy entry first: the importer must pick rocket.ork.
            let mut entries: Vec<(&[u8], u16, Vec<u8>, usize)> = vec![(b"decals/readme.txt".as_slice(), 0, b"hi".to_vec(), 2)];
            entries.push((name.as_slice(), method, payload.clone(), MINI.len()));
            let mut central = Vec::new();
            for (n, m, data, size) in &entries {
                let offset = zip.len() as u32;
                for (bytes, header) in [(&mut zip, true), (&mut central, false)] {
                    bytes.extend_from_slice(if header { &[0x50, 0x4b, 3, 4] } else { &[0x50, 0x4b, 1, 2] });
                    if !header {
                        bytes.extend_from_slice(&[20, 0]);
                    }
                    bytes.extend_from_slice(&[20, 0, 0, 0]);
                    bytes.extend_from_slice(&m.to_le_bytes());
                    bytes.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0]); // time, date, crc (unchecked)
                    bytes.extend_from_slice(&(data.len() as u32).to_le_bytes());
                    bytes.extend_from_slice(&(*size as u32).to_le_bytes());
                    bytes.extend_from_slice(&(n.len() as u16).to_le_bytes());
                    bytes.extend_from_slice(&[0, 0]);
                    if !header {
                        bytes.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]); // comment, disk, attributes
                        bytes.extend_from_slice(&offset.to_le_bytes());
                    }
                    bytes.extend_from_slice(n);
                    if header {
                        bytes.extend_from_slice(data);
                    }
                }
            }
            let start = zip.len() as u32;
            zip.extend_from_slice(&central);
            zip.extend_from_slice(&[0x50, 0x4b, 5, 6, 0, 0, 0, 0]);
            zip.extend_from_slice(&(entries.len() as u16).to_le_bytes());
            zip.extend_from_slice(&(entries.len() as u16).to_le_bytes());
            zip.extend_from_slice(&(central.len() as u32).to_le_bytes());
            zip.extend_from_slice(&start.to_le_bytes());
            zip.extend_from_slice(&[0, 0]);
            assert_eq!(import(&zip).unwrap().rocket, plain.rocket, "deflate = {deflate}");
        }
    }

    #[test]
    fn a_motor_reaching_past_a_stage_joint_is_tucked_in() {
        // An upper-stage motor that sticks 3 mm out of its airframe into the next stage (OpenRocket's default overhang).
        let mount = |overhang: f64, config: &str, designation: &str| {
            format!(
                "<innertube><name>mount</name><axialoffset method=\"bottom\">0.0</axialoffset><length>0.1</length><outerradius>0.015</outerradius><thickness>0.001</thickness>\
                 <motormount><ignitionevent>automatic</ignitionevent><overhang>{overhang}</overhang><motor configid=\"{config}\"><designation>{designation}</designation><manufacturer>X</manufacturer><delay>3</delay></motor></motormount></innertube>"
            )
        };
        let xml = format!(
            "<openrocket><rocket><name>t</name><motorconfiguration configid=\"c1\" default=\"true\"/><subcomponents>\
             <stage><name>upper</name><subcomponents>{NOSE}<bodytube><name>a</name><length>0.3</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>{}</subcomponents></bodytube></subcomponents></stage>\
             <stage><name>lower</name><subcomponents><bodytube><name>b</name><length>0.3</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>{}</subcomponents></bodytube></subcomponents></stage>\
             </subcomponents></rocket></openrocket>",
            mount(0.003, "c1", "U100"),
            mount(0.0, "c1", "L100"),
        );
        let imp = import(xml.as_bytes()).unwrap();
        assert!(imp.rocket.stage_issues().is_empty(), "{:?}", imp.rocket.stage_issues());
        assert_eq!(imp.rocket.stages().len(), 2);
        let upper = imp.rocket.motor_mounts()[0];
        close(upper.motor_aft_station, imp.rocket.stations()[2], 1e-9);
        assert!(imp.warnings.iter().any(|w| w.contains("3.0 mm past a stage joint")), "{:?}", imp.warnings);
    }

    #[test]
    fn bad_input_is_an_error() {
        assert!(matches!(import(b"<html/>"), Err(OrkError::NotOpenRocket)));
        assert!(matches!(import(b"<openrocket><oops></openrocket>"), Err(OrkError::Xml(_))));
        assert!(matches!(import(b"<openrocket/>"), Err(OrkError::NotOpenRocket)));
        assert!(matches!(import(b"PK\x03\x04 truncated"), Err(OrkError::Archive(_))));
        assert!(import_file("/no/such/file.ork").is_err());
    }

    #[test]
    fn a_body_tube_can_be_its_own_motor_mount() {
        // A 24.1 mm bore body tube holding a D12 directly, 6.35 mm overhang, as in the *Tube fin rocket* example.
        let tube = |mount: &str| {
            format!(
                "<openrocket><rocket><name>t</name><motorconfiguration configid=\"c1\" default=\"true\"/><subcomponents><stage><name>s</name><subcomponents>{NOSE}\
                 <bodytube><name>body</name><material type=\"bulk\" density=\"894.4\">paper</material><length>0.4572</length><radius>0.0124</radius><thickness>0.00033</thickness>{mount}</bodytube>\
                 </subcomponents></stage></subcomponents></rocket></openrocket>"
            )
        };
        let mount = "<motormount><ignitionevent>automatic</ignitionevent><ignitiondelay>0.0</ignitiondelay><overhang>0.00635</overhang>\
                     <motor configid=\"c1\"><type>single</type><manufacturer>Estes</manufacturer><designation>D12</designation><delay>7.0</delay></motor></motormount>";
        let plain = import(tube("").as_bytes()).unwrap();
        let imp = import(tube(mount).as_bytes()).unwrap();
        let r = &imp.rocket;
        let mounts = r.motor_mounts();
        assert_eq!(mounts.len(), 1);
        let m = mounts[0];
        close(m.inner_diameter, 2.0 * (0.0124 - 0.00033), 1e-9);
        close(m.length, 0.4572, 1e-12);
        close(m.fore_station, 0.2, 1e-12);
        close(m.motor_aft_station, 0.2 + 0.4572 + 0.00635, 1e-12);
        let slot = r.active_configuration().slot(m.id).cloned().unwrap();
        assert_eq!(slot.motor, Some(MotorRef::new("D12", "Estes")));
        assert_eq!(slot.delay, 7.0);
        assert_eq!(imp.motors, vec![MotorRef::new("D12", "Estes")]);
        // The stand-in tube adds no mass: the body tube already carries the wall.
        close(r.dry_mass().mass, plain.rocket.dry_mass().mass, 1e-12);
        close(r.dry_mass().cg, plain.rocket.dry_mass().cg, 1e-12);
        assert!(imp.warnings.iter().any(|w| w.contains("body: the body tube is its own motor mount")), "{:?}", imp.warnings);
    }
}
