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
//! * **Radial parts.** Trapezoidal, elliptical and freeform fin sets (count, cant, cross-section, thickness, fillet, through-the-wall
//!   tab, position),
//!   launch lugs and rail buttons (linear `instancecount`/`instanceseparation` instances become one group each), pod sets and
//!   parallel stages (as pods), tube fin sets (as a pod of one open tube).
//! * **Internal parts.** Inner tubes (motor mounts with their overhang; a body tube that is its own motor mount gets a massless
//!   inner tube of its bore and length to hold the motor, with a warning), centering rings, bulkheads, tube couplers, engine
//!   blocks, parachutes, streamers, shock cord and mass components, placed exactly where OpenRocket puts them (the part's OpenRocket
//!   parent keeps it, even when it overhangs a joint).
//! * **Materials.** The model's material table is fixed. A part takes the table entry of the same *family* by the words in its name
//!   (`Aircraft plywood (LOC)` is plywood, so it keeps a plywood's shear modulus for fin flutter), else the entry of the same kind
//!   nearest in density; when the densities differ the part's mass is *overridden* with OpenRocket's mass (`density × the model's own
//!   volume`), so mass and CG match. Components with an OpenRocket mass override carry it (a group's override is for the whole group; with
//!   "override subcomponents" the children are zeroed). CG overrides carry over too.
//! * **Configurations.** Each OpenRocket flight configuration becomes a [`Configuration`](crate::Configuration) (motor
//!   designation and manufacturer, ejection delay, `none` is a plugged motor, ignition and separation events, per-configuration
//!   recovery deployment). Thrust curves are not in the design XML; the motors are listed in [`OrkImport::motors`] to be resolved
//!   against a motor database (e.g. RASP files exported by `tools/openrocket-harness`).
//! * **Simulations.** Each stored simulation becomes a [`FlightDef`] (rail, wind, site, turbulence) and an [`OrkSimulation`] with
//!   the values the model has no field for, plus OpenRocket's stored result summary.
//!
//! * **Thrust curves.** Older files carry none. OpenRocket 26.xx snapshots (file version 1.11) store user-defined curves inside the
//!   ZIP as `thrustcurves/<digest>.rse` (RockSim engine XML) beside `rocket.ork`, and each `<motor>` element names its curve by
//!   `<digest>`. [`import`] returns the entries raw in [`OrkImport::embedded_curves`] (this crate does not read motor files) and each
//!   motor reference's digest in [`OrkImport::motor_digests`]; one digest may carry several designations. A caller resolves
//!   them against the curves. Bare-XML and gzip files have none; an unreadable entry is a warning.
//!
//! Not modelled (reported as warnings): clustered motors beyond the first tube, secant ogives, non-conical transitions,
//! motor mounts in body tubes, drag overrides on internal components, custom reference lengths, "never" deployment/ignition.
//!
//! References: the OpenRocket file format (`net.sf.openrocket.file.openrocket`), Niskanen (2013) *OpenRocket technical
//! documentation* for the component definitions.

mod builder;
mod container;
mod import;
#[cfg(test)]
mod tests;
mod xml;

use std::collections::HashMap;
use std::fmt;
use std::path::Path;

use roxmltree::Document;

use crate::config::{ConfigId, MotorRef};
use crate::internal::Deployment;
use crate::rocket::Rocket;
use crate::PartId;

use container::unpack;
use xml::child;

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

/// A thrust curve stored inside the `.ork` ZIP (`thrustcurves/<digest>.rse`, newer OpenRocket versions): the user-defined motors
/// of the design, in RockSim engine format (parse it with `bowshock_motors::parse_rse`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EmbeddedCurve {
    /// The file name without its extension: the digest OpenRocket identifies the motor by (a motor's `<digest>` element).
    pub digest: String,
    /// The file's text.
    pub rse: String,
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
    /// The digest (`<digest>`) of every motor in [`motors`](Self::motors) that has one, once per distinct reference and digest.
    /// OpenRocket names a motor by its digest: one digest can carry several names (the same curve as `J33N` and `I32N`).
    pub motor_digests: Vec<(MotorRef, String)>,
    /// The thrust curves stored in the file, in archive order. Empty for bare-XML and gzip files.
    pub embedded_curves: Vec<EmbeddedCurve>,
}

/// Imports the `.ork` file at `path`.
pub fn import_file(path: impl AsRef<Path>) -> Result<OrkImport, OrkError> {
    let bytes = std::fs::read(path.as_ref()).map_err(|e| OrkError::Io(e.to_string()))?;
    import(&bytes)
}

/// Imports an `.ork` held in memory: a ZIP archive with a `rocket.ork` entry, a gzip stream or bare XML.
pub fn import(bytes: &[u8]) -> Result<OrkImport, OrkError> {
    let mut warnings = Vec::new();
    let (xml, embedded_curves) = unpack(bytes, &mut warnings)?;
    let doc = Document::parse(&xml).map_err(|e| OrkError::Xml(e.to_string()))?;
    let root = doc.root_element();
    if root.tag_name().name() != "openrocket" {
        return Err(OrkError::NotOpenRocket);
    }
    let rocket_node = child(root, "rocket").ok_or(OrkError::NotOpenRocket)?;
    let mut ctx = Import::new(&mut warnings);
    ctx.rocket_node(rocket_node);
    ctx.simulations(child(root, "simulations"));
    let Import { b, config_ids, simulations, motors, motor_digests, .. } = ctx;
    let mut rocket = b.r;
    rocket.repair_flights();
    Ok(OrkImport { rocket, warnings, motors, simulations, config_ids, motor_digests, embedded_curves })
}

// ---- containers ---------------------------------------------------------------------------------------------------------
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
    /// The motor's `<digest>` (empty when the file has none).
    digest: String,
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
    motor_digests: Vec<(MotorRef, String)>,
    /// `(stage separation default, per-config)` of every OpenRocket stage, by stage index.
    separations: Vec<SeparationRec>,
}

#[derive(Clone, Debug, Default)]
struct SeparationRec {
    default: Option<(String, f64)>,
    per_config: Vec<(String, String, f64)>,
}
