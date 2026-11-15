//! The lanes' data: the rocket turned into bars in stations (metres from the tip), sub-rows, stage marks and the mass profile.
//! Pure: no widgets and no pixels (those are `geometry` and `canvas`), so the packing and the numbers are unit-tested.

use bowshock_model::{ConfigId, InternalKind, PartClass, PartId, PartKind, PartRef, RadialKind, RadialPart, Rocket, Stage, System};
use bowshock_motors::{Motor, MotorDb};
use bowshock_sim::from_model::Phase;

use crate::app::ui;
use crate::units::{Quantity, UnitSystem};

/// Width of one bin of the mass shading, metres.
pub const MASS_BIN: f64 = 0.05;
/// Parts at most this long (m) are "short": drawn as blocks over what they sit in, never opening a sub-row of their own.
pub const THIN: f64 = 0.02;
/// Two stations closer than this (m) are the same.
const EPS: f64 = 1e-6;
/// Spans overlapping by less than this (m) only touch.
const TOUCH: f64 = 5e-4;

/// What a bar stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BarKey {
    /// A part of the rocket, of any class.
    Part(PartId),
    /// The motor configuration `config` puts in motor mount `mount`.
    Motor {
        /// The motor mount (an inner tube).
        mount: PartId,
        /// The configuration the motor belongs to.
        config: ConfigId,
    },
}

impl BarKey {
    /// The part the bar selects: the part itself, or a motor's mount.
    pub fn part(self) -> PartId {
        match self {
            BarKey::Part(id) => id,
            BarKey::Motor { mount, .. } => mount,
        }
    }
}

/// What the user may do to a bar by dragging.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Edit {
    /// The bar moves along the length.
    pub movable: bool,
    /// The aft end can be dragged to change the length.
    pub resize_aft: bool,
    /// The fore end can be dragged (the part's offset changes with its length).
    pub resize_fore: bool,
}

/// The pod set a bar belongs to: every part standing inside a pod is drawn in its own system's lane, bracketed with the others
/// of the same pod (the link glyph and `Pod ×2` of a symmetry group).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PodMark {
    /// The pod set (what the bracket selects).
    pub id: PartId,
    /// The group label: `Pod ×2` (the pod's name and its count; just the name for a single pod).
    pub label: String,
    /// Copies of the pod.
    pub count: u8,
}

/// The group label of a pod set, as its bar and the brackets of its parts show it.
pub fn pod_label(pod: &RadialPart) -> String {
    let base = pod.name.clone().unwrap_or_else(|| "Pod".to_string());
    if pod.count > 1 {
        format!("{base} ×{}", pod.count)
    } else {
        base
    }
}

/// One bar: a part (or a motor) at its real station in its system's lane.
#[derive(Clone, Debug, PartialEq)]
pub struct Bar {
    /// What it is.
    pub key: BarKey,
    /// The lane.
    pub system: System,
    /// Station of the fore edge, metres from the tip.
    pub start: f64,
    /// Station of the aft edge.
    pub end: f64,
    /// Name shown in the bar (`Body tube`, `Fins ×3`, `H128W`).
    pub name: String,
    /// The key dimension, in the display unit (`871.4 · Ø76.2`); empty for none.
    pub dim: String,
    /// The per-copy overrides of a symmetry group, listed in mono on its bar (`B: cant`); empty for any other bar.
    pub overrides: String,
    /// Sub-row within the lane, from 0.
    pub sub: usize,
    /// Part of a stage that has dropped away in the phase on show.
    pub dropped: bool,
    /// Copies of a symmetry group (1 for a single part).
    pub count: u8,
    /// Which class of part (`None` for a motor).
    pub class: Option<PartClass>,
    /// The part it sits on or in.
    pub parent: Option<PartId>,
    /// The span of that parent, the container walls shown while dragging.
    pub container: Option<(f64, f64)>,
    /// What the container is called in the drag tag (`tube`, `nose`, `transition`).
    pub container_word: &'static str,
    /// Short (see [`THIN`]).
    pub thin: bool,
    /// A non-physical overlap: two discs at one station.
    pub warn: bool,
    /// Drawn as a dashed outline: a shock cord, or another configuration's motor.
    pub dashed: bool,
    /// The motor of a configuration other than the current one (comparison).
    pub alt: bool,
    /// A fin (fins read `from aft` in the drag tag).
    pub fin: bool,
    /// A part's shoulder inside the neighbouring tube: drawn as a short block, not something to snap to.
    pub shoulder: bool,
    /// The pod set the part stands in (`None` for a part of the core, and for the pod set itself).
    pub pod: Option<PodMark>,
    /// A pod set itself (its bar is in the Structure lane; its parts carry the [`PodMark`]).
    pub pod_set: bool,
    /// Drag abilities.
    pub edit: Edit,
}

impl Bar {
    /// The part the bar selects: the part itself, or a motor's mount.
    pub fn part(&self) -> PartId {
        match self.key {
            BarKey::Part(id) => id,
            BarKey::Motor { mount, .. } => mount,
        }
    }

    /// Length in metres.
    pub fn extent(&self) -> f64 {
        self.end - self.start
    }

    /// Packing order: the core (0), then pod sets (1), then the parts inside pods (2).
    fn tier(&self) -> u8 {
        match (self.pod_set, self.pod.is_some()) {
            (_, true) => 2,
            (true, _) => 1,
            _ => 0,
        }
    }
}

/// A lane that has parts, and how many sub-rows they need.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lane {
    /// The system.
    pub system: System,
    /// Rows, at least 1.
    pub subs: usize,
}

/// A separation between two stages.
#[derive(Clone, Debug, PartialEq)]
pub struct Sep {
    /// `Stage::index` of the stage that separates here (what `Selection::Separation` holds).
    pub stage: usize,
    /// Station of the joint.
    pub station: f64,
    /// Name of the stage ahead of the joint.
    pub upper: String,
    /// Name of the stage behind it.
    pub lower: String,
    /// The stage behind has dropped away in the phase on show.
    pub lower_dropped: bool,
}

/// A harness: the dotted line a shock cord draws to the recovery gear it is tied to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Harness {
    /// Fore end of the line.
    pub from: f64,
    /// Aft end (the cord).
    pub to: f64,
    /// Sub-row it is drawn in.
    pub sub: usize,
}

/// CG, CP and the margin of what is on show.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Marks {
    /// Centre of gravity, station.
    pub cg: f64,
    /// Centre of pressure, station.
    pub cp: f64,
    /// Static margin, calibers.
    pub margin: f64,
}

/// The CG of a configuration other than the current one.
#[derive(Clone, Debug, PartialEq)]
pub struct OtherCg {
    /// The configuration's name.
    pub name: String,
    /// Its CG, station.
    pub cg: f64,
}

/// What [`Scene::build`] reads.
pub struct Inputs<'a> {
    /// The document.
    pub rocket: &'a Rocket,
    /// Its stages (firing order).
    pub stages: &'a [Stage],
    /// What flies.
    pub phase: Phase,
    /// Motor database, for the motors' lengths.
    pub motors: &'a MotorDb,
    /// Display units for the labels.
    pub units: UnitSystem,
    /// Draw every other configuration's motor too.
    pub compare: bool,
}

/// Everything the lanes draw.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Scene {
    /// Length of the rocket, metres.
    pub length: f64,
    /// The lanes that have parts, in the fixed order.
    pub lanes: Vec<Lane>,
    /// Every bar.
    pub bars: Vec<Bar>,
    /// Harness lines.
    pub harness: Vec<Harness>,
    /// Separations (one per stage joint).
    pub seps: Vec<Sep>,
    /// Stations from which everything is dropped away, when the phase is an upper stage alone.
    pub dropped_from: Option<f64>,
    /// CG, CP and margin; set by the caller from the physics cache.
    pub marks: Option<Marks>,
    /// Mass per [`MASS_BIN`] from the tip, kg.
    pub mass: Vec<f32>,
    /// The other configurations' CGs.
    pub other_cgs: Vec<OtherCg>,
}

/// The motor `reference` names, as the bridge finds it.
fn find_motor<'a>(db: &'a MotorDb, designation: &str, manufacturer: &str) -> Option<&'a Motor> {
    db.motors
        .iter()
        .find(|m| m.designation.eq_ignore_ascii_case(designation.trim()) && m.manufacturer.eq_ignore_ascii_case(manufacturer.trim()))
        .or_else(|| db.find(designation))
}

/// How the container is called in the drag tag.
fn container_word(kind: &PartKind) -> &'static str {
    match kind {
        PartKind::NoseCone { .. } => "nose",
        PartKind::BodyTube { .. } => "tube",
        PartKind::Transition { .. } => "transition",
    }
}

/// Whether an internal kind is a flat disc (rings, bulkheads, blocks): two of them at one station are a mistake.
fn is_disc(kind: &InternalKind) -> bool {
    matches!(kind, InternalKind::CenteringRing { .. } | InternalKind::Bulkhead { .. } | InternalKind::EngineBlock { .. })
}

/// Whether an internal kind has a length to drag.
fn internal_resizable(kind: &InternalKind) -> bool {
    !is_disc(kind)
}

impl Scene {
    /// Turns the document into bars. Marks and the other configurations' CGs are left for the caller to fill.
    pub fn build(inputs: &Inputs) -> Scene {
        let Inputs { rocket, stages, phase, motors, units, compare } = *inputs;
        let len = |m: f64| ui::bare(m, Quantity::Length, units);
        let length = rocket.length();
        let first = phase.first_stage();
        let dropped_from = if stages.len() > 1 && first > 1 { stages.iter().find(|s| s.index == first).map(|s| s.aft_station) } else { None };
        let dropped = |start: f64| dropped_from.is_some_and(|from| start >= from - EPS);
        // A pod, and everything in it, goes with the stage of the core part the pod stands on (it may overhang that stage's joint).
        let dropped_in = |id: PartId, start: f64, in_pod: bool| {
            if in_pod && dropped_from.is_some() {
                rocket.stage_of(id).is_some_and(|s| s < first)
            } else {
                dropped(start)
            }
        };
        let stations = rocket.stations();
        let mut bars: Vec<Bar> = Vec::new();

        // Airframe.
        let pod_mark = |pod: &RadialPart| PodMark { id: pod.id, label: pod_label(pod), count: pod.count };
        let parts_in_pods = rocket.pods().flat_map(|pod| {
            let mark = pod_mark(pod);
            pod.kind.pod_body().into_iter().flat_map(|b| b.parts()).filter_map(move |p| {
                let (start, end) = rocket.part_span(p.id)?;
                Some((p, start, end, Some(mark.clone())))
            })
        });
        let core_parts = rocket.parts().iter().enumerate().map(|(i, p)| (p, stations[i], stations[i + 1], None));
        for (part, start, end, pod) in core_parts.chain(parts_in_pods) {
            let dim = match part.kind {
                PartKind::BodyTube { diameter, .. } => format!("{} · Ø{}", len(end - start), len(diameter)),
                _ => len(end - start),
            };
            bars.push(Bar {
                key: BarKey::Part(part.id),
                system: part.kind.system(),
                start,
                end,
                name: part.display_name(),
                dim,
                sub: 0,
                overrides: String::new(),
                dropped: dropped_in(part.id, start, pod.is_some()),
                count: 1,
                class: Some(PartClass::Axial),
                parent: None,
                container: None,
                container_word: "",
                thin: false,
                warn: false,
                dashed: false,
                alt: false,
                fin: false,
                shoulder: false,
                pod: pod.clone(),
                pod_set: false,
                edit: Edit { movable: true, resize_aft: true, resize_fore: false },
            });
            // A shoulder inside the neighbouring tube: a short dashed block of the part's own bar, in the lane's structure row.
            for aft in [false, true] {
                let Some(s) = rocket.shoulder(part.id, aft).filter(|s| s.is_some()) else { continue };
                let (from, to) = if aft { (end, end + s.length) } else { (start - s.length, start) };
                bars.push(Bar {
                    key: BarKey::Part(part.id),
                    system: part.kind.system(),
                    start: from,
                    end: to,
                    name: "Shoulder".to_string(),
                    dim: len(s.length),
                    sub: 0,
                    overrides: String::new(),
                    dropped: dropped_in(part.id, start, pod.is_some()),
                    count: 1,
                    class: Some(PartClass::Axial),
                    parent: None,
                    container: None,
                    container_word: "",
                    thin: true,
                    warn: false,
                    dashed: true,
                    alt: false,
                    fin: false,
                    shoulder: true,
                    pod: pod.clone(),
                    pod_set: false,
                    edit: Edit::default(),
                });
            }
        }

        // Radial groups.
        let radials_in_pods = rocket.pods().flat_map(|pod| {
            let mark = pod_mark(pod);
            pod.kind.pod_body().into_iter().flat_map(|b| b.radials()).map(move |r| (r, Some(mark.clone())))
        });
        for (radial, pod) in rocket.radials().iter().map(|r| (r, None)).chain(radials_in_pods) {
            let Some(start) = rocket.radial_station(radial) else { continue };
            let end = start + radial.kind.extent();
            let parent = rocket.part(radial.parent);
            let base = radial.name.clone().unwrap_or_else(|| match &radial.kind {
                k if k.is_fin() => "Fins".to_string(),
                RadialKind::LaunchLug { .. } => "Lug".to_string(),
                RadialKind::Pod { .. } => "Pod".to_string(),
                _ => "Rail".to_string(),
            });
            let name = if radial.count > 1 { format!("{base} ×{}{}", radial.count, remaining_copies(radial)) } else { base };
            bars.push(Bar {
                key: BarKey::Part(radial.id),
                system: radial.kind.system(),
                start,
                end,
                name,
                dim: String::new(),
                sub: 0,
                overrides: radial.override_summary(),
                dropped: dropped_in(radial.id, start, pod.is_some() || radial.kind.is_pod()),
                count: radial.count,
                class: Some(PartClass::Radial),
                parent: Some(radial.parent),
                container: rocket.part_span(radial.parent),
                container_word: parent.map_or("", |p| container_word(&p.kind)),
                thin: end - start <= THIN || matches!(radial.kind, RadialKind::RailButton { .. }),
                warn: false,
                dashed: false,
                alt: false,
                fin: radial.kind.is_fin(),
                shoulder: false,
                pod,
                pod_set: radial.kind.is_pod(),
                edit: Edit {
                    movable: true,
                    resize_aft: matches!(radial.kind, RadialKind::FinSet { .. } | RadialKind::EllipticalFin { .. } | RadialKind::LaunchLug { .. }),
                    resize_fore: false,
                },
            });
        }

        // Internal parts.
        let internals_in_pods = rocket.pods().flat_map(|pod| {
            let mark = pod_mark(pod);
            pod.kind.pod_body().into_iter().flat_map(|b| b.internals()).map(move |p| (p, Some(mark.clone())))
        });
        for (part, pod) in rocket.internals().iter().map(|p| (p, None)).chain(internals_in_pods) {
            let Some(start) = rocket.part_station(part.id) else { continue };
            let extent = part.kind.extent();
            let end = start + extent;
            let parent = rocket.part(part.parent);
            let thin = extent <= THIN || is_disc(&part.kind);
            let name = match (&part.name, &part.kind) {
                (Some(n), _) => n.clone(),
                (None, InternalKind::Parachute { .. }) => "Chute".to_string(),
                (None, InternalKind::ShockCord { .. }) => "Cord".to_string(),
                (None, k) => k.name().to_string(),
            };
            let dim = match part.kind {
                InternalKind::InnerTube { outer_diameter, .. } => format!("Ø{}", len(outer_diameter)),
                InternalKind::Coupler { length, .. } => len(length),
                InternalKind::Parachute { diameter, .. } => len(diameter),
                InternalKind::Streamer { length, .. } => len(length),
                _ => String::new(),
            };
            bars.push(Bar {
                key: BarKey::Part(part.id),
                system: part.kind.system(),
                start,
                end,
                name,
                dim,
                sub: 0,
                overrides: String::new(),
                dropped: dropped_in(part.id, start, pod.is_some()),
                count: 1,
                class: Some(PartClass::Internal),
                parent: Some(part.parent),
                container: rocket.part_span(part.parent),
                container_word: parent.map_or("", |p| container_word(&p.kind)),
                thin,
                warn: false,
                dashed: matches!(part.kind, InternalKind::ShockCord { .. }),
                alt: false,
                fin: false,
                shoulder: false,
                pod,
                pod_set: false,
                edit: Edit { movable: true, resize_aft: internal_resizable(&part.kind), resize_fore: internal_resizable(&part.kind) },
            });
        }

        // Motors: the current configuration's, and with `compare` the others'.
        let mounts = rocket.motor_mounts();
        let active = rocket.active_config_id();
        for config in rocket.configurations() {
            if config.id != active && !compare {
                continue;
            }
            for mount in &mounts {
                let Some(motor) = config.slot(mount.id).and_then(|s| s.motor.as_ref()).and_then(|m| find_motor(motors, &m.designation, &m.manufacturer)) else {
                    continue;
                };
                let (end, start) = (mount.motor_aft_station, mount.motor_aft_station - motor.length);
                // A pod's mount holds one motor per copy of the pod: one bar, `×N`, in the pod's bracket.
                let name = if mount.copies > 1 { format!("{} ×{}", motor.designation, mount.copies) } else { motor.designation.clone() };
                bars.push(Bar {
                    key: BarKey::Motor { mount: mount.id, config: config.id },
                    system: System::Propulsion,
                    start,
                    end,
                    name,
                    dim: len(motor.length),
                    sub: 0,
                    overrides: String::new(),
                    dropped: dropped_in(mount.id, start, mount.pod.is_some()),
                    count: mount.copies.max(1),
                    class: None,
                    parent: None,
                    container: Some((mount.fore_station, mount.aft_station)),
                    container_word: "mount",
                    thin: false,
                    warn: false,
                    dashed: config.id != active,
                    alt: config.id != active,
                    fin: false,
                    shoulder: false,
                    pod: mount.pod.and_then(|id| rocket.pods().find(|p| p.id == id)).map(pod_mark),
                    pod_set: false,
                    edit: Edit::default(),
                });
            }
        }

        mark_overlaps(rocket, &mut bars);
        let lanes = pack(&mut bars);
        let harness = harnesses(&bars);

        let seps = if stages.len() > 1 {
            stages
                .iter()
                .filter_map(|s| {
                    let sep = s.separation?;
                    let above = stages.iter().find(|u| u.index == s.index + 1)?;
                    Some(Sep { stage: s.index, station: sep.station, upper: above.name.clone(), lower: s.name.clone(), lower_dropped: s.index < first })
                })
                .collect()
        } else {
            Vec::new()
        };

        let mass = mass_profile(rocket, &bars, length);
        Scene { length, lanes, bars, harness, seps, dropped_from, marks: None, mass, other_cgs: Vec::new() }
    }

    /// The bar `key` stands for.
    pub fn bar(&self, key: BarKey) -> Option<&Bar> {
        self.bars.iter().find(|b| b.key == key)
    }

    /// The bar of part `id`.
    pub fn part_bar(&self, id: PartId) -> Option<&Bar> {
        self.bar(BarKey::Part(id))
    }

    /// Makes sure the lane of `system` is on show (a carried part's lane, even if it has no parts yet).
    pub fn ensure_lane(&mut self, system: System) {
        if self.lanes.iter().all(|l| l.system != system) {
            self.lanes.push(Lane { system, subs: 1 });
            self.lanes.sort_by_key(|l| l.system);
        }
    }

    /// Station of every part face: what Shift snaps to.
    pub fn faces(&self, except: Option<BarKey>) -> Vec<f64> {
        let mut faces: Vec<f64> = self.bars.iter().filter(|b| Some(b.key) != except && !b.alt && !b.shoulder).flat_map(|b| [b.start, b.end]).collect();
        faces.sort_by(f64::total_cmp);
        faces.dedup_by(|a, b| (*a - *b).abs() < EPS);
        faces
    }
}

/// Flags the discs that sit at one station inside one tube.
fn mark_overlaps(rocket: &Rocket, bars: &mut [Bar]) {
    let discs: Vec<usize> = bars
        .iter()
        .enumerate()
        .filter(|(_, b)| matches!(b.key, BarKey::Part(id) if rocket.internal(id).is_some_and(|p| is_disc(&p.kind))))
        .map(|(i, _)| i)
        .collect();
    for (n, &a) in discs.iter().enumerate() {
        for &b in &discs[n + 1..] {
            let (x, y) = (&bars[a], &bars[b]);
            let overlap = x.end.min(y.end) - x.start.max(y.start);
            if x.parent == y.parent && overlap > TOUCH.min(0.5 * x.extent().min(y.extent())) {
                bars[a].warn = true;
                bars[b].warn = true;
            }
        }
    }
}

/// Greedy interval packing per lane. Regular parts take the first sub-row they fit in; short parts sit over whatever
/// they overlap (the row of the first regular bar under them) or in the first row they fit. Returns the lanes with parts.
fn pack(bars: &mut [Bar]) -> Vec<Lane> {
    let mut lanes = Vec::new();
    for system in System::ALL {
        let mut idx: Vec<usize> = (0..bars.len()).filter(|&i| bars[i].system == system).collect();
        if idx.is_empty() {
            continue;
        }
        // Current parts before the other configurations' motors; the core before the pod sets, those before the parts inside
        // them (so a pod never pushes the core's own bars down a row); then fore to aft, longer first.
        idx.sort_by(|&a, &b| {
            let (x, y) = (&bars[a], &bars[b]);
            x.alt.cmp(&y.alt).then(x.tier().cmp(&y.tier())).then(x.start.total_cmp(&y.start)).then(y.end.total_cmp(&x.end))
        });
        let mut rows: Vec<f64> = Vec::new();
        let mut placed: Vec<(usize, usize)> = Vec::new(); // (bar, row) of the regular bars
        let (short, regular): (Vec<usize>, Vec<usize>) = idx.iter().copied().partition(|&i| bars[i].thin);
        for i in regular {
            let start = bars[i].start;
            // Another configuration's motor sits in the sub-row of the current one of its mount, so comparing never changes
            // the panel's height (design/components/StationLanes, *Configurations*).
            if let (true, BarKey::Motor { mount, .. }) = (bars[i].alt, bars[i].key) {
                let twin = placed.iter().find(|&&(j, _)| !bars[j].alt && matches!(bars[j].key, BarKey::Motor { mount: m, .. } if m == mount));
                if let Some(&(_, row)) = twin {
                    bars[i].sub = row;
                    placed.push((i, row));
                    continue;
                }
            }
            let row = rows.iter().position(|&end| end <= start + EPS).unwrap_or_else(|| {
                rows.push(f64::NEG_INFINITY);
                rows.len() - 1
            });
            rows[row] = bars[i].end;
            bars[i].sub = row;
            placed.push((i, row));
        }
        for i in short {
            let (s, e) = (bars[i].start, bars[i].end);
            let under = placed.iter().find(|&&(j, _)| bars[j].start < e - EPS && bars[j].end > s + EPS).map(|&(_, r)| r);
            let row = under.unwrap_or_else(|| {
                // Free of everything regular: the first row that has no regular bar over the block.
                (0..rows.len().max(1)).find(|&r| !placed.iter().any(|&(j, jr)| jr == r && bars[j].start < e && bars[j].end > s)).unwrap_or(0)
            });
            bars[i].sub = row;
            if rows.is_empty() {
                rows.push(e);
            }
        }
        lanes.push(Lane { system, subs: rows.len().max(1) });
    }
    lanes
}

/// A dotted harness line from each shock cord back to the recovery gear in its tube.
fn harnesses(bars: &[Bar]) -> Vec<Harness> {
    let mut lines = Vec::new();
    for cord in bars.iter().filter(|b| b.dashed && !b.alt && b.system == System::Recovery) {
        let tied: Vec<&Bar> = bars.iter().filter(|b| b.key != cord.key && b.system == System::Recovery && b.parent == cord.parent && !b.alt).collect();
        let Some(far) = tied.iter().map(|b| b.start).reduce(f64::min) else { continue };
        let fore = cord.container.map_or(far, |(f, _)| f);
        lines.push(Harness { from: (far - 0.10).max(fore).min(cord.start), to: cord.start, sub: cord.sub });
    }
    lines
}

/// Mass per [`MASS_BIN`]: each part's mass spread evenly over its span, and the current configuration's motors.
fn mass_profile(rocket: &Rocket, bars: &[Bar], length: f64) -> Vec<f32> {
    let n = (length / MASS_BIN).ceil().max(1.0) as usize;
    let mut bins = vec![0.0f64; n];
    let mut spread = |mass: f64, start: f64, end: f64| {
        let span = (end - start).max(0.0);
        if span < EPS {
            let i = ((start / MASS_BIN).floor().max(0.0) as usize).min(n - 1);
            bins[i] += mass;
            return;
        }
        let first = ((start / MASS_BIN).floor().max(0.0) as usize).min(n - 1);
        let last = ((end / MASS_BIN).ceil().max(1.0) as usize).min(n);
        for (i, bin) in bins.iter_mut().enumerate().take(last).skip(first) {
            let overlap = end.min((i + 1) as f64 * MASS_BIN) - start.max(i as f64 * MASS_BIN);
            if overlap > 0.0 {
                *bin += mass * overlap / span;
            }
        }
    };
    for (id, props) in rocket.component_masses() {
        let Some(bar) = bars.iter().find(|b| b.key == BarKey::Part(id)) else { continue };
        if !bar.dropped {
            spread(props.mass, bar.start, bar.end);
        }
    }
    bins.iter().map(|&m| m as f32).collect()
}

/// A short description of what `bar` is for the hover tag: its name and station.
pub fn describe(bar: &Bar, rocket: &Rocket, units: UnitSystem) -> String {
    let at = ui::bare(bar.start, Quantity::Length, units);
    let kind = match bar.key {
        BarKey::Part(id) => rocket.kind_of(id).map_or("", |p: PartRef<'_>| p.kind_name()),
        BarKey::Motor { .. } => "Motor",
    };
    let pod = bar.pod.as_ref().map_or(String::new(), |p| format!(" · in {}", p.label));
    if bar.name.eq_ignore_ascii_case(kind) || kind.is_empty() {
        format!("{} · {at}{pod}", bar.name)
    } else {
        format!("{} ({}) · {at}{pod}", bar.name, kind.to_lowercase())
    }
}

/// The letters of the copies left in a group that is not evenly spaced (one was detached), as ` (A, C)`: the smallest even
/// spacing of `2π/n` that all the angles fall on gives each copy's original index. The model renumbers the copies when one
/// leaves, so the first remaining copy is always `A`. Empty for an evenly spaced group.
fn remaining_copies(radial: &bowshock_model::RadialPart) -> String {
    use std::f64::consts::TAU;
    if radial.instance_angles.is_none() {
        return String::new();
    }
    let angles = radial.angles();
    let Some(&first) = angles.first() else { return String::new() };
    for n in (usize::from(radial.count)..=16).filter(|n| *n > 1) {
        let step = TAU / n as f64;
        let indices: Vec<usize> = angles.iter().map(|a| ((a - first).rem_euclid(TAU) / step).round() as usize % n).collect();
        let on_grid = angles
            .iter()
            .zip(&indices)
            .all(|(a, i)| ((a - first).rem_euclid(TAU) - *i as f64 * step).abs() < 1e-6 || ((a - first).rem_euclid(TAU) - *i as f64 * step - TAU).abs() < 1e-6);
        if on_grid {
            let letters: Vec<String> = indices.iter().map(|&i| bowshock_model::instance_letter(i)).collect();
            return format!(" ({})", letters.join(", "));
        }
    }
    String::new()
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind};

    fn example() -> Rocket {
        Rocket::from_json(include_str!("../../../assets/examples/hpr_l1_cert.bowshock")).expect("the example loads")
    }

    fn build(rocket: &Rocket, compare: bool, phase: Phase) -> Scene {
        let db = MotorDb::bundled();
        let stages = rocket.stages();
        Scene::build(&Inputs { rocket, stages: &stages, phase, motors: &db, units: UnitSystem::MmGS, compare })
    }

    #[test]
    fn every_part_gets_a_bar_in_its_system_lane() {
        let rocket = example();
        let scene = build(&rocket, false, Phase::Full);
        let parts = rocket.parts().len() + rocket.radials().len() + rocket.internals().len();
        assert!(scene.bars.iter().filter(|b| matches!(b.key, BarKey::Part(_))).count() == parts);
        for bar in &scene.bars {
            assert!(scene.lanes.iter().any(|l| l.system == bar.system && bar.sub < l.subs));
        }
        // The lanes come in the fixed order.
        let order: Vec<System> = scene.lanes.iter().map(|l| l.system).collect();
        let mut sorted = order.clone();
        sorted.sort();
        assert_eq!(order, sorted);
        assert!(scene.mass.iter().sum::<f32>() > 0.0);
    }

    #[test]
    fn a_detached_group_lists_the_letters_of_its_remaining_copies() {
        let mut rocket = example();
        let fins = rocket.radials().iter().find(|r| r.kind.is_fin()).map(|r| r.id).unwrap();
        assert_eq!(remaining_copies(rocket.radial(fins).unwrap()), "");
        rocket.detach_radial(fins, 1).unwrap();
        assert_eq!(remaining_copies(rocket.radial(fins).unwrap()), " (A, C)");
        let scene = build(&rocket, false, Phase::Full);
        assert!(scene.part_bar(fins).unwrap().name.ends_with("×2 (A, C)"));
    }

    #[test]
    fn the_motor_sits_under_its_mount_in_a_second_sub_row() {
        let rocket = example();
        let scene = build(&rocket, false, Phase::Full);
        let motor = scene.bars.iter().find(|b| matches!(b.key, BarKey::Motor { .. })).expect("the example has a motor");
        let mount = scene.part_bar(motor.part()).unwrap();
        assert_eq!(mount.sub, 0);
        assert_eq!(motor.sub, 1);
        assert!(scene.lanes.iter().any(|l| l.system == System::Propulsion && l.subs >= 2));
        // The motor ends at the mount's motor aft station.
        let m = rocket.motor_mounts().into_iter().find(|m| m.id == motor.part()).unwrap();
        assert!((motor.end - m.motor_aft_station).abs() < 1e-9);
    }

    #[test]
    fn empty_lanes_are_hidden_and_short_parts_open_no_row() {
        let mut rocket = Rocket::default();
        let nose = rocket.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }).unwrap();
        let tube = rocket.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let _ = nose;
        let ring = |r: &mut Rocket, at: f64| {
            r.attach_internal(tube, at, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 }).unwrap()
        };
        let a = ring(&mut rocket, 0.1);
        let b = ring(&mut rocket, 0.3);
        let scene = build(&rocket, false, Phase::Full);
        assert_eq!(scene.lanes, vec![Lane { system: System::Structure, subs: 1 }]);
        assert_eq!(scene.part_bar(a).unwrap().sub, 0);
        assert_eq!(scene.part_bar(b).unwrap().sub, 0);
        assert!(scene.part_bar(a).unwrap().thin && !scene.part_bar(a).unwrap().warn);
    }

    #[test]
    fn two_discs_at_one_station_are_flagged() {
        let mut rocket = Rocket::default();
        rocket.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }).unwrap();
        let tube = rocket.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let kind = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
        let a = rocket.attach_internal(tube, 0.1, kind.clone()).unwrap();
        let b = rocket.attach_internal(tube, 0.102, kind.clone()).unwrap();
        let c = rocket.attach_internal(tube, 0.3, kind).unwrap();
        let scene = build(&rocket, false, Phase::Full);
        assert!(scene.part_bar(a).unwrap().warn && scene.part_bar(b).unwrap().warn);
        assert!(!scene.part_bar(c).unwrap().warn);
    }

    #[test]
    fn overlapping_regular_bars_open_sub_rows_and_touching_ones_do_not() {
        let rocket = example();
        let scene = build(&rocket, false, Phase::Full);
        // Within one lane and sub-row, regular bars never overlap.
        for a in scene.bars.iter().filter(|b| !b.thin) {
            for b in scene.bars.iter().filter(|b| !b.thin && b.key != a.key && b.system == a.system && b.sub == a.sub) {
                assert!(a.end <= b.start + 1e-6 || b.end <= a.start + 1e-6, "{} overlaps {}", a.name, b.name);
            }
        }
    }

    #[test]
    fn comparing_adds_the_other_configurations_motors_as_dashed_bars() {
        let mut rocket = example();
        let first = rocket.active_config_id();
        let second = rocket.duplicate_configuration(first).unwrap();
        let mount = rocket.motor_mounts()[0].id;
        rocket.set_motor(second, mount, Some(bowshock_model::MotorRef::new("I284W", "Aerotech"))).unwrap();
        let alone = build(&rocket, false, Phase::Full);
        let all = build(&rocket, true, Phase::Full);
        let motors = |s: &Scene| s.bars.iter().filter(|b| matches!(b.key, BarKey::Motor { .. })).count();
        assert_eq!((motors(&alone), motors(&all)), (1, 2));
        let alt = all.bars.iter().find(|b| b.alt).unwrap();
        assert!(alt.dashed && alt.sub >= 1);
    }

    #[test]
    fn the_phase_fades_what_has_dropped_away() {
        let mut rocket = Rocket::default();
        rocket.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }).unwrap();
        let sustainer = rocket.insert(1, PartKind::BodyTube { length: 0.6, diameter: 0.1 }).unwrap();
        let booster = rocket.insert(2, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let mount = |r: &mut Rocket, tube: PartId| {
            r.attach_internal(tube, 0.3, InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.002, length: 0.25, motor_mount: true, overhang: 0.0 })
                .unwrap()
        };
        mount(&mut rocket, sustainer);
        mount(&mut rocket, booster);
        let stages = rocket.stages();
        assert_eq!(stages.len(), 2, "two mounts in line are two stages");
        let full = build(&rocket, false, Phase::Full);
        let upper = build(&rocket, false, Phase::UpperFrom(2));
        assert!(full.bars.iter().all(|b| !b.dropped) && full.dropped_from.is_none());
        assert_eq!(full.seps.len(), 1);
        assert_eq!((full.seps[0].upper.as_str(), full.seps[0].lower.as_str()), ("Stage 2", "Stage 1"));
        let b = upper.part_bar(booster).unwrap();
        assert!(b.dropped && !upper.part_bar(sustainer).unwrap().dropped);
        assert!((upper.dropped_from.unwrap() - 0.8).abs() < 1e-9);
        assert!(upper.seps[0].lower_dropped);
    }
}

#[cfg(test)]
mod pod_tests {
    use super::*;
    use bowshock_model::{MassRole, NoseShape};

    /// A 0.2 m nose and a 0.8 m tube carrying a pod set of two standard pods at 0.2 m of the tube, with three fins on the pod's
    /// tube and a ballast in it. Returns the rocket, the pod set, the pod's tube, its fins and the ballast.
    fn with_pods() -> (Rocket, PartId, PartId, PartId, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }).unwrap();
        let core = r.insert(1, PartKind::BodyTube { length: 0.8, diameter: 0.1 }).unwrap();
        let kind = r.default_pod();
        let pod = r.attach_radial(core, 0.2, 0.0, 2, kind).unwrap();
        let tube = r.pods().next().and_then(|p| p.kind.pod_body()).unwrap().parts().last().unwrap().id;
        let fins =
            r.attach_radial(tube, 0.1, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.04, sweep: 0.03, thickness: 0.003 }).unwrap();
        let ballast = r.attach_internal(tube, 0.05, InternalKind::MassComponent { mass: 0.1, length: 0.05, diameter: None, role: MassRole::Ballast }).unwrap();
        (r, pod, tube, fins, ballast)
    }

    fn build(rocket: &Rocket) -> Scene {
        let db = MotorDb::bundled();
        let stages = rocket.stages();
        Scene::build(&Inputs { rocket, stages: &stages, phase: Phase::Full, motors: &db, units: UnitSystem::MmGS, compare: false })
    }

    #[test]
    fn a_pod_set_is_a_structure_bar_labelled_with_its_count_and_selects_the_pod() {
        let (rocket, pod, ..) = with_pods();
        let scene = build(&rocket);
        let bar = scene.part_bar(pod).expect("the pod has a bar");
        assert_eq!((bar.system, bar.name.as_str(), bar.count, bar.class), (System::Structure, "Pod ×2", 2, Some(PartClass::Radial)));
        assert_eq!(bar.part(), pod);
        // It spans the pod's stations, and is on no pod itself.
        let (fore, aft) = rocket.part_span(pod).unwrap();
        assert!((bar.start - fore).abs() < 1e-9 && (bar.end - aft).abs() < 1e-9 && (fore - 0.4).abs() < 1e-9);
        assert!(bar.pod.is_none() && bar.edit.movable && !bar.edit.resize_aft);
        assert!(scene.lanes.iter().any(|l| l.system == System::Structure));
    }

    #[test]
    fn the_parts_of_a_pod_sit_in_their_own_lanes_at_their_stations_grouped_as_the_pod() {
        let (rocket, pod, tube, fins, ballast) = with_pods();
        let scene = build(&rocket);
        let body = rocket.pods().next().and_then(|p| p.kind.pod_body()).unwrap();
        // Every part of the pod has a bar in its system's lane, bracketed `Pod ×2`.
        for id in body.ids() {
            let bar = scene.part_bar(id).unwrap_or_else(|| panic!("no bar for {id}"));
            let mark = bar.pod.as_ref().expect("a member of the pod");
            assert_eq!((mark.id, mark.label.as_str(), mark.count), (pod, "Pod ×2", 2));
            assert_eq!(bar.system, rocket.kind_of(id).unwrap().system());
            let (fore, aft) = rocket.part_span(id).unwrap();
            assert!((bar.start - fore).abs() < 1e-9 && (bar.end - aft).abs() < 1e-9, "{}", bar.name);
            assert!(scene.lanes.iter().any(|l| l.system == bar.system && bar.sub < l.subs));
        }
        // The fins are in Aero with their own count, the ballast in Avionics, the tube in Structure.
        let fin_bar = scene.part_bar(fins).unwrap();
        assert_eq!((fin_bar.system, fin_bar.name.as_str(), fin_bar.count, fin_bar.fin), (System::Aero, "Fins ×3", 3, true));
        assert_eq!(scene.part_bar(ballast).unwrap().system, System::Avionics);
        let tube_bar = scene.part_bar(tube).unwrap();
        assert_eq!(tube_bar.system, System::Structure);
        // The pod tube is where the model puts it: behind the pod's nose, which starts at the pod's station.
        assert!((tube_bar.start - (0.4 + body.parts()[0].kind.length())).abs() < 1e-9);
        // A bar is only grouped with its own pod: the core has no marks, and each id has exactly one bar.
        assert!(scene.bars.iter().filter(|b| b.pod.is_none()).all(|b| rocket.pod_of(b.part()).is_none()));
        let n = rocket.ids().len();
        assert_eq!(scene.bars.iter().filter(|b| matches!(b.key, BarKey::Part(_)) && !b.shoulder).count(), n);
        // Hovering names the pod.
        assert!(describe(fin_bar, &rocket, UnitSystem::MmGS).ends_with("in Pod ×2"));
        // The pod's mass is on the mass shading.
        assert!(scene.mass.iter().sum::<f32>() > 0.0);
    }

    #[test]
    fn a_single_pod_is_just_called_pod_and_a_second_set_is_grouped_apart() {
        let (mut rocket, pod, ..) = with_pods();
        assert!(rocket.set_radial(pod, rocket.radial(pod).unwrap().kind.clone(), 1));
        let core = rocket.parts().last().unwrap().id;
        let other_kind = rocket.default_pod();
        let other = rocket.attach_radial(core, 0.5, 1.0, 3, other_kind).unwrap();
        let scene = build(&rocket);
        assert_eq!(scene.part_bar(pod).unwrap().name, "Pod");
        assert!(scene.part_bar(pod).unwrap().count == 1);
        assert_eq!(scene.part_bar(other).unwrap().name, "Pod ×3");
        let first_nose = rocket.pods().find(|p| p.id == pod).and_then(|p| p.kind.pod_body()).unwrap().parts()[0].id;
        let second_nose = rocket.pods().find(|p| p.id == other).and_then(|p| p.kind.pod_body()).unwrap().parts()[0].id;
        assert_eq!(scene.part_bar(first_nose).unwrap().pod.as_ref().map(|m| m.id), Some(pod));
        assert_eq!(scene.part_bar(second_nose).unwrap().pod.as_ref().map(|m| (m.id, m.label.as_str())), Some((other, "Pod ×3")));
    }

    /// An inner tube that is a motor mount, `length` long, 0.05 m down the tube `parent`.
    fn mount(r: &mut Rocket, parent: PartId, length: f64) -> PartId {
        r.attach_internal(parent, 0.05, InternalKind::InnerTube { outer_diameter: 0.03, thickness: 0.002, length, motor_mount: true, overhang: 0.0 }).unwrap()
    }

    fn pod_tube(r: &Rocket, pod: PartId) -> PartId {
        r.pods().find(|p| p.id == pod).and_then(|p| p.kind.pod_body()).unwrap().parts().last().unwrap().id
    }

    #[test]
    fn a_pod_mounts_motor_is_one_bar_in_the_propulsion_lane_labelled_with_its_copies_and_grouped_with_the_pod() {
        let (mut rocket, pod, tube, ..) = with_pods();
        let in_pod = mount(&mut rocket, tube, 0.4);
        let config = rocket.active_config_id();
        rocket.set_motor(config, in_pod, Some(bowshock_model::MotorRef::new("H128W", "Aerotech"))).unwrap();
        // The core has a mount and a motor of its own, so the scene mixes both kinds.
        let core = rocket.parts().last().unwrap().id;
        let core_mount = mount(&mut rocket, core, 0.7);
        rocket.set_motor(config, core_mount, Some(bowshock_model::MotorRef::new("I284W", "Aerotech"))).unwrap();

        let mounts = rocket.motor_mounts();
        assert_eq!(mounts.len(), 2);
        let m = mounts.iter().find(|m| m.id == in_pod).unwrap();
        assert_eq!((m.pod, m.copies), (Some(pod), 2));

        let scene = build(&rocket);
        // One bar for the pod's mount, however many copies; one for the core's.
        let motors: Vec<&Bar> = scene.bars.iter().filter(|b| matches!(b.key, BarKey::Motor { .. })).collect();
        assert_eq!(motors.len(), 2);
        let bar = motors.iter().find(|b| b.part() == in_pod).expect("the pod's motor has a bar");
        let motor = MotorDb::bundled().find("H128W").unwrap().length;
        assert_eq!((bar.system, bar.name.as_str(), bar.count), (System::Propulsion, "H128W ×2", 2));
        // At the mount's own station (its aft end is the motor's), inside the pod's bracket, under the pod's inner tube.
        assert!((bar.end - m.motor_aft_station).abs() < 1e-9 && (bar.start - (m.motor_aft_station - motor)).abs() < 1e-9);
        assert_eq!(bar.container, Some((m.fore_station, m.aft_station)));
        let mark = bar.pod.as_ref().expect("grouped with its pod");
        assert_eq!((mark.id, mark.label.as_str(), mark.count), (pod, "Pod ×2", 2));
        assert!(scene.part_bar(in_pod).unwrap().sub < bar.sub, "the motor sits in a row under its mount");
        assert!((scene.part_bar(in_pod).unwrap().start - m.fore_station).abs() < 1e-9);
        // The core's motor is a plain bar.
        let plain = motors.iter().find(|b| b.part() == core_mount).unwrap();
        assert_eq!((plain.name.as_str(), plain.count, plain.pod.is_none()), ("I284W", 1, true));
        // Hovering names the motor and its pod.
        assert!(describe(bar, &rocket, UnitSystem::MmGS).ends_with("in Pod ×2"));
        // The pod's mount is not a stage: the core's one mount is the only one the stage lists.
        let stages = rocket.stages();
        assert_eq!(stages.len(), 1);
        assert_eq!(stages[0].mounts, vec![core_mount]);
        assert_eq!(rocket.stage_of(in_pod), Some(stages[0].index));
    }

    #[test]
    fn a_pods_motor_drops_away_with_the_stage_the_pod_stands_on() {
        let mut rocket = Rocket::default();
        rocket.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }).unwrap();
        let sustainer = rocket.insert(1, PartKind::BodyTube { length: 0.6, diameter: 0.1 }).unwrap();
        let booster = rocket.insert(2, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let (s_mount, b_mount) = (mount(&mut rocket, sustainer, 0.25), mount(&mut rocket, booster, 0.25));
        assert_eq!(rocket.stages().len(), 2, "two mounts in line are two stages");
        // A strap-on on each stage, each with a motor of its own.
        let kind = rocket.default_pod();
        let on_booster = rocket.attach_radial(booster, 0.02, 0.0, 2, kind.clone()).unwrap();
        let on_sustainer = rocket.attach_radial(sustainer, 0.02, 0.0, 3, kind).unwrap();
        let config = rocket.active_config_id();
        let strap = |r: &mut Rocket, pod: PartId| {
            let m = mount(r, pod_tube(r, pod), 0.2);
            r.set_motor(config, m, Some(bowshock_model::MotorRef::new("I284W", "Aerotech"))).unwrap();
            m
        };
        let (booster_strap, sustainer_strap) = (strap(&mut rocket, on_booster), strap(&mut rocket, on_sustainer));

        // Only the two core mounts are stages; each strap-on's mount belongs to the stage of the part its pod stands on.
        let stages = rocket.stages();
        assert_eq!(stages.iter().map(|s| s.mounts.clone()).collect::<Vec<_>>(), vec![vec![b_mount], vec![s_mount]]);
        assert_eq!((rocket.stage_of(booster_strap), rocket.stage_of(sustainer_strap)), (Some(1), Some(2)));

        let build_phase = |phase| {
            let db = MotorDb::bundled();
            Scene::build(&Inputs { rocket: &rocket, stages: &stages, phase, motors: &db, units: UnitSystem::MmGS, compare: false })
        };
        let full = build_phase(Phase::Full);
        assert!(full.bars.iter().all(|b| !b.dropped));
        let upper = build_phase(Phase::UpperFrom(2));
        let bar = |scene: &Scene, mount: PartId| scene.bars.iter().find(|b| b.key.part() == mount && matches!(b.key, BarKey::Motor { .. })).unwrap().dropped;
        assert!(bar(&upper, booster_strap), "the booster's strap-on is gone with the booster");
        assert!(!bar(&upper, sustainer_strap), "the sustainer's strap-on flies on");
        // So is everything else in each pod, and the pod set itself.
        for (pod, gone) in [(on_booster, true), (on_sustainer, false)] {
            assert_eq!(upper.part_bar(pod).unwrap().dropped, gone);
            for id in rocket.pods().find(|p| p.id == pod).and_then(|p| p.kind.pod_body()).unwrap().ids() {
                assert_eq!(upper.part_bar(id).unwrap().dropped, gone, "part {id}");
            }
        }
    }
}
