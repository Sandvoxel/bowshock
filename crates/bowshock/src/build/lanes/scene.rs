//! The lanes' data: the rocket turned into bars in stations (metres from the tip), sub-rows, stage marks and the mass profile.
//! Pure: no widgets and no pixels (those are `geometry` and `canvas`), so the packing and the numbers are unit-tested.

use bowshock_model::{ConfigId, InternalKind, PartClass, PartId, PartKind, PartRef, RadialKind, Rocket, Stage, System};
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
        let stations = rocket.stations();
        let mut bars: Vec<Bar> = Vec::new();

        // Airframe.
        for (i, part) in rocket.parts().iter().enumerate() {
            let (start, end) = (stations[i], stations[i + 1]);
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
                dropped: dropped(start),
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
                edit: Edit { movable: true, resize_aft: true, resize_fore: false },
            });
        }

        // Radial groups.
        for radial in rocket.radials() {
            let Some(start) = rocket.radial_station(radial) else { continue };
            let end = start + radial.kind.extent();
            let parent = rocket.part(radial.parent);
            let base = radial.name.clone().unwrap_or_else(|| match &radial.kind {
                k if k.is_fin() => "Fins".to_string(),
                RadialKind::LaunchLug { .. } => "Lug".to_string(),
                _ => "Rail".to_string(),
            });
            let name = if radial.count > 1 { format!("{base} ×{}", radial.count) } else { base };
            bars.push(Bar {
                key: BarKey::Part(radial.id),
                system: radial.kind.system(),
                start,
                end,
                name,
                dim: String::new(),
                sub: 0,
                overrides: radial.override_summary(),
                dropped: dropped(start),
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
                edit: Edit {
                    movable: true,
                    resize_aft: matches!(radial.kind, RadialKind::FinSet { .. } | RadialKind::EllipticalFin { .. } | RadialKind::LaunchLug { .. }),
                    resize_fore: false,
                },
            });
        }

        // Internal parts.
        for part in rocket.internals() {
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
                dropped: dropped(start),
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
                bars.push(Bar {
                    key: BarKey::Motor { mount: mount.id, config: config.id },
                    system: System::Propulsion,
                    start,
                    end,
                    name: motor.designation.clone(),
                    dim: len(motor.length),
                    sub: 0,
                    overrides: String::new(),
                    dropped: dropped(start),
                    count: 1,
                    class: None,
                    parent: None,
                    container: Some((mount.fore_station, mount.aft_station)),
                    container_word: "mount",
                    thin: false,
                    warn: false,
                    dashed: config.id != active,
                    alt: config.id != active,
                    fin: false,
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
        let mut faces: Vec<f64> = self.bars.iter().filter(|b| Some(b.key) != except && !b.alt).flat_map(|b| [b.start, b.end]).collect();
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
        // Current parts before the other configurations' motors, then fore to aft, longer first.
        idx.sort_by(|&a, &b| {
            let (x, y) = (&bars[a], &bars[b]);
            x.alt.cmp(&y.alt).then(x.start.total_cmp(&y.start)).then(y.end.total_cmp(&x.end))
        });
        let mut rows: Vec<f64> = Vec::new();
        let mut placed: Vec<(usize, usize)> = Vec::new(); // (bar, row) of the regular bars
        let (short, regular): (Vec<usize>, Vec<usize>) = idx.iter().copied().partition(|&i| bars[i].thin);
        for i in regular {
            let start = bars[i].start;
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
    if bar.name.eq_ignore_ascii_case(kind) || kind.is_empty() {
        format!("{} · {at}", bar.name)
    } else {
        format!("{} ({}) · {at}", bar.name, kind.to_lowercase())
    }
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
