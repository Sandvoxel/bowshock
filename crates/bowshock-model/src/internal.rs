//! Internal components (inner tubes, rings, bulkheads, recovery gear, avionics) and the system lanes parts belong to.

use serde::{Deserialize, Serialize};

use crate::geometry::{Part, PartKind, RadialKind, RadialPart};
use crate::material::{MaterialId, MaterialKind};
use crate::PartId;

/// The systems of a rocket: one lane per system in Station Lanes, one colour per system everywhere.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum System {
    /// Nose cone, body tubes, transitions, couplers, bulkheads, centering rings.
    Structure,
    /// Fins, launch lugs, rail buttons.
    Aero,
    /// Motor mount tube, engine block (and the motor).
    Propulsion,
    /// Parachutes, streamers, shock cord.
    Recovery,
    /// Altimeters, batteries, trackers, payload, ballast.
    Avionics,
}

impl System {
    /// Every system, in lane order.
    pub const ALL: [System; 5] = [System::Structure, System::Aero, System::Propulsion, System::Recovery, System::Avionics];

    /// Display name of the lane.
    pub fn name(self) -> &'static str {
        match self {
            System::Structure => "Structure",
            System::Aero => "Aero",
            System::Propulsion => "Propulsion",
            System::Recovery => "Recovery",
            System::Avionics => "Avionics & payload",
        }
    }
}

/// What a recovery device waits for before it deploys.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum DeployEvent {
    /// At apogee.
    Apogee,
    /// On descent through this altitude above the launch site, metres.
    Altitude(f64),
    /// With the motor's ejection charge.
    Ejection,
}

/// When a recovery device deploys: an event and a delay after it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Deployment {
    /// The trigger.
    pub event: DeployEvent,
    /// Seconds after the trigger.
    pub delay: f64,
}

impl Default for Deployment {
    fn default() -> Self {
        Deployment { event: DeployEvent::Apogee, delay: 0.0 }
    }
}

/// What a [`InternalKind::MassComponent`] is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MassRole {
    /// Flight computer.
    Altimeter,
    /// Battery.
    Battery,
    /// GPS or radio tracker.
    Tracker,
    /// Payload.
    Payload,
    /// Ballast.
    Ballast,
    /// Anything else.
    Generic,
}

impl MassRole {
    /// Every role.
    pub const ALL: [MassRole; 6] = [MassRole::Altimeter, MassRole::Battery, MassRole::Tracker, MassRole::Payload, MassRole::Ballast, MassRole::Generic];

    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            MassRole::Altimeter => "Altimeter",
            MassRole::Battery => "Battery",
            MassRole::Tracker => "Tracker",
            MassRole::Payload => "Payload",
            MassRole::Ballast => "Ballast",
            MassRole::Generic => "Mass",
        }
    }
}

/// Geometry of an internal component: it lives inside an axial part and is placed by its `offset` from that part's fore face.
///
/// Diameters given as `Option` are **auto**: `None` follows the parent (see [`Rocket::resolved`](crate::Rocket::resolved)),
/// so the UI can show them as `AUTO` and they follow when the parent is resized.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum InternalKind {
    /// A tube inside the airframe, typically the motor mount.
    InnerTube {
        /// Outer diameter.
        outer_diameter: f64,
        /// Wall thickness.
        thickness: f64,
        /// Length along the axis.
        length: f64,
        /// Whether a motor goes in it.
        motor_mount: bool,
        /// How far the motor's aft end sticks out behind the tube's aft end (negative: recessed).
        overhang: f64,
    },
    /// An annular disc that centres an inner tube in the body tube.
    CenteringRing {
        /// Outer diameter; auto is the parent's inner diameter.
        outer_diameter: Option<f64>,
        /// Inner diameter; auto is the outer diameter of the inner tube it sits on, else 0 (a solid disc).
        inner_diameter: Option<f64>,
        /// Thickness along the axis.
        thickness: f64,
    },
    /// A solid disc closing the tube.
    Bulkhead {
        /// Diameter; auto is the parent's inner diameter.
        diameter: Option<f64>,
        /// Thickness along the axis.
        thickness: f64,
    },
    /// A tube joining two airframe sections, sliding inside both.
    Coupler {
        /// Outer diameter; auto is the parent's inner diameter.
        outer_diameter: Option<f64>,
        /// Wall thickness.
        thickness: f64,
        /// Length along the axis.
        length: f64,
    },
    /// A ring that stops the motor sliding forward.
    EngineBlock {
        /// Outer diameter; auto is the inner diameter of the inner tube it sits in, else of the parent.
        outer: Option<f64>,
        /// Inner (hole) diameter.
        inner: f64,
        /// Thickness along the axis.
        thickness: f64,
    },
    /// A parachute: canopy and shroud lines, packed.
    Parachute {
        /// Canopy diameter (flat).
        diameter: f64,
        /// Drag coefficient on the canopy area.
        cd: f64,
        /// Number of shroud lines.
        line_count: u32,
        /// Length of each shroud line.
        line_length: f64,
        /// What the lines are made of (a line material); the canopy fabric is the part's `material`.
        line_material: MaterialId,
        /// Length it takes up along the axis when packed.
        packed_length: f64,
        /// Diameter of the packed bundle; auto is a fraction of the parent's inner diameter.
        packed_diameter: Option<f64>,
        /// When it deploys.
        deploy: Deployment,
    },
    /// A streamer: a strip of fabric.
    Streamer {
        /// Length of the strip.
        length: f64,
        /// Width of the strip.
        width: f64,
        /// Drag coefficient on the strip's area.
        cd: f64,
        /// When it deploys. It packs as a roll as long as its width.
        deploy: Deployment,
    },
    /// Shock cord (the harness joining sections).
    ShockCord {
        /// Deployed length of cord.
        length: f64,
        /// Length it takes up along the axis when packed.
        packed_length: f64,
    },
    /// A lump of mass: avionics, battery, payload, ballast.
    MassComponent {
        /// Mass, kg.
        mass: f64,
        /// Length along the axis.
        length: f64,
        /// Diameter (a solid cylinder); auto is a fraction of the parent's inner diameter.
        diameter: Option<f64>,
        /// What it is.
        role: MassRole,
    },
}

/// Fraction of the parent's inner diameter an auto-sized packed bundle (parachute, streamer, cord, mass) fills.
pub const AUTO_FILL: f64 = 0.8;

impl InternalKind {
    /// Display name.
    pub fn name(&self) -> &'static str {
        match self {
            InternalKind::InnerTube { motor_mount: true, .. } => "Motor mount",
            InternalKind::InnerTube { .. } => "Inner tube",
            InternalKind::CenteringRing { .. } => "Centering ring",
            InternalKind::Bulkhead { .. } => "Bulkhead",
            InternalKind::Coupler { .. } => "Coupler",
            InternalKind::EngineBlock { .. } => "Engine block",
            InternalKind::Parachute { .. } => "Parachute",
            InternalKind::Streamer { .. } => "Streamer",
            InternalKind::ShockCord { .. } => "Shock cord",
            InternalKind::MassComponent { role, .. } => role.name(),
        }
    }

    /// The lane this part is drawn in and the colour it takes (see the Station Lanes table).
    pub fn system(&self) -> System {
        match self {
            InternalKind::InnerTube { motor_mount: true, .. } | InternalKind::EngineBlock { .. } => System::Propulsion,
            InternalKind::InnerTube { .. } | InternalKind::CenteringRing { .. } | InternalKind::Bulkhead { .. } | InternalKind::Coupler { .. } => {
                System::Structure
            }
            InternalKind::Parachute { .. } | InternalKind::Streamer { .. } | InternalKind::ShockCord { .. } => System::Recovery,
            InternalKind::MassComponent { .. } => System::Avionics,
        }
    }

    /// The sort of stock the part's `material` must be: surface for a parachute canopy or streamer, line for shock cord, bulk
    /// for tubes and discs; `None` for a mass component (any).
    pub fn material_kind(&self) -> Option<MaterialKind> {
        match self {
            InternalKind::Parachute { .. } | InternalKind::Streamer { .. } => Some(MaterialKind::Surface),
            InternalKind::ShockCord { .. } => Some(MaterialKind::Line),
            InternalKind::MassComponent { .. } => None,
            _ => Some(MaterialKind::Bulk),
        }
    }

    /// Length it takes up along the axis.
    pub fn extent(&self) -> f64 {
        match *self {
            InternalKind::InnerTube { length, .. } | InternalKind::Coupler { length, .. } | InternalKind::MassComponent { length, .. } => length,
            InternalKind::CenteringRing { thickness, .. } | InternalKind::Bulkhead { thickness, .. } | InternalKind::EngineBlock { thickness, .. } => thickness,
            InternalKind::Parachute { packed_length, .. } | InternalKind::ShockCord { packed_length, .. } => packed_length,
            InternalKind::Streamer { width, .. } => width,
        }
    }

    /// The stock a new part of this kind is made of.
    pub fn default_material(&self) -> MaterialId {
        match self {
            InternalKind::InnerTube { .. } | InternalKind::Coupler { .. } => MaterialId::CARDBOARD,
            InternalKind::CenteringRing { .. } | InternalKind::Bulkhead { .. } | InternalKind::EngineBlock { .. } => MaterialId::BIRCH_PLYWOOD,
            InternalKind::Parachute { .. } => MaterialId::RIPSTOP_NYLON,
            InternalKind::Streamer { .. } => MaterialId::MYLAR,
            InternalKind::ShockCord { .. } => MaterialId::BRAIDED_NYLON_3MM,
            InternalKind::MassComponent { role: MassRole::Ballast, .. } => MaterialId::LEAD,
            InternalKind::MassComponent { .. } => MaterialId::ALUMINUM,
        }
    }

    /// Whether this is an inner tube a motor goes in.
    pub fn is_motor_mount(&self) -> bool {
        matches!(self, InternalKind::InnerTube { motor_mount: true, .. })
    }
}

/// An internal component placed in an axial part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InternalPart {
    /// Unique id, from the same sequence as axial part ids.
    pub id: PartId,
    /// Geometry.
    pub kind: InternalKind,
    /// The axial part it sits in.
    pub parent: PartId,
    /// Distance from the parent's fore face to the fore edge of the part.
    pub offset: f64,
    /// User-given name, `None` for the kind's name.
    #[serde(default)]
    pub name: Option<String>,
    /// What it is made of: bulk stock for tubes and discs, a surface material for a parachute canopy or streamer, a line
    /// material for shock cord. Ignored by mass components.
    pub material: MaterialId,
    /// Mass that replaces the computed one, kg.
    #[serde(default)]
    pub mass_override: Option<f64>,
    /// CG that replaces the computed one, metres behind the part's fore face.
    #[serde(default)]
    pub cg_override: Option<f64>,
}

impl InternalPart {
    /// A part with the default material for its kind and no overrides.
    pub fn new(id: PartId, kind: InternalKind, parent: PartId, offset: f64) -> InternalPart {
        let material = kind.default_material();
        InternalPart { id, kind, parent, offset, name: None, material, mass_override: None, cg_override: None }
    }

    /// The user's name, or the kind's ("Motor mount").
    pub fn display_name(&self) -> String {
        self.name.clone().unwrap_or_else(|| self.kind.name().to_string())
    }
}

/// A part of any class, borrowed from its [`Rocket`](crate::Rocket).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PartRef<'a> {
    /// Nose cone, body tube or transition.
    Axial(&'a Part),
    /// Fins, lug or rail button group.
    Radial(&'a RadialPart),
    /// Internal component.
    Internal(&'a InternalPart),
}

/// Which class a part is of.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PartClass {
    /// In the axial stack.
    Axial,
    /// Attached to the outside of an axial part.
    Radial,
    /// Inside an axial part.
    Internal,
}

impl PartRef<'_> {
    /// The part's id.
    pub fn id(&self) -> PartId {
        match self {
            PartRef::Axial(p) => p.id,
            PartRef::Radial(p) => p.id,
            PartRef::Internal(p) => p.id,
        }
    }

    /// Which class it is of.
    pub fn class(&self) -> PartClass {
        match self {
            PartRef::Axial(_) => PartClass::Axial,
            PartRef::Radial(_) => PartClass::Radial,
            PartRef::Internal(_) => PartClass::Internal,
        }
    }

    /// The system lane it belongs to.
    pub fn system(&self) -> System {
        match self {
            PartRef::Axial(p) => p.kind.system(),
            PartRef::Radial(p) => p.kind.system(),
            PartRef::Internal(p) => p.kind.system(),
        }
    }

    /// The kind's name ("Body tube", "Fin set", "Motor mount").
    pub fn kind_name(&self) -> &'static str {
        match self {
            PartRef::Axial(p) => p.kind.name(),
            PartRef::Radial(p) => p.kind.name(),
            PartRef::Internal(p) => p.kind.name(),
        }
    }

    /// The user's name, or the kind's.
    pub fn name(&self) -> String {
        match self {
            PartRef::Axial(p) => p.display_name(),
            PartRef::Radial(p) => p.display_name(),
            PartRef::Internal(p) => p.display_name(),
        }
    }

    /// The part this one sits on or in; `None` for axial parts.
    pub fn parent(&self) -> Option<PartId> {
        match self {
            PartRef::Axial(_) => None,
            PartRef::Radial(p) => Some(p.parent),
            PartRef::Internal(p) => Some(p.parent),
        }
    }
}

impl PartKind {
    /// The lane: nose cones, tubes and transitions are airframe, so [`System::Structure`].
    pub fn system(&self) -> System {
        System::Structure
    }
}

impl RadialKind {
    /// The lane: fins, lugs and rail buttons are all [`System::Aero`].
    pub fn system(&self) -> System {
        System::Aero
    }
}
