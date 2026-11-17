//! Rocket document model for the Bowshock builder. All lengths are SI metres, masses kilograms, angles radians.
//!
//! A [`Rocket`] is an axial stack of [`Part`]s (nose, tubes, transitions) carrying [`RadialPart`] groups (fins, lugs, rail
//! buttons) on its outside and [`InternalPart`]s (inner tubes, rings, recovery gear, avionics) inside, all sharing one
//! [`PartId`] space. Parts have materials ([`MaterialId`]) and finishes ([`Finish`]), mass and CG follow from geometry and material
//! ([`MassProps`], [`Rocket::dry_mass`]), motor [`Configuration`]s reference motors by name, and [`Stage`]s are derived from the
//! motor mounts. Flights (stored Simulate scenarios, [`FlightDef`]) are part of the document too. A document saves to and loads
//! from the binary `.bowshock` format ([`Rocket::to_bytes`], [`Rocket::from_bytes`]).
//!
//! A rocket also holds reusable assemblies (a bay, a stage, a set of fins): [`AssemblyDef`]s with [`AssemblyInstance`]s whose members
//! are ordinary parts ([`Rocket::group`], [`Rocket::place`], [`Rocket::update_assembly`]).
//!
//! The model has no UI and no physics dependencies; `bowshock-sim` and the app read it.

mod assembly;
mod bands;
mod cfd;
mod config;
mod expr;
mod fin_tab;
mod flights;
mod format;
mod geometry;
mod handles;
mod internal;
mod mass;
mod material;
mod nose;
mod notes;
pub mod ork;
mod pod;
mod rocket;
mod scale;
mod snap;
mod stage;
mod symmetry;

/// Stable identifier of a part within a [`Rocket`], whatever class of part it is.
pub type PartId = u64;

pub use assembly::{
    AssemblyDef, AssemblyError, AssemblyId, AssemblyInstance, Fit, Fragment, FragmentKind, Host, InstanceId, InstanceUpdate, Placed, Placement,
    ASSEMBLY_FORMAT_VERSION, ASSEMBLY_MAGIC,
};

pub use bands::StabilityBands;
pub use cfd::{CfdGrid, CfdLiftPoint, CfdLiftTable, CfdPoint, CfdQuality, CfdStore, CfdTable, MAX_CFD_TABLES};
pub use config::{Ballast, ConfigError, ConfigId, Configuration, EventKind, MotorMount, MotorRef, MotorSlot, RecoveryTiming, StageEvent, StageTrigger};
pub use expr::{dim_from_key, dim_key, dim_kind, EvalError, Expr, ExprError, FieldRef, Formula, FormulaError, FormulaStatus, Kind, Op, Value};
pub use fin_tab::{TabIssue, TabTarget};
pub use flights::{
    example_flights, sanitized_levels, AirStart, Conditions, DragSource, FlightDef, FlightId, FlightMotor, Location, PlotLayout, PlotPanel, PlotX, Turbulence,
    WindLevel, MAX_LAUNCH_ANGLE, MAX_PANEL_SOURCES, MAX_PLOT_PANELS, MAX_WIND_LEVELS,
};
pub use format::{FormatError, FORMAT_VERSION, MAGIC};
pub use geometry::{Anchor, FinPlanform, FinSection, FinTab, NoseShape, Part, PartKind, PodBody, RadialKind, RadialPart, Shoulder, MAX_COUNT};
pub use handles::{Dim, DimEdit, HandleKind, HandleSpec, HandleView};
pub use internal::{DeployEvent, Deployment, InternalKind, InternalPart, MassRole, PartClass, PartRef, System, AUTO_FILL};
pub use mass::MassProps;
pub use material::{materials, materials_of_kind, Finish, Material, MaterialId, MaterialKind};
pub use nose::{blunting, NoseProfile};
pub use pod::PodFrame;
pub use rocket::{InsertError, InternalError, RadialError, Resolved, Rocket};
pub use scale::ScaleScope;
pub use snap::Target;
pub use stage::{JointOption, Separation, SplitError, Stage, StageIssue};
pub use symmetry::{instance_letter, FieldOverride, InstanceParams, RadialField, Winner};
