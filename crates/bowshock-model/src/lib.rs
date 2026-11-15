//! Rocket document model for the Bowshock builder. All lengths are SI metres, masses kilograms, angles radians.
//!
//! A [`Rocket`] is an axial stack of [`Part`]s (nose, tubes, transitions) carrying [`RadialPart`] groups (fins, lugs, rail
//! buttons) on its outside and [`InternalPart`]s (inner tubes, rings, recovery gear, avionics) inside, all sharing one
//! [`PartId`] space. Parts have materials ([`MaterialId`]) and finishes ([`Finish`]), mass and CG follow from geometry and material
//! ([`MassProps`], [`Rocket::dry_mass`]), motor [`Configuration`]s reference motors by name, and [`Stage`]s are derived from the
//! motor mounts. Flights (stored Simulate scenarios, [`FlightDef`]) are part of the document too. A document saves to and loads
//! from JSON ([`Rocket::to_json`], [`Rocket::from_json`]).
//!
//! The model has no UI and no physics dependencies; `bowshock-sim` and the app read it.

mod bands;
mod cfd;
mod config;
mod expr;
mod flights;
mod geometry;
mod handles;
mod internal;
mod json;
mod mass;
mod material;
mod notes;
pub mod ork;
mod pod;
mod rocket;
mod snap;
mod stage;
mod symmetry;

/// Stable identifier of a part within a [`Rocket`], whatever class of part it is.
pub type PartId = u64;

pub use bands::StabilityBands;
pub use cfd::{CfdPoint, CfdQuality, CfdStore, CfdTable, MAX_CFD_TABLES};
pub use config::{Ballast, ConfigError, ConfigId, Configuration, EventKind, MotorMount, MotorRef, MotorSlot, RecoveryTiming, StageEvent, StageTrigger};
pub use expr::{dim_from_key, dim_key, dim_kind, EvalError, Expr, ExprError, FieldRef, Formula, FormulaError, FormulaStatus, Kind, Op, Value};
pub use flights::{
    example_flights, Conditions, FlightDef, FlightId, FlightMotor, PlotLayout, PlotPanel, PlotX, Turbulence, MAX_LAUNCH_ANGLE, MAX_PANEL_SOURCES,
    MAX_PLOT_PANELS,
};
pub use geometry::{Anchor, FinPlanform, FinSection, NoseShape, Part, PartKind, PodBody, RadialKind, RadialPart, Shoulder, MAX_COUNT};
pub use handles::{Dim, DimEdit, HandleKind, HandleSpec, HandleView};
pub use internal::{DeployEvent, Deployment, InternalKind, InternalPart, MassRole, PartClass, PartRef, System, AUTO_FILL};
pub use json::{JsonError, FORMAT_VERSION};
pub use mass::MassProps;
pub use material::{materials, materials_of_kind, Finish, Material, MaterialId, MaterialKind};
pub use pod::PodFrame;
pub use rocket::{InsertError, InternalError, RadialError, Resolved, Rocket};
pub use snap::Target;
pub use stage::{Separation, SplitError, Stage, StageIssue};
pub use symmetry::{instance_letter, FieldOverride, InstanceParams, RadialField, Winner};
