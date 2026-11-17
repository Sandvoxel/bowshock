//! The context every area shares: [`Ctx`], what is selected, the mode on show and the file command held by the unsaved-changes modal.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use bowshock_model::PartId;
use iced::Point;

use super::document::Document;
use super::drag::Drags;
use super::physics::Physics;
use super::{env, files, live, multi, part_menu, refs, scale, selection_set, symmetry};
use crate::tokens::{self, Tokens};
use crate::units::UnitSystem;

/// How long a status message stays up.
const NOTICE_TIME: Duration = Duration::from_secs(3);

/// The three modes of the top bar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Build the rocket.
    Build,
    /// Define and run flights.
    Simulate,
    /// Aerodynamics across the speed range.
    Analyze,
}

/// What is selected, app-wide: the viewport, the lanes and the inspector all read and set it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Selection {
    /// Nothing: the inspector shows the rocket.
    #[default]
    None,
    /// A part of any class (axial, radial group or internal), by id.
    Part(PartId),
    /// The separation of stage `n` from the stage ahead of it (`Stage::index`, from 2).
    Separation(usize),
}

impl Selection {
    /// The selected part, if a part is selected.
    pub fn part(self) -> Option<PartId> {
        match self {
            Selection::Part(id) => Some(id),
            _ => None,
        }
    }
}

/// A file command held back by the "Save changes?" modal until the user answers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Pending {
    /// File > New.
    New,
    /// Open (the file dialog comes after the answer).
    Open,
    /// Open the built-in example.
    Example,
    /// Import an OpenRocket design (the file dialog comes after the answer).
    Import,
    /// Open recent file (the path is already known).
    Recent(std::path::PathBuf),
    /// Quit the application (the window was asked to close).
    Exit,
}

/// Everything the areas share: the document and its physics, the selection, drags in progress, the unit system and the
/// handful of app-wide settings. Modules receive it in `update(.., ctx: &mut Ctx)` and `view(.., ctx: &Ctx)`.
pub struct Ctx {
    /// The open document. Every change to the rocket goes through its `edit*` methods.
    pub doc: Document,
    /// What is selected.
    pub selection: Selection,
    /// Drags in progress, and the radial placement settings.
    pub drag: Drags,
    /// Balance, warnings and motor queries for the document, kept current by the app after every message.
    pub physics: Physics,
    /// The display unit system (top bar).
    pub units: UnitSystem,
    /// Where the CFD solves run (Analyze, CFD table); kept in the user's settings.
    pub compute: super::settings::Compute,
    /// What the last successful connection test of the remote GPU found (`NVIDIA GeForce RTX 3090 on omarchy`); `None` until one ran.
    pub compute_device: Option<String>,
    /// The device the solves on this computer use (Preferences > Compute); kept in the user's settings.
    pub local_solver: super::settings::LocalSolver,
    /// Where the user's settings (the unit set) are written; `None` keeps them for the session only.
    pub settings_file: Option<PathBuf>,
    /// The process environment as read at start-up (see [`env`]).
    pub env: env::Env,
    /// The cursor in window coordinates, tracked app-wide so a drag can leave the parts bin.
    pub cursor: Option<Point>,
    /// The mode on show.
    pub mode: Mode,
    /// Dark theme (else light).
    pub dark: bool,
    /// "Compare all in lanes": the lanes draw every configuration's motor and CG.
    pub compare_configs: bool,
    /// Whether the viewport and the lanes draw the CG marker (the status strip's CG cell toggles it).
    pub show_cg: bool,
    /// Whether the viewport and the lanes draw the CP marker (the status strip's CP cell toggles it).
    pub show_cp: bool,
    /// A transient status message and when it expires.
    pub notice: Option<(String, Instant)>,
    /// What an OpenRocket import could not carry over exactly, one line each (the warnings chip lists them); cleared on every load.
    pub import_notes: Vec<String>,
    /// The File menu is open (the top bar's file name toggles it).
    pub file_menu: bool,
    /// The recently opened or saved documents (File menu).
    pub recent: files::Recent,
    /// Ask the viewport to frame the rocket once its mesh is current (set after adding a part or loading a document).
    pub camera_fit: bool,
    /// The document was just replaced: select nothing and let the gauge jump rather than animate.
    pub(super) reloaded: bool,
    /// A file command that would discard unsaved changes, waiting on the "Save changes?" modal (which is open while this is set).
    pub(super) unsaved: Option<Pending>,
    /// The same command while the Save As dialog the modal's Save opened is up; cancelling the dialog abandons it.
    pub(super) saving_for: Option<Pending>,
    /// Which copy of the selected symmetry group is the clicked one, and a pending rejoin (see [`symmetry`]).
    pub symmetry: symmetry::SymmetryUi,
    /// Pick mode and the formula UI's transient state (see [`refs`]).
    pub refs: refs::Ui,
    /// The part just dropped or edited and when: the viewport outlines it and the lanes flash its bar (see
    /// [`crate::viewport::flash`]).
    pub flash: Option<(PartId, Instant)>,
    /// The parts selected along with `selection`'s primary one (see [`multi`]).
    pub(super) multi: selection_set::MultiSet<PartId>,
    /// The baseline of the drag in progress, for the status strip's semibold readouts (see [`live`]).
    pub(super) live: live::Live,
    /// The balance of the part being carried as it would land (see [`live::Preview`]).
    pub preview: live::Preview,
    /// The part context menu (right-click on a part), if open (see [`part_menu`]).
    pub part_menu: Option<part_menu::Open>,
    /// The Scale overlay, if open (see [`scale`]).
    pub scale: Option<scale::Dialog>,
    /// The flight selected on the Simulate screen (the Optimize analysis starts from it).
    pub selected_flight: Option<bowshock_model::FlightId>,
}

impl Ctx {
    #[cfg(test)]
    pub(crate) fn new() -> Self {
        Ctx::with_env(env::Env::for_tests())
    }

    pub(super) fn with_env(env: env::Env) -> Self {
        Ctx {
            doc: Document::default(),
            selection: Selection::None,
            drag: Drags::default(),
            physics: Physics::from_env(&env),
            units: UnitSystem::default(),
            compute: super::settings::Compute::default(),
            compute_device: None,
            local_solver: super::settings::LocalSolver::default(),
            settings_file: env.settings_file.clone(),
            env,
            cursor: None,
            mode: Mode::Build,
            dark: true,
            compare_configs: false,
            show_cg: true,
            show_cp: true,
            notice: None,
            import_notes: Vec::new(),
            recent: files::Recent::default(),
            file_menu: false,
            camera_fit: false,
            reloaded: false,
            unsaved: None,
            saving_for: None,
            symmetry: symmetry::SymmetryUi::default(),
            refs: refs::Ui::default(),
            flash: None,
            multi: selection_set::MultiSet::default(),
            live: live::Live::default(),
            preview: live::Preview::default(),
            part_menu: None,
            scale: None,
            selected_flight: None,
        }
    }

    /// The design tokens of the current theme.
    pub fn tokens(&self) -> &'static Tokens {
        if self.dark {
            &tokens::DARK
        } else {
            &tokens::LIGHT
        }
    }

    /// Shows `message` in the status strip for a few seconds.
    pub fn notify(&mut self, message: impl Into<String>) {
        self.notice = Some((message.into(), Instant::now() + NOTICE_TIME));
    }

    /// Whether anything needs frame ticks because of the context itself.
    pub(super) fn animating(&self) -> bool {
        self.notice.is_some() || self.physics.animating() || (self.flash.is_some() && self.env.animate && !self.env.freeze_flash)
    }

    /// Starts the one-time flash and settle of part `id` (it was just dropped or its edit was committed).
    pub fn flash_part(&mut self, id: PartId) {
        if self.env.animate || self.env.freeze_flash {
            self.flash = Some((id, Instant::now()));
        }
    }

    /// The flash as the views draw it now: `(part, outline strength 0..1, height above its place in metres)`.
    pub fn flash_levels(&self) -> Option<(PartId, f32, f32)> {
        let (id, since) = self.flash?;
        let age = crate::viewport::flash::age(since, Instant::now(), self.env.freeze_flash);
        let level = crate::viewport::flash::level(age);
        (level > 0.0).then(|| (id, level, crate::viewport::flash::lift(age)))
    }

    /// Call after replacing the document wholesale (New, Open, the example).
    pub(crate) fn loaded(&mut self) {
        self.selection = Selection::None;
        self.flash = None;
        self.multi.clear();
        self.import_notes.clear();
        self.symmetry = symmetry::SymmetryUi::default();
        self.part_menu = None;
        self.scale = None;
        self.refs = refs::Ui::default();
        self.drag = Drags { symmetry: self.drag.symmetry, angle_snap: self.drag.angle_snap, ..Drags::default() };
        self.camera_fit = true;
        self.reloaded = true;
    }

    /// Drops a selection that no longer exists (after a delete, an undo, a load).
    pub(super) fn sanitize_selection(&mut self) {
        // A part being carried is out of the rocket but still selected.
        if self.drag.active() {
            return;
        }
        let rocket = self.doc.rocket();
        let exists = match self.selection {
            Selection::None => true,
            Selection::Part(id) => rocket.kind_of(id).is_some(),
            Selection::Separation(stage) => self.physics.stages().iter().any(|s| s.index == stage && s.separation.is_some()),
        };
        if !exists {
            self.selection = Selection::None;
        }
        multi::validate(&mut self.multi, self.selection, self.doc.rocket());
        self.symmetry.validate(self.doc.rocket());
        if self.symmetry.instance.is_some_and(|(id, _)| self.doc.rocket().radial(id).is_none()) {
            self.symmetry.instance = None;
        }
    }
}
