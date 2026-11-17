//! The open document: the rocket plus everything about it being a file (path, dirty flag) and its undo history.
//!
//! **Every mutation of the rocket goes through here**, so undo/redo, the dirty flag and the physics refresh (which watches
//! [`Document::revision`]) happen in one place:
//!
//! * [`Document::edit`]: one undo step. The closure mutates a `&mut Rocket`; a step is recorded only if it changed something.
//! * [`Document::edit_merging`]: edits sharing a key coalesce into one step until [`Document::seal`] (the app seals when the
//!   mouse is released), so scrubbing a field or dragging a resize handle is one undo step however many messages it sends.
//! * [`Document::begin_drag`] / [`Document::drag_update`] / [`Document::end_drag`]: a drag that takes parts out of the rocket
//!   and puts them back. The state before the drag is kept whole; cancelling restores it exactly (materials, finishes,
//!   overrides, internals, motors in every configuration), committing records one step.
//!
//! Files are the model's binary format ([`Rocket::to_bytes`] / [`Rocket::from_bytes`]: a magic header, then deflated CBOR) with
//! the `.bowshock` extension.

use std::fmt;
use std::path::{Path, PathBuf};

use bowshock_model::{FormatError, Rocket};

/// File extension of a Bowshock document (no dot).
pub const EXTENSION: &str = "bowshock";
/// Name shown for a document that has never been saved.
pub const UNTITLED: &str = "untitled";
/// Most steps of history kept.
const MAX_UNDO: usize = 200;

/// A state of the rocket to go back to, and what the edit that left it was called.
#[derive(Clone, Debug)]
struct Step {
    rocket: Rocket,
    label: String,
    /// How far the viewport's camera moved along the axis with this step (a fore-end length drag keeps the part's aft end
    /// still on screen); undo moves it back by as much, redo forward again. Metres.
    shift: f32,
}

/// Why a document could not be opened or saved.
#[derive(Debug)]
pub enum DocError {
    /// Reading or writing the file failed.
    Io(std::io::Error),
    /// The file is not a valid document.
    Format(FormatError),
    /// There is no path to save to (use Save As).
    NoPath,
}

impl fmt::Display for DocError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DocError::Io(e) => write!(f, "{e}"),
            DocError::Format(e) => write!(f, "{e}"),
            DocError::NoPath => f.write_str("the document has no file yet"),
        }
    }
}

impl std::error::Error for DocError {}

/// The open document. See the module docs.
#[derive(Debug)]
pub struct Document {
    rocket: Rocket,
    path: Option<PathBuf>,
    dirty: bool,
    undo: Vec<Step>,
    redo: Vec<Step>,
    revision: u64,
    /// The state before the drag in progress.
    drag: Option<Step>,
    /// Key of the coalescing run in progress.
    merging: Option<String>,
}

impl Default for Document {
    fn default() -> Self {
        Document::new(Rocket::default())
    }
}

impl Document {
    /// An unsaved document holding `rocket`, with no history.
    pub fn new(rocket: Rocket) -> Self {
        Document { rocket, path: None, dirty: false, undo: Vec::new(), redo: Vec::new(), revision: 0, drag: None, merging: None }
    }

    /// Reads the document at `path`.
    pub fn open(path: &Path) -> Result<Self, DocError> {
        let bytes = std::fs::read(path).map_err(DocError::Io)?;
        let rocket = Rocket::from_bytes(&bytes).map_err(DocError::Format)?;
        let mut doc = Document::new(rocket);
        doc.path = Some(path.to_path_buf());
        Ok(doc)
    }

    /// The rocket. Read it freely; change it only through the editing methods.
    pub fn rocket(&self) -> &Rocket {
        &self.rocket
    }

    /// Where the document is saved, if it has been.
    pub fn path(&self) -> Option<&Path> {
        self.path.as_deref()
    }

    /// The file name without its extension (`hpr_l1_cert`); an unsaved document shows the rocket's name, else [`UNTITLED`].
    pub fn title(&self) -> String {
        match self.path.as_ref().and_then(|p| p.file_stem()) {
            Some(stem) => stem.to_string_lossy().into_owned(),
            None if !self.rocket.name.is_empty() => self.rocket.name.clone(),
            None => UNTITLED.to_string(),
        }
    }

    /// Whether there are changes since the last save (or since it was created or opened).
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Counts every change to the rocket, including live drag updates, undo and redo. Anything derived from the rocket
    /// (mesh, physics) is stale when this differs from the value it was computed at.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Whether [`undo`](Self::undo) would do something.
    pub fn can_undo(&self) -> bool {
        self.drag.is_none() && !self.undo.is_empty()
    }

    /// Whether [`redo`](Self::redo) would do something.
    pub fn can_redo(&self) -> bool {
        self.drag.is_none() && !self.redo.is_empty()
    }

    /// The label of the step [`undo`](Self::undo) would revert.
    pub fn undo_label(&self) -> Option<&str> {
        self.undo.last().map(|s| s.label.as_str())
    }

    /// The label of the step [`redo`](Self::redo) would reapply.
    pub fn redo_label(&self) -> Option<&str> {
        self.redo.last().map(|s| s.label.as_str())
    }

    /// Records that the last step moved the viewport's camera `shift` along the axis (see [`Step::shift`]).
    pub fn set_undo_shift(&mut self, shift: f32) {
        if let Some(step) = self.undo.last_mut() {
            step.shift = shift;
        }
    }

    /// The camera shift [`undo`](Self::undo) would revert.
    pub fn undo_shift(&self) -> f32 {
        if self.drag.is_some() {
            return 0.0;
        }
        self.undo.last().map_or(0.0, |s| s.shift)
    }

    /// The camera shift [`redo`](Self::redo) would reapply.
    pub fn redo_shift(&self) -> f32 {
        if self.drag.is_some() {
            return 0.0;
        }
        self.redo.last().map_or(0.0, |s| s.shift)
    }

    fn push_undo(&mut self, step: Step) {
        self.undo.push(step);
        if self.undo.len() > MAX_UNDO {
            self.undo.remove(0);
        }
        self.redo.clear();
    }

    fn changed(&mut self) {
        self.dirty = true;
        self.revision += 1;
    }

    /// After an edit: the formulas (see `bowshock_model::Formula`) are evaluated again so every dependant follows its source, and
    /// the formulas of parts that are gone are dropped (not while a drag carries parts out of the rocket).
    fn settle(&mut self) {
        self.rocket.apply_formulas();
        if self.drag.is_none() {
            self.rocket.prune_formulas();
        }
    }

    /// Applies `f` to the rocket as one undo step called `label` (recorded only if the rocket changed). Ends any
    /// coalescing run first.
    pub fn edit<R>(&mut self, label: &str, f: impl FnOnce(&mut Rocket) -> R) -> R {
        self.seal();
        let before = self.rocket.clone();
        let out = f(&mut self.rocket);
        self.settle();
        if self.rocket != before {
            self.push_undo(Step { rocket: before, label: label.to_string(), shift: 0.0 });
            self.changed();
        }
        out
    }

    /// Like [`edit`](Self::edit), but consecutive calls with the same `key` are one step until [`seal`](Self::seal) (or any
    /// other edit, undo or redo) ends the run.
    pub fn edit_merging<R>(&mut self, label: &str, key: &str, f: impl FnOnce(&mut Rocket) -> R) -> R {
        if self.merging.as_deref() == Some(key) && !self.undo.is_empty() {
            let before = self.rocket.clone();
            let out = f(&mut self.rocket);
            self.settle();
            if self.rocket != before {
                self.changed();
            }
            return out;
        }
        let out = self.edit(label, f);
        if self.undo.last().is_some_and(|s| s.label == label && s.rocket != self.rocket) {
            self.merging = Some(key.to_string());
        }
        out
    }

    /// Ends the coalescing run in progress. If the run ended exactly where it began (a drag put back where it started) its
    /// step is dropped.
    pub fn seal(&mut self) {
        if self.merging.take().is_some() && self.undo.last().is_some_and(|s| s.rocket == self.rocket) {
            self.undo.pop();
        }
    }

    /// Starts a drag that will change the rocket live. Until [`end_drag`](Self::end_drag), changes go through
    /// [`drag_update`](Self::drag_update) and undo/redo are unavailable.
    pub fn begin_drag(&mut self, label: &str) {
        self.seal();
        if self.drag.is_none() {
            self.drag = Some(Step { rocket: self.rocket.clone(), label: label.to_string(), shift: 0.0 });
        }
    }

    /// Gives the drag in progress's undo step another label (what the drag turned out to do).
    pub fn rename_drag(&mut self, label: &str) {
        if let Some(step) = self.drag.as_mut() {
            step.label = label.to_string();
        }
    }

    /// Whether a drag is in progress.
    pub fn dragging(&self) -> bool {
        self.drag.is_some()
    }

    /// Whether an interaction is changing the rocket live: a drag in progress, or a run of merged edits (a handle, a lane bar, a
    /// scrubbed field) that has not been sealed by the release yet.
    pub fn editing_live(&self) -> bool {
        self.drag.is_some() || self.merging.is_some()
    }

    /// The rocket as it was when the drag began.
    pub fn drag_base(&self) -> Option<&Rocket> {
        self.drag.as_ref().map(|s| &s.rocket)
    }

    /// Changes the rocket during a drag without recording a step. Outside a drag this is a plain [`edit`](Self::edit).
    pub fn drag_update<R>(&mut self, f: impl FnOnce(&mut Rocket) -> R) -> R {
        if self.drag.is_none() {
            return self.edit("Edit", f);
        }
        let before = self.rocket.clone();
        let out = f(&mut self.rocket);
        self.rocket.apply_formulas();
        if self.rocket != before {
            self.revision += 1;
        }
        out
    }

    /// Ends the drag. With `commit` the drag is one undo step (if it changed anything); without, the rocket goes back to
    /// exactly what it was. Returns whether the document differs from before the drag.
    pub fn end_drag(&mut self, commit: bool) -> bool {
        let Some(base) = self.drag.take() else {
            return false;
        };
        if commit && self.rocket != base.rocket {
            self.push_undo(base);
            self.changed();
            true
        } else {
            if self.rocket != base.rocket {
                let mut restored = base.rocket;
                restored.carry_cfd_from(&self.rocket);
                self.rocket = restored;
                self.revision += 1;
            }
            false
        }
    }

    /// Reverts the last step; returns its label.
    pub fn undo(&mut self) -> Option<String> {
        if self.drag.is_some() {
            return None;
        }
        self.seal();
        let mut step = self.undo.pop()?;
        step.rocket.carry_cfd_from(&self.rocket);
        let current = std::mem::replace(&mut self.rocket, step.rocket);
        self.redo.push(Step { rocket: current, label: step.label.clone(), shift: step.shift });
        self.changed();
        Some(step.label)
    }

    /// Reapplies the last undone step; returns its label.
    pub fn redo(&mut self) -> Option<String> {
        if self.drag.is_some() {
            return None;
        }
        self.seal();
        let mut step = self.redo.pop()?;
        step.rocket.carry_cfd_from(&self.rocket);
        let current = std::mem::replace(&mut self.rocket, step.rocket);
        self.undo.push(Step { rocket: current, label: step.label.clone(), shift: step.shift });
        self.changed();
        Some(step.label)
    }

    /// Stores a solved CFD table with the document (see `bowshock_model::CfdTable`). Not an edit: no undo step and no revision
    /// bump (nothing derived from the rocket changes), but the file differs from what is on disk, so it is marked unsaved.
    /// Returns whether the table was accepted.
    pub fn store_cfd_table(&mut self, table: bowshock_model::CfdTable) -> bool {
        let stored = self.rocket.store_cfd_table(table);
        self.dirty |= stored;
        stored
    }

    /// Replaces the whole document (New, Open, the example): clears history and the dirty flag.
    pub fn replace(&mut self, rocket: Rocket, path: Option<PathBuf>) {
        self.rocket = rocket;
        self.path = path;
        self.dirty = false;
        self.undo.clear();
        self.redo.clear();
        self.drag = None;
        self.merging = None;
        self.revision += 1;
    }

    /// Starts a new empty document.
    pub fn new_document(&mut self) {
        self.replace(Rocket::default(), None);
    }

    /// Opens the file at `path`, replacing the document. On failure the document is untouched.
    pub fn load(&mut self, path: &Path) -> Result<(), DocError> {
        let opened = Document::open(path)?;
        self.replace(opened.rocket, opened.path);
        Ok(())
    }

    /// Writes the document to its path.
    pub fn save(&mut self) -> Result<(), DocError> {
        let path = self.path.clone().ok_or(DocError::NoPath)?;
        self.save_as(&path)
    }

    /// Writes the document to `path` (given the `.bowshock` extension if it has none) and makes that its path.
    pub fn save_as(&mut self, path: &Path) -> Result<(), DocError> {
        let path = if path.extension().is_none() { path.with_extension(EXTENSION) } else { path.to_path_buf() };
        std::fs::write(&path, self.rocket.to_bytes()).map_err(DocError::Io)?;
        self.path = Some(path);
        self.dirty = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind};

    fn nose() -> PartKind {
        PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1, tip_radius: 0.0 }
    }

    fn tube(length: f64) -> PartKind {
        PartKind::BodyTube { length, diameter: 0.1 }
    }

    #[test]
    fn edits_undo_and_redo_one_step_each() {
        let mut doc = Document::default();
        assert!(!doc.is_dirty() && !doc.can_undo());
        doc.edit("Add nose", |r| r.insert(0, nose()).unwrap());
        let tube_id = doc.edit("Add tube", |r| r.insert(1, tube(0.5)).unwrap());
        assert!(doc.is_dirty());
        assert_eq!(doc.rocket().parts().len(), 2);
        assert_eq!(doc.undo().as_deref(), Some("Add tube"));
        assert_eq!(doc.rocket().parts().len(), 1);
        assert_eq!(doc.redo().as_deref(), Some("Add tube"));
        assert!(doc.rocket().part(tube_id).is_some());
        // A new edit clears redo.
        doc.undo();
        doc.edit("Add other", |r| r.insert(1, tube(0.3)).unwrap());
        assert!(!doc.can_redo());
        // An edit that changes nothing records nothing.
        let steps = doc.undo.len();
        doc.edit("Nothing", |_| ());
        assert_eq!(doc.undo.len(), steps);
    }

    #[test]
    fn a_stored_cfd_table_is_not_an_edit_and_survives_undo_and_the_file() {
        use bowshock_model::{CfdPoint, CfdQuality, CfdTable};
        let mut doc = Document::default();
        doc.edit("Add nose", |r| r.insert(0, nose()).unwrap());
        doc.edit("Add tube", |r| r.insert(1, tube(0.5)).unwrap());
        let point = |mach: f64| CfdPoint {
            mach,
            cd: 0.4,
            friction: 0.1,
            pressure: 0.1,
            base: 0.1,
            fins: 0.1,
            other: 0.0,
            interference: 0.0,
            quality: CfdQuality::Converged,
        };
        let table = CfdTable {
            key: 1,
            geometry: 2,
            stage: 1,
            solved_at: 1_700_000_000,
            solve_seconds: 30.0,
            backend: "GPU (Luma)".into(),
            fallback: None,
            altitude: 120.0,
            notes: Vec::new(),
            points: vec![point(0.5), point(1.0)],
            grid: Default::default(),
            lift: None,
        };
        let (revision, steps) = (doc.revision(), doc.undo.len());
        assert!(doc.store_cfd_table(table.clone()));
        assert_eq!((doc.revision(), doc.undo.len()), (revision, steps), "a table is a cache: no revision bump, no undo step");
        // Undo and redo swap whole rockets; the table stays.
        doc.undo();
        assert_eq!(doc.rocket().parts().len(), 1);
        assert_eq!(doc.rocket().cfd_table(1), Some(&table));
        doc.redo();
        assert_eq!(doc.rocket().cfd_table(1), Some(&table));
        // And it is written with the document.
        let back = Rocket::from_bytes(&doc.rocket().to_bytes()).unwrap();
        assert_eq!(back.cfd_table(1), Some(&table));
        // A document with nothing to save is dirty once a table arrives.
        doc.replace(Rocket::default(), None);
        assert!(!doc.is_dirty());
        assert!(doc.store_cfd_table(table));
        assert!(doc.is_dirty());
    }

    #[test]
    fn merged_edits_are_one_step_and_a_round_trip_is_none() {
        let mut doc = Document::default();
        doc.edit("Add", |r| r.insert(0, nose()).unwrap());
        let id = doc.rocket().parts()[0].id;
        let revision = doc.revision();
        for length in [0.21, 0.22, 0.25, 0.3] {
            let kind = PartKind::NoseCone { shape: NoseShape::TangentOgive, length, diameter: 0.1, tip_radius: 0.0 };
            doc.edit_merging("Resize", "resize", |r| r.set_kind(id, kind));
        }
        assert!(doc.revision() > revision);
        doc.seal();
        assert_eq!(doc.undo.len(), 2);
        assert_eq!(doc.undo().as_deref(), Some("Resize"));
        assert_eq!(doc.rocket().part(id).unwrap().kind.length(), 0.2);

        // A drag that ends where it started leaves no step.
        let steps = doc.undo.len();
        for length in [0.3, 0.2] {
            let kind = PartKind::NoseCone { shape: NoseShape::TangentOgive, length, diameter: 0.1, tip_radius: 0.0 };
            doc.edit_merging("Resize", "resize", |r| r.set_kind(id, kind));
        }
        doc.seal();
        assert_eq!(doc.undo.len(), steps);
    }

    #[test]
    fn a_drag_is_one_step_and_cancelling_restores_everything() {
        let mut doc = Document::default();
        doc.edit("Add", |r| {
            r.insert(0, nose()).unwrap();
            let t = r.insert(1, tube(0.5)).unwrap();
            r.set_name(t, Some("Payload".to_string()));
            r.set_thickness(t, 0.004);
        });
        let tube_id = doc.rocket().parts()[1].id;
        let before = doc.rocket().clone();

        doc.begin_drag("Move part");
        assert!(doc.dragging() && !doc.can_undo());
        doc.drag_update(|r| r.remove(tube_id));
        assert!(doc.rocket().part(tube_id).is_none());
        assert!(!doc.end_drag(false));
        assert_eq!(doc.rocket(), &before);
        // The name and wall survived because the whole rocket came back.
        assert_eq!(doc.rocket().part(tube_id).unwrap().name.as_deref(), Some("Payload"));

        doc.begin_drag("Move part");
        doc.drag_update(|r| r.remove(tube_id));
        assert!(doc.end_drag(true));
        assert_eq!(doc.undo().as_deref(), Some("Move part"));
        assert_eq!(doc.rocket(), &before);
    }

    #[test]
    fn save_and_load_round_trip() {
        let dir = std::env::temp_dir().join(format!("bowshock-document-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut doc = Document::default();
        assert!(matches!(doc.save(), Err(DocError::NoPath)));
        doc.edit("Add", |r| {
            r.insert(0, nose()).unwrap();
            r.insert(1, tube(0.5)).unwrap();
        });
        assert!(doc.is_dirty());
        // The extension is added when missing.
        doc.save_as(&dir.join("rocket")).unwrap();
        assert!(!doc.is_dirty());
        assert_eq!(doc.path().unwrap(), dir.join("rocket.bowshock"));
        assert_eq!(doc.title(), "rocket");

        let mut other = Document::default();
        other.load(&dir.join("rocket.bowshock")).unwrap();
        assert_eq!(other.rocket(), doc.rocket());
        assert!(!other.is_dirty() && !other.can_undo());

        // Edit, save again in place; a bad file leaves the open document alone.
        other.edit("Longer", |r| {
            let id = r.parts()[1].id;
            r.set_kind(id, tube(0.6));
        });
        assert!(other.is_dirty());
        other.save().unwrap();
        assert!(!other.is_dirty());
        // Plain text (including the old JSON format) is refused with the plain message.
        std::fs::write(dir.join("bad.bowshock"), "{\"format_version\":2}").unwrap();
        let refused = other.load(&dir.join("bad.bowshock"));
        assert!(matches!(refused, Err(DocError::Format(FormatError::NotBowshock))));
        assert_eq!(refused.unwrap_err().to_string(), "Not a Bowshock document");
        assert_eq!(other.rocket().parts()[1].kind.length(), 0.6);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
