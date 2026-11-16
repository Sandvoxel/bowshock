//! The Simulate screen (design/components/SimulateScreen, SimulateCompare, GroundTrack, PlotZoom): flights list and conditions on
//! the left, the events strip and linked plots in the centre, results and the ground track on the right.
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`], [`State::on_key`], [`State::tick`] and [`State::animating`] with the shared [`Ctx`].
//!
//! * **Flights** ([`flight`]) are stored in the document (`Rocket::flights`: name, motor choice, launch conditions), so they are
//!   saved in the `.bowshock` file, loaded back, edited through `ctx.doc` (dirty dot, undo) and a new document has one, `Nominal`.
//!   Results are never saved: they are re-run. A flight is flown against the rocket *as it is when you press Run*. [`State::sync`]
//!   (called by the app after every message) mirrors the document's list; [`State::commit`] writes edits back.
//! * **Run** ([`Message::Run`], the top bar's Simulate button and `F5`) flies the selected flights with
//!   [`bowshock_sim::from_model::sim_input`] on a worker thread and shows the result when it lands; **Run all** flies every flight.
//!   A result whose conditions or document have changed since is *stale* and is drawn in `ink-faint` until it is run again.
//! * **Edit selection**: a click selects one flight, Cmd/Ctrl-click toggles one, Shift-click selects a range (`Pick`); Cmd/Ctrl+A
//!   selects all. With several selected the Conditions panel edits all of them at once: fields they agree on show the value, the
//!   others the mixed `—`, and an edit sets that field on every selected flight as one undo step. The compare ticks are separate.
//! * **Compare**: ticking two or more flights (up to four, colours `series-1…4` that stay with the flight) overlays them on every
//!   plot and turns the results into a comparison against the reference (the selected flight when ticked).
//! * The plots are [`widgets::plot`](crate::widgets::plot) groups with PlotZoom (drag a band, `Enter` zooms, `Esc` steps back); the
//!   ground track shares the plots' crosshair; every result number moves it to the moment it happened.
//! * **Plot sources** ([`channels`], [`layout`], [`picker`]): every quantity a flight records is a data source (position, velocity,
//!   acceleration, aerodynamics, forces, mass and balance, atmosphere, recovery). Each panel's title is a button that opens a
//!   searchable menu of them; `+ series` overlays another source *of the same unit* (one y-axis per plot, never two: the others are
//!   listed disabled, with the reason); panels add, remove and move; the presets row (Flight profile, Aerodynamics, Forces,
//!   Stability, Roll, Custom) sets whole layouts. With several flights overlaid the colour is the flight and the line style is the source.
//!   The layout is stored in the document (`Rocket::plot_layout`), so it saves, loads and undoes; [`State::edit_layout`] writes it.

mod channels;
pub(crate) mod flight;
mod forecast;
pub(crate) mod format;
mod keys;
mod layout;
mod left;
mod picker;
mod plots;
mod results;
pub(crate) mod sites;

use std::sync::Arc;
use std::time::{Duration, Instant};

use bowshock_motors::MotorDb;
use bowshock_sim::{FlightResult, Turbulence};
use iced::widget::{column, container, row, stack, text, Space};
use iced::{keyboard, Element, Fill, Point, Rectangle, Task, Widget};

use crate::app::{self, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{BIN_W, RAIL_W, RIGHT_PANEL_W, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::units::UnitSystem;
use crate::widgets::button::icon_button;
use crate::widgets::chip::{chip, Severity};
use crate::widgets::glyph::Mark;
use crate::widgets::ground_track::{GroundMessage, GroundTrack};
use crate::widgets::plot::{header_parts, legend, PanelAction, PlotGroup, PlotMessage, PlotState, PlotView, SeriesColor};
use crate::widgets::segmented::{segmented_dense, Dense};

use bowshock_model::{FlightDef, PlotLayout, PlotX, Rocket, StabilityBands};
use flight::{Dispersed, Flight, FlightId, MotorChoice, Run, Spec, Times};
use layout::Preset;
use picker::{Mode, Picker};
use plots::Entry;

/// Most flights that can be overlaid.
pub const MAX_COMPARED: usize = 4;

/// An edit of the selected flight's conditions.
#[derive(Debug, Clone, PartialEq)]
pub enum Edit {
    /// The motor.
    Motor(MotorChoice),
    /// Ejection delay (s).
    Delay(f64),
    /// Rail length (m).
    RailLength(f64),
    /// Rail tilt from vertical (rad).
    LaunchAngle(f64),
    /// Mean wind speed (m/s).
    Wind(f64),
    /// Direction the wind blows from (rad).
    WindFrom(f64),
    /// Gust intensity.
    Turbulence(Turbulence),
    /// Where the drag comes from.
    Drag(bowshock_model::DragSource),
    /// Site altitude (m).
    SiteAltitude(f64),
    /// Ground temperature (K).
    Temperature(f64),
    /// Field radius (m).
    FieldRadius(f64),
    /// The wind profile above the ground wind.
    WindLevels(Vec<bowshock_model::WindLevel>),
    /// The launch latitude (degrees); `None` clears the location. A flight without a location gets longitude 0.
    Latitude(Option<f64>),
    /// The launch longitude (degrees); `None` clears the location. A flight without a location gets latitude 0.
    Longitude(Option<f64>),
    /// A start in the air, `None` for a pad launch.
    AirStart(Option<bowshock_model::AirStart>),
    /// Built-in launch site `i` of [`sites::SITES`]: its altitude and location, and the standard temperature there when the
    /// temperature is still the standard sea-level one.
    Site(usize),
    /// A forecast wind: the ground wind, the levels above it and, when asked for, the forecast temperature (K).
    Forecast { speed: f64, from: f64, levels: Vec<bowshock_model::WindLevel>, temperature: Option<f64> },
}

impl Edit {
    /// What the edit changes, in the words of the undo step (`Set rail length on 3 flights`).
    fn label(&self) -> &'static str {
        match self {
            Edit::Motor(_) => "motor",
            Edit::Delay(_) => "delay",
            Edit::RailLength(_) => "rail length",
            Edit::LaunchAngle(_) => "launch angle",
            Edit::Wind(_) => "wind",
            Edit::WindFrom(_) => "wind from",
            Edit::Turbulence(_) => "turbulence",
            Edit::Drag(_) => "drag source",
            Edit::SiteAltitude(_) => "site altitude",
            Edit::Temperature(_) => "temperature",
            Edit::FieldRadius(_) => "field radius",
            Edit::WindLevels(_) => "wind profile",
            Edit::Latitude(_) | Edit::Longitude(_) => "launch location",
            Edit::AirStart(_) => "air start",
            Edit::Site(_) => "launch site",
            Edit::Forecast { .. } => "forecast wind",
        }
    }

    /// Whether a single flight's undo step names the edit (`Set wind profile`) instead of the generic `Edit flight`.
    fn named(&self) -> bool {
        matches!(self, Edit::WindLevels(_) | Edit::Latitude(_) | Edit::Longitude(_) | Edit::AirStart(_) | Edit::Site(_) | Edit::Forecast { .. })
    }

    /// The undo step of a single flight's edit that [`Edit::named`] names.
    fn title(&self) -> String {
        match self {
            Edit::Forecast { .. } => "Wind from forecast".to_string(),
            other => format!("Set {}", other.label()),
        }
    }
}

/// How a click on a flight row changes the edit selection (the kit's multi-select conventions, as in the viewport and Station
/// Lanes): a plain click selects one, Cmd/Ctrl-click toggles, Shift-click selects the range from the last click, and Shift with
/// Cmd/Ctrl adds that range to what is selected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pick {
    Only,
    Toggle,
    Range,
    AddRange,
}

impl Pick {
    /// What a click does with `modifiers` held.
    fn of(modifiers: keyboard::Modifiers) -> Pick {
        match (modifiers.shift(), modifiers.command() || modifiers.control()) {
            (true, true) => Pick::AddRange,
            (true, false) => Pick::Range,
            (false, true) => Pick::Toggle,
            (false, false) => Pick::Only,
        }
    }
}

/// An edit of the plots' layout: the presets row, the panel chrome and the source picker all send these.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LayoutEdit {
    /// Replace the layout with a preset (`Custom` starts a one-panel layout to build on).
    Preset(Preset),
    /// Add a panel at the bottom.
    AddPanel,
    /// Remove a panel.
    RemovePanel(usize),
    /// Move a panel up (`true`) or down.
    Move(usize, bool),
    /// Make the source (by key) the panel's primary one.
    SetPrimary(usize, &'static str),
    /// Add the source to the panel, or remove it from there.
    ToggleSource(usize, &'static str),
    /// Plot every panel against this (time, altitude, Mach, ground distance).
    X(PlotX),
}

/// A finished simulation, as the worker thread reports it.
#[derive(Debug, Clone)]
pub struct Done {
    flight: FlightId,
    ticket: u64,
    spec: Spec,
    revision: u64,
    delay: f64,
    dt: f64,
    outcome: Result<(Arc<FlightResult>, Duration), String>,
}

/// A finished dispersion study, as the worker thread reports it.
#[derive(Debug, Clone)]
pub struct DispersionDone {
    flight: FlightId,
    ticket: u64,
    spec: Spec,
    revision: u64,
    outcome: Result<Vec<(f64, f64)>, String>,
}

/// The Simulate screen's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// Run the flights selected for editing (top bar button, `F5`).
    Run,
    /// Run every flight.
    RunAll,
    /// A simulation finished on its worker thread.
    Done(Box<Done>),
    /// A click on a flight row: selects it (the reference of a comparison), toggles it or extends the selection to it, by the
    /// modifiers held (see [`Pick`]).
    Click(FlightId),
    /// The held modifier keys changed (reported by the invisible tracker in the Flights list).
    Modifiers(keyboard::Modifiers),
    /// Select every flight (Cmd/Ctrl+A).
    SelectAll,
    /// Tick or untick this flight for comparison.
    Tick(FlightId),
    /// `compare +`: add a flight with the selected one's conditions.
    AddFlight,
    /// Remove the selected flight.
    RemoveFlight,
    /// One flight per motor configuration of the document, ticked and run.
    PerConfiguration,
    /// Fly the selected flight many times with its conditions varied and show where the landings scatter.
    Disperse,
    /// A dispersion study finished on its worker thread.
    Dispersed(Box<DispersionDone>),
    /// Edit the selected flight's conditions.
    Edit(Edit),
    /// The linked plots: hover, band, zoom.
    Plot(PlotMessage),
    /// The ground track: hover, click on a landing.
    Ground(GroundMessage),
    /// A result number was clicked: move the crosshair to that moment.
    Seek(f64),
    /// Double-click on a flight's name: edit it in place.
    StartRename(FlightId),
    /// The rename field's text.
    RenameInput(String),
    /// Enter in the rename field: the name goes through the document (one undo step).
    CommitRename,
    /// Escape in the rename field.
    CancelRename,
    /// An edit of the plots' layout.
    Layout(LayoutEdit),
    /// The source picker's search text.
    PickerQuery(String),
    /// Close the source picker (a click outside it).
    ClosePicker,
    /// A click inside the source picker that no row took: it stays open.
    KeepPicker,
    /// Show or hide the wind profile table.
    WindTable(bool),
    /// Text typed in the latitude (`false`) or longitude (`true`) field.
    CoordInput(bool, String),
    /// Enter in a coordinate field: a number sets it, a blank field clears the location.
    CoordCommit,
    /// A control that needs no action (the `Custom` launch site).
    Ignore,
    /// Open (`true`) or close the forecast row of the wind profile.
    ForecastOpen(bool),
    /// The launch time typed in the forecast row (`YYYY-MM-DDTHH:00`, the site's local time).
    ForecastTime(String),
    /// Whether the forecast also sets the temperature.
    ForecastTemperature(bool),
    /// Fetch the forecast for the selected flight's location and the typed time.
    ForecastFetch,
    /// The forecast arrived (or why it did not).
    ForecastDone(Result<forecast::Forecast, String>),
}

/// What the plots and the ground track show: derived from the flights whenever a run lands or the ticks change.
#[derive(Debug, Default)]
struct Shown {
    /// The flights on show, in tick (slot) order; the tracks of `ground` are in the same order.
    ids: Vec<FlightId>,
    /// Index of the reference flight in `ids`.
    reference: usize,
    /// Two or more flights ticked.
    compare: bool,
    /// The plots in the unit set `State::units`, `None` while nothing on show has run.
    group: Option<PlotGroup>,
    /// The ground tracks (the selected flight's wind and field when nothing has run).
    ground: GroundTrack,
}

/// The Simulate screen's state.
///
/// The flight list is stored in the document (`Rocket::flights`), so it saves with the file, loads back and undoes. This state
/// mirrors it (and adds the never-saved runtime: results, compare ticks): every change to the list is written to the document by
/// [`State::commit`], and [`State::sync`] (the app calls it after every message) adopts what the document holds when it changed
/// from outside (open, New, undo, redo).
#[derive(Debug)]
pub struct State {
    flights: Vec<Flight>,
    selected: FlightId,
    /// The flights selected with `selected` for editing (Cmd/Ctrl-click, Shift-click): the edit selection is `selected` plus
    /// these. The compare ticks are separate.
    also: Vec<FlightId>,
    /// Where a Shift-click range starts: the last flight clicked.
    anchor: FlightId,
    /// The modifier keys held, as the tracker last reported them (a drive script holds `BOWSHOCK_DRIVE_MODIFIERS`).
    modifiers: keyboard::Modifiers,
    next_id: FlightId,
    plot: PlotState,
    shown: Shown,
    /// The flight list as the document stores it, as of the last time this state and the document agreed.
    stored: Vec<FlightDef>,
    /// Which sources each plot draws: the document's layout, repaired for drawing (unknown sources dropped).
    layout: PlotLayout,
    /// The layout as the document stores it, as of the last time this state and the document agreed.
    stored_layout: PlotLayout,
    /// The source picker, while it is open.
    picker: Option<Picker>,
    /// The document's stability thresholds: the stability plot's `Min` line.
    bands: StabilityBands,
    /// How many document revisions this screen's own flight edits made. Results are stale when the *rocket* changed, not when a
    /// flight was edited (that only stales the flight itself), so [`State::revision`] discounts these.
    own_edits: u64,
    /// The flight being renamed and the text typed so far.
    renaming: Option<(FlightId, String)>,
    /// The unit set the plots were built in: the zoom's full x range follows it (lengths along x differ by unit).
    units: UnitSystem,
    /// Whether the wind profile table is open.
    wind_open: bool,
    /// The text of the latitude and longitude fields: the flights' location, or what is being typed in
    /// [`State::coord_editing`]'s field.
    coord_text: [String; 2],
    /// The coordinate field being typed in (`true` for longitude) and the flight selected when typing began.
    coord_editing: Option<(bool, FlightId)>,
    /// The forecast row of the wind profile, while open.
    forecast: Option<ForecastRow>,
}

impl Default for State {
    fn default() -> Self {
        State::new()
    }
}

impl State {
    /// The flights of a new document (`Nominal`), selected, nothing run yet.
    pub fn new() -> Self {
        let new = Rocket::default();
        State::of(new.flights(), new.plot_layout(), new.stability_bands())
    }

    /// A state holding `defs` (never empty) and the plots `layout`, the first flight selected, nothing run yet.
    fn of(defs: &[FlightDef], layout: &PlotLayout, bands: StabilityBands) -> Self {
        let flights: Vec<Flight> = defs.iter().map(Flight::from_def).collect();
        let mut state = State {
            selected: flights[0].id,
            also: Vec::new(),
            anchor: flights[0].id,
            modifiers: crate::viewport::manipulate::drive_modifiers(),
            next_id: defs.iter().map(|d| d.id).max().map_or(1, |m| m + 1),
            flights,
            plot: PlotState::new((0.0, 1.0)),
            shown: Shown::default(),
            stored: defs.to_vec(),
            layout: repaired(layout),
            stored_layout: layout.clone(),
            picker: None,
            bands,
            own_edits: 0,
            renaming: None,
            units: UnitSystem::default(),
            wind_open: false,
            coord_text: [String::new(), String::new()],
            coord_editing: None,
            forecast: None,
        };
        state.rebuild();
        state.refresh_coords();
        state
    }

    /// Brings the screen in step with the document after a message: a document that was just opened or created replaces
    /// everything (`reloaded`); otherwise a flight list that changed under us (undo, redo) is adopted, keeping the results of
    /// the flights that are still there.
    pub fn sync(&mut self, ctx: &Ctx, reloaded: bool) {
        let rocket = ctx.doc.rocket();
        let (defs, layout, bands) = (rocket.flights(), rocket.plot_layout(), rocket.stability_bands());
        if reloaded {
            let modifiers = self.modifiers;
            *self = State::of(defs, layout, bands);
            self.modifiers = modifiers;
            self.units = ctx.units;
            self.rebuild();
            self.refresh_coords();
            return;
        }
        if self.units != ctx.units {
            self.units = ctx.units;
            self.rebuild();
        }
        if self.bands != bands {
            self.bands = bands;
            self.rebuild();
        }
        if self.stored != defs {
            self.adopt(defs);
        }
        if self.stored_layout != *layout {
            self.adopt_layout(layout);
        }
        self.refresh_coords();
    }

    /// Shows the edit selection's launch location in the coordinate fields (blank when they have none or differ), unless one
    /// is being typed in for the same selected flight.
    fn refresh_coords(&mut self) {
        if self.coord_editing.is_some_and(|(_, id)| id == self.selected) {
            return;
        }
        self.coord_editing = None;
        let locations: Vec<_> = self.edit_flights().iter().map(|f| f.spec.location).collect();
        let agreed = locations.first().copied().filter(|first| locations.iter().all(|l| l == first)).flatten();
        self.coord_text = match agreed {
            Some(at) => [format!("{:.5}", at.latitude), format!("{:.5}", at.longitude)],
            None => [String::new(), String::new()],
        };
    }

    /// Takes `layout` as the plots' layout (undo, redo): the picker, whose panel may be gone, closes.
    fn adopt_layout(&mut self, layout: &PlotLayout) {
        self.stored_layout = layout.clone();
        self.layout = repaired(layout);
        self.picker = None;
        self.rebuild();
    }

    /// Applies `edit` to the plots' layout as one undo step. A refused edit says why in the status strip and changes nothing.
    fn edit_layout(&mut self, edit: LayoutEdit, ctx: &mut Ctx) {
        let mut next = self.layout.clone();
        let (label, outcome): (&str, Result<(), String>) = match edit {
            LayoutEdit::Preset(Preset::Custom) if Preset::of(&next) == Preset::Custom => ("Set plots", Ok(())),
            LayoutEdit::Preset(preset) => {
                next = PlotLayout { x: next.x, ..preset.layout() };
                ("Set plots", Ok(()))
            }
            LayoutEdit::X(x) => {
                next.x = x;
                ("Plot against", Ok(()))
            }
            LayoutEdit::AddPanel => ("Add plot", if layout::add_panel(&mut next) { Ok(()) } else { Err("The plots hold at most 8 panels".to_string()) }),
            LayoutEdit::RemovePanel(i) => {
                ("Remove plot", if layout::remove_panel(&mut next, i) { Ok(()) } else { Err("The last plot cannot be removed".to_string()) })
            }
            LayoutEdit::Move(i, up) => {
                ("Move plot", if layout::move_panel(&mut next, i, up) { Ok(()) } else { Err("The plot is already at the end".to_string()) })
            }
            LayoutEdit::SetPrimary(i, key) => {
                layout::set_primary(&mut next, i, key);
                ("Change plot source", Ok(()))
            }
            LayoutEdit::ToggleSource(i, key) => ("Edit plot sources", layout::toggle_source(&mut next, i, key, ctx.units)),
        };
        self.picker = None;
        if let Err(reason) = outcome {
            ctx.notify(reason);
            return;
        }
        layout::repair(&mut next);
        if next == self.layout {
            return;
        }
        let before = ctx.doc.revision();
        ctx.doc.edit(label, |rocket| rocket.set_plot_layout(next));
        self.own_edits += ctx.doc.revision() - before;
        let kept = ctx.doc.rocket().plot_layout().clone();
        self.adopt_layout(&kept);
    }

    /// A click on an editable panel's chrome: the title and `+ series` open the source picker at the chip, the rest edit the layout.
    fn panel_action(&mut self, panel: usize, action: PanelAction, chip: Rectangle, ctx: &mut Ctx) -> Task<app::Message> {
        let mode = match action {
            PanelAction::Pick => Mode::Primary,
            PanelAction::AddSeries => Mode::Add,
            PanelAction::MoveUp => {
                self.edit_layout(LayoutEdit::Move(panel, true), ctx);
                return Task::none();
            }
            PanelAction::MoveDown => {
                self.edit_layout(LayoutEdit::Move(panel, false), ctx);
                return Task::none();
            }
            PanelAction::Remove => {
                self.edit_layout(LayoutEdit::RemovePanel(panel), ctx);
                return Task::none();
            }
        };
        self.picker = Some(Picker { panel, mode, at: Point::new(chip.x, chip.y + chip.height + 6.0), query: String::new() });
        iced::widget::operation::focus(picker::SEARCH_ID)
    }

    /// Closes the source picker if it is open (Esc). Whether there was one to close.
    pub fn dismiss_picker(&mut self) -> bool {
        self.picker.take().is_some()
    }

    /// Takes `defs` as the flight list. A flight with the same id keeps its results, compare slot and tickets.
    fn adopt(&mut self, defs: &[FlightDef]) {
        let mut old = std::mem::take(&mut self.flights);
        self.flights = defs
            .iter()
            .map(|d| match old.iter().position(|f| f.id == d.id) {
                Some(i) => {
                    let mut f = old.swap_remove(i);
                    f.name = d.name.clone();
                    f.spec = Spec::of(d);
                    f
                }
                None => Flight::from_def(d),
            })
            .collect();
        self.next_id = self.next_id.max(defs.iter().map(|d| d.id).max().map_or(1, |m| m + 1));
        self.also.retain(|id| self.flights.iter().any(|f| f.id == *id));
        if self.flight(self.selected).is_none() {
            let first = self.flights[0].id;
            self.select_only(first);
        } else if self.flight(self.anchor).is_none() {
            self.anchor = self.selected;
        }
        self.stored = defs.to_vec();
        self.rebuild();
    }

    /// Writes the flight list to the document as one undo step (`key` merges a run of edits, e.g. scrubbing one field, into one),
    /// then adopts what the document kept (it repairs out-of-range values). Does nothing when nothing changed.
    fn commit(&mut self, ctx: &mut Ctx, label: &str, key: Option<&str>) {
        let defs: Vec<FlightDef> = self.flights.iter().map(Flight::def).collect();
        if defs == self.stored {
            return;
        }
        let before = ctx.doc.revision();
        let write = |rocket: &mut Rocket| rocket.set_flights(defs);
        match key {
            Some(key) => ctx.doc.edit_merging(label, key, write),
            None => ctx.doc.edit(label, write),
        }
        self.own_edits += ctx.doc.revision() - before;
        let kept = ctx.doc.rocket().flights().to_vec();
        self.adopt(&kept);
    }

    /// The revision of the rocket a result is compared with: the document's, leaving out this screen's own flight edits.
    fn revision(&self, ctx: &Ctx) -> u64 {
        ctx.doc.revision().saturating_sub(self.own_edits)
    }

    /// What a result of a flight with `spec` is compared with: the [`State::revision`], and for a flight on the CFD table the keys of
    /// the tables it flies on (storing a table is not a document revision), so a new solve makes that flight stale.
    fn stamp(&self, ctx: &Ctx, spec: &Spec) -> u64 {
        let base = self.revision(ctx);
        if spec.drag != bowshock_model::DragSource::CfdTable {
            return base;
        }
        let rocket = ctx.doc.rocket();
        let keys = (1..=rocket.stages().len().max(1)).fold(0xcbf2_9ce4_8422_2325u64, |h, stage| {
            let key = rocket.cfd_table(stage as u32).map_or(0, |t| t.key);
            (h ^ key).wrapping_mul(0x0000_0100_0000_01b3)
        });
        base.wrapping_add(keys | 1)
    }

    fn flight(&self, id: FlightId) -> Option<&Flight> {
        self.flights.iter().find(|f| f.id == id)
    }

    /// The id of the selected flight (the reference of a comparison, and the one the Conditions panel shows first).
    pub fn selected_id(&self) -> FlightId {
        self.selected_flight().id
    }

    fn selected_flight(&self) -> &Flight {
        self.flight(self.selected).unwrap_or(&self.flights[0])
    }

    /// The flights the Conditions panel edits: the selected one and those selected with it, in list order.
    fn edit_ids(&self) -> Vec<FlightId> {
        let ids: Vec<FlightId> = self.flights.iter().filter(|f| f.id == self.selected || self.also.contains(&f.id)).map(|f| f.id).collect();
        if ids.is_empty() {
            vec![self.selected_flight().id]
        } else {
            ids
        }
    }

    /// [`Self::edit_ids`] as flights.
    fn edit_flights(&self) -> Vec<&Flight> {
        self.edit_ids().into_iter().filter_map(|id| self.flight(id)).collect()
    }

    /// Whether `id` is in the edit selection.
    fn is_selected(&self, id: FlightId) -> bool {
        id == self.selected || self.also.contains(&id)
    }

    /// Makes `id` the only selected flight.
    fn select_only(&mut self, id: FlightId) {
        self.selected = id;
        self.also.clear();
        self.anchor = id;
    }

    /// Changes the edit selection by a click on `id`. A toggle never empties the selection; a range runs from the last flight
    /// clicked to `id`, in list order, and `id` becomes the selected (reference) flight.
    fn pick(&mut self, id: FlightId, how: Pick) {
        if self.flight(id).is_none() {
            return;
        }
        let mut set = self.edit_ids();
        match how {
            Pick::Only => {
                self.select_only(id);
                return;
            }
            Pick::Toggle => match set.iter().position(|f| *f == id) {
                Some(at) if set.len() > 1 => {
                    set.remove(at);
                    if self.selected == id {
                        self.selected = set[0];
                    }
                    self.anchor = self.selected;
                }
                Some(_) => {}
                None => {
                    set.push(id);
                    self.selected = id;
                    self.anchor = id;
                }
            },
            Pick::Range | Pick::AddRange => {
                let at = |id| self.flights.iter().position(|f| f.id == id);
                let (Some(from), Some(to)) = (at(self.anchor).or_else(|| at(self.selected)), at(id)) else { return };
                if how == Pick::Range {
                    set.clear();
                }
                for f in &self.flights[from.min(to)..=from.max(to)] {
                    if !set.contains(&f.id) {
                        set.push(f.id);
                    }
                }
                self.selected = id;
            }
        }
        self.also = set.into_iter().filter(|f| *f != self.selected).collect();
    }

    /// Flights ticked for comparison.
    fn ticked(&self) -> usize {
        self.flights.iter().filter(|f| f.slot.is_some()).count()
    }

    /// Recomputes what the plots and ground track show.
    fn rebuild(&mut self) {
        let compare = self.ticked() >= 2;
        let mut order: Vec<&Flight> = if compare {
            let mut ticked: Vec<&Flight> = self.flights.iter().filter(|f| f.slot.is_some() && f.run.is_some()).collect();
            ticked.sort_by_key(|f| f.slot);
            ticked
        } else {
            self.flights.iter().filter(|f| f.id == self.selected && f.run.is_some()).collect()
        };
        order.truncate(MAX_COMPARED);
        let entries: Vec<Entry> = order
            .iter()
            .filter_map(|f| {
                let run = f.run.as_ref()?;
                let color = if compare { SeriesColor::Series(f.slot.unwrap_or(1)) } else { SeriesColor::Ink };
                Some(Entry { name: &f.name, color, run, scatter: f.dispersion_of_run().map(|d| d.landings.as_slice()) })
            })
            .collect();
        let ids: Vec<FlightId> = order.iter().map(|f| f.id).collect();
        // The reference is the selected flight when it is on show, else the first.
        let reference = ids.iter().position(|id| *id == self.selected).unwrap_or(0);

        let group = plots::group(&entries, reference, self.units, &self.layout, self.bands.stable);
        let ground = if entries.is_empty() {
            GroundTrack {
                tracks: Vec::new(),
                wind: Some(plots::wind(&self.selected_flight().spec)),
                field_radius: Some(self.selected_flight().spec.field_radius),
                units: self.units,
            }
        } else {
            plots::ground(&entries, reference, self.units)
        };
        if let Some(g) = group.as_ref() {
            self.plot.set_full(g.x_extent());
        }
        // `entries` borrows the flights; end it before assigning.
        drop(entries);
        self.shown = Shown { ids, reference, compare, group, ground };
    }

    /// Starts the simulation of flight `id` on a worker thread.
    fn launch(&mut self, id: FlightId, ctx: &Ctx) -> Task<app::Message> {
        let Some(spec) = self.flight(id).map(|f| f.spec.clone()) else { return Task::none() };
        let revision = self.stamp(ctx, &spec);
        let Some(flight) = self.flights.iter_mut().find(|f| f.id == id) else { return Task::none() };
        let (input, delay) = match flight::prepare(ctx, &spec) {
            Ok(prepared) => prepared,
            Err(message) => {
                flight.error = Some(message);
                flight.running = false;
                return Task::none();
            }
        };
        flight.ticket += 1;
        flight.running = true;
        flight.error = None;
        let (ticket, dt) = (flight.ticket, input.dt);
        Task::perform(compute(input), move |outcome| {
            app::Message::Simulate(Message::Done(Box::new(Done { flight: id, ticket, spec: spec.clone(), revision, delay, dt, outcome })))
        })
    }

    fn launch_all(&mut self, ids: Vec<FlightId>, ctx: &Ctx) -> Task<app::Message> {
        Task::batch(ids.into_iter().map(|id| self.launch(id, ctx)))
    }

    /// Handles one of the screen's messages.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Run => {
                let ids = self.edit_ids();
                let task = self.launch_all(ids.clone(), ctx);
                let failed = ids.iter().filter_map(|id| self.flight(*id)).find_map(|f| f.error.clone().map(|e| (f.name.clone(), e)));
                match failed {
                    Some((_, error)) if ids.len() == 1 => ctx.notify(error),
                    Some((name, error)) => ctx.notify(format!("{name}: {error}")),
                    None => {}
                }
                self.rebuild();
                return task;
            }
            Message::RunAll => {
                let ids = self.flights.iter().map(|f| f.id).collect();
                let task = self.launch_all(ids, ctx);
                if let Some(f) = self.flights.iter().find(|f| f.error.is_some()) {
                    ctx.notify(format!("{}: {}", f.name, f.error.clone().unwrap_or_default()));
                }
                self.rebuild();
                return task;
            }
            Message::Done(done) => self.finished(*done),
            Message::Click(id) => {
                self.pick(id, Pick::of(self.modifiers));
                self.rebuild();
            }
            Message::Modifiers(modifiers) => self.modifiers = modifiers,
            Message::SelectAll => {
                self.also = self.flights.iter().map(|f| f.id).filter(|id| *id != self.selected).collect();
                self.anchor = self.selected;
            }
            Message::Tick(id) => self.tick_flight(id, ctx),
            Message::AddFlight => {
                let spec = self.selected_flight().spec.clone();
                let id = self.next_id;
                self.next_id += 1;
                self.flights.push(Flight::new(id, format!("Flight {}", self.flights.len() + 1), spec));
                self.select_only(id);
                self.commit(ctx, "Add flight", None);
            }
            Message::RemoveFlight => {
                let ids = self.edit_ids();
                if ids.len() < self.flights.len() {
                    self.flights.retain(|f| !ids.contains(&f.id));
                    let first = self.flights[0].id;
                    self.select_only(first);
                    self.commit(ctx, if ids.len() > 1 { "Remove flights" } else { "Remove flight" }, None);
                } else if self.flights.len() == 1 {
                    ctx.notify("The last flight cannot be removed");
                } else {
                    ctx.notify("At least one flight must stay");
                }
            }
            Message::PerConfiguration => return self.per_configuration(ctx),
            Message::Disperse => {
                let id = self.selected;
                let task = self.launch_dispersion(id, ctx);
                self.rebuild();
                return task;
            }
            Message::Dispersed(done) => self.dispersed(*done),
            Message::Edit(edit) => {
                // One undo step per field and selection: scrubbing the wind is one step, changing the motor afterwards another.
                // With several flights selected the edit sets the field on every one of them.
                let ids = self.edit_ids();
                let (label, key) = match ids.as_slice() {
                    [id] if edit.named() => (edit.title(), format!("flight:{id}:{}", edit.label())),
                    [id] => ("Edit flight".to_string(), format!("flight:{id}:{}", edit.label())),
                    _ => {
                        let joined: Vec<String> = ids.iter().map(ToString::to_string).collect();
                        (format!("Set {} on {} flights", edit.label(), ids.len()), format!("flights:{}:{}", joined.join(","), edit.label()))
                    }
                };
                self.edit(edit, &ids, ctx.physics.motors());
                self.commit(ctx, &label, Some(&key));
            }
            Message::Plot(PlotMessage::Panel(panel, action, chip)) => return self.panel_action(panel, action, chip, ctx),
            Message::Plot(m) => self.plot.update(m),
            Message::Layout(edit) => self.edit_layout(edit, ctx),
            Message::PickerQuery(query) => {
                if let Some(picker) = &mut self.picker {
                    picker.query = query;
                }
            }
            Message::ClosePicker => self.picker = None,
            Message::KeepPicker => {}
            Message::WindTable(open) => self.wind_open = open,
            Message::CoordInput(lon, text) => {
                self.coord_editing = Some((lon, self.selected));
                self.coord_text[usize::from(lon)] = text;
            }
            Message::CoordCommit => {
                if let Some((lon, _)) = self.coord_editing.take() {
                    let text = self.coord_text[usize::from(lon)].trim();
                    let value = if text.is_empty() { Some(None) } else { text.parse::<f64>().ok().filter(|v| v.is_finite()).map(Some) };
                    if let Some(value) = value {
                        let edit = if lon { Edit::Longitude(value) } else { Edit::Latitude(value) };
                        return self.update(Message::Edit(edit), ctx);
                    }
                }
            }
            Message::Ignore => {}
            Message::ForecastOpen(open) => {
                self.forecast = open.then(|| ForecastRow { time: forecast::next_local_hour(), temperature: true, busy: false, error: None });
            }
            Message::ForecastTime(time) => {
                if let Some(row) = &mut self.forecast {
                    row.time = time;
                }
            }
            Message::ForecastTemperature(on) => {
                if let Some(row) = &mut self.forecast {
                    row.temperature = on;
                }
            }
            Message::ForecastFetch => return self.fetch_forecast(),
            Message::ForecastDone(result) => {
                let Some(row) = &mut self.forecast else { return Task::none() };
                row.busy = false;
                match result {
                    Ok(f) => {
                        let temperature = row.temperature.then_some(f.temperature_k).flatten();
                        self.forecast = None;
                        let edit = Edit::Forecast { speed: f.ground.speed, from: f.ground.from, levels: f.levels, temperature };
                        return self.update(Message::Edit(edit), ctx);
                    }
                    Err(e) => row.error = Some(e),
                }
            }
            Message::Ground(GroundMessage::Hover(t)) => self.plot.set_hover(t),
            Message::Ground(GroundMessage::Select(i)) => {
                if let Some(id) = self.shown.ids.get(i).copied() {
                    self.select_only(id);
                    self.rebuild();
                }
            }
            Message::Seek(t) => self.plot.set_hover(Some(t)),
            Message::StartRename(id) => {
                if let Some(flight) = self.flight(id) {
                    self.renaming = Some((id, flight.name.clone()));
                    self.select_only(id);
                    self.rebuild();
                    return iced::widget::operation::focus(left::RENAME_ID).chain(iced::widget::operation::text_input::select_all(left::RENAME_ID));
                }
            }
            Message::RenameInput(text) => {
                if let Some((_, buffer)) = &mut self.renaming {
                    *buffer = text;
                }
            }
            Message::CancelRename => self.renaming = None,
            Message::CommitRename => {
                if let Some((id, name)) = self.renaming.take() {
                    let name = name.trim().to_string();
                    if let Some(flight) = self.flights.iter_mut().find(|f| f.id == id).filter(|f| !name.is_empty() && f.name != name) {
                        flight.name = name;
                        self.commit(ctx, "Rename flight", None);
                    }
                }
            }
        }
        Task::none()
    }

    fn dispersed(&mut self, done: DispersionDone) {
        let Some(flight) = self.flights.iter_mut().find(|f| f.id == done.flight) else { return };
        if flight.dispersion_ticket != done.ticket {
            return;
        }
        flight.dispersing = false;
        match done.outcome {
            Ok(landings) => {
                flight.dispersion = Some(Dispersed { spec: done.spec, revision: done.revision, landings });
                flight.error = None;
            }
            Err(message) => flight.error = Some(message),
        }
        self.rebuild();
    }

    /// Starts a dispersion study of flight `id`: the flight flown [`flight::DISPERSION_RUNS`] times with its conditions varied.
    fn launch_dispersion(&mut self, id: FlightId, ctx: &Ctx) -> Task<app::Message> {
        let Some(spec) = self.flight(id).map(|f| f.spec.clone()) else { return Task::none() };
        let revision = self.stamp(ctx, &spec);
        let Some(flight) = self.flights.iter_mut().find(|f| f.id == id) else { return Task::none() };
        let input = match flight::prepare(ctx, &spec) {
            Ok((input, _)) => input,
            Err(message) => {
                flight.error = Some(message);
                return Task::none();
            }
        };
        flight.dispersion_ticket += 1;
        flight.dispersing = true;
        let ticket = flight.dispersion_ticket;
        let sent = spec.clone();
        Task::perform(disperse(input, spec), move |outcome| {
            app::Message::Simulate(Message::Dispersed(Box::new(DispersionDone { flight: id, ticket, spec: sent.clone(), revision, outcome })))
        })
    }

    fn finished(&mut self, done: Done) {
        let Some(flight) = self.flights.iter_mut().find(|f| f.id == done.flight) else { return };
        if flight.ticket != done.ticket {
            return;
        }
        flight.running = false;
        match done.outcome {
            Ok((result, took)) => {
                let times = Times::of(&result);
                flight.run = Some(Run { spec: done.spec, revision: done.revision, result, took, dt: done.dt, delay: done.delay, times });
                flight.error = None;
            }
            Err(message) => flight.error = Some(message),
        }
        self.rebuild();
    }

    /// Ticks or unticks `id`. A fifth tick is refused; the colour slot stays with the flight until it is unticked.
    fn tick_flight(&mut self, id: FlightId, ctx: &mut Ctx) {
        let used: Vec<u8> = self.flights.iter().filter_map(|f| f.slot).collect();
        let free = (1..=MAX_COMPARED as u8).find(|s| !used.contains(s));
        let Some(flight) = self.flights.iter_mut().find(|f| f.id == id) else { return };
        match (flight.slot, free) {
            (Some(_), _) => flight.slot = None,
            (None, Some(slot)) => flight.slot = Some(slot),
            (None, None) => {
                ctx.notify(format!("Compare up to {MAX_COMPARED} flights: untick one first"));
                return;
            }
        }
        self.rebuild();
    }

    /// One flight per configuration of the document that has none yet, then ticks the first four configurations' flights and
    /// runs them (ConfigPicker, "In other modes": the quickest way to compare motors).
    fn per_configuration(&mut self, ctx: &mut Ctx) -> Task<app::Message> {
        let rocket = ctx.doc.rocket();
        let configs: Vec<(u64, String)> = rocket.configurations().iter().filter_map(|c| Some((c.id, rocket.configuration_name(c.id)?))).collect();
        if configs.len() < 2 {
            ctx.notify("The rocket has only one configuration");
            return Task::none();
        }
        let base = Spec { motor: MotorChoice::Active, ..self.selected_flight().spec.clone() };
        let mut chosen = Vec::new();
        for (config, name) in configs {
            let existing = self.flights.iter().find(|f| f.spec.motor == MotorChoice::Config(config)).map(|f| f.id);
            let id = existing.unwrap_or_else(|| {
                let id = self.next_id;
                self.next_id += 1;
                self.flights.push(Flight::new(id, name, Spec { motor: MotorChoice::Config(config), delay: None, ..base.clone() }));
                id
            });
            chosen.push(id);
        }
        chosen.truncate(MAX_COMPARED);
        for f in &mut self.flights {
            f.slot = None;
        }
        for (i, id) in chosen.iter().enumerate() {
            if let Some(f) = self.flights.iter_mut().find(|f| f.id == *id) {
                f.slot = Some(i as u8 + 1);
            }
        }
        self.select_only(chosen[0]);
        self.commit(ctx, "Add flights", None);
        let task = self.launch_all(chosen, ctx);
        self.rebuild();
        task
    }

    /// Applies `edit` to the conditions of every flight in `ids`.
    fn edit(&mut self, edit: Edit, ids: &[FlightId], motors: &MotorDb) {
        for flight in self.flights.iter_mut().filter(|f| ids.contains(&f.id)) {
            let spec = &mut flight.spec;
            match &edit {
                Edit::Motor(choice) => {
                    spec.delay = match (choice, spec.delay) {
                        // Keep a typed delay when it is one the new motor comes with, else the nearest it offers.
                        (MotorChoice::Motor(m), Some(d)) => match bowshock_sim::from_model::find_motor(motors, m) {
                            Some(motor) if !motor.delays.is_empty() => motor.delays.iter().copied().min_by(|a, b| (a - d).abs().total_cmp(&(b - d).abs())),
                            _ => Some(d),
                        },
                        _ => None,
                    };
                    spec.motor = choice.clone();
                }
                Edit::Delay(v) => spec.delay = Some(v.max(0.0)),
                Edit::RailLength(v) => spec.rail_length = v.max(0.05),
                Edit::LaunchAngle(v) => spec.launch_angle = v.clamp(0.0, 70f64.to_radians()),
                Edit::Wind(v) => spec.wind_speed = v.max(0.0),
                Edit::WindFrom(v) => spec.wind_from = v.rem_euclid(std::f64::consts::TAU),
                Edit::Turbulence(t) => spec.turbulence = *t,
                Edit::Drag(d) => spec.drag = *d,
                Edit::SiteAltitude(v) => spec.site_altitude = v.clamp(-400.0, 6000.0),
                Edit::Temperature(v) => spec.temperature = v.clamp(220.0, 330.0),
                Edit::FieldRadius(v) => spec.field_radius = v.max(1.0),
                Edit::WindLevels(levels) => spec.wind_levels = bowshock_model::sanitized_levels(levels),
                Edit::Latitude(lat) => {
                    spec.location = lat.filter(|v| v.is_finite()).map(|latitude| bowshock_model::Location {
                        latitude: latitude.clamp(-90.0, 90.0),
                        longitude: spec.location.map_or(0.0, |l| l.longitude),
                    })
                }
                Edit::Longitude(lon) => {
                    spec.location = lon.filter(|v| v.is_finite()).map(|longitude| bowshock_model::Location {
                        latitude: spec.location.map_or(0.0, |l| l.latitude),
                        longitude: (longitude + 180.0).rem_euclid(360.0) - 180.0,
                    })
                }
                Edit::AirStart(start) => {
                    spec.air_start = start.map(|a| bowshock_model::AirStart { altitude: a.altitude.clamp(0.0, 30_000.0), speed: a.speed.clamp(0.0, 2000.0) });
                }
                Edit::Site(i) => {
                    if let Some(site) = sites::SITES.get(*i) {
                        spec.site_altitude = site.altitude;
                        spec.location = Some(bowshock_model::Location { latitude: site.latitude, longitude: site.longitude });
                        if (spec.temperature - 288.15).abs() < 1e-9 {
                            spec.temperature = sites::isa_temperature(site.altitude);
                        }
                    }
                }
                Edit::Forecast { speed, from, levels, temperature } => {
                    spec.wind_speed = speed.max(0.0);
                    spec.wind_from = from.rem_euclid(std::f64::consts::TAU);
                    spec.wind_levels = bowshock_model::sanitized_levels(levels);
                    if let Some(t) = temperature {
                        spec.temperature = t.clamp(220.0, 330.0);
                    }
                }
            }
        }
    }

    /// The shortcut `key` maps to on this screen, if any. Only called for keys no widget handled. `F5` and `Esc` are global; the
    /// plots take `Enter` and `Esc` themselves while the cursor is over them, and `Enter` zooms to the band from anywhere else.
    pub fn on_key(&self, key: &keyboard::Key, modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        if self.renaming.is_some() && matches!(key, keyboard::Key::Named(keyboard::key::Named::Escape)) {
            return Some(Message::CancelRename);
        }
        // Cmd/Ctrl+A selects every flight (a focused text field takes the key first, so this is only reached from the list).
        if let keyboard::Key::Character(c) = key {
            if modifiers.command() && !modifiers.shift() && c.eq_ignore_ascii_case("a") {
                return Some(Message::SelectAll);
            }
        }
        let enter = matches!(key, keyboard::Key::Named(keyboard::key::Named::Enter));
        (enter && modifiers.is_empty() && self.plot.can_zoom_in()).then_some(Message::Plot(PlotMessage::ZoomIn))
    }

    /// Advances animations (called on frame ticks while [`animating`](Self::animating)).
    pub fn tick(&mut self, _now: Instant) {}

    /// Whether the screen needs frame ticks.
    pub fn animating(&self) -> bool {
        false
    }

    /// The screen: flights and conditions | plots | results and ground track.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let left = left::view(self, ctx);
        let right = results::view(self, ctx);
        row![
            container(left).width(RAIL_W + BIN_W).height(Fill).style(theme::panel),
            container(self.centre(ctx)).width(Fill).height(Fill).style(crate::widgets::plot::surface),
            container(right).width(RIGHT_PANEL_W).height(Fill).style(theme::panel),
        ]
        .height(Fill)
        .boxed()
    }

    /// The centre column: the PlotZoom header, the presets row, the legend of a comparison, the plots (with the source picker over
    /// them while it is open).
    fn centre<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let Some(group) = self.shown.group.as_ref() else { return self.placeholder() };
        let bar = |e: Element<'a, Message>| container(e).padding([SPACE_2, SPACE_3]).boxed();
        let (crumbs, hint) = header_parts(group, &self.plot, true);
        // One row: the zoom breadcrumb, the plot presets, then the hints; it wraps onto a second line in a narrow window instead of
        // running off the pane.
        let mut items: Vec<Element<'a, Message>> = vec![crumbs.map(Message::Plot).boxed()];
        items.extend(self.presets());
        items.push(hint.map(Message::Plot).boxed());
        let head = iced::widget::Row::with_children(items).spacing(SPACE_2).align_y(iced::Center).wrap().vertical_spacing(SPACE_2);
        let mut parts: Vec<Element<'a, Message>> = vec![bar(head.boxed())];
        if self.shown.compare {
            let items: Vec<(SeriesColor, String)> =
                self.shown.ids.iter().filter_map(|id| self.flight(*id)).map(|f| (SeriesColor::Series(f.slot.unwrap_or(1)), f.name.clone())).collect();
            let reference = self.shown.ids.get(self.shown.reference).and_then(|id| self.flight(*id)).map_or("", |f| f.name.as_str());
            let note = text(format!("events and timeline: {reference} (reference)")).font(MONO).size(TEXT_LABEL).style(theme::faint);
            parts.push(bar(row![legend(&items), Space::new().width(Fill), note].align_y(iced::Center).boxed()));
        }
        let canvas = container(PlotView::new(group, &self.plot).pinned(self.picker.is_some()).selection_chip(true).element().map(Message::Plot))
            .width(Fill)
            .height(Fill);
        parts.push(match &self.picker {
            Some(picker) => stack![canvas, picker.view(&self.layout, ctx.units)].boxed(),
            None => canvas.boxed(),
        });
        column(parts).boxed()
    }

    /// The presets row: which layout the plots follow, and a button to add a plot.
    fn presets(&self) -> Vec<Element<'_, Message>> {
        let options: Vec<(Preset, &str)> = Preset::ALL.iter().map(|p| (*p, p.label())).collect();
        let strip = segmented_dense(
            &options,
            Preset::of(&self.layout),
            |p| Message::Layout(LayoutEdit::Preset(p)),
            Dense { height: 24.0, pad: 6.0, size: 12.0 },
            false,
        );
        let mut add = icon_button(Mark::Plus, "Add a plot below the others");
        if self.layout.panels.len() >= bowshock_model::MAX_PLOT_PANELS {
            add = add.disabled("The plots hold at most 8 panels");
        } else {
            add = add.on_press(Message::Layout(LayoutEdit::AddPanel));
        }
        let against: Vec<(PlotX, &str)> = PlotX::ALL.iter().map(|x| (*x, plots::x_label(*x))).collect();
        let x = segmented_dense(&against, self.layout.x, |x| Message::Layout(LayoutEdit::X(x)), Dense { height: 24.0, pad: 6.0, size: 12.0 }, false);
        vec![strip, add.into(), crate::app::ui::label("VS"), x]
    }

    /// What the centre shows before the selected flight has run.
    fn placeholder<'a>(&'a self) -> Element<'a, Message> {
        let flight = self.selected_flight();
        let status: Element<'a, Message> = if flight.running {
            text("Running…").size(TEXT_BODY).style(theme::muted).boxed()
        } else if let Some(error) = &flight.error {
            chip(Severity::Err, error.clone()).into()
        } else {
            let what = match self.edit_ids().len() {
                1 => flight.name.clone(),
                n => format!("{n} flights"),
            };
            text(format!("Press Run (F5) to fly {what}")).size(TEXT_BODY).style(theme::muted).boxed()
        };
        container(column![crate::app::ui::label("NO RESULT YET"), status].spacing(SPACE_3).align_x(iced::Center)).center(Fill).boxed()
    }
}

/// `layout` as the plots draw it: sources this version does not know, or that cannot share their plot's axis, are left out.
fn repaired(layout: &PlotLayout) -> PlotLayout {
    let mut layout = layout.clone();
    layout::repair(&mut layout);
    layout
}

/// A thin vertical scrollbar, so the panels' content is not narrowed.
fn thin_scrollbar() -> iced::widget::scrollable::Direction {
    iced::widget::scrollable::Direction::Vertical(iced::widget::scrollable::Scrollbar::new().width(4).scroller_width(4).margin(1))
}

/// The scrollbar in token colours: no rail, a square `line-200` scroller that brightens under the cursor.
fn scroll_style(theme: &iced::Theme, status: iced::widget::scrollable::Status) -> iced::widget::scrollable::Style {
    use iced::widget::scrollable::{Rail, Scroller, Status};
    let t = theme::tokens(theme);
    let hot = matches!(status, Status::Hovered { .. } | Status::Dragged { .. });
    let rail = Rail {
        background: None,
        border: iced::Border::default(),
        scroller: Scroller { background: iced::Background::Color(if hot { t.ink_faint } else { t.line_200 }), border: iced::Border::default() },
    };
    iced::widget::scrollable::Style { vertical_rail: rail, horizontal_rail: rail, ..iced::widget::scrollable::default(theme, status) }
}

/// Flies `input` on a worker thread and waits for it without blocking the UI's executor.
async fn compute(input: bowshock_sim::SimInput) -> Result<(Arc<FlightResult>, Duration), String> {
    offload(move || {
        let started = Instant::now();
        let result = bowshock_sim::simulate(&input);
        (Arc::new(result), started.elapsed())
    })
    .await
}

/// Flies a dispersion study of `input` (see [`flight::tolerances`]) on a worker thread (the study itself fans out over cores).
async fn disperse(input: bowshock_sim::SimInput, spec: Spec) -> Result<Vec<(f64, f64)>, String> {
    offload(move || {
        let study = bowshock_sim::monte_carlo(&input, flight::DISPERSION_RUNS, &flight::tolerances(&spec));
        study.runs.iter().map(|r| (r.landing[0], r.landing[1])).collect()
    })
    .await
}

/// Runs `work` on its own thread and waits for it without blocking the UI's executor.
async fn offload<T: Send + 'static>(work: impl FnOnce() -> T + Send + 'static) -> Result<T, String> {
    let (tx, rx) = iced::futures::channel::oneshot::channel();
    std::thread::spawn(move || {
        // The receiver is gone only if the app quit meanwhile.
        let _ = tx.send(work());
    });
    rx.await.map_err(|_| "the simulation stopped unexpectedly".to_string())
}

/// The forecast row of the wind profile: the launch time, whether the forecast sets the temperature, a fetch in flight and the
/// last error.
#[derive(Debug, Clone, PartialEq)]
struct ForecastRow {
    /// The launch hour, `YYYY-MM-DDTHH:00` in the site's local time.
    time: String,
    /// Use the forecast temperature.
    temperature: bool,
    /// A fetch is running.
    busy: bool,
    /// Why the last fetch failed.
    error: Option<String>,
}

impl State {
    /// Starts fetching the forecast for the selected flight's location and the typed hour (see [`forecast::fetch`]).
    fn fetch_forecast(&mut self) -> Task<app::Message> {
        let spec = self.selected_flight().spec.clone();
        let Some(row) = &mut self.forecast else { return Task::none() };
        let Some(at) = spec.location else {
            row.error = Some("Set the launch latitude and longitude first".to_string());
            return Task::none();
        };
        row.busy = true;
        row.error = None;
        let (time, altitude) = (row.time.clone(), spec.site_altitude);
        Task::perform(offload(move || forecast::fetch(at.latitude, at.longitude, altitude, &time)), |result| {
            app::Message::Simulate(Message::ForecastDone(result.and_then(|r| r)))
        })
    }
}

#[cfg(test)]
#[allow(unused_must_use)] // The tests drive `update` directly and have no runtime to hand its tasks to.
mod tests;
