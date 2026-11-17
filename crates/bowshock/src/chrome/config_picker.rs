//! The motor picker (design/components/ConfigPicker): the modal that builds a new configuration. The header states the mount it
//! filters by, the table lists every motor with the stability margin the rocket would have with it, motors that do not fit stay
//! listed faint with the reason, and a staged rocket gets one tab per stage.
//!
//! Two sources share the table: the **Catalogue** (the bundled ThrustCurve.org snapshot, the user's `.eng` files and the
//! motors downloaded before) and a live **ThrustCurve.org** search. Typing in the live search waits for a pause
//! ([`DEBOUNCE`]), then searches on a worker thread; picking a result downloads its curve (also off the UI thread), caches it
//! in the user motors directory, and selects it in the catalogue table. Without a connection the search says so and the
//! catalogue keeps working.

use std::collections::BTreeMap;
use std::time::Duration;

use bowshock_model::StabilityBands;
use bowshock_model::{MotorMount, MotorRef, PartId};
use bowshock_motors::thrustcurve::{self as tc, Client, RemoteMotor, TcError};
use bowshock_motors::{Motor, MotorDb};
use bowshock_sim::from_model;
use iced::widget::{button, column, container, mouse_area, row, scrollable, text, text_input, Space};
use iced::{Background, Center, Element, Fill, Length, Task, Theme, Widget};

use super::Message as ChromeMessage;
use crate::app::physics::MotorOrigin;
use crate::app::{self, ui, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{Tokens, SPACE_2, SPACE_3, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::button::{button as action, Weight};
use crate::widgets::segmented::segmented;
use crate::widgets::spinner::spinner;
use crate::widgets::unit_input::unit_input;

/// Tolerance for a motor wider or longer than its mount (the bridge's own).
const FIT_TOL: f64 = 0.5e-3;
/// Width of the picker panel: the kit's ConfigPicker, which leaves most of the viewport and the model in sight.
const WIDTH: f32 = 520.0;
/// Height cap of the scrolling table (about seven rows; the rest scrolls inside it).
const TABLE_H: f32 = 190.0;
/// Table text: the kit's 11.5 px mono; column heads are 10 px.
const ROW_SIZE: f32 = 11.5;
const HEAD_SIZE: f32 = 10.0;
/// Labels of the stage tabs (a rocket with more stages than this does not exist in practice).
const STAGE_LABELS: [&str; 6] = ["Stage 1", "Stage 2", "Stage 3", "Stage 4", "Stage 5", "Stage 6"];
/// Most rows the table lays out; the rest are counted in a last row that asks to narrow the list.
const MAX_ROWS: usize = 150;
/// How long typing must pause before the live search runs.
pub const DEBOUNCE: Duration = Duration::from_millis(400);
/// The longest a live search or download may take before it is reported as unreachable.
const NETWORK_TIMEOUT: Duration = Duration::from_secs(12);
/// Id of the search field (for scripts: `click #motor-search`).
pub const SEARCH_ID: &str = "motor-search";

/// The ThrustCurve.org client of the picker. `api_base` (`BOWSHOCK_THRUSTCURVE_API`) replaces the API root (a mirror, or an
/// unreachable address to see the offline state).
fn client(api_base: Option<&str>) -> Client {
    let client = Client::with_timeout(NETWORK_TIMEOUT);
    match api_base {
        Some(base) => client.with_base(base),
        None => client,
    }
}

/// Where the table's motors come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// The motor database: the bundled catalogue, the user's files and earlier downloads.
    Catalogue,
    /// A live ThrustCurve.org search.
    ThrustCurve,
}

/// The picker's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// The search text changed (of the source on show).
    Search(String),
    /// An impulse class was toggled in the filter.
    ToggleClass(char),
    /// A stage tab (model stage index, from 1).
    Tab(usize),
    /// The source switch.
    Source(Source),
    /// A motor row was picked (index into the motor database).
    Select(usize),
    /// The ejection delay for the picked motor, seconds.
    Delay(f64),
    /// Add the configuration.
    Add,
    /// Close without adding.
    Cancel,
    /// "Load .eng…": pick motor files of one's own.
    LoadFiles,
    /// Typing paused: search if `generation` is still the latest text.
    Debounced(u64),
    /// The answer to the live search of `generation`.
    Results(u64, Result<Vec<RemoteMotor>, TcError>),
    /// A live search result was picked (index into the results).
    Pick(usize),
    /// The download of ThrustCurve motorId `.0` finished.
    Downloaded(String, Result<Box<Motor>, TcError>),
}

/// What the chrome should do after a picker message.
#[derive(Debug, PartialEq)]
pub enum Outcome {
    /// Keep the picker open.
    Stay,
    /// Close it.
    Close,
    /// Open the file dialog for the user's own `.eng` files.
    LoadFiles,
    /// Start the typing pause of search `generation` ([`debounce`]).
    Debounce(u64),
    /// Run live search `generation` for the text ([`search`]).
    Search(u64, String),
    /// Download this motor's curve ([`download`]).
    Download(Box<RemoteMotor>),
}

/// A motor chosen for a mount, with its ejection delay.
#[derive(Debug, Clone, Copy)]
struct Choice {
    motor: usize,
    delay: f64,
}

/// One row of the table.
#[derive(Debug, Clone)]
struct Row {
    motor: usize,
    /// Why it does not fit the mount, if it does not.
    problem: Option<&'static str>,
    /// The margin the rocket would have with it.
    margin: Option<f64>,
}

/// The state of the live search.
#[derive(Debug, Clone, PartialEq)]
enum Remote {
    /// Nothing typed yet.
    Idle,
    /// A search is running (or about to, after the typing pause).
    Searching,
    /// What ThrustCurve.org answered.
    Results(Vec<RemoteMotor>),
    /// Why the search failed, as shown.
    Failed(String),
}

/// The motor picker's state.
#[derive(Debug)]
pub struct Picker {
    /// The stage whose mount the table is for (model stage index, from 1).
    stage: usize,
    search: String,
    /// Impulse classes the table is narrowed to; none selected shows all.
    classes: Vec<char>,
    /// The motor and delay picked for each mount.
    choices: BTreeMap<PartId, Choice>,
    rows: Vec<Row>,
    source: Source,
    /// The live search's text, its generation (bumped per keystroke, so only the latest answer counts) and state.
    remote_query: String,
    generation: u64,
    remote: Remote,
    /// The motorId being downloaded.
    downloading: Option<String>,
    /// Why the last download failed.
    download_error: Option<String>,
}

/// Why a live request failed, in the words the table shows.
fn failure(e: &TcError) -> String {
    match e {
        TcError::Unreachable(_) => "ThrustCurve unreachable".to_string(),
        other => other.to_string(),
    }
}

impl Picker {
    /// Opens the picker on stage 1, starting from the active configuration's motors; `Err` says why it cannot open.
    pub fn open(ctx: &Ctx) -> Result<Picker, &'static str> {
        let rocket = ctx.doc.rocket();
        let mounts = rocket.motor_mounts();
        if mounts.is_empty() {
            return Err("Add a motor mount to the rocket first");
        }
        let motors = ctx.physics.motors();
        let mut choices = BTreeMap::new();
        for slot in &rocket.active_configuration().motors {
            let Some(reference) = &slot.motor else { continue };
            if let Some(motor) = from_model::find_motor_index(motors, reference) {
                let delay = if slot.delay.is_finite() && slot.delay > 0.0 { slot.delay } else { motors.motors[motor].delays.first().copied().unwrap_or(0.0) };
                choices.insert(slot.mount, Choice { motor, delay });
            }
        }
        let first = ctx.physics.stages().first().map_or(1, |s| s.index);
        let mut picker = Picker {
            stage: first,
            search: String::new(),
            classes: Vec::new(),
            choices,
            rows: Vec::new(),
            source: Source::Catalogue,
            remote_query: String::new(),
            generation: 0,
            remote: Remote::Idle,
            downloading: None,
            download_error: None,
        };
        picker.rebuild(ctx);
        picker.narrow_to_current(ctx);
        Ok(picker)
    }

    /// Narrows the table to the class of the motor now in the mount and its neighbours (a new configuration is usually a step
    /// up or down), so the list opens short and around the current choice. With no motor in the mount it shows all.
    fn narrow_to_current(&mut self, ctx: &Ctx) {
        let motors = &ctx.physics.motors().motors;
        let Some(choice) = self.mount(ctx).and_then(|m| self.choices.get(&m.id)) else {
            return;
        };
        let class = motors[choice.motor].impulse_class();
        let present: Vec<char> = self.rows.iter().map(|r| motors[r.motor].impulse_class()).collect();
        let near = [(class as u8).saturating_sub(1) as char, class, (class as u8 + 1) as char];
        self.classes = near.into_iter().filter(|c| present.contains(c)).collect();
    }

    /// The motor mount the table is for: the first of the stage (a cluster flies its first motor).
    fn mount(&self, ctx: &Ctx) -> Option<MotorMount> {
        ctx.physics.mounts_of_stage(ctx.doc.rocket(), self.stage).into_iter().next()
    }

    /// Rebuilds the table after the motor database grew, and shows every class so the new motors are visible.
    pub fn refresh(&mut self, ctx: &Ctx) {
        self.classes.clear();
        self.rebuild(ctx);
    }

    /// Whether a live search or download is running (the app keeps frames coming for the spinner).
    pub fn busy(&self) -> bool {
        self.remote == Remote::Searching || self.downloading.is_some()
    }

    /// Why motor `motor` does not fit `mount`, if it does not.
    fn problem(mount: &MotorMount, diameter: f64, length: f64) -> Option<&'static str> {
        if diameter > mount.inner_diameter + FIT_TOL {
            Some("Too wide for mount")
        } else if length > mount.length + mount.overhang + FIT_TOL {
            Some("Too long for mount")
        } else {
            None
        }
    }

    /// Recomputes the table for the current stage: fit, and the margin with each fitting motor.
    fn rebuild(&mut self, ctx: &Ctx) {
        self.rows.clear();
        let Some(mount) = self.mount(ctx) else {
            return;
        };
        let motors = &ctx.physics.motors().motors;
        self.rows =
            motors.iter().enumerate().map(|(index, m)| Row { motor: index, problem: Self::problem(&mount, m.diameter, m.length), margin: None }).collect();
        let fitting: Vec<usize> = self.rows.iter().filter(|r| r.problem.is_none()).map(|r| r.motor).collect();
        let margins = ctx.physics.margins_with_motors(ctx.doc.rocket(), &mount, &fitting);
        for (index, margin) in fitting.into_iter().zip(margins) {
            self.rows[index].margin = margin;
        }
        // Motors that fit first, then the ones a longer mount would take, then the ones a wider mount would; by impulse.
        let rank = |r: &Row| match r.problem {
            None => 0,
            Some("Too long for mount") => 1,
            Some(_) => 2,
        };
        self.rows.sort_by(|a, b| rank(a).cmp(&rank(b)).then(motors[a.motor].total_impulse().total_cmp(&motors[b.motor].total_impulse())));
    }

    /// Shows motor `index` picked in the catalogue table: the table switches to the catalogue, searched for its designation.
    fn show_picked(&mut self, ctx: &Ctx, index: usize) {
        self.source = Source::Catalogue;
        self.search = ctx.physics.motors().motors[index].designation.clone();
        self.classes.clear();
        self.rebuild(ctx);
        if let Some(mount) = self.mount(ctx) {
            let delay = ctx.physics.motors().motors[index].delays.first().copied().unwrap_or(0.0);
            self.choices.insert(mount.id, Choice { motor: index, delay });
        }
    }

    /// Handles a picker message.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Outcome {
        match message {
            Message::Search(s) if self.source == Source::ThrustCurve => {
                self.remote_query = s;
                self.generation += 1;
                if self.remote_query.trim().is_empty() {
                    self.remote = Remote::Idle;
                } else {
                    self.remote = Remote::Searching;
                    return Outcome::Debounce(self.generation);
                }
            }
            Message::Search(s) => self.search = s,
            Message::ToggleClass(c) => match self.classes.iter().position(|x| *x == c) {
                Some(i) => {
                    self.classes.remove(i);
                }
                None => self.classes.push(c),
            },
            Message::Tab(stage) => {
                self.stage = stage;
                self.rebuild(ctx);
            }
            Message::Source(source) => {
                self.source = source;
                self.download_error = None;
            }
            Message::Select(motor) => {
                if let Some(mount) = self.mount(ctx) {
                    let delay = ctx.physics.motors().motors[motor].delays.first().copied().unwrap_or(0.0);
                    self.choices.insert(mount.id, Choice { motor, delay });
                }
            }
            Message::Delay(delay) => {
                if let Some(choice) = self.mount(ctx).and_then(|m| self.choices.get_mut(&m.id)) {
                    choice.delay = delay;
                }
            }
            Message::LoadFiles => return Outcome::LoadFiles,
            Message::Cancel => return Outcome::Close,
            Message::Add => {
                self.add(ctx);
                return Outcome::Close;
            }
            Message::Debounced(generation) => {
                if generation == self.generation && !self.remote_query.trim().is_empty() {
                    return Outcome::Search(generation, self.remote_query.trim().to_string());
                }
            }
            Message::Results(generation, result) => {
                if generation == self.generation {
                    self.remote = match result {
                        Ok(found) => Remote::Results(found),
                        Err(e) => Remote::Failed(failure(&e)),
                    };
                }
            }
            Message::Pick(i) => {
                let Remote::Results(found) = &self.remote else {
                    return Outcome::Stay;
                };
                let Some(remote) = found.get(i).cloned() else {
                    return Outcome::Stay;
                };
                if self.downloading.is_some() {
                    return Outcome::Stay;
                }
                self.download_error = None;
                let known = ctx.physics.motors().position(&remote.id());
                match known {
                    // Downloaded before: the cached copy is selected without fetching it again.
                    Some(index) if ctx.physics.motor_origin(index) == MotorOrigin::Downloaded => self.show_picked(ctx, index),
                    _ if remote.data_files == 0 => self.download_error = Some(format!("ThrustCurve has no thrust curve for {}", remote.designation)),
                    // New, or in the bundled catalogue: fetch ThrustCurve's current data, which then replaces the snapshot's copy.
                    _ => {
                        self.downloading = Some(remote.motor_id.clone());
                        return Outcome::Download(Box::new(remote));
                    }
                }
            }
            Message::Downloaded(motor_id, result) => {
                if self.downloading.as_deref() == Some(motor_id.as_str()) {
                    self.downloading = None;
                }
                match result {
                    Ok(motor) => {
                        let (index, note) = ctx.physics.add_downloaded(*motor);
                        if let Some(note) = note {
                            ctx.notify(note);
                        }
                        self.show_picked(ctx, index);
                    }
                    Err(e) => self.download_error = Some(failure(&e)),
                }
            }
        }
        Outcome::Stay
    }

    /// Adds the configuration and makes it current. It starts as a copy of the active one (its motors, staging and recovery
    /// timing), without the ballast, which belongs to the motor it balanced; stages with no pick keep what the copy has.
    fn add(&self, ctx: &mut Ctx) {
        let motors: Vec<(PartId, MotorRef, f64)> =
            self.choices.iter().map(|(&mount, c)| (mount, from_model::motor_ref(&ctx.physics.motors().motors[c.motor]), c.delay)).collect();
        ctx.doc.edit("Add configuration", |r| {
            let Some(id) = r.duplicate_configuration(r.active_config_id()) else {
                return;
            };
            // A new configuration is named after its motors until it is renamed.
            r.rename_configuration(id, "");
            let _ = r.set_ballast(id, None);
            for (mount, motor, delay) in motors {
                let _ = r.set_motor(id, mount, Some(motor));
                let _ = r.set_motor_delay(id, mount, delay);
            }
            r.select_configuration(id);
        });
    }

    fn visible(&self, ctx: &Ctx) -> Vec<&Row> {
        let motors = &ctx.physics.motors().motors;
        let needle = self.search.trim().to_lowercase();
        self.rows
            .iter()
            .filter(|r| {
                let m = &motors[r.motor];
                let maker_name = m.thrustcurve.as_ref().map_or("", |i| i.manufacturer_name.as_str());
                (self.classes.is_empty() || self.classes.contains(&m.impulse_class()))
                    && (needle.is_empty()
                        || m.designation.to_lowercase().contains(&needle)
                        || m.manufacturer.to_lowercase().contains(&needle)
                        || maker_name.to_lowercase().contains(&needle))
            })
            .collect()
    }

    /// The modal: a scrim that closes it and the panel.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, ChromeMessage> {
        let msg = ChromeMessage::Picker;
        let units = ctx.units;
        let motors = ctx.physics.motors();
        let mount = self.mount(ctx);

        let constraint = match &mount {
            Some(m) => format!(
                "Mount Ø{} · up to {} long",
                ui::bare(m.inner_diameter, Quantity::Length, units),
                ui::bare(m.length + m.overhang, Quantity::Length, units)
            ),
            None => String::new(),
        };
        let header = row![ui::label("NEW CONFIGURATION · MOTOR"), Space::new().width(Fill), text(constraint).font(MONO).size(ROW_SIZE).style(theme::muted)]
            .align_y(Center);

        // The source switch, with the data credit.
        let credit = match self.source {
            Source::Catalogue => format!("{} · {}", tc::CREDIT, MotorDb::catalogue_info().snapshot),
            Source::ThrustCurve => format!("{} · live", tc::CREDIT),
        };
        let sources = row![
            segmented(&[(Source::Catalogue, "Catalogue"), (Source::ThrustCurve, "ThrustCurve.org")], self.source, move |s| msg(Message::Source(s))),
            Space::new().width(Fill),
            text(credit).font(MONO).size(HEAD_SIZE).style(theme::faint),
        ]
        .align_y(Center);

        let stages = ctx.physics.stages();
        let tabs: Option<Element<'a, ChromeMessage>> = (stages.len() > 1).then(|| {
            let options: Vec<(usize, &str)> = stages.iter().map(|s| (s.index, STAGE_LABELS[(s.index - 1).min(STAGE_LABELS.len() - 1)])).collect();
            segmented(&options, self.stage, |s| msg(Message::Tab(s)))
        });

        let (placeholder, value) = match self.source {
            Source::Catalogue => ("Search designation or maker", &self.search),
            Source::ThrustCurve => ("Search ThrustCurve.org: H128W, H128, Cesaroni J", &self.remote_query),
        };
        let search =
            text_input(placeholder, value).id(SEARCH_ID).on_input(move |s| msg(Message::Search(s))).size(TEXT_BODY).padding([4.0, SPACE_2]).style(theme::input);
        let filters: Element<'a, ChromeMessage> = match self.source {
            Source::Catalogue => {
                let mut classes: Vec<char> =
                    self.rows.iter().filter(|r| r.problem != Some("Too wide for mount")).map(|r| motors.motors[r.motor].impulse_class()).collect();
                classes.sort_unstable();
                classes.dedup();
                let class_buttons = row(classes.into_iter().map(|c| {
                    let active = self.classes.contains(&c);
                    let label = c.to_string();
                    let cell: Element<'a, ChromeMessage> = column![
                        button(text(label).font(MONO).size(TEXT_BODY))
                            .padding([0.0, SPACE_2])
                            .height(24)
                            .style(theme::segment(active))
                            .on_press(msg(Message::ToggleClass(c))),
                        container(Space::new()).width(Fill).height(2).style(theme::underline(active)),
                    ]
                    .boxed();
                    cell
                }))
                .spacing(1);
                row![container(search).width(Fill), container(class_buttons).padding(1).style(theme::segmented_frame)].spacing(SPACE_2).align_y(Center).boxed()
            }
            Source::ThrustCurve => container(search).width(Fill).boxed(),
        };

        let column_head = |name: &'static str, width: Length| container(text(name).font(MONO).size(HEAD_SIZE).style(theme::faint)).width(width);
        let head = match self.source {
            Source::Catalogue => row![
                column_head("MOTOR", Length::FillPortion(11)),
                column_head("IMPULSE", Length::FillPortion(10)),
                column_head("LENGTH", Length::FillPortion(10)),
                column_head("MASS", Length::FillPortion(10)),
                column_head("WITH THIS MOTOR", Length::FillPortion(17)),
            ],
            Source::ThrustCurve => row![
                column_head("MOTOR", Length::FillPortion(11)),
                column_head("MAKER", Length::FillPortion(11)),
                column_head("SIZE", Length::FillPortion(12)),
                column_head("IMPULSE", Length::FillPortion(10)),
                column_head("STATUS", Length::FillPortion(14)),
            ],
        }
        .padding([0.0, SPACE_3 - 2.0])
        .align_y(Center);
        let head = container(head).height(26).center_y(26);

        let picked = mount.as_ref().and_then(|m| self.choices.get(&m.id)).copied();
        let (table, n_rows) = match self.source {
            Source::Catalogue => self.catalogue_table(ctx, picked),
            Source::ThrustCurve => self.remote_table(ctx, mount.as_ref()),
        };
        let table_h = (n_rows as f32 * 27.0).clamp(40.0, TABLE_H);

        let delay: Element<'a, ChromeMessage> = match picked {
            Some(choice) => unit_input("Delay", choice.delay, Quantity::Time, &units).label_width(48.0).on_commit(move |d| msg(Message::Delay(d))).into(),
            None => Space::new().boxed(),
        };
        let cancel: Element<'a, ChromeMessage> = action("Cancel", Weight::Secondary).on_press(msg(Message::Cancel)).into();
        let add: Element<'a, ChromeMessage> = match picked {
            Some(_) => action("Add configuration", Weight::Primary).on_press(msg(Message::Add)).into(),
            None => action("Add configuration", Weight::Primary).disabled("Pick a motor").into(),
        };
        let load: Element<'a, ChromeMessage> = action("Load .eng…", Weight::Secondary).on_press(msg(Message::LoadFiles)).into();
        let footer = row![container(delay).width(160), load, Space::new().width(Fill), cancel, add].spacing(SPACE_2).align_y(Center);

        let rule = || container(Space::new()).width(Fill).height(1).style(theme::hairline).boxed();
        let mut body = column![
            container(header)
                .padding([0.0, SPACE_3])
                .height(30)
                .center_y(30)
                .style(|t: &Theme| container::Style { background: Some(Background::Color(theme::tokens(t).bg_100)), ..container::Style::default() })
                .boxed(),
            rule(),
            container(sources).padding([SPACE_2, SPACE_3]).boxed(),
            rule()
        ];
        if let Some(tabs) = tabs {
            body = body.push(container(tabs).padding([SPACE_2, SPACE_3]).boxed()).push(rule());
        }
        let body = body
            .push(container(filters).padding([SPACE_2, SPACE_3 - 2.0]).boxed())
            .push(rule())
            .push(head.boxed())
            .push(rule())
            .push(scrollable(table).direction(theme::thin_scrollbar()).style(theme::scroll).height(table_h).boxed())
            .push(container(footer).padding([SPACE_2, SPACE_3 - 2.0]).boxed());
        let panel = container(body).width(WIDTH).style(theme::overlay);

        // A flat panel on the strip's CONFIG cell, like the menu: no scrim, a click outside closes it, a press inside does not.
        super::floating(mouse_area(panel).on_press(ChromeMessage::Nothing).boxed(), super::Anchor::BottomLeft)
    }

    /// The catalogue table and its row count.
    fn catalogue_table<'a>(&'a self, ctx: &'a Ctx, picked: Option<Choice>) -> (Element<'a, ChromeMessage>, usize) {
        let motors = &ctx.physics.motors().motors;
        let bands = ctx.doc.rocket().stability_bands();
        let rows = self.visible(ctx);
        if rows.is_empty() {
            return (note_row("No motors match", theme::faint), 1);
        }
        let more = rows.len().saturating_sub(MAX_ROWS);
        let mut list = column(rows.iter().take(MAX_ROWS).map(|r| {
            table_row(
                r,
                &motors[r.motor],
                ctx.physics.motor_origin(r.motor),
                picked.is_some_and(|c| c.motor == r.motor),
                bands,
                ctx.units,
                ChromeMessage::Picker,
            )
        }));
        if more > 0 {
            list = list.push(note_row(format!("{more} more · narrow by class or search"), theme::faint));
        }
        (list.boxed(), rows.len().min(MAX_ROWS) + usize::from(more > 0))
    }

    /// The live search's table (results, or a line saying what is happening) and its row count.
    fn remote_table<'a>(&'a self, ctx: &'a Ctx, mount: Option<&MotorMount>) -> (Element<'a, ChromeMessage>, usize) {
        let error = self.download_error.as_ref().map(|e| note_row(e.clone(), err_text));
        let (body, n): (Element<'a, ChromeMessage>, usize) = match &self.remote {
            Remote::Idle => (note_row("Designation (H128W), common name (H128) or maker and class (Cesaroni J)", theme::faint), 1),
            Remote::Searching => (busy_row("Searching ThrustCurve.org…".to_string()), 1),
            Remote::Failed(why) => {
                (column![note_row(why.clone(), err_text), note_row("The catalogue and downloaded motors still work.", theme::faint)].boxed(), 2)
            }
            Remote::Results(found) if found.is_empty() => (note_row("No ThrustCurve.org motors match", theme::faint), 1),
            Remote::Results(found) => {
                let motors = ctx.physics.motors();
                let more = found.len().saturating_sub(MAX_ROWS);
                let mut list = column(found.iter().take(MAX_ROWS).enumerate().map(|(i, m)| {
                    let known = motors.position(&m.id()).map(|index| ctx.physics.motor_origin(index));
                    let downloading = self.downloading.as_deref() == Some(m.motor_id.as_str());
                    let fits = mount.is_none_or(|mount| Self::problem(mount, m.diameter, m.length).is_none());
                    remote_row(i, m, known, downloading, fits, ctx.units)
                }));
                if more > 0 {
                    list = list.push(note_row(format!("{more} more · type more of the designation"), theme::faint));
                }
                (list.boxed(), found.len().min(MAX_ROWS) + usize::from(more > 0))
            }
        };
        match error {
            Some(error) => (column![error, body].boxed(), n + 1),
            None => (body, n),
        }
    }
}

fn to_app(message: Message) -> app::Message {
    app::Message::Chrome(ChromeMessage::Picker(message))
}

/// The typing pause of live search `generation`: answers [`Message::Debounced`] after [`DEBOUNCE`].
pub fn debounce(generation: u64) -> Task<app::Message> {
    app::worker::perform(|| std::thread::sleep(DEBOUNCE), move |_| to_app(Message::Debounced(generation)))
}

/// Live search `generation` for `text` on a worker thread: answers [`Message::Results`].
pub fn search(generation: u64, text: String, api_base: Option<String>) -> Task<app::Message> {
    app::worker::perform(
        move || client(api_base.as_deref()).search_text(&text),
        move |result| to_app(Message::Results(generation, result.unwrap_or_else(|| Err(TcError::Unreachable("the search stopped".into()))))),
    )
}

/// Downloads `motor`'s curve on a worker thread: answers [`Message::Downloaded`].
pub fn download(motor: Box<RemoteMotor>, api_base: Option<String>) -> Task<app::Message> {
    let id = motor.motor_id.clone();
    let fetch = move || {
        let found = client(api_base.as_deref()).download(std::slice::from_ref(&*motor))?;
        found.into_iter().next().map(Box::new).ok_or(TcError::NoData(motor.designation.clone()))
    };
    app::worker::perform(fetch, move |result| {
        to_app(Message::Downloaded(id, result.unwrap_or_else(|| Err(TcError::Unreachable("the download stopped".into())))))
    })
}

/// The colour a margin is shown in against the document's thresholds: `ok` in the stable band, `warn` when marginal or
/// over-stable, `err` when unstable.
pub fn margin_color(t: &Tokens, bands: StabilityBands, margin: f64) -> iced::Color {
    if margin < bands.marginal {
        t.err
    } else if margin < bands.stable || margin > bands.over {
        t.warn
    } else {
        t.ok
    }
}

/// Text in the `err` colour.
fn err_text(t: &Theme) -> text::Style {
    text::Style { color: Some(theme::tokens(t).err), selection: None }
}

/// A one-line note in place of rows, as tall as a row.
fn note_row<'a>(note: impl Into<String>, style: fn(&Theme) -> text::Style) -> Element<'a, ChromeMessage> {
    container(text(note.into()).size(TEXT_BODY).style(style)).padding([0.0, SPACE_3]).width(Fill).height(27).center_y(27).boxed()
}

/// A note with a spinner: something is on its way.
fn busy_row<'a>(note: String) -> Element<'a, ChromeMessage> {
    let line = row![spinner(12.0), text(note).size(TEXT_BODY).style(theme::muted)].spacing(SPACE_2).align_y(Center);
    container(line).padding([0.0, SPACE_3]).width(Fill).height(27).center_y(27).boxed()
}

/// The small tag after a designation: where the motor came from, or that it is out of production.
fn tag(origin: MotorOrigin, motor: &Motor) -> Option<&'static str> {
    match origin {
        MotorOrigin::File => Some("USER"),
        MotorOrigin::Downloaded => Some("TC"),
        MotorOrigin::Catalogue => motor.thrustcurve.as_ref().is_some_and(|i| i.out_of_production()).then_some("OOP"),
    }
}

fn table_row<'a>(
    r: &Row,
    motor: &Motor,
    origin: MotorOrigin,
    picked: bool,
    bands: StabilityBands,
    units: crate::units::UnitSystem,
    msg: fn(Message) -> ChromeMessage,
) -> Element<'a, ChromeMessage> {
    let faint = r.problem.is_some();
    let cell = move |s: String, width: Length| {
        container(text(s).font(MONO).size(ROW_SIZE).style(move |t: &Theme| {
            let tk = theme::tokens(t);
            text::Style { color: Some(if faint { tk.ink_faint } else { tk.ink }), selection: None }
        }))
        .width(width)
    };
    let last: Element<'a, ChromeMessage> = match (r.problem, r.margin) {
        (Some(reason), _) => text(reason).font(MONO).size(ROW_SIZE).style(theme::faint).boxed(),
        (None, Some(margin)) => text(format!("{margin:.2} cal"))
            .font(MONO)
            .size(ROW_SIZE)
            .style(move |t: &Theme| text::Style { color: Some(margin_color(theme::tokens(t), bands, margin)), selection: None })
            .boxed(),
        (None, None) => text("—").font(MONO).size(ROW_SIZE).style(theme::faint).boxed(),
    };
    let mut name = row![text(motor.designation.clone()).font(MONO).size(ROW_SIZE).style(move |t: &Theme| {
        let tk = theme::tokens(t);
        text::Style { color: Some(if faint { tk.ink_faint } else { tk.ink }), selection: None }
    })]
    .spacing(SPACE_2)
    .align_y(Center);
    if let Some(tag) = tag(origin, motor) {
        name = name.push(text(tag).font(MONO).size(HEAD_SIZE).style(theme::faint).boxed());
    }
    let content = row![
        container(name).width(Length::FillPortion(11)),
        cell(ui::with_unit(motor.total_impulse(), Quantity::Impulse, units), Length::FillPortion(10)),
        cell(ui::with_unit(motor.length, Quantity::Length, units), Length::FillPortion(10)),
        cell(ui::with_unit(motor.total_mass, Quantity::Mass, units), Length::FillPortion(10)),
        container(last).width(Length::FillPortion(17)),
    ]
    .align_y(Center);
    let index = r.motor;
    framed_row(content.boxed(), picked, msg(Message::Select(index)))
}

/// One live search result: designation, maker, diameter × length, listed impulse, and its status (availability, already
/// known, downloading, or no curve to download). A result that would not fit the mount is faint.
fn remote_row<'a>(
    index: usize,
    m: &RemoteMotor,
    known: Option<MotorOrigin>,
    downloading: bool,
    fits: bool,
    units: crate::units::UnitSystem,
) -> Element<'a, ChromeMessage> {
    let faint = !fits || m.data_files == 0;
    let cell = move |s: String, width: Length| {
        container(text(s).font(MONO).size(ROW_SIZE).style(move |t: &Theme| {
            let tk = theme::tokens(t);
            text::Style { color: Some(if faint { tk.ink_faint } else { tk.ink }), selection: None }
        }))
        .width(width)
    };
    let size = format!("Ø{} × {}", ui::bare(m.diameter, Quantity::Length, units), ui::bare(m.length, Quantity::Length, units));
    let status: Element<'a, ChromeMessage> = if downloading {
        row![spinner(10.0), text("Downloading").font(MONO).size(ROW_SIZE).style(theme::muted)].spacing(SPACE_2).align_y(Center).boxed()
    } else {
        let (word, style): (String, fn(&Theme) -> text::Style) = match known {
            Some(MotorOrigin::Downloaded) => ("Downloaded".into(), theme::muted),
            Some(_) => ("In catalogue".into(), theme::muted),
            None if m.data_files == 0 => ("No curve".into(), theme::faint),
            None if m.availability.eq_ignore_ascii_case("oop") => ("OOP".into(), theme::faint),
            None => (m.availability.clone(), theme::muted),
        };
        text(word).font(MONO).size(ROW_SIZE).style(style).boxed()
    };
    let content = row![
        cell(m.designation.clone(), Length::FillPortion(11)),
        cell(m.manufacturer.clone(), Length::FillPortion(11)),
        cell(size, Length::FillPortion(12)),
        cell(ui::with_unit(m.total_impulse, Quantity::Impulse, units), Length::FillPortion(10)),
        container(status).width(Length::FillPortion(14)),
    ]
    .align_y(Center);
    framed_row(content.boxed(), false, ChromeMessage::Picker(Message::Pick(index)))
}

/// A clickable table row with the picked edge and tint, and its hairline.
fn framed_row<'a>(content: Element<'a, ChromeMessage>, picked: bool, on_press: ChromeMessage) -> Element<'a, ChromeMessage> {
    let edge = container(Space::new()).width(2).height(Fill).style(move |t: &Theme| container::Style {
        background: Some(Background::Color(if picked { theme::tokens(t).accent } else { iced::Color::TRANSPARENT })),
        ..container::Style::default()
    });
    let body = container(row![edge, container(content).padding([0.0, SPACE_3 - 2.0]).center_y(Fill)]).width(Fill).height(26).style(move |t: &Theme| {
        let tk = theme::tokens(t);
        container::Style { background: picked.then_some(Background::Color(tk.accent_soft)), ..container::Style::default() }
    });
    let rule = container(Space::new()).width(Fill).height(1).style(theme::hairline);
    mouse_area(column![body, rule]).on_press(on_press).interaction(iced::mouse::Interaction::Pointer).boxed()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::App;
    use bowshock_model::{InternalKind, NoseShape, PartKind};

    /// A rocket with a nose, a tube and a 38 mm motor mount.
    fn with_mount(app: &mut App) -> PartId {
        let ctx = app.ctx_mut();
        ctx.doc.edit("Build", |r| {
            r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
            let tube = r.insert(1, PartKind::BodyTube { length: 0.9, diameter: 0.0762 }).unwrap();
            r.attach_internal(tube, 0.5, InternalKind::InnerTube { outer_diameter: 0.040, thickness: 0.001, length: 0.34, motor_mount: true, overhang: 0.0 })
                .unwrap()
        })
    }

    #[test]
    fn the_picker_lists_fitting_motors_first_and_adds_a_configuration() {
        let mut app = App::new();
        assert!(Picker::open(app.ctx()).is_err());
        let mount = with_mount(&mut app);
        app.sync();
        let ctx = app.ctx_mut();
        let mut picker = Picker::open(ctx).unwrap();
        let motors = &ctx.physics.motors().motors;
        assert!(!picker.rows.is_empty());
        // Fitting motors come first and have a margin; a motor wider than the 38 mm bore is flagged.
        let first_problem = picker.rows.iter().position(|r| r.problem.is_some()).unwrap_or(picker.rows.len());
        assert!(picker.rows[..first_problem].iter().all(|r| r.margin.is_some() && motors[r.motor].diameter <= 0.0385));
        assert!(picker.rows[first_problem..].iter().all(|r| r.problem.is_some()));
        // Search and class filters narrow the table.
        picker.update(Message::Search("zzzz".into()), ctx);
        assert!(picker.visible(ctx).is_empty());
        picker.update(Message::Search("cesaroni technology".into()), ctx);
        assert!(!picker.visible(ctx).is_empty() && picker.visible(ctx).iter().all(|r| ctx.physics.motors().motors[r.motor].manufacturer == "Cesaroni"));
        picker.update(Message::Search(String::new()), ctx);
        let fitting = picker.rows[0].motor;
        picker.update(Message::Select(fitting), ctx);
        let before = ctx.doc.rocket().configurations().len();
        assert_eq!(picker.update(Message::Add, ctx), Outcome::Close);
        let rocket = ctx.doc.rocket();
        assert_eq!(rocket.configurations().len(), before + 1);
        let slot = rocket.active_configuration().slot(mount).unwrap();
        let reference = slot.motor.as_ref().unwrap();
        assert_eq!(reference.designation, ctx.physics.motors().motors[fitting].designation);
        // The configuration names the exact curve.
        assert_eq!(reference.id, ctx.physics.motors().motors[fitting].id);
    }

    fn remote(id: &str, designation: &str) -> RemoteMotor {
        RemoteMotor {
            motor_id: id.into(),
            designation: designation.into(),
            manufacturer: "AeroTech".into(),
            diameter: 0.038,
            length: 0.25,
            total_impulse: 600.0,
            availability: "regular".into(),
            data_files: 1,
            ..Default::default()
        }
    }

    #[test]
    fn live_search_debounces_and_keeps_only_the_latest_answer() {
        let mut app = App::new();
        with_mount(&mut app);
        app.sync();
        let ctx = app.ctx_mut();
        let mut picker = Picker::open(ctx).unwrap();
        picker.update(Message::Source(Source::ThrustCurve), ctx);
        assert_eq!(picker.update(Message::Search("I2".into()), ctx), Outcome::Debounce(1));
        assert_eq!(picker.update(Message::Search("I28".into()), ctx), Outcome::Debounce(2));
        assert!(picker.busy());
        // The first pause is stale; the second searches.
        assert_eq!(picker.update(Message::Debounced(1), ctx), Outcome::Stay);
        assert_eq!(picker.update(Message::Debounced(2), ctx), Outcome::Search(2, "I28".into()));
        // A late answer to an older search is dropped.
        picker.update(Message::Results(1, Ok(vec![remote("a", "OLD")])), ctx);
        assert_eq!(picker.remote, Remote::Searching);
        picker.update(Message::Results(2, Ok(vec![remote("b", "I284W")])), ctx);
        assert!(matches!(&picker.remote, Remote::Results(r) if r[0].designation == "I284W"));
        assert!(!picker.busy());
        // The catalogue's own search text is untouched.
        assert!(picker.search.is_empty());
        // Offline: a clear state, and the catalogue still lists motors.
        picker.update(Message::Search("H128".into()), ctx);
        picker.update(Message::Results(3, Err(TcError::Unreachable("dns error".into()))), ctx);
        assert_eq!(picker.remote, Remote::Failed("ThrustCurve unreachable".into()));
        picker.update(Message::Source(Source::Catalogue), ctx);
        assert!(!picker.visible(ctx).is_empty());
        // Clearing the text goes back to the hint, without a request.
        picker.update(Message::Source(Source::ThrustCurve), ctx);
        assert_eq!(picker.update(Message::Search("  ".into()), ctx), Outcome::Stay);
        assert_eq!(picker.remote, Remote::Idle);
    }

    #[test]
    fn picking_a_result_downloads_caches_and_selects_it() {
        let mut app = App::new();
        let mount = with_mount(&mut app);
        app.sync();
        let ctx = app.ctx_mut();
        let mut picker = Picker::open(ctx).unwrap();
        picker.update(Message::Source(Source::ThrustCurve), ctx);
        picker.update(Message::Search("I284W".into()), ctx);
        let i284 = ctx.physics.motors().find("I284W").unwrap().clone();
        let at = ctx.physics.motors().position(&i284.id).unwrap();
        let known = RemoteMotor { motor_id: i284.thrustcurve_id().unwrap().into(), ..remote("", "I284W") };
        let new = remote("ffffffffffffffffffffffff", "I999Z");
        picker.update(Message::Results(1, Ok(vec![known.clone(), new.clone()])), ctx);
        // A catalogue motor is fetched fresh; the download replaces the snapshot's copy in place and is selected.
        assert_eq!(picker.update(Message::Pick(0), ctx), Outcome::Download(Box::new(known.clone())));
        let before = ctx.physics.motors().len();
        picker.update(Message::Downloaded(known.motor_id.clone(), Ok(Box::new(i284.clone()))), ctx);
        assert_eq!(ctx.physics.motors().len(), before);
        assert_eq!(ctx.physics.motor_origin(at), MotorOrigin::Downloaded);
        assert_eq!((picker.source, picker.choices[&mount].motor), (Source::Catalogue, at));
        // Picked again, the cached copy is selected without a request.
        picker.update(Message::Source(Source::ThrustCurve), ctx);
        picker.update(Message::Select(0), ctx);
        assert_eq!(picker.update(Message::Pick(0), ctx), Outcome::Stay);
        assert_eq!(picker.choices[&mount].motor, at);
        // A motor the catalogue lacks is downloaded and added.
        picker.update(Message::Source(Source::ThrustCurve), ctx);
        assert_eq!(picker.update(Message::Pick(1), ctx), Outcome::Download(Box::new(new.clone())));
        assert!(picker.busy());
        let mut motor = i284.clone();
        motor.id = new.id();
        motor.designation = "I999Z".into();
        picker.update(Message::Downloaded(new.motor_id.clone(), Ok(Box::new(motor))), ctx);
        assert!(!picker.busy());
        assert_eq!(ctx.physics.motors().len(), before + 1);
        let index = ctx.physics.motors().position(&new.id()).unwrap();
        assert_eq!(ctx.physics.motor_origin(index), MotorOrigin::Downloaded);
        assert_eq!((picker.source, picker.search.as_str()), (Source::Catalogue, "I999Z"));
        assert_eq!(picker.choices[&mount].motor, index);
        assert!(picker.visible(ctx).iter().any(|r| r.motor == index && r.margin.is_some()));
        // A failed download says why and leaves the selection alone.
        picker.update(Message::Source(Source::ThrustCurve), ctx);
        picker.update(Message::Downloaded("x".into(), Err(TcError::Unreachable("timeout".into()))), ctx);
        assert_eq!(picker.download_error.as_deref(), Some("ThrustCurve unreachable"));
        assert_eq!(picker.choices[&mount].motor, index);
    }
}
