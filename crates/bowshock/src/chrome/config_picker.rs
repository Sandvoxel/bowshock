//! The motor picker (design/components/ConfigPicker): the modal that builds a new configuration. The header states the mount it
//! filters by, the table lists every motor with the stability margin the rocket would have with it, motors that do not fit stay
//! listed faint with the reason, and a staged rocket gets one tab per stage.

use std::collections::BTreeMap;

use bowshock_model::StabilityBands;
use bowshock_model::{MotorMount, MotorRef, PartId};
use bowshock_motors::Motor;
use iced::widget::{button, column, container, mouse_area, row, scrollable, text, text_input, Space};
use iced::{Background, Center, Element, Fill, Length, Theme, Widget};

use super::Message as ChromeMessage;
use crate::app::{ui, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{Tokens, SPACE_2, SPACE_3, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::button::{button as action, Weight};
use crate::widgets::segmented::segmented;
use crate::widgets::unit_input::unit_input;

/// Tolerance for a motor wider or longer than its mount (the bridge's own).
const FIT_TOL: f64 = 0.5e-3;
/// Width of the picker panel.
const WIDTH: f32 = 560.0;
/// Height of the scrolling table.
const TABLE_H: f32 = 260.0;
/// Table text: the kit's 11.5 px mono; column heads are 10 px.
const ROW_SIZE: f32 = 11.5;
const HEAD_SIZE: f32 = 10.0;
/// Labels of the stage tabs (a rocket with more stages than this does not exist in practice).
const STAGE_LABELS: [&str; 6] = ["Stage 1", "Stage 2", "Stage 3", "Stage 4", "Stage 5", "Stage 6"];

/// The picker's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// The search text changed.
    Search(String),
    /// An impulse class was toggled in the filter.
    ToggleClass(char),
    /// A stage tab (model stage index, from 1).
    Tab(usize),
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
}

/// What the chrome should do after a picker message.
#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Keep the picker open.
    Stay,
    /// Close it.
    Close,
    /// Open the file dialog for the user's own `.eng` files.
    LoadFiles,
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
}

impl Picker {
    /// Opens the picker on stage 1, starting from the active configuration's motors; `Err` says why it cannot open.
    pub fn open(ctx: &Ctx) -> Result<Picker, &'static str> {
        let rocket = ctx.doc.rocket();
        let mounts = rocket.motor_mounts();
        if mounts.is_empty() {
            return Err("Add a motor mount to the rocket first");
        }
        let motors = &ctx.physics.motors().motors;
        let mut choices = BTreeMap::new();
        for slot in &rocket.active_configuration().motors {
            let Some(reference) = &slot.motor else { continue };
            if let Some(motor) = motors.iter().position(|m| m.designation == reference.designation && m.manufacturer == reference.manufacturer) {
                choices.insert(slot.mount, Choice { motor, delay: if slot.delay.is_finite() { slot.delay } else { 0.0 } });
            }
        }
        let first = ctx.physics.stages().first().map_or(1, |s| s.index);
        let mut picker = Picker { stage: first, search: String::new(), classes: Vec::new(), choices, rows: Vec::new() };
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

    /// Recomputes the table for the current stage: fit, and the margin with each fitting motor.
    fn rebuild(&mut self, ctx: &Ctx) {
        self.rows.clear();
        let Some(mount) = self.mount(ctx) else {
            return;
        };
        let rocket = ctx.doc.rocket();
        let reach = mount.length + mount.overhang;
        for (index, motor) in ctx.physics.motors().motors.iter().enumerate() {
            let problem = if motor.diameter > mount.inner_diameter + FIT_TOL {
                Some("Too wide for mount")
            } else if motor.length > reach + FIT_TOL {
                Some("Too long for mount")
            } else {
                None
            };
            let margin = problem.is_none().then(|| ctx.physics.margin_with_motor(rocket, mount.id, motor)).flatten();
            self.rows.push(Row { motor: index, problem, margin });
        }
        // Motors that fit first, then the ones a longer mount would take, then the ones a wider mount would; by impulse.
        let motors = &ctx.physics.motors().motors;
        let rank = |r: &Row| match r.problem {
            None => 0,
            Some("Too long for mount") => 1,
            Some(_) => 2,
        };
        self.rows.sort_by(|a, b| rank(a).cmp(&rank(b)).then(motors[a.motor].total_impulse().total_cmp(&motors[b.motor].total_impulse())));
    }

    /// Handles a picker message.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Outcome {
        match message {
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
        }
        Outcome::Stay
    }

    /// Adds the configuration and makes it current. It starts as a copy of the active one (its motors, staging and recovery
    /// timing), without the ballast, which belongs to the motor it balanced; stages with no pick keep what the copy has.
    fn add(&self, ctx: &mut Ctx) {
        let motors: Vec<(PartId, MotorRef, f64)> = self
            .choices
            .iter()
            .map(|(&mount, c)| {
                let m = &ctx.physics.motors().motors[c.motor];
                (mount, MotorRef::new(m.designation.clone(), m.manufacturer.clone()), c.delay)
            })
            .collect();
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
                (self.classes.is_empty() || self.classes.contains(&m.impulse_class()))
                    && (needle.is_empty() || m.designation.to_lowercase().contains(&needle) || m.manufacturer.to_lowercase().contains(&needle))
            })
            .collect()
    }

    /// The modal: a scrim that closes it and the panel.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, ChromeMessage> {
        let msg = ChromeMessage::Picker;
        let units = ctx.units;
        let motors = &ctx.physics.motors().motors;
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

        let stages = ctx.physics.stages();
        let tabs: Option<Element<'a, ChromeMessage>> = (stages.len() > 1).then(|| {
            let options: Vec<(usize, &str)> = stages.iter().map(|s| (s.index, STAGE_LABELS[(s.index - 1).min(STAGE_LABELS.len() - 1)])).collect();
            segmented(&options, self.stage, |s| msg(Message::Tab(s)))
        });

        let search = text_input("Search designation or maker", &self.search)
            .on_input(move |s| msg(Message::Search(s)))
            .size(TEXT_BODY)
            .padding([4.0, SPACE_2])
            .style(theme::input);
        let mut classes: Vec<char> = self.rows.iter().filter(|r| r.problem != Some("Too wide for mount")).map(|r| motors[r.motor].impulse_class()).collect();
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
        let filters = row![container(search).width(Fill), container(class_buttons).padding(1).style(theme::segmented_frame)].spacing(SPACE_2).align_y(Center);

        let column_head = |name: &'static str, width: Length| container(text(name).font(MONO).size(HEAD_SIZE).style(theme::faint)).width(width);
        let head = row![
            column_head("MOTOR", Length::FillPortion(11)),
            column_head("IMPULSE", Length::FillPortion(10)),
            column_head("LENGTH", Length::FillPortion(10)),
            column_head("MASS", Length::FillPortion(10)),
            column_head("WITH THIS MOTOR", Length::FillPortion(17)),
        ]
        .padding([0.0, SPACE_3 - 2.0])
        .align_y(Center);
        let head = container(head).height(26).center_y(26);

        let picked = mount.as_ref().and_then(|m| self.choices.get(&m.id)).copied();
        let rows = self.visible(ctx);
        let table_h = (rows.len() as f32 * 27.0).clamp(40.0, TABLE_H);
        let bands = ctx.doc.rocket().stability_bands();
        let table: Element<'a, ChromeMessage> = if rows.is_empty() {
            container(text("No motors match").size(TEXT_BODY).style(theme::faint)).padding(SPACE_3).boxed()
        } else {
            column(
                rows.into_iter()
                    .map(|r| table_row(r, &motors[r.motor], ctx.physics.is_user_motor(r.motor), picked.is_some_and(|c| c.motor == r.motor), bands, units, msg)),
            )
            .boxed()
        };

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
            .push(scrollable(table).height(table_h).boxed())
            .push(container(footer).padding([SPACE_2, SPACE_3 - 2.0]).boxed());
        let panel = container(body).width(WIDTH).style(theme::overlay);

        // The scrim closes the picker; the panel swallows presses so that clicking in it does not.
        let scrim = mouse_area(container(Space::new()).width(Fill).height(Fill).style(scrim_style)).on_press(msg(Message::Cancel));
        iced::widget::stack![scrim, container(mouse_area(panel).on_press(ChromeMessage::Nothing)).center(Fill)].boxed()
    }
}

fn scrim_style(theme: &Theme) -> container::Style {
    container::Style { background: Some(Background::Color(theme::tokens(theme).bg_000.scale_alpha(0.55))), ..container::Style::default() }
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

fn table_row<'a>(
    r: &Row,
    motor: &Motor,
    user: bool,
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
    if user {
        name = name.push(text("USER").font(MONO).size(HEAD_SIZE).style(theme::faint).boxed());
    }
    let content = row![
        container(name).width(Length::FillPortion(11)),
        cell(ui::with_unit(motor.total_impulse(), Quantity::Impulse, units), Length::FillPortion(10)),
        cell(ui::with_unit(motor.length, Quantity::Length, units), Length::FillPortion(10)),
        cell(ui::with_unit(motor.total_mass, Quantity::Mass, units), Length::FillPortion(10)),
        container(last).width(Length::FillPortion(17)),
    ]
    .align_y(Center);
    let edge = container(Space::new()).width(2).height(Fill).style(move |t: &Theme| container::Style {
        background: Some(Background::Color(if picked { theme::tokens(t).accent } else { iced::Color::TRANSPARENT })),
        ..container::Style::default()
    });
    let body = container(row![edge, container(content).padding([0.0, SPACE_3 - 2.0]).center_y(Fill)]).width(Fill).height(26).style(move |t: &Theme| {
        let tk = theme::tokens(t);
        container::Style { background: picked.then_some(Background::Color(tk.accent_soft)), ..container::Style::default() }
    });
    let index = r.motor;
    let rule = container(Space::new()).width(Fill).height(1).style(theme::hairline);
    mouse_area(column![body, rule]).on_press(msg(Message::Select(index))).interaction(iced::mouse::Interaction::Pointer).boxed()
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
        picker.update(Message::Search(String::new()), ctx);
        let fitting = picker.rows[0].motor;
        picker.update(Message::Select(fitting), ctx);
        let before = ctx.doc.rocket().configurations().len();
        assert_eq!(picker.update(Message::Add, ctx), Outcome::Close);
        let rocket = ctx.doc.rocket();
        assert_eq!(rocket.configurations().len(), before + 1);
        let slot = rocket.active_configuration().slot(mount).unwrap();
        assert_eq!(slot.motor.as_ref().unwrap().designation, ctx.physics.motors().motors[fitting].designation);
    }
}
