//! The window chrome shared by every mode: the [`top_bar`], the [`status_strip`] and the overlays that open from them (the
//! configuration menu over the foot of the parts bin, the issues list over the strip's right end, the motor picker modal).
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is the chrome's own state (which menu is open, the rename buffer, the
//! picker), [`Message`] its messages; the top bar's buttons send `app::Message` directly (undo, mode, units, run), everything
//! that opens a menu or touches configurations sends [`Message`] wrapped in `app::Message::Chrome`.

pub mod config_picker;
mod status_strip;
#[cfg(test)]
mod tests;
mod top_bar;

use bowshock_model::{ConfigId, PartId};
use bowshock_sim::from_model::{Level, Phase};
use iced::widget::{button, column, container, mouse_area, row, stack, text, text_input, Space};
use iced::{Background, Center, Element, Fill, Padding, Task, Theme, Widget};

use crate::app::{self, ui, Ctx, Selection};
use crate::theme::{self, MONO};
use crate::tokens::{BIN_W, RAIL_W, SPACE_1, SPACE_2, SPACE_3, STATUSBAR_H, TEXT_BODY, TOPBAR_H};
use crate::units::Quantity;
use crate::widgets::glyph::{self, Mark, Tone};
use crate::widgets::unit_input::unit_input;
pub use status_strip::status_strip;
pub use top_bar::top_bar;

/// Id of the rename field, so it can take focus when it appears.
const RENAME_ID: iced::widget::Id = iced::widget::Id::new("rename-configuration");
/// Height of a menu row.
const ROW_H: f32 = 26.0;
/// Menu text: the kit's 12.5 px rows with 11.5 px mono hints, 10 px side padding.
const MENU_SIZE: f32 = 12.5;
const MENU_HINT: f32 = 11.5;
const MENU_PAD: f32 = 10.0;
/// Width of the Simulate button's menu.
const RUN_MENU_W: f32 = 240.0;
/// Width of the issues list.
const ISSUES_W: f32 = 420.0;

/// Which menu is open from the status strip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Menu {
    Closed,
    Config,
    Issues,
    Run,
}

/// The chrome's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// The CONFIG cell: open or close the configuration menu.
    ToggleConfigMenu,
    /// The warnings chip: open or close the issues list.
    ToggleIssues,
    /// The Simulate button's caret: open or close its menu.
    ToggleRunMenu,
    /// "Run current configuration" in that menu.
    RunCurrent,
    /// "Run all configurations" in that menu.
    RunAll,
    /// Close whatever is open (Escape, a click outside).
    Dismiss,
    /// Make this configuration current.
    SelectConfig(ConfigId),
    /// Copy the current configuration (timing and motors) and make the copy current.
    DuplicateConfig,
    /// The ballast mass of the current configuration (kg); zero removes it.
    BallastMass(f64),
    /// The station of the current configuration's ballast, from the nose tip (m).
    BallastStation(f64),
    /// Delete this configuration (right-click on its row); the last one stays.
    DeleteConfig(ConfigId),
    /// "Compare all in lanes".
    ToggleCompare,
    /// Start renaming the current configuration (`F2`).
    BeginRename,
    /// The rename field changed.
    RenameInput(String),
    /// Enter in the rename field.
    CommitRename,
    /// Open the motor picker for a new configuration (`Ctrl/Cmd+L`).
    NewConfiguration,
    /// The Phase switch.
    Phase(Phase),
    /// A line of the issues list was clicked: select the part it is about and, when the warning names one, its field.
    SelectIssue(Option<PartId>, Option<bowshock_model::Dim>),
    /// A message of the motor picker.
    Picker(config_picker::Message),
    /// The CG cell: show or hide the CG marker in the viewport and lanes.
    ToggleCg,
    /// The CP cell: show or hide the CP marker in the viewport and lanes.
    ToggleCp,
    /// The files chosen in the "Load .eng…" dialog (empty when it was cancelled).
    MotorFiles(Vec<std::path::PathBuf>),
    /// Swallows a press (so clicking inside a modal panel does not close it).
    Nothing,
}

/// The chrome's state.
pub struct State {
    menu: Menu,
    /// The name being typed while renaming the current configuration.
    rename: Option<String>,
    picker: Option<config_picker::Picker>,
}

impl State {
    /// Everything closed.
    pub fn new() -> Self {
        State { menu: Menu::Closed, rename: None, picker: None }
    }

    /// Whether the configuration menu is open (the CONFIG cell lights its caret).
    fn menu_is_config(&self) -> bool {
        self.menu == Menu::Config
    }

    /// Closes menus and the picker (Escape, switching mode).
    pub fn dismiss(&mut self) {
        self.menu = Menu::Closed;
        self.rename = None;
        self.picker = None;
    }

    /// Handles one of the chrome's messages.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::ToggleConfigMenu => {
                self.menu = if self.menu == Menu::Config { Menu::Closed } else { Menu::Config };
                self.rename = None;
            }
            Message::ToggleIssues => self.menu = if self.menu == Menu::Issues { Menu::Closed } else { Menu::Issues },
            Message::ToggleRunMenu => self.menu = if self.menu == Menu::Run { Menu::Closed } else { Menu::Run },
            Message::RunCurrent => {
                self.dismiss();
                return Task::done(app::Message::Run);
            }
            Message::RunAll => {
                self.dismiss();
                return Task::done(app::Message::RunAll);
            }
            Message::Dismiss => self.dismiss(),
            Message::SelectConfig(id) => {
                ctx.doc.edit("Switch configuration", |r| r.select_configuration(id));
                self.dismiss();
            }
            Message::DuplicateConfig => {
                ctx.doc.edit("Duplicate configuration", |r| {
                    if let Some(copy) = r.duplicate_configuration(r.active_config_id()) {
                        r.select_configuration(copy);
                    }
                });
                self.dismiss();
            }
            Message::BallastMass(mass) => {
                ctx.doc.edit("Edit ballast", |r| {
                    // The first ballast goes in the nose, where it helps stability most.
                    let station = r.active_configuration().ballast.map_or(0.1 * r.length(), |b| b.station);
                    let config = r.active_config_id();
                    let _ = r.set_ballast(config, Some(bowshock_model::Ballast { mass, station }));
                });
            }
            Message::BallastStation(station) => {
                ctx.doc.edit("Move ballast", |r| {
                    let config = r.active_config_id();
                    if let Some(b) = r.active_configuration().ballast {
                        let _ = r.set_ballast(config, Some(bowshock_model::Ballast { station, ..b }));
                    }
                });
            }
            Message::DeleteConfig(id) => {
                if let Err(e) = ctx.doc.edit("Delete configuration", |r| r.remove_configuration(id).map(|_| ())) {
                    ctx.notify(format!("Cannot delete: {e}"));
                }
            }
            Message::ToggleCompare => ctx.compare_configs = !ctx.compare_configs,
            Message::BeginRename => {
                let rocket = ctx.doc.rocket();
                self.rename = Some(rocket.configuration_name(rocket.active_config_id()).unwrap_or_default());
                self.menu = Menu::Config;
                return iced::widget::operation::focus(RENAME_ID);
            }
            Message::RenameInput(name) => self.rename = Some(name),
            Message::CommitRename => {
                if let Some(name) = self.rename.take() {
                    ctx.doc.edit("Rename configuration", |r| r.rename_configuration(r.active_config_id(), name.trim()));
                }
                self.dismiss();
            }
            Message::NewConfiguration => {
                self.dismiss();
                match config_picker::Picker::open(ctx) {
                    Ok(picker) => self.picker = Some(picker),
                    Err(why) => ctx.notify(why),
                }
            }
            Message::Phase(phase) => ctx.physics.set_phase(phase),
            Message::SelectIssue(part, field) => {
                self.dismiss();
                if let Some(id) = part {
                    ctx.selection = Selection::Part(id);
                    // The inspector scrolls to the field that fixes the warning and focuses it, once it shows the part.
                    if let Some(dim) = field {
                        return Task::done(app::Message::Inspector(crate::build::inspector::Message::Focus(dim)));
                    }
                }
            }
            Message::Picker(m) => {
                if let Some(picker) = self.picker.as_mut() {
                    match picker.update(m, ctx) {
                        config_picker::Outcome::Close => self.picker = None,
                        config_picker::Outcome::Stay => {}
                        config_picker::Outcome::LoadFiles => return pick_motor_files(),
                    }
                }
            }
            Message::MotorFiles(paths) => {
                if !paths.is_empty() {
                    let notice = ctx.physics.load_user_motors(&paths);
                    ctx.notify(notice);
                    if let Some(picker) = self.picker.as_mut() {
                        picker.refresh(ctx);
                    }
                }
            }
            Message::ToggleCg => ctx.show_cg = !ctx.show_cg,
            Message::ToggleCp => ctx.show_cp = !ctx.show_cp,
            Message::Nothing => {}
        }
        Task::none()
    }

    /// What floats over the screen: the motor picker, the configuration menu or the issues list; `None` when all are closed.
    pub fn overlay<'a>(&'a self, ctx: &'a Ctx) -> Option<Element<'a, app::Message>> {
        let overlay: Element<'a, Message> = if let Some(picker) = &self.picker {
            picker.view(ctx)
        } else {
            match self.menu {
                Menu::Closed => return None,
                Menu::Config => self.config_menu(ctx),
                Menu::Issues => issues_list(ctx),
                Menu::Run => run_menu(ctx),
            }
        };
        Some(overlay.map(app::Message::Chrome).boxed())
    }

    /// The configuration menu: one row per configuration (name, margin), then the commands. It opens upward from the CONFIG
    /// cell, as wide as the parts bin.
    fn config_menu<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let rocket = ctx.doc.rocket();
        let active = rocket.active_config_id();
        let bands = rocket.stability_bands();
        let mut rows = column![];
        for config in rocket.configurations() {
            let id = config.id;
            let name = rocket.configuration_name(id).unwrap_or_default();
            let margin: Element<'a, Message> = match ctx.physics.config_margin(id) {
                Some(m) => text(format!("{m:.2} cal"))
                    .font(MONO)
                    .size(MENU_HINT)
                    .style(move |t: &Theme| text::Style { color: Some(config_picker::margin_color(theme::tokens(t), bands, m)), selection: None })
                    .boxed(),
                None => text("—").font(MONO).size(MENU_HINT).style(theme::faint).boxed(),
            };
            let current = id == active;
            let ballast: Element<'a, Message> = match config.ballast {
                Some(b) => text(format!("+{}", ui::with_unit(b.mass, Quantity::Mass, ctx.units))).font(MONO).size(MENU_HINT).style(theme::faint).boxed(),
                None => Space::new().boxed(),
            };
            let line = row![text(name).size(MENU_SIZE), Space::new().width(Fill), ballast, Space::new().width(SPACE_3), margin].align_y(Center);
            let item = button(line).width(Fill).height(ROW_H).padding([0.0, MENU_PAD]).style(menu_item(current, false)).on_press(Message::SelectConfig(id));
            // Deleting is on the row's right-click menu.
            rows = rows.push(mouse_area(item).on_right_press(Message::DeleteConfig(id)).boxed());
            rows = rows.push(container(Space::new()).width(Fill).height(1).style(theme::hairline).boxed());
        }

        let command = |label: &'static str, hint: &'static str, accent: bool, message: Message| -> Element<'a, Message> {
            let line = row![text(label).size(MENU_SIZE), Space::new().width(Fill), text(hint).font(MONO).size(MENU_HINT).style(theme::muted)].align_y(Center);
            let item = button(line).width(Fill).height(ROW_H).padding([0.0, MENU_PAD]).style(menu_item(false, accent)).on_press(message);
            column![item, container(Space::new()).width(Fill).height(1).style(theme::hairline)].boxed()
        };
        let compare = if ctx.compare_configs { "on" } else { "off" };
        let renaming: Element<'a, Message> = match &self.rename {
            Some(name) => container(
                text_input("Configuration name", name)
                    .id(RENAME_ID)
                    .on_input(Message::RenameInput)
                    .on_submit(Message::CommitRename)
                    .size(TEXT_BODY)
                    .padding([3.0, SPACE_2])
                    .style(theme::input),
            )
            .padding([SPACE_1, SPACE_3])
            .boxed(),
            None => command("Rename…", "F2", false, Message::BeginRename),
        };
        // Ballast of the current configuration: a mass and, once it has one, where it sits.
        let units = ctx.units;
        let ballast = rocket.active_configuration().ballast;
        let mass_field: Element<'a, Message> =
            unit_input("Ballast", ballast.map_or(0.0, |b| b.mass), Quantity::Mass, &units).label_width(56.0).on_commit(Message::BallastMass).into();
        let mut ballast_rows = column![mass_field].spacing(SPACE_1);
        if let Some(b) = ballast {
            let station: Element<'a, Message> =
                unit_input("Station", b.station, Quantity::Length, &units).label_width(56.0).on_commit(Message::BallastStation).into();
            ballast_rows = ballast_rows.push(station);
        }
        let menu = column![
            rows,
            command("New configuration…", if cfg!(target_os = "macos") { "⌘L" } else { "Ctrl L" }, true, Message::NewConfiguration),
            command("Duplicate current", "", false, Message::DuplicateConfig),
            renaming,
            container(ballast_rows).padding([SPACE_1, SPACE_3]),
            command("Compare all in lanes", compare, false, Message::ToggleCompare),
        ];
        let panel = container(mouse_area(menu).on_press(Message::Nothing)).width(RAIL_W + BIN_W).style(theme::overlay);
        floating(panel.boxed(), Anchor::BottomLeft)
    }
}

/// The Simulate button's menu: run the current configuration or all of them (ConfigPicker, "In other modes"). It hangs from the
/// top bar's right end.
fn run_menu<'a>(ctx: &'a Ctx) -> Element<'a, Message> {
    let count = ctx.doc.rocket().configurations().len();
    let item = |label: &'static str, hint: String, message: Message| -> Element<'a, Message> {
        let line = row![text(label).size(MENU_SIZE), Space::new().width(Fill), text(hint).font(MONO).size(MENU_HINT).style(theme::muted)].align_y(Center);
        let item = button(line).width(Fill).height(ROW_H).padding([0.0, MENU_PAD]).style(menu_item(false, false)).on_press(message);
        column![item, container(Space::new()).width(Fill).height(1).style(theme::hairline)].boxed()
    };
    let menu =
        column![item("Run current configuration", "F5".to_string(), Message::RunCurrent), item("Run all configurations", format!("{count}"), Message::RunAll),];
    let panel = container(mouse_area(menu).on_press(Message::Nothing)).width(RUN_MENU_W).style(theme::overlay);
    floating(panel.boxed(), Anchor::TopRight)
}

/// Where an overlay panel sits: above the status strip, or below the top bar.
enum Anchor {
    BottomLeft,
    BottomRight,
    TopRight,
}

/// `panel` floated over the screen at `anchor`, against the status strip or the top bar, on a transparent layer that closes it
/// when clicked.
fn floating<'a>(panel: Element<'a, Message>, anchor: Anchor) -> Element<'a, Message> {
    let scrim = mouse_area(Space::new().width(Fill).height(Fill)).on_press(Message::Dismiss);
    let pad = match anchor {
        Anchor::TopRight => Padding { top: TOPBAR_H, ..Padding::ZERO },
        _ => Padding { bottom: STATUSBAR_H, ..Padding::ZERO },
    };
    let placed = container(panel).width(Fill).height(Fill).padding(pad);
    let placed = match anchor {
        Anchor::BottomLeft => placed.align_left(Fill).align_bottom(Fill),
        Anchor::BottomRight => placed.align_right(Fill).align_bottom(Fill),
        Anchor::TopRight => placed.align_right(Fill).align_top(Fill),
    };
    stack![scrim, placed].boxed()
}

/// A menu row: `bg-300` with an `accent` inside edge when current, `bg-400` on hover, the accent colour for the primary command.
fn menu_item(current: bool, accent: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |theme, status| {
        let t = theme::tokens(theme);
        let hovered = matches!(status, button::Status::Hovered | button::Status::Pressed);
        let background = if current {
            Some(Background::Color(t.accent_soft))
        } else if hovered {
            Some(Background::Color(t.bg_400))
        } else {
            None
        };
        button::Style { background, text_color: if accent { t.accent } else { t.ink }, border: iced::Border::default(), ..button::Style::default() }
    }
}

/// The issues list: every warning and error of the active configuration; a line with a part selects it when clicked.
fn issues_list<'a>(ctx: &'a Ctx) -> Element<'a, Message> {
    let header = container(ui::label("ISSUES")).padding([SPACE_2, SPACE_3]);
    let lines = column(ctx.physics.issues().map(|w| {
        let (mark, tone) = if w.level == Level::Err { (Mark::Error, Tone::Err) } else { (Mark::Warning, Tone::Warn) };
        let line = row![glyph::icon(mark, tone, 13.0), text(w.message.as_str()).size(TEXT_BODY)].spacing(SPACE_2).align_y(Center);
        let item = button(line).width(Fill).padding([SPACE_1 + 2.0, SPACE_3]).style(menu_item(false, false));
        let item: Element<'a, Message> = item.on_press(Message::SelectIssue(w.part, w.field)).boxed();
        item
    }));
    let panel = container(mouse_area(column![header, lines]).on_press(Message::Nothing)).width(ISSUES_W).style(theme::overlay);
    floating(panel.boxed(), Anchor::BottomRight)
}

/// The "Load .eng…" dialog (multi-select); the answer arrives as [`Message::MotorFiles`].
fn pick_motor_files() -> Task<app::Message> {
    Task::perform(
        async {
            let files = rfd::AsyncFileDialog::new().set_title("Load motor files").add_filter("RASP motor file", &["eng"]).pick_files().await;
            files.map(|f| f.into_iter().map(|h| h.path().to_path_buf()).collect()).unwrap_or_default()
        },
        |paths| app::Message::Chrome(Message::MotorFiles(paths)),
    )
}
