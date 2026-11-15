//! The parts bin: the rail of seven categories, the ruled rows of each (silhouette, name, default size), the search across every
//! category, "size to selection" and the symmetry footer (design/components/PartsBin, Symmetry).
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`] and [`State::on_key`] with the shared [`Ctx`].
//!
//! * A press on a row that moves past [`DRAG_SLOP`] starts a drag (axial, radial and internal rows) through `crate::app::drag`;
//!   one that does not is a click: it attaches the part aft of the selection or at the best node ([`items`]). A second click on
//!   the same row within [`DOUBLE_CLICK`] does not add another part: it focuses the inspector's first field
//!   ([`INSPECTOR_FIRST_FIELD`]).
//! * Motor rows list the motors that fit the selected (else first) motor mount; a click sets the active configuration's motor.
//!   Preset rows add a saved assembly ([`presets`]); a right click on a user preset deletes it.
//! * Keys: `/` focuses search, `X` / `Shift+X` cycle the symmetry count, and `1` to `7` switch category **only while the cursor is
//!   over the bin**, so the viewport keeps `1` to `4` for its inside views everywhere else.

pub mod items;
pub mod presets;
mod rows;

use std::time::{Duration, Instant};

use bowshock_model::{InternalKind, PartKind};
use iced::widget::canvas::{self, Frame, Geometry, Path, Stroke};
use iced::widget::{button, canvas as canvas_widget, checkbox, column, container, mouse_area, row, scrollable, stack, text, text_input, Space};
use iced::{keyboard, mouse, Background, Element, Fill, Point, Rectangle, Renderer, Task, Theme, Widget};

// Re-exported for the surfaces that add parts elsewhere (Station Lanes menus).
use items::Item;
#[allow(unused_imports)]
pub use items::{airframe_items, axial_catalogue, internal_catalogue, mass_catalogue, radial_catalogue, recovery_catalogue, sized, structure_catalogue, RowId};
use presets::PresetEntry;
use rows::{RowState, RowView};

use crate::app::ui::{label, mm, radial_size_label, size_label};
use crate::app::{self, drag, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{RAIL_W, SPACE_1, SPACE_2, TEXT_BODY, TEXT_LABEL};
use crate::widgets::icons::{rail_button, RailIcon};
use crate::widgets::segmented::{segmented, segmented_fill};

/// Movement in px after pressing a row beyond which the press is a drag rather than a click.
const DRAG_SLOP: f32 = 4.0;
/// Two clicks on a row this close together are a double click.
const DOUBLE_CLICK: Duration = Duration::from_millis(400);
/// The id the inspector gives its first field so a double click on a bin row can focus it (`iced::widget::operation::focus`).
pub const INSPECTOR_FIRST_FIELD: &str = "inspector-first-field";
/// The id of the search field.
const SEARCH_ID: &str = "parts-search";

/// Symmetry counts for radial parts (design/components/Symmetry).
pub const SYMMETRY: [(u8, &str); 6] = [(1, "1"), (2, "2"), (3, "3"), (4, "4"), (6, "6"), (8, "8")];

/// Angle snap choices for radial parts, in degrees.
pub const ANGLE_SNAP: [(u8, &str); 3] = [(0, "off"), (15, "15°"), (45, "45°")];

/// The rail's categories.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Category {
    /// Nose, tubes, transitions, couplers.
    Airframe,
    /// Fin sets, lugs, rail buttons.
    Fins,
    /// Inner tubes, rings, bulkheads.
    Internal,
    /// Parachutes, streamers, shock cord.
    Recovery,
    /// Avionics, payload, ballast.
    Mass,
    /// Motors.
    Motors,
    /// Saved assemblies.
    Presets,
}

impl Category {
    /// Every category, in rail order.
    pub const ALL: [Category; 7] =
        [Category::Airframe, Category::Fins, Category::Internal, Category::Recovery, Category::Mass, Category::Motors, Category::Presets];

    /// The uppercase header label.
    pub fn label(self) -> &'static str {
        match self {
            Category::Airframe => "AIRFRAME",
            Category::Fins => "FINS & GUIDES",
            Category::Internal => "INTERNAL",
            Category::Recovery => "RECOVERY",
            Category::Mass => "MASS",
            Category::Motors => "MOTORS",
            Category::Presets => "PRESETS",
        }
    }

    fn icon(self) -> RailIcon {
        match self {
            Category::Airframe => RailIcon::Airframe,
            Category::Fins => RailIcon::FinsGuides,
            Category::Internal => RailIcon::Internal,
            Category::Recovery => RailIcon::Recovery,
            Category::Mass => RailIcon::Mass,
            Category::Motors => RailIcon::Motors,
            Category::Presets => RailIcon::Presets,
        }
    }
}

/// The parts bin's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// A rail category was picked.
    Category(Category),
    /// Symmetry count for new radial parts.
    Symmetry(u8),
    /// Angle snap for radial parts, degrees (0 for none).
    AngleSnap(u8),
    /// The search text changed.
    Search(String),
    /// `/`: focus the search field.
    FocusSearch,
    /// "Size to selection" toggled.
    SizeToSelection(bool),
    /// Left button pressed on a row.
    RowPress(RowId),
    /// The cursor entered or left a row.
    RowHover(Option<RowId>),
    /// The cursor entered or left the bin (the rail keys `1`..`7` only work while it is over).
    Hover(bool),
    /// The cursor moved (sent by the app for every move, so a press can turn into a drag).
    CursorMoved,
    /// Left button released anywhere nothing took it: a press that never became a drag is a click.
    Released,
    /// Escape: forget the press.
    Cancel,
    /// `X` / `Shift+X`: the next (or previous) symmetry count.
    CycleSymmetry(bool),
    /// Save the selected part (with what it carries) as a user preset.
    SavePreset,
    /// Delete user preset `n` (index into the bin's presets).
    DeletePreset(usize),
}

/// A pressed row that has not yet become a click or a drag.
#[derive(Debug)]
struct Press {
    row: RowId,
    origin: Option<Point>,
}

/// The parts bin's state.
pub struct State {
    category: Category,
    search: String,
    size_to_selection: bool,
    /// The row under the cursor.
    row_hover: Option<RowId>,
    press: Option<Press>,
    /// The row a drag in progress came from.
    dragging: Option<RowId>,
    /// The cursor is over the bin.
    hovered: bool,
    last_click: Option<(RowId, Instant)>,
    presets: Vec<PresetEntry>,
}

impl State {
    /// The bin on the airframe list, with the built-in and the user's presets loaded.
    pub fn new() -> Self {
        State {
            category: Category::Airframe,
            search: String::new(),
            size_to_selection: true,
            row_hover: None,
            press: None,
            dragging: None,
            hovered: false,
            last_click: None,
            presets: presets::load_all(),
        }
    }

    /// The reference diameter new parts are sized to, when "size to selection" is on.
    fn sizing(&self, ctx: &Ctx) -> Option<f64> {
        self.size_to_selection.then(|| ctx.doc.rocket().reference_diameter()).flatten()
    }

    /// The rows of `category` (the full list, whatever the search says).
    fn items(&self, category: Category, ctx: &Ctx) -> Vec<Item> {
        let from = |list: Vec<(&'static str, InternalKind)>| list.into_iter().map(|(n, k)| Item::Internal(n, k)).collect();
        match category {
            Category::Airframe => airframe_items(),
            Category::Fins => radial_catalogue().into_iter().map(|(n, k)| Item::Radial(n, k)).collect(),
            Category::Internal => from(structure_catalogue()),
            Category::Recovery => from(recovery_catalogue()),
            Category::Mass => from(mass_catalogue()),
            Category::Motors => items::motor_mount(ctx.doc.rocket(), ctx.selection)
                .map(|mount| items::fitting_motors(ctx.physics.motors(), &mount).into_iter().map(Item::Motor).collect())
                .unwrap_or_default(),
            Category::Presets => (0..self.presets.len()).map(Item::Preset).collect(),
        }
    }

    fn item(&self, id: RowId, ctx: &Ctx) -> Option<Item> {
        self.items(id.category, ctx).into_iter().nth(id.index)
    }

    fn name_of(&self, item: &Item, ctx: &Ctx) -> String {
        match item {
            Item::Axial(n, _) | Item::Radial(n, _) | Item::Internal(n, _) => (*n).to_string(),
            Item::Motor(i) => ctx.physics.motors().motors[*i].designation.clone(),
            Item::Preset(i) => self.presets[*i].preset.name.clone(),
        }
    }

    /// Why `item` cannot be added right now, if it cannot.
    fn unavailable(&self, item: &Item, ctx: &Ctx) -> Option<String> {
        let rocket = ctx.doc.rocket();
        match item {
            Item::Axial(name, kind) => {
                let kind = sized(kind, self.sizing(ctx));
                if rocket.nodes_for(&kind).is_empty() {
                    return Some(if matches!(kind, PartKind::NoseCone { .. }) {
                        "A rocket has one nose cone, at the fore end".into()
                    } else {
                        "Nothing in the stack to attach it to".into()
                    });
                }
                (*name == "Payload bay" && !rocket.internals().iter().any(|i| matches!(i.kind, InternalKind::Coupler { .. })))
                    .then(|| "Payload bay needs a coupler above it".into())
            }
            Item::Radial(..) => rocket.parts().is_empty().then(|| "Needs an airframe part to attach to".into()),
            Item::Internal(_, kind) => items::host_for(rocket, ctx.selection, items::zone_of(kind)).is_none().then(|| "Needs a body tube to go in".into()),
            Item::Motor(_) => None,
            Item::Preset(i) => self.presets[*i].preset.unavailable(rocket, ctx.selection).map(Into::into),
        }
    }

    /// Starts dragging row `id`, if it can be dragged and placed.
    fn start_drag(&mut self, id: RowId, ctx: &mut Ctx) {
        let Some(item) = self.item(id, ctx) else {
            return;
        };
        if self.unavailable(&item, ctx).is_some() {
            return;
        }
        match item {
            Item::Axial(name, kind) => drag::start_axial(ctx, id.index, name, sized(&kind, self.sizing(ctx))),
            Item::Radial(_, kind) => drag::start_radial(ctx, id.index, kind),
            Item::Internal(name, kind) => {
                let rocket = ctx.doc.rocket();
                let kind = match items::host_for(rocket, ctx.selection, items::zone_of(&kind)).and_then(|h| items::host_inner(rocket, h)) {
                    Some(inner) => items::fitted(&kind, inner),
                    None => kind,
                };
                drag::start_internal(ctx, Some(id.index), name, kind);
            }
            Item::Motor(_) | Item::Preset(_) => return,
        }
        self.dragging = Some(id);
    }

    /// Adds `item` by a click. Returns whether something was added.
    fn click(&mut self, item: Item, ctx: &mut Ctx) -> bool {
        match item {
            Item::Axial(_, kind) => items::attach_axial(ctx, sized(&kind, self.sizing(ctx))).is_some(),
            Item::Radial(name, kind) => items::attach_radial(ctx, name, kind).is_some(),
            Item::Internal(name, kind) => items::attach_internal(ctx, name, kind).is_some(),
            Item::Motor(i) => items::motor_mount(ctx.doc.rocket(), ctx.selection).is_some_and(|m| items::set_motor(ctx, m.id, i)),
            Item::Preset(i) => self.presets[i].preset.clone().apply(ctx).is_some(),
        }
    }

    fn reload_presets(&mut self) {
        self.presets = presets::load_all();
    }

    /// Handles one of the bin's messages.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Category(c) => self.category = c,
            Message::Symmetry(n) => ctx.drag.symmetry = n,
            Message::AngleSnap(degrees) => ctx.drag.angle_snap = degrees,
            Message::Search(s) => self.search = s,
            Message::FocusSearch => return iced::widget::operation::focus(SEARCH_ID),
            Message::SizeToSelection(on) => self.size_to_selection = on,
            Message::RowPress(row) => self.press = Some(Press { row, origin: ctx.cursor }),
            Message::RowHover(row) => self.row_hover = row,
            Message::Hover(on) => self.hovered = on,
            Message::CursorMoved => {
                let at = ctx.cursor;
                let started = self.press.as_ref().filter(|p| matches!((p.origin, at), (Some(o), Some(at)) if o.distance(at) > DRAG_SLOP)).map(|p| p.row);
                if let (Some(row), false) = (started, ctx.drag.active()) {
                    self.press = None;
                    self.start_drag(row, ctx);
                }
            }
            Message::Released => {
                self.dragging = None;
                if let Some(press) = self.press.take() {
                    let Some(item) = self.item(press.row, ctx).filter(|i| self.unavailable(i, ctx).is_none()) else {
                        return Task::none();
                    };
                    let now = Instant::now();
                    if self.last_click.is_some_and(|(row, at)| row == press.row && now.duration_since(at) < DOUBLE_CLICK) {
                        // The first click already added the part: the second only moves on to editing it.
                        self.last_click = None;
                        return iced::widget::operation::focus(INSPECTOR_FIRST_FIELD);
                    }
                    if self.click(item, ctx) {
                        self.last_click = Some((press.row, now));
                    }
                }
            }
            Message::Cancel => {
                self.press = None;
                self.dragging = None;
            }
            Message::CycleSymmetry(forward) => {
                let step = |n: u8| {
                    let at = SYMMETRY.iter().position(|(c, _)| *c == n).unwrap_or(0);
                    SYMMETRY[(at + if forward { 1 } else { SYMMETRY.len() - 1 }) % SYMMETRY.len()].0
                };
                match ctx.drag.radial.as_mut() {
                    Some(drag) => {
                        drag.count = step(drag.count);
                        // A group dragged from the bin also sets the count for the next one; a picked-up group keeps its own.
                        if drag.origin.is_none() {
                            ctx.drag.symmetry = drag.count;
                        }
                    }
                    None => ctx.drag.symmetry = step(ctx.drag.symmetry),
                }
            }
            Message::SavePreset => {
                let captured = ctx.selection.part().and_then(|id| presets::Preset::capture(ctx.doc.rocket(), id));
                match captured {
                    Some(preset) => match presets::save(&preset) {
                        Ok(_) => {
                            ctx.notify(format!("Saved preset {}", preset.name));
                            self.reload_presets();
                        }
                        Err(e) => ctx.notify(format!("Could not save the preset: {e}")),
                    },
                    None => ctx.notify("Select a part to save as a preset"),
                }
            }
            Message::DeletePreset(i) => {
                if let Some(file) = self.presets.get(i).and_then(|p| p.file.clone()) {
                    let _ = std::fs::remove_file(file);
                    self.reload_presets();
                }
            }
        }
        Task::none()
    }

    /// `/` focuses search, `X` / `Shift+X` cycle the symmetry count, and `1` to `7` pick a category while the cursor is over
    /// the bin. Only called for keys no widget handled.
    pub fn on_key(&self, key: &keyboard::Key, modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        let keyboard::Key::Character(c) = key else {
            return None;
        };
        let plain = modifiers.is_empty();
        match c.as_str() {
            "/" if plain => Some(Message::FocusSearch),
            c if (plain || modifiers == keyboard::Modifiers::SHIFT) && c.eq_ignore_ascii_case("x") => Some(Message::CycleSymmetry(!modifiers.shift())),
            digit if plain && self.hovered => {
                let n: usize = digit.parse().ok()?;
                (1..=Category::ALL.len()).contains(&n).then(|| Message::Category(Category::ALL[n - 1]))
            }
            _ => None,
        }
    }

    /// The row of `item`.
    fn row_view(&self, id: RowId, item: &Item, ctx: &Ctx) -> RowView {
        let rocket = ctx.doc.rocket();
        let reason = self.unavailable(item, ctx);
        let mut current = false;
        let (size, sil, deletable) = match item {
            Item::Axial(_, kind) => {
                let kind = sized(kind, self.sizing(ctx));
                let size = match &kind {
                    PartKind::NoseCone { shape: bowshock_model::NoseShape::TangentOgive, .. } => format!("Ogive · {}", size_label(&kind)),
                    _ => size_label(&kind),
                };
                (size, rows::Sil::fill(rows::axial_outline(&kind)), None)
            }
            Item::Radial(_, kind) => (radial_size_label(kind), rows::Sil::fill(rows::radial_outline(kind)), None),
            Item::Internal(_, kind) => {
                let inner = items::host_for(rocket, ctx.selection, items::zone_of(kind)).and_then(|h| items::host_inner(rocket, h));
                (internal_size(kind, inner), rows::internal_outline(kind), None)
            }
            Item::Motor(i) => {
                let m = &ctx.physics.motors().motors[*i];
                current = items::motor_mount(rocket, ctx.selection)
                    .and_then(|mount| rocket.active_configuration().slot(mount.id).and_then(|s| s.motor.as_ref()))
                    .is_some_and(|r| r.designation == m.designation);
                (format!("Ø{:.0} × {:.0} · {:.0} Ns", m.diameter * 1000.0, m.length * 1000.0, m.total_impulse()), rows::motor_outline(), None)
            }
            Item::Preset(i) => {
                let entry = &self.presets[*i];
                (entry.preset.summary(), rows::preset_outline(&entry.preset), (!entry.builtin).then_some(*i))
            }
        };
        let active = ctx.drag.active();
        let carried = match item {
            Item::Axial(..) => ctx.drag.axial.as_ref().and_then(|d| d.row),
            Item::Radial(..) => ctx.drag.radial.as_ref().and_then(|d| d.row),
            Item::Internal(..) => ctx.drag.internal.as_ref().and_then(|d| d.row),
            Item::Motor(_) | Item::Preset(_) => None,
        };
        let state = if reason.is_some() {
            RowState::Unavailable
        } else if carried == Some(id.index) && self.dragging.is_some_and(|d| d.category == id.category) {
            RowState::Dragging
        } else if current {
            RowState::Current
        } else if !active && self.row_hover == Some(id) {
            RowState::Hovered
        } else {
            RowState::Idle
        };
        RowView { id, name: self.name_of(item, ctx), size, sil, state, reason: reason.unwrap_or_default(), deletable }
    }

    /// The rows to list: the category's, or with a search text every category's rows whose name matches.
    fn listed(&self, ctx: &Ctx) -> Vec<(RowId, Item)> {
        let needle = self.search.trim().to_lowercase();
        let categories: &[Category] = if needle.is_empty() { std::slice::from_ref(&self.category) } else { &Category::ALL };
        categories
            .iter()
            .flat_map(|&c| self.items(c, ctx).into_iter().enumerate().map(move |(i, item)| (RowId::new(c, i), item)))
            .filter(|(_, item)| needle.is_empty() || self.name_of(item, ctx).to_lowercase().contains(&needle))
            .collect()
    }

    /// The bin: rail, header, the list, symmetry footer.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let rocket = ctx.doc.rocket();
        let rail = column(Category::ALL.iter().map(|&c| rail_button(c.icon(), c == self.category && self.search.trim().is_empty(), Message::Category(c))))
            .width(RAIL_W);
        let rail = container(rail)
            .height(Fill)
            .style(|theme: &Theme| container::Style { background: Some(Background::Color(theme::tokens(theme).bg_100)), ..container::Style::default() });

        let listed = self.listed(ctx);
        let searching = !self.search.trim().is_empty();
        let title = if searching { "RESULTS" } else { self.category.label() };
        let header = container(row![label(title), Space::new().width(Fill), label(listed.len().to_string())].align_y(iced::Center))
            .padding([SPACE_1, theme::PAD_PANEL])
            .width(Fill);

        let search = stack![
            text_input("Search parts", &self.search)
                .id(SEARCH_ID)
                .on_input(Message::Search)
                .size(TEXT_BODY)
                .padding([4.0, SPACE_2])
                .padding(iced::Padding { left: 26.0, right: 24.0, top: 4.0, bottom: 4.0 })
                .style(theme::input),
            container(canvas_widget(Magnifier).width(14).height(14)).padding(iced::Padding { left: 8.0, top: 8.0, ..iced::Padding::ZERO }),
            container(container(text("/").font(MONO).size(TEXT_LABEL)).padding([0.0, 4.0]).style(theme::keycap)).align_right(Fill).padding(iced::Padding {
                right: 6.0,
                top: 6.0,
                ..iced::Padding::ZERO
            }),
        ];
        let sizing = row![
            checkbox(self.size_to_selection)
                .label("Size to selection")
                .size(12)
                .text_size(TEXT_LABEL)
                .spacing(6)
                .style(theme::check)
                .on_toggle(Message::SizeToSelection),
            text(rocket.reference_diameter().map(|d| format!("Ø{}", mm(d))).unwrap_or_default()).font(MONO).size(TEXT_LABEL),
        ]
        .spacing(SPACE_1)
        .align_y(iced::Center);
        let controls = container(column![search, sizing].spacing(SPACE_2)).padding([SPACE_1, theme::PAD_PANEL]);

        let mut rows: Vec<Element<'_, Message>> = Vec::new();
        if self.category == Category::Presets && !searching {
            let can_save = ctx.selection.part().is_some();
            rows.push(
                container(
                    button(text("Save selection as preset").size(TEXT_LABEL)).style(theme::ghost).on_press_maybe(can_save.then_some(Message::SavePreset)),
                )
                .padding([SPACE_1, theme::PAD_PANEL])
                .boxed(),
            );
        }
        let empty = listed.is_empty().then(|| match self.category {
            _ if searching => "No part matches".to_string(),
            Category::Motors => match items::motor_mount(rocket, ctx.selection) {
                None => "No motor mount. Add one under Internal.".to_string(),
                Some(m) => format!("No bundled motor fits Ø{} mm.", mm(m.inner_diameter)),
            },
            _ => String::new(),
        });
        rows.extend(listed.iter().map(|(id, item)| rows::entry(self.row_view(*id, item, ctx))));
        if let Some(message) = empty.filter(|m| !m.is_empty()) {
            rows.push(container(text(message).size(TEXT_LABEL).style(theme::faint)).padding(theme::PAD_PANEL).boxed());
        }
        let list = scrollable(column(rows).width(Fill))
            .height(Fill)
            .direction(scrollable::Direction::Vertical(scrollable::Scrollbar::new().width(4).scroller_width(4)));

        let rule = || container(Space::new()).width(Fill).height(1).style(theme::hairline);
        let body = column![header, rule(), controls, rule(), list, rule(), symmetry_footer(ctx)].width(Fill).height(Fill);
        let content = container(row![rail, body]).height(Fill).style(theme::panel);
        mouse_area(content).on_enter(Message::Hover(true)).on_exit(Message::Hover(false)).boxed()
    }
}

/// The footer pinned at the bin's foot: the symmetry count (with the `X` key that cycles it) and the angle snap.
fn symmetry_footer<'a>(ctx: &'a Ctx) -> Element<'a, Message> {
    let key = button(container(text("X").font(MONO).size(TEXT_LABEL)).padding([0.0, 4.0]).style(theme::keycap))
        .padding(0)
        .style(|_, _| button::Style::default())
        .on_press(Message::CycleSymmetry(true));
    let head = row![label("SYMMETRY"), Space::new().width(Fill), key].align_y(iced::Center);
    let count =
        row![canvas_widget(SymmetryGlyph { count: ctx.drag.symmetry }).width(22).height(22), segmented_fill(&SYMMETRY, ctx.drag.symmetry, Message::Symmetry)]
            .spacing(SPACE_2)
            .align_y(iced::Center);
    let snap =
        row![text("Angle snap").size(TEXT_BODY).style(theme::muted), Space::new().width(Fill), segmented(&ANGLE_SNAP, ctx.drag.angle_snap, Message::AngleSnap)]
            .align_y(iced::Center);
    container(column![head, count, snap].spacing(SPACE_2)).padding(theme::PAD_PANEL).width(Fill).boxed()
}

/// The size line of an internal part, in mm. Diameters that follow the part it goes in (`inner`, its bore) are shown resolved.
fn internal_size(kind: &InternalKind, inner: Option<f64>) -> String {
    let dia = |v: Option<f64>| v.or(inner).map_or("Ø auto".to_string(), |d| format!("Ø{}", mm(d)));
    match *kind {
        InternalKind::InnerTube { outer_diameter, length, .. } => format!("Ø{} × {}", mm(outer_diameter), mm(length)),
        InternalKind::CenteringRing { outer_diameter, thickness, .. } => format!("{} × {}", dia(outer_diameter), mm(thickness)),
        InternalKind::Bulkhead { diameter, thickness } => format!("{} × {}", dia(diameter), mm(thickness)),
        InternalKind::Coupler { outer_diameter, length, .. } => format!("{} × {}", dia(outer_diameter), mm(length)),
        InternalKind::EngineBlock { inner: bore, thickness, .. } => format!("Ø{} bore × {}", mm(bore), mm(thickness)),
        InternalKind::Parachute { diameter, line_count, .. } => format!("Ø{} · {line_count} lines", mm(diameter)),
        InternalKind::Streamer { length, width, .. } => format!("{} × {}", mm(width), mm(length)),
        InternalKind::ShockCord { length, .. } => format!("{} long", mm(length)),
        InternalKind::MassComponent { mass, length, .. } => format!("{:.0} g · {}", mass * 1000.0, mm(length)),
    }
}

/// The magnifier in the search field.
struct Magnifier;

impl<M> canvas::Program<M> for Magnifier {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let mut frame = Frame::new(renderer, bounds.size());
        let stroke = Stroke::default().with_color(theme::tokens(theme).ink_faint).with_width(1.25);
        frame.stroke(&Path::circle(Point::new(5.5, 5.5), 4.0), stroke);
        frame.stroke(&Path::line(Point::new(8.5, 8.5), Point::new(12.5, 12.5)), stroke);
        vec![frame.into_geometry()]
    }
}

/// The symmetry glyph: `count` spokes around a small ring, in `accent`.
struct SymmetryGlyph {
    count: u8,
}

impl<M> canvas::Program<M> for SymmetryGlyph {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let mut frame = Frame::new(renderer, bounds.size());
        let stroke = Stroke::default().with_color(theme::tokens(theme).accent).with_width(1.5);
        let c = Point::new(bounds.width / 2.0, bounds.height / 2.0);
        let r = bounds.width.min(bounds.height) / 2.0 - 1.0;
        for k in 0..self.count.max(1) {
            let a = -std::f32::consts::FRAC_PI_2 + std::f32::consts::TAU * f32::from(k) / f32::from(self.count.max(1));
            frame.stroke(&Path::line(Point::new(c.x + a.cos() * r * 0.35, c.y + a.sin() * r * 0.35), Point::new(c.x + a.cos() * r, c.y + a.sin() * r)), stroke);
        }
        frame.stroke(&Path::circle(c, r * 0.3), stroke);
        vec![frame.into_geometry()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::ui::size_label;

    #[test]
    fn defaults_follow_the_reference_diameter() {
        let cat = axial_catalogue();
        let (nose, tube, transition) = (&cat[0].1, &cat[5].1, &cat[6].1);
        assert_eq!(size_label(tube), "Ø76.2 × 600.0");
        assert_eq!(size_label(transition), "Ø76.2 → Ø54.0 × 76.2");
        assert_eq!(sized(nose, Some(0.1)).fore_diameter().max(sized(nose, Some(0.1)).aft_diameter()), 0.1);
        // A transition keeps its ratio and its length.
        let big = sized(transition, Some(0.1524));
        assert_eq!(size_label(&big), "Ø152.4 → Ø108.0 × 76.2");
        assert_eq!(sized(tube, None), *tube);
    }

    #[test]
    fn size_lines_follow_the_kit() {
        let coupler = &structure_catalogue()[5].1;
        assert_eq!(internal_size(coupler, Some(0.0737)), "Ø73.7 × 152.4");
        assert_eq!(internal_size(coupler, None), "Ø auto × 152.4");
        assert_eq!(internal_size(&mass_catalogue()[4].1, None), "100 g · 30.0");
    }
}
