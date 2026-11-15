//! Bowshock desktop app: the rocket builder. Layout and styling follow `design/` (the Bowshock UI kit).
// wgpu-core's auto-trait chain (Pipeline: Sync -> ... -> Hub: Send) exceeds the default depth on current nightly.
#![recursion_limit = "256"]

mod drive;
mod logo;
mod gallery;
mod parts;
mod theme;
mod tokens;
#[allow(dead_code)]
mod units;
mod viewport;
mod widgets;

use bowshock_model::{Part, PartId, PartKind, RadialKind, Rocket};
use iced::widget::pane_grid::{self, Axis, Configuration, PaneGrid};
use iced::widget::{button, canvas, checkbox, column, container, mouse_area, row, stack, text, text_input, Space};
use iced::{keyboard, mouse, Element, Fill, Length, Point, Size, Theme, Widget};
use std::sync::Arc;
use std::time::{Duration, Instant};

use parts::RowState;
use viewport::assembly::{Drag, Origin, Overlay};
use viewport::manipulate::{Hover, Overlay as Manipulator};
use viewport::pick::{Pick, SurfaceTarget};
use viewport::surface::{Overlay as SurfaceOverlay, RadialDrag};

use theme::{MONO, SANS, SANS_SEMIBOLD};
use tokens::*;

fn main() -> std::process::ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        [] => match app().run() {
            Ok(()) => std::process::ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("bowshock: {e}");
                std::process::ExitCode::FAILURE
            }
        },
        ["drive", script, rest @ ..] if rest.len() <= 1 => {
            let out = std::path::Path::new(rest.first().copied().unwrap_or("out/drive"));
            viewport::set_animation(false);
            match drive::run(&app(), std::path::Path::new(script), out) {
                Ok(()) => std::process::ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("bowshock drive: {e}");
                    std::process::ExitCode::FAILURE
                }
            }
        }
        ["gallery"] => match gallery::app().run() {
            Ok(()) => std::process::ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("bowshock gallery: {e}");
                std::process::ExitCode::FAILURE
            }
        },
        ["gallery", script, rest @ ..] if rest.len() <= 1 => {
            let out = std::path::Path::new(rest.first().copied().unwrap_or("out/gallery"));
            match drive::run(&gallery::app(), std::path::Path::new(script), out) {
                Ok(()) => std::process::ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("bowshock gallery: {e}");
                    std::process::ExitCode::FAILURE
                }
            }
        }
        _ => {
            eprintln!("usage: bowshock                       open the app\n       bowshock drive <script> [out-dir]  run a UI script headless, saving screenshots (see src/drive.rs)");
            std::process::ExitCode::from(2)
        }
    }
}

fn app() -> iced::Application<impl iced::Program<State = App, Message = Message, Theme = Theme>> {
    iced::application(App::new, App::update, App::view)
        .title("Bowshock")
        .theme(App::theme)
        .subscription(App::subscription)
        .fonts(theme::FONTS)
        .font(SANS)
        .window_size(Size::new(1280.0, 800.0))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pane {
    PartsBin,
    Viewport,
    Lanes,
    Inspector,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Build,
    Simulate,
    Analyze,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Projection {
    ThreeD,
    Profile,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InsideView {
    Solid,
    Cutaway,
    XRay,
    Exploded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Category {
    Airframe,
    Fins,
    Internal,
    Recovery,
    Mass,
    Motors,
    Presets,
}

impl Category {
    const ALL: [Category; 7] =
        [Category::Airframe, Category::Fins, Category::Internal, Category::Recovery, Category::Mass, Category::Motors, Category::Presets];

    fn label(self) -> &'static str {
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
}

/// Symmetry counts for radial parts (design/components/Symmetry).
const SYMMETRY: [(u8, &str); 6] = [(1, "1"), (2, "2"), (3, "3"), (4, "4"), (6, "6"), (8, "8")];

/// Angle snap choices for radial parts, in degrees.
const ANGLE_SNAP: [(u8, &str); 3] = [(0, "Off"), (15, "15°"), (45, "45°")];

#[derive(Debug, Clone)]
enum Message {
    Resized(pane_grid::ResizeEvent),
    Mode(Mode),
    Projection(Projection),
    InsideView(InsideView),
    Category(Category),
    Symmetry(u8),
    ToggleTheme,
    Viewport(viewport::Message),
    /// Animation / notice clock tick.
    Frame(Instant),
    /// The cursor moved (window coordinates); tracked app-wide so a drag can leave the parts bin.
    Cursor(Point),
    /// Left button released anywhere nothing took it.
    Released,
    /// Escape: abandon the press or drag in progress.
    Cancel,
    Search(String),
    SizeToSelection(bool),
    /// Left button pressed on a parts bin row (index into the catalogue).
    RowPress(usize),
    RowHover(Option<usize>),
    /// The drag overlay's nearest attach node changed.
    Snap(Option<usize>),
    /// What is under the cursor changed.
    Hover(Hover),
    /// A click on a part (or on empty space, `None`).
    Select(Option<Pick>),
    /// A part was pressed and dragged: pick it up.
    Grab(Pick),
    /// A resize handle moved the part to this size.
    Resize(PartId, PartKind),
    /// A resize handle moved the radial part to this size.
    ResizeRadial(PartId, RadialKind),
    /// The surface overlay's target for the dragged radial part changed.
    Surface(Option<SurfaceTarget>),
    /// Released over the viewport: drop the dragged radial part on its surface target, if any.
    DropRadial,
    /// Angle snap for radial parts, in degrees (0 for none).
    AngleSnap(u8),
    /// `X` / `Shift+X`: the next (or previous) symmetry count.
    CycleSymmetry(bool),
    /// Delete / Backspace: remove the selected part.
    Delete,
    /// Released over the viewport: drop the dragged part on the snap node, if any.
    Drop,
}

/// Movement in px after pressing a parts bin row beyond which the press is a drag rather than a click.
const DRAG_SLOP: f32 = 4.0;
/// How long a status message stays up.
const NOTICE_TIME: Duration = Duration::from_secs(3);

/// A pressed parts bin row that has not yet become a click or a drag.
struct Press {
    row: usize,
    origin: Option<Point>,
}

struct App {
    panes: pane_grid::State<Pane>,
    dark: bool,
    mode: Mode,
    projection: Projection,
    inside: InsideView,
    category: Category,
    symmetry: u8,
    view: viewport::View,
    rocket: Rocket,
    search: String,
    size_to_selection: bool,
    /// The parts bin row under the cursor.
    row_hover: Option<usize>,
    /// What is under the cursor in the viewport.
    hover: Hover,
    selected: Option<PartId>,
    /// Which copy of a selected radial group was clicked.
    instance: usize,
    /// Angle snap for radial parts, degrees.
    angle_snap: u8,
    /// A radial part being dragged (never together with `drag`).
    radial_drag: Option<RadialDrag>,
    cursor: Option<Point>,
    press: Option<Press>,
    drag: Option<Drag>,
    /// A transient status message and when it expires.
    notice: Option<(String, Instant)>,
}

impl App {
    fn new() -> Self {
        let width = 1280.0;
        let bin = (RAIL_W + BIN_W) / width;
        let inspector = RIGHT_PANEL_W / (width * (1.0 - bin));
        let centre = Configuration::Split {
            axis: Axis::Horizontal,
            ratio: 1.0 - LANES_H / (800.0 - TOPBAR_H - STATUSBAR_H),
            a: Box::new(Configuration::Pane(Pane::Viewport)),
            b: Box::new(Configuration::Pane(Pane::Lanes)),
        };
        let layout = Configuration::Split {
            axis: Axis::Vertical,
            ratio: bin,
            a: Box::new(Configuration::Pane(Pane::PartsBin)),
            b: Box::new(Configuration::Split {
                axis: Axis::Vertical,
                ratio: 1.0 - inspector,
                a: Box::new(centre),
                b: Box::new(Configuration::Pane(Pane::Inspector)),
            }),
        };
        App {
            panes: pane_grid::State::with_configuration(layout),
            dark: true,
            mode: Mode::Build,
            projection: Projection::ThreeD,
            inside: InsideView::Solid,
            category: Category::Airframe,
            symmetry: 3,
            view: viewport::View::new(),
            rocket: Rocket::default(),
            search: String::new(),
            size_to_selection: true,
            row_hover: None,
            hover: Hover::None,
            selected: None,
            instance: 0,
            angle_snap: 0,
            radial_drag: None,
            cursor: None,
            press: None,
            drag: None,
            notice: None,
        }
    }

    /// The reference diameter new parts are sized to, when "size to selection" is on.
    fn sizing(&self) -> Option<f64> {
        self.size_to_selection.then(|| self.rocket.reference_diameter()).flatten()
    }

    /// The part a catalogue row would add right now.
    fn row_kind(&self, row: usize) -> PartKind {
        parts::sized(&parts::catalogue()[row].1, self.sizing())
    }

    /// Rebuilds the model mesh from the rocket: the ghost while dragging over a node, the hovered and the selected part.
    fn refresh_mesh(&mut self) {
        let ghost = self.drag.as_ref().and_then(|d| d.snap.map(|node| (node, self.rocket.matched(node, &d.kind))));
        let ghost_radial = self.radial_drag.as_ref().and_then(|d| {
            let t = d.target?;
            Some(viewport::GhostRadial { kind: &d.kind, parent: t.parent, offset: t.offset, angle: t.angle, count: d.count })
        });
        let marks =
            viewport::Marks { ghost: ghost.as_ref().map(|(node, kind)| (*node, kind)), ghost_radial, hovered: self.hover.part(), selected: self.selected };
        self.view.set_mesh(Arc::new(viewport::Mesh::from_rocket(&self.rocket, marks)));
    }

    /// Inserts `kind` at attach node `index` (keeping `id` for a part that was moved), and reports a diameter change. A new
    /// part also frames the rocket; moving or resizing leaves the camera where the user put it.
    fn place(&mut self, index: usize, kind: PartKind, id: Option<PartId>) -> bool {
        let matched = self.rocket.matched(index, &kind);
        let placed = match id {
            Some(id) => self.rocket.insert_with_id(index, id, kind.clone()).is_ok(),
            None => self.rocket.insert(index, kind.clone()).is_ok(),
        };
        if placed {
            let differs = |a: f64, b: f64| (a - b).abs() > 1e-9;
            if differs(matched.fore_diameter(), kind.fore_diameter()) || differs(matched.aft_diameter(), kind.aft_diameter()) {
                let diameter = if differs(matched.fore_diameter(), kind.fore_diameter()) && matched.fore_diameter() > 0.0 {
                    matched.fore_diameter()
                } else {
                    matched.aft_diameter()
                };
                self.notice = Some((format!("Diameter matched to {} mm", parts::mm(diameter)), Instant::now() + NOTICE_TIME));
            }
        }
        self.refresh_mesh();
        if placed && id.is_none() {
            self.view.fit();
        }
        placed
    }

    /// Ends the drag: drops it on its snap node if `drop`, else abandons it. A part picked up from the rocket that is not
    /// dropped on a node goes back where it was, as it was.
    fn finish_drag(&mut self, drop: bool) {
        let Some(drag) = self.drag.take() else {
            return;
        };
        let placed = match drag.snap {
            Some(node) if drop => self.place(node, drag.kind, drag.origin.as_ref().map(|o| o.id)),
            _ => false,
        };
        match (placed, drag.origin) {
            // The radial parts it carried go back on it, wherever it landed.
            (true, Some(origin)) => self.rocket.restore_radials(origin.radials),
            (false, Some(origin)) => {
                let _ = self.rocket.restore(origin.index, Part::new(origin.id, origin.kind));
                self.rocket.restore_radials(origin.radials);
            }
            (_, None) => {}
        }
        self.refresh_mesh();
    }

    /// Ends the radial drag: drops it on its surface target if `drop`, else abandons it. A group picked up from the rocket that
    /// is not dropped on a target goes back as it was.
    fn finish_radial_drag(&mut self, drop: bool) {
        let Some(drag) = self.radial_drag.take() else {
            return;
        };
        let placed = match (drop, drag.target) {
            (true, Some(t)) => match drag.origin.clone() {
                Some(origin) => {
                    let moved =
                        bowshock_model::RadialPart { kind: drag.kind.clone(), parent: t.parent, offset: t.offset, angle: t.angle, count: drag.count, ..origin };
                    self.rocket.restore_radial(moved).is_ok()
                }
                None => self.rocket.attach_radial(t.parent, t.offset, t.angle, drag.count, drag.kind.clone()).is_ok(),
            },
            _ => false,
        };
        if let (false, Some(origin)) = (placed, drag.origin) {
            let _ = self.rocket.restore_radial(origin);
        }
        self.refresh_mesh();
    }

    /// Attaches a radial part by a click on its row: to the aftmost body tube (else the aftmost part), flush with its aft end.
    fn attach_radial_aft(&mut self, kind: RadialKind) {
        let parts = self.rocket.parts();
        let Some(parent) = parts.iter().rev().find(|p| matches!(p.kind, PartKind::BodyTube { .. })).or(parts.last()) else {
            return;
        };
        let offset = parent.kind.length() - kind.extent();
        let _ = self.rocket.attach_radial(parent.id, offset, 0.0, self.symmetry, kind);
        self.refresh_mesh();
    }

    /// Starts dragging bin row `row` of the current category, if the part can be placed.
    fn start_bin_drag(&mut self, row: usize) {
        match self.category {
            Category::Airframe => {
                let kind = self.row_kind(row);
                if !self.rocket.nodes_for(&kind).is_empty() {
                    self.drag = Some(Drag { row: Some(row), origin: None, name: parts::catalogue()[row].0, kind, snap: None });
                }
            }
            Category::Fins if !self.rocket.parts().is_empty() => {
                let kind = parts::radial_catalogue()[row].1.clone();
                self.radial_drag = Some(RadialDrag { row: Some(row), kind, count: self.symmetry, origin: None, target: None });
            }
            _ => {}
        }
        if self.drag.is_some() || self.radial_drag.is_some() {
            self.hover = Hover::None;
            self.refresh_mesh();
        }
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::Resized(e) => self.panes.resize(e.split, e.ratio),
            Message::Mode(m) => self.mode = m,
            Message::Projection(p) => {
                self.projection = p;
                self.view.set_profile(p == Projection::Profile);
            }
            Message::InsideView(v) => self.inside = v,
            Message::Category(c) => self.category = c,
            Message::Symmetry(n) => self.symmetry = n,
            Message::ToggleTheme => self.dark = !self.dark,
            Message::Viewport(m) => self.view.update(m),
            Message::Frame(now) => {
                self.view.update(viewport::Message::Tick(now));
                if self.notice.as_ref().is_some_and(|(_, until)| now >= *until) {
                    self.notice = None;
                }
            }
            Message::Cursor(at) => {
                self.cursor = Some(at);
                let started = self.press.as_ref().filter(|p| p.origin.is_some_and(|o| o.distance(at) > DRAG_SLOP)).map(|p| p.row);
                if let (Some(row), None, None) = (started, &self.drag, &self.radial_drag) {
                    self.press = None;
                    self.start_bin_drag(row);
                }
            }
            Message::Released => {
                if let Some(press) = self.press.take() {
                    // No drag started: a click attaches aft, at the last node the part may take.
                    match self.category {
                        Category::Airframe => {
                            let kind = self.row_kind(press.row);
                            if let Some(&(node, _)) = self.rocket.nodes_for(&kind).last() {
                                self.place(node, kind, None);
                            }
                        }
                        Category::Fins => self.attach_radial_aft(parts::radial_catalogue()[press.row].1.clone()),
                        _ => {}
                    }
                }
                self.finish_drag(false);
                self.finish_radial_drag(false);
            }
            Message::Cancel => {
                self.press = None;
                self.finish_drag(false);
                self.finish_radial_drag(false);
            }
            Message::Search(s) => self.search = s,
            Message::SizeToSelection(on) => self.size_to_selection = on,
            Message::RowPress(row) => self.press = Some(Press { row, origin: self.cursor }),
            Message::RowHover(row) => self.row_hover = row,
            Message::Snap(node) => {
                if let Some(drag) = self.drag.as_mut().filter(|d| d.snap != node) {
                    drag.snap = node;
                    self.refresh_mesh();
                }
            }
            Message::Drop => self.finish_drag(true),
            Message::Hover(hover) => {
                let repaint = hover.part() != self.hover.part();
                self.hover = hover;
                if repaint {
                    self.refresh_mesh();
                }
            }
            Message::Select(pick) => {
                self.selected = pick.map(Pick::id);
                self.instance = match pick {
                    Some(Pick::Radial { instance, .. }) => instance,
                    _ => 0,
                };
                self.refresh_mesh();
            }
            Message::Grab(Pick::Axial(id)) => {
                if self.drag.is_none() && self.radial_drag.is_none() {
                    if let Some(index) = self.rocket.index_of(id) {
                        // Its radial parts ride along: taken off now, put back on it where it lands.
                        let radials = self.rocket.take_radials(id);
                        if let Some(part) = self.rocket.remove(id) {
                            self.selected = Some(id);
                            self.hover = Hover::None;
                            self.drag = Some(Drag {
                                row: None,
                                name: part.kind.name(),
                                kind: part.kind.clone(),
                                snap: None,
                                origin: Some(Origin { id, index, kind: part.kind, radials }),
                            });
                            self.refresh_mesh();
                        }
                    }
                }
            }
            Message::Grab(Pick::Radial { id, .. }) => {
                if self.drag.is_none() && self.radial_drag.is_none() {
                    if let Some(radial) = self.rocket.remove_radial(id) {
                        self.selected = Some(id);
                        self.instance = 0;
                        self.hover = Hover::None;
                        self.radial_drag = Some(RadialDrag { row: None, kind: radial.kind.clone(), count: radial.count, origin: Some(radial), target: None });
                        self.refresh_mesh();
                    }
                }
            }
            Message::Resize(id, kind) => {
                if self.rocket.set_kind(id, kind) {
                    self.refresh_mesh();
                }
            }
            Message::ResizeRadial(id, kind) => {
                let count = self.rocket.radial(id).map_or(1, |r| r.count);
                if self.rocket.set_radial(id, kind, count) {
                    self.refresh_mesh();
                }
            }
            Message::Surface(target) => {
                if let Some(drag) = self.radial_drag.as_mut().filter(|d| d.target != target) {
                    drag.target = target;
                    self.refresh_mesh();
                }
            }
            Message::DropRadial => self.finish_radial_drag(true),
            Message::AngleSnap(degrees) => self.angle_snap = degrees,
            Message::CycleSymmetry(forward) => {
                let step = |n: u8| {
                    let at = SYMMETRY.iter().position(|(c, _)| *c == n).unwrap_or(0);
                    SYMMETRY[(at + if forward { 1 } else { SYMMETRY.len() - 1 }) % SYMMETRY.len()].0
                };
                match self.radial_drag.as_mut() {
                    Some(drag) => {
                        drag.count = step(drag.count);
                        // A group dragged from the bin also sets the count for the next one; a picked-up group keeps its own.
                        if drag.origin.is_none() {
                            self.symmetry = drag.count;
                        }
                        self.refresh_mesh();
                    }
                    None => self.symmetry = step(self.symmetry),
                }
            }
            Message::Delete => {
                if let (None, None, Some(id)) = (&self.drag, &self.radial_drag, self.selected.take()) {
                    // An axial part takes its radial parts with it; a radial part is a whole group.
                    if self.rocket.remove(id).is_none() {
                        self.rocket.remove_radial(id);
                    }
                    self.hover = Hover::None;
                    self.refresh_mesh();
                }
            }
        }
    }

    fn theme(&self) -> Theme {
        if self.dark {
            theme::dark()
        } else {
            theme::light()
        }
    }

    fn subscription(&self) -> iced::Subscription<Message> {
        // Global events: view shortcuts, Escape to abandon a drag, and the button release that ends a drag or click wherever it
        // lands (the window's root widget also sees releases the viewport and bin did not take, but not ones outside the window).
        let events = iced::event::listen_with(|event, status, _| {
            let ignored = status == iced::event::Status::Ignored;
            match event {
                iced::Event::Keyboard(keyboard::Event::KeyPressed { key: keyboard::Key::Named(keyboard::key::Named::Escape), .. }) => Some(Message::Cancel),
                iced::Event::Keyboard(keyboard::Event::KeyPressed { key: keyboard::Key::Character(c), modifiers, .. }) if ignored && modifiers.is_empty() => {
                    match c.as_str() {
                        "1" => Some(Message::InsideView(InsideView::Solid)),
                        "2" => Some(Message::InsideView(InsideView::Cutaway)),
                        "3" => Some(Message::InsideView(InsideView::XRay)),
                        "4" => Some(Message::InsideView(InsideView::Exploded)),
                        _ => None,
                    }
                }
                iced::Event::Keyboard(keyboard::Event::KeyPressed { key: keyboard::Key::Character(c), modifiers, .. })
                    if ignored && (modifiers.is_empty() || modifiers == keyboard::Modifiers::SHIFT) && c.eq_ignore_ascii_case("x") =>
                {
                    Some(Message::CycleSymmetry(!modifiers.shift()))
                }
                iced::Event::Keyboard(keyboard::Event::KeyPressed {
                    key: keyboard::Key::Named(keyboard::key::Named::Delete | keyboard::key::Named::Backspace),
                    ..
                }) if ignored => Some(Message::Delete),
                iced::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) if ignored => Some(Message::Released),
                _ => None,
            }
        });
        let frames = (self.view.animating() || self.notice.is_some()).then(|| iced::window::frames().map(Message::Frame));
        iced::Subscription::batch([events].into_iter().chain(frames))
    }

    fn view(&self) -> Element<'_, Message> {
        let panes = PaneGrid::new(&self.panes, |_, pane, _| {
            pane_grid::Content::new(match pane {
                Pane::PartsBin => self.parts_bin(),
                Pane::Viewport => self.viewport(),
                Pane::Lanes => self.lanes(),
                Pane::Inspector => self.inspector(),
            })
        })
        .on_resize(6, Message::Resized)
        .style(theme::panes);

        // The root sees every cursor move and any release the widgets below leave alone: that is what lets a drag begun in the
        // parts bin follow the cursor anywhere, and end (or be cancelled) wherever it is released.
        mouse_area(column![self.top_bar(), panes, self.status_strip()]).on_move(Message::Cursor).on_release(Message::Released).boxed()
    }

    fn top_bar(&self) -> Element<'_, Message> {
        let logo = row![canvas(logo::Mark).width(20).height(20), text("BOWSHOCK").font(SANS_SEMIBOLD).size(TEXT_BODY),].spacing(SPACE_2).align_y(iced::Center);
        let modes = segmented(&[(Mode::Build, "Build"), (Mode::Simulate, "Simulate"), (Mode::Analyze, "Analyze")], self.mode, Message::Mode);
        let theme_toggle =
            button(text(if self.dark { "Light" } else { "Dark" }).size(TEXT_BODY)).style(theme::ghost).height(CONTROL_H).on_press(Message::ToggleTheme);
        // Disabled until there is an analysis to run.
        let simulate = button(text("Simulate").font(theme::SANS_MEDIUM).size(TEXT_BODY)).style(theme::primary).height(CONTROL_H);

        container(
            row![
                logo,
                text("untitled.bowshock").font(MONO).size(TEXT_BODY),
                Space::new().width(Fill),
                modes,
                Space::new().width(Fill),
                theme_toggle,
                simulate,
            ]
            .spacing(SPACE_3)
            .padding([0.0, SPACE_3])
            .align_y(iced::Center),
        )
        .height(TOPBAR_H)
        .center_y(TOPBAR_H)
        .style(theme::bar)
        .boxed()
    }

    fn parts_bin(&self) -> Element<'_, Message> {
        let rail = column(Category::ALL.iter().map(|&c| {
            let active = c == self.category;
            let glyph = text(&c.label()[..1]).font(MONO).size(TEXT_BODY);
            row![
                container(Space::new()).width(2).height(RAIL_W).style(theme::underline(active)),
                button(container(glyph).center(Fill)).width(Fill).height(RAIL_W).style(theme::segment(active)).on_press(Message::Category(c)),
            ]
            .boxed()
        }))
        .width(RAIL_W);

        let symmetry = column![
            label("SYMMETRY"),
            segmented(&SYMMETRY, self.symmetry, Message::Symmetry),
            label("ANGLE SNAP"),
            segmented(&ANGLE_SNAP, self.angle_snap, Message::AngleSnap),
        ]
        .spacing(SPACE_2);

        let rows: Vec<Element<'_, Message>> = match self.category {
            Category::Airframe => {
                let needle = self.search.to_lowercase();
                parts::catalogue()
                    .iter()
                    .enumerate()
                    .map(|(index, (name, _))| (index, *name, self.row_kind(index)))
                    .filter(|(_, name, _)| name.to_lowercase().contains(&needle))
                    .map(|(index, name, kind)| {
                        let state = if self.rocket.nodes_for(&kind).is_empty() {
                            RowState::Unavailable
                        } else if self.drag.as_ref().is_some_and(|d| d.row == Some(index)) {
                            RowState::Dragging
                        } else if self.row_hover == Some(index) && self.drag.is_none() {
                            RowState::Hovered
                        } else {
                            RowState::Idle
                        };
                        parts::entry(index, name, parts::size_label(&kind), parts::axial_outline(&kind), state, "A rocket has one nose cone, at the fore end")
                    })
                    .collect()
            }
            Category::Fins => {
                let needle = self.search.to_lowercase();
                let attachable = !self.rocket.parts().is_empty();
                parts::radial_catalogue()
                    .iter()
                    .enumerate()
                    .filter(|(_, (name, _))| name.to_lowercase().contains(&needle))
                    .map(|(index, (name, kind))| {
                        let state = if !attachable {
                            RowState::Unavailable
                        } else if self.radial_drag.as_ref().is_some_and(|d| d.row == Some(index)) {
                            RowState::Dragging
                        } else if self.row_hover == Some(index) && self.radial_drag.is_none() {
                            RowState::Hovered
                        } else {
                            RowState::Idle
                        };
                        parts::entry(index, name, parts::radial_size_label(kind), parts::radial_outline(kind), state, "Needs an airframe part to attach to")
                    })
                    .collect()
            }
            // Internals, recovery, mass, motors and presets are not in the bin yet.
            _ => Vec::new(),
        };

        let search = text_input("Search parts", &self.search).on_input(Message::Search).size(TEXT_BODY).padding([4.0, SPACE_2]).style(theme::input);
        let sizing = row![
            checkbox(self.size_to_selection)
                .label("Size to selection")
                .size(12)
                .text_size(TEXT_LABEL)
                .spacing(6)
                .style(theme::check)
                .on_toggle(Message::SizeToSelection),
            text(self.rocket.reference_diameter().map(|d| format!("Ø{}", parts::mm(d))).unwrap_or_default()).font(MONO).size(TEXT_LABEL),
        ]
        .spacing(SPACE_1)
        .align_y(iced::Center);

        let header = container(column![label(self.category.label()), search, sizing].spacing(SPACE_2)).padding(theme::PAD_PANEL);
        let list = column![header, column(rows), Space::new().height(Fill), container(symmetry).padding(theme::PAD_PANEL),].width(Fill);

        container(row![rail, list]).height(Fill).style(theme::panel).boxed()
    }

    fn viewport(&self) -> Element<'_, Message> {
        let tokens = theme::tokens(&self.theme());
        // While a part is dragged an assembly overlay owns the viewport; otherwise the manipulation overlay does.
        let camera = self.view.camera();
        let (overlay, interaction) = if let Some(drag) = &self.drag {
            (canvas(Overlay { rocket: &self.rocket, drag, camera, cursor: self.cursor }).width(Fill).height(Fill).boxed(), mouse::Interaction::default())
        } else if let Some(drag) = &self.radial_drag {
            (
                canvas(SurfaceOverlay { rocket: &self.rocket, drag, camera, snap: self.angle_snap, cursor: self.cursor }).width(Fill).height(Fill).boxed(),
                mouse::Interaction::default(),
            )
        } else {
            (
                canvas(Manipulator { rocket: &self.rocket, camera, selected: self.selected, instance: self.instance, hover: self.hover, cursor: self.cursor })
                    .width(Fill)
                    .height(Fill)
                    .boxed(),
                self.hover.interaction(),
            )
        };
        let scene = self.view.view(tokens, Message::Viewport, Some(overlay), interaction);
        let toolbar = container(
            row![
                segmented(&[(Projection::ThreeD, "3D"), (Projection::Profile, "2D")], self.projection, Message::Projection),
                segmented(
                    &[(InsideView::Solid, "Solid"), (InsideView::Cutaway, "Cutaway"), (InsideView::XRay, "X-ray"), (InsideView::Exploded, "Exploded"),],
                    self.inside,
                    Message::InsideView,
                ),
            ]
            .spacing(SPACE_2),
        )
        .padding(SPACE_1)
        .style(theme::overlay);
        // Overlays float `space-4` from the viewport edges.
        let toolbar = container(toolbar).padding(SPACE_4);
        stack![scene, toolbar].boxed()
    }

    fn lanes(&self) -> Element<'_, Message> {
        container(column![label("MM FROM TIP")].padding(theme::PAD_PANEL)).width(Fill).height(Fill).style(theme::ground).boxed()
    }

    fn inspector(&self) -> Element<'_, Message> {
        let (title, note) = match self.selected {
            Some(id) => match (self.rocket.part(id), self.rocket.radial(id)) {
                (Some(part), _) => (part.kind.name().to_string(), "SELECTED"),
                (_, Some(radial)) if radial.count > 1 => (format!("{} · LINKED ×{}", radial.kind.name(), radial.count), "SELECTED"),
                (_, Some(radial)) => (radial.kind.name().to_string(), "SELECTED"),
                _ => ("Rocket".to_string(), "NOTHING SELECTED"),
            },
            None => ("Rocket".to_string(), "NOTHING SELECTED"),
        };
        let rocket = column![text(title).font(SANS_SEMIBOLD).size(TEXT_BODY + 2.0), label(note),].spacing(SPACE_2).padding(theme::PAD_PANEL);
        container(rocket).width(Fill).height(Fill).style(theme::panel).boxed()
    }

    fn status_strip(&self) -> Element<'_, Message> {
        // Order fixed by design/components/StatusStrip: Mass, Length, Ø ref, CG, CP, Stability, Motor.
        let length = (!self.rocket.parts().is_empty()).then(|| parts::mm(self.rocket.length()));
        let reference = self.rocket.reference_diameter().map(parts::mm);
        let cells = ["MASS", "LEN", "Ø REF", "CG", "CP", "STAB", "MOTOR"].map(|key| {
            let value = match key {
                "LEN" => length.clone(),
                "Ø REF" => reference.clone(),
                _ => None,
            };
            container(row![label(key), text(value.unwrap_or_else(|| "—".into())).font(MONO).size(TEXT_BODY)].spacing(SPACE_2).align_y(iced::Center))
                .padding([0.0, SPACE_3])
                .center_y(Fill)
                .style(theme::cell)
                .boxed()
        });
        let notice = self.notice.as_ref().map(|(message, _)| {
            container(text(message).size(TEXT_BODY).style(|theme: &Theme| text::Style { color: Some(theme::tokens(theme).ok), selection: None }))
                .padding([0.0, SPACE_3])
                .center_y(Fill)
                .boxed()
        });
        let strip = cells.into_iter().chain([Space::new().width(Fill).boxed()]).chain(notice);
        container(row(strip)).height(STATUSBAR_H).width(Fill).style(theme::bar).boxed()
    }
}

/// An uppercase mono section label in `ink-faint`.
fn label(s: &str) -> Element<'_, Message> {
    text(s).font(MONO).size(TEXT_LABEL).style(|theme: &Theme| text::Style { color: Some(theme::tokens(theme).ink_faint), selection: None }).boxed()
}

/// A row of mutually exclusive options: `bg-100` strip, active segment `bg-300` with a 2 px `accent` underline.
fn segmented<'a, T: Copy + PartialEq + 'a>(options: &[(T, &'a str)], selected: T, on_select: fn(T) -> Message) -> Element<'a, Message> {
    let segments = options.iter().map(|&(value, name)| {
        let active = value == selected;
        column![
            button(text(name).size(TEXT_BODY)).height(CONTROL_H - 2.0).padding([0.0, SPACE_3]).style(theme::segment(active)).on_press(on_select(value)),
            container(Space::new()).width(Length::Fill).height(2).style(theme::underline(active)),
        ]
        .width(Length::Shrink)
        .boxed()
    });
    container(row(segments).spacing(1)).padding(1).style(theme::segmented_frame).boxed()
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::NoseShape;

    /// An app holding a nose, a tube and a transition (fore to aft).
    fn app() -> (App, [PartId; 3]) {
        let mut app = App::new();
        app.place(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }, None);
        app.place(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }, None);
        app.place(2, PartKind::Transition { length: 0.1, fore_diameter: 0.1, aft_diameter: 0.06 }, None);
        let ids = [0, 1, 2].map(|i| app.rocket.parts()[i].id);
        (app, ids)
    }

    #[test]
    fn select_and_delete() {
        let (mut app, [_, tube, _]) = app();
        app.update(Message::Select(Some(Pick::Axial(tube))));
        assert_eq!(app.selected, Some(tube));
        app.update(Message::Delete);
        assert!(app.rocket.part(tube).is_none() && app.selected.is_none());
        assert_eq!(app.rocket.parts().len(), 2);
        // Clicking empty space deselects.
        app.update(Message::Select(None));
        app.update(Message::Delete);
        assert_eq!(app.rocket.parts().len(), 2);
    }

    #[test]
    fn grabbing_a_part_moves_it_and_cancelling_puts_it_back() {
        let (mut app, [nose, tube, transition]) = app();
        let before = app.rocket.parts().to_vec();
        app.update(Message::Grab(Pick::Axial(tube)));
        assert!(app.rocket.part(tube).is_none() && app.drag.is_some() && app.selected == Some(tube));
        app.update(Message::Cancel);
        assert_eq!(app.rocket.parts(), before.as_slice());

        // No snap target on release: back where it was.
        app.update(Message::Grab(Pick::Axial(transition)));
        app.update(Message::Drop);
        assert_eq!(app.rocket.parts(), before.as_slice());

        // Dropped on a node: moved, keeping its id, and matched to its new neighbours.
        app.update(Message::Grab(Pick::Axial(transition)));
        app.update(Message::Snap(Some(1)));
        assert!(app.drag.as_ref().is_some_and(|d| d.snap == Some(1)));
        app.update(Message::Drop);
        assert_eq!(app.rocket.index_of(transition), Some(1));
        assert_eq!(app.rocket.index_of(nose), Some(0));
        assert_eq!(app.rocket.part(transition).unwrap().kind.fore_diameter(), 0.1);
        assert_eq!(app.rocket.part(transition).unwrap().kind.aft_diameter(), 0.1);
        assert!(app.drag.is_none());
    }

    #[test]
    fn resizing_changes_only_that_part() {
        let (mut app, [_, tube, transition]) = app();
        let kind = PartKind::BodyTube { length: 0.7, diameter: 0.12 };
        app.update(Message::Resize(tube, kind.clone()));
        assert_eq!(app.rocket.part(tube).unwrap().kind, kind);
        assert_eq!(app.rocket.part(transition).unwrap().kind.fore_diameter(), 0.1);
        // The kind of part cannot change through a resize.
        app.update(Message::Resize(tube, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.1, diameter: 0.1 }));
        assert_eq!(app.rocket.part(tube).unwrap().kind, kind);
    }

    fn target(parent: PartId, offset: f64, angle: f64) -> SurfaceTarget {
        SurfaceTarget { parent, offset, angle }
    }

    #[test]
    fn a_click_attaches_radial_parts_to_the_aftmost_tube_with_the_symmetry_count() {
        let (mut app, [_, tube, _]) = app();
        app.category = Category::Fins;
        app.symmetry = 4;
        app.update(Message::RowPress(0));
        app.update(Message::Released);
        let [fins] = app.rocket.radials() else { panic!("one radial part") };
        assert_eq!((fins.parent, fins.count, fins.angle), (tube, 4, 0.0));
        // Flush with the aft end of the tube (0.5 long, 0.12 root chord).
        assert!((fins.offset - (0.5 - 0.12)).abs() < 1e-9);
        // With nothing to attach to, a click does nothing.
        let mut empty = App::new();
        empty.category = Category::Fins;
        empty.update(Message::RowPress(1));
        empty.update(Message::Released);
        assert!(empty.rocket.radials().is_empty());
    }

    #[test]
    fn dragging_a_radial_part_onto_the_surface_creates_a_group_with_the_live_count() {
        let (mut app, [_, tube, _]) = app();
        app.category = Category::Fins;
        app.symmetry = 3;
        app.start_bin_drag(1);
        assert!(app.radial_drag.as_ref().is_some_and(|d| d.count == 3 && d.row == Some(1)));
        // X cycles the count (and the footer's, for the next one); Shift+X goes back.
        app.update(Message::CycleSymmetry(true));
        assert_eq!((app.radial_drag.as_ref().unwrap().count, app.symmetry), (4, 4));
        app.update(Message::CycleSymmetry(false));
        app.update(Message::CycleSymmetry(false));
        assert_eq!(app.radial_drag.as_ref().unwrap().count, 2);
        app.update(Message::Surface(Some(target(tube, 0.1, 1.0))));
        assert!(app.radial_drag.as_ref().is_some_and(|d| d.target == Some(target(tube, 0.1, 1.0))));
        app.update(Message::DropRadial);
        let [lug] = app.rocket.radials() else { panic!("one radial part") };
        assert_eq!((lug.parent, lug.count, lug.offset, lug.angle), (tube, 2, 0.1, 1.0));
        assert!(app.radial_drag.is_none());

        // No target: cancelled. Escape: cancelled.
        app.start_bin_drag(2);
        app.update(Message::DropRadial);
        app.start_bin_drag(2);
        app.update(Message::Surface(Some(target(tube, 0.1, 1.0))));
        app.update(Message::Cancel);
        assert_eq!(app.rocket.radials().len(), 1);
    }

    #[test]
    fn a_radial_group_can_be_picked_up_moved_and_put_back_and_deleted() {
        let (mut app, [_, tube, transition]) = app();
        let id = app.rocket.attach_radial(tube, 0.0, 0.0, 3, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap();
        let before = app.rocket.radials().to_vec();
        app.update(Message::Grab(Pick::Radial { id, instance: 1 }));
        assert!(app.rocket.radials().is_empty() && app.selected == Some(id));
        // Cancelled, or dropped with no target: back as it was.
        app.update(Message::Cancel);
        assert_eq!(app.rocket.radials(), before.as_slice());
        app.update(Message::Grab(Pick::Radial { id, instance: 1 }));
        app.update(Message::DropRadial);
        assert_eq!(app.rocket.radials(), before.as_slice());
        // Dropped on another part: moved there, same id, same count.
        app.update(Message::Grab(Pick::Radial { id, instance: 1 }));
        app.update(Message::Surface(Some(target(transition, 0.0, 2.0))));
        app.update(Message::DropRadial);
        let moved = app.rocket.radial(id).unwrap();
        assert_eq!((moved.parent, moved.count, moved.angle), (transition, 3, 2.0));
        // Deleting the selected group removes all of it; its ids are not reused.
        app.update(Message::Delete);
        assert!(app.rocket.radials().is_empty() && app.selected.is_none());
        assert_eq!(app.rocket.parts().len(), 3);
    }

    #[test]
    fn moving_or_deleting_an_axial_part_takes_its_radial_parts_along() {
        let (mut app, [nose, tube, transition]) = app();
        let id = app.rocket.attach_radial(tube, 0.3, 0.5, 2, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap();
        // Grab the tube: its radial part rides along (out of the rocket) and returns with it, offset unchanged.
        app.update(Message::Grab(Pick::Axial(tube)));
        assert!(app.rocket.radials().is_empty());
        app.update(Message::Cancel);
        assert_eq!(app.rocket.radial(id).map(|r| (r.parent, r.offset)), Some((tube, 0.3)));
        // Move the transition forward of the tube: the tube's radial part is unaffected.
        app.update(Message::Grab(Pick::Axial(transition)));
        app.update(Message::Snap(Some(1)));
        app.update(Message::Drop);
        assert_eq!(app.rocket.index_of(tube), Some(2));
        assert_eq!(app.rocket.radial(id).map(|r| r.parent), Some(tube));
        // Resizing the parent keeps the radial part on it.
        app.update(Message::Resize(tube, PartKind::BodyTube { length: 0.2, diameter: 0.1 }));
        assert!((app.rocket.radial(id).unwrap().offset - 0.15).abs() < 1e-9);
        // Deleting the tube deletes its radial part.
        app.update(Message::Select(Some(Pick::Axial(tube))));
        app.update(Message::Delete);
        assert!(app.rocket.radials().is_empty() && app.rocket.index_of(nose) == Some(0));
    }

    #[test]
    fn resizing_a_fin_set_changes_the_group() {
        let (mut app, [_, tube, _]) = app();
        let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 };
        let id = app.rocket.attach_radial(tube, 0.0, 0.0, 4, fins).unwrap();
        let longer = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.09, sweep: 0.03, thickness: 0.003 };
        app.update(Message::ResizeRadial(id, longer.clone()));
        assert_eq!((app.rocket.radial(id).unwrap().kind.clone(), app.rocket.radial(id).unwrap().count), (longer, 4));
    }
}
