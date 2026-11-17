//! The flow viewer: the meridian slice of a solved (or solving) flow, mirrored about the axis so the whole silhouette shows, coloured
//! by Mach number, pressure coefficient or a schlieren (the log of the density gradient), with the body drawn solid on top, a colour
//! bar, a grid toggle, zoom and pan, a hover readout, and under it the Cp along the body sharing its x axis.
//!
//! # Rendering
//!
//! The coloured cells are drawn by the GPU ([`gpu`]): the grid's nodes and triangles go up once per picture and pan, zoom and the
//! choice of scalar change a uniform block. An iced `canvas` was the other candidate; it would tessellate every cell on the CPU
//! whenever the view moved, which a 27 000-cell coarse grid (and four times that on the medium one) does not afford at 60 frames a
//! second, where the GPU draws either in about a millisecond (measured in `gpu.rs`'s test, noted in `ARCHITECTURE.md`). A `canvas` is on top for
//! what is small: the body, the axis, the ticks, the hover readout and the wall Cp under it, and it takes the mouse.
//!
//! The picture is built (the three scalars, the colour ranges, the vertices) in [`Prepared`] once per picture. [`State`] holds the
//! view (scalar, grid, camera) and is session-only; the fields themselves live in [`super::cfd::FieldStore`] and the live
//! snapshot of a running solve in [`super::Solving`].
//!
//! The pointer is in a screen transform [`Xf`] with the mirror in it: screen y runs down, the radius up, and a pointer below the axis
//! reads the same flow as its mirror above it.

pub(in crate::analyze) mod axi;
mod gpu;
mod gpu3;
mod map;
mod meridian;
mod prepared;
mod scene3;

use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::widget::{column, container, row, stack, text, Space};
use iced::{alignment, mouse, Color, Element, Fill, Length, Pixels, Point, Rectangle, Renderer, Size, Theme, Vector, Widget};
use std::sync::Arc;
use std::time::{Duration, Instant};

use bowshock_core::Field3;
use bowshock_model::CfdTable;

use super::solve::thousands;
use super::Message;
use crate::app::{ui, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{CONTROL_H, SPACE_1, SPACE_2, SPACE_3};
use crate::units::{Quantity, Unit};
use crate::widgets::button::{button as push, Weight};
use crate::widgets::field::rule;
use crate::widgets::plot::ticks::{fmt_fixed, nice_ticks, range_decimals};
use crate::widgets::segmented::segmented;

pub use map::Scalar;
pub use prepared::Prepared;

/// Height of the wall Cp plot under the picture, axis labels included.
pub const WALL_H: f32 = 84.0;
const PLOT_TOP: f32 = 8.0;
const AXIS_H: f32 = 16.0;
const DOUBLE_CLICK: Duration = Duration::from_millis(350);

// ---------------------------------------------------------------------------------------------------------------------
// The view: a camera and the screen transform
// ---------------------------------------------------------------------------------------------------------------------

/// Where the view looks: the point `(x, r)` at the middle of the picture and how many logical pixels one unit of length (a metre)
/// takes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    /// Axial position and signed radius at the centre.
    pub centre: (f64, f64),
    /// Logical pixels per metre.
    pub scale: f64,
}

impl Camera {
    /// The camera that shows `(x0, x1, r_max)` (the box of [`Prepared::fit_box`], mirrored about the axis) in `size` with a margin.
    pub fn fit(box_: (f32, f32, f32), size: Size) -> Camera {
        let (x0, x1, r) = (f64::from(box_.0), f64::from(box_.1), f64::from(box_.2));
        let (w, h) = (f64::from(size.width.max(1.0)), f64::from(size.height.max(1.0)));
        let scale = (w / (x1 - x0).max(1e-9)).min(h / (2.0 * r).max(1e-9)) * 0.96;
        Camera { centre: (0.5 * (x0 + x1), 0.0), scale }
    }

    /// The camera `factor` times closer with the point of the picture under `at` held still, within `limits` of scale.
    pub fn zoomed(&self, factor: f64, at: Point, size: Size, limits: (f64, f64)) -> Camera {
        let before = Xf { camera: *self, size };
        let (x, r) = before.to_data(at);
        let scale = (self.scale * factor).clamp(limits.0, limits.1);
        let (w, h) = (f64::from(size.width), f64::from(size.height));
        Camera { centre: (x - (f64::from(at.x) - w / 2.0) / scale, r + (f64::from(at.y) - h / 2.0) / scale), scale }
    }

    /// The camera after the picture was dragged by `by` pixels.
    pub fn panned(&self, by: Vector) -> Camera {
        Camera { centre: (self.centre.0 - f64::from(by.x) / self.scale, self.centre.1 + f64::from(by.y) / self.scale), scale: self.scale }
    }
}

/// The screen transform of the picture: data `(x, r)` to logical pixels in a rectangle of `size`, and back. `r` is signed: the picture
/// is mirrored about the axis, so a point below the axis has a negative `r`.
#[derive(Debug, Clone, Copy)]
pub struct Xf {
    pub camera: Camera,
    pub size: Size,
}

impl Xf {
    /// Pixels of the point `(x, r)`.
    pub fn to_screen(self, x: f64, r: f64) -> Point {
        let (w, h) = (f64::from(self.size.width), f64::from(self.size.height));
        let c = &self.camera;
        Point::new((w / 2.0 + (x - c.centre.0) * c.scale) as f32, (h / 2.0 - (r - c.centre.1) * c.scale) as f32)
    }

    /// The point `(x, r)` under the pixel `p` (`r` signed).
    pub fn to_data(self, p: Point) -> (f64, f64) {
        let (w, h) = (f64::from(self.size.width), f64::from(self.size.height));
        let c = &self.camera;
        ((f64::from(p.x) - w / 2.0) / c.scale + c.centre.0, -(f64::from(p.y) - h / 2.0) / c.scale + c.centre.1)
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// State and messages
// ---------------------------------------------------------------------------------------------------------------------

/// What the solve screen gives most of its room to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Emphasis {
    /// The flow picture, the Cd curve a strip under it.
    #[default]
    Flow,
    /// The Cd curve, the flow picture a strip over it.
    Curve,
}

/// The flow viewer's messages.
#[derive(Debug, Clone)]
pub enum Msg {
    /// Colour by another quantity.
    Scalar(Scalar),
    /// Show or hide the cell edges.
    Grid,
    /// Frame the body again.
    Fit,
    /// The picture was dragged by this many pixels.
    Pan(Vector),
    /// Zoom by `factor` about the pixel `at` of the picture.
    Zoom { factor: f32, at: Point },
    /// The pointer is at this pixel of the viewer (`None`: it left).
    Hover(Option<Point>),
    /// The picture's rectangle has this size.
    Resized(Size),
    /// The Flow field view of a finished table was opened or closed.
    Open(bool),
    /// The Mach number of the Flow field view moves by this many table points.
    Step(i32),
    /// The Mach number of the Flow field view is this one.
    Pick(f64),
    /// The solve screen's emphasis.
    Emphasis(Emphasis),
}

/// What is under the pointer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hover {
    /// The pixel in the viewer.
    pub at: Point,
    /// The point of the meridian plane (`r` is a radius).
    pub x: f64,
    pub r: f64,
    /// The pointer is over the picture (rather than the wall Cp plot under it).
    pub in_field: bool,
    /// The flow cell there, if any.
    pub cell: Option<usize>,
}

/// The viewer's state: session-only, not part of the document.
pub struct State {
    /// What the picture is coloured by.
    pub scalar: Scalar,
    /// Whether the cell edges are drawn.
    pub grid: bool,
    /// The Flow field view of the finished table is on show.
    pub open: bool,
    /// What the solve screen emphasises.
    pub emphasis: Emphasis,
    /// The Mach number the Flow field view shows (a node of the table).
    pub selected: Option<f64>,
    /// The Mach numbers of the table on show, ascending, for the stepper.
    pub machs: Vec<f64>,
    camera: Camera,
    /// The camera is the fit of the picture (nobody zoomed or panned since).
    fitted: bool,
    /// The picture's rectangle (without the wall plot).
    size: Size,
    picture: Option<Arc<Prepared>>,
    hover: Option<Hover>,
}

impl Default for State {
    fn default() -> Self {
        State::new()
    }
}

impl State {
    /// A viewer with nothing to show: Mach, no grid, fitted.
    pub fn new() -> State {
        State {
            scalar: Scalar::Mach,
            grid: false,
            open: false,
            emphasis: Emphasis::Flow,
            selected: None,
            machs: Vec::new(),
            camera: Camera { centre: (0.0, 0.0), scale: 1.0 },
            fitted: true,
            size: Size::ZERO,
            picture: None,
            hover: None,
        }
    }

    /// Shows `field` (the picture the screen wants now), preparing it if it is not the one already on show.
    pub fn set_field(&mut self, field: Option<&Arc<Field3>>) {
        match (field, &self.picture) {
            (Some(f), Some(p)) if Arc::ptr_eq(f, &p.field) => {}
            (Some(f), _) => {
                self.picture = Some(Arc::new(Prepared::new(f.clone())));
                self.hover = None;
                self.refit();
            }
            (None, Some(_)) => self.picture = None,
            (None, None) => {}
        }
    }

    /// `(Mach number, iteration, cells along i, cells along j)` of the picture on show.
    pub fn picture_info(&self) -> Option<(f64, usize, usize, usize)> {
        self.picture.as_ref().map(|p| (p.field.mach, p.field.iteration, p.field.ni, p.field.nj))
    }

    /// The Mach number of the picture on show.
    pub fn picture_mach(&self) -> Option<f64> {
        self.picture.as_ref().map(|p| p.field.mach)
    }

    /// Frames the body when the camera is still the fit.
    fn refit(&mut self) {
        if let (true, Some(p)) = (self.fitted && self.size.width > 1.0, &self.picture) {
            self.camera = Camera::fit(p.fit_box(), self.size);
        }
    }

    /// Scale limits of the zoom: from a tenth of the fit to a few hundred times it.
    fn limits(&self) -> (f64, f64) {
        let fit = self.picture.as_ref().map_or(1.0, |p| Camera::fit(p.fit_box(), self.size).scale);
        (0.1 * fit, 400.0 * fit)
    }

    /// Handles a message of the viewer.
    pub fn update(&mut self, msg: Msg) {
        match msg {
            Msg::Scalar(s) => self.scalar = s,
            Msg::Grid => self.grid = !self.grid,
            Msg::Fit => {
                self.fitted = true;
                self.refit();
            }
            Msg::Pan(by) => {
                self.camera = self.camera.panned(by);
                self.fitted = false;
            }
            Msg::Zoom { factor, at } => {
                self.camera = self.camera.zoomed(f64::from(factor), at, self.size, self.limits());
                self.fitted = false;
            }
            Msg::Hover(at) => self.hover = at.and_then(|at| self.hover_at(at)),
            Msg::Resized(size) => {
                self.size = size;
                self.refit();
            }
            Msg::Open(on) => self.open = on,
            Msg::Step(by) => {
                let at = self.selected.and_then(|m| self.machs.iter().position(|x| (x - m).abs() < 1e-9)).unwrap_or(0) as i32;
                let to = (at + by).clamp(0, self.machs.len().saturating_sub(1) as i32) as usize;
                self.selected = self.machs.get(to).copied();
            }
            Msg::Pick(mach) => self.selected = Some(mach),
            Msg::Emphasis(e) => self.emphasis = e,
        }
    }

    fn hover_at(&self, at: Point) -> Option<Hover> {
        let p = self.picture.as_ref()?;
        let in_field = at.y < self.size.height;
        let (x, r) = Xf { camera: self.camera, size: self.size }.to_data(at);
        let cell = if in_field { p.cell_at(x as f32, r.abs() as f32) } else { None };
        Some(Hover { at, x, r: r.abs(), in_field, cell })
    }

    #[cfg(test)]
    /// What is under the pointer now.
    pub fn hover(&self) -> Option<&Hover> {
        self.hover.as_ref()
    }

    /// The viewer: a row of controls and the colour bar, the picture with the wall Cp under it. `caption` is drawn in the corner of
    /// the picture; `notice` replaces the picture with a line of text (nothing to show); `compact` leaves the wall plot out.
    pub fn viewer<'a>(&'a self, ctx: &'a Ctx, caption: String, notice: Option<String>, compact: bool) -> Element<'a, Msg> {
        let scalars = segmented(&Scalar::ALL.map(|s| (s, s.name())), self.scalar, Msg::Scalar);
        let grid: Element<'a, Msg> = push("Grid", if self.grid { Weight::Primary } else { Weight::Secondary }).small().on_press(Msg::Grid).into();
        let fit: Element<'a, Msg> = push("Fit", Weight::Secondary).small().on_press(Msg::Fit).into();
        let unit = Quantity::Length.unit_for(ctx.units);
        let mut controls = row![scalars, grid, fit].spacing(SPACE_2).align_y(iced::Center);
        let body: Element<'a, Msg> = match (&self.picture, notice) {
            (Some(picture), None) => {
                controls = controls.push(Space::new().width(Fill).boxed()).push(
                    canvas::Canvas::new(Bar { picture, scalar: self.scalar, dark: ctx.dark, unit }).width(Length::Fixed(BAR_W)).height(CONTROL_H).boxed(),
                );
                let wall = if compact { 0.0 } else { WALL_H };
                let tokens = ctx.tokens();
                let draw = gpu::Draw {
                    picture: picture.clone(),
                    scalar: self.scalar,
                    dark: ctx.dark,
                    grid: self.grid,
                    line: Color { a: 0.28, ..tokens.ink },
                    centre: self.camera.centre,
                    scale: self.camera.scale,
                };
                let field = iced::widget::shader(draw).width(Fill).height(Fill);
                let under = column![field, Space::new().height(wall)];
                let overlay = Overlay { picture, camera: self.camera, wall, hover: self.hover.as_ref(), unit, caption };
                stack![under, canvas::Canvas::new(overlay).width(Fill).height(Fill)].width(Fill).height(Fill).boxed()
            }
            (_, notice) => container(text(notice.unwrap_or_else(|| "No picture yet".to_string())).size(13).style(theme::muted)).center(Fill).boxed(),
        };
        column![
            container(controls).padding([SPACE_1, 0.0]),
            container(body)
                .width(Fill)
                .height(Fill)
                .style(|t: &Theme| container::Style { border: theme::frame(theme::tokens(t).line_200), ..container::Style::default() })
        ]
        .spacing(SPACE_1)
        .height(Fill)
        .boxed()
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The overlay: body, axis, ticks, readout, wall Cp, mouse
// ---------------------------------------------------------------------------------------------------------------------

/// Width of the colour bar with its labels.
const BAR_W: f32 = 250.0;

fn put(f: &mut Frame, content: &str, at: Point, color: Color, size: f32, ax: iced::advanced::text::Alignment, ay: alignment::Vertical) {
    f.fill_text(Text { content: content.to_string(), position: at, color, size: Pixels(size), font: MONO, align_x: ax, align_y: ay, ..Text::default() });
}

fn line(color: Color, width: f32) -> Stroke<'static> {
    Stroke::default().with_color(color).with_width(width)
}

/// The mouse state of the overlay.
#[derive(Default)]
pub struct Interaction {
    drag: Option<Point>,
    /// The drag began over the wall plot: it moves the picture along x only.
    drag_wall: bool,
    reported: Size,
    hovering: bool,
    last_click: Option<(Instant, Point)>,
}

struct Overlay<'a> {
    picture: &'a Prepared,
    camera: Camera,
    /// Height of the wall plot under the picture (0: none).
    wall: f32,
    hover: Option<&'a Hover>,
    unit: &'static Unit,
    caption: String,
}

impl Overlay<'_> {
    fn field(&self, bounds: Rectangle) -> Size {
        Size::new(bounds.width, (bounds.height - self.wall).max(1.0))
    }
}

impl canvas::Program<Msg> for Overlay<'_> {
    type State = Interaction;

    fn update(&self, st: &mut Interaction, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Msg>> {
        let field = self.field(bounds);
        if st.reported != field {
            st.reported = field;
            return Some(Action::publish(Msg::Resized(field)));
        }
        match event {
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let p = cursor.position_in(bounds)?;
                let now = Instant::now();
                if st.last_click.is_some_and(|(then, at)| now.duration_since(then) < DOUBLE_CLICK && at.distance(p) < 6.0) {
                    st.last_click = None;
                    return Some(Action::publish(Msg::Fit).and_capture());
                }
                st.last_click = Some((now, p));
                st.drag = Some(p);
                st.drag_wall = p.y >= field.height;
                Some(Action::capture())
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => st.drag.take().map(|_| Action::capture()),
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                let local = cursor.position_in(bounds);
                if let (Some(from), Some(p)) = (st.drag, local) {
                    st.drag = Some(p);
                    let by = Vector::new(p.x - from.x, if st.drag_wall { 0.0 } else { p.y - from.y });
                    return Some(Action::publish(Msg::Pan(by)).and_capture());
                }
                match local {
                    Some(p) => {
                        st.hovering = true;
                        Some(Action::publish(Msg::Hover(Some(p))))
                    }
                    None => std::mem::take(&mut st.hovering).then(|| Action::publish(Msg::Hover(None))),
                }
            }
            Event::Mouse(mouse::Event::CursorLeft) => {
                st.drag = None;
                std::mem::take(&mut st.hovering).then(|| Action::publish(Msg::Hover(None)))
            }
            Event::Mouse(mouse::Event::WheelScrolled { .. } | mouse::Event::PinchGesture { .. }) => {
                let p = cursor.position_in(bounds)?;
                let factor = match event {
                    Event::Mouse(mouse::Event::PinchGesture { delta, .. }) => crate::widgets::pinch::factor(*delta),
                    Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                        let lines = match delta {
                            mouse::ScrollDelta::Lines { y, .. } => *y,
                            mouse::ScrollDelta::Pixels { y, .. } => *y / 40.0,
                        };
                        1.176f32.powf(lines)
                    }
                    _ => return None,
                };
                // over the wall plot the zoom keeps the x under the pointer and the picture's own y
                let at = if p.y >= field.height { Point::new(p.x, field.height / 2.0) } else { p };
                Some(Action::publish(Msg::Zoom { factor, at }).and_capture())
            }
            _ => None,
        }
    }

    fn mouse_interaction(&self, st: &Interaction, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        if st.drag.is_some() {
            mouse::Interaction::Grabbing
        } else if cursor.is_over(bounds) {
            mouse::Interaction::Crosshair
        } else {
            mouse::Interaction::default()
        }
    }

    fn draw(&self, _: &Interaction, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        use iced::advanced::text::Alignment::{Center, Left, Right};
        let t = theme::tokens(theme);
        let field = self.field(bounds);
        let xf = Xf { camera: self.camera, size: field };
        let mut f = Frame::new(renderer, bounds.size());
        let picture = self.picture;

        // inside the picture: the axis, the body on top of the flow, the pointer
        f.with_clip(Rectangle::new(Point::ORIGIN, field), |f| {
            let axis = xf.to_screen(0.0, 0.0).y.floor() + 0.5;
            f.stroke(
                &Path::line(Point::new(0.0, axis), Point::new(field.width, axis)),
                Stroke { line_dash: LineDash { segments: &[5.0, 4.0], offset: 0 }, ..line(t.ink_muted.scale_alpha(0.6), 1.0) },
            );
            if picture.outline.len() > 2 {
                let body = Path::new(|b| {
                    let up = |p: &[f32; 2]| xf.to_screen(f64::from(p[0]), f64::from(p[1]));
                    let down = |p: &[f32; 2]| xf.to_screen(f64::from(p[0]), -f64::from(p[1]));
                    b.move_to(up(&picture.outline[0]));
                    picture.outline[1..].iter().for_each(|p| b.line_to(up(p)));
                    picture.outline.iter().rev().for_each(|p| b.line_to(down(p)));
                    b.close();
                });
                f.fill(&body, t.part_body);
                f.stroke(&body, line(t.ink, 1.2));
            }
            if let Some(h) = self.hover.filter(|h| h.in_field) {
                f.stroke(&Path::circle(h.at, 4.0), line(t.accent, 1.5));
            }
        });
        f.stroke(&Path::rectangle(Point::ORIGIN, field), line(t.line_200, 1.0));

        // the x axis the picture and the wall plot share: ticks in the user's unit
        let (x_lo, x_hi) = (xf.to_data(Point::ORIGIN).0, xf.to_data(Point::new(field.width, 0.0)).0);
        let (u_lo, u_hi) = (self.unit.from_si(x_lo), self.unit.from_si(x_hi));
        let decimals = range_decimals(u_hi - u_lo);
        let x_ticks: Vec<(f32, String)> =
            nice_ticks(u_lo, u_hi, 8).into_iter().map(|v| (xf.to_screen(self.unit.to_si(v), 0.0).x.floor() + 0.5, fmt_fixed(v, decimals.min(3)))).collect();
        for (x, _) in &x_ticks {
            f.stroke(&Path::line(Point::new(*x, field.height - 5.0), Point::new(*x, field.height)), line(t.line_200, 1.0));
        }

        // the readout under the pointer and the caption
        let panel = |f: &mut Frame, s: &str, at: Point, right: bool| {
            let w = 6.6 * s.chars().count() as f32 + 12.0;
            let x = if right { at.x - w } else { at.x };
            f.fill_rectangle(Point::new(x, at.y), Size::new(w, 18.0), t.bg_100.scale_alpha(0.86));
            put(f, s, Point::new(x + 6.0, at.y + 9.0), t.ink, 11.0, Left, alignment::Vertical::Center);
        };
        // the readout has the corner; the caption keeps the other one only if the two fit side by side
        let readout = self.readout();
        if !self.caption.is_empty() && 6.6 * (self.caption.chars().count() + readout.chars().count()) as f32 + 48.0 < field.width {
            panel(&mut f, &self.caption, Point::new(field.width - 8.0, 8.0), true);
        }
        panel(&mut f, &readout, Point::new(8.0, 8.0), false);

        // the wall Cp
        if self.wall > 0.0 {
            let (top, bottom) = (field.height + PLOT_TOP, bounds.height - AXIS_H);
            let wall = &picture.wall;
            let (lo, hi) = wall.iter().fold((0.0f64, 0.0f64), |(l, h), p| (l.min(p.1), h.max(p.1)));
            let (lo, hi) = if hi - lo < 0.1 { (lo - 0.05, hi + 0.05) } else { (lo, hi) };
            let pad = 0.06 * (hi - lo);
            let (lo, hi) = (lo - pad, hi + pad);
            let py = |cp: f64| top + (1.0 - ((cp - lo) / (hi - lo)) as f32) * (bottom - top);
            f.with_clip(Rectangle::new(Point::new(0.0, field.height), Size::new(bounds.width, self.wall)), |f| {
                for (x, _) in &x_ticks {
                    f.stroke(&Path::line(Point::new(*x, top), Point::new(*x, bottom)), line(t.line_100, 1.0));
                }
                for v in nice_ticks(lo, hi, 3) {
                    let y = py(v).floor() + 0.5;
                    f.stroke(&Path::line(Point::new(0.0, y), Point::new(bounds.width, y)), line(if v == 0.0 { t.line_200 } else { t.line_100 }, 1.0));
                    put(f, &fmt_fixed(v, 1), Point::new(4.0, y - 1.0), t.ink_faint, 10.0, Left, alignment::Vertical::Bottom);
                }
                if wall.len() > 1 {
                    let path = Path::new(|b| {
                        let at = |p: &(f64, f64)| Point::new(xf.to_screen(p.0, 0.0).x, py(p.1));
                        b.move_to(at(&wall[0]));
                        wall[1..].iter().for_each(|p| b.line_to(at(p)));
                    });
                    f.stroke(&path, line(t.ink, 1.5));
                }
                put(f, "Cp at the wall", Point::new(bounds.width - 6.0, top + 1.0), t.ink_faint, 10.0, Right, alignment::Vertical::Top);
                if let Some(h) = self.hover {
                    let x = xf.to_screen(h.x, 0.0).x;
                    f.stroke(&Path::line(Point::new(x, top), Point::new(x, bottom)), line(t.accent.scale_alpha(0.8), 1.0));
                    if let Some(cp) = picture.wall_cp_at(h.x) {
                        f.stroke(&Path::circle(Point::new(x, py(cp)), 3.0), line(t.accent, 1.5));
                    }
                }
            });
            for (x, label) in &x_ticks {
                put(&mut f, label, Point::new(*x, bottom + 3.0), t.ink_faint, 10.0, Center, alignment::Vertical::Top);
            }
            put(&mut f, self.unit.symbol, Point::new(bounds.width - 4.0, bottom + 3.0), t.ink_faint, 10.0, Right, alignment::Vertical::Top);
        } else {
            for (x, label) in &x_ticks {
                put(&mut f, label, Point::new(*x, field.height - 8.0), t.ink_faint, 10.0, Center, alignment::Vertical::Bottom);
            }
        }
        vec![f.into_geometry()]
    }
}

impl Overlay<'_> {
    /// The line under the pointer: where it is, and what the flow there is.
    fn readout(&self) -> String {
        let Some(h) = self.hover else { return "hover for values · wheel zooms · drag pans".to_string() };
        let u = self.unit;
        let place = format!("x {:.1} {} · r {:.1} {}", u.from_si(h.x), u.symbol, u.from_si(h.r), u.symbol);
        match h.cell.filter(|_| h.in_field) {
            Some(c) => {
                let p = self.picture;
                // the density gradient per unit of the user's length, over the free-stream density
                let grad = f64::from(p.gradient[c]) * u.to_si;
                format!("{place} · M {:.2} · Cp {:+.2} · ∇ρ/ρ∞ {} /{}", p.mach[c], p.cp[c], compact_number(grad), u.symbol)
            }
            None if h.in_field => format!("{place} · body"),
            None => match self.picture.wall_cp_at(h.x) {
                Some(cp) => format!("x {:.1} {} · wall Cp {cp:+.3}", u.from_si(h.x), u.symbol),
                None => format!("x {:.1} {}", u.from_si(h.x), u.symbol),
            },
        }
    }
}

/// A number in three characters or so: `0.42`, `12`, `3.4e2`.
fn compact_number(v: f64) -> String {
    let a = v.abs();
    if a == 0.0 {
        "0".to_string()
    } else if (0.01..10.0).contains(&a) {
        format!("{v:.2}")
    } else if (10.0..1000.0).contains(&a) {
        format!("{v:.0}")
    } else {
        format!("{v:.1e}")
    }
}

/// The colour bar with its end labels.
struct Bar<'a> {
    picture: &'a Prepared,
    scalar: Scalar,
    dark: bool,
    unit: &'static Unit,
}

impl Bar<'_> {
    /// The labels at the two ends of the bar.
    fn labels(&self) -> (String, String) {
        let (lo, hi) = self.picture.ranges[self.scalar.index()];
        match self.scalar {
            Scalar::Mach => ("0".to_string(), format!("{hi:.2}")),
            Scalar::Cp => (format!("{lo:+.2}"), format!("{hi:+.2}")),
            Scalar::Schlieren => {
                let per = |log: f32| compact_number(10f64.powf(f64::from(log)) * self.unit.to_si);
                (per(lo), format!("{} /{}", per(hi), self.unit.symbol))
            }
        }
    }
}

impl canvas::Program<Msg> for Bar<'_> {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        use iced::advanced::text::Alignment::{Left, Right};
        let t = theme::tokens(theme);
        let mut f = Frame::new(renderer, bounds.size());
        let (lo, hi) = self.labels();
        let (x0, x1) = (54.0, bounds.width - 96.0);
        let (y0, h) = (bounds.height / 2.0 - 5.0, 10.0);
        let steps = 48;
        let stops = self.scalar.stops(self.dark);
        for k in 0..steps {
            let (a, b) = (x0 + (x1 - x0) * k as f32 / steps as f32, x0 + (x1 - x0) * (k + 1) as f32 / steps as f32);
            f.fill_rectangle(Point::new(a, y0), Size::new(b - a + 0.5, h), map::color(stops, (k as f32 + 0.5) / steps as f32));
        }
        f.stroke(&Path::rectangle(Point::new(x0, y0), Size::new(x1 - x0, h)), line(t.line_200, 1.0));
        if self.scalar == Scalar::Cp {
            let mid = (x0 + x1) / 2.0;
            f.stroke(&Path::line(Point::new(mid, y0 + h), Point::new(mid, y0 + h + 3.0)), line(t.ink_muted, 1.0));
        }
        put(&mut f, &lo, Point::new(x0 - 5.0, bounds.height / 2.0), t.ink_muted, 10.5, Right, alignment::Vertical::Center);
        put(&mut f, &hi, Point::new(x1 + 5.0, bounds.height / 2.0), t.ink_muted, 10.5, Left, alignment::Vertical::Center);
        vec![f.into_geometry()]
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The Mach number picker of the Flow field view
// ---------------------------------------------------------------------------------------------------------------------

/// The Cd(Mach) of the table as a strip to choose the Mach number from: a point is filled when it has a field and ringed when it is the
/// chosen one.
struct Picker {
    /// `(mach, cd, has a field)`.
    points: Vec<(f64, f64, bool)>,
    selected: Option<f64>,
}

const PICK_PAD: f32 = 22.0;

impl Picker {
    /// Where Mach `m` and `cd` are in a strip of `bounds`.
    fn at(&self, bounds: Size, m: f64, cd: f64) -> Point {
        let (lo, hi) = self.points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| (l.min(p.0), h.max(p.0)));
        let top = self.points.iter().map(|p| p.1).fold(0.0f64, f64::max).max(0.05) * 1.15;
        let fx = if hi > lo { (m - lo) / (hi - lo) } else { 0.5 };
        Point::new(PICK_PAD + fx as f32 * (bounds.width - 2.0 * PICK_PAD), 8.0 + (1.0 - (cd / top) as f32) * (bounds.height - 26.0))
    }
}

impl canvas::Program<Msg> for Picker {
    type State = ();

    fn update(&self, _: &mut (), event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Msg>> {
        let Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) = event else { return None };
        let p = cursor.position_in(bounds)?;
        let nearest = self.points.iter().min_by(|a, b| {
            let (da, db) = ((self.at(bounds.size(), a.0, a.1).x - p.x).abs(), (self.at(bounds.size(), b.0, b.1).x - p.x).abs());
            da.total_cmp(&db)
        })?;
        Some(Action::publish(Msg::Pick(nearest.0)).and_capture())
    }

    fn mouse_interaction(&self, _: &(), bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            mouse::Interaction::Pointer
        } else {
            mouse::Interaction::default()
        }
    }

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        use iced::advanced::text::Alignment::{Center, Left};
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut f = Frame::new(renderer, size);
        let base = size.height - 18.0;
        f.stroke(&Path::line(Point::new(PICK_PAD, base + 0.5), Point::new(size.width - PICK_PAD, base + 0.5)), line(t.line_200, 1.0));
        if self.points.len() > 1 {
            let path = Path::new(|b| {
                b.move_to(self.at(size, self.points[0].0, self.points[0].1));
                self.points[1..].iter().for_each(|p| b.line_to(self.at(size, p.0, p.1)));
            });
            f.stroke(&path, line(t.ink_muted, 1.4));
        }
        let mut last_label: Option<f32> = None;
        for (m, cd, has) in &self.points {
            let at = self.at(size, *m, *cd);
            if *has {
                f.fill(&Path::circle(at, 3.8), t.ink);
            } else {
                f.fill(&Path::circle(at, 3.6), t.bg_000);
                f.stroke(&Path::circle(at, 3.6), line(t.ink_muted, 1.3));
            }
            // a label only where there is room for it
            if last_label.is_none_or(|x| at.x - x >= 34.0) {
                put(&mut f, &format!("{m:.2}"), Point::new(at.x, base + 4.0), t.ink_faint, 10.0, Center, alignment::Vertical::Top);
                last_label = Some(at.x);
            }
        }
        if let Some((m, cd, _)) = self.points.iter().find(|p| self.selected.is_some_and(|s| (s - p.0).abs() < 1e-9)) {
            let at = self.at(size, *m, *cd);
            f.stroke(&Path::circle(at, 7.0), line(t.accent, 1.8));
            f.fill(&Path::circle(at, 2.2), t.accent);
        }
        put(&mut f, "Cd", Point::new(2.0, 2.0), t.ink_faint, 10.0, Left, alignment::Vertical::Top);
        vec![f.into_geometry()]
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The screens
// ---------------------------------------------------------------------------------------------------------------------

/// The line that says why a point of `table` has no picture to show.
fn why_none(state: &super::State, table: &CfdTable) -> String {
    if state.cfd.status == super::cfd::Status::Stale {
        "No flow field: the geometry or the conditions changed since this table was solved. Re-solve for fields that match.".to_string()
    } else if state.cfd.fields.has_any(table) {
        "No flow field for this Mach number: its point has no field.".to_string()
    } else {
        "No flow fields for this table: it was loaded from a file, not solved in this session. Re-solve to see them.".to_string()
    }
}

/// The Mach number the Flow field view starts at: the node nearest Mach 2, where the nose shock is a clean cone to look at.
pub fn default_mach(machs: &[f64]) -> Option<f64> {
    machs.iter().copied().min_by(|a, b| (a - 2.0).abs().total_cmp(&(b - 2.0).abs()))
}

/// The Flow field view of the finished table, when it is on show: the picture of the selected Mach number, a Mach stepper and the table's
/// Cd curve to pick the Mach number from.
pub fn centre<'a>(state: &'a super::State, ctx: &'a Ctx) -> Option<Element<'a, Message>> {
    let flow = &state.flow;
    if !flow.open {
        return None;
    }
    let table = state.cfd.table.as_ref().filter(|_| state.cfd_ready())?;
    let selected = flow.selected;
    let pos = selected.and_then(|m| flow.machs.iter().position(|x| (x - m).abs() < 1e-9));
    let grid = match table.grid {
        bowshock_model::CfdGrid::Coarse => "coarse grid",
        bowshock_model::CfdGrid::Medium => "medium grid",
    };
    let at = |label: &'a str, by: i32, ok: bool| -> Element<'a, Message> {
        let b = push(label, Weight::Secondary).small();
        if ok {
            b.on_press(Message::Flow(Msg::Step(by))).into()
        } else {
            b.disabled("No further point").into()
        }
    };
    let stepper = row![
        at("◀", -1, pos.is_some_and(|p| p > 0)),
        text(selected.map_or_else(|| "—".to_string(), |m| format!("Mach {m:.2}"))).font(MONO).size(14).style(theme::ink),
        at("▶", 1, pos.is_some_and(|p| p + 1 < flow.machs.len())),
    ]
    .spacing(SPACE_2)
    .align_y(iced::Center);
    let back: Element<'a, Message> = push("Plots", Weight::Secondary).on_press(Message::Flow(Msg::Open(false))).into();
    let head =
        row![ui::label(format!("FLOW FIELD · CFD TABLE · {}", grid.to_uppercase())), Space::new().width(Fill), stepper, Space::new().width(SPACE_3), back]
            .align_y(iced::Center);

    let picture = flow.picture.as_ref();
    let notice = match (picture, selected) {
        (Some(_), _) => None,
        (None, Some(_)) => Some(why_none(state, table)),
        (None, None) => Some("No Mach number".to_string()),
    };
    let caption = picture
        .map_or_else(String::new, |p| format!("M {:.2} · {} iterations · {} × {} cells", p.field.mach, thousands(p.field.iteration), p.field.ni, p.field.nj));
    let viewer = flow.viewer(ctx, caption, notice, false).map(Message::Flow);

    let points: Vec<(f64, f64, bool)> = table.points.iter().map(|p| (p.mach, p.cd, state.cfd.fields.get(table, p.mach).is_some())).collect();
    let picker = canvas::Canvas::new(Picker { points, selected }).width(Fill).height(Length::Fixed(100.0)).boxed();
    let picker: Element<'a, Message> = picker.map(Message::Flow).boxed();
    let picker = column![container(ui::label("CD · CLICK A POINT")).padding([SPACE_1, 0.0]), picker];
    Some(
        column![
            container(head).padding([SPACE_2, SPACE_3]),
            container(viewer).padding([0.0, SPACE_3]).height(Fill),
            rule(),
            container(picker).padding([SPACE_1, SPACE_3]),
        ]
        .height(Fill)
        .boxed(),
    )
}

#[cfg(test)]
mod tests {
    use super::prepared::tests::{sized, small};
    use super::*;

    #[test]
    fn the_screen_transform_round_trips_a_point_above_and_below_the_axis() {
        let xf = Xf { camera: Camera { centre: (0.6, 0.02), scale: 840.0 }, size: Size::new(700.0, 240.0) };
        for (x, r) in [(0.0, 0.0), (0.35, 0.038), (1.1, -0.038), (-0.2, 0.15), (0.6, -0.02)] {
            let p = xf.to_screen(x, r);
            let (bx, br) = xf.to_data(p);
            assert!((bx - x).abs() < 1e-5 && (br - r).abs() < 1e-5, "({x}, {r}) -> {p:?} -> ({bx}, {br})");
        }
        // the mirror: the same distance above and below the axis is the same distance from the axis' pixel row
        let axis = xf.to_screen(0.3, 0.0).y;
        let (up, down) = (xf.to_screen(0.3, 0.05).y, xf.to_screen(0.3, -0.05).y);
        assert!(((axis - up) - (down - axis)).abs() < 1e-3 && up < axis && down > axis, "up {up}, axis {axis}, down {down}");
        // a pointer below the axis reads the radius of its mirror
        let below = xf.to_data(Point::new(300.0, axis + 20.0));
        let above = xf.to_data(Point::new(300.0, axis - 20.0));
        assert!((below.1 + above.1).abs() < 1e-6, "{below:?} {above:?}");
    }

    #[test]
    fn zooming_holds_the_point_under_the_pointer_and_panning_moves_the_picture_with_it() {
        let size = Size::new(700.0, 240.0);
        let cam = Camera { centre: (0.6, 0.0), scale: 500.0 };
        let at = Point::new(210.0, 70.0);
        let before = Xf { camera: cam, size }.to_data(at);
        let zoomed = cam.zoomed(2.0, at, size, (1.0, 1e6));
        assert_eq!(zoomed.scale, 1000.0);
        let after = Xf { camera: zoomed, size }.to_data(at);
        assert!((before.0 - after.0).abs() < 1e-9 && (before.1 - after.1).abs() < 1e-9);
        // the limits hold
        assert_eq!(cam.zoomed(1e9, at, size, (1.0, 2000.0)).scale, 2000.0);
        // a drag of 30 px to the right moves the picture right: the centre moves left by 30 px of data
        let panned = cam.panned(Vector::new(30.0, -10.0));
        let (xf0, xf1) = (Xf { camera: cam, size }, Xf { camera: panned, size });
        let p = xf0.to_screen(0.9, 0.04);
        let q = xf1.to_screen(0.9, 0.04);
        assert!((q.x - p.x - 30.0).abs() < 1e-3 && (q.y - p.y + 10.0).abs() < 1e-3, "{p:?} -> {q:?}");
    }

    #[test]
    fn fit_frames_the_body_with_its_wake_and_keeps_the_axis_in_the_middle() {
        let p = Prepared::new(Arc::new(small(2.0)));
        let size = Size::new(800.0, 200.0);
        let cam = Camera::fit(p.fit_box(), size);
        let xf = Xf { camera: cam, size };
        let (x0, x1, r) = p.fit_box();
        let (a, b) = (xf.to_screen(f64::from(x0), f64::from(r)), xf.to_screen(f64::from(x1), -f64::from(r)));
        assert!(a.x >= 0.0 && b.x <= size.width && a.y >= 0.0 && b.y <= size.height, "{a:?} {b:?}");
        assert!((xf.to_screen(0.0, 0.0).y - size.height / 2.0).abs() < 1e-3);
    }

    #[test]
    fn the_hover_finds_the_cell_for_a_pointer_above_or_below_the_axis_and_the_state_follows_its_messages() {
        let mut s = State::new();
        s.set_field(Some(&Arc::new(small(2.0))));
        s.update(Msg::Resized(Size::new(400.0, 300.0)));
        let xf = Xf { camera: s.camera, size: s.size };
        // the flow cell (0, 1) in the grid of `small`: x in [-1, 0], r in [1, 2]
        for r in [1.5, -1.5] {
            s.update(Msg::Hover(Some(xf.to_screen(-0.5, r))));
            let h = s.hover().expect("hover");
            assert_eq!(h.cell, Some(4), "r = {r}");
            assert!((h.r - 1.5).abs() < 1e-3 && (h.x + 0.5).abs() < 1e-3);
        }
        // the body has no cell
        s.update(Msg::Hover(Some(xf.to_screen(1.0, 0.5))));
        assert_eq!(s.hover().unwrap().cell, None);
        // below the picture is the wall plot: the x but no cell
        s.update(Msg::Hover(Some(Point::new(100.0, 310.0))));
        assert!(!s.hover().unwrap().in_field && s.hover().unwrap().cell.is_none());
        s.update(Msg::Hover(None));
        assert!(s.hover().is_none());

        // zooming leaves the fit; Fit returns to it
        let fit = s.camera;
        s.update(Msg::Zoom { factor: 2.0, at: Point::new(100.0, 100.0) });
        assert!(s.camera.scale > fit.scale && !s.fitted);
        s.update(Msg::Fit);
        assert_eq!(s.camera, fit);
        // the stepper walks the table's Mach numbers and stops at the ends
        s.machs = vec![0.5, 1.0, 2.0];
        s.selected = Some(1.0);
        s.update(Msg::Step(1));
        assert_eq!(s.selected, Some(2.0));
        s.update(Msg::Step(1));
        assert_eq!(s.selected, Some(2.0));
        s.update(Msg::Step(-5));
        assert_eq!(s.selected, Some(0.5));
        s.update(Msg::Pick(1.0));
        assert_eq!(s.selected, Some(1.0));
    }

    #[test]
    fn preparing_a_coarse_sized_field_takes_milliseconds() {
        let (ni, nj) = (220usize, 124usize);
        let n = ni * nj;
        let f = sized(ni, nj);
        let t = Instant::now();
        let p = Prepared::new(Arc::new(f));
        let took = t.elapsed();
        eprintln!("Prepared::new on {n} cells (debug build): {took:?}");
        assert!(p.triangles.len() > 6 * n / 2, "the cells of the flow are drawn");
        assert!(took < Duration::from_secs(2), "{took:?}");
        let t = Instant::now();
        let _ = p.cell_at(0.7, 0.2);
        eprintln!("cell_at on {n} cells (debug build): {:?}", t.elapsed());
    }
}
