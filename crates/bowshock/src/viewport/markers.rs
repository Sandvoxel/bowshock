//! The viewport's drawn overlays and the small HUD panels (design/README.md section 8, InsideViews, Staging, Navigation).
//!
//! [`Markers`] is one canvas above the model and the manipulation overlay: CG and CP markers on the centreline with value
//! labels and the margin bracket, the separation seams, diamonds and stage brackets of a staged rocket, the exploded view's
//! dashed leaders and the cutaway's section handle. It is interactive only where it draws something clickable (a separation's
//! diamond, the section handle, a part in the exploded view) and otherwise lets events through to the layers below.
//!
//! The framed HUD panels are widgets: [`legend`] (bottom left), [`hints`] (bottom centre) and the toolbar in `mod.rs`.

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use bowshock_model::{PartId, Rocket, Stage};
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::widget::{container, responsive, row, text, Space};
use iced::{alignment, keyboard, mouse, Element, Fill, Point, Rectangle, Renderer, Size, Theme, Vector, Widget};

use super::mesh::{Cut, Mesh};
use super::{Camera, InsideView, Message};
use crate::app::ui::bare;
use crate::app::Selection;
use crate::theme::{self, MONO};
use crate::tokens::{Tokens, SPACE_1, SPACE_2, SPACE_3, SPACE_4, TEXT_LABEL};
use crate::units::{Quantity, UnitSystem};
use crate::widgets::glyph;

/// Radius of the CG and CP glyphs.
const GLYPH_R: f32 = 6.5;
/// How close (px) a click must be to a diamond or the section handle to hit it.
const HIT: f32 = 10.0;
/// Half-diagonal of a separation's diamond.
const DIAMOND: f32 = 5.5;
/// Radius of the cutaway's section handle.
const HANDLE_R: f32 = 8.0;
/// Distance from the nose tip to the section handle, px.
const HANDLE_GAP: f32 = 26.0;
/// |view direction . rocket axis| where the CG / CP labels start to fade, and where they are gone.
const AXIAL_FADE_FROM: f32 = 0.7;
const AXIAL_FADE_TO: f32 = 0.9;

/// What the markers draw, and the state they read.
pub struct Markers<'a> {
    pub rocket: &'a Rocket,
    pub camera: Camera,
    pub mesh: &'a Mesh,
    /// `(cg, cp, margin in calibers)` of the phase on show, from the physics cache.
    pub balance: Option<(f64, f64, f64)>,
    /// The committed `(cg, cp)` while a change is previewed: drawn faint where they were, joined to the previewed ones.
    pub ghost: Option<(f64, f64)>,
    /// Whether the CG and the CP are drawn (the status strip's cells toggle them); the margin bracket needs both.
    pub show: (bool, bool),
    pub stages: &'a [Stage],
    /// The motors of each stage in the active configuration, `(stage index, "H128W")`.
    pub motors: Vec<(usize, String)>,
    pub selection: Selection,
    pub inside: InsideView,
    pub explode: f32,
    pub cut: Cut,
    pub units: UnitSystem,
    /// Clickable things react (off while a part is being dragged in).
    pub interactive: bool,
    /// This is the transparent layer above the manipulation overlay that takes clicks (it draws nothing); the other instance
    /// draws and takes none.
    pub hit: bool,
}

/// Interaction state: the section handle being dragged, and the modifiers.
#[derive(Debug, Default)]
pub struct State {
    modifiers: keyboard::Modifiers,
    /// Cursor y and the span when the handle was grabbed.
    handle: Option<(f32, f32)>,
}

/// A separation drawn on the rocket.
struct Seam {
    stage: usize,
    station: f32,
    /// A warning: no joint found, drawn dashed in `warn`.
    issue: bool,
}

/// Largest outer radius of the airframe, metres.
fn rmax_of(rocket: &Rocket) -> f32 {
    rocket.parts().iter().map(|p| p.kind.fore_diameter().max(p.kind.aft_diameter()) as f32 / 2.0).fold(0.0, f32::max)
}

/// Outer radius of the airframe at station `z`.
fn radius_at_of(rocket: &Rocket, z: f32) -> f32 {
    let stations = rocket.stations();
    rocket.parts().iter().enumerate().find(|(i, _)| z as f64 <= stations[i + 1] + 1e-9).map_or(0.0, |(i, p)| p.kind.radius_at(z as f64 - stations[i]) as f32)
}

/// What the CG / CP markers show, for working out where their labels fall.
#[derive(Debug, Clone, Copy)]
pub struct BalanceMarks {
    pub cg: f64,
    pub cp: f64,
    pub margin: f64,
    /// Whether the CG and the CP are drawn.
    pub show: (bool, bool),
    /// A staged rocket puts the values beside the glyphs instead of above the body.
    pub staged: bool,
}

/// Screen rectangles of the CG and CP value labels and of the margin label, as [`Markers`] draws them (px in the viewport), so
/// other overlays can keep their text off them.
pub fn balance_label_rects(rocket: &Rocket, camera: &Camera, size: Size, marks: &BalanceMarks, units: UnitSystem) -> Vec<Rectangle> {
    let axis = |z: f32| camera.project(Vec3::new(0.0, 0.0, z), size);
    let z = rocket.length() as f32 / 2.0;
    let body = axis(z).distance(camera.project(Vec3::new(0.0, 0.0, z) + camera.up() * rmax_of(rocket).max(0.005), size));
    let glyph_row = body + if marks.staged { 78.0 } else { 40.0 };
    let rect = |w: f32, x: f32, y_top: f32| Rectangle::new(Point::new(x, y_top), Size::new(w, TEXT_LABEL));
    let width = |s: &str| s.chars().count() as f32 * TEXT_LABEL * 0.6;
    let (a, b) = (axis(marks.cg as f32), axis(marks.cp as f32));
    let mut out = Vec::new();
    // The labels lean away from each other (see [`label_sides`]).
    let (cg_right, cp_right) = label_sides(a, b);
    for (zc, at, name, value, right) in [(marks.cg as f32, a, "CG", marks.cg, cg_right), (marks.cp as f32, b, "CP", marks.cp, cp_right)] {
        if !(if name == "CG" { marks.show.0 } else { marks.show.1 }) {
            continue;
        }
        let text = format!("{name} {}", bare(value, Quantity::Length, units));
        let w = width(&text);
        if marks.staged {
            let (x, y) = if right { (at.x - GLYPH_R - 5.0 - w, at.y + glyph_row) } else { (at.x + GLYPH_R + 5.0, at.y + glyph_row) };
            out.push(rect(w, x, y - TEXT_LABEL / 2.0));
        } else {
            let r = radius_at_of(rocket, zc).max(0.005);
            let r_px = axis(zc).distance(camera.project(Vec3::new(0.0, 0.0, zc) + camera.up() * r, size)).max(body * 0.2);
            let y = at.y - r_px - 24.0 - TEXT_LABEL;
            let x = if right { at.x - 4.0 - w } else { at.x + 4.0 };
            out.push(rect(w, x, y));
            // The stem up to the label.
            out.push(Rectangle::new(Point::new(at.x - 1.0, at.y - r_px - 22.0), Size::new(2.0, 22.0)));
        }
    }
    if marks.show.0 && marks.show.1 {
        let text = format!("{:.2} cal", marks.margin);
        let w = width(&text);
        let y = a.y.max(b.y) + glyph_row + 22.0;
        out.push(rect(w, (a.x + b.x) / 2.0 - w / 2.0, y + 4.0));
    }
    out
}

impl Markers<'_> {
    /// Largest outer radius of the airframe, metres.
    fn rmax(&self) -> f32 {
        rmax_of(self.rocket)
    }

    /// Outer radius of the airframe at station `z`.
    fn radius_at(&self, z: f32) -> f32 {
        radius_at_of(self.rocket, z)
    }

    fn axis(&self, z: f32, size: Size) -> Point {
        self.camera.project(Vec3::new(0.0, 0.0, z), size)
    }

    /// Screen length of the airframe's outer radius, px.
    fn radius_px(&self, z: f32, size: Size) -> f32 {
        let p = Vec3::new(0.0, 0.0, z);
        let (a, b) = (self.camera.project(p, size), self.camera.project(p + self.camera.up() * self.radius_at(z).max(0.005), size));
        a.distance(b)
    }

    /// Screen length of the largest radius at the middle of the rocket, px (the rows under and over the body hang from it).
    fn body_px(&self, size: Size) -> f32 {
        let z = self.rocket.length() as f32 / 2.0;
        let p = Vec3::new(0.0, 0.0, z);
        self.camera.project(p, size).distance(self.camera.project(p + self.camera.up() * self.rmax().max(0.005), size))
    }

    fn seams(&self) -> Vec<Seam> {
        if self.stages.len() < 2 || self.explode > 0.01 {
            return Vec::new();
        }
        self.stages
            .iter()
            .filter_map(|s| match (&s.separation, &s.issue) {
                (Some(sep), _) => Some(Seam { stage: s.index, station: sep.station as f32, issue: false }),
                (None, Some(bowshock_model::StageIssue::NoJoint { station, .. })) => Some(Seam { stage: s.index, station: *station as f32, issue: true }),
                _ => None,
            })
            .collect()
    }

    /// Centre and radius of the section handle, Cutaway only.
    fn handle(&self, size: Size) -> Option<(Point, f32)> {
        (self.inside == InsideView::Cutaway).then(|| (self.axis(0.0, size) + Vector::new(-HANDLE_GAP, 0.0), HANDLE_R))
    }

    /// Screen rectangle of a part's box moved by `explode`, for picking in the exploded view.
    fn exploded_rect(&self, min: Vec3, max: Vec3, offset: Vec3, size: Size) -> Rectangle {
        let (mut lo, mut hi) = (Point::new(f32::MAX, f32::MAX), Point::new(f32::MIN, f32::MIN));
        for i in 0..8 {
            let c = Vec3::new(if i & 1 == 0 { min.x } else { max.x }, if i & 2 == 0 { min.y } else { max.y }, if i & 4 == 0 { min.z } else { max.z });
            let p = self.camera.project(c + offset * self.explode, size);
            (lo.x, lo.y, hi.x, hi.y) = (lo.x.min(p.x), lo.y.min(p.y), hi.x.max(p.x), hi.y.max(p.y));
        }
        Rectangle::new(lo, Size::new(hi.x - lo.x, hi.y - lo.y))
    }

    fn exploded_pick(&self, at: Point, size: Size) -> Option<PartId> {
        self.mesh
            .parts
            .iter()
            .map(|b| (b.id, self.exploded_rect(b.min, b.max, b.explode, size)))
            .filter(|(_, r)| Rectangle::new(Point::new(r.x - 3.0, r.y - 3.0), Size::new(r.width + 6.0, r.height + 6.0)).contains(at))
            .min_by(|a, b| (a.1.width * a.1.height).total_cmp(&(b.1.width * b.1.height)))
            .map(|(id, _)| id)
    }

    /// Where the stage brackets, CG/CP glyphs and margin bracket hang, px below the axis.
    fn rows(&self, size: Size) -> (f32, f32) {
        let body = self.body_px(size);
        let staged = self.stages.len() > 1;
        let stage_row = body + 30.0;
        let glyph_row = body + if staged { 78.0 } else { 40.0 };
        (stage_row, glyph_row)
    }
}

/// Which of the CG (`.0`) and CP (`.1`) value labels lean left of their marker: the marker further left leans left and the
/// other right, so the two never run into each other however close they are or in which order; together (`a.x == b.x`) the CG
/// leans left. `a` is the CG's screen point, `b` the CP's.
fn label_sides(a: Point, b: Point) -> (bool, bool) {
    let cg_left = a.x <= b.x;
    (cg_left, !cg_left)
}

pub(super) fn margin_color(t: &Tokens, bands: bowshock_model::StabilityBands, margin: f64) -> iced::Color {
    if margin >= bands.stable {
        t.ok
    } else if margin >= bands.marginal {
        t.warn
    } else {
        t.err
    }
}

fn label(frame: &mut Frame, content: String, at: Point, color: iced::Color, align: alignment::Horizontal, valign: alignment::Vertical) {
    frame.fill_text(Text { content, position: at, color, size: TEXT_LABEL.into(), font: MONO, align_x: align.into(), align_y: valign, ..Text::default() });
}

/// The CG glyph: a ring with two opposite quadrants filled.
fn cg_glyph(frame: &mut Frame, c: Point, color: iced::Color, bg: iced::Color) {
    frame.fill(&Path::circle(c, GLYPH_R), bg);
    // Top-left and bottom-right quadrants (screen y is down), as polygons sampled along the arc.
    for start in [PI, 0.0] {
        let quadrant = Path::new(|b| {
            b.move_to(c);
            for i in 0..=12 {
                let a = start + FRAC_PI_2 * i as f32 / 12.0;
                b.line_to(Point::new(c.x + GLYPH_R * a.cos(), c.y + GLYPH_R * a.sin()));
            }
            b.close();
        });
        frame.fill(&quadrant, color);
    }
    frame.stroke(&Path::circle(c, GLYPH_R), Stroke::default().with_color(color).with_width(1.5));
}

/// The CP glyph: a ring with a centred dot.
fn cp_glyph(frame: &mut Frame, c: Point, color: iced::Color, bg: iced::Color) {
    frame.fill(&Path::circle(c, GLYPH_R), bg);
    frame.stroke(&Path::circle(c, GLYPH_R), Stroke::default().with_color(color).with_width(1.5));
    frame.fill(&Path::circle(c, 2.4), color);
}

fn diamond(c: Point, half: f32) -> Path {
    Path::new(|b| {
        b.move_to(Point::new(c.x, c.y - half));
        b.line_to(Point::new(c.x + half, c.y));
        b.line_to(Point::new(c.x, c.y + half));
        b.line_to(Point::new(c.x - half, c.y));
        b.close();
    })
}

impl canvas::Program<Message> for Markers<'_> {
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        if !self.hit || !self.interactive {
            state.handle = None;
            return None;
        }
        let size = bounds.size();
        match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(m)) => {
                state.modifiers = *m;
                None
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let at = cursor.position_in(bounds)?;
                if let Some((c, r)) = self.handle(size) {
                    if at.distance(c) <= r + 3.0 {
                        state.handle = Some((at.y, self.cut.span));
                        return Some(Action::capture());
                    }
                }
                if let Some(seam) = self.seams().into_iter().find(|s| !s.issue && self.axis(s.station, size).distance(at) <= HIT) {
                    return Some(Action::publish(Message::SelectSeparation(seam.stage)).and_capture());
                }
                if self.inside == InsideView::Exploded && self.explode > 0.5 {
                    return Some(Action::publish(Message::SelectPart(self.exploded_pick(at, size))).and_capture());
                }
                None
            }
            Event::Mouse(mouse::Event::CursorMoved { position }) => {
                let (y0, span0) = state.handle?;
                // One degree per pixel dragged up; Shift snaps to quarter turns (a quarter, half or three-quarter cut).
                let y = position.y - bounds.y;
                let mut span = span0 + (y0 - y) * PI / 180.0;
                if state.modifiers.shift() {
                    span = (span / FRAC_PI_2).round() * FRAC_PI_2;
                }
                let (lo, hi) = Cut::SPAN_RANGE;
                let span = span.clamp(if state.modifiers.shift() { FRAC_PI_2 } else { lo }, hi);
                Some(Action::publish(Message::Cut(Cut { span, ..self.cut })).and_capture())
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => state.handle.take().map(|_| Action::capture()),
            _ => None,
        }
    }

    fn mouse_interaction(&self, state: &State, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        if !self.hit || !self.interactive {
            return mouse::Interaction::None;
        }
        if state.handle.is_some() {
            return mouse::Interaction::Grabbing;
        }
        let Some(at) = cursor.position_in(bounds) else {
            return mouse::Interaction::None;
        };
        let size = bounds.size();
        let on_handle = self.handle(size).is_some_and(|(c, r)| at.distance(c) <= r + 3.0);
        let on_seam = self.seams().iter().any(|s| !s.issue && self.axis(s.station, size).distance(at) <= HIT);
        if on_handle {
            mouse::Interaction::Grab
        } else if on_seam {
            mouse::Interaction::Pointer
        } else {
            mouse::Interaction::None
        }
    }

    fn draw(&self, _: &State, renderer: &Renderer, theme: &Theme, bounds: Rectangle, cursor: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        if self.hit || self.mesh.is_empty() {
            return vec![frame.into_geometry()];
        }
        let dotted =
            |color: iced::Color| Stroke { line_dash: LineDash { segments: &[2.0, 3.0], offset: 0 }, ..Stroke::default().with_color(color).with_width(1.0) };
        let dashed =
            |color: iced::Color| Stroke { line_dash: LineDash { segments: &[5.0, 3.0], offset: 0 }, ..Stroke::default().with_color(color).with_width(1.0) };
        let (stage_row, glyph_row) = self.rows(size);
        let body = self.body_px(size);

        // Exploded view: dashed leaders from each internal part back to its station.
        if self.explode > 0.01 {
            for l in &self.mesh.leaders {
                let a = self.camera.project(l.origin + Vec3::new(0.0, 0.0, l.offset.z * self.explode), size);
                let b = self.camera.project(l.origin + l.offset * self.explode + self.camera.up() * l.radius, size);
                frame.stroke(&Path::line(a, b), dotted(t.ink_faint.scale_alpha(self.explode)));
            }
        }

        // Separation seams, diamonds and stage brackets.
        for seam in self.seams() {
            let selected = self.selection == Selection::Separation(seam.stage);
            let color = if seam.issue { t.warn } else { t.accent };
            let r = self.radius_at(seam.station);
            let back = self.camera.back();
            let ring: Vec<(Point, bool)> = (0..=48)
                .map(|i| {
                    let a = TAU * i as f32 / 48.0;
                    let (s, c) = a.sin_cos();
                    (self.camera.project(Vec3::new(r * c, r * s, seam.station), size), Vec3::new(c, s, 0.0).dot(back) > -0.02)
                })
                .collect();
            let arc = Path::new(|b| {
                let mut pen = false;
                for &(p, visible) in &ring {
                    match (visible, pen) {
                        (true, false) => {
                            b.move_to(p);
                            pen = true;
                        }
                        (true, true) => b.line_to(p),
                        (false, _) => pen = false,
                    }
                }
            });
            frame.stroke(&arc, if seam.issue { dashed(color) } else { Stroke::default().with_color(color).with_width(1.0) });
            let c = self.axis(seam.station, size);
            let tag_color = if selected || seam.issue { color } else { t.ink_muted };
            let station = bare(f64::from(seam.station), Quantity::Length, self.units);
            label(
                &mut frame,
                if seam.issue { format!("NO JOINT · {station}") } else { format!("SEPARATION · {station}") },
                Point::new(c.x, c.y - body - 14.0),
                tag_color,
                alignment::Horizontal::Center,
                alignment::Vertical::Bottom,
            );
            if !seam.issue {
                let d = diamond(c, DIAMOND);
                frame.fill(&d, if selected { t.accent } else { t.bg_000 });
                frame.stroke(&d, Stroke::default().with_color(t.accent).with_width(1.5));
            }
        }
        if self.stages.len() > 1 && self.explode < 0.01 {
            for s in self.stages {
                let (x0, x1) = (self.axis(s.fore_station as f32, size).x, self.axis(s.aft_station as f32, size).x);
                let y = self.axis(0.0, size).y + stage_row;
                let bracket = Path::new(|b| {
                    b.move_to(Point::new(x0 + 1.0, y - 5.0));
                    b.line_to(Point::new(x0 + 1.0, y));
                    b.line_to(Point::new(x1 - 1.0, y));
                    b.line_to(Point::new(x1 - 1.0, y - 5.0));
                });
                frame.stroke(&bracket, Stroke::default().with_color(t.ink_faint).with_width(1.0));
                let top = s.index == self.stages.len();
                let motors: Vec<&str> = self.motors.iter().filter(|(i, _)| *i == s.index).map(|(_, m)| m.as_str()).collect();
                let mut name = format!("STAGE {}", s.index);
                if top {
                    name.push_str(" · SUSTAINER");
                }
                if !motors.is_empty() {
                    name.push_str(&format!(" · {}", motors.join(" + ")));
                }
                let (x, align) = if s.fore_station <= 1e-9 { (x0 + 2.0, alignment::Horizontal::Left) } else { (x1 - 2.0, alignment::Horizontal::Right) };
                label(&mut frame, name, Point::new(x, y + 5.0), t.ink_muted, align, alignment::Vertical::Top);
            }
        }

        // CG and CP: value labels above the body with a stem, glyphs below it, and the margin bracket between them.
        // They are meaningless looking down the axis (everything stacks at the centre): fade out as the view turns axial.
        let axial = self.camera.back().z.abs();
        let fade = 1.0 - ((axial - AXIAL_FADE_FROM) / (AXIAL_FADE_TO - AXIAL_FADE_FROM)).clamp(0.0, 1.0);
        if let Some((cg, cp, margin)) = self.balance.filter(|_| fade > 0.02) {
            let (zc, zp) = (cg as f32, cp as f32);
            // The stage brackets hang from the nose's height, which in a tilted view is not the CG's: the glyph row never rises
            // into them (their labels would collide with the value labels and the margin).
            let floor = if self.stages.len() > 1 && self.explode < 0.01 { self.axis(0.0, size).y + stage_row + 24.0 + GLYPH_R } else { f32::MIN };
            let glyph_y = |at: Point| (at.y + glyph_row).max(floor);
            let (a, b) = (self.axis(zc, size), self.axis(zp, size));
            let side = |leans_left: bool| if leans_left { alignment::Horizontal::Right } else { alignment::Horizontal::Left };
            let (cg_left, cp_left) = label_sides(a, b);
            if let Some((gcg, gcp)) = self.ghost {
                for (z, now, color, show, cp) in [(gcg, cg, t.cg, self.show.0, false), (gcp, cp, t.cp, self.show.1, true)] {
                    if !show || (z - now).abs() < 1e-9 {
                        continue;
                    }
                    let (from, to) = (self.axis(z as f32, size), self.axis(now as f32, size));
                    let faint = color.scale_alpha(0.45 * fade);
                    let (g0, g1) = (Point::new(from.x, glyph_y(from)), Point::new(to.x, glyph_y(to)));
                    let r = self.radius_px(z as f32, size).max(body * 0.2);
                    let thin = Stroke::default().with_color(faint).with_width(1.0);
                    frame.stroke(&Path::line(Point::new(from.x, from.y + r), Point::new(from.x, g0.y - GLYPH_R)), thin);
                    // From where it was to where it would be.
                    let lead = GLYPH_R.copysign(g1.x - g0.x);
                    frame.stroke(&Path::line(Point::new(g0.x + lead, g0.y), Point::new(g1.x - lead, g1.y)), thin);
                    if cp {
                        cp_glyph(&mut frame, g0, faint, t.bg_000.scale_alpha(fade));
                    } else {
                        cg_glyph(&mut frame, g0, faint, t.bg_000.scale_alpha(fade));
                    }
                }
            }
            for (z, at, color, name, value, dash, align) in
                [(zc, a, t.cg.scale_alpha(fade), "CG", cg, false, side(cg_left)), (zp, b, t.cp.scale_alpha(fade), "CP", cp, true, side(cp_left))]
            {
                if !(if name == "CG" { self.show.0 } else { self.show.1 }) {
                    continue;
                }
                let r = self.radius_px(z, size).max(body * 0.2);
                let value = bare(value, Quantity::Length, self.units);
                let top = at.y - r;
                let g = Point::new(at.x, glyph_y(at));
                if self.stages.len() > 1 {
                    // A staged rocket has the separation's tag above the body: the values sit beside the glyphs instead.
                    let (dx, side) = if align == alignment::Horizontal::Right {
                        (-GLYPH_R - 5.0, alignment::Horizontal::Right)
                    } else {
                        (GLYPH_R + 5.0, alignment::Horizontal::Left)
                    };
                    label(&mut frame, format!("{name} {value}"), Point::new(g.x + dx, g.y), color, side, alignment::Vertical::Center);
                } else {
                    let stem = Path::line(Point::new(at.x, top - 22.0), Point::new(at.x, top));
                    frame.stroke(&stem, if dash { dashed(color) } else { Stroke::default().with_color(color).with_width(1.0) });
                    let dx = if align == alignment::Horizontal::Right { -4.0 } else { 4.0 };
                    label(&mut frame, format!("{name} {value}"), Point::new(at.x + dx, top - 24.0), color, align, alignment::Vertical::Bottom);
                }
                // The glyph hangs below the body on a thin stem.
                frame.stroke(&Path::line(Point::new(at.x, at.y + r), Point::new(at.x, g.y - GLYPH_R)), dotted(color.scale_alpha(0.7)));
                if dash {
                    cp_glyph(&mut frame, g, color, t.bg_000.scale_alpha(fade));
                } else {
                    cg_glyph(&mut frame, g, color, t.bg_000.scale_alpha(fade));
                }
            }
            if self.show.0 && self.show.1 {
                let y = glyph_y(a).max(glyph_y(b)) + 22.0;
                let color = margin_color(t, self.rocket.stability_bands(), margin).scale_alpha(fade);
                let bracket = Path::new(|p| {
                    p.move_to(Point::new(a.x, y - 5.0));
                    p.line_to(Point::new(a.x, y));
                    p.line_to(Point::new(b.x, y));
                    p.line_to(Point::new(b.x, y - 5.0));
                });
                frame.stroke(&bracket, Stroke::default().with_color(color).with_width(1.5));
                label(
                    &mut frame,
                    format!("{margin:.2} cal"),
                    Point::new((a.x + b.x) / 2.0, y + 4.0),
                    color,
                    alignment::Horizontal::Center,
                    alignment::Vertical::Top,
                );
            }
        }

        // The cutaway's section handle: a round dial at the nose end of the axis.
        if let Some((c, r)) = self.handle(size) {
            let hot = cursor.position_in(bounds).is_some_and(|p| p.distance(c) <= r + 3.0);
            let color = if hot { t.accent } else { t.ink_muted };
            frame.fill(&Path::circle(c, r), t.bg_000);
            frame.stroke(&Path::circle(c, r), Stroke::default().with_color(color).with_width(1.5));
            frame.stroke(&Path::line(Point::new(c.x - 3.5, c.y), Point::new(c.x + 3.5, c.y)), Stroke::default().with_color(color).with_width(1.5));
            label(
                &mut frame,
                format!("CUT {:.0}°", self.cut.span.to_degrees()),
                Point::new(c.x, c.y - r - 6.0),
                t.ink_faint,
                alignment::Horizontal::Center,
                alignment::Vertical::Bottom,
            );
        }
        vec![frame.into_geometry()]
    }
}

/// The bottom-left legend: `◐ CG  ◉ CP  M 0.30 · α 0.0°`, framed like every overlay.
pub fn legend<'a, M: 'a>(mach: f64, alpha_deg: f64) -> Element<'a, M> {
    let mono = |s: String| text(s).font(MONO).size(TEXT_LABEL).style(theme::muted);
    let rule = || container(Space::new()).width(1).height(TEXT_LABEL + 6.0).style(theme::hairline);
    container(
        row![
            row![glyph::cg(14.0), mono("CG".into())].spacing(SPACE_1).align_y(iced::Center),
            row![glyph::cp(14.0), mono("CP".into())].spacing(SPACE_1).align_y(iced::Center),
            rule(),
            mono(format!("M {mach:.2} · α {alpha_deg:.1}°")),
        ]
        .spacing(SPACE_3)
        .align_y(iced::Center),
    )
    .padding([SPACE_1, SPACE_2])
    .style(theme::overlay)
    .boxed()
}

/// The bottom-centre key-hint bar: pairs of `(key, what it does)`, a key drawn as a framed mono keycap; an empty key is plain
/// text (`Drag tip`).
pub fn hints<'a, M: 'a>(pairs: &[(&'static str, &'static str)]) -> Element<'a, M> {
    let mut items: Vec<Element<'a, M>> = Vec::new();
    for &(key, what) in pairs {
        let mut part: Vec<Element<'a, M>> = Vec::new();
        if !key.is_empty() {
            part.push(container(text(key).font(MONO).size(TEXT_LABEL - 1.0)).padding([0.0, 4.0]).style(theme::keycap).boxed());
        }
        part.push(text(what).size(TEXT_LABEL + 1.0).boxed());
        items.push(row(part).spacing(SPACE_1).align_y(iced::Center).boxed());
    }
    container(row(items).spacing(SPACE_3).align_y(iced::Center)).padding([SPACE_1 + 2.0, SPACE_3]).style(theme::overlay).boxed()
}

/// Width the legend takes at the bottom left and the scale bar at the bottom right, px (see `sheet`).
const LEGEND_W: f32 = 226.0;
const SCALE_W: f32 = 120.0;

/// The pairs of `pairs` that fit in `room` px: the first always, then each next while the bar stays within `room`. The widths are
/// estimates from the glyph counts (keycap `TEXT_LABEL - 1` mono, words `TEXT_LABEL + 1` sans), the paddings and the gaps.
fn fit_hints(pairs: &[(&'static str, &'static str)], room: f32) -> usize {
    let pair = |(key, what): &(&str, &str)| {
        let cap = if key.is_empty() { 0.0 } else { key.chars().count() as f32 * (TEXT_LABEL - 1.0) * 0.6 + 10.0 + SPACE_1 };
        cap + what.chars().count() as f32 * (TEXT_LABEL + 1.0) * 0.52
    };
    let mut width = 2.0 * SPACE_3;
    let mut count = 0;
    for p in pairs {
        width += pair(p) + if count > 0 { SPACE_3 } else { 0.0 };
        if count > 0 && width > room {
            break;
        }
        count += 1;
    }
    count
}

/// The bottom row of the viewport, `space-4` from the edges: the legend at the left, the key hints in the middle (when there
/// are any) and room at the right for the scale bar. The hint bar keeps to the room between the legend and the scale bar: pairs
/// that do not fit are left off (the last ones, `Esc cancel` before the main verb), so it never covers either.
pub fn bottom_bar<'a, M: 'a>(mach: f64, alpha_deg: f64, pairs: Vec<(&'static str, &'static str)>) -> Element<'a, M> {
    responsive(move |size| {
        let room = size.width - 2.0 * SPACE_4 - LEGEND_W - SCALE_W - 2.0 * SPACE_3;
        let mut items: Vec<Element<'a, M>> = vec![legend(mach, alpha_deg), Space::new().width(Fill).boxed()];
        if !pairs.is_empty() {
            items.push(hints(&pairs[..fit_hints(&pairs, room)]));
        }
        items.push(Space::new().width(Fill).boxed());
        items.push(Space::new().width(SCALE_W).boxed());
        container(row(items).align_y(alignment::Vertical::Bottom)).width(Fill).height(Fill).padding(SPACE_4).align_y(alignment::Vertical::Bottom).boxed()
    })
    .boxed()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cg_and_cp_labels_lean_away_from_each_other_in_either_order() {
        // CG fore of the CP (a stable rocket): CG label to the left, CP label to the right.
        assert_eq!(label_sides(Point::new(100.0, 0.0), Point::new(110.0, 0.0)), (true, false));
        // CP fore of the CG (a preview of an unstable rocket, however far apart): the other way round, never towards each other.
        assert_eq!(label_sides(Point::new(200.0, 0.0), Point::new(110.0, 0.0)), (false, true));
        // Together: one each way.
        let (cg, cp) = label_sides(Point::new(50.0, 0.0), Point::new(50.0, 0.0));
        assert_ne!(cg, cp);
    }
}
