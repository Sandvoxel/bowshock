//! The ViewCube: a small canvas showing the camera orientation. Hover highlights one of 26 regions (6 faces, 12 edges,
//! 8 corners); clicking snaps the camera to it, dragging orbits, the house resets, and when looking straight at a face
//! the arrows step to the neighbouring faces.

use glam::{IVec3, Vec3};
use iced::advanced::text::Alignment;
use iced::widget::canvas::{self, Action, Event, Fill, Frame, Path, Stroke, Text};
use iced::{alignment, mouse, Color, Pixels, Point, Radians, Rectangle, Size, Vector};

use super::{Camera, Message};
use crate::theme::{tokens, MONO};

/// Cube half-extent (unit cube) in pixels, as a fraction of the canvas size.
const CUBE_FRACTION: f32 = 0.28;
/// Reciprocal eye distance (cube units) of the off-face perspective.
const PERSPECTIVE: f32 = 0.12;
/// Cell boundaries along a face's in-plane axes.
const BOUNDS: [f32; 4] = [-1.0, -0.62, 0.62, 1.0];
/// Drag distance in px before a press becomes an orbit.
const DRAG_SLOP: f32 = 3.0;
/// Resolution the label text is laid out at before the face transform shrinks it back.
const LABEL_K: f32 = 100.0;

struct Face {
    n: IVec3,
    u: IVec3,
    v: IVec3,
    label: &'static str,
}

const FACES: [Face; 6] = [
    // Named views, as a pilot would: the rocket flies nose first (-Z) with +Y up, so -X is its port side and +X its starboard.
    Face { n: IVec3::NEG_X, u: IVec3::Z, v: IVec3::Y, label: "PORT" },
    Face { n: IVec3::X, u: IVec3::NEG_Z, v: IVec3::Y, label: "STBD" },
    Face { n: IVec3::NEG_Z, u: IVec3::NEG_X, v: IVec3::Y, label: "NOSE" },
    Face { n: IVec3::Z, u: IVec3::X, v: IVec3::Y, label: "TAIL" },
    Face { n: IVec3::Y, u: IVec3::Z, v: IVec3::X, label: "TOP" },
    Face { n: IVec3::NEG_Y, u: IVec3::Z, v: IVec3::NEG_X, label: "BOTTOM" },
];

/// The region a face's cell `(i, j)` selects.
fn region(face: &Face, i: i32, j: i32) -> IVec3 {
    face.n + face.u * i + face.v * j
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Hit {
    Region(IVec3),
    Home,
    Arrow(IVec3),
    Roll(i8),
}

#[derive(Debug, Default)]
pub struct State {
    hover: Option<Hit>,
    press: Option<(Point, Option<Hit>)>,
    dragging: bool,
    last: Point,
}

pub struct ViewCube {
    pub camera: Camera,
}

/// Projection of the cube for one canvas size.
struct View {
    camera: Camera,
    centre: Point,
    s: f32,
    /// Perspective strength: reciprocal eye distance in cube units; 0 is orthographic.
    k: f32,
}

impl View {
    fn new(camera: Camera, size: Size) -> Self {
        let aligned = camera.back().abs().max_element();
        // Ease from orthographic (face-on) to full perspective within a few degrees of a face.
        let t = ((1.0 - aligned) * 20.0).clamp(0.0, 1.0);
        // The cube fills the square at the right of the canvas; the strip left of it holds the axis triad.
        let side = size.height.min(size.width);
        Self { camera, centre: Point::new(size.width - side / 2.0, size.height / 2.0), s: CUBE_FRACTION * side, k: PERSPECTIVE * t }
    }

    /// Screen offset of a world direction, in px (y down).
    fn dir(&self, d: Vec3) -> Vector {
        Vector::new(d.dot(self.camera.right()), -d.dot(self.camera.up())) * self.s
    }

    /// Perspective-projected screen point. Off-face the cube is seen in perspective (like Fusion); face-on it is
    /// orthographic so the view is undistorted.
    fn project(&self, p: Vec3) -> Point {
        let depth = 1.0 - p.dot(self.camera.back()) * self.k;
        self.centre + self.dir(p) / depth
    }

    fn facing(&self, n: IVec3) -> f32 {
        n.as_vec3().dot(self.camera.back())
    }

    /// A face is visible when the eye (at distance `1 / k`) is on its outer side.
    fn visible(&self, n: IVec3) -> bool {
        self.facing(n) > self.k + 1e-3
    }

    /// Projected corners of cell `(i, j)` on `face`.
    fn cell(&self, face: &Face, i: i32, j: i32) -> [Point; 4] {
        let (a0, a1) = (BOUNDS[(i + 1) as usize], BOUNDS[(i + 2) as usize]);
        let (b0, b1) = (BOUNDS[(j + 1) as usize], BOUNDS[(j + 2) as usize]);
        let at = |a: f32, b: f32| self.project(face.n.as_vec3() + face.u.as_vec3() * a + face.v.as_vec3() * b);
        [at(a0, b0), at(a1, b0), at(a1, b1), at(a0, b1)]
    }

    fn face_outline(&self, face: &Face) -> [Point; 4] {
        let at = |a: f32, b: f32| self.project(face.n.as_vec3() + face.u.as_vec3() * a + face.v.as_vec3() * b);
        [at(-1.0, -1.0), at(1.0, -1.0), at(1.0, 1.0), at(-1.0, 1.0)]
    }

    /// The face the camera looks straight at, if any.
    fn aligned(&self) -> bool {
        let b = self.camera.back();
        b.abs().max_element() > 1.0 - 1e-3
    }

    /// Neighbour arrows: (snap target, centre, outward screen direction). Empty unless face-aligned.
    fn arrows(&self) -> Vec<(IVec3, Point, Vector)> {
        if !self.aligned() {
            return Vec::new();
        }
        let (up, right) = (self.camera.up(), self.camera.right());
        let d = self.s + 10.0;
        [
            (dominant(up), Vector::new(0.0, -1.0)),
            (dominant(-up), Vector::new(0.0, 1.0)),
            (dominant(-right), Vector::new(-1.0, 0.0)),
            (dominant(right), Vector::new(1.0, 0.0)),
        ]
        .into_iter()
        .map(|(target, dir)| (target, self.centre + dir * d, dir))
        .collect()
    }

    /// Roll arrows, shown only when face-aligned: (direction, arc centre).
    fn roll_arrows(&self) -> Vec<(i8, Point)> {
        if !self.aligned() {
            return Vec::new();
        }
        let w = self.centre.x * 2.0;
        vec![(-1, Point::new(w - 32.0, 16.0)), (1, Point::new(w - 14.0, 16.0))]
    }

    fn hit(&self, p: Point) -> Option<Hit> {
        for (dir, c) in self.roll_arrows() {
            if Rectangle::new(Point::new(c.x - 9.0, c.y - 9.0), Size::new(18.0, 18.0)).contains(p) {
                return Some(Hit::Roll(dir));
            }
        }
        if home_rect().contains(p) {
            return Some(Hit::Home);
        }
        for (target, c, _) in self.arrows() {
            if Rectangle::new(Point::new(c.x - 8.0, c.y - 8.0), Size::new(16.0, 16.0)).contains(p) {
                return Some(Hit::Arrow(target));
            }
        }
        for face in &FACES {
            if !self.visible(face.n) {
                continue;
            }
            for i in -1..=1 {
                for j in -1..=1 {
                    if in_quad(p, &self.cell(face, i, j)) {
                        return Some(Hit::Region(region(face, i, j)));
                    }
                }
            }
        }
        None
    }
}

/// Radius of the roll arrows' arcs, px.
const ROLL_R: f32 = 6.0;

/// Polyline of the roll arc around `c`, the arrow tip, and the unit tangent there. Clockwise on screen for `dir > 0`.
fn roll_arc(c: Point, dir: i8) -> ([Point; 9], Point, Vector) {
    let sign = f32::from(dir.signum());
    let (start, sweep) = if dir > 0 { (-150.0_f32, 70.0_f32) } else { (-30.0, -70.0) };
    let at = |deg: f32| {
        let t = deg.to_radians();
        Point::new(c.x + ROLL_R * t.cos(), c.y + ROLL_R * t.sin())
    };
    let arc = std::array::from_fn(|k| at(start + sweep * k as f32 / 8.0));
    let end = (start + sweep).to_radians();
    // Direction of travel at the end: d/dθ of (cos, sin) is (-sin, cos), reversed when sweeping anticlockwise.
    let tangent = Vector::new(-end.sin(), end.cos()) * sign;
    (arc, at(start + sweep) + tangent * 3.0, tangent)
}

fn home_rect() -> Rectangle {
    Rectangle::new(Point::new(2.0, 2.0), Size::new(18.0, 18.0))
}

/// The signed world axis `v` points along most.
fn dominant(v: Vec3) -> IVec3 {
    let a = v.abs();
    if a.x >= a.y && a.x >= a.z {
        IVec3::new(v.x.signum() as i32, 0, 0)
    } else if a.y >= a.z {
        IVec3::new(0, v.y.signum() as i32, 0)
    } else {
        IVec3::new(0, 0, v.z.signum() as i32)
    }
}

/// Point-in-convex-quad: all edge cross products share a sign.
fn in_quad(p: Point, q: &[Point; 4]) -> bool {
    let mut pos = false;
    let mut neg = false;
    for k in 0..4 {
        let (a, b) = (q[k], q[(k + 1) % 4]);
        let cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if cross > 0.0 {
            pos = true;
        } else if cross < 0.0 {
            neg = true;
        }
    }
    !(pos && neg)
}

fn mix(a: Color, b: Color, t: f32) -> Color {
    Color::from_rgba(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t)
}

fn polygon(points: &[Point]) -> Path {
    Path::new(|b| {
        b.move_to(points[0]);
        for p in &points[1..] {
            b.line_to(*p);
        }
        b.close();
    })
}

/// Decomposes the 2x2 matrix `[[a, b], [c, d]]` as `rotate(alpha) * diag(s1, s2) * rotate(beta)`.
/// Returns `(alpha, s1, s2, beta)`; `s2` is negative for a reflection.
fn svd2(a: f32, b: f32, c: f32, d: f32) -> (f32, f32, f32, f32) {
    let e = (a + d) / 2.0;
    let f = (a - d) / 2.0;
    let g = (c + b) / 2.0;
    let h = (c - b) / 2.0;
    let q = e.hypot(h);
    let r = f.hypot(g);
    let a1 = g.atan2(f);
    let a2 = h.atan2(e);
    ((a2 + a1) / 2.0, q + r, q - r, (a2 - a1) / 2.0)
}

fn label_text(content: &str, color: Color) -> Text {
    Text {
        content: content.to_string(),
        position: Point::ORIGIN,
        color,
        size: Pixels(0.36 * LABEL_K),
        font: MONO,
        align_x: Alignment::Center,
        align_y: alignment::Vertical::Center,
        ..Text::default()
    }
}

fn small_text(content: &str, position: Point, color: Color) -> Text {
    Text {
        content: content.to_string(),
        position,
        color,
        size: Pixels(crate::tokens::TEXT_LABEL),
        font: MONO,
        align_x: Alignment::Center,
        align_y: alignment::Vertical::Center,
        ..Text::default()
    }
}

impl canvas::Program<Message> for ViewCube {
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        let Event::Mouse(event) = event else {
            return None;
        };
        let view = View::new(self.camera, bounds.size());
        let local_hit = || cursor.position_in(bounds).and_then(|p| view.hit(p));

        match event {
            mouse::Event::CursorMoved { position } => {
                if let Some((start, _)) = state.press {
                    if !state.dragging && position.distance(start) > DRAG_SLOP {
                        state.dragging = true;
                    }
                    if state.dragging {
                        let delta = *position - state.last;
                        state.last = *position;
                        return Some(Action::publish(Message::Orbit(delta)).and_capture());
                    }
                    return Some(Action::capture());
                }
                let hover = local_hit();
                let changed = hover != state.hover;
                state.hover = hover;
                match (changed, hover.is_some()) {
                    (true, true) => Some(Action::request_redraw().and_capture()),
                    (true, false) => Some(Action::request_redraw()),
                    (false, true) => Some(Action::capture()),
                    (false, false) => None,
                }
            }
            mouse::Event::ButtonPressed(mouse::Button::Left) => {
                let hit = local_hit()?;
                let position = cursor.position()?;
                state.press = Some((position, Some(hit)));
                state.dragging = false;
                state.last = position;
                Some(Action::capture())
            }
            mouse::Event::ButtonReleased(mouse::Button::Left) => {
                let (_, hit) = state.press.take()?;
                if std::mem::take(&mut state.dragging) {
                    return Some(Action::capture());
                }
                let action = match hit? {
                    Hit::Region(r) | Hit::Arrow(r) => Action::publish(Message::Snap(r)),
                    Hit::Home => Action::publish(Message::Home),
                    Hit::Roll(d) => Action::publish(Message::Roll(d)),
                };
                Some(action.and_capture())
            }
            _ => None,
        }
    }

    fn draw(
        &self,
        state: &State,
        renderer: &iced::Renderer,
        theme: &iced::Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry<iced::Renderer>> {
        let tk = tokens(theme);
        let view = View::new(self.camera, bounds.size());
        let mut frame = Frame::new(renderer, bounds.size());
        let back = self.camera.back();

        let hovered_region = match state.hover {
            Some(Hit::Region(r)) => Some(r),
            _ => None,
        };
        let highlight = Color { a: 0.55, ..tk.accent };

        for face in &FACES {
            if !view.visible(face.n) {
                continue;
            }
            let facing = view.facing(face.n);
            let base = mix(tk.bg_300, tk.bg_400, facing.clamp(0.0, 1.0));
            for i in -1..=1 {
                for j in -1..=1 {
                    let path = polygon(&view.cell(face, i, j));
                    frame.fill(&path, Fill::from(base));
                    if hovered_region == Some(region(face, i, j)) {
                        frame.fill(&path, Fill::from(highlight));
                    }
                }
            }
            frame.stroke(&polygon(&view.face_outline(face)), Stroke::default().with_color(tk.line_200).with_width(1.0));
        }

        // Labels, skewed with their face.
        for face in &FACES {
            if face.n.as_vec3().dot(back) <= 0.25 {
                continue;
            }
            let color = if hovered_region == Some(face.n) { tk.accent_ink } else { tk.ink_muted };
            let c0 = view.project(face.n.as_vec3());
            let u = view.project(face.n.as_vec3() + face.u.as_vec3()) - c0;
            let v = view.project(face.n.as_vec3() + face.v.as_vec3()) - c0;
            // A = [U, -V]: text x runs along the face's u, text y (down) against its v.
            let (alpha, s1, s2, beta) = svd2(u.x, -v.x, u.y, -v.y);
            let centre = view.project(face.n.as_vec3());
            frame.with_save(|frame| {
                frame.translate(Vector::new(centre.x, centre.y));
                frame.rotate(Radians(alpha));
                frame.scale_nonuniform(Vector::new(s1 / LABEL_K, s2 / LABEL_K));
                frame.rotate(Radians(beta));
                frame.fill_text(label_text(face.label, color));
            });
        }

        // Home glyph.
        let home_color = if state.hover == Some(Hit::Home) { tk.accent } else { tk.ink_faint };
        let o = Point::new(4.0, 4.0);
        let pt = |x: f32, y: f32| Point::new(o.x + x, o.y + y);
        let roof = Path::new(|b| {
            b.move_to(pt(0.5, 7.0));
            b.line_to(pt(7.0, 1.0));
            b.line_to(pt(13.5, 7.0));
        });
        let body = Path::new(|b| {
            b.move_to(pt(2.5, 6.0));
            b.line_to(pt(2.5, 13.0));
            b.line_to(pt(11.5, 13.0));
            b.line_to(pt(11.5, 6.0));
        });
        frame.stroke(&roof, Stroke::default().with_color(home_color).with_width(1.2));
        frame.stroke(&body, Stroke::default().with_color(home_color).with_width(1.2));

        // Roll arrows.
        for (dir, c) in view.roll_arrows() {
            let color = if state.hover == Some(Hit::Roll(dir)) { tk.accent } else { tk.ink_faint };
            let (arc, tip, tangent) = roll_arc(c, dir);
            frame.stroke(
                &Path::new(|b| {
                    b.move_to(arc[0]);
                    for p in &arc[1..] {
                        b.line_to(*p);
                    }
                }),
                Stroke::default().with_color(color).with_width(1.2),
            );
            let perp = Vector::new(-tangent.y, tangent.x);
            let base = tip - tangent * 4.0;
            frame.fill(&polygon(&[tip, base + perp * 3.0, base - perp * 3.0]), Fill::from(color));
        }

        // Neighbour arrows.
        for (target, c, dir) in view.arrows() {
            let perp = Vector::new(-dir.y, dir.x);
            let tip = c + dir * 5.0;
            let base = c - dir * 3.0;
            let tri = polygon(&[tip, base + perp * 5.0, base - perp * 5.0]);
            let color = if state.hover == Some(Hit::Arrow(target)) { tk.accent } else { tk.ink_muted };
            frame.fill(&tri, Fill::from(color));
        }

        // Axis triad, bottom-left in its own strip so it never overlaps the cube: 14 px axes, letters at the kit's label size.
        let origin = Point::new(super::TRIAD_GUTTER / 2.0, bounds.height - 30.0);
        let triad = View { camera: self.camera, centre: origin, s: 14.0, k: 0.0 };
        for (axis, color, name) in [(Vec3::X, tk.axis_x, "X"), (Vec3::Y, tk.axis_y, "Y"), (Vec3::Z, tk.axis_z, "Z")] {
            let d = triad.dir(axis);
            let end = origin + d;
            frame.stroke(&Path::line(origin, end), Stroke::default().with_color(color).with_width(1.5));
            let len = d.x.hypot(d.y).max(1e-3);
            let label_at = end + d * (7.0 / len);
            frame.fill_text(small_text(name, label_at, color));
        }

        vec![frame.into_geometry()]
    }

    fn mouse_interaction(&self, state: &State, _bounds: Rectangle, _cursor: mouse::Cursor) -> mouse::Interaction {
        if state.dragging {
            mouse::Interaction::Grabbing
        } else if state.hover.is_some() {
            mouse::Interaction::Pointer
        } else {
            mouse::Interaction::None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roll_zero_corner_by_270_and_nose() {
        let top = FACES.iter().find(|f| f.label == "TOP").unwrap();
        // i runs along +Z (tail), j along +X (starboard): the port/nose corner is (-1, -1).
        assert_eq!(region(top, -1, -1), IVec3::new(-1, 1, -1));
        assert_eq!(region(top, 0, 0), IVec3::Y);
        assert_eq!(region(top, 0, 1), IVec3::new(1, 1, 0));
    }

    #[test]
    fn svd_reconstructs() {
        for (a, b, c, d) in [(3.0, -1.0, 2.0, 5.0), (1.0, 0.0, 0.0, -1.0), (0.3, 4.0, -2.0, 0.1)] {
            let (al, s1, s2, be) = svd2(a, b, c, d);
            let rot = |t: f32| [[t.cos(), -t.sin()], [t.sin(), t.cos()]];
            let (ra, rb) = (rot(al), rot(be));
            let m = [[ra[0][0] * s1, ra[0][1] * s2], [ra[1][0] * s1, ra[1][1] * s2]];
            let out = |i: usize, j: usize| m[i][0] * rb[0][j] + m[i][1] * rb[1][j];
            for (got, want) in [(out(0, 0), a), (out(0, 1), b), (out(1, 0), c), (out(1, 1), d)] {
                assert!((got - want).abs() < 1e-4, "{got} vs {want}");
            }
        }
    }
}
