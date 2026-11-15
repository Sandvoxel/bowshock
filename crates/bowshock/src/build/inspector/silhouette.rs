//! The small drawings of the inspector: the part silhouette in the header and the link glyph of the symmetry bar. Canvas
//! drawings tinted by the system colour of the part, like the parts bin rows.

use bowshock_model::{InternalKind, MassRole, PartId, PartRef, RadialKind, Rocket, System};
use iced::widget::canvas::{self, Frame, Geometry, LineCap, Path, Stroke};
use iced::{mouse, Color, Element, Point, Rectangle, Renderer, Theme, Widget};

use crate::theme::tokens;
use crate::tokens::Tokens;

/// Size of the header silhouette, logical px.
pub const SIZE: (f32, f32) = (24.0, 16.0);

/// A shape in the unit square `0..1` (y down): a closed outline, optionally filled, or an open line.
#[derive(Clone, Debug, PartialEq)]
pub struct Shape {
    /// Vertices.
    pub points: Vec<(f32, f32)>,
    /// Filled (else an outline only).
    pub fill: bool,
    /// The last vertex joins the first.
    pub closed: bool,
}

fn poly(points: &[(f32, f32)], fill: bool) -> Shape {
    Shape { points: points.to_vec(), fill, closed: true }
}

fn line(points: &[(f32, f32)]) -> Shape {
    Shape { points: points.to_vec(), fill: false, closed: false }
}

fn rect(x0: f32, y0: f32, x1: f32, y1: f32, fill: bool) -> Shape {
    poly(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1)], fill)
}

/// Maps `(x, y)` points (y up, in `width` by `height` units) into the unit square, keeping the aspect and centring it.
fn fit(points: &[(f64, f64)], width: f64, height: f64) -> Vec<(f32, f32)> {
    let aspect = f64::from(SIZE.0 / SIZE.1);
    let (w, h) = if width / height.max(1e-12) > aspect { (1.0, aspect * height / width) } else { (width / (aspect * height), 1.0) };
    points.iter().map(|&(x, y)| (((1.0 - w) / 2.0 + x / width * w) as f32, (1.0 - (1.0 - h) / 2.0 - y / height * h) as f32)).collect()
}

/// The shapes that draw `id`, and the system it is coloured as. Empty for an unknown id.
pub fn shapes(rocket: &Rocket, id: PartId) -> (Vec<Shape>, System) {
    let Some(part) = rocket.kind_of(id) else {
        return (Vec::new(), System::Structure);
    };
    let system = part.system();
    let shapes = match part {
        PartRef::Axial(p) => {
            let profile = p.kind.profile();
            let length = profile.last().map_or(1.0, |q| q.0).max(1e-9);
            let radius = profile.iter().fold(0.0f64, |m, q| m.max(q.1)).max(1e-9);
            let top = profile.iter().map(|&(z, r)| (z, r + radius));
            let bottom = profile.iter().rev().map(|&(z, r)| (z, radius - r));
            let outline: Vec<(f64, f64)> = top.chain(bottom).collect();
            // A long tube would be a hairline; cap the drawn aspect at 3 : 1.
            let width = length.min(6.0 * radius);
            let outline: Vec<(f64, f64)> = outline.iter().map(|&(z, y)| (z * width / length, y)).collect();
            vec![poly(&fit(&outline, width, 2.0 * radius), true)]
        }
        PartRef::Radial(r) => match &r.kind {
            RadialKind::Pod { .. } => vec![poly(&[(0.92, 0.28), (0.36, 0.28), (0.08, 0.5), (0.36, 0.72), (0.92, 0.72)], true)],
            RadialKind::LaunchLug { .. } => vec![rect(0.15, 0.4, 0.85, 0.6, true)],
            RadialKind::RailButton { .. } => {
                vec![poly(&[(0.35, 0.8), (0.35, 0.5), (0.2, 0.5), (0.2, 0.2), (0.8, 0.2), (0.8, 0.5), (0.65, 0.5), (0.65, 0.8)], true)]
            }
            kind => {
                let planform = kind.planform().unwrap_or_default();
                let (width, height) = (kind.extent().max(1e-9), kind.fin_span().unwrap_or(0.0).max(1e-9));
                let extent = planform.iter().fold(width, |m, p| m.max(p.0));
                vec![poly(&fit(&planform, extent, height), true)]
            }
        },
        PartRef::Internal(p) => match &p.kind {
            InternalKind::InnerTube { .. } | InternalKind::Coupler { .. } => vec![rect(0.08, 0.3, 0.92, 0.7, false)],
            InternalKind::CenteringRing { .. } | InternalKind::EngineBlock { .. } => vec![rect(0.38, 0.1, 0.62, 0.35, true), rect(0.38, 0.65, 0.62, 0.9, true)],
            InternalKind::Bulkhead { .. } => vec![rect(0.42, 0.1, 0.58, 0.9, true)],
            InternalKind::Parachute { .. } => {
                let dome: Vec<(f32, f32)> = (0..=10)
                    .map(|i| {
                        let a = std::f32::consts::PI * i as f32 / 10.0;
                        (0.5 - 0.42 * a.cos(), 0.55 - 0.45 * a.sin())
                    })
                    .collect();
                vec![poly(&dome, true), line(&[(0.08, 0.55), (0.5, 0.95), (0.92, 0.55)])]
            }
            InternalKind::Streamer { .. } => vec![poly(&[(0.08, 0.35), (0.45, 0.45), (0.92, 0.3), (0.92, 0.6), (0.45, 0.75), (0.08, 0.65)], true)],
            InternalKind::ShockCord { .. } => vec![line(&[(0.08, 0.7), (0.3, 0.3), (0.5, 0.7), (0.7, 0.3), (0.92, 0.7)])],
            InternalKind::MassComponent { role, .. } => match role {
                MassRole::Ballast => vec![poly(&[(0.2, 0.8), (0.3, 0.3), (0.7, 0.3), (0.8, 0.8)], true)],
                _ => vec![rect(0.12, 0.25, 0.88, 0.75, true)],
            },
        },
    };
    (shapes, system)
}

/// The colour a system's parts are drawn in.
pub fn system_color(t: &Tokens, system: System) -> Color {
    match system {
        System::Structure => t.part_body,
        System::Aero => t.sys_aero,
        System::Propulsion => t.sys_propulsion,
        System::Recovery => t.sys_recovery,
        System::Avionics => t.sys_avionics,
    }
}

struct Drawing {
    shapes: Vec<Shape>,
    system: System,
}

impl<M> canvas::Program<M> for Drawing {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let color = system_color(tokens(theme), self.system);
        let mut frame = Frame::new(renderer, bounds.size());
        let at = |p: &(f32, f32)| Point::new(p.0 * bounds.width, p.1 * bounds.height);
        for shape in &self.shapes {
            let Some((first, rest)) = shape.points.split_first() else {
                continue;
            };
            let closed = shape.closed;
            let path = Path::new(|b| {
                b.move_to(at(first));
                rest.iter().for_each(|p| b.line_to(at(p)));
                if closed {
                    b.close();
                }
            });
            if shape.fill {
                frame.fill(&path, color);
            } else {
                frame.stroke(&path, Stroke::default().with_color(color).with_width(1.5).with_line_cap(LineCap::Square));
            }
        }
        vec![frame.into_geometry()]
    }
}

/// The silhouette of part `id`, `SIZE` px.
pub fn silhouette<'a, M: 'a>(rocket: &Rocket, id: PartId) -> Element<'a, M> {
    // The kit draws the header icon in the neutral part colour whatever the part's system.
    let (shapes, _) = shapes(rocket, id);
    canvas::Canvas::new(Drawing { shapes, system: System::Structure }).width(SIZE.0).height(SIZE.1).boxed()
}

/// The silhouette of the whole rocket's profile (the Rocket inspector's header): the body stack mirrored about the axis.
pub fn rocket_silhouette<'a, M: 'a>(rocket: &Rocket) -> Element<'a, M> {
    let stations = rocket.stations();
    let mut top: Vec<(f64, f64)> = Vec::new();
    for (part, &z0) in rocket.parts().iter().zip(&stations) {
        top.extend(part.kind.profile().into_iter().map(|(z, r)| (z0 + z, r)));
    }
    let length = rocket.length().max(1e-9);
    let radius = top.iter().fold(0.0f64, |m, p| m.max(p.1)).max(1e-9);
    let width = length.min(8.0 * radius);
    let scale = width / length;
    let outline: Vec<(f64, f64)> = top.iter().map(|&(z, r)| (z * scale, r + radius)).chain(top.iter().rev().map(|&(z, r)| (z * scale, radius - r))).collect();
    let shapes = if top.is_empty() { Vec::new() } else { vec![poly(&fit(&outline, width, 2.0 * radius), true)] };
    canvas::Canvas::new(Drawing { shapes, system: System::Structure }).width(SIZE.0).height(SIZE.1).boxed()
}

/// A chain-link glyph, `size` px, in the `LINKED` tag's colour.
pub fn link<'a, M: 'a>(size: f32, color: fn(&Tokens) -> Color) -> Element<'a, M> {
    canvas::Canvas::new(Link { color, broken: false }).width(size).height(size).boxed()
}

/// A broken chain-link glyph (the two links pulled apart), `size` px: a field one copy has unlinked from its group.
pub fn unlink<'a, M: 'a>(size: f32, color: fn(&Tokens) -> Color) -> Element<'a, M> {
    canvas::Canvas::new(Link { color, broken: true }).width(size).height(size).boxed()
}

struct Link {
    color: fn(&Tokens) -> Color,
    broken: bool,
}

impl<M> canvas::Program<M> for Link {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let mut frame = Frame::new(renderer, bounds.size());
        let u = bounds.width.min(bounds.height) / 14.0;
        let stroke = Stroke::default().with_color((self.color)(tokens(theme))).with_width((1.4 * u).max(1.0)).with_line_cap(LineCap::Round);
        // Two stadium links, tilted 45 degrees, overlapping in the middle (pulled apart when broken).
        let (rx, ry) = (3.7 * u, 2.1 * u);
        let gap = if self.broken { 1.9 } else { 0.0 };
        for (cx, cy) in [(4.8 - gap, 9.2 + gap), (9.2 + gap, 4.8 - gap)] {
            frame.with_save(|f| {
                f.translate(iced::Vector::new(cx * u, cy * u));
                f.rotate(-std::f32::consts::FRAC_PI_4);
                f.stroke(&Path::rounded_rectangle(Point::new(-rx, -ry), iced::Size::new(2.0 * rx, 2.0 * ry), ry.into()), stroke);
            });
        }
        vec![frame.into_geometry()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::App;

    #[test]
    fn every_part_of_the_example_has_a_silhouette_in_its_system_colour() {
        let app = App::with_example();
        let rocket = app.ctx().doc.rocket();
        for id in rocket.ids() {
            let (shapes, system) = shapes(rocket, id);
            assert!(!shapes.is_empty(), "no silhouette for part {id}");
            let want = rocket.kind_of(id).unwrap().system();
            assert_eq!(system, want);
            for shape in &shapes {
                assert!(shape.points.iter().all(|&(x, y)| (-1e-3..=1.0 + 1e-3).contains(&x) && (-1e-3..=1.0 + 1e-3).contains(&y)), "{id}: {:?}", shape.points);
            }
        }
        assert!(shapes(rocket, 999).0.is_empty());
        assert_eq!(rocket.kind_of(rocket.parts()[0].id).unwrap().system(), System::Structure);
    }
}
