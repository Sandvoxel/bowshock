//! The Bowshock mark (design/components/Logo): a detached bow shock (a hyperbola whose arms tend to the Mach 2.4 angle) standing
//! ahead of a Von Kármán nose, on the kit's 22-unit grid, plus the same geometry as the application icon.
//!
//! * The geometry ([`nose`], [`shock`]) is plain points in grid units, shared by the canvas [`Mark`] (top bar), the Cover drawn on
//!   an empty viewport (`viewport/empty.rs`) and the icon raster ([`icon_rgba`]).
//! * Three cuts ([`Cut::for_size`]): the master (2 u stroke) from 24 px up, the same with a 2.4 u stroke at 20 px, and the 16 px
//!   optical cut (3 u stroke, shorter arms, a fatter nose) below that.
//! * The application icon is the mark on `bg-100` in a 1 px `line-200` frame, rendered at [`ICON_SIZES`] with tiny-skia.

use iced::widget::canvas::{self, Frame, Geometry, LineCap, LineJoin, Path, Stroke};
use iced::{mouse, Color, Point, Rectangle, Renderer, Theme};

use crate::theme;
use crate::tokens::DARK;

/// The mark's grid is 22 units square; the nose axis is at 11.
pub const GRID: f32 = 22.0;
/// The nose axis, in grid units.
pub const AXIS: f32 = 11.0;
/// The freestream Mach number the shock is drawn for.
pub const MACH: f32 = 2.4;

/// A cut of the mark: the dimensions that change between the master and the 16 px optical cut. Grid units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cut {
    /// Shock stroke width.
    pub stroke: f32,
    /// Where the shock crosses the axis (the centre of its stroke).
    pub apex: f32,
    /// How far from the axis the shock arms end.
    pub arm: f32,
    /// Radius of curvature of the shock at the axis.
    pub curvature: f32,
    /// Where the nose tip is.
    pub tip: f32,
    /// Nose length.
    pub nose_length: f32,
    /// Nose radius.
    pub nose_radius: f32,
}

impl Cut {
    /// The master: 2 u stroke, shock standing 3.6 u ahead of a 14 × Ø8 nose.
    pub const MASTER: Cut = Cut { stroke: 2.0, apex: 3.0, arm: 9.2, curvature: 2.0, tip: 6.6, nose_length: 14.0, nose_radius: 4.0 };
    /// The master at 20 px, where the stroke is 2.4 u so it still reads.
    pub const AT_20: Cut = Cut { stroke: 2.4, ..Cut::MASTER };
    /// The 16 px optical cut: heavier 3 u stroke, shorter arms, a fatter nose.
    pub const SMALL: Cut = Cut { stroke: 3.0, apex: 2.6, arm: 8.6, curvature: 2.4, tip: 6.4, nose_length: 14.4, nose_radius: 4.6 };

    /// The cut for a mark drawn `px` pixels square: the optical cut at 16 px and below, a 2.4 u stroke up to 20 px, else the master.
    pub fn for_size(px: f32) -> Cut {
        if px <= 16.0 {
            Cut::SMALL
        } else if px <= 20.0 {
            Cut::AT_20
        } else {
            Cut::MASTER
        }
    }

    /// The nose outline of this cut on the grid.
    pub fn nose(&self) -> Vec<[f32; 2]> {
        nose(self.tip, self.nose_length, self.nose_radius, NOSE_STEPS)
    }

    /// The shock centre line of this cut on the grid.
    pub fn shock(&self) -> Vec<[f32; 2]> {
        shock(self.apex, self.curvature, self.arm, SHOCK_STEPS)
    }
}

const NOSE_STEPS: usize = 64;
const SHOCK_STEPS: usize = 64;

/// The Mach angle μ = asin(1/M) the shock arms tend to (24.6° at Mach 2.4).
pub fn mach_angle() -> f32 {
    (1.0 / MACH).asin()
}

/// A Von Kármán (Haack C = 0) nose with its tip at `tip` on the axis, `length` long and `radius` at the base, pointing left, as a
/// closed outline (upper side from the tip to the base, then the lower side back): `y = R·√((θ − sin 2θ / 2) / π)` with
/// `θ = acos(1 − 2x/L)`. Sampled uniformly in θ so the steep tip gets the points it needs.
pub fn nose(tip: f32, length: f32, radius: f32, steps: usize) -> Vec<[f32; 2]> {
    let half = |i: usize| {
        let th = std::f32::consts::PI * i as f32 / steps as f32;
        let x = length * (1.0 - th.cos()) / 2.0;
        let y = radius * ((th - (2.0 * th).sin() / 2.0) / std::f32::consts::PI).max(0.0).sqrt();
        (tip + x, y)
    };
    let upper = (0..=steps).map(|i| {
        let (x, y) = half(i);
        [x, AXIS - y]
    });
    let lower = (0..steps).rev().map(|i| {
        let (x, y) = half(i);
        [x, AXIS + y]
    });
    upper.chain(lower).collect()
}

/// The shock's centre line: the hyperbola `x = apex + a(√(1 + (y/b)²) − 1)`, normal to the flow on the axis (where its radius of
/// curvature is `curvature`) and straightening into arms that tend to the Mach angle, from `AXIS − arm` to `AXIS + arm`.
pub fn shock(apex: f32, curvature: f32, arm: f32, steps: usize) -> Vec<[f32; 2]> {
    // Asymptote slope dy/dx = b/a = tan μ, and the radius of curvature at the apex is b²/a = b·tan μ.
    let tan_mu = mach_angle().tan();
    let b = curvature / tan_mu;
    let a = b / tan_mu;
    (0..=steps)
        .map(|i| {
            let y = -arm + 2.0 * arm * i as f32 / steps as f32;
            [apex + a * ((1.0 + (y / b).powi(2)).sqrt() - 1.0), AXIS + y]
        })
        .collect()
}

/// `points` (grid units) as an open or closed canvas path at `scale` pixels per unit from `origin`.
pub fn path(points: &[[f32; 2]], origin: Point, scale: f32, closed: bool) -> Path {
    Path::new(|p| {
        for (i, [x, y]) in points.iter().enumerate() {
            let pt = Point::new(origin.x + x * scale, origin.y + y * scale);
            if i == 0 {
                p.move_to(pt);
            } else {
                p.line_to(pt);
            }
        }
        if closed {
            p.close();
        }
    })
}

/// A stroke for the shock: square caps, round joins.
pub fn shock_stroke(color: Color, width: f32) -> Stroke<'static> {
    Stroke::default().with_color(color).with_width(width).with_line_cap(LineCap::Square).with_line_join(LineJoin::Round)
}

/// The canvas program that draws the mark, square, filling the smaller side of its bounds.
pub struct Mark;

impl<Message> canvas::Program<Message> for Mark {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let mut frame = Frame::new(renderer, bounds.size());
        let side = bounds.width.min(bounds.height);
        let u = side / GRID;
        let origin = Point::new((bounds.width - side) / 2.0, (bounds.height - side) / 2.0);
        let cut = Cut::for_size(side);
        frame.fill(&path(&cut.nose(), origin, u, true), t.ink);
        frame.stroke(&path(&cut.shock(), origin, u, false), shock_stroke(t.accent, cut.stroke * u));
        vec![frame.into_geometry()]
    }
}

/// Sizes the application icon is rendered at, largest first.
pub const ICON_SIZES: [u32; 4] = [256, 64, 32, 16];

/// The application icon at `px` pixels square as straight RGBA: the mark on `bg-100` in a 1 px `line-200` frame (square-cornered,
/// the OS rounds it). The mark fills 61 % of the icon (the kit's 44 of 72), 75 % at 16 px so the optical cut has 12 px; it
/// uses the cut for its drawn size ([`Cut::for_size`]). Always the dark palette: an icon does not follow the theme.
pub fn icon_rgba(px: u32) -> Vec<u8> {
    use tiny_skia::{FillRule, Paint, PathBuilder, Pixmap, Stroke as SkStroke, Transform};

    let paint = |c: Color| {
        let [r, g, b, a] = c.into_rgba8();
        let mut p = Paint::default();
        p.set_color_rgba8(r, g, b, a);
        p.anti_alias = true;
        p
    };
    let mut pixmap = Pixmap::new(px, px).expect("icon sizes are non-zero");
    let side = px as f32;
    let [r, g, b, _] = DARK.bg_100.into_rgba8();
    pixmap.fill(tiny_skia::Color::from_rgba8(r, g, b, 255));

    // The frame: a 1 px line inside the edge.
    let frame = PathBuilder::from_rect(tiny_skia::Rect::from_xywh(0.5, 0.5, side - 1.0, side - 1.0).expect("a square"));
    pixmap.stroke_path(&frame, &paint(DARK.line_200), &SkStroke { width: 1.0, ..SkStroke::default() }, Transform::identity(), None);

    // The mark, centred.
    let mark = side * if px <= 16 { 0.75 } else { 44.0 / 72.0 };
    let u = mark / GRID;
    let origin = (side - mark) / 2.0;
    let cut = Cut::for_size(mark);
    let build = |points: &[[f32; 2]], closed: bool| {
        let mut b = PathBuilder::new();
        for (i, [x, y]) in points.iter().enumerate() {
            let (x, y) = (origin + x * u, origin + y * u);
            if i == 0 {
                b.move_to(x, y);
            } else {
                b.line_to(x, y);
            }
        }
        if closed {
            b.close();
        }
        b.finish().expect("the mark has points")
    };
    pixmap.fill_path(&build(&cut.nose(), true), &paint(DARK.ink), FillRule::Winding, Transform::identity(), None);
    let stroke = SkStroke { width: cut.stroke * u, line_cap: tiny_skia::LineCap::Square, line_join: tiny_skia::LineJoin::Round, ..SkStroke::default() };
    pixmap.stroke_path(&build(&cut.shock(), false), &paint(DARK.accent), &stroke, Transform::identity(), None);

    // The background is opaque and the marks are drawn over it, so premultiplied and straight RGBA are the same.
    pixmap.take()
}

/// The window icon: [`icon_rgba`] at its largest size (the OS scales it for the title bar and the task bar).
pub fn window_icon() -> Option<iced::window::Icon> {
    let px = ICON_SIZES[0];
    iced::window::icon::from_rgba(icon_rgba(px), px, px).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pixel(rgba: &[u8], px: u32, x: u32, y: u32) -> [u8; 4] {
        let i = ((y * px + x) * 4) as usize;
        [rgba[i], rgba[i + 1], rgba[i + 2], rgba[i + 3]]
    }

    /// The nose is the Von Kármán profile of the kit (Haack series, C = 0): 14 u long and 8 u across, tip at 6.6.
    #[test]
    fn the_nose_is_the_von_karman_profile() {
        let nose = Cut::MASTER.nose();
        let tip = nose[0];
        assert_eq!(tip, [6.6, AXIS]);
        let (max_x, max_y) = nose.iter().fold((f32::MIN, f32::MIN), |(mx, my), p| (mx.max(p[0]), my.max((p[1] - AXIS).abs())));
        assert!((max_x - 20.6).abs() < 1e-4 && (max_y - 4.0).abs() < 1e-4, "{max_x} {max_y}");
        // Half way along: y = R sqrt((π/2 - 0) / π) = R / sqrt 2 (θ = π/2: sin 2θ = 0).
        let mid = nose.iter().find(|p| (p[0] - 13.6).abs() < 1e-3 && p[1] < AXIS).expect("a point at half length");
        assert!((AXIS - mid[1] - 4.0 / 2f32.sqrt()).abs() < 1e-3, "{mid:?}");
    }

    /// The shock stands 3.6 u ahead of the nose, is normal to the flow on the axis and its arms tend to the Mach 2.4 angle.
    #[test]
    fn the_shock_is_a_mach_hyperbola_standing_3_6_ahead() {
        let c = Cut::MASTER;
        assert!((c.tip - c.apex - 3.6).abs() < 1e-6);
        let line = c.shock();
        let mid = line[line.len() / 2];
        assert!((mid[0] - c.apex).abs() < 1e-5 && (mid[1] - AXIS).abs() < 1e-5, "{mid:?}");
        // The kit's arm ends: (15.70, 1.80) and (15.70, 20.20).
        let end = line[0];
        assert!((end[0] - 15.70).abs() < 0.01 && (end[1] - 1.8).abs() < 1e-4, "{end:?}");
        // Slope of the last segment against the Mach angle: tan 24.6° = 0.458 (the arms are close to, not yet at, asymptotic).
        let (a, b) = (line[1], line[0]);
        let slope = (b[1] - a[1]).abs() / (a[0] - b[0]).abs();
        assert!((slope - mach_angle().tan()).abs() < 0.12, "{slope}");
        assert!((mach_angle().to_degrees() - 24.62).abs() < 0.01);
        // Normal to the flow on the axis: symmetric about it.
        assert!((line[0][0] - line[line.len() - 1][0]).abs() < 1e-5);
    }

    #[test]
    fn the_cut_follows_the_drawn_size() {
        assert_eq!(Cut::for_size(64.0), Cut::MASTER);
        assert_eq!(Cut::for_size(32.0), Cut::MASTER);
        assert_eq!(Cut::for_size(20.0).stroke, 2.4);
        assert_eq!(Cut::for_size(16.0), Cut::SMALL);
        assert_eq!(Cut::for_size(10.0), Cut::SMALL);
        // The optical cut: heavier stroke, shorter arms, fatter nose.
        const { assert!(Cut::SMALL.stroke > Cut::AT_20.stroke && Cut::SMALL.arm < Cut::MASTER.arm && Cut::SMALL.nose_radius > Cut::MASTER.nose_radius) };
    }

    /// The icon at every size: opaque `bg-100` ground, a `line-200` frame, the ink nose and the accent shock in the right places.
    #[test]
    fn the_icon_is_the_mark_on_bg_100_in_a_line_200_frame() {
        let rgb = |c: Color| {
            let [r, g, b, _] = c.into_rgba8();
            [r, g, b, 255]
        };
        for px in ICON_SIZES {
            let rgba = icon_rgba(px);
            assert_eq!(rgba.len(), (px * px * 4) as usize);
            assert!(rgba.chunks(4).all(|p| p[3] == 255), "{px}: opaque");
            assert_eq!(pixel(&rgba, px, px / 2, 0), rgb(DARK.line_200), "{px}: frame on the top edge");
            assert_eq!(pixel(&rgba, px, 0, px / 2), rgb(DARK.line_200), "{px}: frame on the left edge");
            assert_eq!(pixel(&rgba, px, 1, 1), rgb(DARK.bg_100), "{px}: ground just inside the frame");
            // The nose base sits at x = 20.6 on the axis (11) of the 22-unit grid; the shock apex at x = 3.0 (2.6 in the cut).
            let mark = px as f32 * if px <= 16 { 0.75 } else { 44.0 / 72.0 };
            let cut = Cut::for_size(mark);
            let to_px = |gx: f32, gy: f32| {
                let o = (px as f32 - mark) / 2.0;
                (((o + gx * mark / GRID) as u32).min(px - 1), ((o + gy * mark / GRID) as u32).min(px - 1))
            };
            let (nx, ny) = to_px(cut.tip + cut.nose_length * 0.8, AXIS);
            assert_eq!(pixel(&rgba, px, nx, ny), rgb(DARK.ink), "{px}: ink inside the nose");
            let (sx, sy) = to_px(cut.apex, AXIS);
            let p = pixel(&rgba, px, sx, sy);
            assert!(p[0] > 200 && p[2] < 120, "{px}: the shock is accent-coloured at its apex, got {p:?}");
        }
        assert_eq!(Cut::for_size(16.0 * 0.75), Cut::SMALL, "the 16 px icon uses the optical cut");
        assert_eq!(Cut::for_size(64.0 * 44.0 / 72.0), Cut::MASTER);
    }

    /// With `BOWSHOCK_WRITE_ICONS=<dir>` the icons are written as PNGs for inspection.
    #[test]
    fn writes_icon_pngs_on_request() {
        let Some(dir) = std::env::var_os("BOWSHOCK_WRITE_ICONS") else { return };
        std::fs::create_dir_all(&dir).unwrap();
        for px in ICON_SIZES {
            let file = std::fs::File::create(std::path::Path::new(&dir).join(format!("icon-{px}.png"))).unwrap();
            let mut enc = png::Encoder::new(std::io::BufWriter::new(file), px, px);
            enc.set_color(png::ColorType::Rgba);
            enc.set_depth(png::BitDepth::Eight);
            enc.write_header().unwrap().write_image_data(&icon_rgba(px)).unwrap();
        }
    }
}
