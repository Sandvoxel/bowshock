//! Stations to pixels: the horizontal [`Axis`] (fit, user zoom/pan), the rows the lanes occupy for a given
//! height, the ruler's ticks and hit-testing. Pure, so it is unit-tested.

use iced::{Point, Rectangle};

use super::scene::{Bar, BarKey, Lane, Scene};

use crate::units::Unit;

/// Width of the lane-name column on the left.
pub const LABEL_W: f32 = 84.0;
/// Height of the ruler.
pub const RULER_H: f32 = 24.0;
/// Height of the footer under the lanes (CG / CP labels, margin bar).
pub const FOOTER_H: f32 = 20.0;
/// Padding either side of the rocket at the default fit.
const PAD_L: f32 = 8.0;
const PAD_R: f32 = 20.0;
/// Pitch of sub-rows and the extra height each one adds to a lane.
pub const SUB_PITCH: f32 = 22.0;
/// Height of a bar in a full-size row, and the gap above it.
pub const BAR_H: f32 = 20.0;
const BAR_GAP: f32 = 4.0;
/// Shortest bar drawn, px: a ring is a block you can see and grab.
pub const MIN_BAR_W: f32 = 6.0;
/// Width of the grab zone at either end of a bar.
pub const EDGE_W: f32 = 5.0;

/// Stations to pixels: `x = x0 + ppm * station`. `ppm` is negative when the viewport looks at the rocket from the other side.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Axis {
    /// Pixel of station 0 (the tip).
    pub x0: f32,
    /// Pixels per metre.
    pub ppm: f32,
}

impl Axis {
    /// The default: the rocket's length between the lane names and the right edge.
    pub fn fit(width: f32, length: f64) -> Axis {
        let usable = (width - LABEL_W - PAD_L - PAD_R).max(40.0);
        Axis { x0: LABEL_W + PAD_L, ppm: usable / length.max(0.05) as f32 }
    }

    /// Pixel of `station`.
    pub fn px(&self, station: f64) -> f32 {
        self.x0 + self.ppm * station as f32
    }

    /// Station under pixel `x`.
    pub fn station(&self, x: f32) -> f64 {
        f64::from((x - self.x0) / self.ppm)
    }

    /// This axis zoomed by `k` about pixel `anchor` (the station under it stays put), its scale kept between `min` and `max`
    /// pixels per metre.
    pub fn zoomed(self, k: f32, anchor: f32, min: f32, max: f32) -> Axis {
        let ppm = (self.ppm.abs() * k).clamp(min, max) * self.ppm.signum();
        let k = ppm / self.ppm;
        Axis { x0: anchor + (self.x0 - anchor) * k, ppm }
    }

    /// This axis shifted by `dx` px.
    pub fn panned(self, dx: f32) -> Axis {
        Axis { x0: self.x0 + dx, ..self }
    }

    /// The pixel span of a bar `[start, end]`, left to right and at least [`MIN_BAR_W`] wide.
    pub fn span(&self, start: f64, end: f64) -> (f32, f32) {
        let (a, b) = (self.px(start), self.px(end));
        let (left, right) = (a.min(b), a.max(b));
        (left, right.max(left + MIN_BAR_W))
    }
}

/// The user's zoom and pan of the default axis: `x' = factor * x + offset`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Zoom {
    /// Scale factor on the fit (1 is the whole rocket).
    pub factor: f32,
    /// Shift in px.
    pub offset: f32,
}

impl Default for Zoom {
    fn default() -> Self {
        Zoom { factor: 1.0, offset: 0.0 }
    }
}

impl Zoom {
    /// The axis after zooming `base`.
    pub fn apply(&self, base: Axis) -> Axis {
        Axis { x0: base.x0 * self.factor + self.offset, ppm: base.ppm * self.factor }
    }

    /// The zoom that turns the default axis `fit` into `axis`.
    pub fn between(fit: Axis, axis: Axis) -> Zoom {
        let factor = axis.ppm / fit.ppm;
        Zoom { factor, offset: axis.x0 - factor * fit.x0 }
    }
}

/// Where one lane sits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LaneBox {
    /// The lane.
    pub lane: Lane,
    /// Top edge.
    pub y: f32,
    /// Height.
    pub h: f32,
    /// How much of the natural size it was drawn at (1 unless the lanes had to be squeezed into the panel).
    pub k: f32,
}

impl LaneBox {
    /// The rectangle of sub-row `sub` (full width is the caller's: `x`/`w` here are 0).
    pub fn row(&self, sub: usize) -> (f32, f32) {
        (self.y + (BAR_GAP + SUB_PITCH * sub as f32) * self.k, BAR_H * self.k)
    }
}

/// The lanes' vertical layout for a panel `height` high: natural lane heights (28 px + 22 per extra sub-row) squeezed
/// uniformly when they would not fit between the ruler and the footer.
pub fn rows(lanes: &[Lane], height: f32, lane_h: f32) -> Vec<LaneBox> {
    let natural = |l: &Lane| lane_h + SUB_PITCH * (l.subs as f32 - 1.0);
    let total: f32 = lanes.iter().map(natural).sum();
    let room = (height - RULER_H - FOOTER_H).max(1.0);
    let k = if total > room { room / total } else { 1.0 };
    let mut y = RULER_H;
    lanes
        .iter()
        .map(|lane| {
            let h = natural(lane) * k;
            let b = LaneBox { lane: *lane, y, h, k };
            y += h;
            b
        })
        .collect()
}

/// The rectangle of `bar` in the panel.
pub fn bar_rect(bar: &Bar, lanes: &[LaneBox], axis: &Axis) -> Option<Rectangle> {
    let lane = lanes.iter().find(|l| l.lane.system == bar.system)?;
    let (y, h) = lane.row(bar.sub);
    let (x0, x1) = axis.span(bar.start, bar.end);
    // A shock cord is a thin dashed block, not a full bar.
    let (y, h) = if bar.dashed && !bar.alt { (y + h * 0.3, h * 0.4) } else { (y, h) };
    Some(Rectangle { x: x0, y, width: x1 - x0, height: h })
}

/// Which part of a bar the cursor is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Zone {
    /// The body: move.
    Body,
    /// The fore end.
    Fore,
    /// The aft end.
    Aft,
}

/// A bar under the cursor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    /// The bar.
    pub key: BarKey,
    /// The zone of it.
    pub zone: Zone,
}

/// Indices of the scene's bars in the order they are painted: other configurations' motors, short bars, then (as dashed outlines) other configurations' motors on top.
pub fn paint_order(scene: &Scene) -> Vec<usize> {
    let mut order: Vec<usize> = (0..scene.bars.len()).collect();
    order.sort_by_key(|&i| {
        let b = &scene.bars[i];
        (b.thin, b.alt)
    });
    order
}

/// The topmost bar at `p`. Other configurations' motors only count where they are drawn as dashed outlines (they are
/// not selectable themselves, but hovering names them).
pub fn hit(scene: &Scene, lanes: &[LaneBox], axis: &Axis, p: Point) -> Option<Hit> {
    if p.x < LABEL_W {
        return None;
    }
    let mut best: Option<(&Bar, Rectangle)> = None;
    for i in paint_order(scene) {
        let bar = &scene.bars[i];
        let Some(rect) = bar_rect(bar, lanes, axis) else { continue };
        // A little slack so a 6 px ring can be hit.
        let grab = Rectangle { x: rect.x - 1.0, y: rect.y - 1.0, width: rect.width + 2.0, height: rect.height + 2.0 };
        if grab.contains(p) && !(bar.alt && best.is_some_and(|(b, _)| !b.alt)) {
            best = Some((bar, rect));
        }
    }
    let (bar, rect) = best?;
    let edge = EDGE_W.min(rect.width / 3.0);
    let zone = if rect.width >= 20.0 && bar.edit.resize_fore && p.x <= rect.x + edge {
        Zone::Fore
    } else if rect.width >= 20.0 && bar.edit.resize_aft && p.x >= rect.x + rect.width - edge {
        Zone::Aft
    } else {
        Zone::Body
    };
    // With the axis flipped the fore end is on the right.
    let zone = if axis.ppm < 0.0 {
        match zone {
            Zone::Fore => Zone::Aft,
            Zone::Aft => Zone::Fore,
            z => z,
        }
    } else {
        zone
    };
    Some(Hit { key: bar.key, zone })
}

/// The lane under `y`.
pub fn lane_at(lanes: &[LaneBox], y: f32) -> Option<&LaneBox> {
    lanes.iter().find(|l| y >= l.y && y < l.y + l.h)
}

/// One tick of the ruler.
#[derive(Clone, Debug, PartialEq)]
pub struct Tick {
    /// Pixel.
    pub x: f32,
    /// 8 px (labelled), 5 px or 3 px high.
    pub height: f32,
    /// The number, for the labelled ticks.
    pub label: Option<String>,
}

/// The ruler ticks visible between pixels `from` and `to`, in the display `unit`: a labelled tick every 1, 2 or 5 × 10ⁿ units
/// (at least 56 px apart), shorter ones between.
pub fn ticks(axis: &Axis, from: f32, to: f32, unit: &Unit) -> Vec<Tick> {
    let px_per_unit = axis.ppm.abs() as f64 * unit.to_si;
    if px_per_unit <= 0.0 || !px_per_unit.is_finite() {
        return Vec::new();
    }
    let major = nice_step(56.0 / px_per_unit);
    let mantissa = (major / 10f64.powf(major.log10().floor())).round() as u32;
    let fine = if mantissa == 2 { major / 4.0 } else { major / 10.0 };
    let (a, b) = (axis.station(from.min(to)).min(axis.station(from.max(to))), axis.station(from.min(to)).max(axis.station(from.max(to))));
    let (lo, hi) = (unit.from_si(a.min(b)), unit.from_si(a.max(b)));
    let first = (lo / fine).floor() as i64;
    let last = (hi / fine).ceil() as i64;
    let per_major = (major / fine).round() as i64;
    let decimals = if major >= 1.0 { 0 } else { (-major.log10().floor()) as usize };
    let mut ticks = Vec::new();
    for i in first..=last {
        let value = i as f64 * fine;
        let x = axis.px((value - unit.offset) * unit.to_si);
        if x < from.min(to) - 1.0 || x > from.max(to) + 1.0 {
            continue;
        }
        let (height, label) = if i.rem_euclid(per_major) == 0 {
            (8.0, Some(format!("{:.*}", decimals, value)))
        } else if (per_major % 2 == 0) && i.rem_euclid(per_major) == per_major / 2 {
            (5.0, None)
        } else {
            (3.0, None)
        };
        ticks.push(Tick { x, height, label });
    }
    ticks
}

/// The smallest 1, 2 or 5 × 10ⁿ not below `at_least`.
fn nice_step(at_least: f64) -> f64 {
    let exp = at_least.log10().floor();
    let base = 10f64.powf(exp);
    [1.0, 2.0, 5.0, 10.0].into_iter().map(|m| m * base).find(|&s| s >= at_least - 1e-12).unwrap_or(10.0 * base)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{Quantity, UnitSystem};
    use bowshock_model::System;

    fn mm() -> Unit {
        *Quantity::Length.unit_for(UnitSystem::MmGS)
    }

    #[test]
    fn fit_puts_the_tip_after_the_lane_names_and_the_tail_before_the_edge() {
        let axis = Axis::fit(700.0, 1.142);
        assert!((axis.px(0.0) - 92.0).abs() < 1e-3);
        assert!((axis.px(1.142) - 680.0).abs() < 1e-2);
        assert!((axis.station(axis.px(0.5)) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn zoom_keeps_the_station_under_the_cursor_and_clamps_the_scale() {
        let fit = Axis::fit(700.0, 1.0);
        let zoomed = fit.zoomed(3.0, 300.0, fit.ppm, fit.ppm * 400.0);
        assert!((fit.station(300.0) - zoomed.station(300.0)).abs() < 1e-5);
        assert!((zoomed.ppm - 3.0 * fit.ppm).abs() < 1e-3);
        // Zooming out past the fit stops at the fit; zooming in stops at the limit.
        assert_eq!(zoomed.zoomed(0.01, 300.0, fit.ppm, fit.ppm * 400.0).ppm, fit.ppm);
        assert!((fit.zoomed(1e6, 300.0, fit.ppm, fit.ppm * 400.0).ppm / fit.ppm - 400.0).abs() < 1e-3);
        // A pan is a plain shift, and Zoom::between recovers the user's view from the axis they made.
        let panned = zoomed.panned(25.0);
        assert!((panned.x0 - zoomed.x0 - 25.0).abs() < 1e-4);
        let z = Zoom::between(fit, panned);
        let again = z.apply(fit);
        assert!((again.x0 - panned.x0).abs() < 1e-3 && (again.ppm - panned.ppm).abs() < 1e-3);
        assert_eq!(Zoom::between(fit, fit), Zoom::default());
    }

    #[test]
    fn ticks_are_one_two_five_steps_at_least_56_px_apart() {
        // 1142 mm across 588 px: 0.51 px per mm, so the labelled tick is every 200 mm (102 px), like the kit's ruler.
        let axis = Axis::fit(700.0, 1.142);
        let ticks = ticks(&axis, LABEL_W, 700.0, &mm());
        let labels: Vec<&str> = ticks.iter().filter_map(|t| t.label.as_deref()).collect();
        assert_eq!(labels, ["0", "200", "400", "600", "800", "1000"]);
        assert!(ticks.iter().any(|t| t.height == 5.0) && ticks.iter().any(|t| t.height == 3.0));
        // 5 labelled ticks of 8 px between.
        let major: Vec<f32> = ticks.iter().filter(|t| t.label.is_some()).map(|t| t.x).collect();
        assert!(major.windows(2).all(|w| w[1] - w[0] >= 56.0));
    }

    #[test]
    fn ticks_follow_the_unit_system() {
        let inch = *Quantity::Length.unit_for(UnitSystem::InOzS);
        let axis = Axis::fit(700.0, 1.0);
        let labels: Vec<String> = ticks(&axis, LABEL_W, 700.0, &inch).into_iter().filter_map(|t| t.label).collect();
        assert!(labels.len() >= 3 && labels[0] == "0", "{labels:?}");
        // 39.37 in across 588 px: 14.9 px per inch, so labelled every 5 in.
        assert_eq!(labels[1], "5");
    }

    fn scene_with(lanes: &[(System, usize)]) -> Scene {
        Scene { lanes: lanes.iter().map(|&(system, subs)| Lane { system, subs }).collect(), ..Scene::default() }
    }

    #[test]
    fn rows_use_natural_heights_and_squeeze_to_fit() {
        // The kit's panel: 5 lanes, one of them two rows tall, in 206 px: 24 + 4 * 28 + 50 + 20.
        let scene = scene_with(&[(System::Structure, 1), (System::Aero, 1), (System::Propulsion, 2), (System::Recovery, 1), (System::Avionics, 1)]);
        let rows = rows(&scene.lanes, 206.0, 28.0);
        assert_eq!(rows[0].y, RULER_H);
        assert!((rows[2].h - 50.0).abs() < 1e-4 && (rows.last().unwrap().y + rows.last().unwrap().h - 186.0).abs() < 1e-3);
        assert!(rows.iter().all(|r| r.k == 1.0));
        // One more row does not fit: everything shrinks to keep the footer.
        let tall = scene_with(&[(System::Structure, 2), (System::Aero, 1), (System::Propulsion, 3), (System::Recovery, 1), (System::Avionics, 1)]);
        let squeezed = self::rows(&tall.lanes, 206.0, 28.0);
        let bottom = squeezed.last().map(|l| l.y + l.h).unwrap();
        assert!((bottom - (206.0 - FOOTER_H)).abs() < 1e-3 && squeezed[0].k < 1.0);
    }
}
