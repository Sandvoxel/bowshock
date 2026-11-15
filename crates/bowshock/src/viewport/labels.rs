//! The label collision pass of the manipulation overlay.
//!
//! A selected part shows a tag and, with every handle on show, one label per handle. Left alone they pile up: the labels of
//! neighbouring handles overprint each other, and the aft-view inset or the toolbar sit on top of them. [`layout`] places
//! the labels most important first: each one tries its candidate spots in order and takes the first that is inside the
//! area, clear of the fixed rectangles (toolbar, inset, handles) and clear of every label placed before it. A label with no
//! free spot is dropped; the caller lists the dropped ones in its hint.

use iced::{Point, Rectangle, Size, Vector};

/// Gap kept between two labels (and between a label and an obstacle), px.
const PAD: f32 = 1.0;
/// Distance between a handle and the nearest edge of its label, px.
const GAP: f32 = 8.0;
/// How much further out the second ring of candidate spots sits, px.
const NUDGE: f32 = 16.0;

/// One label to place.
#[derive(Debug, Clone)]
pub struct Request {
    /// Size of the label rectangle.
    pub size: Size,
    /// Lower is more important and is placed first (selection tag 0, primary dimensions 1, secondary ones 2).
    pub priority: u8,
    /// Top-left positions to try, in order of preference.
    pub spots: Vec<Point>,
}

impl Request {
    /// A label for the handle at `at`: the four corners around it (above-right first), then the same four a little further
    /// out along the leader from the handle.
    pub fn at_handle(at: Point, size: Size, priority: u8) -> Request {
        let corner = |sx: f32, sy: f32, extra: f32| {
            let x = if sx > 0.0 { at.x + GAP + extra } else { at.x - GAP - extra - size.width };
            let y = if sy > 0.0 { at.y + GAP + extra } else { at.y - GAP - extra - size.height };
            Point::new(x, y)
        };
        let mut spots = Vec::with_capacity(8);
        for extra in [0.0, NUDGE] {
            for (sx, sy) in [(1.0, -1.0), (1.0, 1.0), (-1.0, -1.0), (-1.0, 1.0)] {
                spots.push(corner(sx, sy, extra));
            }
        }
        Request { size, priority, spots }
    }

    /// Adds spots in the bands above and below the picture inside `area` (a panel), under the handle's x where the label fits.
    pub fn with_bands(mut self, at: Point, area: Rectangle, top: f32) -> Request {
        let (w, h) = (self.size.width, self.size.height);
        let x = (at.x - w / 2.0).clamp(area.x, (area.x + area.width - w).max(area.x));
        let rows = [area.y + top, area.y + area.height - h - 2.0, area.y + area.height - 2.0 * h - 4.0, area.y + area.height - 3.0 * h - 6.0];
        for y in rows {
            self.spots.push(Point::new(x, y));
            self.spots.push(Point::new(area.x + 2.0, y));
            self.spots.push(Point::new(area.x + area.width - w - 2.0, y));
        }
        self
    }

    /// The leader from the handle at `at` to the label placed at `rect`, when the label had to move away from the handle.
    pub fn leader(at: Point, rect: Rectangle) -> Option<(Point, Point)> {
        let near = Point::new(at.x.clamp(rect.x, rect.x + rect.width), at.y.clamp(rect.y, rect.y + rect.height));
        (near.distance(at) > GAP + 2.0).then_some((at, near))
    }
}

fn clear(rect: Rectangle, others: &[Rectangle]) -> bool {
    let grown = Rectangle::new(Point::new(rect.x - PAD, rect.y - PAD), Size::new(rect.width + 2.0 * PAD, rect.height + 2.0 * PAD));
    !others.iter().any(|o| o.intersects(&grown))
}

fn inside(rect: Rectangle, area: Rectangle) -> bool {
    rect.x >= area.x && rect.y >= area.y && rect.x + rect.width <= area.x + area.width && rect.y + rect.height <= area.y + area.height
}

/// Places `requests` inside `area`, clear of `blocked` and of each other. `soft` are things to prefer not to cover (other parts'
/// silhouettes): a spot clear of them wins, else one that only keeps clear of `blocked`. Returns the rectangle chosen for every
/// request (`None`: no free spot, the label is dropped), in the order of `requests`. Priority 0 requests always get a spot: when
/// none is free the one inside `area` covering the least of `blocked` is used, so the selection tag never disappears.
pub fn layout(requests: &[Request], blocked: &[Rectangle], soft: &[Rectangle], area: Rectangle) -> Vec<Option<Rectangle>> {
    let mut order: Vec<usize> = (0..requests.len()).collect();
    order.sort_by_key(|i| requests[*i].priority);
    let mut taken: Vec<Rectangle> = blocked.to_vec();
    let mut out = vec![None; requests.len()];
    for i in order {
        let r = &requests[i];
        let rect_at = |p: Point| Rectangle::new(p, r.size);
        let fits = || r.spots.iter().map(|p| rect_at(*p)).filter(|rect| inside(*rect, area));
        let found = fits().find(|rect| clear(*rect, &taken) && clear(*rect, soft)).or_else(|| fits().find(|rect| clear(*rect, &taken)));
        let found = found.or_else(|| {
            (r.priority == 0)
                .then(|| fits().min_by(|a, b| covered(*a, &taken).total_cmp(&covered(*b, &taken))).or_else(|| r.spots.first().map(|p| rect_at(*p))))
                .flatten()
        });
        if let Some(rect) = found {
            taken.push(rect);
            out[i] = Some(rect);
        }
    }
    out
}

/// Area of `rect` covered by `others`.
fn covered(rect: Rectangle, others: &[Rectangle]) -> f32 {
    others.iter().filter_map(|o| rect.intersection(o)).map(|i| i.width * i.height).sum()
}

/// A rectangle of `size` centred on `p`, for obstacles such as handle shapes.
pub fn around(p: Point, size: Size) -> Rectangle {
    Rectangle::new(p - Vector::new(size.width / 2.0, size.height / 2.0), size)
}

#[cfg(test)]
mod tests {
    use super::*;

    const AREA: Rectangle = Rectangle { x: 0.0, y: 0.0, width: 400.0, height: 300.0 };

    #[test]
    fn colliding_labels_are_nudged_apart_and_lower_priorities_give_way() {
        let size = Size::new(60.0, 12.0);
        // Two handles on top of each other: the first label takes the best corner, the second the next.
        let reqs = [Request::at_handle(Point::new(100.0, 100.0), size, 2), Request::at_handle(Point::new(100.0, 100.0), size, 1)];
        let out = layout(&reqs, &[], &[], AREA);
        let (a, b) = (out[0].unwrap(), out[1].unwrap());
        assert!(!a.intersects(&b));
        // The more important one (priority 1) has the preferred spot.
        assert_eq!(b.position(), reqs[1].spots[0]);
    }

    #[test]
    fn a_label_with_no_free_spot_is_dropped_but_the_tag_never_is() {
        let size = Size::new(60.0, 12.0);
        let wall = [Rectangle::new(Point::new(0.0, 0.0), Size::new(400.0, 300.0))];
        let out = layout(&[Request::at_handle(Point::new(100.0, 100.0), size, 1)], &wall, &[], AREA);
        assert!(out[0].is_none());
        let tag = Request { size, priority: 0, spots: vec![Point::new(10.0, 10.0)] };
        assert_eq!(layout(&[tag], &wall, &[], AREA)[0].map(|r| r.position()), Some(Point::new(10.0, 10.0)));
    }

    #[test]
    fn soft_obstacles_are_avoided_when_there_is_a_free_spot_and_given_up_when_not() {
        let size = Size::new(60.0, 12.0);
        let at = Point::new(100.0, 100.0);
        let req = Request::at_handle(at, size, 1);
        let first = Rectangle::new(req.spots[0], size);
        // The preferred spot is soft-blocked: another one is taken.
        let out = layout(std::slice::from_ref(&req), &[], &[first], AREA);
        assert!(!out[0].unwrap().intersects(&first));
        // Everything soft-blocked: the preferred spot is used anyway.
        let all = [Rectangle::new(Point::new(0.0, 0.0), Size::new(400.0, 300.0))];
        assert_eq!(layout(&[req], &[], &all, AREA)[0].unwrap().position(), first.position());
    }

    #[test]
    fn a_label_stays_inside_the_area_and_off_obstacles() {
        let size = Size::new(60.0, 12.0);
        let obstacle = Rectangle::new(Point::new(104.0, 80.0), Size::new(80.0, 20.0));
        let out = layout(&[Request::at_handle(Point::new(100.0, 100.0), size, 1)], &[obstacle], &[], AREA);
        let rect = out[0].unwrap();
        assert!(!rect.intersects(&obstacle) && inside(rect, AREA));
        // At the very corner of the area the label flips inward.
        let out = layout(&[Request::at_handle(Point::new(2.0, 2.0), size, 1)], &[], &[], AREA);
        assert!(inside(out[0].unwrap(), AREA));
    }
}
