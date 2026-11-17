//! Fin outlines for the 3D mesher: one chordwise interval per height above the body, tabulated (S11). The trapezoid of [`crate::engine::grid3::Fins3`]'s four
//! numbers is the special case with straight edges; an ellipse and a freeform polygon (the model's `EllipticalFin` and `FreeformFin`) are the others. Lengths are in
//! nose diameters, `x` is measured from the fore end of the root, `h` is the height above the body surface under the root.

use crate::error::Error;

/// A fin outline: rows `[h, x_le, x_te]` by increasing height from the root (`h = 0`, `x_le = 0`) to the tip (`h = span`), the edges linear between the rows.
/// The chord `x_te - x_le` is positive below the tip and may be zero at it (a pointed or elliptical tip).
#[derive(Clone, Debug, PartialEq)]
pub struct Planform {
    rows: Vec<[f64; 3]>,
}

impl Planform {
    /// A planform from its rows (`[h, x_le, x_te]`, `h` strictly increasing from 0, `x_le` of the root row 0, a positive chord everywhere but at the tip).
    pub fn new(rows: Vec<[f64; 3]>) -> Result<Planform, Error> {
        let bad = |m: String| Err(Error::InvalidInput(format!("fin planform: {m}")));
        if rows.len() < 2 {
            return bad("at least two rows (root and tip) are needed".into());
        }
        if rows.iter().flatten().any(|v| !v.is_finite()) {
            return bad("the rows must be finite".into());
        }
        if rows[0][0] != 0.0 || rows[0][1] != 0.0 {
            return bad("the first row is the root: h = 0 and x_le = 0".into());
        }
        if rows.windows(2).any(|w| w[1][0] <= w[0][0]) {
            return bad("the heights must increase strictly".into());
        }
        let last = rows.len() - 1;
        for (k, r) in rows.iter().enumerate() {
            let chord = r[2] - r[1];
            if chord < 0.0 || (chord == 0.0 && k != last) {
                return bad(format!("the chord at h = {:.4} is {chord:.4}: it must be positive below the tip (and not negative at it)", r[0]));
            }
        }
        Ok(Planform { rows })
    }

    /// The trapezoid of root chord, tip chord, span and the sweep of the tip leading edge aft of the root leading edge.
    pub fn trapezoid(root_chord: f64, tip_chord: f64, span: f64, sweep: f64) -> Result<Planform, Error> {
        Planform::new(vec![[0.0, 0.0, root_chord], [span, sweep, sweep + tip_chord]])
    }

    /// The half ellipse standing on the root: semi-axes `root_chord / 2` along the body and `span` out (the model's `EllipticalFin`), the rows at
    /// `h = span sin(phi)` so that they cluster towards the tip, where the chord falls to zero.
    pub fn ellipse(root_chord: f64, span: f64) -> Result<Planform, Error> {
        const N: usize = 48;
        let a = 0.5 * root_chord;
        let rows = (0..=N)
            .map(|k| {
                let u = (0.5 * std::f64::consts::PI * k as f64 / N as f64).sin();
                let w = if k == N { 0.0 } else { (1.0 - u * u).sqrt() };
                [span * u, a * (1.0 - w), a * (1.0 + w)]
            })
            .collect();
        Planform::new(rows)
    }

    /// The outline of a closed polygon of `(z, height)` points (the model's `FreeformFin`: `z` along the axis, `height` above the body; the first and the last point
    /// are the ends of the root, at height 0). One chordwise interval per height is required: a notched or lobed outline is refused.
    pub fn from_polygon(points: &[(f64, f64)]) -> Result<Planform, Error> {
        let bad = |m: String| Err(Error::InvalidInput(format!("fin outline: {m}")));
        if points.len() < 3 {
            return bad("a polygon needs at least three points".into());
        }
        if points.iter().any(|p| !(p.0.is_finite() && p.1.is_finite())) {
            return bad("the points must be finite".into());
        }
        let eps = 1e-12;
        if points[0].1.abs() > eps || points[points.len() - 1].1.abs() > eps {
            return bad("the first and the last point are the ends of the root and must have height 0".into());
        }
        if points.iter().any(|p| p.1 < -eps) {
            return bad("no point may lie below the root (negative height)".into());
        }
        let mut heights: Vec<f64> = points.iter().map(|p| p.1.max(0.0)).collect();
        heights.sort_by(f64::total_cmp);
        heights.dedup_by(|a, b| (*a - *b).abs() <= eps);
        if heights.len() < 2 {
            return bad("the outline has no height".into());
        }
        let n = points.len();
        // the crossings of the polygon with the line at height `h`: edge intersections, vertices on the line included once
        let crossings = |h: f64| -> Vec<f64> {
            let mut xs = Vec::new();
            for k in 0..n {
                let (p, q) = (points[k], points[(k + 1) % n]);
                let (lo, hi) = (p.1.min(q.1), p.1.max(q.1));
                if h < lo - eps || h > hi + eps {
                    continue;
                }
                if (q.1 - p.1).abs() <= eps {
                    xs.push(p.0);
                    xs.push(q.0);
                } else {
                    xs.push(p.0 + (q.0 - p.0) * (h - p.1) / (q.1 - p.1));
                }
            }
            xs
        };
        let mut rows: Vec<[f64; 3]> = Vec::new();
        for (k, &h) in heights.iter().enumerate() {
            let xs = crossings(h);
            let (le, te) = (xs.iter().cloned().fold(f64::INFINITY, f64::min), xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
            rows.push([h, le, te]);
            // one interval between this row and the next: two crossings at the middle of the band
            if let Some(&h1) = heights.get(k + 1) {
                let mid = crossings(0.5 * (h + h1));
                if mid.len() != 2 {
                    return bad(format!(
                        "{} chordwise intervals at height {:.4}: a notched or multi-lobed outline is not supported (one interval per height)",
                        mid.len() / 2,
                        0.5 * (h + h1)
                    ));
                }
            }
        }
        let x0 = rows[0][1];
        for r in &mut rows {
            r[1] -= x0;
            r[2] -= x0;
        }
        rows[0][0] = 0.0;
        Planform::new(rows)
    }

    pub fn rows(&self) -> &[[f64; 3]] {
        &self.rows
    }

    /// Height of the tip.
    pub fn span(&self) -> f64 {
        self.rows[self.rows.len() - 1][0]
    }

    /// `x` of the leading edge at height `h` (clamped to `0..=span`).
    pub fn x_le(&self, h: f64) -> f64 {
        self.at(h, 1)
    }

    /// `x` of the trailing edge at height `h`.
    pub fn x_te(&self, h: f64) -> f64 {
        self.at(h, 2)
    }

    pub fn chord(&self, h: f64) -> f64 {
        self.x_te(h) - self.x_le(h)
    }

    pub fn root_chord(&self) -> f64 {
        self.chord(0.0)
    }

    pub fn tip_chord(&self) -> f64 {
        self.chord(self.span())
    }

    /// Sweep of the tip's leading edge aft of the root's.
    pub fn sweep(&self) -> f64 {
        self.x_le(self.span())
    }

    /// Planform area, `integral of chord dh`.
    pub fn area(&self) -> f64 {
        self.rows.windows(2).map(|w| 0.5 * ((w[0][2] - w[0][1]) + (w[1][2] - w[1][1])) * (w[1][0] - w[0][0])).sum()
    }

    /// Smallest `x_le` and largest `x_te` of the outline.
    pub fn x_extent(&self) -> (f64, f64) {
        (self.rows.iter().map(|r| r[1]).fold(f64::INFINITY, f64::min), self.rows.iter().map(|r| r[2]).fold(f64::NEG_INFINITY, f64::max))
    }

    /// The height up to which the chord is at least `floor` (the whole span when the tip chord is): the grid cannot take a chord that goes to zero, so a pointed or
    /// elliptical tip is cut where the chord has fallen to `floor`.
    pub fn height_with_chord_at_least(&self, floor: f64) -> f64 {
        if self.tip_chord() >= floor {
            return self.span();
        }
        for w in self.rows.windows(2) {
            let (c0, c1) = (w[0][2] - w[0][1], w[1][2] - w[1][1]);
            if c1 < floor {
                // the chord is linear in the band
                return if c0 > floor { w[0][0] + (w[1][0] - w[0][0]) * (c0 - floor) / (c0 - c1) } else { w[0][0] };
            }
        }
        self.span()
    }

    /// Smallest `x_le` and largest `x_te` of the outline up to height `h`.
    pub fn x_extent_up_to(&self, h: f64) -> (f64, f64) {
        let rows = self.rows.iter().filter(|r| r[0] < h).map(|r| (r[1], r[2])).chain([(self.x_le(h), self.x_te(h))]);
        rows.fold((f64::INFINITY, f64::NEG_INFINITY), |a, r| (a.0.min(r.0), a.1.max(r.1)))
    }

    fn at(&self, h: f64, col: usize) -> f64 {
        let h = h.clamp(0.0, self.span());
        let k = self.rows.partition_point(|r| r[0] <= h).clamp(1, self.rows.len() - 1);
        let (a, b) = (self.rows[k - 1], self.rows[k]);
        a[col] + (b[col] - a[col]) * (h - a[0]) / (b[0] - a[0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_trapezoid_is_straight_between_root_and_tip() {
        let p = Planform::trapezoid(1.0, 0.6, 1.0, 0.3).unwrap();
        assert!((p.root_chord() - 1.0).abs() + (p.tip_chord() - 0.6).abs() + (p.span() - 1.0).abs() + (p.sweep() - 0.3).abs() < 1e-15);
        assert!((p.chord(0.5) - 0.8).abs() < 1e-15 && (p.x_le(0.5) - 0.15).abs() < 1e-15);
        assert!((p.area() - 0.8).abs() < 1e-15);
    }

    #[test]
    fn the_ellipse_has_the_area_of_a_half_ellipse() {
        let p = Planform::ellipse(0.4, 1.0).unwrap();
        // half ellipse of semi-axes 0.2 and 1: pi a b / 2
        let exact = std::f64::consts::PI * 0.2 * 1.0 / 2.0;
        assert!((p.area() / exact - 1.0).abs() < 2e-3, "{} vs {exact}", p.area());
        assert_eq!(p.tip_chord(), 0.0);
        // cut where the chord is a tenth of the root's
        let h = p.height_with_chord_at_least(0.04);
        assert!((h - (1.0 - 0.5 * 0.1f64.powi(2))).abs() < 1e-3 && h < 1.0, "{h}");
    }

    #[test]
    fn a_polygon_that_is_a_trapezoid_is_the_trapezoid() {
        let poly = Planform::from_polygon(&[(0.0, 0.0), (0.3, 1.0), (0.9, 1.0), (1.0, 0.0)]).unwrap();
        let trap = Planform::trapezoid(1.0, 0.6, 1.0, 0.3).unwrap();
        for h in [0.0, 0.1, 0.5, 0.97, 1.0] {
            assert!((poly.x_le(h) - trap.x_le(h)).abs() < 1e-14 && (poly.x_te(h) - trap.x_te(h)).abs() < 1e-14, "h = {h}");
        }
    }

    #[test]
    fn a_polygon_may_start_aft_of_zero_and_be_pointed() {
        let tri = Planform::from_polygon(&[(2.0, 0.0), (2.5, 0.8), (3.0, 0.0)]).unwrap();
        assert_eq!((tri.root_chord(), tri.tip_chord(), tri.span()), (1.0, 0.0, 0.8));
        assert!((tri.x_le(0.8) - 0.5).abs() < 1e-14);
    }

    #[test]
    fn outlines_with_two_intervals_or_off_the_root_are_refused_with_the_reason() {
        // a notch from the top: two chordwise intervals at the top
        let notched =
            Planform::from_polygon(&[(0.0, 0.0), (0.0, 1.0), (0.4, 1.0), (0.4, 0.5), (0.6, 0.5), (0.6, 1.0), (1.0, 1.0), (1.0, 0.0)]).unwrap_err().to_string();
        assert!(notched.contains("chordwise intervals"), "{notched}");
        let off = Planform::from_polygon(&[(0.0, 0.1), (0.5, 1.0), (1.0, 0.0)]).unwrap_err().to_string();
        assert!(off.contains("height 0"), "{off}");
        let below = Planform::from_polygon(&[(0.0, 0.0), (0.5, -0.1), (1.0, 0.0)]).unwrap_err().to_string();
        assert!(below.contains("negative height"), "{below}");
    }
}
