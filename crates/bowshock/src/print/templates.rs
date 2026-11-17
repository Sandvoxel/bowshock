//! Full-size templates for building the rocket: fin outlines, centering rings and bulkheads, flat patterns of cones and
//! transitions, and a strip that marks where the fins go around a tube.
//!
//! Every template is a [`Drawing`] in millimetres at 1:1. [`emit`] lays drawings out on pages (small ones packed in shelves,
//! largest first; one larger than the printable area is tiled over several pages with a 10 mm overlap, crop marks at the corners
//! of each tile and "Tile r/c of R×C"). Every page carries a 25 mm and a 1 in square under "Print at 100 % (no scaling)" so
//! the print can be checked with a ruler.
//!
//! The flat pattern of a frustum is the development of a right circular cone (any geometry text): slant length `l`, apex
//! distances `ρ1 = r1·l/(r2 − r1)` and `ρ2 = ρ1 + l`, and a sector angle `θ = 2π·r2/ρ2` (equal to `2π·r1/ρ1`).

use std::f64::consts::{PI, TAU};

use bowshock_model::{instance_letter, FinTab, InternalKind, NoseShape, Part, PartKind, RadialKind, Rocket};

use super::pdf::{Page, PageSize, Pdf};
use crate::app::ui;
use crate::units::{Quantity, UnitSystem};

/// Page margin, mm (half an inch).
pub const MARGIN: f64 = 12.7;
/// Height of the title line above the drawing area, mm.
const HEADER: f64 = 10.0;
/// Height of the calibration strip under the drawing area, mm.
const FOOTER: f64 = 31.0;
/// How far neighbouring tiles of a big drawing overlap, mm.
const OVERLAP: f64 = 10.0;
/// Gap between packed drawings, mm.
const GAP: f64 = 6.0;
/// How many gores make up a curved nose.
const GORES: usize = 6;
/// Width of the glue tab of a flat pattern, mm.
const GLUE_TAB: f64 = 10.0;
/// The largest side a pattern is drawn at, mm.
const MAX_SIDE: f64 = 1500.0;
/// Line spacing of the labels under a drawing, mm.
const LINE: f64 = 4.0;
/// Stroke width of outlines, mm.
const OUTLINE: f64 = 0.25;

/// One mark of a [`Drawing`].
#[derive(Clone, Debug, PartialEq)]
pub enum Item {
    /// A polyline or polygon.
    Path { pts: Vec<(f64, f64)>, closed: bool, width: f64, dash: Option<f64> },
    /// A circle.
    Circle { c: (f64, f64), r: f64, width: f64 },
    /// Text, baseline start, size in pt.
    Text { at: (f64, f64), size: f64, bold: bool, s: String },
}

/// Something to print at full size, in millimetres from its top-left corner.
#[derive(Clone, Debug, PartialEq)]
pub struct Drawing {
    /// Width, mm.
    pub w: f64,
    /// Height, mm.
    pub h: f64,
    /// The marks.
    pub items: Vec<Item>,
}

impl Drawing {
    fn new() -> Drawing {
        Drawing { w: 0.0, h: 0.0, items: Vec::new() }
    }

    fn path(&mut self, pts: Vec<(f64, f64)>, closed: bool, width: f64, dash: Option<f64>) {
        self.items.push(Item::Path { pts, closed, width, dash });
    }

    fn text(&mut self, at: (f64, f64), size: f64, bold: bool, s: impl Into<String>) {
        self.items.push(Item::Text { at, size, bold, s: s.into() });
    }

    /// Makes the drawing at least as wide as its widest text and as the shapes.
    fn fit_width(&mut self, shape_w: f64) {
        let text_w = self
            .items
            .iter()
            .filter_map(|i| match i {
                Item::Text { at, size, bold, s } => Some(at.0 + Page::text_width(*size, *bold, s)),
                _ => None,
            })
            .fold(0.0, f64::max);
        self.w = shape_w.max(text_w);
    }

    /// Draws the part of the drawing from `shift` on at `origin` on `page` (the caller clips).
    fn draw(&self, page: &mut Page, origin: (f64, f64), shift: (f64, f64)) {
        let at = |p: (f64, f64)| (origin.0 + p.0 - shift.0, origin.1 + p.1 - shift.1);
        for item in &self.items {
            match item {
                Item::Path { pts, closed, width, dash } => {
                    let pts: Vec<(f64, f64)> = pts.iter().map(|&p| at(p)).collect();
                    page.path(&pts, *closed, *width, *dash);
                }
                Item::Circle { c, r, width } => page.circle(at(*c), *r, *width),
                Item::Text { at: p, size, bold, s } => page.text(at(*p), *size, *bold, s),
            }
        }
    }

    /// The bounding box `(min x, min y, max x, max y)` of every path.
    #[cfg(test)]
    pub fn path_bbox(&self) -> Option<(f64, f64, f64, f64)> {
        let mut b: Option<(f64, f64, f64, f64)> = None;
        for p in self.items.iter().filter_map(|i| match i {
            Item::Path { pts, .. } => Some(pts),
            _ => None,
        }) {
            for &(x, y) in p {
                b = Some(b.map_or((x, y, x, y), |(a, c, d, e)| (a.min(x), c.min(y), d.max(x), e.max(y))));
            }
        }
        b
    }
}

fn len(v: f64, units: UnitSystem) -> String {
    ui::with_unit(v, Quantity::Length, units)
}

/// A drawing of the label lines `lines` at 8 pt, the first bold.
fn label_lines(d: &mut Drawing, top: f64, lines: &[String]) -> f64 {
    for (i, line) in lines.iter().enumerate() {
        d.text((0.0, top + LINE * (i as f64 + 1.0) - 1.0), 7.5, i == 0, line.clone());
    }
    top + LINE * lines.len() as f64 + 1.0
}

/// The fin templates: one per fin group, and one more for every copy whose planform differs from the group's.
pub fn fin_drawings(rocket: &Rocket, units: UnitSystem) -> Vec<Drawing> {
    let mut out = Vec::new();
    for radial in rocket.radials().iter().filter(|r| r.kind.is_fin()) {
        // Copies sharing a planform share a template.
        let mut kinds: Vec<(RadialKind, Vec<usize>)> = Vec::new();
        for i in 0..usize::from(radial.count.max(1)) {
            let kind = radial.effective(i).kind;
            match kinds.iter_mut().find(|(k, _)| *k == kind) {
                Some((_, copies)) => copies.push(i),
                None => kinds.push((kind, vec![i])),
            }
        }
        let several = kinds.len() > 1;
        for (kind, copies) in kinds {
            let name = if several {
                let letters: Vec<String> = copies.iter().map(|&i| instance_letter(radial.copy_index(i))).collect();
                format!("{} ({})", radial.display_name(), letters.join(", "))
            } else {
                radial.display_name()
            };
            if let Some(d) = fin_drawing(&name, &kind, copies.len(), radial.material.name(), radial.tab, units) {
                out.push(d);
            }
        }
    }
    out
}

/// One fin's outline at 1:1 with the root along the bottom, the through-the-wall tab below it and the labels under that.
fn fin_drawing(name: &str, kind: &RadialKind, count: usize, material: &str, tab: Option<FinTab>, units: UnitSystem) -> Option<Drawing> {
    let outline = kind.planform()?;
    let span = outline.iter().map(|p| p.1).fold(0.0, f64::max) * 1000.0;
    let right = outline.iter().map(|p| p.0).fold(0.0, f64::max) * 1000.0;
    let root = kind.extent() * 1000.0;
    let mut d = Drawing::new();
    d.path(outline.iter().map(|&(z, h)| (z * 1000.0, span - h * 1000.0)).collect(), true, OUTLINE, None);
    let mut shape_w = right.max(root);
    let mut below = span;
    if let Some(t) = tab {
        let (x0, x1, depth) = (t.position * 1000.0, (t.position + t.length) * 1000.0, t.height * 1000.0);
        d.path(vec![(x0, span), (x1, span), (x1, span + depth), (x0, span + depth)], true, OUTLINE, None);
        shape_w = shape_w.max(x1);
        below = span + depth;
    }
    d.path(vec![(0.0, span), (root, span)], false, 0.2, Some(1.5));
    d.text((root + 1.5, span + 0.8), 6.0, false, "Body surface");
    let mut lines = vec![name.to_string()];
    let thickness = kind.thickness().map_or(String::new(), |t| format!("Thickness {}, ", len(t, units)));
    lines.push(format!("{thickness}{material}"));
    lines.push(match kind {
        RadialKind::FinSet { root_chord, tip_chord, span, sweep, .. } => {
            format!("Root {} · Tip {} · Span {} · Sweep {}", len(*root_chord, units), len(*tip_chord, units), len(*span, units), len(*sweep, units))
        }
        _ => format!("Root {} · Span {}", len(kind.extent(), units), len(kind.fin_span().unwrap_or(0.0), units)),
    });
    if tab.is_some() {
        lines.push("Through-the-wall tab below the root".to_string());
    }
    lines.push(format!("Cut {count}"));
    d.h = label_lines(&mut d, below + 3.0, &lines);
    d.fit_width(shape_w);
    Some(d)
}

/// The centering rings and bulkheads, largest first: the outer and inner circle, a centre cross and the dimensions.
pub fn ring_drawings(rocket: &Rocket, units: UnitSystem) -> Vec<Drawing> {
    let mut out = Vec::new();
    for part in rocket.internals() {
        if !matches!(part.kind, InternalKind::CenteringRing { .. } | InternalKind::Bulkhead { .. }) {
            continue;
        }
        let Some(res) = rocket.resolved(part.id) else { continue };
        let (od, id) = (res.outer_diameter * 1000.0, res.inner_diameter * 1000.0);
        if od <= 0.0 {
            continue;
        }
        let mut lines = vec![part.display_name()];
        lines.push(if id > 0.0 {
            format!("OD {} · ID {} · Thickness {}", len(res.outer_diameter, units), len(res.inner_diameter, units), len(res.length, units))
        } else {
            format!("Ø {} · Thickness {}", len(res.outer_diameter, units), len(res.length, units))
        });
        lines.push(part.material.name().to_string());
        let mut d = Drawing::new();
        // The widest label decides the width; the circles are centred in it.
        let text_w = lines.iter().map(|l| Page::text_width(7.5, false, l)).fold(0.0, f64::max);
        let w = od.max(text_w);
        let c = (w / 2.0, od / 2.0);
        d.items.push(Item::Circle { c, r: od / 2.0, width: OUTLINE });
        if id > 0.0 {
            d.items.push(Item::Circle { c, r: id / 2.0, width: OUTLINE });
        }
        let arm = (od / 4.0).min(4.0);
        d.path(vec![(c.0 - arm, c.1), (c.0 + arm, c.1)], false, 0.15, None);
        d.path(vec![(c.0, c.1 - arm), (c.0, c.1 + arm)], false, 0.15, None);
        let mut label = Drawing::new();
        d.h = label_lines(&mut label, od + 3.0, &lines);
        // Centre the labels under the circles.
        for item in label.items {
            if let Item::Text { at, size, bold, s } = item {
                let x = (w - Page::text_width(size, bold, &s)) / 2.0;
                d.text((x.max(0.0) + at.0, at.1), size, bold, s);
            }
        }
        d.w = w;
        out.push(d);
    }
    out.sort_by(|a, b| b.w.total_cmp(&a.w));
    out
}

/// The development of a frustum: `(ρ1, ρ2, θ)` of the apex distances of the small and large rims and the sector angle (rad).
pub fn frustum(r1: f64, r2: f64, length: f64) -> (f64, f64, f64) {
    let slant = length.hypot(r2 - r1);
    let rho1 = r1 * slant / (r2 - r1);
    let rho2 = rho1 + slant;
    (rho1, rho2, TAU * r2 / rho2)
}

/// The flat pattern of a frustum of small radius `r1`, large radius `r2` and axial `length` (metres), with a glue tab.
fn wrap_drawing(name: &str, r1: f64, r2: f64, length: f64, material: &str, thickness: f64, units: UnitSystem) -> Result<Drawing, String> {
    let (rho1, rho2, theta) = frustum(r1, r2, length);
    let (rho1, rho2) = (rho1 * 1000.0, rho2 * 1000.0);
    if rho2 * 2.0 > MAX_SIDE * 4.0 {
        return Err(format!("{name}: the pattern is larger than {MAX_SIDE} mm"));
    }
    // The sector opens downwards from the apex, symmetric about the vertical.
    let (a0, a1) = (PI / 2.0 - theta / 2.0, PI / 2.0 + theta / 2.0);
    let at = |rho: f64, a: f64| (rho * a.cos(), rho * a.sin());
    let steps = ((theta.to_degrees() / 2.0).ceil() as usize).max(4);
    let arc = |rho: f64, from: f64, to: f64| (0..=steps).map(|i| at(rho, from + (to - from) * i as f64 / steps as f64)).collect::<Vec<_>>();
    let mut sector = arc(rho2, a0, a1);
    if rho1 > 1e-9 {
        sector.extend(arc(rho1, a1, a0));
    } else {
        sector.push((0.0, 0.0));
    }
    // The glue tab on the edge at `a1`, tapered a little at its far end.
    let (a, b) = (at(rho1, a1), at(rho2, a1));
    let tangent = (-a1.sin(), a1.cos());
    let along = (b.0 - a.0, b.1 - a.1);
    let taper = 0.06;
    let c = (b.0 + tangent.0 * GLUE_TAB - along.0 * taper, b.1 + tangent.1 * GLUE_TAB - along.1 * taper);
    let e = (a.0 + tangent.0 * GLUE_TAB + along.0 * taper, a.1 + tangent.1 * GLUE_TAB + along.1 * taper);
    let tab = vec![a, b, c, e];
    let all: Vec<(f64, f64)> = sector.iter().chain(&tab).copied().collect();
    let (min_x, min_y) = all.iter().fold((f64::MAX, f64::MAX), |m, p| (m.0.min(p.0), m.1.min(p.1)));
    let (max_x, max_y) = all.iter().fold((f64::MIN, f64::MIN), |m, p| (m.0.max(p.0), m.1.max(p.1)));
    if max_x - min_x > MAX_SIDE || max_y - min_y > MAX_SIDE {
        return Err(format!("{name}: the pattern is larger than {MAX_SIDE} mm"));
    }
    let shift = |p: &(f64, f64)| (p.0 - min_x, p.1 - min_y);
    let mut d = Drawing::new();
    d.path(sector.iter().map(shift).collect(), true, OUTLINE, None);
    d.path(tab.iter().map(shift).collect(), true, OUTLINE, None);
    d.path(vec![shift(&a), shift(&b)], false, 0.2, Some(1.5));
    let lines = vec![
        name.to_string(),
        format!("Ø {} → Ø {} · Length {} · Slant {}", len(r1 * 2.0, units), len(r2 * 2.0, units), len(length, units), len(length.hypot(r2 - r1), units)),
        format!("Arc radii {} and {} · Sector {:.1}°", len(rho1 / 1000.0, units), len(rho2 / 1000.0, units), theta.to_degrees()),
        format!("Glue tab {} (dashed edge is the seam) · {material}, {}", len(GLUE_TAB / 1000.0, units), len(thickness, units)),
    ];
    d.h = label_lines(&mut d, max_y - min_y + 3.0, &lines);
    d.fit_width(max_x - min_x);
    Ok(d)
}

/// The gore of the nose cone `part`: one of `GORES` identical petals that, edge to edge, make up the nose.
///
/// A nose that curves along its length has no exact flat pattern. At each point of the outline, `s` metres along it, the gore is
/// `2π·r(s)/GORES` wide: the circumference there divided between the gores, which is how a globe's gores are cut. The tip is at the left.
fn gore_drawing(name: &str, part: &Part, units: UnitSystem) -> Result<Drawing, String> {
    let PartKind::NoseCone { shape, length, diameter } = part.kind else { return Err(format!("{name}: not a nose cone")) };
    let (material, thickness) = (part.material.name(), part.thickness);
    // The outline in 96 steps, finer than the model's 32, so that the cut edge is smooth.
    let profile: Vec<(f64, f64)> = (0..=96).map(|i| length * f64::from(i) / 96.0).map(|z| (z, part.kind.radius_at(z))).collect();
    let mut s_mm = 0.0;
    let mut rows: Vec<(f64, f64)> = Vec::with_capacity(profile.len());
    let mut last = profile.first().copied().unwrap_or((0.0, 0.0));
    for &(z, r) in &profile {
        s_mm += (z - last.0).hypot(r - last.1) * 1000.0;
        last = (z, r);
        rows.push((s_mm, r * 1000.0 * PI / GORES as f64));
    }
    let half = rows.iter().map(|r| r.1).fold(0.0, f64::max);
    if s_mm > MAX_SIDE || half * 2.0 > MAX_SIDE {
        return Err(format!("{name}: the pattern is larger than {MAX_SIDE} mm"));
    }
    // Lying down, tip to the left: the gore is longer than it is wide, and a long drawing fits a sideways page.
    let mut outline: Vec<(f64, f64)> = rows.iter().map(|&(s, w)| (s, half - w)).collect();
    outline.extend(rows.iter().rev().map(|&(s, w)| (s, half + w)));
    let mut d = Drawing::new();
    d.path(outline, true, OUTLINE, None);
    d.path(vec![(0.0, half), (s_mm, half)], false, 0.2, Some(1.5));
    let lines = [
        name.to_string(),
        format!("{} nose · Ø {} · Length {} · Along the surface {}", shape.name(), len(diameter, units), len(length, units), len(s_mm / 1000.0, units)),
        format!("Cut {GORES} gores, each {} wide at the base (dashed line is the centre), and join them edge to edge", len(half * 2.0 / 1000.0, units)),
        format!("{material}, {} · An approximate pattern: a curved nose cannot be flattened exactly", len(thickness, units)),
    ];
    d.h = label_lines(&mut d, half * 2.0 + 3.0, &lines);
    d.fit_width(s_mm);
    Ok(d)
}

/// The flat patterns of the transitions and conical noses, the gores of the other noses, and a note for each part whose
/// pattern is too large.
pub fn wrap_drawings(rocket: &Rocket, units: UnitSystem) -> (Vec<Drawing>, Vec<String>) {
    let mut drawings = Vec::new();
    let mut notes = Vec::new();
    for part in rocket.parts() {
        let name = part.display_name();
        let pattern = match part.kind {
            PartKind::Transition { length, fore_diameter, aft_diameter } => {
                let (r1, r2) = (fore_diameter.min(aft_diameter) / 2.0, fore_diameter.max(aft_diameter) / 2.0);
                if r2 - r1 < 1e-6 {
                    continue;
                }
                wrap_drawing(&name, r1, r2, length, part.material.name(), part.thickness, units)
            }
            PartKind::NoseCone { shape: NoseShape::Conical, length, diameter } => {
                wrap_drawing(&name, 0.0, diameter / 2.0, length, part.material.name(), part.thickness, units)
            }
            PartKind::NoseCone { .. } => gore_drawing(&name, part, units),
            PartKind::BodyTube { .. } => continue,
        };
        match pattern {
            Ok(d) => drawings.push(d),
            Err(e) => notes.push(e),
        }
    }
    drawings.sort_by(|a, b| (b.w * b.h).total_cmp(&(a.w * a.h)));
    (drawings, notes)
}

/// A strip for each body tube that carries radial parts: lines at each copy's angle round the tube, from the left edge (0°, the
/// +Y side) to the seam.
pub fn marking_drawings(rocket: &Rocket, units: UnitSystem) -> Vec<Drawing> {
    const LABEL_ROWS: usize = 3;
    const ROW: f64 = 3.6;
    const STRIP_H: f64 = 40.0;
    let mut out = Vec::new();
    for part in rocket.parts() {
        let PartKind::BodyTube { diameter, .. } = part.kind else { continue };
        let carried: Vec<_> = rocket.radials().iter().filter(|r| r.parent == part.id && !r.kind.is_pod()).collect();
        if carried.is_empty() {
            continue;
        }
        let circumference = PI * diameter * 1000.0;
        let top = LABEL_ROWS as f64 * ROW + 2.0;
        let mut marks: Vec<(f64, String)> = Vec::new();
        for r in carried {
            for (i, angle) in r.angles().into_iter().enumerate() {
                marks.push((angle / TAU * circumference, format!("{} {} {:.0}°", r.display_name(), instance_letter(r.copy_index(i)), angle.to_degrees())));
            }
        }
        marks.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut d = Drawing::new();
        d.path(vec![(0.0, top), (circumference, top), (circumference, top + STRIP_H), (0.0, top + STRIP_H)], true, OUTLINE, None);
        for deg in (0..360).step_by(30) {
            let x = f64::from(deg) / 360.0 * circumference;
            d.path(vec![(x, top + STRIP_H), (x, top + STRIP_H - 2.0)], false, 0.15, None);
            if deg % 90 == 0 {
                d.text((x + 0.6, top + STRIP_H - 3.0), 5.5, false, format!("{deg}°"));
            }
        }
        for (k, (x, label)) in marks.iter().enumerate() {
            let row = k % LABEL_ROWS;
            let y = row as f64 * ROW;
            d.path(vec![(*x, y + 0.5), (*x, top + STRIP_H)], false, 0.3, None);
            d.text((x + 0.8, y + ROW - 0.8), 6.0, false, label.clone());
        }
        let lines = vec![
            format!("Fin marking guide · {}", part.display_name()),
            format!(
                "Wrap round the tube (Ø {}, circumference {}) with the left edge on the +Y side; the ends meet at the seam.",
                len(diameter, units),
                len(circumference / 1000.0, units)
            ),
        ];
        d.h = label_lines(&mut d, top + STRIP_H + 2.0, &lines);
        d.fit_width(circumference);
        out.push(d);
    }
    out
}

/// A list of notes as a drawing of text lines, `None` if there are none.
pub fn notes_drawing(notes: &[String]) -> Option<Drawing> {
    if notes.is_empty() {
        return None;
    }
    let mut d = Drawing::new();
    for (i, n) in notes.iter().enumerate() {
        d.text((0.0, LINE * (i as f64 + 1.0) - 1.0), 8.0, false, n.clone());
    }
    d.h = LINE * notes.len() as f64 + 1.0;
    d.fit_width(0.0);
    Some(d)
}

/// The printable area of a page, upright or turned on its side: `(x, y, width, height)` mm.
fn area(size: PageSize, landscape: bool) -> (f64, f64, f64, f64) {
    let (w, h) = if landscape { (size.mm().1, size.mm().0) } else { size.mm() };
    (MARGIN, MARGIN + HEADER, w - 2.0 * MARGIN, h - 2.0 * MARGIN - HEADER - FOOTER)
}

/// The title, the calibration squares and the note on `page`.
fn frame(page: &mut Page, title: &str, note: Option<&str>) {
    let h = page.size_mm().1;
    page.text((MARGIN, MARGIN + 3.5), 10.0, true, title);
    let top = h - MARGIN - 26.0;
    page.path(&[(MARGIN, top), (MARGIN + 25.0, top), (MARGIN + 25.0, top + 25.0), (MARGIN, top + 25.0)], true, 0.2, None);
    page.text((MARGIN + 2.0, top + 13.5), 7.0, false, "25 mm");
    let x = MARGIN + 31.0;
    page.path(&[(x, top), (x + 25.4, top), (x + 25.4, top + 25.4), (x, top + 25.4)], true, 0.2, None);
    page.text((x + 2.0, top + 13.5), 7.0, false, "1 in");
    let tx = x + 25.4 + 6.0;
    page.text((tx, top + 6.0), 9.0, true, "Print at 100 % (no scaling)");
    if let Some(note) = note {
        page.text((tx, top + 12.0), 8.0, false, note);
    }
}

/// Short corner marks around the rectangle `origin`, `size`.
fn crop_marks(page: &mut Page, origin: (f64, f64), size: (f64, f64)) {
    let (x0, y0, x1, y1) = (origin.0, origin.1, origin.0 + size.0, origin.1 + size.1);
    for (x, sx) in [(x0, -1.0), (x1, 1.0)] {
        for (y, sy) in [(y0, -1.0), (y1, 1.0)] {
            page.line((x + sx * 1.5, y), (x + sx * 4.5, y), 0.15);
            page.line((x, y + sy * 1.5), (x, y + sy * 4.5), 0.15);
        }
    }
}

/// How many tiles `(columns, rows)` the drawing `d` takes on a printable area `aw` × `ah`, tiles overlapping by [`OVERLAP`].
fn tiles(d: &Drawing, aw: f64, ah: f64) -> (usize, usize) {
    if d.w <= aw && d.h <= ah {
        return (1, 1);
    }
    let cols = (((d.w - OVERLAP) / (aw - OVERLAP)).ceil() as usize).max(1);
    let rows = (((d.h - OVERLAP) / (ah - OVERLAP)).ceil() as usize).max(1);
    (cols, rows)
}

/// Pages for `drawings`, each page titled `title`; returns how many pages were added.
///
/// Drawings that fit an upright page share pages. A drawing that does not is put on a page of its own, turned on its side when
/// that fits it on one page (or needs fewer tiles), and otherwise tiled at 1:1 with the overlap, crop marks and tile numbers.
pub fn emit(pdf: &mut Pdf, size: PageSize, title: &str, drawings: &[Drawing]) -> usize {
    let before = pdf.page_count();
    let (x0, y0, aw, ah) = area(size, false);
    let (small, big): (Vec<&Drawing>, Vec<&Drawing>) = drawings.iter().partition(|d| d.w <= aw && d.h <= ah);
    // Shelf packing, tallest first.
    let mut small = small;
    small.sort_by(|a, b| b.h.total_cmp(&a.h));
    let mut placed: Vec<Vec<(&Drawing, (f64, f64))>> = Vec::new();
    let (mut x, mut y, mut shelf) = (0.0, 0.0, 0.0);
    for d in small {
        if placed.is_empty() {
            placed.push(Vec::new());
        }
        if x > 0.0 && x + d.w > aw {
            x = 0.0;
            y += shelf + GAP;
            shelf = 0.0;
        }
        if y + d.h > ah {
            placed.push(Vec::new());
            (x, y, shelf) = (0.0, 0.0, 0.0);
        }
        placed.last_mut().expect("a page exists").push((d, (x, y)));
        x += d.w + GAP;
        shelf = f64::max(shelf, d.h);
    }
    for sheet in &placed {
        let page = pdf.page(size);
        frame(page, title, None);
        for (d, (px, py)) in sheet {
            d.draw(page, (x0 + px, y0 + py), (0.0, 0.0));
        }
    }
    for d in big {
        let upright = tiles(d, aw, ah);
        let (lx0, ly0, law, lah) = area(size, true);
        let sideways = tiles(d, law, lah);
        let landscape = sideways.0 * sideways.1 < upright.0 * upright.1;
        let ((x0, y0, aw, ah), (cols, rows)) = if landscape { ((lx0, ly0, law, lah), sideways) } else { ((x0, y0, aw, ah), upright) };
        let step = (aw - OVERLAP, ah - OVERLAP);
        for r in 0..rows {
            for c in 0..cols {
                let shift = (c as f64 * step.0, r as f64 * step.1);
                let clip = ((d.w - shift.0).min(aw), (d.h - shift.1).min(ah));
                let page = if landscape { pdf.landscape_page(size) } else { pdf.page(size) };
                let note = match (rows * cols, landscape) {
                    (1, true) => Some("Landscape page".to_string()),
                    (1, false) => None,
                    (n, _) => Some(format!(
                        "Tile {}/{n} (row {} of {rows}, column {} of {cols}), 10 mm overlap{}",
                        r * cols + c + 1,
                        r + 1,
                        c + 1,
                        if landscape { ", landscape" } else { "" }
                    )),
                };
                frame(page, title, note.as_deref());
                crop_marks(page, (x0, y0), clip);
                page.clip_rect((x0, y0), clip);
                d.draw(page, (x0, y0), shift);
                page.restore();
            }
        }
    }
    pdf.page_count() - before
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, Rocket};

    fn rocket_with_fin(root: f64, tip: f64, span: f64, sweep: f64) -> Rocket {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.076 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: 0.076 }).unwrap();
        let _ = r.attach_radial(tube, 0.45, 0.0, 3, RadialKind::FinSet { root_chord: root, tip_chord: tip, span, sweep, thickness: 0.004 });
        r
    }

    #[test]
    fn the_fin_template_is_one_to_one() {
        let rocket = rocket_with_fin(0.120, 0.050, 0.080, 0.060);
        let drawings = fin_drawings(&rocket, UnitSystem::MM_G_S);
        assert_eq!(drawings.len(), 1);
        let (x0, y0, x1, y1) = drawings[0].path_bbox().unwrap();
        assert!((x1 - x0 - 120.0).abs() < 0.1, "width {}", x1 - x0);
        assert!((y1 - y0 - 80.0).abs() < 0.1, "height {}", y1 - y0);
        let texts: Vec<&str> = drawings[0].items.iter().filter_map(|i| if let Item::Text { s, .. } = i { Some(s.as_str()) } else { None }).collect();
        assert!(texts.contains(&"Cut 3"), "{texts:?}");
    }

    #[test]
    fn a_copy_with_its_own_planform_gets_its_own_template() {
        let mut rocket = rocket_with_fin(0.120, 0.050, 0.080, 0.060);
        let id = rocket.radials()[0].id;
        assert!(rocket.unlink_field(id, 1, bowshock_model::RadialField::Span));
        assert!(rocket.set_instance_dim(id, 1, bowshock_model::Dim::Span, 0.1).is_some());
        assert_eq!(fin_drawings(&rocket, UnitSystem::MM_G_S).len(), 2);
    }

    #[test]
    fn the_frustum_development_closes_on_its_rims() {
        let (r1, r2, l) = (0.027, 0.038, 0.076);
        let (rho1, rho2, theta) = frustum(r1, r2, l);
        assert!((theta * rho1 - TAU * r1).abs() < 1e-12);
        assert!((theta * rho2 - TAU * r2).abs() < 1e-12);
        assert!((rho2 - rho1 - l.hypot(r2 - r1)).abs() < 1e-12);
    }

    #[test]
    fn wraps_cover_noses_and_transitions() {
        let mut rocket = Rocket::default();
        rocket.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.076 }).unwrap();
        rocket.insert(1, PartKind::Transition { length: 0.076, fore_diameter: 0.076, aft_diameter: 0.054 }).unwrap();
        rocket.insert(2, PartKind::BodyTube { length: 0.3, diameter: 0.054 }).unwrap();
        let (drawings, notes) = wrap_drawings(&rocket, UnitSystem::MM_G_S);
        // The ogive (as gores) and the transition (as a frustum).
        assert_eq!(drawings.len(), 2);
        assert!(notes.is_empty(), "{notes:?}");
    }

    #[test]
    fn a_big_drawing_is_tiled_with_overlap() {
        let mut d = Drawing::new();
        d.w = 400.0;
        d.h = 400.0;
        d.path(vec![(0.0, 0.0), (400.0, 400.0)], false, 0.3, None);
        let mut pdf = Pdf::new("t");
        let pages = emit(&mut pdf, PageSize::Letter, "Tiles", &[d]);
        // 190.5 mm printable width and 213 mm height, 10 mm overlap: 3 columns by 2 rows; sideways it is 2 by 3, no better.
        assert_eq!(pages, 6);
        let bytes = pdf.to_bytes();
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("Tile 1/6 \\(row 1 of 2, column 1 of 3\\)") && text.contains("Print at 100 % \\(no scaling\\)"));
    }

    #[test]
    fn small_drawings_share_pages_and_the_marking_guide_lists_every_copy() {
        let rocket = rocket_with_fin(0.120, 0.050, 0.080, 0.060);
        let strips = marking_drawings(&rocket, UnitSystem::MM_G_S);
        assert_eq!(strips.len(), 1);
        let labels = strips[0].items.iter().filter(|i| matches!(i, Item::Text { s, .. } if s.starts_with("Fin set "))).count();
        assert_eq!(labels, 3);
        let mut pdf = Pdf::new("t");
        let fins = fin_drawings(&rocket, UnitSystem::MM_G_S);
        assert_eq!(emit(&mut pdf, PageSize::A4, "Fins", &[fins[0].clone(), fins[0].clone()]), 1);
    }

    fn pages_in(pdf: &Pdf, size: PageSize, landscape: bool) -> usize {
        let (w, h) = size.points();
        let (w, h) = if landscape { (h, w) } else { (w, h) };
        let bytes = pdf.to_bytes();
        String::from_utf8_lossy(&bytes).matches(&format!("/MediaBox [0 0 {w:.2} {h:.2}]")).count()
    }

    /// Whether a drawing of `d` fits the printable area of an upright or a sideways page.
    fn fits_a_page(d: &Drawing, size: PageSize) -> bool {
        [false, true].into_iter().any(|landscape| {
            let (_, _, aw, ah) = area(size, landscape);
            d.w <= aw && d.h <= ah
        })
    }

    #[test]
    fn a_curved_nose_gets_gores_that_close_round_it() {
        let rocket = rocket_with_fin(0.120, 0.050, 0.080, 0.060);
        let (drawings, notes) = wrap_drawings(&rocket, UnitSystem::MM_G_S);
        assert!(notes.is_empty(), "{notes:?}");
        assert_eq!(drawings.len(), 1, "the ogive has a pattern");
        let (x0, y0, x1, y1) = drawings[0].path_bbox().unwrap();
        // 76 mm diameter: the gore is 2π·38/6 ≈ 39.8 mm across at the base and as long as the outline (a little over 200 mm).
        assert!((y1 - y0 - TAU * 38.0 / GORES as f64).abs() < 0.5, "width {}", y1 - y0);
        assert!(x1 - x0 > 200.0 && x1 - x0 < 215.0, "length {}", x1 - x0);
        let texts: Vec<&str> = drawings[0].items.iter().filter_map(|i| if let Item::Text { s, .. } = i { Some(s.as_str()) } else { None }).collect();
        assert!(texts.iter().any(|t| t.starts_with("Cut 6 gores")), "{texts:?}");
        let mut pdf = Pdf::new("t");
        assert_eq!(emit(&mut pdf, PageSize::Letter, "Wraps", &drawings), 1);
    }

    #[test]
    fn a_conical_nose_is_a_sector_and_a_rocket_without_cones_has_no_wraps() {
        let mut rocket = Rocket::default();
        rocket.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.15, diameter: 0.054 }).unwrap();
        rocket.insert(1, PartKind::BodyTube { length: 0.3, diameter: 0.054 }).unwrap();
        let (drawings, _) = wrap_drawings(&rocket, UnitSystem::MM_G_S);
        assert_eq!(drawings.len(), 1);
        let (x0, _, x1, _) = drawings[0].path_bbox().unwrap();
        assert!(x1 - x0 > 54.0 / 2.0, "a sector ~ the cone's slant wide");
        let mut tubes = Rocket::default();
        tubes.insert(0, PartKind::BodyTube { length: 0.3, diameter: 0.054 }).unwrap();
        let (drawings, notes) = wrap_drawings(&tubes, UnitSystem::MM_G_S);
        assert!(drawings.is_empty() && notes.is_empty());
        assert_eq!(emit(&mut Pdf::new("t"), PageSize::A4, "Wraps", &drawings), 0);
    }

    #[test]
    fn the_marking_guide_of_a_76_mm_tube_fits_one_landscape_page() {
        let rocket = rocket_with_fin(0.120, 0.050, 0.080, 0.060);
        let strips = marking_drawings(&rocket, UnitSystem::MM_G_S);
        assert_eq!(strips.len(), 1);
        // 239 mm round: wider than an upright page's printable width, not than a sideways page's.
        for size in [PageSize::Letter, PageSize::A4] {
            assert!(strips[0].w > area(size, false).2 && fits_a_page(&strips[0], size), "{size:?}: {} x {}", strips[0].w, strips[0].h);
            let mut pdf = Pdf::new("t");
            assert_eq!(emit(&mut pdf, size, "Guide", &strips), 1);
            assert_eq!(pages_in(&pdf, size, true), 1);
            assert_eq!(pages_in(&pdf, size, false), 0);
        }
    }

    #[test]
    fn a_wide_tube_tiles_the_guide_with_overlap_and_covers_all_of_it() {
        // 200 mm tube: 628 mm round.
        let mut rocket = Rocket::default();
        rocket.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.3, diameter: 0.2 }).unwrap();
        let tube = rocket.insert(1, PartKind::BodyTube { length: 0.9, diameter: 0.2 }).unwrap();
        let _ = rocket.attach_radial(tube, 0.7, 0.0, 4, RadialKind::FinSet { root_chord: 0.3, tip_chord: 0.1, span: 0.15, sweep: 0.1, thickness: 0.004 });
        let strips = marking_drawings(&rocket, UnitSystem::MM_G_S);
        assert_eq!(strips.len(), 1);
        let d = &strips[0];
        assert!(!fits_a_page(d, PageSize::A4));
        let (_, _, aw, ah) = area(PageSize::A4, true);
        let (cols, rows) = tiles(d, aw, ah);
        assert_eq!(rows, 1);
        assert!(cols as f64 * (aw - OVERLAP) + OVERLAP >= d.w, "{cols} tiles of {aw} mm cover {} mm", d.w);
        assert!((cols - 1) as f64 * (aw - OVERLAP) + OVERLAP < d.w, "no tile is empty");
        let mut pdf = Pdf::new("t");
        assert_eq!(emit(&mut pdf, PageSize::A4, "Guide", &strips), cols);
        assert_eq!(pages_in(&pdf, PageSize::A4, true), cols, "tiled on sideways pages, which need fewer");
    }

    #[test]
    fn a_long_fin_goes_sideways_when_that_saves_pages() {
        // 230 mm root: wider than the upright page's 190 mm, within the sideways page's 254 mm.
        let rocket = rocket_with_fin(0.230, 0.100, 0.060, 0.120);
        let fins = fin_drawings(&rocket, UnitSystem::MM_G_S);
        assert!(fits_a_page(&fins[0], PageSize::Letter) && fins[0].w > area(PageSize::Letter, false).2);
        let mut pdf = Pdf::new("t");
        assert_eq!(emit(&mut pdf, PageSize::Letter, "Fins", &fins), 1);
        assert_eq!(pages_in(&pdf, PageSize::Letter, true), 1);
    }

    #[test]
    fn the_guide_and_fin_drawings_stay_inside_their_own_bounds() {
        // Everything a drawing draws lies inside its `w` × `h`, so a page that holds the drawing holds all of it.
        for rocket in [rocket_with_fin(0.120, 0.050, 0.080, 0.060), rocket_with_fin(0.400, 0.200, 0.050, 0.300)] {
            for d in marking_drawings(&rocket, UnitSystem::MM_G_S).iter().chain(fin_drawings(&rocket, UnitSystem::MM_G_S).iter()) {
                let (x0, y0, x1, y1) = d.path_bbox().unwrap();
                assert!(x0 >= -1e-9 && y0 >= -1e-9 && x1 <= d.w + 1e-9 && y1 <= d.h + 1e-9, "{x0} {y0} {x1} {y1} in {} x {}", d.w, d.h);
                for item in &d.items {
                    if let Item::Text { at, size, bold, s } = item {
                        assert!(at.0 + Page::text_width(*size, *bold, s) <= d.w + 1e-9, "{s}");
                    }
                }
            }
        }
    }
}
