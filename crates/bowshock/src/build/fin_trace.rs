//! Tracing a fin outline from a picture: a photo or drawing of a fin (dark shape on a light page, or the reverse) becomes the
//! vertex list of a [`RadialKind::FreeformFin`](bowshock_model::RadialKind::FreeformFin).
//!
//! The steps are the textbook ones: the grey image is split into shape and background with Otsu's threshold (Otsu 1979); the
//! largest 8-connected shape is kept and its outer contour followed with Moore-neighbour tracing (stopped by Jacob's criterion);
//! the contour is reduced to a few vertices with the Ramer–Douglas–Peucker algorithm (Ramer 1972; Douglas & Peucker 1973). The
//! fin's root is its flat bottom edge: the picture is scaled so that edge is the requested root chord.

/// Smallest picture side, pixels.
const MIN_SIDE: u32 = 50;
/// The most vertices the traced outline keeps.
const MAX_POINTS: usize = 32;
/// Simplification tolerance as a fraction of the shape's bounding-box diagonal.
const TOLERANCE: f64 = 0.005;
/// How close to the lowest row a vertex must be to lie on the root, as a fraction of the shape's height.
const ROOT_BAND: f64 = 0.02;
/// Fewest pixels a shape must have to count as a fin.
const MIN_SHAPE: usize = 50;

const NO_FIN: &str = "No fin shape found in the image";

/// The fin outline in `(z, height)` metres from the encoded image `bytes`, scaled so its root edge is `root_chord` long. The
/// first point is the root's leading end `(0, 0)`, the outline runs round the tip, and the last is the root's trailing end
/// `(root_chord, 0)`: the order [`RadialKind::FreeformFin`](bowshock_model::RadialKind::FreeformFin) uses.
pub fn trace(bytes: &[u8], root_chord: f64) -> Result<Vec<(f64, f64)>, String> {
    let img = image::load_from_memory(bytes).map_err(|e| format!("Could not read the image: {e}"))?.to_luma8();
    let (w, h) = img.dimensions();
    if w < MIN_SIDE || h < MIN_SIDE {
        return Err("The image is too small (under 50 px)".into());
    }
    let (w, h) = (w as usize, h as usize);
    let grey = img.as_raw();
    let t = otsu(grey);
    // A dark border means a dark page: the fin is then the bright shape.
    let border: Vec<u8> = (0..w).flat_map(|x| [grey[x], grey[(h - 1) * w + x]]).chain((0..h).flat_map(|y| [grey[y * w], grey[y * w + w - 1]])).collect();
    let mean = border.iter().map(|&v| f64::from(v)).sum::<f64>() / border.len() as f64;
    let dark_page = mean <= f64::from(t);
    let mask: Vec<bool> = grey.iter().map(|&v| if dark_page { v > t } else { v <= t }).collect();

    let shape = largest_component(&mask, w, h);
    if shape.iter().filter(|&&b| b).count() < MIN_SHAPE {
        return Err(NO_FIN.into());
    }
    let contour = moore_trace(&shape, w, h);
    if contour.len() < 3 {
        return Err(NO_FIN.into());
    }
    let pts: Vec<(f64, f64)> = contour.iter().map(|&(x, y)| (x as f64, y as f64)).collect();
    let (min_x, max_x) = pts.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.0), a.1.max(p.0)));
    let (min_y, max_y) = pts.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.1), a.1.max(p.1)));
    let diag = (max_x - min_x).hypot(max_y - min_y);
    if diag < 1.0 {
        return Err(NO_FIN.into());
    }
    let mut tolerance = TOLERANCE * diag;
    let mut poly = simplify_closed(&pts, tolerance);
    while poly.len() > MAX_POINTS {
        tolerance *= 1.25;
        poly = simplify_closed(&pts, tolerance);
    }
    orient(poly, max_y, (max_y - min_y) * ROOT_BAND, root_chord)
}

/// Otsu's threshold: the grey level that maximises the between-class variance of the histogram; pixels `<= t` are the dark class.
fn otsu(grey: &[u8]) -> u8 {
    let mut hist = [0u64; 256];
    for &v in grey {
        hist[usize::from(v)] += 1;
    }
    let total = grey.len() as f64;
    let sum_all: f64 = hist.iter().enumerate().map(|(i, &c)| i as f64 * c as f64).sum();
    let (mut w0, mut sum0, mut best, mut best_t) = (0.0, 0.0, -1.0, 0u8);
    for (t, &count) in hist.iter().enumerate() {
        w0 += count as f64;
        if w0 == 0.0 {
            continue;
        }
        let w1 = total - w0;
        if w1 == 0.0 {
            break;
        }
        sum0 += t as f64 * count as f64;
        let (m0, m1) = (sum0 / w0, (sum_all - sum0) / w1);
        let between = w0 * w1 * (m0 - m1) * (m0 - m1);
        if between > best {
            best = between;
            best_t = t as u8;
        }
    }
    best_t
}

/// The pixels of the largest 8-connected component of `mask`.
fn largest_component(mask: &[bool], w: usize, h: usize) -> Vec<bool> {
    let mut label = vec![0u32; mask.len()];
    let (mut best_label, mut best_size, mut next) = (0, 0usize, 1u32);
    let mut stack = Vec::new();
    for start in 0..mask.len() {
        if !mask[start] || label[start] != 0 {
            continue;
        }
        let mut size = 0;
        label[start] = next;
        stack.push(start);
        while let Some(i) = stack.pop() {
            size += 1;
            let (x, y) = ((i % w) as isize, (i / w) as isize);
            for dy in -1..=1isize {
                for dx in -1..=1isize {
                    let (nx, ny) = (x + dx, y + dy);
                    if nx < 0 || ny < 0 || nx >= w as isize || ny >= h as isize {
                        continue;
                    }
                    let j = ny as usize * w + nx as usize;
                    if mask[j] && label[j] == 0 {
                        label[j] = next;
                        stack.push(j);
                    }
                }
            }
        }
        if size > best_size {
            (best_size, best_label) = (size, next);
        }
        next += 1;
    }
    label.iter().map(|&l| l == best_label && l != 0).collect()
}

/// Clockwise neighbour offsets (image coordinates, y down) starting from west.
const DIRS: [(isize, isize); 8] = [(-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1)];

/// Moore-neighbour tracing of the outer contour of the single shape in `shape`, starting at its first pixel in raster order
/// (whose west neighbour is background) and stopping when the walk returns to that pixel the way it started (Jacob's criterion).
fn moore_trace(shape: &[bool], w: usize, h: usize) -> Vec<(isize, isize)> {
    let fg = |x: isize, y: isize| x >= 0 && y >= 0 && x < w as isize && y < h as isize && shape[y as usize * w + x as usize];
    let Some(first) = shape.iter().position(|&b| b) else { return Vec::new() };
    let start = ((first % w) as isize, (first / w) as isize);
    let start_back = (start.0 - 1, start.1);
    let (mut cur, mut back) = (start, start_back);
    let mut out = vec![start];
    for _ in 0..(8 * w * h) {
        let bi = DIRS.iter().position(|&(dx, dy)| (cur.0 + dx, cur.1 + dy) == back).unwrap_or(0);
        let mut last_bg = back;
        let mut next = None;
        for k in 1..=8 {
            let d = DIRS[(bi + k) % 8];
            let q = (cur.0 + d.0, cur.1 + d.1);
            if fg(q.0, q.1) {
                next = Some(q);
                break;
            }
            last_bg = q;
        }
        let Some(q) = next else { break };
        cur = q;
        back = last_bg;
        if cur == start && back == start_back {
            break;
        }
        out.push(cur);
    }
    out.dedup();
    out
}

fn distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = dx.hypot(dy);
    if len < 1e-12 {
        return (p.0 - a.0).hypot(p.1 - a.1);
    }
    ((p.0 - a.0) * dy - (p.1 - a.1) * dx).abs() / len
}

/// Ramer–Douglas–Peucker on an open chain: the vertices kept, ends included.
fn rdp(chain: &[(f64, f64)], tolerance: f64, out: &mut Vec<(f64, f64)>) {
    let (first, last) = (chain[0], chain[chain.len() - 1]);
    let far = chain[1..chain.len() - 1].iter().enumerate().map(|(i, &p)| (i + 1, distance(p, first, last))).max_by(|a, b| a.1.total_cmp(&b.1));
    match far {
        Some((i, d)) if d > tolerance => {
            rdp(&chain[..=i], tolerance, out);
            rdp(&chain[i..], tolerance, out);
        }
        _ => {
            if out.last() != Some(&first) {
                out.push(first);
            }
            out.push(last);
        }
    }
}

/// Ramer–Douglas–Peucker on a closed contour: it is cut at the first point and the one farthest from it, and each half simplified.
fn simplify_closed(pts: &[(f64, f64)], tolerance: f64) -> Vec<(f64, f64)> {
    let far = (1..pts.len()).max_by(|&a, &b| {
        let da = (pts[a].0 - pts[0].0).hypot(pts[a].1 - pts[0].1);
        let db = (pts[b].0 - pts[0].0).hypot(pts[b].1 - pts[0].1);
        da.total_cmp(&db)
    });
    let Some(far) = far else { return pts.to_vec() };
    let mut out = Vec::new();
    rdp(&pts[..=far], tolerance, &mut out);
    let mut back: Vec<(f64, f64)> = pts[far..].to_vec();
    back.push(pts[0]);
    rdp(&back, tolerance, &mut out);
    out.pop(); // the closing point repeats the first
    out
}

/// Puts the polygon in fin form: the root (the vertices near the lowest row `max_y`, within `band`) laid on height 0 and scaled
/// to `root_chord`, ordered from the root's leading end round the tip to its trailing end.
fn orient(poly: Vec<(f64, f64)>, max_y: f64, band: f64, root_chord: f64) -> Result<Vec<(f64, f64)>, String> {
    let on_root: Vec<bool> = poly.iter().map(|p| max_y - p.1 <= band).collect();
    let roots: Vec<f64> = poly.iter().zip(&on_root).filter(|(_, &r)| r).map(|(p, _)| p.0).collect();
    let (root_min, root_max) = roots.iter().fold((f64::MAX, f64::MIN), |a, &x| (a.0.min(x), a.1.max(x)));
    if roots.len() < 2 || root_max - root_min < 2.0 {
        return Err(NO_FIN.into());
    }
    let k = root_chord / (root_max - root_min);
    // Keep only the root's two ends (anything between them lies on the straight root edge).
    let keep = |i: usize| !on_root[i] || poly[i].0 <= root_min || poly[i].0 >= root_max;
    let left = poly.iter().zip(&on_root).position(|(p, &r)| r && p.0 <= root_min).ok_or(NO_FIN)?;
    let n = poly.len();
    let cycle: Vec<usize> = (0..n).map(|i| (left + i) % n).filter(|&i| keep(i)).collect();
    // Walk away from the root's leading end into the fin body: if the cycle's next vertex is the root's other end, reverse it.
    let ordered: Vec<usize> = match cycle.get(1) {
        Some(&second) if on_root[second] => std::iter::once(cycle[0]).chain(cycle[1..].iter().rev().copied()).collect(),
        _ => cycle,
    };
    let mut out: Vec<(f64, f64)> = ordered.iter().map(|&i| ((poly[i].0 - root_min) * k, if on_root[i] { 0.0 } else { (max_y - poly[i].1) * k })).collect();
    if out.len() < 3 {
        return Err(NO_FIN.into());
    }
    if let Some(last) = out.last_mut() {
        last.1 = 0.0;
    }
    out[0] = (0.0, 0.0);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::{GrayImage, ImageFormat, Luma};

    /// A 400×200 white page with a black trapezoid: root (50..350) at the bottom row 180, tip (150..250) at row 20.
    fn trapezoid(invert: bool) -> Vec<u8> {
        let (page, ink) = if invert { (0u8, 255u8) } else { (255u8, 0u8) };
        let mut img = GrayImage::from_pixel(400, 200, Luma([page]));
        for y in 20..=180u32 {
            let t = f64::from(y - 20) / 160.0;
            let (x0, x1) = (150.0 - 100.0 * t, 250.0 + 100.0 * t);
            for x in x0.round() as u32..=x1.round() as u32 {
                img.put_pixel(x, y, Luma([ink]));
            }
        }
        let mut bytes = Vec::new();
        img.write_to(&mut std::io::Cursor::new(&mut bytes), ImageFormat::Png).unwrap();
        bytes
    }

    fn check(points: &[(f64, f64)], chord: f64) {
        let k = chord / 300.0;
        let corners = [(0.0, 0.0), (100.0 * k, 160.0 * k), (200.0 * k, 160.0 * k), (300.0 * k, 0.0)];
        assert_eq!(points.len(), 4, "{points:?}");
        for (got, want) in points.iter().zip(corners) {
            assert!((got.0 - want.0).abs() < 0.02 * chord && (got.1 - want.1).abs() < 0.02 * chord, "{got:?} vs {want:?}");
        }
    }

    #[test]
    fn a_trapezoid_is_traced_to_its_corners() {
        check(&trace(&trapezoid(false), 0.1).unwrap(), 0.1);
    }

    #[test]
    fn a_light_fin_on_a_dark_page_traces_the_same() {
        check(&trace(&trapezoid(true), 0.1).unwrap(), 0.1);
    }

    #[test]
    fn blank_and_tiny_images_are_refused() {
        let mut bytes = Vec::new();
        GrayImage::from_pixel(200, 200, Luma([255])).write_to(&mut std::io::Cursor::new(&mut bytes), ImageFormat::Png).unwrap();
        assert_eq!(trace(&bytes, 0.1).unwrap_err(), "No fin shape found in the image");
        let mut small = Vec::new();
        GrayImage::from_pixel(30, 30, Luma([255])).write_to(&mut std::io::Cursor::new(&mut small), ImageFormat::Png).unwrap();
        assert_eq!(trace(&small, 0.1).unwrap_err(), "The image is too small (under 50 px)");
    }
}
