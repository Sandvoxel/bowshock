//! A small PDF 1.4 writer: pages of vector paths, circles, rectangles and text, with the standard-14 fonts Helvetica and
//! Helvetica-Bold (so nothing is embedded) in WinAnsi encoding. Content streams are written uncompressed.
//!
//! Positions and sizes are millimetres measured from the page's top-left corner (the writer converts to points from the
//! bottom-left); font sizes are in points. Text metrics are the Helvetica and Helvetica-Bold advance widths of the Adobe core-14
//! AFM files (Adobe Font Metrics, "Core 14 AFM files"), for the printable ASCII characters; any other character is measured as
//! 556 units (the width of a digit).

use std::fmt::Write;

/// Points per millimetre's worth: 72 pt per 25.4 mm.
const PT: f64 = 72.0 / 25.4;
/// Bézier constant for a quarter circle.
const KAPPA: f64 = 0.552_284_75;

/// Paper size.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PageSize {
    /// US Letter, 612 × 792 pt.
    Letter,
    /// ISO A4, 595.28 × 841.89 pt.
    A4,
}

impl PageSize {
    /// Width and height in points.
    pub fn points(self) -> (f64, f64) {
        match self {
            PageSize::Letter => (612.0, 792.0),
            PageSize::A4 => (595.28, 841.89),
        }
    }

    /// Width and height in millimetres.
    pub fn mm(self) -> (f64, f64) {
        let (w, h) = self.points();
        (w / PT, h / PT)
    }
}

/// Advance widths (1/1000 em) of Helvetica for the characters 32 to 126.
const HELVETICA: [u16; 95] = [
    278, 278, 355, 556, 556, 889, 667, 191, 333, 333, 389, 584, 278, 333, 278, 278, // space to slash
    556, 556, 556, 556, 556, 556, 556, 556, 556, 556, // 0 to 9
    278, 278, 584, 584, 584, 556, 1015, // colon to at
    667, 667, 722, 722, 667, 611, 778, 722, 278, 500, 667, 556, 833, 722, 778, 667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, // A to Z
    278, 278, 278, 469, 556, 333, // bracketleft to grave
    556, 556, 500, 556, 556, 278, 556, 556, 222, 222, 500, 222, 833, 556, 556, 556, 556, 333, 500, 278, 556, 500, 722, 500, 500, 500, // a to z
    334, 260, 334, 584, // braceleft to asciitilde
];

/// Advance widths (1/1000 em) of Helvetica-Bold for the characters 32 to 126.
const HELVETICA_BOLD: [u16; 95] = [
    278, 333, 474, 556, 556, 889, 722, 238, 333, 333, 389, 584, 278, 333, 278, 278, // space to slash
    556, 556, 556, 556, 556, 556, 556, 556, 556, 556, // 0 to 9
    333, 333, 584, 584, 584, 611, 975, // colon to at
    722, 722, 722, 722, 667, 611, 778, 722, 278, 556, 722, 611, 833, 722, 778, 667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, // A to Z
    333, 278, 333, 584, 556, 333, // bracketleft to grave
    556, 611, 556, 611, 556, 333, 611, 611, 278, 278, 556, 278, 889, 611, 611, 611, 611, 389, 556, 333, 611, 556, 778, 556, 556, 500, // a to z
    389, 280, 389, 584, // braceleft to asciitilde
];

/// `s` with the few symbols WinAnsi lacks written in ASCII (an arrow, the true minus sign, almost-equal).
fn fold(s: &str) -> String {
    s.replace('\u{2192}', "->").replace('\u{2212}', "-").replace('\u{2248}', "~")
}

/// The WinAnsi byte for `c`, or `?` when the encoding has none.
fn win_ansi(c: char) -> u8 {
    match c {
        ' '..='~' => c as u8,
        '\u{a0}'..='\u{ff}' => c as u8,
        '\u{2013}' => 0x96,
        '\u{2014}' => 0x97,
        '\u{2018}' => 0x91,
        '\u{2019}' => 0x92,
        '\u{201c}' => 0x93,
        '\u{201d}' => 0x94,
        '\u{2022}' => 0x95,
        '\u{2026}' => 0x85,
        '\u{20ac}' => 0x80,
        _ => b'?',
    }
}

/// One page: the drawing operators so far.
#[derive(Clone, Debug)]
pub struct Page {
    size: PageSize,
    landscape: bool,
    ops: String,
}

impl Page {
    fn new(size: PageSize, landscape: bool) -> Page {
        Page { size, landscape, ops: String::new() }
    }

    /// The page's width and height as it is printed, points.
    fn dims(&self) -> (f64, f64) {
        let (w, h) = self.size.points();
        if self.landscape {
            (h, w)
        } else {
            (w, h)
        }
    }

    /// The page's width and height as it is printed, mm: swapped for a landscape page.
    pub fn size_mm(&self) -> (f64, f64) {
        let (w, h) = self.dims();
        (w / PT, h / PT)
    }

    /// x in points of `mm`.
    fn x(&self, mm: f64) -> f64 {
        mm * PT
    }

    /// y in points (from the bottom) of `mm` from the top.
    fn y(&self, mm: f64) -> f64 {
        self.dims().1 - mm * PT
    }

    fn point(&self, p: (f64, f64)) -> String {
        format!("{:.3} {:.3}", self.x(p.0), self.y(p.1))
    }

    fn stroke_style(&mut self, w: f64, dash: Option<f64>) {
        let _ = write!(self.ops, "{:.3} w ", (w * PT).max(0.05));
        match dash {
            Some(d) => {
                let d = (d * PT).max(0.5);
                let _ = write!(self.ops, "[{d:.3} {d:.3}] 0 d ");
            }
            None => self.ops.push_str("[] 0 d "),
        }
    }

    /// Sets the stroke and fill colour (0 to 1 per channel).
    pub fn rgb(&mut self, r: f64, g: f64, b: f64) {
        let _ = writeln!(self.ops, "{r:.3} {g:.3} {b:.3} RG {r:.3} {g:.3} {b:.3} rg");
    }

    /// Sets the stroke and fill colour to a grey (0 black, 1 white).
    pub fn gray(&mut self, g: f64) {
        self.rgb(g, g, g);
    }

    /// A straight line of width `w` mm.
    pub fn line(&mut self, a: (f64, f64), b: (f64, f64), w: f64) {
        self.path(&[a, b], false, w, None);
    }

    /// A polyline or polygon (`closed`) stroked with width `w` mm, dashed with `dash` mm on and off if given.
    pub fn path(&mut self, pts: &[(f64, f64)], closed: bool, w: f64, dash: Option<f64>) {
        let Some((first, rest)) = pts.split_first() else { return };
        self.stroke_style(w, dash);
        let _ = write!(self.ops, "{} m ", self.point(*first));
        for p in rest {
            let _ = write!(self.ops, "{} l ", self.point(*p));
        }
        self.ops.push_str(if closed { "s\n" } else { "S\n" });
    }

    /// A circle of radius `r` mm stroked with width `w` mm.
    pub fn circle(&mut self, c: (f64, f64), r: f64, w: f64) {
        self.stroke_style(w, None);
        self.circle_path(c, r);
        self.ops.push_str("S\n");
    }

    /// A filled disc of radius `r` mm.
    pub fn disc(&mut self, c: (f64, f64), r: f64) {
        self.circle_path(c, r);
        self.ops.push_str("f\n");
    }

    fn circle_path(&mut self, c: (f64, f64), r: f64) {
        let k = r * KAPPA;
        let p = |dx: f64, dy: f64| self.point((c.0 + dx, c.1 + dy));
        let curve = |a: (f64, f64), b: (f64, f64), e: (f64, f64)| format!("{} {} {} c ", p(a.0, a.1), p(b.0, b.1), p(e.0, e.1));
        let mut s = format!("{} m ", p(r, 0.0));
        s += &curve((r, k), (k, r), (0.0, r));
        s += &curve((-k, r), (-r, k), (-r, 0.0));
        s += &curve((-r, -k), (-k, -r), (0.0, -r));
        s += &curve((k, -r), (r, -k), (r, 0.0));
        s.push_str("h ");
        self.ops.push_str(&s);
    }

    /// A rectangle of `size` with its top-left corner at `origin`, filled with `gray` (0 black to 1 white).
    pub fn fill_rect(&mut self, origin: (f64, f64), size: (f64, f64), gray: f64) {
        let _ = writeln!(self.ops, "{gray:.3} g {} {:.3} {:.3} re f 0 g", self.point((origin.0, origin.1 + size.1)), size.0 * PT, size.1 * PT);
    }

    /// Clips everything drawn until [`Page::restore`] to the rectangle.
    pub fn clip_rect(&mut self, origin: (f64, f64), size: (f64, f64)) {
        let _ = writeln!(self.ops, "q {} {:.3} {:.3} re W n", self.point((origin.0, origin.1 + size.1)), size.0 * PT, size.1 * PT);
    }

    /// Ends the clip of [`Page::clip_rect`].
    pub fn restore(&mut self) {
        self.ops.push_str("Q\n");
    }

    /// `s` of `size` pt with its baseline starting at `at`.
    pub fn text(&mut self, at: (f64, f64), size: f64, bold: bool, s: &str) {
        let mut escaped = String::with_capacity(s.len());
        for c in fold(s).chars() {
            match win_ansi(c) {
                b @ (b'(' | b')' | b'\\') => {
                    escaped.push('\\');
                    escaped.push(b as char);
                }
                b @ 0x20..=0x7e => escaped.push(b as char),
                b => {
                    let _ = write!(escaped, "\\{b:03o}");
                }
            }
        }
        let _ = writeln!(self.ops, "BT /{} {size:.2} Tf {} Td ({escaped}) Tj ET", if bold { "F2" } else { "F1" }, self.point(at));
    }

    /// The width in mm of `s` set at `size` pt.
    pub fn text_width(size: f64, bold: bool, s: &str) -> f64 {
        let table = if bold { &HELVETICA_BOLD } else { &HELVETICA };
        let units: u32 = fold(s)
            .chars()
            .map(|c| match win_ansi(c) {
                b @ 0x20..=0x7e => u32::from(table[usize::from(b - 0x20)]),
                _ => 556,
            })
            .sum();
        f64::from(units) / 1000.0 * size / PT
    }
}

/// A document: pages in order.
#[derive(Clone, Debug)]
pub struct Pdf {
    pages: Vec<Page>,
    title: String,
}

impl Pdf {
    /// An empty document titled `title`.
    pub fn new(title: &str) -> Pdf {
        Pdf { pages: Vec::new(), title: title.to_string() }
    }

    /// Adds a blank page and returns it for drawing.
    pub fn page(&mut self, size: PageSize) -> &mut Page {
        self.pages.push(Page::new(size, false));
        self.pages.last_mut().expect("just pushed")
    }

    /// Adds a blank page turned on its side (the long edge is the width) and returns it for drawing.
    pub fn landscape_page(&mut self, size: PageSize) -> &mut Page {
        self.pages.push(Page::new(size, true));
        self.pages.last_mut().expect("just pushed")
    }

    /// How many pages there are.
    pub fn page_count(&self) -> usize {
        self.pages.len()
    }

    /// The page most recently added.
    pub fn current(&mut self) -> Option<&mut Page> {
        self.pages.last_mut()
    }

    /// The finished file.
    pub fn to_bytes(&self) -> Vec<u8> {
        // Object numbers: 1 catalog, 2 page tree, 3 and 4 the fonts, 5 the info dictionary, then a page and its content stream
        // for each page.
        let n_objects = 5 + 2 * self.pages.len();
        let mut out: Vec<u8> = Vec::new();
        let mut offsets = vec![0usize; n_objects + 1];
        out.extend_from_slice(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n");
        let mut object = |out: &mut Vec<u8>, n: usize, body: &[u8]| {
            offsets[n] = out.len();
            out.extend_from_slice(format!("{n} 0 obj\n").as_bytes());
            out.extend_from_slice(body);
            out.extend_from_slice(b"\nendobj\n");
        };
        object(&mut out, 1, b"<< /Type /Catalog /Pages 2 0 R >>");
        let kids: String = (0..self.pages.len()).map(|i| format!("{} 0 R", 6 + 2 * i)).collect::<Vec<_>>().join(" ");
        object(&mut out, 2, format!("<< /Type /Pages /Kids [{kids}] /Count {} >>", self.pages.len()).as_bytes());
        object(&mut out, 3, b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>");
        object(&mut out, 4, b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>");
        let mut title = Vec::new();
        for c in self.title.chars() {
            match win_ansi(c) {
                b @ (b'(' | b')' | b'\\') => title.extend_from_slice(&[b'\\', b]),
                b => title.extend_from_slice(format!("\\{b:03o}").as_bytes()),
            }
        }
        let mut info = b"<< /Title (".to_vec();
        info.extend_from_slice(&title);
        info.extend_from_slice(b") /Producer (Bowshock) >>");
        object(&mut out, 5, &info);
        for (i, page) in self.pages.iter().enumerate() {
            let (page_obj, content_obj) = (6 + 2 * i, 7 + 2 * i);
            let (w, h) = page.dims();
            object(
                &mut out,
                page_obj,
                format!(
                    "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {w:.2} {h:.2}] /Resources << /Font << /F1 3 0 R /F2 4 0 R >> >> /Contents {content_obj} 0 R >>"
                )
                .as_bytes(),
            );
            let mut stream = format!("<< /Length {} >>\nstream\n", page.ops.len()).into_bytes();
            stream.extend_from_slice(page.ops.as_bytes());
            stream.extend_from_slice(b"\nendstream");
            object(&mut out, content_obj, &stream);
        }
        let xref = out.len();
        out.extend_from_slice(format!("xref\n0 {}\n0000000000 65535 f \n", n_objects + 1).as_bytes());
        for offset in &offsets[1..] {
            out.extend_from_slice(format!("{offset:010} 00000 n \n").as_bytes());
        }
        out.extend_from_slice(format!("trailer\n<< /Size {} /Root 1 0 R /Info 5 0 R >>\nstartxref\n{xref}\n%%EOF", n_objects + 1).as_bytes());
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Vec<u8> {
        let mut pdf = Pdf::new("Test (rocket) · 1");
        {
            let page = pdf.page(PageSize::Letter);
            page.line((10.0, 10.0), (100.0, 10.0), 0.3);
            page.path(&[(10.0, 20.0), (50.0, 20.0), (30.0, 40.0)], true, 0.2, Some(2.0));
            page.circle((60.0, 60.0), 12.0, 0.2);
            page.fill_rect((10.0, 80.0), (25.0, 25.0), 0.8);
            page.text((10.0, 120.0), 10.0, true, "Fin (A) \\ 25° × 3 — ok");
        }
        pdf.page(PageSize::A4).text((10.0, 10.0), 9.0, false, "second page");
        pdf.to_bytes()
    }

    fn find(haystack: &[u8], needle: &[u8], from: usize) -> Option<usize> {
        haystack[from..].windows(needle.len()).position(|w| w == needle).map(|p| p + from)
    }

    #[test]
    fn the_file_is_framed_and_every_xref_offset_points_at_its_object() {
        let bytes = sample();
        assert!(bytes.starts_with(b"%PDF-1.4"));
        assert!(bytes.ends_with(b"%%EOF"));
        let start = find(&bytes, b"startxref\n", 0).unwrap() + "startxref\n".len();
        let end = find(&bytes, b"\n", start).unwrap();
        let xref: usize = std::str::from_utf8(&bytes[start..end]).unwrap().parse().unwrap();
        assert!(bytes[xref..].starts_with(b"xref\n"));
        let table = std::str::from_utf8(&bytes[xref..start]).unwrap();
        let mut lines = table.lines().skip(1);
        let count: usize = lines.next().unwrap().split(' ').nth(1).unwrap().parse().unwrap();
        assert_eq!(count, 5 + 2 * 2 + 1);
        assert_eq!(lines.next().unwrap(), "0000000000 65535 f ");
        for (n, line) in lines.take_while(|l| !l.starts_with("trailer")).enumerate() {
            assert_eq!(line.len(), 19, "entries are 20 bytes with the newline");
            let offset: usize = line[..10].parse().unwrap();
            assert!(bytes[offset..].starts_with(format!("{} 0 obj\n", n + 1).as_bytes()), "object {} at {offset}", n + 1);
        }
    }

    #[test]
    fn stream_lengths_are_exact() {
        let bytes = sample();
        let mut at = 0;
        let mut streams = 0;
        while let Some(p) = find(&bytes, b"/Length ", at) {
            let digits: String = bytes[p + 8..].iter().take_while(|b| b.is_ascii_digit()).map(|&b| b as char).collect();
            let length: usize = digits.parse().unwrap();
            let body = find(&bytes, b"stream\n", p).unwrap() + 7;
            assert!(bytes[body + length..].starts_with(b"\nendstream"));
            at = body + length;
            streams += 1;
        }
        assert_eq!(streams, 2);
    }

    #[test]
    fn text_is_escaped_and_encoded() {
        let bytes = sample();
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("(Fin \\(A\\) \\\\ 25\\260 \\327 3 \\227 ok) Tj"), "{text}");
        assert!(text.contains("/F2 10.00 Tf"));
    }

    #[test]
    fn widths_come_from_the_afm_tables() {
        // "A" is 667 units in Helvetica, "m" is 889 in the bold face, and a space is 278 in both.
        let em = |w: f64, size: f64| w / 1000.0 * size / PT;
        assert!((Page::text_width(10.0, false, "A") - em(667.0, 10.0)).abs() < 1e-9);
        assert!((Page::text_width(12.0, true, "m") - em(889.0, 12.0)).abs() < 1e-9);
        assert!((Page::text_width(10.0, true, " ") - em(278.0, 10.0)).abs() < 1e-9);
        assert!((Page::text_width(10.0, false, "é") - em(556.0, 10.0)).abs() < 1e-9);
        assert_eq!(HELVETICA.len(), 95);
        assert_eq!(HELVETICA_BOLD.len(), 95);
    }

    #[test]
    fn a_landscape_page_swaps_its_sides() {
        let mut pdf = Pdf::new("t");
        let (w, h) = pdf.landscape_page(PageSize::A4).size_mm();
        assert!((w - 297.0).abs() < 0.1 && (h - 210.0).abs() < 0.1);
        let bytes = pdf.to_bytes();
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("/MediaBox [0 0 841.89 595.28]"), "{text}");
        // The top-left corner stays the origin: 10 mm from the top is 10 mm below the page's height.
        let mut pdf = Pdf::new("t");
        pdf.landscape_page(PageSize::A4).text((0.0, 10.0), 9.0, false, "x");
        let bytes = pdf.to_bytes();
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("0.000 566.93"), "{text}");
    }

    #[test]
    fn page_sizes_are_in_millimetres() {
        let (w, h) = PageSize::A4.mm();
        assert!((w - 210.0).abs() < 0.1 && (h - 297.0).abs() < 0.1);
        let (w, h) = PageSize::Letter.mm();
        assert!((w - 215.9).abs() < 0.01 && (h - 279.4).abs() < 0.01);
    }
}
