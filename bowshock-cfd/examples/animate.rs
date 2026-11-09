//! Animate the ogive-cylinder start-up (M = 1.98, medium grid) from uniform free stream.
//!
//! `cargo run --release --example animate [-- --schlieren] [--sweep] [--frames N]`
//!   default      : Mach-number field, `out/anim/ogive_mach.{gif,mp4}`, frames in `out/anim/frames/`
//!   --schlieren  : grayscale exp(-k |grad rho|), `ogive_schlieren.*`
//!   --sweep      : warm-started quasi-steady Mach sweep 1.5 -> 4, `sweep_mach.*`
//! Top half = solution, bottom half = mirror about the axis; body is dark gray.
use bowshock_cfd::gas::{cons_to_prim, freestream, sound_speed};
use bowshock_cfd::ogive::OgiveCase;
use bowshock_cfd::solver::Solver;
use image::RgbImage;
use std::fs::{self, File};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Instant;

const W: usize = 1200;
const H: usize = 600;
const X0: f64 = -0.5;
const X1: f64 = 9.0; // 9.5 d wide -> 126.3 px/d, +-4.75 d tall
const BODY: [u8; 3] = [60, 60, 60];

fn turbo(t: f64) -> [u8; 3] {
    let x = t.clamp(0.0, 1.0);
    let r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
    let g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))));
    let b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
    [r, g, b].map(|c| (c.clamp(0.0, 1.0) * 255.0) as u8)
}

fn mach_field(s: &Solver) -> Vec<f64> {
    s.u.iter().map(|q| {
        let w = cons_to_prim(q);
        (w[1] * w[1] + w[2] * w[2]).sqrt() / sound_speed(&w)
    }).collect()
}

/// |grad rho| per cell, from computational-space central differences and the cell-centre metric.
fn grad_rho(s: &Solver) -> Vec<f64> {
    let g = &s.grid;
    let (ni, nj) = (g.ni, g.nj);
    let rho: Vec<f64> = s.u.iter().map(|q| q[0]).collect();
    let mut out = vec![0.0; ni * nj];
    for j in 0..nj {
        for i in 0..ni {
            let (ia, ib) = (i.saturating_sub(1), (i + 1).min(ni - 1));
            let (ja, jb) = (j.saturating_sub(1), (j + 1).min(nj - 1));
            let (a, b) = (j * ni + ia, j * ni + ib);
            let (c, d) = (ja * ni + i, jb * ni + i);
            let (xi_x, xi_r, dr_i) = (g.xc[b] - g.xc[a], g.rc[b] - g.rc[a], rho[b] - rho[a]);
            let (et_x, et_r, dr_j) = (g.xc[d] - g.xc[c], g.rc[d] - g.rc[c], rho[d] - rho[c]);
            let det = xi_x * et_r - xi_r * et_x;
            if det.abs() < 1e-14 { continue; }
            let gx = (dr_i * et_r - dr_j * xi_r) / det;
            let gr = (xi_x * dr_j - et_x * dr_i) / det;
            out[j * ni + i] = (gx * gx + gr * gr).sqrt();
        }
    }
    out
}

fn tri(img: &mut [u8], p: [(f64, f64); 3], col: [u8; 3]) {
    let minx = p.iter().map(|q| q.0).fold(f64::MAX, f64::min).floor().max(0.0) as usize;
    let maxx = p.iter().map(|q| q.0).fold(f64::MIN, f64::max).ceil().min(W as f64 - 1.0);
    let miny = p.iter().map(|q| q.1).fold(f64::MAX, f64::min).floor().max(0.0) as usize;
    let maxy = p.iter().map(|q| q.1).fold(f64::MIN, f64::max).ceil().min(H as f64 - 1.0);
    if maxx < 0.0 || maxy < 0.0 { return; }
    let area = (p[1].0 - p[0].0) * (p[2].1 - p[0].1) - (p[1].1 - p[0].1) * (p[2].0 - p[0].0);
    if area.abs() < 1e-12 { return; }
    let e = -1e-6;
    for y in miny..=(maxy as usize) {
        for x in minx..=(maxx as usize) {
            let (px, py) = (x as f64 + 0.5, y as f64 + 0.5);
            let w0 = ((p[1].0 - px) * (p[2].1 - py) - (p[1].1 - py) * (p[2].0 - px)) / area;
            let w1 = ((p[2].0 - px) * (p[0].1 - py) - (p[2].1 - py) * (p[0].0 - px)) / area;
            let w2 = 1.0 - w0 - w1;
            if w0 >= e && w1 >= e && w2 >= e {
                img[(y * W + x) * 3..][..3].copy_from_slice(&col);
            }
        }
    }
}

/// Render per-cell scalar `val` (already mapped to colours) to an RGB buffer.
fn render(s: &Solver, cols: &[[u8; 3]]) -> Vec<u8> {
    let g = &s.grid;
    let mut img = vec![0u8; W * H * 3];
    for px in img.chunks_exact_mut(3) { px.copy_from_slice(&BODY); }
    let sc = W as f64 / (X1 - X0);
    let pt = |x: f64, r: f64, sign: f64| ((x - X0) * sc, H as f64 / 2.0 - sign * r * sc);
    for j in 0..g.nj {
        for i in 0..g.ni {
            let n = [g.node(i, j), g.node(i + 1, j), g.node(i + 1, j + 1), g.node(i, j + 1)];
            if n.iter().all(|q| q.0 > X1 + 1.0) || n.iter().all(|q| q.0 < X0 - 1.0) { continue; }
            let col = cols[j * g.ni + i];
            for sign in [1.0, -1.0] {
                let q = n.map(|(x, r)| pt(x, r, sign));
                tri(&mut img, [q[0], q[1], q[2]], col);
                tri(&mut img, [q[0], q[2], q[3]], col);
            }
        }
    }
    img
}

/// Color range scaled to the freestream Mach number so shocks (Mach drop) and
/// expansions (Mach rise) stay visible at every speed.
fn mach_cols(s: &Solver, m_inf: f64) -> Vec<[u8; 3]> {
    let (lo, hi) = (0.6 * m_inf, 1.15 * m_inf);
    mach_field(s).iter().map(|&m| turbo((m - lo) / (hi - lo))).collect()
}

fn schlieren_cols(s: &Solver) -> Vec<[u8; 3]> {
    let g = grad_rho(s);
    let k = 0.25; // shock gradients ~ 10-30
    g.iter().map(|&v| { let c = ((-k * v / 1.0).exp() * 255.0) as u8; [c, c, c] }).collect()
}

struct Out {
    dir: PathBuf,
    frames: PathBuf,
    name: String,
    n: usize,
    gif: gif::Encoder<File>,
}

impl Out {
    fn new(name: &str) -> Out {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../out/anim");
        let frames = dir.join("frames");
        fs::create_dir_all(&frames).unwrap();
        for e in fs::read_dir(&frames).unwrap().flatten() {
            if e.file_name().to_string_lossy().starts_with(name) { fs::remove_file(e.path()).unwrap(); }
        }
        let mut gif = gif::Encoder::new(File::create(dir.join(format!("{name}.gif"))).unwrap(), W as u16, H as u16, &[]).unwrap();
        gif.set_repeat(gif::Repeat::Infinite).unwrap();
        Out { dir, frames, name: name.into(), n: 0, gif }
    }
    fn push(&mut self, img: Vec<u8>, tag: &str) {
        let mut f = gif::Frame::from_rgb_speed(W as u16, H as u16, &img, 20);
        f.delay = 7; // ~15 fps
        self.gif.write_frame(&f).unwrap();
        let path = self.frames.join(format!("{}_{:04}_{}.png", self.name, self.n, tag));
        RgbImage::from_raw(W as u32, H as u32, img).unwrap().save(path).unwrap();
        self.n += 1;
    }
    fn finish(self) {
        let Out { dir, frames, name, n, gif } = self;
        drop(gif);
        // ffmpeg wants a plain sequence: link frames under a simple pattern.
        let seq = dir.join(format!("seq_{name}"));
        let _ = fs::remove_dir_all(&seq);
        fs::create_dir_all(&seq).unwrap();
        let mut files: Vec<_> = fs::read_dir(&frames).unwrap().flatten().map(|e| e.path())
            .filter(|p| p.file_name().unwrap().to_string_lossy().starts_with(&format!("{name}_"))).collect();
        files.sort();
        for (k, f) in files.iter().enumerate() { std::os::unix::fs::symlink(fs::canonicalize(f).unwrap(), seq.join(format!("{k:05}.png"))).unwrap(); }
        let mp4 = dir.join(format!("{name}.mp4"));
        let ok = Command::new("ffmpeg").args(["-y", "-loglevel", "error", "-framerate", "24", "-i"])
            .arg(seq.join("%05d.png")).args(["-pix_fmt", "yuv420p", "-crf", "20", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2"]).arg(&mp4)
            .status().map(|s| s.success()).unwrap_or(false);
        let _ = fs::remove_dir_all(&seq);
        println!("{name}: {n} frames -> {} ({} B){}", dir.join(format!("{name}.gif")).display(),
            fs::metadata(dir.join(format!("{name}.gif"))).unwrap().len(),
            if ok { format!(", {} ({} B)", mp4.display(), fs::metadata(&mp4).unwrap().len()) } else { ", mp4 failed/ffmpeg missing".into() });
    }
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let schl = a.iter().any(|x| x == "--schlieren");
    let sweep = a.iter().any(|x| x == "--sweep");
    let nframes: usize = a.iter().position(|x| x == "--frames").and_then(|i| a.get(i + 1)).and_then(|v| v.parse().ok()).unwrap_or(120);
    let t0 = Instant::now();
    let field = if schl { "schlieren" } else { "mach" };

    if sweep {
        let mut case = OgiveCase::level(1.5, 1); // outer radius sized for the lowest Mach (widest shock)
        case.cfl = 0.5;
        let mut s = case.build_solver();
        for _ in 0..4000 { s.step(); }
        let mut out = Out::new(&format!("sweep_{field}"));
        for k in 0..nframes {
            let m = 1.5 + 2.5 * k as f64 / (nframes - 1) as f64;
            s.cfg.freestream = freestream(m);
            for _ in 0..400 { s.step(); }
            let cols = if schl { schlieren_cols(&s) } else { mach_cols(&s, m) };
            out.push(render(&s, &cols), &format!("M{m:.2}"));
        }
        out.finish();
    } else {
        let case = OgiveCase::level(1.98, 1);
        let mut s = case.build_solver();
        println!("grid {:?}", case.cells());
        let mut out = Out::new(&format!("ogive_{field}"));
        let mut next_it = 0usize;
        for k in 0..nframes {
            let target = (8.0 * (k as f64 * 120.0 / nframes as f64).powf(1.4)).round() as usize;
            while next_it < target { s.step(); next_it += 1; }
            let cols = if schl { schlieren_cols(&s) } else { mach_cols(&s, 1.98) };
            out.push(render(&s, &cols), &format!("it{next_it}"));
        }
        out.finish();
    }
    println!("time {:.1}s", t0.elapsed().as_secs_f64());
}
