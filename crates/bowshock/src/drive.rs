//! `bowshock drive <script> [out-dir]`: run the app headless from a script and save screenshots, so the UI can be
//! exercised and inspected without a display (by people, CI, or coding agents).
//!
//! The script is iced's `.ice` instruction syntax plus a few directives, one per line:
//!
//! ```text
//! viewport 1280x800        # logical size; only before the first instruction (default 1280x800)
//! scale 2                  # screenshot scale factor (default 2)
//! click "Cutaway"          # iced instructions: click/move/press/release <target>, type "text", type enter, expect "text"
//! click (640, 400)         # a target is "text", #widget-id, or (x, y) in logical px
//! screenshot cutaway.png   # save a PNG into the output directory
//! # comment
//! ```
//!
//! Rendering uses iced's headless renderer, the same one the window uses (`ICED_TEST_BACKEND` selects it).

use std::path::{Path, PathBuf};
use std::{fmt, fs};

use iced::Program;
use iced::{Size, Theme};
use iced_test::emulator::{self, Emulator};
use iced_test::futures::futures::channel::mpsc;
use iced_test::futures::futures::executor;
use iced_test::futures::futures::StreamExt;
use iced_test::Instruction;

enum Step {
    Run(Instruction),
    Screenshot(PathBuf),
}

#[derive(Debug)]
pub struct Error(String);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

struct Script {
    viewport: Size,
    scale: f32,
    steps: Vec<Step>,
}

fn parse(source: &str, out_dir: &Path) -> Result<Script, Error> {
    let mut script = Script { viewport: Size::new(1280.0, 800.0), scale: 2.0, steps: Vec::new() };
    for (n, raw) in source.lines().enumerate() {
        let line = raw.trim();
        let at = |msg: String| Error(format!("line {}: {msg}: {line}", n + 1));
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(size) = line.strip_prefix("viewport ") {
            if !script.steps.is_empty() {
                return Err(at("viewport must come before the first instruction".into()));
            }
            let (w, h) = size.split_once('x').ok_or_else(|| at("expected WxH".into()))?;
            let parse = |s: &str| s.trim().parse::<f32>().map_err(|e| at(e.to_string()));
            script.viewport = Size::new(parse(w)?, parse(h)?);
        } else if let Some(scale) = line.strip_prefix("scale ") {
            script.scale = scale.trim().parse().map_err(|e: std::num::ParseFloatError| at(e.to_string()))?;
        } else if let Some(name) = line.strip_prefix("screenshot ") {
            script.steps.push(Step::Screenshot(out_dir.join(name.trim())));
        } else {
            script.steps.push(Step::Run(Instruction::parse(line).map_err(|e| at(e.to_string()))?));
        }
    }
    Ok(script)
}

/// Runs the script at `path` against `program`, writing screenshots into `out_dir`.
pub fn run<P>(program: &P, path: &Path, out_dir: &Path) -> Result<(), Error>
where
    P: Program<Theme = Theme> + 'static,
{
    let source = fs::read_to_string(path).map_err(|e| Error(format!("{}: {e}", path.display())))?;
    let script = parse(&source, out_dir)?;
    fs::create_dir_all(out_dir).map_err(|e| Error(format!("{}: {e}", out_dir.display())))?;

    let (sender, mut receiver) = mpsc::channel(1);
    let mut emulator = Emulator::new(sender, program, emulator::Mode::Zen, script.viewport);
    let mut steps = script.steps.iter();

    loop {
        let event = executor::block_on(receiver.next()).ok_or_else(|| Error("emulator stopped".into()))?;
        match event {
            emulator::Event::Action(action) => emulator.perform(program, action),
            emulator::Event::Failed(instruction) => {
                let failed = out_dir.join("failed.png");
                save(&mut emulator, program, script.scale, &failed)?;
                return Err(Error(format!("instruction failed: {instruction} (screen saved to {})", failed.display())));
            }
            emulator::Event::Ready => loop {
                match steps.next() {
                    None => return Ok(()),
                    Some(Step::Screenshot(file)) => {
                        save(&mut emulator, program, script.scale, file)?;
                        println!("{}", file.display());
                    }
                    Some(Step::Run(instruction)) => {
                        emulator.run(program, instruction);
                        break;
                    }
                }
            },
        }
    }
}

fn save<P>(emulator: &mut Emulator<P>, program: &P, scale: f32, file: &Path) -> Result<(), Error>
where
    P: Program<Theme = Theme> + 'static,
{
    let theme = emulator.theme(program).unwrap_or(Theme::Dark);
    let shot = emulator.screenshot(program, &theme, scale);
    let io = |e: std::io::Error| Error(format!("{}: {e}", file.display()));
    let png_err = |e: png::EncodingError| Error(format!("{}: {e}", file.display()));
    let mut encoder = png::Encoder::new(fs::File::create(file).map_err(io)?, shot.size.width, shot.size.height);
    encoder.set_color(png::ColorType::Rgba);
    let mut writer = encoder.write_header().map_err(png_err)?;
    writer.write_image_data(&shot.rgba).map_err(png_err)?;
    writer.finish().map_err(png_err)
}
