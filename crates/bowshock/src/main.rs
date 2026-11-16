// Release builds on Windows are a GUI-subsystem program (no console window); `platform::attach_console` brings the output of the
// command-line modes (`drive`, `import`, ...) back when started from a terminal.
#![cfg_attr(all(windows, not(debug_assertions)), windows_subsystem = "windows")]
//! Bowshock desktop app: the rocket builder. Layout and styling follow `design/` (the Bowshock UI kit).
// wgpu-core's auto-trait chain (Pipeline: Sync -> ... -> Hub: Send) exceeds the default depth on current nightly.
#![recursion_limit = "256"]

mod analyze;
mod app;
mod build;
mod chrome;
mod drive;
mod gallery;
mod icons;
mod logo;
mod platform;
mod simulate;
mod theme;
mod tokens;
#[allow(dead_code)]
mod units;
mod viewport;
mod widgets;

use iced::Size;
use iced::Theme;

use app::{App, Message};
use theme::SANS;

const USAGE: &str = "usage: bowshock [--example]                         open the app (optionally with the example rocket loaded)
       bowshock drive [--example] <script> [out-dir]  run a UI script headless, saving screenshots (see src/drive.rs)
       bowshock gallery [script [out-dir]]            the widget gallery
       bowshock import <file.ork> <out.bowshock>      convert an OpenRocket design (warnings on stderr)
       bowshock icons <out-dir>                       write bowshock.ico and bowshock.icns (the application icon files)";

fn main() -> std::process::ExitCode {
    if std::env::args_os().len() > 1 {
        platform::attach_console();
    }
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    // `--example` loads the built-in example rocket first (also the way a drive script gets one: it cannot press Ctrl+E).
    let example = args.iter().any(|a| a == "--example");
    args.retain(|a| a != "--example");
    match args.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        [] => match app(example, true).run() {
            Ok(()) => std::process::ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("bowshock: {e}");
                std::process::ExitCode::FAILURE
            }
        },
        ["drive", script, rest @ ..] if rest.len() <= 1 => {
            let out = std::path::Path::new(rest.first().copied().unwrap_or("out/drive"));
            viewport::set_animation(false);
            match drive::run(&app(example, false), std::path::Path::new(script), out) {
                Ok(()) => std::process::ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("bowshock drive: {e}");
                    std::process::ExitCode::FAILURE
                }
            }
        }
        ["import", ork, out] => import_ork(std::path::Path::new(ork), std::path::Path::new(out)),
        ["icons", dir] => match icons::write_all(std::path::Path::new(dir)) {
            Ok(()) => {
                println!("wrote {dir}/bowshock.ico and {dir}/bowshock.icns");
                std::process::ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("bowshock icons: {dir}: {e}");
                std::process::ExitCode::FAILURE
            }
        },
        ["gallery"] => match gallery::app().run() {
            Ok(()) => std::process::ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("bowshock gallery: {e}");
                std::process::ExitCode::FAILURE
            }
        },
        ["gallery", script, rest @ ..] if rest.len() <= 1 => {
            let out = std::path::Path::new(rest.first().copied().unwrap_or("out/gallery"));
            match drive::run(&gallery::app(), std::path::Path::new(script), out) {
                Ok(()) => std::process::ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("bowshock gallery: {e}");
                    std::process::ExitCode::FAILURE
                }
            }
        }
        _ => {
            eprintln!("{USAGE}");
            std::process::ExitCode::from(2)
        }
    }
}

/// `bowshock import`: converts an OpenRocket `.ork` into a `.bowshock` document. What could not be carried over is listed on
/// stderr, with the motors the design uses and whether the bundled database or `$BOWSHOCK_MOTORS_DIR` holds them.
fn import_ork(ork: &std::path::Path, out: &std::path::Path) -> std::process::ExitCode {
    let imp = match bowshock_model::ork::import_file(ork) {
        Ok(imp) => imp,
        Err(e) => {
            eprintln!("bowshock import: {}: {e}", ork.display());
            return std::process::ExitCode::FAILURE;
        }
    };
    if let Err(e) = std::fs::write(out, imp.rocket.to_bytes()) {
        eprintln!("bowshock import: {}: {e}", out.display());
        return std::process::ExitCode::FAILURE;
    }
    for w in &imp.warnings {
        eprintln!("warning: {w}");
    }
    let mut db = bowshock_motors::MotorDb::bundled();
    if let Some(dir) = app::physics::user_motors_dir() {
        for motor in bowshock_motors::load_cached(&dir).0 {
            db.upsert(motor);
        }
        for entry in std::fs::read_dir(dir).into_iter().flatten().flatten() {
            if entry.path().extension().is_some_and(|x| x.eq_ignore_ascii_case("eng")) {
                let _ = db.load_eng_file(entry.path());
            }
        }
    }
    for m in &imp.motors {
        let known = db.motors.iter().any(|k| k.designation.eq_ignore_ascii_case(&m.designation));
        eprintln!(
            "motor {} ({}): {}",
            m.designation,
            m.manufacturer,
            if known { "found" } else { "NOT in the motor database; add its .eng to $BOWSHOCK_MOTORS_DIR" }
        );
    }
    println!("{} -> {} ({} warnings)", ork.display(), out.display(), imp.warnings.len());
    std::process::ExitCode::SUCCESS
}

/// The application; with `example` the built-in example rocket is loaded at start, and `BOWSHOCK_OPEN=<file>` opens a document.
fn app(example: bool, interactive: bool) -> iced::Application<impl iced::Program<State = App, Message = Message, Theme = Theme>> {
    iced::application(
        move || {
            if interactive {
                platform::set_dock_icon();
            }
            App::launch(example)
        },
        App::update,
        App::view,
    )
    .title("Bowshock")
    .theme(App::theme)
    .subscription(App::subscription)
    .fonts(theme::FONTS)
    .font(SANS)
    .window(iced::window::Settings {
        size: Size::new(1280.0, 800.0),
        icon: logo::window_icon(),
        // A close request goes through the app, which asks about unsaved changes before exiting.
        exit_on_close_request: false,
        ..iced::window::Settings::default()
    })
}
