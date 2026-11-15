//! Bowshock desktop app: the rocket builder. Layout and styling follow `design/` (the Bowshock UI kit).
// wgpu-core's auto-trait chain (Pipeline: Sync -> ... -> Hub: Send) exceeds the default depth on current nightly.
#![recursion_limit = "256"]

mod analyze;
mod app;
mod build;
mod chrome;
mod drive;
mod gallery;
mod logo;
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
       bowshock gallery [script [out-dir]]            the widget gallery";

fn main() -> std::process::ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    // `--example` loads the built-in example rocket first (also the way a drive script gets one: it cannot press Ctrl+E).
    let example = args.iter().any(|a| a == "--example");
    args.retain(|a| a != "--example");
    match args.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        [] => match app(example).run() {
            Ok(()) => std::process::ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("bowshock: {e}");
                std::process::ExitCode::FAILURE
            }
        },
        ["drive", script, rest @ ..] if rest.len() <= 1 => {
            let out = std::path::Path::new(rest.first().copied().unwrap_or("out/drive"));
            viewport::set_animation(false);
            match drive::run(&app(example), std::path::Path::new(script), out) {
                Ok(()) => std::process::ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("bowshock drive: {e}");
                    std::process::ExitCode::FAILURE
                }
            }
        }
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

/// The application; with `example` the built-in example rocket is loaded at start.
fn app(example: bool) -> iced::Application<impl iced::Program<State = App, Message = Message, Theme = Theme>> {
    iced::application(move || if example { App::with_example() } else { App::new() }, App::update, App::view)
        .title("Bowshock")
        .theme(App::theme)
        .subscription(App::subscription)
        .fonts(theme::FONTS)
        .font(SANS)
        .window(iced::window::Settings { size: Size::new(1280.0, 800.0), icon: logo::window_icon(), ..iced::window::Settings::default() })
}
