//! File > *Export 3D model…* and *Export image…*: the two dialogs, their file dialogs and the work behind them.
//!
//! The 3D model is the viewport's mesh written as OBJ or STL ([`mesh_export`]). The image is one render of the viewport's model at
//! the chosen pixel size (see [`crate::viewport::capture`]): the viewport draws it on its next frame and hands the pixels back, and
//! they are written as a PNG on a worker thread. The shared look of the dialogs ([`modal`]) is also what the Print dialog uses.
//!
//! `BOWSHOCK_EXPORT_FILE=<path>` skips the file dialog and saves there: headless `drive` runs cannot answer one.

pub mod mesh_export;

use std::path::PathBuf;

use iced::widget::{column, row, text, text_input, Space};
use iced::{Element, Fill, Task, Widget};

use crate::app::{self, Ctx, Mode};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_2, SPACE_3, TEXT_BODY};
use crate::viewport;
use crate::widgets::button::{button, Weight};
use crate::widgets::field::{checkbox, labeled};
use crate::widgets::modal::modal;
use crate::widgets::segmented::{segmented, segmented_fill};

/// Largest image side, pixels.
const MAX_SIDE: u32 = 8192;
/// Largest image, pixels (the render holds a 4× multisampled copy).
const MAX_PIXELS: u64 = 40_000_000;
/// Smallest image side, pixels.
const MIN_SIDE: u32 = 16;

/// Which dialog is open.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Export 3D model.
    Model,
    /// Export image.
    Image,
}

/// The 3D file format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// Wavefront OBJ (text, grouped by system).
    Obj,
    /// Binary STL.
    Stl,
}

/// The image size choices.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Size {
    /// 1920 × 1080.
    Hd,
    /// 2560 × 1440.
    Qhd,
    /// 3840 × 2160.
    Uhd,
    /// The two fields below.
    Custom,
}

/// The dialogs' messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// Open a dialog.
    Open(Kind),
    /// Close the open dialog (Esc).
    Close,
    /// A click inside the card: swallowed so the scrim does not take it.
    Nothing,
    /// The 3D format.
    Format(Format),
    /// Include the internal parts in the 3D model.
    Internals(bool),
    /// The image size choice.
    Size(Size),
    /// The custom width field.
    Width(String),
    /// The custom height field.
    Height(String),
    /// Leave the image background clear.
    Transparent(bool),
    /// The dialog's *Save…*: ask for a file.
    Save,
    /// The file dialog answered.
    Path(Option<PathBuf>),
    /// The viewport rendered the image: `(file, pixels)`.
    Captured(PathBuf, Result<Vec<u8>, String>),
    /// A file was written, or why not: the text for the status strip.
    Written(Result<String, String>),
}

/// The state of the two dialogs.
pub struct State {
    open: Option<Kind>,
    format: Format,
    internals: bool,
    size: Size,
    width: String,
    height: String,
    transparent: bool,
    /// An image render is under way (a second request waits).
    rendering: bool,
}

impl State {
    /// Both dialogs closed, OBJ, 3840 × 2160, opaque.
    pub fn new() -> State {
        State {
            open: None,
            format: Format::Obj,
            internals: false,
            size: Size::Uhd,
            width: "3840".to_string(),
            height: "2160".to_string(),
            transparent: false,
            rendering: false,
        }
    }

    /// Whether a dialog is up (it takes every message but the clock and the cursor).
    pub fn is_open(&self) -> bool {
        self.open.is_some()
    }

    /// The image size in pixels, or why it cannot be rendered.
    fn pixels(&self) -> Result<(u32, u32), String> {
        let (w, h) = match self.size {
            Size::Hd => (1920, 1080),
            Size::Qhd => (2560, 1440),
            Size::Uhd => (3840, 2160),
            Size::Custom => {
                let parse = |s: &str| s.trim().parse::<u32>().map_err(|_| "Width and height are whole numbers of pixels".to_string());
                (parse(&self.width)?, parse(&self.height)?)
            }
        };
        if !(MIN_SIDE..=MAX_SIDE).contains(&w) || !(MIN_SIDE..=MAX_SIDE).contains(&h) {
            return Err(format!("Each side is {MIN_SIDE} to {MAX_SIDE} pixels"));
        }
        if u64::from(w) * u64::from(h) > MAX_PIXELS {
            return Err(format!("At most {} megapixels", MAX_PIXELS / 1_000_000));
        }
        Ok((w, h))
    }

    /// Handles a message. The viewport draws the image, so it is passed in.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx, viewport: &mut viewport::View) -> Task<app::Message> {
        match message {
            Message::Open(kind) => {
                if ctx.doc.rocket().parts().is_empty() {
                    ctx.notify("Nothing to export: the rocket has no parts");
                } else if self.rendering && kind == Kind::Image {
                    ctx.notify("An image is still rendering");
                } else {
                    self.open = Some(kind);
                }
            }
            Message::Close => self.open = None,
            Message::Nothing => {}
            Message::Format(format) => self.format = format,
            Message::Internals(on) => self.internals = on,
            Message::Size(size) => self.size = size,
            Message::Width(s) => self.width = s,
            Message::Height(s) => self.height = s,
            Message::Transparent(on) => self.transparent = on,
            Message::Save => {
                let Some(kind) = self.open else { return Task::none() };
                if kind == Kind::Image && self.pixels().is_err() {
                    return Task::none();
                }
                return self.pick(kind, ctx);
            }
            Message::Path(None) => {}
            Message::Path(Some(path)) => {
                let Some(kind) = self.open.take() else { return Task::none() };
                match kind {
                    Kind::Model => {
                        let (rocket, format, internals) = (ctx.doc.rocket().clone(), self.format, self.internals);
                        return crate::app::worker::perform(
                            move || {
                                let bytes = match format {
                                    Format::Obj => mesh_export::obj(&rocket, internals).into_bytes(),
                                    Format::Stl => mesh_export::stl(&rocket, internals),
                                };
                                std::fs::write(&path, bytes)
                                    .map(|()| format!("Saved {}", file_name(&path)))
                                    .map_err(|e| format!("Could not save {}: {e}", path.display()))
                            },
                            |result| app::Message::Export(Message::Written(result.unwrap_or_else(|| Err("The export stopped".to_string())))),
                        );
                    }
                    Kind::Image => {
                        let Ok((w, h)) = self.pixels() else { return Task::none() };
                        // The viewport draws the capture, so it has to be on show.
                        ctx.mode = Mode::Build;
                        self.rendering = true;
                        let t = ctx.tokens().bg_000;
                        let pixels = viewport.start_capture(ctx, w, h, self.transparent, [t.r, t.g, t.b, 1.0]);
                        ctx.notify(format!("Rendering {w}×{h}…"));
                        return Task::perform(pixels, move |result| {
                            app::Message::Export(Message::Captured(path.clone(), result.unwrap_or_else(|_| Err("The render was cancelled".to_string()))))
                        });
                    }
                }
            }
            Message::Captured(path, result) => {
                self.rendering = false;
                let _ = viewport.update(viewport::Message::Capture(None), ctx);
                match result {
                    Ok(rgba) => {
                        let (w, h) = self.pixels().unwrap_or((0, 0));
                        return crate::app::worker::perform(
                            move || write_png(&path, w, h, &rgba),
                            |result| app::Message::Export(Message::Written(result.unwrap_or_else(|| Err("The export stopped".to_string())))),
                        );
                    }
                    Err(e) => ctx.notify(format!("Could not render the image: {e}")),
                }
            }
            Message::Written(Ok(text)) => ctx.notify(text),
            Message::Written(Err(e)) => ctx.notify(e),
        }
        Task::none()
    }

    /// The file dialog for `kind`: answers [`Message::Path`] (at once, with no dialog, under `BOWSHOCK_EXPORT_FILE`).
    fn pick(&self, kind: Kind, ctx: &Ctx) -> Task<app::Message> {
        let base = ctx.doc.title();
        let (title, name, filter) = match (kind, self.format) {
            (Kind::Model, Format::Obj) => ("Export 3D model", format!("{base}.obj"), ("OBJ", &["obj"][..])),
            (Kind::Model, Format::Stl) => ("Export 3D model", format!("{base}.stl"), ("STL", &["stl"][..])),
            (Kind::Image, _) => ("Export image", format!("{base}.png"), ("PNG image", &["png"][..])),
        };
        crate::app::dialogs::save_file(ctx.env.export_file.clone(), title, name, filter, |path| app::Message::Export(Message::Path(path)))
    }

    /// The open dialog over a dimmed backdrop, `None` while both are closed.
    pub fn view(&self) -> Option<Element<'_, app::Message>> {
        let kind = self.open?;
        let (title, body, save): (&str, Element<'_, Message>, Option<Message>) = match kind {
            Kind::Model => ("Export 3D model", self.model_body(), Some(Message::Save)),
            Kind::Image => ("Export image", self.image_body(), self.pixels().is_ok().then_some(Message::Save)),
        };
        let cancel: Element<'_, Message> = button("Cancel", Weight::Secondary).key("Esc").on_press(Message::Close).into();
        let save_button: Element<'_, Message> = match save {
            Some(message) => button("Save…", Weight::Primary).key("↵").on_press(message).into(),
            None => button("Save…", Weight::Primary).key("↵").disabled("Fix the image size first").into(),
        };
        let buttons = row![Space::new().width(Fill), cancel, save_button].spacing(SPACE_2).align_y(iced::Center).boxed();
        Some(modal(title, body, buttons, 440.0, Message::Nothing, Message::Nothing).map(app::Message::Export).boxed())
    }

    fn model_body(&self) -> Element<'_, Message> {
        let format = segmented(&[(Format::Obj, "OBJ"), (Format::Stl, "STL")], self.format, Message::Format);
        column![
            labeled("Format", format),
            checkbox("Include internal parts", self.internals, Message::Internals),
            text("Millimetres, nose along +Z. The airframe is the shell with its wall; fins, lugs and buttons are separate solids.")
                .size(TEXT_BODY)
                .style(theme::muted)
                .boxed(),
        ]
        .spacing(SPACE_3)
        .boxed()
    }

    fn image_body(&self) -> Element<'_, Message> {
        let presets =
            segmented_fill(&[(Size::Hd, "1920×1080"), (Size::Qhd, "2560×1440"), (Size::Uhd, "3840×2160"), (Size::Custom, "Custom")], self.size, Message::Size);
        fn field(value: &str, on_input: fn(String) -> Message) -> Element<'_, Message> {
            text_input("", value).on_input(on_input).font(MONO).size(TEXT_BODY).padding([4.0, SPACE_2]).style(theme::input).width(Fill).boxed()
        }
        let mut body = column![labeled("Size", presets)].spacing(SPACE_3);
        if self.size == Size::Custom {
            body = body.push(
                row![labeled("Width", field(&self.width, Message::Width)), labeled("Height", field(&self.height, Message::Height))].spacing(SPACE_3).boxed(),
            );
        }
        body = body.push(checkbox("Transparent background", self.transparent, Message::Transparent));
        let note = match self.pixels() {
            Ok((w, h)) => format!("{w} × {h} px, framed as the viewport shows the model now. The body only: no CG / CP markers or labels."),
            Err(e) => e,
        };
        body.push(text(note).size(TEXT_BODY).style(theme::muted).boxed()).boxed()
    }
}

/// The file name of `path` for the status strip.
pub fn file_name(path: &std::path::Path) -> String {
    path.file_name().map_or_else(|| path.display().to_string(), |n| n.to_string_lossy().into_owned())
}

/// Writes straight RGBA8 `rgba` (`width` × `height`) as a PNG.
fn write_png(path: &std::path::Path, width: u32, height: u32, rgba: &[u8]) -> Result<String, String> {
    let fail = |e: &dyn std::fmt::Display| format!("Could not save {}: {e}", path.display());
    let file = std::fs::File::create(path).map_err(|e| fail(&e))?;
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(file), width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(|e| fail(&e))?;
    writer.write_image_data(rgba).map_err(|e| fail(&e))?;
    writer.finish().map_err(|e| fail(&e))?;
    Ok(format!("Saved {} ({width}×{height})", file_name(path)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn image_sizes_are_checked() {
        let mut s = State::new();
        assert_eq!(s.pixels(), Ok((3840, 2160)));
        s.size = Size::Custom;
        s.width = "9000".into();
        assert!(s.pixels().is_err());
        s.width = "8192".into();
        s.height = "8192".into();
        assert!(s.pixels().unwrap_err().contains("megapixels"));
        s.height = "x".into();
        assert!(s.pixels().is_err());
        s.width = "1000".into();
        s.height = "500".into();
        assert_eq!(s.pixels(), Ok((1000, 500)));
    }

    #[test]
    fn a_png_round_trips() {
        let path = std::env::temp_dir().join(format!("bowshock-{}-export.png", std::process::id()));
        let rgba: Vec<u8> = (0..4 * 3 * 2).map(|i| i as u8).collect();
        write_png(&path, 3, 2, &rgba).unwrap();
        let decoder = png::Decoder::new(std::io::BufReader::new(std::fs::File::open(&path).unwrap()));
        let mut reader = decoder.read_info().unwrap();
        let mut buffer = vec![0; reader.output_buffer_size().unwrap()];
        let info = reader.next_frame(&mut buffer).unwrap();
        assert_eq!((info.width, info.height), (3, 2));
        assert_eq!(&buffer[..24], &rgba[..]);
        let _ = std::fs::remove_file(path);
    }
}
