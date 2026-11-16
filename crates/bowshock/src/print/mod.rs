//! File > *Print / export PDF…*: the dialog and the build of the PDF. The design report and the parts list ([`report`]) and the
//! full-size templates ([`templates`]) are drawn with a small writer ([`pdf`]). Building runs on a worker thread, because the
//! report flies every flight of the document fresh.
//!
//! `BOWSHOCK_EXPORT_FILE=<path>` skips the file dialog (headless `drive` runs cannot answer one).

pub mod pdf;
pub mod report;
pub mod templates;

use std::path::PathBuf;

use bowshock_model::Rocket;
use iced::widget::{column, row, text, Space};
use iced::{Element, Fill, Task, Widget};

use crate::app::{self, Ctx};
use crate::export::{file_name, modal, off_thread};
use crate::theme;
use crate::tokens::{SPACE_2, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::button::{button, Weight};
use crate::widgets::field::{checkbox, labeled};
use crate::widgets::segmented::segmented;
use crate::widgets::spinner::spinner;
use pdf::{PageSize, Pdf};

/// What goes into the file.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Options {
    /// The design report.
    pub report: bool,
    /// The parts list.
    pub parts: bool,
    /// Fin templates.
    pub fins: bool,
    /// Centering rings and bulkheads.
    pub rings: bool,
    /// Flat patterns of cones and transitions.
    pub wraps: bool,
    /// The fin marking guide.
    pub marking: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options { report: true, parts: true, fins: true, rings: true, wraps: true, marking: true }
    }
}

/// One checkbox of the dialog.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Section {
    /// Design report.
    Report,
    /// Parts list.
    Parts,
    /// Fin templates.
    Fins,
    /// Rings and bulkheads.
    Rings,
    /// Cone and transition wraps.
    Wraps,
    /// Fin marking guide.
    Marking,
}

/// The dialog's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// Open the dialog.
    Open,
    /// Close it (Esc).
    Close,
    /// A click inside the card.
    Nothing,
    /// A checkbox.
    Toggle(Section, bool),
    /// The paper size.
    Size(PageSize),
    /// *Save PDF…*.
    Save,
    /// The file dialog answered.
    Path(Option<PathBuf>),
    /// The worker finished: the text for the status strip, or why not.
    Built(Result<String, String>),
}

/// The dialog's state.
pub struct State {
    open: bool,
    options: Options,
    size: PageSize,
    working: bool,
}

impl State {
    /// Closed, everything on, Letter.
    pub fn new() -> State {
        State { open: false, options: Options::default(), size: PageSize::Letter, working: false }
    }

    /// Whether the dialog is up.
    pub fn is_open(&self) -> bool {
        self.open
    }

    /// Whether a PDF is being built (the spinner needs frames).
    pub fn animating(&self) -> bool {
        self.working
    }

    /// Handles a message.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Open => {
                if ctx.doc.rocket().parts().is_empty() {
                    ctx.notify("Nothing to print: the rocket has no parts");
                } else if self.working {
                    ctx.notify("A PDF is still being built");
                } else {
                    // Letter where the length unit is imperial, A4 elsewhere.
                    let imperial = matches!(Quantity::Length.unit_for(ctx.units).symbol, "in" | "ft");
                    self.size = if imperial { PageSize::Letter } else { PageSize::A4 };
                    self.open = true;
                }
            }
            Message::Close => self.open = false,
            Message::Nothing => {}
            Message::Toggle(section, on) => match section {
                Section::Report => self.options.report = on,
                Section::Parts => self.options.parts = on,
                Section::Fins => self.options.fins = on,
                Section::Rings => self.options.rings = on,
                Section::Wraps => self.options.wraps = on,
                Section::Marking => self.options.marking = on,
            },
            Message::Size(size) => self.size = size,
            Message::Save => {
                if self.options == (Options { report: false, parts: false, fins: false, rings: false, wraps: false, marking: false }) {
                    return Task::none();
                }
                if let Some(path) = std::env::var_os("BOWSHOCK_EXPORT_FILE").filter(|p| !p.is_empty()) {
                    return self.update(Message::Path(Some(PathBuf::from(path))), ctx);
                }
                let name = format!("{}.pdf", ctx.doc.title());
                return Task::perform(
                    async move {
                        let file =
                            rfd::AsyncFileDialog::new().set_title("Print / export PDF").set_file_name(name).add_filter("PDF", &["pdf"]).save_file().await;
                        file.map(|f| f.path().to_path_buf())
                    },
                    |path| app::Message::Print(Message::Path(path)),
                );
            }
            Message::Path(None) => {}
            Message::Path(Some(path)) => {
                self.working = true;
                let (rocket, options, size) = (ctx.doc.rocket().clone(), self.options, self.size);
                let inputs = report::gather(ctx, options.report);
                let title = ctx.doc.title();
                ctx.notify("Building the PDF…");
                return Task::perform(
                    off_thread(move || {
                        let flown = options.report.then(|| report::Flown::fly(&inputs.flights));
                        let pdf = build(&rocket, &inputs, flown.as_ref(), options, size, &title);
                        let pages = pdf.page_count();
                        if pages == 0 {
                            return Err("There is nothing to print for these choices".to_string());
                        }
                        std::fs::write(&path, pdf.to_bytes())
                            .map(|()| format!("Saved {} ({pages} {})", file_name(&path), if pages == 1 { "page" } else { "pages" }))
                            .map_err(|e| format!("Could not save {}: {e}", path.display()))
                    }),
                    |result| app::Message::Print(Message::Built(result.unwrap_or_else(|| Err("The PDF build stopped".to_string())))),
                );
            }
            Message::Built(result) => {
                self.working = false;
                self.open = false;
                ctx.notify(result.unwrap_or_else(|e| e));
            }
        }
        Task::none()
    }

    /// The dialog over a dimmed backdrop, `None` while closed.
    pub fn view(&self) -> Option<Element<'_, app::Message>> {
        if !self.open {
            return None;
        }
        let o = self.options;
        let box_ = |label: &'static str, on: bool, s: Section| checkbox(label, on, move |v| Message::Toggle(s, v));
        let sizes = segmented(&[(PageSize::Letter, "Letter"), (PageSize::A4, "A4")], self.size, Message::Size);
        let body = column![
            box_("Design report", o.report, Section::Report),
            box_("Parts list", o.parts, Section::Parts),
            box_("Fin templates", o.fins, Section::Fins),
            box_("Rings & bulkheads", o.rings, Section::Rings),
            box_("Cone & transition wraps", o.wraps, Section::Wraps),
            box_("Fin marking guide", o.marking, Section::Marking),
            labeled("Paper", sizes),
            text("Templates print at full size: set the printer to 100 % (no scaling). Pages with a 25 mm and a 1 in square check it.")
                .size(TEXT_BODY)
                .style(theme::muted)
                .boxed(),
        ]
        .spacing(SPACE_2 + 2.0)
        .boxed();
        let none = o == (Options { report: false, parts: false, fins: false, rings: false, wraps: false, marking: false });
        let save = button("Save PDF…", Weight::Primary).key("↵");
        let save: Element<'_, Message> = if none {
            save.disabled("Choose something to print").into()
        } else if self.working {
            save.disabled("A PDF is being built").into()
        } else {
            save.on_press(Message::Save).into()
        };
        let working: Element<'_, Message> = if self.working {
            row![spinner(14.0), text("Building…").size(TEXT_BODY)].spacing(SPACE_2).align_y(iced::Center).boxed()
        } else {
            Space::new().boxed()
        };
        let cancel: Element<'_, Message> = button("Cancel", Weight::Secondary).key("Esc").on_press(Message::Close).into();
        let buttons = row![working, Space::new().width(Fill), cancel, save].spacing(SPACE_2).align_y(iced::Center).boxed();
        Some(modal("Print / export PDF", body, buttons, 440.0, Message::Nothing).map(app::Message::Print).boxed())
    }
}

/// The whole PDF for `rocket`: the chosen sections in order (report, parts list, fins, rings, wraps, marking guide).
pub fn build(rocket: &Rocket, inputs: &report::Inputs, flown: Option<&report::Flown>, options: Options, size: PageSize, title: &str) -> Pdf {
    let units = inputs.units;
    let name = if rocket.name.trim().is_empty() { title.to_string() } else { rocket.name.trim().to_string() };
    let mut pdf = Pdf::new(&name);
    if options.report {
        report::design_report(&mut pdf, rocket, inputs, flown, size);
    }
    if options.parts {
        report::parts_list(&mut pdf, rocket, units, size);
    }
    if options.fins {
        templates::emit(&mut pdf, size, &format!("{name} - Fin templates"), &templates::fin_drawings(rocket, units));
    }
    if options.rings {
        templates::emit(&mut pdf, size, &format!("{name} - Rings and bulkheads"), &templates::ring_drawings(rocket, units));
    }
    if options.wraps {
        let (mut drawings, notes) = templates::wrap_drawings(rocket, units);
        drawings.extend(templates::notes_drawing(&notes));
        templates::emit(&mut pdf, size, &format!("{name} - Cone and transition wraps"), &drawings);
    }
    if options.marking {
        templates::emit(&mut pdf, size, &format!("{name} - Fin marking guide"), &templates::marking_drawings(rocket, units));
    }
    pdf
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::UnitSystem;

    fn example() -> Rocket {
        Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).expect("the example loads")
    }

    #[test]
    fn the_example_prints_every_section() {
        let rocket = example();
        let inputs = report::Inputs { units: UnitSystem::MM_G_S, balance: Err("none".into()), configs: Vec::new(), warnings: Vec::new(), flights: Vec::new() };
        let all = build(&rocket, &inputs, None, Options::default(), PageSize::Letter, "x");
        let report_only = build(
            &rocket,
            &inputs,
            None,
            Options { parts: false, fins: false, rings: false, wraps: false, marking: false, ..Options::default() },
            PageSize::A4,
            "x",
        );
        assert_eq!(report_only.page_count(), 2);
        assert!(all.page_count() > report_only.page_count());
        let bytes = all.to_bytes();
        assert!(bytes.starts_with(b"%PDF-1.4") && bytes.ends_with(b"%%EOF"));
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("Fin templates") && text.contains("Fin marking guide"));
    }
}
