//! `bowshock gallery <script> [out-dir]`: every kit widget in dark and light with live state, so the widgets can be
//! looked at and compared with `design/components/*/<Name>-{dark,light}.png`:
//!
//! ```text
//! cargo run -q -p bowshock -- gallery crates/bowshock/scripts/gallery.drive out/gallery
//! ```

use std::time::Instant;

use iced::widget::{column, container, row, scrollable, slider, text, Space};
use iced::{Element, Fill, Size, Theme, Widget};

use crate::theme::{self, MONO};
use crate::tokens::{SPACE_2, SPACE_3, SPACE_4, TEXT_BODY, TEXT_LABEL};
use crate::units::{Quantity, UnitSystem};
use crate::widgets::button::{button, icon_button, Weight};
use crate::widgets::chip::{chip, Severity};
use crate::widgets::field::{self, checkbox, labeled, section, select};
use crate::widgets::gauge::{self, Bands, GaugeState};
use crate::widgets::glyph::{self, Mark, Tone};
use crate::widgets::icons::{rail_button, RailIcon};
use crate::widgets::plot::demo::{Demo, DemoMessage};
use crate::widgets::segmented::{segmented, segmented_fill};
use crate::widgets::status;
use crate::widgets::unit_input::{unit_input, Badge, State};

/// The gallery program, for `drive::run` and `iced::run`.
pub fn app() -> iced::Application<impl iced::Program<State = Gallery, Message = Message, Theme = Theme>> {
    iced::application(Gallery::new, Gallery::update, Gallery::view)
        .title("Bowshock widget gallery")
        .theme(Gallery::theme)
        .subscription(Gallery::subscription)
        .fonts(theme::FONTS)
        .font(theme::SANS)
        .window_size(Size::new(1280.0, 900.0))
}

/// Gallery messages.
#[derive(Debug, Clone)]
pub enum Message {
    ToggleTheme,
    Units(UnitSystem),
    RootChord(f64),
    TipChord(f64),
    Mass(f64),
    Angle(f64),
    Fins(u8),
    Shape(&'static str),
    Mode(&'static str),
    Rail(RailIcon),
    Margin(f32),
    Frame(Instant),
    Check(bool),
    Pick(Option<&'static str>),
    Noop,
    Plots(DemoMessage),
    Page(bool),
}

/// The gallery state.
pub struct Gallery {
    dark: bool,
    units: UnitSystem,
    root_chord: f64,
    tip_chord: f64,
    mass: f64,
    angle: f64,
    fins: u8,
    shape: &'static str,
    mode: &'static str,
    rail: RailIcon,
    margin: f32,
    gauge: GaugeState,
    checked: bool,
    pick: Option<&'static str>,
    plots: Demo,
    show_plots: bool,
}

const SHAPES: [(&str, &str); 3] = [("Square", "Square"), ("Rounded", "Rounded"), ("Airfoil", "Airfoil")];
const FINS: [(u8, &str); 5] = [(3, "3"), (4, "4"), (5, "5"), (6, "6"), (8, "8")];
const MODES: [(&str, &str); 3] = [("Build", "Build"), ("Simulate", "Simulate"), ("Analyze", "Analyze")];
const PICKS: [&str; 3] = ["Trapezoidal", "Elliptical", "Freeform"];

impl Gallery {
    fn new() -> Self {
        Self {
            dark: true,
            units: UnitSystem::MmGS,
            root_chord: 0.120,
            tip_chord: 0.050,
            mass: 0.0482,
            angle: std::f64::consts::FRAC_PI_6,
            fins: 3,
            shape: "Rounded",
            mode: "Build",
            rail: RailIcon::FinsGuides,
            margin: 1.92,
            gauge: GaugeState::new(1.92),
            checked: true,
            pick: Some("Trapezoidal"),
            plots: Demo::default(),
            show_plots: false,
        }
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::ToggleTheme => self.dark = !self.dark,
            Message::Units(u) => self.units = u,
            Message::RootChord(v) => self.root_chord = v,
            Message::TipChord(v) => self.tip_chord = v,
            Message::Mass(v) => self.mass = v,
            Message::Angle(v) => self.angle = v,
            Message::Fins(n) => self.fins = n,
            Message::Shape(s) => self.shape = s,
            Message::Mode(s) => self.mode = s,
            Message::Rail(r) => self.rail = r,
            Message::Margin(m) => {
                self.margin = m;
                self.gauge.set(m);
            }
            Message::Frame(now) => self.gauge.tick(now),
            Message::Check(c) => self.checked = c,
            Message::Pick(p) => self.pick = p,
            Message::Noop => {}
            Message::Plots(m) => self.plots.update(m),
            Message::Page(p) => self.show_plots = p,
        }
    }

    fn theme(&self) -> Theme {
        if self.dark {
            theme::dark()
        } else {
            theme::light()
        }
    }

    fn subscription(&self) -> iced::Subscription<Message> {
        if self.gauge.moving() {
            iced::window::frames().map(Message::Frame)
        } else {
            iced::Subscription::none()
        }
    }

    fn heading(title: &str) -> Element<'_, Message> {
        text(title.to_uppercase()).font(MONO).size(TEXT_LABEL).style(theme::faint).boxed()
    }

    fn view(&self) -> Element<'_, Message> {
        let theme_toggle: Element<'_, Message> = button(if self.dark { "Light" } else { "Dark" }, Weight::Secondary).on_press(Message::ToggleTheme).into();
        let units = select(UnitSystem::ALL.map(|u| u.label()).to_vec(), Some(self.units.label()), |label| {
            Message::Units(UnitSystem::ALL.into_iter().find(|u| u.label() == label).unwrap_or_default())
        });
        let header = row![
            text("WIDGET GALLERY").font(MONO).size(TEXT_BODY).style(theme::ink),
            Space::new().width(Fill),
            container(units).width(140),
            segmented(&[(false, "Widgets"), (true, "Plots")], self.show_plots, Message::Page),
            theme_toggle
        ]
        .spacing(SPACE_3)
        .align_y(iced::Center);

        let columns = row![self.fields_column(), self.controls_column(), self.readouts_column()].spacing(SPACE_4);

        let strip = status::strip(
            vec![
                status::config("H128W", false, Some(Message::Noop)),
                status::readout("MASS", "1.284 kg", Some(Message::Noop)),
                status::readout("LEN", "1142.0 mm", Some(Message::Noop)),
                status::readout("Ø REF", "76.2 mm", Some(Message::Noop)),
                status::cg("712.4", Some(Message::Noop)),
                status::cp("858.7", Some(Message::Noop)),
                status::stability(&self.gauge, &Bands::default(), Some(Message::Noop)),
            ],
            vec![chip(Severity::Warn, "1 warning").on_press(Message::Noop).into()],
        );

        let body: Element<'_, Message> =
            if self.show_plots { self.plots.view().map(Message::Plots).boxed() } else { scrollable(container(columns).width(Fill)).height(Fill).boxed() };
        container(column![container(column![header, body].spacing(SPACE_3)).padding(SPACE_4).height(Fill), strip])
            .style(|t: &Theme| container::Style {
                background: Some(theme::tokens(t).bg_100.into()),
                text_color: Some(theme::tokens(t).ink),
                ..container::Style::default()
            })
            .boxed()
    }

    /// Inspector-style panel with UnitInputs in every state.
    fn fields_column(&self) -> Element<'_, Message> {
        let u = &self.units;
        let inputs: Vec<Element<'_, Message>> = vec![
            unit_input("Root chord", self.root_chord, Quantity::Length, u).on_scrub(Message::RootChord).on_commit(Message::RootChord).into(),
            unit_input("Tip chord", self.tip_chord, Quantity::Length, u).on_scrub(Message::TipChord).on_commit(Message::TipChord).into(),
            unit_input("Diameter", 0.0762, Quantity::Length, u).state(State::Auto).hint("Matched to Body tube. Type to override.").into(),
            unit_input("Mass", self.mass, Quantity::Mass, u).state(State::Override).on_commit(Message::Mass).into(),
            unit_input("Root chord", 0.1524, Quantity::Length, u).state(State::Formula).formula("2 * {body_tube.od}").hint("= 2 × [Body tube · Ø]").into(),
            unit_input("Tip chord", 0.05, Quantity::Length, u)
                .state(State::Error("Circular: Root chord already uses Tip chord. The last valid value (50.00) is kept.".into()))
                .into(),
            unit_input("Outer Ø", 0.0762, Quantity::Length, u).badge(Badge::Refs(2)).into(),
            unit_input("Sweep", self.angle, Quantity::Angle, u).on_scrub(Message::Angle).on_commit(Message::Angle).into(),
            unit_input("Thickness", 0.00318, Quantity::Length, u).state(State::PickTarget).into(),
            unit_input("Height", 0.0381, Quantity::Length, u).state(State::Picking).into(),
            unit_input("Motor delay", 4.0, Quantity::Time, u).badge(Badge::PerConfig).into(),
        ];
        let geometry = section(1, "Geometry · Fin set", false, inputs);
        let selects = section(
            2,
            "Position",
            true,
            vec![
                labeled("Fin count", segmented(&FINS, self.fins, Message::Fins)),
                labeled("Cross-section", segmented_fill(&SHAPES, self.shape, Message::Shape)),
                labeled("Fin set", select(PICKS.to_vec(), self.pick, |p| Message::Pick(Some(p)))),
                labeled("Linked", checkbox("Edit all fins", self.checked, Message::Check)),
            ],
        );
        column![Self::heading("UnitInput · Inspector"), container(column![geometry, selects]).width(304).style(|t: &Theme| theme::panel(t)),]
            .spacing(SPACE_2)
            .boxed()
    }

    fn controls_column(&self) -> Element<'_, Message> {
        let main_buttons: Vec<Element<'_, Message>> = vec![
            button("Simulate", Weight::Primary).icon(Mark::Play).key("F5").on_press(Message::Noop).into(),
            button("Export…", Weight::Secondary).on_press(Message::Noop).into(),
            button("Disabled", Weight::Secondary).disabled("Run a simulation first").into(),
        ];
        let ghosts: Vec<Element<'_, Message>> = vec![
            icon_button(Mark::Undo, "Undo").shortcut("Ctrl+Z").on_press(Message::Noop).into(),
            icon_button(Mark::Redo, "Redo").shortcut("Ctrl+Shift+Z").on_press(Message::Noop).into(),
            icon_button(Mark::Ellipsis, "More").on_press(Message::Noop).into(),
            button("Add stage", Weight::Ghost).icon(Mark::Plus).on_press(Message::Noop).into(),
        ];
        let chips: Vec<Element<'_, Message>> = vec![
            chip(Severity::Ok, "Stable · 1.92 cal").into(),
            chip(Severity::Warn, "Diameter mismatch · 1.6 mm").into(),
            chip(Severity::Err, "Unstable · 0.31 cal (min 1.0)").into(),
            chip(Severity::Info, "Mach 0.3 · AoA 0°").into(),
        ];
        let glyphs = row![
            glyph::cg(14.0),
            text("CG").font(MONO).size(TEXT_LABEL).style(theme::colored(|t| t.cg)),
            glyph::cp(14.0),
            text("CP").font(MONO).size(TEXT_LABEL).style(theme::colored(|t| t.cp)),
            glyph::icon(Mark::Check, Tone::Ok, 14.0),
            glyph::icon(Mark::Warning, Tone::Warn, 14.0),
            glyph::icon(Mark::Error, Tone::Err, 14.0),
            glyph::icon(Mark::CaretUp, Tone::Muted, 14.0),
            glyph::icon(Mark::Close, Tone::Muted, 14.0),
            glyph::cg(28.0),
            glyph::cp(28.0),
        ]
        .spacing(SPACE_2)
        .align_y(iced::Center);
        let rail = row(RailIcon::ALL.map(|r| rail_button(r, r == self.rail, Message::Rail(r)))).spacing(0);

        column![
            Self::heading("Button"),
            row(main_buttons).spacing(SPACE_2),
            row(ghosts).spacing(SPACE_2).align_y(iced::Center),
            Self::heading("SegmentedControl"),
            segmented(&MODES, self.mode, Message::Mode),
            segmented_fill(&FINS, self.fins, Message::Fins),
            Self::heading("WarningChip"),
            column(chips).spacing(SPACE_2),
            Self::heading("Glyphs"),
            glyphs,
            Self::heading("Rail icons (PartsBin)"),
            container(rail).style(|t: &Theme| theme::panel(t)),
            // PLOT DEMO: the Plot agent adds its section here.
        ]
        .spacing(SPACE_2)
        .width(Fill)
        .boxed()
    }

    fn readouts_column(&self) -> Element<'_, Message> {
        let bands = Bands::default();
        let samples: Vec<Element<'_, Message>> = [(1.92_f32, "146.3 mm"), (0.74, "56.4 mm"), (0.31, "23.6 mm")]
            .into_iter()
            .map(|(m, length)| gauge::full(&GaugeState::new(m), &bands, Some(length.into())))
            .collect();
        column![
            Self::heading("StabilityGauge · live"),
            gauge::full(&self.gauge, &bands, Some(format!("{:.1} mm", self.margin * 76.2))),
            row![
                slider(0.0..=3.5, self.margin, Message::Margin).step(0.01f32).width(Fill),
                text(format!("{:.2}", self.margin)).font(MONO).size(TEXT_BODY).style(theme::ink),
            ]
            .spacing(SPACE_2)
            .align_y(iced::Center),
            row![text("compact").font(MONO).size(TEXT_LABEL).style(theme::faint), gauge::compact(&self.gauge, &bands)].spacing(SPACE_2).align_y(iced::Center),
            Self::heading("StabilityGauge · bands"),
            column(samples).spacing(SPACE_2),
            Self::heading("Hairline / section"),
            field::rule(),
        ]
        .spacing(SPACE_2)
        .width(Fill)
        .boxed()
    }
}
