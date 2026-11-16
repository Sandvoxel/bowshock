//! Synthetic data and a self-contained showcase of the plot widgets for the gallery: a Simulate flight with its events
//! strip and ground track, a three-flight comparison, an Analyze sweep over Mach (static margin, CP/CG, stacked Cd)
//! and a dispersion ground track.
//!
//! The flights come from a tiny 1-D point-mass model (thrust, quadratic drag, drogue and main descent), so the curves
//! have the right shapes and magnitudes; they are not a simulation of anything.
//!
//! ```ignore
//! // state
//! showcase: plot::demo::Showcase,
//! // update
//! Message::Plots(m) => self.showcase.update(m),
//! // view (one of the sections)
//! self.showcase.view(Section::Simulate).map(Message::Plots)
//! ```

use iced::widget::{column, container, row, text};
use iced::{Element, Length, Widget};

use super::{
    header, legend, Axis, EventMark, EventRole, HLine, LabelAt, LineStyle, Panel, Phase, PlotGroup, PlotMessage, PlotState, PlotView, Series, SeriesColor, Tag,
};
use crate::theme::{self, MONO};
use crate::tokens;
use crate::widgets::ground_track::{GroundMessage, GroundTrack, GroundTrackView, Track, TrackPoint, Wind};

const G: f64 = 9.81;

/// A flight of the demo model.
#[derive(Debug, Clone, Copy)]
struct Motor {
    /// Initial thrust acceleration, m/s².
    accel: f64,
    /// Burn time, s.
    burn: f64,
    /// Quadratic drag coefficient k in a = −k v |v|, 1/m.
    drag: f64,
    /// Where it lands: distance (m) and bearing (°) from the pad.
    landing: (f64, f64),
}

/// The time series of a simulated flight.
struct Flight {
    t: Vec<f64>,
    alt: Vec<f64>,
    vel: Vec<f64>,
    acc_g: Vec<f64>,
    rail_exit: f64,
    burnout: f64,
    apogee: f64,
    drogue: f64,
    main: f64,
    landing: f64,
}

fn simulate(m: Motor) -> Flight {
    let dt = 0.01;
    let (mut t, mut alt, mut v) = (0.0f64, 0.0f64, 0.0f64);
    let mut f =
        Flight { t: vec![], alt: vec![], vel: vec![], acc_g: vec![], rail_exit: 0.0, burnout: m.burn, apogee: 0.0, drogue: 0.0, main: 0.0, landing: 0.0 };
    let mut phase = 0; // 0 ascent, 1 drogue, 2 main
    let mut step = 0;
    let mut rail_set = false;
    while t < 200.0 {
        let thrust = if t < m.burn { m.accel * (1.0 - 0.25 * t / m.burn) } else { 0.0 };
        let a = match phase {
            0 | 3 => thrust - G - m.drag * v * v.abs(),
            1 => -G + G * (v / 22.0).powi(2),
            _ => -G + G * (v / 5.0).powi(2),
        };
        if step % 10 == 0 {
            f.t.push(t);
            f.alt.push(alt);
            f.vel.push(v);
            f.acc_g.push(a / G);
        }
        v += a * dt;
        alt += v * dt;
        t += dt;
        step += 1;
        if !rail_set && alt > 1.52 {
            f.rail_exit = t;
            rail_set = true;
        }
        if phase == 0 && v < 0.0 && t > m.burn {
            f.apogee = t;
            phase = 3; // coasting over the top until the drogue fires
        }
        if phase == 3 && t >= f.apogee + 0.8 {
            f.drogue = t;
            phase = 1;
        }
        if phase == 1 && alt < 150.0 {
            f.main = t;
            phase = 2;
        }
        if alt <= 0.0 && t > m.burn {
            f.landing = t;
            break;
        }
    }
    f
}

impl Flight {
    fn series(&self, name: &str, ys: &[f64], color: SeriesColor) -> Series {
        Series::new(name, self.t.iter().copied().zip(ys.iter().copied()).collect()).color(color)
    }

    fn events(&self) -> Vec<EventMark> {
        vec![
            EventMark::new(self.rail_exit, "Rail exit").role(EventRole::RailExit),
            EventMark::new(self.burnout, "Burnout").role(EventRole::Burnout),
            EventMark::new(self.apogee, "Apogee").role(EventRole::Apogee),
            EventMark::new(self.drogue, "Drogue").role(EventRole::Deploy),
            EventMark::new(self.main, "Main").role(EventRole::Deploy),
            EventMark::new(self.landing, "Landing").role(EventRole::Landing),
        ]
    }

    /// The ground track: leans into the wind on the way up, drifts downwind on the way down.
    fn track(&self, color: SeriesColor, landing: (f64, f64)) -> Track {
        let (range, bearing) = landing;
        let b = bearing.to_radians();
        let apogee = self.alt.iter().copied().fold(0.0, f64::max);
        let lean = 0.18 * range;
        let points = self
            .t
            .iter()
            .zip(&self.alt)
            .map(|(&t, &alt)| {
                let along = if t <= self.apogee {
                    -lean * (alt / apogee).clamp(0.0, 1.0)
                } else {
                    let k = ((t - self.apogee) / (self.landing - self.apogee)).clamp(0.0, 1.0);
                    -lean + (range + lean) * (k * k * (3.0 - 2.0 * k))
                };
                let across = 0.05 * range * (std::f64::consts::PI * t / self.landing).sin();
                TrackPoint { t, east: along * b.sin() + across * b.cos(), north: along * b.cos() - across * b.sin(), alt }
            })
            .collect();
        Track::new(color, points)
    }
}

const NOMINAL: Motor = Motor { accel: 108.0, burn: 1.6, drag: 0.0008, landing: (104.0, 68.0) };
const WINDY: Motor = Motor { accel: 108.0, burn: 1.6, drag: 0.00093, landing: (337.0, 67.0) };
const BIGGER: Motor = Motor { accel: 150.0, burn: 1.9, drag: 0.0008, landing: (158.0, 70.0) };

/// Simulate: altitude, vertical velocity and acceleration against time, with the events strip and the flight's events.
pub fn simulate_group() -> PlotGroup {
    let f = simulate(NOMINAL);
    let mut g = PlotGroup::new(Axis::time())
        .panel(Panel::new("Altitude", "m").weight(1.2).zero_based().with(f.series("Altitude", &f.alt, SeriesColor::Ink)))
        .panel(Panel::new("Vertical velocity", "m/s").with(f.series("Velocity", &f.vel, SeriesColor::Ink)))
        .panel(Panel::new("Acceleration", "g").with(f.series("Acceleration", &f.acc_g, SeriesColor::Ink)));
    g.events = f.events();
    g.phases = vec![Phase { from: 0.0, to: f.burnout, color: SeriesColor::Propulsion }, Phase { from: f.drogue, to: f.landing, color: SeriesColor::Recovery }];
    g.timeline = true;
    g
}

/// SimulateCompare: three flights on one axis, labelled at their right ends, reference events only.
pub fn compare_group() -> PlotGroup {
    let flights = [("Nominal", NOMINAL, 1u8), ("Windy", WINDY, 2), ("Bigger motor", BIGGER, 3)];
    let sims: Vec<Flight> = flights.iter().map(|(_, m, _)| simulate(*m)).collect();
    let mut alt = Panel::new("Altitude", "m").weight(1.2).zero_based();
    let mut vel = Panel::new("Vertical velocity", "m/s").tag(Tag::Off);
    let mut acc = Panel::new("Acceleration", "g").tag(Tag::Off);
    for ((name, _, slot), f) in flights.iter().zip(&sims) {
        let c = SeriesColor::Series(*slot);
        alt = alt.with(f.series(name, &f.alt, c).label_margin());
        vel = vel.with(f.series(name, &f.vel, c));
        acc = acc.with(f.series(name, &f.acc_g, c));
    }
    let mut g = PlotGroup::new(Axis::time()).panel(alt).panel(vel).panel(acc);
    g.events = sims[0].events().into_iter().filter(|e| e.label != "Landing" && e.label != "Main").collect();
    g.right_margin = 96.0;
    g.x_range = Some((0.0, 75.0));
    g
}

/// Ground tracks of the compared flights, with the wind and field radius of the design.
pub fn compare_ground() -> GroundTrack {
    let tracks = [(NOMINAL, 1u8), (WINDY, 2), (BIGGER, 3)].iter().map(|(m, slot)| simulate(*m).track(SeriesColor::Series(*slot), m.landing)).collect();
    GroundTrack { tracks, wind: Some(Wind { speed: 2.2, from_deg: 250.0 }), field_radius: Some(300.0), units: Default::default() }
}

/// A single flight's ground track.
pub fn simulate_ground() -> GroundTrack {
    let track = simulate(NOMINAL).track(SeriesColor::Ink, NOMINAL.landing);
    GroundTrack { tracks: vec![track], wind: Some(Wind { speed: 2.2, from_deg: 250.0 }), field_radius: Some(300.0), units: Default::default() }
}

/// Dispersion: 200 landings of the windy flight with their 95 % ellipse.
pub fn dispersion_ground() -> GroundTrack {
    let f = simulate(WINDY);
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut uniform = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let b = WINDY.landing.1.to_radians();
    let (e0, n0) = (WINDY.landing.0 * b.sin(), WINDY.landing.0 * b.cos());
    let scatter: Vec<(f64, f64)> = (0..200)
        .map(|_| {
            // Box–Muller: 55 m along the wind, 25 m across it.
            let (u1, u2) = (uniform().max(1e-12), uniform());
            let r = (-2.0 * u1.ln()).sqrt();
            let (z1, z2) = (r * (std::f64::consts::TAU * u2).cos(), r * (std::f64::consts::TAU * u2).sin());
            let (along, across) = (55.0 * z1, 25.0 * z2);
            (e0 + along * b.sin() + across * b.cos(), n0 + along * b.cos() - across * b.sin())
        })
        .collect();
    let track = f.track(SeriesColor::Series(2), WINDY.landing).with_scatter(scatter);
    GroundTrack { tracks: vec![track], wind: Some(Wind { speed: 7.0, from_deg: 250.0 }), field_radius: Some(300.0), units: Default::default() }
}

fn bump(x: f64, centre: f64, width: f64) -> f64 {
    (-((x - centre) / width).powi(2)).exp()
}

/// AnalyzeScreen: static margin with its limits, CP and CG, and the drag coefficient stacked by source, against Mach.
pub fn analyze_group() -> PlotGroup {
    let machs: Vec<f64> = (0..=58).map(|i| 0.1 + 0.05 * i as f64).collect();
    let curve = |f: &dyn Fn(f64) -> f64| -> Vec<(f64, f64)> { machs.iter().map(|&m| (m, f(m))).collect() };
    let margin = curve(&|m| 1.45 + 0.5 * (-0.9 * (m - 0.1)).exp() + 0.6 * bump(m, 1.1, 0.22));
    let cp = curve(&|m| 825.0 + 38.0 * (-0.8 * (m - 0.1)).exp() + 60.0 * bump(m, 1.1, 0.22));
    let cg_lift = curve(&|_| 712.4);
    let cg_burn = curve(&|_| 684.0);
    let skin = curve(&|m| 0.200 - 0.014 * (m - 0.1));
    let pressure = curve(&|m| 0.030 + 0.19 * bump(m, 1.1, 0.22) + 0.004 * m);
    let base = curve(&|m| 0.090 + 0.045 * bump(m, 1.0, 0.35) - 0.008 * m);
    let fins = curve(&|m| 0.095 + 0.04 * bump(m, 1.15, 0.25) - 0.012 * m);
    let total: Vec<(f64, f64)> = (0..machs.len()).map(|i| (machs[i], skin[i].1 + pressure[i].1 + base[i].1 + fins[i].1)).collect();

    let mut g = PlotGroup::new(Axis::mach())
        .panel(
            Panel::new("Static margin", "cal")
                .with(Series::new("Margin", margin))
                .hline(HLine::dashed(3.0, "Over-stable 3.0", SeriesColor::Warn))
                .hline(HLine::dashed(1.0, "Min 1.0", SeriesColor::Ok)),
        )
        .panel(
            Panel::new("CP and CG", "mm from tip")
                .value_unit("mm")
                .tag(Tag::Only(0))
                .with(Series::new("CP", cp).color(SeriesColor::Cp).label_inside())
                .with(Series::new("CG liftoff", cg_lift).color(SeriesColor::Cg).label_inside())
                .with(Series::new("CG burnout", cg_burn).color(SeriesColor::Cg).style(LineStyle::Dashed).label_inside()),
        )
        .panel(
            Panel::new("Drag coefficient Cd", "stacked by source")
                .weight(1.4)
                .tag(Tag::Only(4))
                .with(Series::new("Skin friction", skin).color(SeriesColor::Series(1)).filled().label_margin())
                .with(Series::new("Pressure", pressure).color(SeriesColor::Series(2)).filled().label_margin())
                .with(Series::new("Base", base).color(SeriesColor::Series(3)).filled().label_margin())
                .with(Series::new("Fins", fins).color(SeriesColor::Series(4)).filled().label_margin())
                .with(Series::new("Cd", total).label_as("Total", LabelAt::Inside)),
        );
    g.right_margin = 100.0;
    g
}

/// The sections of the showcase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Section {
    /// SimulateScreen centre column + ground track.
    Simulate,
    /// SimulateCompare: three flights.
    Compare,
    /// AnalyzeScreen: linked plots against Mach.
    Analyze,
    /// GroundTrack with dispersion.
    Dispersion,
}

impl Section {
    /// Every section, in display order.
    pub const ALL: [Section; 4] = [Section::Simulate, Section::Compare, Section::Analyze, Section::Dispersion];

    /// Short name.
    pub fn label(self) -> &'static str {
        match self {
            Section::Simulate => "Simulate",
            Section::Compare => "Compare",
            Section::Analyze => "Analyze",
            Section::Dispersion => "Dispersion",
        }
    }
}

/// What the showcase reports.
#[derive(Debug, Clone, Copy)]
pub enum ShowMessage {
    /// From the Simulate plots.
    Simulate(PlotMessage),
    /// From the Compare plots.
    Compare(PlotMessage),
    /// From the Analyze plots.
    Analyze(PlotMessage),
    /// From the Simulate ground track.
    SimulateGround(GroundMessage),
    /// From the Compare ground track.
    CompareGround(GroundMessage),
    /// From the Dispersion ground track.
    DispersionGround,
}

/// The demo's data and plot states.
pub struct Showcase {
    simulate: (PlotGroup, PlotState),
    compare: (PlotGroup, PlotState),
    analyze: (PlotGroup, PlotState),
    simulate_ground: GroundTrack,
    compare_ground: GroundTrack,
    dispersion: GroundTrack,
    /// The flight picked by clicking a landing marker in Compare.
    pub selected: usize,
}

impl Default for Showcase {
    fn default() -> Self {
        Self::new()
    }
}

impl Showcase {
    /// Builds all the demo data.
    pub fn new() -> Self {
        let with_state = |g: PlotGroup| {
            let s = PlotState::for_group(&g);
            (g, s)
        };
        Showcase {
            simulate: with_state(simulate_group()),
            compare: with_state(compare_group()),
            analyze: with_state(analyze_group()),
            simulate_ground: simulate_ground(),
            compare_ground: compare_ground(),
            dispersion: dispersion_ground(),
            selected: 0,
        }
    }

    /// Applies a message.
    pub fn update(&mut self, message: ShowMessage) {
        match message {
            ShowMessage::Simulate(m) => self.simulate.1.update(m),
            ShowMessage::Compare(m) => self.compare.1.update(m),
            ShowMessage::Analyze(m) => self.analyze.1.update(m),
            ShowMessage::SimulateGround(GroundMessage::Hover(t)) => self.simulate.1.set_hover(t),
            ShowMessage::CompareGround(GroundMessage::Hover(t)) => self.compare.1.set_hover(t),
            ShowMessage::CompareGround(GroundMessage::Select(i)) => self.selected = i,
            ShowMessage::SimulateGround(GroundMessage::Select(_)) | ShowMessage::DispersionGround => {}
        }
    }

    #[cfg(test)]
    /// The state of a section's plots (for the "values at cursor" readout).
    pub fn state(&self, section: Section) -> Option<&PlotState> {
        match section {
            Section::Simulate => Some(&self.simulate.1),
            Section::Compare => Some(&self.compare.1),
            Section::Analyze => Some(&self.analyze.1),
            Section::Dispersion => None,
        }
    }

    /// A section of the showcase, sized for the gallery (about 900 × 560).
    pub fn view(&self, section: Section) -> Element<'_, ShowMessage> {
        match section {
            Section::Simulate => {
                let (g, s) = &self.simulate;
                let plots =
                    column![header(g, s).map(ShowMessage::Simulate), framed(PlotView::new(g, s).element().map(ShowMessage::Simulate))].spacing(tokens::SPACE_2);
                let ground = GroundTrackView::new(&self.simulate_ground, s.hover_x()).element().map(ShowMessage::SimulateGround);
                row![
                    plots.width(Length::FillPortion(3)),
                    column![caption("GROUND TRACK"), framed(ground)].spacing(tokens::SPACE_2).width(Length::FillPortion(1))
                ]
                .spacing(tokens::SPACE_3)
                .boxed()
            }
            Section::Compare => {
                let (g, s) = &self.compare;
                let names = [
                    (SeriesColor::Series(1), "Nominal".to_string()),
                    (SeriesColor::Series(2), "Windy".to_string()),
                    (SeriesColor::Series(3), "Bigger motor".to_string()),
                ];
                let plots = column![header(g, s).map(ShowMessage::Compare), legend(&names), framed(PlotView::new(g, s).element().map(ShowMessage::Compare))]
                    .spacing(tokens::SPACE_2);
                let ground = GroundTrackView::new(&self.compare_ground, s.hover_x()).element().map(ShowMessage::CompareGround);
                row![
                    plots.width(Length::FillPortion(3)),
                    column![caption("GROUND TRACK"), framed(ground)].spacing(tokens::SPACE_2).width(Length::FillPortion(1))
                ]
                .spacing(tokens::SPACE_3)
                .boxed()
            }
            Section::Analyze => {
                let (g, s) = &self.analyze;
                column![header(g, s).map(ShowMessage::Analyze), framed(PlotView::new(g, s).element().map(ShowMessage::Analyze))]
                    .spacing(tokens::SPACE_2)
                    .boxed()
            }
            Section::Dispersion => {
                let ground = GroundTrackView::new(&self.dispersion, None).element().map(|_| ShowMessage::DispersionGround);
                column![caption("DISPERSION · 200 RUNS OF WINDY WITH VARIED WIND AND LAUNCH ANGLE"), framed(ground)].spacing(tokens::SPACE_2).boxed()
            }
        }
    }
}

fn caption<'a, M: 'a>(s: &'static str) -> Element<'a, M> {
    text(s)
        .font(MONO)
        .size(tokens::TEXT_LABEL)
        .style(|theme| iced::widget::text::Style { color: Some(theme::tokens(theme).ink_faint), ..Default::default() })
        .boxed()
}

fn framed<'a, M: 'a>(content: impl Widget<M> + 'a) -> Element<'a, M> {
    container(content).width(Length::Fill).height(Length::Fill).style(theme::ground).boxed()
}

/// What the demo page reports.
#[derive(Debug, Clone, Copy)]
pub enum DemoMessage {
    /// Switch section.
    Section(Section),
    /// A plot or ground-track message.
    Show(ShowMessage),
    /// Dark ↔ light (standalone app only).
    #[cfg(test)]
    ToggleTheme,
}

/// The showcase with a section switcher: embed it in the gallery as a page, (the standalone app is test-only).
pub struct Demo {
    show: Showcase,
    section: Section,
    #[cfg(test)]
    dark: bool,
}

impl Default for Demo {
    fn default() -> Self {
        Demo {
            show: Showcase::new(),
            section: Section::Simulate,
            #[cfg(test)]
            dark: true,
        }
    }
}

impl Demo {
    /// Applies a message.
    pub fn update(&mut self, message: DemoMessage) {
        match message {
            DemoMessage::Section(s) => self.section = s,
            DemoMessage::Show(m) => self.show.update(m),
            #[cfg(test)]
            DemoMessage::ToggleTheme => self.dark = !self.dark,
        }
    }

    /// The switcher above the active section.
    pub fn view(&self) -> Element<'_, DemoMessage> {
        let options = Section::ALL.map(|s| (s, s.label()));
        let switcher =
            container(crate::widgets::segmented::segmented(&options, self.section, DemoMessage::Section)).height(tokens::CONTROL_H).align_left(Length::Shrink);
        column![switcher, self.show.view(self.section).map(DemoMessage::Show)].spacing(tokens::SPACE_3).height(Length::Fill).boxed()
    }

    #[cfg(test)]
    fn theme(&self) -> iced::Theme {
        if self.dark {
            theme::dark()
        } else {
            theme::light()
        }
    }

    #[cfg(test)]
    fn standalone(&self) -> Element<'_, DemoMessage> {
        let toggle: Element<'_, DemoMessage> =
            crate::widgets::button::button(if self.dark { "Light" } else { "Dark" }, crate::widgets::button::Weight::Secondary)
                .on_press(DemoMessage::ToggleTheme)
                .into();
        container(column![row![iced::widget::Space::new().width(Length::Fill), toggle], self.view()].spacing(tokens::SPACE_3))
            .padding(tokens::SPACE_4)
            .width(Length::Fill)
            .height(Length::Fill)
            .style(|t: &iced::Theme| container::Style {
                background: Some(theme::tokens(t).bg_100.into()),
                text_color: Some(theme::tokens(t).ink),
                ..container::Style::default()
            })
            .boxed()
    }
}

/// The demo as a standalone app, for `drive::run` (see `scripts/plot.drive`).
#[cfg(test)]
pub fn app() -> iced::Application<impl iced::Program<State = Demo, Message = DemoMessage, Theme = iced::Theme>> {
    iced::application(Demo::default, Demo::update, Demo::standalone)
        .title("Bowshock plot demo")
        .theme(Demo::theme)
        .fonts(theme::FONTS)
        .font(theme::SANS)
        .window_size(iced::Size::new(1280.0, 720.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_demo_flight_has_plausible_numbers() {
        let f = simulate(NOMINAL);
        let max = |v: &[f64]| v.iter().copied().fold(f64::MIN, f64::max);
        assert!((500.0..750.0).contains(&max(&f.alt)), "apogee {}", max(&f.alt));
        assert!((100.0..160.0).contains(&max(&f.vel)), "vmax {}", max(&f.vel));
        assert!((9.0..13.0).contains(&f.apogee), "t_apogee {}", f.apogee);
        assert!(f.rail_exit > 0.1 && f.rail_exit < f.burnout);
        assert!(f.burnout < f.apogee && f.apogee < f.drogue && f.drogue < f.main && f.main < f.landing);
        assert!((50.0..75.0).contains(&f.landing), "landing {}", f.landing);
        // Drogue descent is about 22 m/s, main about 5 m/s.
        let vel_at = |t: f64| f.vel[f.t.partition_point(|x| *x < t).min(f.vel.len() - 1)];
        assert!((vel_at((f.drogue + f.main) / 2.0) + 22.0).abs() < 2.0);
        assert!((vel_at(f.landing - 1.0) + 5.0).abs() < 1.0);
    }

    #[test]
    fn the_bigger_motor_flies_higher_and_the_tracks_land_where_declared() {
        let (a, b) = (simulate(NOMINAL), simulate(BIGGER));
        assert!(b.alt.iter().copied().fold(0.0, f64::max) > a.alt.iter().copied().fold(0.0, f64::max));
        let t = a.track(SeriesColor::Ink, NOMINAL.landing);
        let l = t.landing().unwrap();
        assert!((l.range() - 104.0).abs() < 8.0, "{}", l.range());
        // Ground tracks start at the pad.
        assert_eq!((t.points[0].east, t.points[0].north), (0.0, 0.0));
    }

    #[test]
    fn the_demo_groups_are_well_formed() {
        for g in [simulate_group(), compare_group(), analyze_group()] {
            let (a, b) = g.x_extent();
            assert!(a < b);
            for s in g.panels.iter().flat_map(|p| &p.series) {
                assert!(s.points.windows(2).all(|w| w[0].0 <= w[1].0), "{} not sorted", s.name);
            }
        }
        let g = analyze_group();
        // The stacked layers sum to the Cd line.
        let p = &g.panels[2];
        for x in [0.3, 1.1, 2.7] {
            let total = p.stack_total_at(x).unwrap();
            assert!((total - p.series[4].value_at(x).unwrap()).abs() < 1e-9);
        }
        assert!(Showcase::new().state(Section::Dispersion).is_none());
    }

    #[test]
    fn hover_links_the_plots_and_the_ground_track() {
        let mut s = Showcase::new();
        s.update(ShowMessage::SimulateGround(GroundMessage::Hover(Some(4.2))));
        assert_eq!(s.state(Section::Simulate).unwrap().hover_x(), Some(4.2));
        s.update(ShowMessage::Simulate(PlotMessage::Hover(None)));
        assert_eq!(s.state(Section::Simulate).unwrap().hover_x(), None);
        s.update(ShowMessage::CompareGround(GroundMessage::Select(2)));
        assert_eq!(s.selected, 2);
    }

    /// Screenshots the standalone demo through the drive script:
    /// `PLOT_DRIVE=$PWD/crates/bowshock/scripts/plot.drive PLOT_OUT=$PWD/out/plot cargo test -p bowshock plot_screenshots -- --ignored`
    #[test]
    #[ignore = "writes screenshots; run by hand"]
    fn plot_screenshots() {
        let script = std::env::var("PLOT_DRIVE").expect("PLOT_DRIVE=<script>");
        let out = std::env::var("PLOT_OUT").unwrap_or_else(|_| "out/plot".into());
        crate::drive::run(&app(), std::path::Path::new(&script), std::path::Path::new(&out)).expect("drive script");
    }
}
