//! The data sources of the Simulate plots: every quantity a flight records per sample ([`bowshock_sim::Sample`]), named, grouped
//! for the source picker, with the unit it is shown in.
//!
//! A plot has **one y-axis** (design/components/PlotZoom: "never dual-axis"), so two sources can share a plot only when they
//! measure the same kind of thing: that is [`Dim`], and [`Channel::joins`] is the rule. Channels are addressed by a stable string
//! key (the key the document stores in `PlotLayout`), so a saved layout survives reordering this table.
//!
//! Values are SI in the samples; [`Channel::unit`] gives the display unit through `units.rs` (`Quantity`) for the app's unit set.

use bowshock_sim::Sample;

use crate::units::{Quantity, Unit, UnitSystem};
use crate::widgets::plot::SeriesColor;

/// What a source measures: the compatibility class of the y-axis. Sources join one plot only if they have the same `Dim`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dim {
    /// Altitudes, positions and distances (m, ft).
    Length,
    /// Stations from the nose tip (mm, in).
    Station,
    /// Speeds (m/s, ft/s).
    Velocity,
    /// Accelerations, shown in g.
    Acceleration,
    /// Angles (°).
    Angle,
    /// Mach number.
    Mach,
    /// Pressures (kPa, psi).
    Pressure,
    /// Air density (kg/m³, lb/ft³).
    Density,
    /// Temperatures (°C, °F).
    Temperature,
    /// Forces (N, lbf).
    Force,
    /// Masses (kg, lb).
    Mass,
    /// Drag areas `Cd A` (m², in²).
    Area,
    /// Drag coefficients.
    DragCoefficient,
    /// Reynolds number.
    Reynolds,
    /// Static margin in calibers.
    Calibers,
    /// Rotation rates (°/s).
    AngularRate,
}

/// Reynolds numbers reach millions: shown in units of 10⁶ so the axis labels stay short.
static REYNOLDS: Unit = Unit { symbol: "×10⁶", to_si: 1e6, offset: 0.0, decimals: 2 };
/// Dimensionless: no unit text.
static PLAIN: Unit = Unit { symbol: "", to_si: 1.0, offset: 0.0, decimals: 2 };
/// Rotation rates: degrees per second.
static ANGULAR_RATE: Unit = Unit { symbol: "°/s", to_si: std::f64::consts::PI / 180.0, offset: 0.0, decimals: 1 };

impl Dim {
    /// The display unit in `units`: the pick of the dimension's family (distances in the altitude family, stations in the
    /// length family), so a plot axis and a readout of the same quantity always agree.
    pub fn unit(self, units: UnitSystem) -> Unit {
        match self {
            Dim::Length => *Quantity::Altitude.unit_for(units),
            Dim::Station => *Quantity::Length.unit_for(units),
            Dim::Velocity => *Quantity::Velocity.unit_for(units),
            Dim::Acceleration => *Quantity::Acceleration.unit_for(units),
            Dim::Angle => *Quantity::Angle.unit_for(units),
            Dim::Mach | Dim::DragCoefficient => PLAIN,
            Dim::Pressure => *Quantity::Pressure.unit_for(units),
            Dim::Density => *Quantity::Density.unit_for(units),
            Dim::Temperature => *Quantity::Temperature.unit_for(units),
            Dim::Force => *Quantity::Force.unit_for(units),
            Dim::Mass => *Quantity::Mass.unit_for(units),
            Dim::Area => *Quantity::Area.unit_for(units),
            Dim::Reynolds => REYNOLDS,
            Dim::Calibers => *Quantity::Calibers.unit_for(units),
            Dim::AngularRate => ANGULAR_RATE,
        }
    }

    /// What a plot of this dimension is called when it holds several sources (`Speed`, `Force`).
    pub fn title(self) -> &'static str {
        match self {
            Dim::Length => "Distance",
            Dim::Station => "Station",
            Dim::Velocity => "Speed",
            Dim::Acceleration => "Acceleration",
            Dim::Angle => "Angle",
            Dim::Mach => "Mach",
            Dim::Pressure => "Pressure",
            Dim::Density => "Density",
            Dim::Temperature => "Temperature",
            Dim::Force => "Force",
            Dim::Mass => "Mass",
            Dim::Area => "Drag area",
            Dim::DragCoefficient => "Drag coefficient",
            Dim::Reynolds => "Reynolds number",
            Dim::Calibers => "Static margin",
            Dim::AngularRate => "Angular rate",
        }
    }

    /// The dimension as a noun for a sentence (`a length`, `a speed`).
    pub fn noun(self) -> &'static str {
        match self {
            Dim::Length => "length",
            Dim::Station => "station from the nose tip",
            Dim::Velocity => "speed",
            Dim::Acceleration => "acceleration",
            Dim::Angle => "angle",
            Dim::Mach => "Mach number",
            Dim::Pressure => "pressure",
            Dim::Density => "density",
            Dim::Temperature => "temperature",
            Dim::Force => "force",
            Dim::Mass => "mass",
            Dim::Area => "drag area",
            Dim::DragCoefficient => "drag coefficient",
            Dim::Reynolds => "Reynolds number",
            Dim::Calibers => "margin in calibers",
            Dim::AngularRate => "angular rate",
        }
    }
}

/// The picker's sections, in menu order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Group {
    /// Where the rocket is.
    Position,
    /// How fast and which way it moves.
    Velocity,
    /// How fast it speeds up.
    Acceleration,
    /// Mach, dynamic pressure, drag coefficients.
    Aerodynamics,
    /// Where the nose points and how fast it turns.
    Attitude,
    /// Thrust and drag.
    Forces,
    /// Mass, CG, CP, margin.
    MassBalance,
    /// The air and the wind.
    Atmosphere,
    /// Deployed recovery devices.
    Recovery,
}

impl Group {
    /// Every group, in menu order.
    pub const ALL: [Group; 9] = [
        Group::Position,
        Group::Velocity,
        Group::Acceleration,
        Group::Attitude,
        Group::Aerodynamics,
        Group::Forces,
        Group::MassBalance,
        Group::Atmosphere,
        Group::Recovery,
    ];

    /// The section heading.
    pub fn title(self) -> &'static str {
        match self {
            Group::Position => "Position",
            Group::Velocity => "Velocity",
            Group::Acceleration => "Acceleration",
            Group::Aerodynamics => "Aerodynamics",
            Group::Attitude => "Attitude",
            Group::Forces => "Forces",
            Group::MassBalance => "Mass & balance",
            Group::Atmosphere => "Atmosphere",
            Group::Recovery => "Recovery",
        }
    }
}

/// One data source: a recorded quantity of a flight.
pub struct Channel {
    /// The stable key stored in the document (`"altitude"`).
    pub key: &'static str,
    /// The name in the picker, the legend and the hover tooltip.
    pub label: &'static str,
    /// The picker section.
    pub group: Group,
    /// What it measures (the y-axis compatibility class).
    pub dim: Dim,
    /// Never negative: its plot anchors y = 0 at the bottom.
    pub zero_based: bool,
    /// The colour of its line when one flight is on show (`Ink` unless the app has a role for it, like CG and CP).
    pub color: SeriesColor,
    /// One plain sentence on what it is, shown when the cursor rests on its picker row (empty when the name says it all).
    pub hint: &'static str,
    get: fn(&Sample) -> f64,
}

impl Channel {
    /// The value in SI units at `sample`.
    pub fn si(&self, sample: &Sample) -> f64 {
        (self.get)(sample)
    }

    /// The display unit in `units`.
    pub fn unit(&self, units: UnitSystem) -> Unit {
        self.dim.unit(units)
    }

    /// `(time, value in unit)` for every sample.
    pub fn series(&self, samples: &[Sample], unit: &Unit) -> Vec<(f64, f64)> {
        samples.iter().map(|s| (s.t, unit.from_si(self.si(s)))).collect()
    }

    /// Whether this source can share a plot whose y-axis measures `plot`: one y-axis per plot, never two.
    pub fn joins(&self, plot: Dim) -> bool {
        self.dim == plot
    }

    /// Why this source cannot join a plot measuring `plot`, or `None` when it can.
    pub fn refusal(&self, plot: Dim, units: UnitSystem) -> Option<String> {
        if self.joins(plot) {
            return None;
        }
        let unit = plot.unit(units).symbol;
        let shown = if unit.is_empty() { plot.noun().to_string() } else { format!("{} ({unit})", plot.noun()) };
        Some(format!("{} is a {}, but this plot shows {shown}. There is one y-axis per plot.", self.label, self.dim.noun()))
    }

    /// Whether `query` (case-insensitive words) matches the label, the group heading or the key.
    pub fn matches(&self, query: &str) -> bool {
        let haystack = format!("{} {} {}", self.label, self.group.title(), self.key).to_lowercase();
        query.to_lowercase().split_whitespace().all(|word| haystack.contains(word))
    }
}

const fn ch(key: &'static str, label: &'static str, group: Group, dim: Dim, zero_based: bool, get: fn(&Sample) -> f64) -> Channel {
    Channel { key, label, group, dim, zero_based, color: SeriesColor::Ink, hint: "", get }
}

const fn tinted(mut c: Channel, color: SeriesColor) -> Channel {
    c.color = color;
    c
}

const fn described(mut c: Channel, hint: &'static str) -> Channel {
    c.hint = hint;
    c
}

/// Every source the plots offer, in picker order.
pub static CHANNELS: [Channel; 38] = [
    // Position.
    ch("altitude", "Altitude (AGL)", Group::Position, Dim::Length, true, |s| s.altitude),
    ch("altitude_msl", "Altitude (MSL)", Group::Position, Dim::Length, false, |s| s.altitude_msl),
    ch("position_east", "Position east", Group::Position, Dim::Length, false, |s| s.position[0]),
    ch("position_north", "Position north", Group::Position, Dim::Length, false, |s| s.position[1]),
    ch("ground_distance", "Ground distance", Group::Position, Dim::Length, true, |s| s.ground_distance),
    // Velocity.
    ch("vertical_velocity", "Vertical velocity", Group::Velocity, Dim::Velocity, false, |s| s.vertical_velocity),
    ch("horizontal_velocity", "Horizontal velocity", Group::Velocity, Dim::Velocity, true, |s| s.horizontal_velocity),
    ch("total_velocity", "Total velocity", Group::Velocity, Dim::Velocity, true, |s| s.speed),
    ch("air_speed", "Air-relative speed", Group::Velocity, Dim::Velocity, true, |s| s.air_speed),
    ch("flight_path_angle", "Flight-path angle", Group::Velocity, Dim::Angle, false, |s| s.flight_path_angle),
    // Acceleration (in g, the kit's unit).
    ch("vertical_acceleration", "Vertical acceleration", Group::Acceleration, Dim::Acceleration, false, |s| s.vertical_acceleration),
    ch("total_acceleration", "Total acceleration", Group::Acceleration, Dim::Acceleration, true, |s| s.acceleration),
    // Attitude and rotation.
    described(
        ch("attitude_angle", "Attitude angle", Group::Attitude, Dim::Angle, false, |s| s.attitude_angle),
        "How high the nose points above the horizon: 90° straight up, 0° level, negative nose down. It differs from the flight-path angle by the angle of attack.",
    ),
    described(
        ch("pitch_rate", "Pitch rate", Group::Attitude, Dim::AngularRate, true, |s| s.pitch_rate),
        "How fast the nose swings (pitch and yaw together). Zero under a parachute, where the descent is flown as a point mass.",
    ),
    described(
        ch("roll_rate", "Roll rate", Group::Attitude, Dim::AngularRate, false, |s| s.roll_rate),
        "How fast the rocket spins about its axis. Only canted fins drive it; positive turns the way increasing fin angle does.",
    ),
    // Aerodynamics.
    ch("mach", "Mach number", Group::Aerodynamics, Dim::Mach, true, |s| s.mach),
    ch("dynamic_pressure", "Dynamic pressure", Group::Aerodynamics, Dim::Pressure, true, |s| s.dynamic_pressure),
    ch("reynolds", "Reynolds number", Group::Aerodynamics, Dim::Reynolds, true, |s| s.reynolds),
    described(
        ch("cd", "Drag coefficient (total)", Group::Aerodynamics, Dim::DragCoefficient, true, |s| s.cd),
        "The drag parts below added up (the airframe only: it fades to zero as a parachute fills; see Recovery Cd·A). Drag force = dynamic pressure × Cd × frontal area.",
    ),
    described(
        ch("cd_friction", "Cd · skin friction", Group::Aerodynamics, Dim::DragCoefficient, true, |s| s.cd_friction),
        "Air rubbing along the surface of the nose and body tube. Grows with surface area and roughness.",
    ),
    described(
        ch("cd_pressure", "Cd · nose & body pressure", Group::Aerodynamics, Dim::DragCoefficient, true, |s| s.cd_pressure),
        "Drag from the shape of the nose and transitions: air piling up in front and, near Mach 1, shock waves.",
    ),
    described(
        ch("cd_base", "Cd · base (tail suction)", Group::Aerodynamics, Dim::DragCoefficient, true, |s| s.cd_base),
        "The low-pressure wake behind the blunt rear end pulling the rocket back. Smaller while the motor burns, as the exhaust fills it.",
    ),
    described(
        ch("cd_fins", "Cd · fins", Group::Aerodynamics, Dim::DragCoefficient, true, |s| s.cd_fins),
        "The fins' own skin friction plus the drag of their leading and trailing edges.",
    ),
    described(
        ch("cd_protuberances", "Cd · lugs & buttons", Group::Aerodynamics, Dim::DragCoefficient, true, |s| s.cd_protuberances),
        "Launch lugs and rail buttons sticking out of the body.",
    ),
    described(
        ch("angle_of_attack", "Angle of attack", Group::Aerodynamics, Dim::Angle, true, |s| s.angle_of_attack),
        "Angle between the rocket's axis and the oncoming air. Wind, gusts and the turn of the flight path push the rocket off the air velocity and the fins swing it back (weathercocking); 180° is tail first.",
    ),
    // Forces.
    ch("thrust", "Thrust", Group::Forces, Dim::Force, true, |s| s.thrust),
    ch("drag", "Drag force", Group::Forces, Dim::Force, true, |s| s.drag),
    // Mass & balance.
    ch("mass", "Mass", Group::MassBalance, Dim::Mass, true, |s| s.mass),
    ch("propellant_mass", "Propellant mass", Group::MassBalance, Dim::Mass, true, |s| s.propellant_mass),
    tinted(ch("cg_station", "CG station", Group::MassBalance, Dim::Station, true, |s| s.cg), SeriesColor::Cg),
    tinted(ch("cp_station", "CP station", Group::MassBalance, Dim::Station, true, |s| s.cp), SeriesColor::Cp),
    ch("stability_margin", "Stability margin", Group::MassBalance, Dim::Calibers, false, |s| s.stability_margin),
    // Atmosphere.
    ch("air_density", "Air density", Group::Atmosphere, Dim::Density, true, |s| s.air_density),
    ch("air_temperature", "Air temperature", Group::Atmosphere, Dim::Temperature, false, |s| s.air_temperature),
    ch("air_pressure", "Air pressure", Group::Atmosphere, Dim::Pressure, true, |s| s.air_pressure),
    ch("speed_of_sound", "Speed of sound", Group::Atmosphere, Dim::Velocity, true, |s| s.speed_of_sound),
    ch("wind_speed", "Wind speed", Group::Atmosphere, Dim::Velocity, true, |s| s.wind_speed),
    // Recovery.
    ch("recovery_cd_area", "Recovery Cd·A", Group::Recovery, Dim::Area, true, |s| s.recovery_cd_area),
];

/// The source with this stored key.
pub fn find(key: &str) -> Option<&'static Channel> {
    CHANNELS.iter().find(|c| c.key == key)
}

/// The sources of `group`, in picker order.
pub fn in_group(group: Group) -> impl Iterator<Item = &'static Channel> {
    CHANNELS.iter().filter(move |c| c.group == group)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_sim::G0;

    fn sample() -> Sample {
        Sample {
            t: 3.0,
            position: [30.0, 40.0, 500.0],
            altitude: 500.0,
            altitude_msl: 620.0,
            vertical_velocity: -20.0,
            speed: 25.0,
            acceleration: 2.0 * G0,
            vertical_acceleration: -G0,
            thrust: 100.0,
            drag: 5.0,
            mass: 1.2,
            cg: 0.7124,
            cp: 0.8587,
            dynamic_pressure: 2000.0,
            air_pressure: 101_325.0,
            air_temperature: 288.15,
            reynolds: 2.5e6,
            recovery_cd_area: 0.9,
            attitude_angle: std::f64::consts::FRAC_PI_3,
            pitch_rate: 0.5,
            roll_rate: -2.0,
            ..Sample::default()
        }
    }

    #[test]
    fn keys_are_unique_and_findable() {
        let mut keys: Vec<&str> = CHANNELS.iter().map(|c| c.key).collect();
        keys.sort_unstable();
        keys.dedup();
        assert_eq!(keys.len(), CHANNELS.len());
        assert!(CHANNELS.iter().all(|c| find(c.key).is_some_and(|f| std::ptr::eq(f, c))));
        assert!(find("no_such_channel").is_none());
        // Every group has at least one source and the table is ordered by group.
        assert!(Group::ALL.iter().all(|g| in_group(*g).count() > 0));
        let order: Vec<usize> = CHANNELS.iter().map(|c| Group::ALL.iter().position(|g| *g == c.group).unwrap()).collect();
        assert!(order.windows(2).all(|w| w[0] <= w[1]));
    }

    #[test]
    fn every_default_and_preset_key_exists() {
        for key in ["altitude", "vertical_velocity", "vertical_acceleration", "mach", "thrust", "cg_station", "cp_station", "stability_margin"] {
            assert!(find(key).is_some(), "{key}");
        }
    }

    #[test]
    fn values_are_extracted_and_converted_to_the_display_unit() {
        let s = sample();
        let value = |key: &str, set: UnitSystem| {
            let c = find(key).unwrap();
            c.unit(set).from_si(c.si(&s))
        };
        let (metric, si, inch, foot) = (UnitSystem::MM_G_S, UnitSystem::M_KG_S, UnitSystem::IN_OZ_S, UnitSystem::FT_LB_S);
        // Altitude: m, then ft.
        assert!((value("altitude", metric) - 500.0).abs() < 1e-12);
        assert!((value("altitude", foot) - 500.0 / 0.3048).abs() < 1e-9);
        assert!((value("altitude_msl", metric) - 620.0).abs() < 1e-12);
        assert_eq!((value("position_east", metric), value("position_north", metric)), (30.0, 40.0));
        // Acceleration is in g in these sets: -g reads -1, twice g reads 2; SI sets read m/s².
        assert!((value("vertical_acceleration", metric) + 1.0).abs() < 1e-12 && (value("vertical_acceleration", inch) + 1.0).abs() < 1e-12);
        assert!((value("total_acceleration", metric) - 2.0).abs() < 1e-12);
        assert!((value("total_acceleration", si) - 2.0 * 9.80665).abs() < 1e-9);
        // Stations: mm, then inches.
        assert!((value("cg_station", metric) - 712.4).abs() < 1e-9);
        assert!((value("cp_station", inch) - 0.8587 / 0.0254).abs() < 1e-9);
        // Pressure: kPa, then psi. 101 325 Pa is 101.325 kPa and 14.696 psi.
        assert!((value("air_pressure", metric) - 101.325).abs() < 1e-9);
        assert!((value("air_pressure", inch) - 14.6959).abs() < 1e-3);
        assert!((value("dynamic_pressure", metric) - 2.0).abs() < 1e-12);
        // Temperature: °C, then °F (288.15 K is 15 °C = 59 °F).
        assert!((value("air_temperature", metric) - 15.0).abs() < 1e-9 && (value("air_temperature", inch) - 59.0).abs() < 1e-9);
        // Speed: m/s, then ft/s. Force: N, then lbf. Mass: kg, then lb.
        assert!((value("total_velocity", inch) - 25.0 / 0.3048).abs() < 1e-9);
        assert!((value("thrust", inch) - 100.0 / 4.448_221_615_260_5).abs() < 1e-9);
        assert!((value("mass", si) - 1.2).abs() < 1e-9);
        assert!((value("mass", foot) - 1.2 / 0.453_592_37).abs() < 1e-9);
        // Reynolds in millions; Cd·A in m², then in².
        assert!((value("reynolds", metric) - 2.5).abs() < 1e-12);
        assert!((value("recovery_cd_area", si) - 0.9).abs() < 1e-12);
        assert!((value("recovery_cd_area", inch) - 0.9 / (0.0254 * 0.0254)).abs() < 1e-6);
        // Attitude angle in degrees; rotation rates in degrees per second (0.5 rad/s is 28.65 deg/s, roll the other way).
        assert!((value("attitude_angle", metric) - 60.0).abs() < 1e-9);
        assert!((value("pitch_rate", metric) - 0.5f64.to_degrees()).abs() < 1e-9 && (value("roll_rate", inch) + 2.0f64.to_degrees()).abs() < 1e-9);
        assert_eq!(find("pitch_rate").unwrap().unit(metric).symbol, "°/s");
        // The unit text matches.
        assert_eq!(find("altitude").unwrap().unit(metric).symbol, "m");
        assert_eq!(find("altitude").unwrap().unit(inch).symbol, "ft");
        assert_eq!(find("air_pressure").unwrap().unit(inch).symbol, "psi");
        assert_eq!(find("mach").unwrap().unit(metric).symbol, "");
    }

    #[test]
    fn a_series_has_one_point_per_sample_in_time_order() {
        let samples = [Sample { t: 0.0, altitude: 0.0, ..Sample::default() }, sample()];
        let c = find("altitude").unwrap();
        assert_eq!(c.series(&samples, &c.unit(UnitSystem::MM_G_S)), [(0.0, 0.0), (3.0, 500.0)]);
    }

    #[test]
    fn sources_share_a_plot_only_with_the_same_dimension() {
        let altitude = find("altitude").unwrap();
        // Altitude, ground distance and east position are all lengths: they overlay.
        for key in ["altitude_msl", "position_east", "ground_distance"] {
            assert!(find(key).unwrap().joins(altitude.dim), "{key}");
        }
        // A speed, a force or a Mach number never joins a length plot.
        for key in ["vertical_velocity", "thrust", "mach", "cg_station", "cd"] {
            assert!(!find(key).unwrap().joins(altitude.dim), "{key}");
        }
        // Speed of sound and wind speed are speeds: they join a velocity plot, an angle does not.
        let velocity = find("vertical_velocity").unwrap();
        assert!(find("wind_speed").unwrap().joins(velocity.dim) && find("speed_of_sound").unwrap().joins(velocity.dim));
        assert!(!find("flight_path_angle").unwrap().joins(velocity.dim));
        // Dimensionless things are still different things: Mach, Cd, Reynolds and the margin do not mix.
        let dims = [Dim::Mach, Dim::DragCoefficient, Dim::Reynolds, Dim::Calibers];
        for (i, a) in dims.iter().enumerate() {
            assert!(dims.iter().enumerate().all(|(j, b)| (a == b) == (i == j)));
        }
        // CG and CP are both stations.
        assert!(find("cp_station").unwrap().joins(find("cg_station").unwrap().dim));
    }

    #[test]
    fn a_refusal_names_both_quantities() {
        let thrust = find("thrust").unwrap();
        let si = UnitSystem::MM_G_S;
        assert!(thrust.refusal(Dim::Force, si).is_none());
        let why = find("vertical_velocity").unwrap().refusal(Dim::Force, si).unwrap();
        assert!(why.contains("Vertical velocity") && why.contains("speed") && why.contains("force (N)") && why.contains("one y-axis"), "{why}");
        let why = find("altitude").unwrap().refusal(Dim::Mach, si).unwrap();
        assert!(why.contains("Mach"), "{why}");
        assert!(find("altitude").unwrap().refusal(Dim::Velocity, UnitSystem::IN_OZ_S).unwrap().contains("ft/s"));
    }

    #[test]
    fn every_dimension_follows_its_family_in_a_mixed_set() {
        use crate::units::Family;
        let set = UnitSystem::MM_G_S.with(Family::Length, "in").with(Family::Mass, "kg").with(Family::Distance, "ft").with(Family::Pressure, "psi");
        assert_eq!(Dim::Station.unit(set).symbol, "in");
        assert_eq!(Dim::Mass.unit(set).symbol, "kg");
        assert_eq!(Dim::Length.unit(set).symbol, "ft");
        assert_eq!(Dim::Pressure.unit(set).symbol, "psi");
        assert_eq!(Dim::Velocity.unit(set).symbol, "m/s");
        assert_eq!(Dim::Force.unit(set).symbol, "N");
    }

    #[test]
    fn the_picker_search_matches_words_of_the_label_group_and_key() {
        let c = find("cd_friction").unwrap();
        assert!(c.matches("") && c.matches("fric") && c.matches("cd fric") && c.matches("AERODYN") && c.matches("cd_friction"));
        assert!(!c.matches("thrust") && !c.matches("cd thrust"));
        let hits: Vec<&str> = CHANNELS.iter().filter(|c| c.matches("velocity")).map(|c| c.key).collect();
        assert!(hits.contains(&"vertical_velocity") && hits.contains(&"total_velocity") && !hits.contains(&"thrust"));
    }

    #[test]
    fn only_signed_sources_are_not_zero_based() {
        let signed = [
            "altitude_msl",
            "position_east",
            "position_north",
            "vertical_velocity",
            "flight_path_angle",
            "attitude_angle",
            "roll_rate",
            "vertical_acceleration",
            "stability_margin",
            "air_temperature",
        ];
        for c in &CHANNELS {
            assert_eq!(!c.zero_based, signed.contains(&c.key), "{}", c.key);
        }
    }
}
