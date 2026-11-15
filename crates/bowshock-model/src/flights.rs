//! Stored flight definitions: the scenarios of the Simulate screen (a name, which motors it burns, launch conditions).
//!
//! Flights are part of the document, so they save, load and undo with the rocket. A flight never copies the rocket: it names a
//! configuration (or one motor to swap in) and the app flies the rocket *as it is when the flight runs*. Results are not stored;
//! they are re-run. A new document has one flight, `Nominal`.

use serde::{Deserialize, Serialize};

use crate::config::{ConfigId, MotorRef};
use crate::rocket::Rocket;

/// Identifies a flight within a [`Rocket`].
pub type FlightId = u64;

/// Largest rail tilt from vertical a flight may have (rad): 70°.
pub const MAX_LAUNCH_ANGLE: f64 = 70.0 * std::f64::consts::PI / 180.0;

/// What a flight burns.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum FlightMotor {
    /// Whatever the document's active configuration holds, now.
    Active,
    /// A named configuration of the document.
    Config(ConfigId),
    /// This motor in the first mount of the first stage, everything else as the active configuration has it.
    Motor(MotorRef),
}

/// Gust intensity (standard deviation of the gusts as a fraction of the mean wind).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Turbulence {
    /// No gusts.
    Off,
    /// Low gusts (5 % of the mean wind).
    #[default]
    Low,
    /// Medium gusts (10 %).
    Medium,
    /// High gusts (20 %).
    High,
}

/// The launch site and weather of a flight. SI throughout: metres, radians, kelvin, m/s.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Conditions {
    /// Launch rail length (m).
    pub rail_length: f64,
    /// Rail tilt from vertical (rad), `0..=`[`MAX_LAUNCH_ANGLE`].
    pub launch_angle: f64,
    /// Mean wind speed (m/s).
    pub wind_speed: f64,
    /// Direction the wind blows from, clockwise from north (rad), `0..2π`.
    pub wind_from: f64,
    /// Gust intensity.
    pub turbulence: Turbulence,
    /// Site altitude above sea level (m).
    pub site_altitude: f64,
    /// Ground temperature (K).
    pub temperature: f64,
    /// The distance from the pad the rocket has to land within (m).
    pub field_radius: f64,
}

impl Default for Conditions {
    /// The kit's nominal flight: 1.52 m rail at 5°, 2.2 m/s breeze from 250°, low turbulence, a 120 m site at 15 °C, a 300 m field.
    fn default() -> Self {
        Conditions {
            rail_length: 1.52,
            launch_angle: 5.0 * std::f64::consts::PI / 180.0,
            wind_speed: 2.2,
            wind_from: 250.0 * std::f64::consts::PI / 180.0,
            turbulence: Turbulence::Low,
            site_altitude: 120.0,
            temperature: 288.15,
            field_radius: 300.0,
        }
    }
}

impl Conditions {
    /// Brings every value into range; a value that is not a number takes its default.
    fn sanitized(&self) -> Conditions {
        let d = Conditions::default();
        let pick = |v: f64, fallback: f64| if v.is_finite() { v } else { fallback };
        Conditions {
            rail_length: pick(self.rail_length, d.rail_length).max(0.05),
            launch_angle: pick(self.launch_angle, d.launch_angle).clamp(0.0, MAX_LAUNCH_ANGLE),
            wind_speed: pick(self.wind_speed, d.wind_speed).max(0.0),
            wind_from: pick(self.wind_from, d.wind_from).rem_euclid(std::f64::consts::TAU),
            turbulence: self.turbulence,
            site_altitude: pick(self.site_altitude, d.site_altitude).clamp(-400.0, 6000.0),
            temperature: pick(self.temperature, d.temperature).clamp(220.0, 330.0),
            field_radius: pick(self.field_radius, d.field_radius).max(1.0),
        }
    }
}

/// One stored flight.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlightDef {
    /// Stable id, unique within the rocket.
    pub id: FlightId,
    /// Name in the Simulate list.
    pub name: String,
    /// What it burns.
    pub motor: FlightMotor,
    /// Ejection delay (s) it flies; `None` takes the configuration's own.
    #[serde(default)]
    pub delay: Option<f64>,
    /// Launch site and weather.
    #[serde(default)]
    pub conditions: Conditions,
}

impl FlightDef {
    /// The default flight, named `Nominal`, on the active configuration with [`Conditions::default`].
    pub fn nominal(id: FlightId) -> FlightDef {
        FlightDef { id, name: "Nominal".to_string(), motor: FlightMotor::Active, delay: None, conditions: Conditions::default() }
    }
}

/// What a new document holds: the `Nominal` flight.
pub(crate) fn default_flights() -> Vec<FlightDef> {
    vec![FlightDef::nominal(1)]
}

/// The kit's four example flights (SimulateScreen): Nominal, Windy (7 m/s), Hot day (35 °C) and Bigger motor (an AeroTech I284W in
/// the first mount). The shipped example rockets store them; a new document has only `Nominal`.
pub fn example_flights() -> Vec<FlightDef> {
    let nominal = FlightDef::nominal(1);
    let with = |id, name: &str, f: &dyn Fn(&mut FlightDef)| {
        let mut def = FlightDef { id, name: name.to_string(), ..FlightDef::nominal(id) };
        f(&mut def);
        def
    };
    vec![
        nominal,
        with(2, "Windy", &|d| d.conditions.wind_speed = 7.0),
        with(3, "Hot day", &|d| d.conditions.temperature = 273.15 + 35.0),
        with(4, "Bigger motor", &|d| d.motor = FlightMotor::Motor(MotorRef::new("I284W", "AeroTech"))),
    ]
}

/// Most panels the Simulate plots hold.
pub const MAX_PLOT_PANELS: usize = 8;

/// Most data sources (lines) one plot panel overlays.
pub const MAX_PANEL_SOURCES: usize = 4;

/// One plot of the Simulate screen: the data sources it draws, by stable key (`"altitude"`, `"mach"`, ...). The keys belong to the
/// app's channel table; the model only stores them. The first source names the panel; every source of a panel has the same unit,
/// because a plot has one y axis.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlotPanel {
    /// Source keys, first is the panel's primary source. Never empty, never repeated, at most [`MAX_PANEL_SOURCES`].
    pub sources: Vec<String>,
}

impl PlotPanel {
    /// A panel drawing `sources`.
    pub fn new<S: AsRef<str>>(sources: &[S]) -> PlotPanel {
        PlotPanel { sources: sources.iter().map(|s| s.as_ref().to_string()).collect() }
    }
}

/// What the Simulate plots draw along x. Time is the default; the others make a plot parametric in time (a flight's altitude
/// rises and falls, so a curve against it doubles back on itself).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlotX {
    /// Flight time, s.
    #[default]
    Time,
    /// Altitude above the pad, m.
    Altitude,
    /// Mach number.
    Mach,
    /// Ground distance from the pad, m.
    Distance,
}

impl PlotX {
    /// Every choice, in selector order.
    pub const ALL: [PlotX; 4] = [PlotX::Time, PlotX::Altitude, PlotX::Mach, PlotX::Distance];

    fn is_time(&self) -> bool {
        *self == PlotX::Time
    }
}

/// The plots of the Simulate screen, top to bottom, saved with the document. The default is the kit's flight profile: altitude,
/// vertical velocity, vertical acceleration against time (and is not written to the file).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlotLayout {
    /// The panels, top to bottom. Never empty, at most [`MAX_PLOT_PANELS`].
    pub panels: Vec<PlotPanel>,
    /// What every panel draws along x (absent from files written before the choice existed: time).
    #[serde(default, skip_serializing_if = "PlotX::is_time")]
    pub x: PlotX,
}

impl Default for PlotLayout {
    fn default() -> Self {
        PlotLayout {
            panels: vec![PlotPanel::new(&["altitude"]), PlotPanel::new(&["vertical_velocity"]), PlotPanel::new(&["vertical_acceleration"])],
            x: PlotX::Time,
        }
    }
}

impl PlotLayout {
    /// Brings the layout into range: repeated sources and empty panels are dropped, panels and sources are capped, and a layout
    /// with nothing left becomes the default.
    pub fn sanitized(mut self) -> PlotLayout {
        for panel in &mut self.panels {
            let mut seen = std::collections::HashSet::new();
            panel.sources.retain(|s| seen.insert(s.clone()));
            panel.sources.truncate(MAX_PANEL_SOURCES);
        }
        self.panels.retain(|p| !p.sources.is_empty());
        self.panels.truncate(MAX_PLOT_PANELS);
        if self.panels.is_empty() {
            PlotLayout::default()
        } else {
            self
        }
    }
}

/// Whether `layout` is the default, so it need not be written to the file.
pub(crate) fn plots_are_default(layout: &PlotLayout) -> bool {
    *layout == PlotLayout::default()
}

impl Rocket {
    /// The Simulate plots' layout (which sources each panel draws).
    pub fn plot_layout(&self) -> &PlotLayout {
        &self.plots
    }

    /// Replaces the Simulate plots' layout; it is repaired as [`PlotLayout::sanitized`] does.
    pub fn set_plot_layout(&mut self, layout: PlotLayout) {
        self.plots = layout.sanitized();
    }

    /// Makes the plot layout valid (see [`PlotLayout::sanitized`]).
    pub(crate) fn repair_plots(&mut self) {
        self.plots = std::mem::take(&mut self.plots).sanitized();
    }

    /// The stored flights, in list order. Never empty.
    pub fn flights(&self) -> &[FlightDef] {
        &self.flights
    }

    /// Replaces the stored flights. The list is repaired the way loading repairs it (see [`Rocket::from_json`]): duplicate ids
    /// get fresh ones, empty names are numbered, conditions are brought into range, a configuration that does not exist falls
    /// back to the active one, and an empty list becomes the `Nominal` flight.
    pub fn set_flights(&mut self, flights: Vec<FlightDef>) {
        self.flights = flights;
        self.repair_flights();
    }

    /// Makes the flight list valid (see [`set_flights`](Self::set_flights)).
    pub(crate) fn repair_flights(&mut self) {
        let mut flights = std::mem::take(&mut self.flights);
        if flights.is_empty() {
            flights = default_flights();
        }
        let mut seen = std::collections::HashSet::new();
        let mut next = flights.iter().map(|f| f.id).max().map_or(1, |m| m + 1);
        for (i, f) in flights.iter_mut().enumerate() {
            if !seen.insert(f.id) {
                f.id = next;
                next += 1;
                seen.insert(f.id);
            }
            if f.name.trim().is_empty() {
                f.name = format!("Flight {}", i + 1);
            }
            if let FlightMotor::Config(id) = f.motor {
                if self.configuration(id).is_none() {
                    f.motor = FlightMotor::Active;
                }
            }
            if f.delay.is_some_and(|d| !d.is_finite() || d < 0.0) {
                f.delay = None;
            }
            f.conditions = f.conditions.sanitized();
        }
        self.flights = flights;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::json::FORMAT_VERSION;

    fn windy() -> FlightDef {
        FlightDef {
            id: 2,
            name: "Windy".to_string(),
            motor: FlightMotor::Motor(MotorRef::new("I284W", "AeroTech")),
            delay: Some(6.0),
            conditions: Conditions { wind_speed: 7.0, turbulence: Turbulence::High, ..Conditions::default() },
        }
    }

    #[test]
    fn a_new_document_has_one_nominal_flight() {
        let r = Rocket::default();
        assert_eq!(r.flights(), [FlightDef::nominal(1)]);
        let c = &r.flights()[0].conditions;
        assert_eq!((c.rail_length, c.wind_speed, c.site_altitude, c.field_radius), (1.52, 2.2, 120.0, 300.0));
        assert!((c.launch_angle - 5f64.to_radians()).abs() < 1e-12 && (c.temperature - 288.15).abs() < 1e-12);
    }

    #[test]
    fn flights_round_trip_through_json() {
        let mut r = Rocket::default();
        let config = r.active_config_id();
        let mut flights = vec![FlightDef::nominal(1), windy()];
        flights.push(FlightDef { id: 3, name: "On config".to_string(), motor: FlightMotor::Config(config), ..FlightDef::nominal(3) });
        r.set_flights(flights.clone());
        let json = r.to_json();
        assert!(json.contains("\"Windy\"") && json.contains("I284W"));
        let back = Rocket::from_json(&json).unwrap();
        assert_eq!(back.flights(), flights);
        assert_eq!(back, r);
    }

    #[test]
    fn a_document_without_flights_loads_with_nominal() {
        // Files written before flights were stored have no `flights` key (the migration is the serde default).
        let mut json: serde_json::Value = serde_json::from_str(&Rocket::default().to_json()).unwrap();
        assert!(json["rocket"].as_object_mut().unwrap().remove("flights").is_some());
        assert!(json["format_version"].as_u64().is_some_and(|v| v >= 1 && v <= FORMAT_VERSION as u64));
        let back = Rocket::from_json(&json.to_string()).unwrap();
        assert_eq!(back.flights(), [FlightDef::nominal(1)]);
        // An empty list repairs to the same.
        json["rocket"]["flights"] = serde_json::json!([]);
        assert_eq!(Rocket::from_json(&json.to_string()).unwrap().flights(), [FlightDef::nominal(1)]);
    }

    #[test]
    fn loading_repairs_a_bad_flight_list() {
        let mut r = Rocket::default();
        let bad = FlightDef {
            id: 1,
            name: " ".to_string(),
            motor: FlightMotor::Config(4242),
            delay: Some(f64::NAN),
            conditions: Conditions { wind_speed: -3.0, launch_angle: 9.0, temperature: 5.0, wind_from: -std::f64::consts::FRAC_PI_2, ..Conditions::default() },
        };
        r.set_flights(vec![FlightDef::nominal(1), bad]);
        let fixed = r.flights();
        assert_eq!(fixed.len(), 2);
        assert_ne!(fixed[0].id, fixed[1].id, "a duplicate id is replaced");
        assert_eq!(fixed[1].name, "Flight 2");
        assert_eq!(fixed[1].motor, FlightMotor::Active, "a configuration that does not exist is dropped");
        assert_eq!(fixed[1].delay, None);
        let c = &fixed[1].conditions;
        assert_eq!((c.wind_speed, c.launch_angle, c.temperature), (0.0, MAX_LAUNCH_ANGLE, 220.0));
        assert!((c.wind_from - 1.5 * std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn the_default_plot_layout_is_the_flight_profile_and_is_not_written() {
        let r = Rocket::default();
        let sources: Vec<&str> = r.plot_layout().panels.iter().map(|p| p.sources[0].as_str()).collect();
        assert_eq!(sources, ["altitude", "vertical_velocity", "vertical_acceleration"]);
        assert!(!r.to_json().contains("\"plots\""), "the default layout stays out of the file");
        // A file from before the layout existed loads with the default.
        assert_eq!(Rocket::from_json(&r.to_json()).unwrap().plot_layout(), &PlotLayout::default());
    }

    #[test]
    fn a_custom_plot_layout_round_trips_through_json() {
        let mut r = Rocket::default();
        let layout = PlotLayout {
            panels: vec![PlotPanel::new(&["mach"]), PlotPanel::new(&["thrust", "drag", "weight"]), PlotPanel::new(&["cg_station", "cp_station"])],
            x: PlotX::Mach,
        };
        r.set_plot_layout(layout.clone());
        let json = r.to_json();
        assert!(json.contains("\"plots\"") && json.contains("cp_station"));
        let back = Rocket::from_json(&json).unwrap();
        assert_eq!(back.plot_layout(), &layout);
        assert!(json.contains("\"x\": \"mach\"") || json.contains("\"x\":\"mach\""), "the x axis is saved");
        assert_eq!(back, r);
    }

    #[test]
    fn plot_layouts_are_repaired() {
        let mut r = Rocket::default();
        // Repeats, empty panels and too many sources or panels are cleaned; nothing left means the default.
        r.set_plot_layout(PlotLayout {
            panels: vec![PlotPanel::new(&["a", "a", "b", "c", "d", "e"]), PlotPanel { sources: Vec::new() }, PlotPanel::new(&["f"])],
            x: PlotX::Time,
        });
        let panels = &r.plot_layout().panels;
        assert_eq!(panels.len(), 2);
        assert_eq!(panels[0].sources, ["a", "b", "c", "d"]);
        r.set_plot_layout(PlotLayout { panels: (0..20).map(|i| PlotPanel::new(&[format!("s{i}")])).collect(), x: PlotX::Time });
        assert_eq!(r.plot_layout().panels.len(), MAX_PLOT_PANELS);
        r.set_plot_layout(PlotLayout { panels: Vec::new(), x: PlotX::Time });
        assert_eq!(r.plot_layout(), &PlotLayout::default());
        // The same repair runs on load.
        let mut json: serde_json::Value = serde_json::from_str(&Rocket::default().to_json()).unwrap();
        json["rocket"]["plots"] = serde_json::json!({ "panels": [{ "sources": [] }] });
        assert_eq!(Rocket::from_json(&json.to_string()).unwrap().plot_layout(), &PlotLayout::default());
    }
}
