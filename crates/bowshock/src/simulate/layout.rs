//! The plots' layout: which sources each panel draws ([`PlotLayout`], stored in the document), the presets, and the edits the
//! panel chrome and the source picker make. Everything here is a pure function of the layout, so it is tested without a window.
//!
//! The rules (design/components/PlotZoom, SimulateScreen, SimulateCompare): a panel has **one y-axis**, so every source of a panel
//! measures the same thing ([`Dim`]); at most [`MAX_PANEL_SOURCES`] lines share a panel (one line style each: solid, dashed,
//! dotted, dash-dot) and at most [`MAX_PLOT_PANELS`] panels stack; there is always at least one panel.

use bowshock_model::{PlotLayout, PlotPanel, MAX_PANEL_SOURCES, MAX_PLOT_PANELS};

use super::channels::{self, Dim};
use crate::units::UnitSystem;

/// A ready-made layout (the presets row above the plots).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    /// Altitude, vertical velocity, vertical acceleration: the kit's default.
    FlightProfile,
    /// Mach, the drag-coefficient build-up, dynamic pressure.
    Aerodynamics,
    /// Thrust against drag, mass and propellant, total acceleration.
    Forces,
    /// CG against CP, the static margin, Mach.
    Stability,
    /// The spin rate against the pitch frequency it must keep clear of, the airspeed that drives the spin, the angle of attack.
    Roll,
    /// Anything else: shown when the layout matches no preset. Choosing it starts a one-panel layout to build on.
    Custom,
}

impl Preset {
    /// The presets in row order.
    pub const ALL: [Preset; 6] = [Preset::FlightProfile, Preset::Aerodynamics, Preset::Forces, Preset::Stability, Preset::Roll, Preset::Custom];

    /// The button label.
    pub fn label(self) -> &'static str {
        match self {
            Preset::FlightProfile => "Flight profile",
            Preset::Aerodynamics => "Aerodynamics",
            Preset::Forces => "Forces",
            Preset::Stability => "Stability",
            Preset::Roll => "Roll",
            Preset::Custom => "Custom",
        }
    }

    /// The layout this preset sets (`Custom`: one altitude panel to build on).
    pub fn layout(self) -> PlotLayout {
        let panels: Vec<&[&str]> = match self {
            Preset::FlightProfile => return PlotLayout::default(),
            Preset::Aerodynamics => vec![&["mach"], &["cd", "cd_friction", "cd_pressure", "cd_base"], &["dynamic_pressure"]],
            Preset::Forces => vec![&["thrust", "drag"], &["mass", "propellant_mass"], &["total_acceleration"]],
            Preset::Stability => vec![&["cg_station", "cp_station"], &["stability_margin"], &["mach"]],
            Preset::Roll => vec![&["roll_rate", "pitch_frequency"], &["control_cant"], &["air_speed"], &["angle_of_attack"]],
            Preset::Custom => vec![&["altitude"]],
        };
        PlotLayout { panels: panels.into_iter().map(PlotPanel::new).collect(), ..PlotLayout::default() }
    }

    /// The preset `layout`'s panels are (whatever it plots against), `Custom` when they are none of them.
    pub fn of(layout: &PlotLayout) -> Preset {
        [Preset::FlightProfile, Preset::Aerodynamics, Preset::Forces, Preset::Stability, Preset::Roll]
            .into_iter()
            .find(|p| p.layout().panels == layout.panels)
            .unwrap_or(Preset::Custom)
    }
}

/// What a panel measures: the dimension of its first known source.
pub fn panel_dim(panel: &PlotPanel) -> Option<Dim> {
    panel.sources.iter().find_map(|k| channels::find(k)).map(|c| c.dim)
}

/// Makes `layout` drawable: unknown sources (a file from a newer version) and sources that cannot share their panel's axis are
/// dropped, then empty panels; a layout with nothing left becomes the default.
pub fn repair(layout: &mut PlotLayout) {
    for panel in &mut layout.panels {
        let dim = panel_dim(panel);
        panel.sources.retain(|k| channels::find(k).is_some_and(|c| dim.is_some_and(|d| c.joins(d))));
        let mut seen = Vec::new();
        panel.sources.retain(|k| {
            let new = !seen.contains(k);
            seen.push(k.clone());
            new
        });
        panel.sources.truncate(MAX_PANEL_SOURCES);
    }
    layout.panels.retain(|p| !p.sources.is_empty());
    layout.panels.truncate(MAX_PLOT_PANELS);
    if layout.panels.is_empty() {
        *layout = PlotLayout::default();
    }
}

/// Makes `key` the panel's primary source (the title). Sources that cannot share the new source's axis are dropped; the others
/// stay, and `key` is not repeated.
pub fn set_primary(layout: &mut PlotLayout, panel: usize, key: &str) {
    let (Some(channel), Some(p)) = (channels::find(key), layout.panels.get_mut(panel)) else { return };
    let mut sources = vec![key.to_string()];
    sources.extend(p.sources.iter().skip(1).filter(|k| k.as_str() != key && channels::find(k).is_some_and(|c| c.joins(channel.dim))).cloned());
    p.sources = sources;
}

/// Adds `key` to the panel, or removes it when the panel already draws it. Refused (with the reason) when it measures something
/// else, when the panel is full, or when it would leave the panel with no source.
pub fn toggle_source(layout: &mut PlotLayout, panel: usize, key: &str, units: UnitSystem) -> Result<(), String> {
    let channel = channels::find(key).ok_or_else(|| format!("Unknown source {key}"))?;
    let p = layout.panels.get_mut(panel).ok_or_else(|| "No such plot".to_string())?;
    if let Some(at) = p.sources.iter().position(|k| k == key) {
        if p.sources.len() == 1 {
            return Err("A plot needs at least one source".to_string());
        }
        p.sources.remove(at);
        return Ok(());
    }
    if let Some(dim) = panel_dim(p) {
        if let Some(reason) = channel.refusal(dim, units) {
            return Err(reason);
        }
    }
    if p.sources.len() >= MAX_PANEL_SOURCES {
        return Err(format!("A plot holds at most {MAX_PANEL_SOURCES} sources"));
    }
    p.sources.push(key.to_string());
    Ok(())
}

/// Sources a new panel tries first, most useful first.
const SUGGESTED: [&str; 8] = ["mach", "thrust", "drag", "dynamic_pressure", "mass", "stability_margin", "total_velocity", "air_density"];

/// Adds a panel at the bottom drawing a source no panel shows yet. `false` at [`MAX_PLOT_PANELS`].
pub fn add_panel(layout: &mut PlotLayout) -> bool {
    if layout.panels.len() >= MAX_PLOT_PANELS {
        return false;
    }
    let shown = |key: &str| layout.panels.iter().any(|p| p.sources.iter().any(|k| k == key));
    let key = SUGGESTED.iter().copied().chain(channels::CHANNELS.iter().map(|c| c.key)).find(|k| !shown(k)).unwrap_or("altitude");
    layout.panels.push(PlotPanel::new(&[key]));
    true
}

/// Removes panel `index`. `false` when it is the last one.
pub fn remove_panel(layout: &mut PlotLayout, index: usize) -> bool {
    if layout.panels.len() <= 1 || index >= layout.panels.len() {
        return false;
    }
    layout.panels.remove(index);
    true
}

/// Moves panel `index` one place up (towards the top) or down. `false` at the ends.
pub fn move_panel(layout: &mut PlotLayout, index: usize, up: bool) -> bool {
    let to = if up { index.checked_sub(1) } else { Some(index + 1) };
    match to.filter(|to| *to < layout.panels.len() && index < layout.panels.len()) {
        Some(to) => {
            layout.panels.swap(index, to);
            true
        }
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn panel(keys: &[&str]) -> PlotPanel {
        PlotPanel::new(keys)
    }

    fn layout(panels: &[&[&str]]) -> PlotLayout {
        PlotLayout { panels: panels.iter().map(|p| panel(p)).collect(), ..PlotLayout::default() }
    }

    #[test]
    fn every_preset_is_valid_and_recognised() {
        for preset in Preset::ALL {
            let mut l = preset.layout();
            let before = l.clone();
            repair(&mut l);
            assert_eq!(l, before, "{preset:?} has an unknown source or an axis clash");
            assert!(!l.panels.is_empty() && l.panels.len() <= MAX_PLOT_PANELS);
            if preset != Preset::Custom {
                assert_eq!(Preset::of(&l), preset);
            }
        }
        assert_eq!(Preset::FlightProfile.layout(), PlotLayout::default());
        assert_eq!(Preset::of(&layout(&[&["mach"]])), Preset::Custom);
        // The presets are distinct, so the highlighted one is never ambiguous.
        let all: Vec<PlotLayout> = Preset::ALL.iter().map(|p| p.layout()).collect();
        assert!(all.iter().enumerate().all(|(i, a)| all.iter().enumerate().all(|(j, b)| (a == b) == (i == j))));
    }

    #[test]
    fn replacing_the_primary_source_keeps_the_compatible_ones_and_drops_the_rest() {
        let mut l = layout(&[&["vertical_velocity", "total_velocity", "wind_speed"]]);
        // Another speed: everything stays, the new one leads.
        set_primary(&mut l, 0, "air_speed");
        assert_eq!(l.panels[0].sources, ["air_speed", "total_velocity", "wind_speed"]);
        // A source already in the panel is not repeated.
        set_primary(&mut l, 0, "wind_speed");
        assert_eq!(l.panels[0].sources, ["wind_speed", "total_velocity"]);
        // A force cannot share an axis with speeds: they go.
        set_primary(&mut l, 0, "thrust");
        assert_eq!(l.panels[0].sources, ["thrust"]);
        // Unknown keys and panels change nothing.
        set_primary(&mut l, 0, "nope");
        set_primary(&mut l, 9, "mach");
        assert_eq!(l.panels[0].sources, ["thrust"]);
    }

    #[test]
    fn a_series_joins_only_a_panel_with_the_same_unit() {
        let mut l = layout(&[&["altitude"]]);
        assert!(toggle_source(&mut l, 0, "ground_distance", UnitSystem::MM_G_S).is_ok());
        assert_eq!(l.panels[0].sources, ["altitude", "ground_distance"]);
        // A speed is refused, with the reason, and the panel is untouched.
        let why = toggle_source(&mut l, 0, "vertical_velocity", UnitSystem::MM_G_S).unwrap_err();
        assert!(why.contains("Vertical velocity") && why.contains("one y-axis"), "{why}");
        assert_eq!(l.panels[0].sources, ["altitude", "ground_distance"]);
        // Toggling a present source removes it, but never the last one.
        assert!(toggle_source(&mut l, 0, "ground_distance", UnitSystem::MM_G_S).is_ok());
        assert!(toggle_source(&mut l, 0, "altitude", UnitSystem::MM_G_S).unwrap_err().contains("at least one"));
        assert_eq!(l.panels[0].sources, ["altitude"]);
    }

    #[test]
    fn a_panel_holds_at_most_four_sources() {
        let mut l = layout(&[&["cd"]]);
        for key in ["cd_friction", "cd_pressure", "cd_base"] {
            assert!(toggle_source(&mut l, 0, key, UnitSystem::MM_G_S).is_ok());
        }
        assert_eq!(l.panels[0].sources.len(), MAX_PANEL_SOURCES);
        assert!(toggle_source(&mut l, 0, "cd_fins", UnitSystem::MM_G_S).unwrap_err().contains("at most 4"));
    }

    #[test]
    fn panels_are_added_removed_and_moved_within_limits() {
        let mut l = PlotLayout::default();
        assert!(add_panel(&mut l));
        assert_eq!(l.panels.len(), 4);
        // The new panel shows something no panel showed yet.
        assert_eq!(l.panels[3].sources, ["mach"]);
        assert!(add_panel(&mut l));
        assert_ne!(l.panels[4].sources, l.panels[3].sources);
        while add_panel(&mut l) {}
        assert_eq!(l.panels.len(), MAX_PLOT_PANELS);
        let keys: Vec<&String> = l.panels.iter().map(|p| &p.sources[0]).collect();
        assert!(keys.iter().enumerate().all(|(i, a)| keys.iter().enumerate().all(|(j, b)| (a == b) == (i == j))), "every added panel is new: {keys:?}");

        // Moves swap neighbours and stop at the ends.
        let mut l = PlotLayout::default();
        assert!(!move_panel(&mut l, 0, true) && !move_panel(&mut l, 2, false) && !move_panel(&mut l, 9, true));
        assert!(move_panel(&mut l, 1, true));
        assert_eq!(l.panels.iter().map(|p| p.sources[0].as_str()).collect::<Vec<_>>(), ["vertical_velocity", "altitude", "vertical_acceleration"]);
        assert!(move_panel(&mut l, 0, false));
        assert_eq!(l.panels[1].sources[0], "vertical_velocity");

        // The last panel stays.
        assert!(remove_panel(&mut l, 0) && remove_panel(&mut l, 0));
        assert!(!remove_panel(&mut l, 0));
        assert_eq!(l.panels.len(), 1);
    }

    #[test]
    fn repair_drops_unknown_and_clashing_sources_and_empty_panels() {
        let mut l = layout(&[&["future_channel", "altitude"], &["thrust", "mach", "thrust", "drag"], &["nothing_known"]]);
        repair(&mut l);
        // The first known source decides the panel's axis; later clashes and repeats go.
        assert_eq!(l.panels, [panel(&["altitude"]), panel(&["thrust", "drag"])]);
        let mut empty = layout(&[&["nothing_known"]]);
        repair(&mut empty);
        assert_eq!(empty, PlotLayout::default());
    }

    #[test]
    fn a_panel_measures_what_its_first_known_source_measures() {
        assert_eq!(panel_dim(&panel(&["mach", "thrust"])), Some(Dim::Mach));
        assert_eq!(panel_dim(&panel(&["unknown", "thrust"])), Some(Dim::Force));
        assert_eq!(panel_dim(&panel(&["unknown"])), None);
    }
}
