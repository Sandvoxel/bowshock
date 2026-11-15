//! Bowshock design tokens for iced. Generated from tokens.json; don't edit by hand.
//! Colours: `Palette::DARK` / `Palette::LIGHT`. Sizes in logical pixels.
use iced::Color;

#[derive(Debug, Clone, Copy)]
pub struct Palette {
    /// 3D viewport ground and 2D profile canvas. The deepest surface, so the rocket reads as the brightest object.
    pub bg_000: Color,
    /// App window background, top bar, status strip.
    pub bg_100: Color,
    /// Side panels: parts bin, inspector, Station Lanes.
    pub bg_200: Color,
    /// Inputs, part tiles, raised rows inside a panel.
    pub bg_300: Color,
    /// Hover on rows, tiles and buttons.
    pub bg_400: Color,
    /// Hairlines: panel frames, section rules, lane separators, minor grid. Decorative only, never a control's only boundary.
    pub line_100: Color,
    /// Control borders and overlay frames (overlays have no shadow, just this 1px frame). 3:1 against bg-200.
    pub line_200: Color,
    /// Primary text and values on any bg.
    pub ink: Color,
    /// Field labels, secondary text, units suffixes.
    pub ink_muted: Color,
    /// Section headers (uppercase), placeholders, disabled values. 4.5:1 on bg-300.
    pub ink_faint: Color,
    /// Shock orange. Selection brackets, primary buttons, focus, the active tab underline, live drag values. One meaning: 'this is what you are acting on'.
    pub accent: Color,
    /// Text and icons on an accent fill.
    pub accent_ink: Color,
    /// Drop-zone and snap-target fill ONLY. Selection never uses a wash; it uses accent brackets and outlines.
    pub accent_soft: Color,
    /// Centre of gravity. Marker, readout and every CG plot line. Always blue, always the quartered-circle glyph.
    pub cg: Color,
    /// Centre of pressure. Marker, readout and every CP plot line. Always red, always the dot-in-ring glyph; never used for errors.
    pub cp: Color,
    /// Stable margin band, passing checks, valid attach node.
    pub ok: Color,
    /// Warnings: marginal stability, diameter mismatch, fin flutter risk.
    pub warn: Color,
    /// Errors: unstable, invalid geometry, illegal drop. Pair with an icon; never confuse with cp, which is a quantity, not a state.
    pub err: Color,
    /// Gizmo / axis triad X (radial).
    pub axis_x: Color,
    /// Gizmo / axis triad Y (radial).
    pub axis_y: Color,
    /// Gizmo / axis triad Z (along the rocket's axis, the one axis most handles move on).
    pub axis_z: Color,
    /// Viewport minor grid on bg-000.
    pub grid_minor: Color,
    /// Viewport major grid lines and the rocket axis centreline.
    pub grid_major: Color,
    /// Structure system colour, and the default airframe shading base in the viewport (nose, tubes, rings, bulkheads).
    pub part_body: Color,
    /// Default fin / flat-plate shading base in Solid view.
    pub part_fin: Color,
    /// Aero system: fins, canards, rail buttons, lugs. The edge of Aero bars in Station Lanes and Aero parts in X-ray / Cutaway.
    pub sys_aero: Color,
    /// Propulsion system: motor mount, motor, engine block, thrust ring, retainer. Lanes edge + parts in Cutaway / X-ray.
    pub sys_propulsion: Color,
    /// Recovery system: chutes, drogue, shock cord, harness, charges. Lanes edge + parts in Cutaway / X-ray.
    pub sys_recovery: Color,
    /// Avionics & payload system: altimeters, batteries, trackers, payload, ballast. Lanes edge + parts in Cutaway / X-ray.
    pub sys_avionics: Color,
    /// Station Lanes: Structure bar fill. Text on it is ink.
    pub lane_structure: Color,
    /// Station Lanes: Aero bar fill.
    pub lane_aero: Color,
    /// Station Lanes: Propulsion bar fill.
    pub lane_propulsion: Color,
    /// Station Lanes: Recovery bar fill.
    pub lane_recovery: Color,
    /// Station Lanes: Avionics & payload bar fill.
    pub lane_avionics: Color,
    /// Hatching on cut faces in Cutaway view (airframe walls, rings, bulkheads where the section plane passes).
    pub cut_face: Color,
    /// The inside of the airframe exposed by Cutaway, behind the internals.
    pub cut_interior: Color,
    /// Chart categorical slot 1 (aqua). Fixed order, never cycled. Validated with slots 2–4: CVD ΔE ≥ 8.4 dark / 9.1 light.
    pub series_1: Color,
    /// Chart categorical slot 2 (yellow). Below 3:1 on light, so ship direct labels or a table.
    pub series_2: Color,
    /// Chart categorical slot 3 (violet).
    pub series_3: Color,
    /// Chart categorical slot 4 (magenta). More than 4 series → fold into Other or use small multiples.
    pub series_4: Color,
}

impl Palette {
    pub const DARK: Palette = Palette {
    bg_000: Color::from_rgba8(10, 13, 17, 1.00),
    bg_100: Color::from_rgba8(16, 20, 26, 1.00),
    bg_200: Color::from_rgba8(22, 27, 34, 1.00),
    bg_300: Color::from_rgba8(29, 35, 44, 1.00),
    bg_400: Color::from_rgba8(38, 46, 57, 1.00),
    line_100: Color::from_rgba8(35, 42, 52, 1.00),
    line_200: Color::from_rgba8(90, 106, 128, 1.00),
    ink: Color::from_rgba8(230, 235, 241, 1.00),
    ink_muted: Color::from_rgba8(161, 171, 184, 1.00),
    ink_faint: Color::from_rgba8(127, 138, 153, 1.00),
    accent: Color::from_rgba8(255, 122, 51, 1.00),
    accent_ink: Color::from_rgba8(26, 12, 3, 1.00),
    accent_soft: Color::from_rgba8(255, 122, 51, 0.14),
    cg: Color::from_rgba8(90, 174, 255, 1.00),
    cp: Color::from_rgba8(255, 84, 104, 1.00),
    ok: Color::from_rgba8(63, 207, 142, 1.00),
    warn: Color::from_rgba8(242, 181, 68, 1.00),
    err: Color::from_rgba8(255, 107, 107, 1.00),
    axis_x: Color::from_rgba8(229, 83, 75, 1.00),
    axis_y: Color::from_rgba8(87, 194, 106, 1.00),
    axis_z: Color::from_rgba8(77, 141, 255, 1.00),
    grid_minor: Color::from_rgba8(20, 25, 32, 1.00),
    grid_major: Color::from_rgba8(31, 38, 48, 1.00),
    part_body: Color::from_rgba8(201, 209, 219, 1.00),
    part_fin: Color::from_rgba8(159, 176, 195, 1.00),
    sys_aero: Color::from_rgba8(92, 201, 192, 1.00),
    sys_propulsion: Color::from_rgba8(224, 122, 210, 1.00),
    sys_recovery: Color::from_rgba8(196, 184, 255, 1.00),
    sys_avionics: Color::from_rgba8(168, 212, 111, 1.00),
    lane_structure: Color::from_rgba8(54, 64, 77, 1.00),
    lane_aero: Color::from_rgba8(31, 58, 58, 1.00),
    lane_propulsion: Color::from_rgba8(61, 37, 64, 1.00),
    lane_recovery: Color::from_rgba8(48, 44, 72, 1.00),
    lane_avionics: Color::from_rgba8(44, 58, 34, 1.00),
    cut_face: Color::from_rgba8(135, 148, 165, 1.00),
    cut_interior: Color::from_rgba8(20, 25, 32, 1.00),
    series_1: Color::from_rgba8(25, 158, 112, 1.00),
    series_2: Color::from_rgba8(201, 133, 0, 1.00),
    series_3: Color::from_rgba8(144, 133, 233, 1.00),
    series_4: Color::from_rgba8(213, 81, 129, 1.00),
    };
    pub const LIGHT: Palette = Palette {
    bg_000: Color::from_rgba8(233, 237, 242, 1.00),
    bg_100: Color::from_rgba8(243, 245, 248, 1.00),
    bg_200: Color::from_rgba8(255, 255, 255, 1.00),
    bg_300: Color::from_rgba8(246, 248, 250, 1.00),
    bg_400: Color::from_rgba8(232, 236, 241, 1.00),
    line_100: Color::from_rgba8(225, 230, 236, 1.00),
    line_200: Color::from_rgba8(122, 133, 148, 1.00),
    ink: Color::from_rgba8(21, 26, 33, 1.00),
    ink_muted: Color::from_rgba8(76, 86, 100, 1.00),
    ink_faint: Color::from_rgba8(95, 106, 120, 1.00),
    accent: Color::from_rgba8(180, 70, 12, 1.00),
    accent_ink: Color::from_rgba8(255, 255, 255, 1.00),
    accent_soft: Color::from_rgba8(180, 70, 12, 0.10),
    cg: Color::from_rgba8(29, 102, 194, 1.00),
    cp: Color::from_rgba8(168, 28, 48, 1.00),
    ok: Color::from_rgba8(22, 121, 76, 1.00),
    warn: Color::from_rgba8(138, 90, 0, 1.00),
    err: Color::from_rgba8(198, 40, 40, 1.00),
    axis_x: Color::from_rgba8(192, 57, 43, 1.00),
    axis_y: Color::from_rgba8(46, 125, 50, 1.00),
    axis_z: Color::from_rgba8(31, 95, 191, 1.00),
    grid_minor: Color::from_rgba8(221, 226, 232, 1.00),
    grid_major: Color::from_rgba8(205, 212, 220, 1.00),
    part_body: Color::from_rgba8(185, 194, 205, 1.00),
    part_fin: Color::from_rgba8(143, 160, 180, 1.00),
    sys_aero: Color::from_rgba8(17, 128, 122, 1.00),
    sys_propulsion: Color::from_rgba8(163, 52, 143, 1.00),
    sys_recovery: Color::from_rgba8(90, 69, 214, 1.00),
    sys_avionics: Color::from_rgba8(77, 125, 22, 1.00),
    lane_structure: Color::from_rgba8(213, 219, 226, 1.00),
    lane_aero: Color::from_rgba8(211, 236, 234, 1.00),
    lane_propulsion: Color::from_rgba8(241, 216, 236, 1.00),
    lane_recovery: Color::from_rgba8(227, 222, 250, 1.00),
    lane_avionics: Color::from_rgba8(225, 237, 210, 1.00),
    cut_face: Color::from_rgba8(107, 119, 135, 1.00),
    cut_interior: Color::from_rgba8(223, 228, 234, 1.00),
    series_1: Color::from_rgba8(27, 175, 122, 1.00),
    series_2: Color::from_rgba8(237, 161, 0, 1.00),
    series_3: Color::from_rgba8(98, 80, 214, 1.00),
    series_4: Color::from_rgba8(232, 123, 164, 1.00),
    };
}

/// Spacing, radius and layout sizes.
pub mod size {
    pub const SPACE_1: f32 = 4.0;
    pub const SPACE_2: f32 = 8.0;
    pub const SPACE_3: f32 = 12.0;
    pub const SPACE_4: f32 = 16.0;
    pub const SPACE_6: f32 = 24.0;
    pub const SPACE_8: f32 = 32.0;
    pub const RADIUS_0: f32 = 0.0;
    pub const RADIUS_1: f32 = 1.0;
    pub const CONTROL_H: f32 = 26.0;
    pub const TOPBAR_H: f32 = 40.0;
    pub const STATUSBAR_H: f32 = 36.0;
    pub const RIGHT_PANEL_W: f32 = 304.0;
    pub const RAIL_W: f32 = 44.0;
    pub const BIN_W: f32 = 232.0;
    pub const PART_ROW_H: f32 = 44.0;
    pub const LANES_H: f32 = 206.0;
    pub const LANE_H: f32 = 28.0;
    pub const BRACKET: f32 = 8.0;
}

#[derive(Debug, Clone, Copy)]
pub struct TextStyle { pub size: f32, pub line_height: f32, pub weight: u16, pub mono: bool }

/// Type styles. `mono: true` means IBM Plex Mono with tabular figures.
pub mod text {
    use super::TextStyle;
    pub const UI_TITLE: TextStyle = TextStyle { size: 15.0, line_height: 20.0, weight: 600, mono: false };
    pub const UI_BODY: TextStyle = TextStyle { size: 13.0, line_height: 18.0, weight: 400, mono: false };
    pub const UI_STRONG: TextStyle = TextStyle { size: 13.0, line_height: 18.0, weight: 600, mono: false };
    pub const UI_SMALL: TextStyle = TextStyle { size: 12.0, line_height: 16.0, weight: 400, mono: false };
    pub const UI_SECTION: TextStyle = TextStyle { size: 10.5, line_height: 14.0, weight: 500, mono: false };
    pub const NUM_BODY: TextStyle = TextStyle { size: 12.5, line_height: 18.0, weight: 500, mono: true };
    pub const NUM_READOUT: TextStyle = TextStyle { size: 20.0, line_height: 24.0, weight: 600, mono: true };
    pub const NUM_LABEL: TextStyle = TextStyle { size: 11.0, line_height: 14.0, weight: 500, mono: true };
}

pub const FONT_SANS: &str = "IBM Plex Sans";
pub const FONT_MONO: &str = "IBM Plex Mono";
