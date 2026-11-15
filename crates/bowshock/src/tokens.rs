//! Design tokens from `design/tokens.json`. Colours are per theme; sizes are shared.

use iced::Color;

/// Colour tokens of one theme (`design/tokens.css`). The full set is kept so it stays a 1:1 copy of the design.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct Tokens {
    pub bg_000: Color,
    pub bg_100: Color,
    pub bg_200: Color,
    pub bg_300: Color,
    pub bg_400: Color,
    pub line_100: Color,
    pub line_200: Color,
    pub ink: Color,
    pub ink_muted: Color,
    pub ink_faint: Color,
    pub accent: Color,
    pub accent_ink: Color,
    pub accent_soft: Color,
    pub cg: Color,
    pub cp: Color,
    pub ok: Color,
    pub warn: Color,
    pub err: Color,
    pub axis_x: Color,
    pub axis_y: Color,
    pub axis_z: Color,
    pub grid_minor: Color,
    pub grid_major: Color,
    pub part_body: Color,
    pub part_fin: Color,
    pub sys_aero: Color,
    pub sys_propulsion: Color,
    pub sys_recovery: Color,
    pub sys_avionics: Color,
    pub lane_structure: Color,
    pub lane_aero: Color,
    pub lane_propulsion: Color,
    pub lane_recovery: Color,
    pub lane_avionics: Color,
    pub cut_face: Color,
    pub cut_interior: Color,
}

const fn hex(rgb: u32) -> Color {
    Color::from_rgb8((rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8)
}

pub const DARK: Tokens = Tokens {
    bg_000: hex(0x0a0d11),
    bg_100: hex(0x10141a),
    bg_200: hex(0x161b22),
    bg_300: hex(0x1d232c),
    bg_400: hex(0x262e39),
    line_100: hex(0x232a34),
    line_200: hex(0x5a6a80),
    ink: hex(0xe6ebf1),
    ink_muted: hex(0xa1abb8),
    ink_faint: hex(0x7f8a99),
    accent: hex(0xff7a33),
    accent_ink: hex(0x1a0c03),
    accent_soft: Color::from_rgba8(255, 122, 51, 0.14),
    cg: hex(0x5aaeff),
    cp: hex(0xff5468),
    ok: hex(0x3fcf8e),
    warn: hex(0xf2b544),
    err: hex(0xff6b6b),
    axis_x: hex(0xe5534b),
    axis_y: hex(0x57c26a),
    axis_z: hex(0x4d8dff),
    grid_minor: hex(0x141920),
    grid_major: hex(0x1f2630),
    part_body: hex(0xc9d1db),
    part_fin: hex(0x9fb0c3),
    sys_aero: hex(0x5cc9c0),
    sys_propulsion: hex(0xe07ad2),
    sys_recovery: hex(0xc4b8ff),
    sys_avionics: hex(0xa8d46f),
    lane_structure: hex(0x36404d),
    lane_aero: hex(0x1f3a3a),
    lane_propulsion: hex(0x3d2540),
    lane_recovery: hex(0x302c48),
    lane_avionics: hex(0x2c3a22),
    cut_face: hex(0x8794a5),
    cut_interior: hex(0x141920),
};

pub const LIGHT: Tokens = Tokens {
    bg_000: hex(0xe9edf2),
    bg_100: hex(0xf3f5f8),
    bg_200: hex(0xffffff),
    bg_300: hex(0xf6f8fa),
    bg_400: hex(0xe8ecf1),
    line_100: hex(0xe1e6ec),
    line_200: hex(0x7a8594),
    ink: hex(0x151a21),
    ink_muted: hex(0x4c5664),
    ink_faint: hex(0x5f6a78),
    accent: hex(0xb4460c),
    accent_ink: hex(0xffffff),
    accent_soft: Color::from_rgba8(180, 70, 12, 0.10),
    cg: hex(0x1d66c2),
    cp: hex(0xa81c30),
    ok: hex(0x16794c),
    warn: hex(0x8a5a00),
    err: hex(0xc62828),
    axis_x: hex(0xc0392b),
    axis_y: hex(0x2e7d32),
    axis_z: hex(0x1f5fbf),
    grid_minor: hex(0xdde2e8),
    grid_major: hex(0xcdd4dc),
    part_body: hex(0xb9c2cd),
    part_fin: hex(0x8fa0b4),
    sys_aero: hex(0x11807a),
    sys_propulsion: hex(0xa3348f),
    sys_recovery: hex(0x5a45d6),
    sys_avionics: hex(0x4d7d16),
    lane_structure: hex(0xd5dbe2),
    lane_aero: hex(0xd3ecea),
    lane_propulsion: hex(0xf1d8ec),
    lane_recovery: hex(0xe3defa),
    lane_avionics: hex(0xe1edd2),
    cut_face: hex(0x6b7787),
    cut_interior: hex(0xdfe4ea),
};

// Sizes in logical px.
pub const SPACE_1: f32 = 4.0;
pub const SPACE_2: f32 = 8.0;
pub const SPACE_3: f32 = 12.0;
pub const SPACE_4: f32 = 16.0;
pub const CONTROL_H: f32 = 26.0;
pub const TOPBAR_H: f32 = 40.0;
pub const STATUSBAR_H: f32 = 36.0;
pub const RIGHT_PANEL_W: f32 = 304.0;
pub const RAIL_W: f32 = 44.0;
pub const BIN_W: f32 = 232.0;
pub const LANES_H: f32 = 176.0;
pub const PART_ROW_H: f32 = 44.0;

// Type sizes.
pub const TEXT_LABEL: f32 = 11.0;
pub const TEXT_BODY: f32 = 13.0;
