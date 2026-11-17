//! The colour maps of the flow viewer: small tables of stops, sampled by linear interpolation in sRGB.
//!
//! * [`VIRIDIS`] (Mach number): the nine stops of matplotlib's `viridis` at 0, 1/8 ... 1, as R's `viridis(9)` lists them (van der
//!   Walt and Smith, matplotlib 2015, CC0). Perceptually uniform, and its lightness rises monotonically.
//! * [`RDBU`] (pressure coefficient): ColorBrewer's 11-class `RdBu` (Harrower and Brewer 2003, Apache 2.0) run from blue (suction)
//!   through white (the free-stream pressure) to red (compression). It is not uniform in the strict sense, but its lightness rises
//!   from each end to the neutral centre, which is what a map that is read about zero needs.
//! * [`GREY`] (schlieren): a grey ramp from near black to near white, reversed in the light theme so that the undisturbed flow is
//!   the quiet end against the screen.
//!
//! The tests check the lightness (CIE L*) of each is monotone.

use iced::Color;

/// Matplotlib viridis, nine stops.
pub const VIRIDIS: [[u8; 3]; 9] = [
    [0x44, 0x01, 0x54],
    [0x47, 0x2D, 0x7B],
    [0x3B, 0x52, 0x8B],
    [0x2C, 0x72, 0x8E],
    [0x21, 0x90, 0x8C],
    [0x27, 0xAD, 0x81],
    [0x5D, 0xC8, 0x63],
    [0xAA, 0xDC, 0x32],
    [0xFD, 0xE7, 0x25],
];

/// ColorBrewer RdBu (11 classes) from the most negative pressure coefficient (blue) to the most positive (red).
pub const RDBU: [[u8; 3]; 11] = [
    [0x05, 0x30, 0x61],
    [0x21, 0x66, 0xAC],
    [0x43, 0x93, 0xC3],
    [0x92, 0xC5, 0xDE],
    [0xD1, 0xE5, 0xF0],
    [0xF7, 0xF7, 0xF7],
    [0xFD, 0xDB, 0xC7],
    [0xF4, 0xA5, 0x82],
    [0xD6, 0x60, 0x4D],
    [0xB2, 0x18, 0x2B],
    [0x67, 0x00, 0x1F],
];

/// The schlieren ramp for the dark theme (undisturbed flow dark, strong gradients bright).
pub const GREY_DARK: [[u8; 3]; 2] = [[0x0C, 0x0E, 0x11], [0xF4, 0xF5, 0xF7]];
/// The same for the light theme, reversed.
pub const GREY_LIGHT: [[u8; 3]; 2] = [[0xF4, 0xF5, 0xF7], [0x0C, 0x0E, 0x11]];

/// What a picture is coloured by.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scalar {
    /// Mach number, from 0.
    Mach,
    /// Pressure coefficient, symmetric about 0.
    Cp,
    /// Magnitude of the density gradient on a log scale.
    Schlieren,
}

impl Scalar {
    /// The three, in the order of the buttons (and of a vertex's values).
    pub const ALL: [Scalar; 3] = [Scalar::Mach, Scalar::Cp, Scalar::Schlieren];

    /// The button label.
    pub fn name(self) -> &'static str {
        match self {
            Scalar::Mach => "Mach",
            Scalar::Cp => "Cp",
            Scalar::Schlieren => "Schlieren",
        }
    }

    /// Index among [`Scalar::ALL`].
    pub fn index(self) -> usize {
        self as usize
    }

    /// The stops this scalar is coloured with.
    pub fn stops(self, dark: bool) -> &'static [[u8; 3]] {
        match (self, dark) {
            (Scalar::Mach, _) => &VIRIDIS,
            (Scalar::Cp, _) => &RDBU,
            (Scalar::Schlieren, true) => &GREY_DARK,
            (Scalar::Schlieren, false) => &GREY_LIGHT,
        }
    }
}

/// The colour (sRGB, 0 to 1) of `stops` at `t` in `[0, 1]` (clamped), interpolated linearly between the stops.
pub fn sample(stops: &[[u8; 3]], t: f32) -> [f32; 3] {
    let t = if t.is_nan() { 0.0 } else { t.clamp(0.0, 1.0) };
    let at = t * (stops.len() - 1) as f32;
    let i = (at.floor() as usize).min(stops.len() - 2);
    let f = at - i as f32;
    let lerp = |c: usize| (f32::from(stops[i][c]) * (1.0 - f) + f32::from(stops[i + 1][c]) * f) / 255.0;
    [lerp(0), lerp(1), lerp(2)]
}

/// [`sample`] as an iced colour.
pub fn color(stops: &[[u8; 3]], t: f32) -> Color {
    let [r, g, b] = sample(stops, t);
    Color::from_rgb(r, g, b)
}

/// Entries of the table the shader looks the colour up in.
pub const LUT: usize = 64;

/// `stops` sampled at [`LUT`] evenly spaced points as the packed (linear for an sRGB target) components the GPU wants, like the
/// rest of the UI's colours.
pub fn lut(stops: &[[u8; 3]]) -> [[f32; 4]; LUT] {
    std::array::from_fn(|k| iced::advanced::graphics::color::pack(color(stops, k as f32 / (LUT - 1) as f32)).components())
}

#[cfg(test)]
/// CIE L* (0 to 100) of an sRGB colour (0 to 1 per channel), D65.
pub fn lightness(rgb: [f32; 3]) -> f32 {
    let linear = |c: f32| if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) };
    let y = 0.2126 * linear(rgb[0]) + 0.7152 * linear(rgb[1]) + 0.0722 * linear(rgb[2]);
    if y > 216.0 / 24389.0 {
        116.0 * y.cbrt() - 16.0
    } else {
        24389.0 / 27.0 * y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn along(stops: &[[u8; 3]]) -> Vec<f32> {
        (0..=100).map(|k| lightness(sample(stops, k as f32 / 100.0))).collect()
    }

    #[test]
    fn the_sequential_and_the_schlieren_ramps_are_monotone_in_lightness() {
        let up = |l: &[f32]| l.windows(2).all(|w| w[1] >= w[0] - 1e-3);
        assert!(up(&along(&VIRIDIS)), "viridis rises");
        assert!(up(&along(&GREY_DARK)), "the dark schlieren ramp rises");
        let light = along(&GREY_LIGHT);
        assert!(light.windows(2).all(|w| w[1] <= w[0] + 1e-3), "the light schlieren ramp falls");
        // the ends are far apart: a map that can be read
        assert!(along(&VIRIDIS)[100] - along(&VIRIDIS)[0] > 60.0);
    }

    #[test]
    fn the_pressure_map_gets_lighter_towards_zero_from_both_ends() {
        let l = along(&RDBU);
        let mid = 50;
        assert!(l[..=mid].windows(2).all(|w| w[1] >= w[0] - 1e-3), "blue side rises to the neutral centre");
        assert!(l[mid..].windows(2).all(|w| w[1] <= w[0] + 1e-3), "red side falls from the neutral centre");
        assert!(l[mid] > 95.0, "the centre is white: {}", l[mid]);
        // blue below, red above
        let (lo, hi) = (sample(&RDBU, 0.0), sample(&RDBU, 1.0));
        assert!(lo[2] > lo[0] && hi[0] > hi[2]);
    }

    #[test]
    fn sampling_hits_the_stops_clamps_and_survives_nan() {
        assert_eq!(sample(&VIRIDIS, 0.0), [68.0 / 255.0, 1.0 / 255.0, 84.0 / 255.0]);
        assert_eq!(sample(&VIRIDIS, 1.0), sample(&VIRIDIS, 7.0));
        assert_eq!(sample(&VIRIDIS, f32::NAN), sample(&VIRIDIS, 0.0));
        let mid = sample(&GREY_DARK, 0.5);
        assert!((mid[0] - (12.0 + 244.0) / 2.0 / 255.0).abs() < 1e-6);
        assert_eq!(lut(&VIRIDIS).len(), LUT);
    }
}
