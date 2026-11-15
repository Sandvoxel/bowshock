//! International Standard Atmosphere (ISA 1976, 0-84.852 km geopotential) with a site
//! temperature offset.
//!
//! The site is described by its altitude above sea level and the ground temperature measured
//! there. The difference between that temperature and the ISA temperature at the site is applied
//! as a constant offset at every altitude; pressure follows the standard ISA profile and density
//! is recomputed from the ideal gas law with the offset temperature.

/// Specific gas constant of dry air (J/(kg K)).
pub const R_AIR: f64 = 287.052_87;
/// Ratio of specific heats of air.
pub const GAMMA: f64 = 1.4;
const G0: f64 = 9.806_65;
const EARTH_R: f64 = 6_356_766.0;

/// `(geopotential base altitude m, base temperature K, lapse rate K/m, base pressure Pa)`.
const LAYERS: [(f64, f64, f64, f64); 7] = [
    (0.0, 288.15, -0.0065, 101_325.0),
    (11_000.0, 216.65, 0.0, 22_632.06),
    (20_000.0, 216.65, 0.001, 5_474.889),
    (32_000.0, 228.65, 0.0028, 868.018_7),
    (47_000.0, 270.65, 0.0, 110.906_3),
    (51_000.0, 270.65, -0.0028, 66.938_87),
    (71_000.0, 214.65, -0.002, 3.956_420),
];

/// Thermodynamic state of the air at one point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirState {
    /// Static temperature (K).
    pub temperature: f64,
    /// Static pressure (Pa).
    pub pressure: f64,
    /// Density (kg/m^3).
    pub density: f64,
    /// Speed of sound (m/s).
    pub speed_of_sound: f64,
    /// Dynamic viscosity, Sutherland's law (Pa s).
    pub viscosity: f64,
}

impl AirState {
    /// Kinematic viscosity (m^2/s).
    pub fn kinematic_viscosity(&self) -> f64 {
        self.viscosity / self.density
    }
}

/// Atmosphere for a launch site.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Atmosphere {
    /// Site altitude above sea level (m).
    pub site_altitude: f64,
    /// Temperature offset from ISA (K), applied at all altitudes.
    pub temperature_offset: f64,
}

impl Default for Atmosphere {
    fn default() -> Self {
        Self::standard()
    }
}

/// ISA temperature and pressure at geometric altitude `h` (m above sea level).
fn isa(h: f64) -> (f64, f64) {
    let h = h.clamp(-500.0, 84_000.0);
    let hg = EARTH_R * h / (EARTH_R + h);
    let (hb, tb, lb, pb) = LAYERS.iter().rev().find(|l| hg >= l.0).copied().unwrap_or(LAYERS[0]);
    let dh = hg - hb;
    let t = tb + lb * dh;
    let p = if lb.abs() < 1e-12 { pb * (-G0 * dh / (R_AIR * tb)).exp() } else { pb * (t / tb).powf(-G0 / (R_AIR * lb)) };
    (t, p)
}

impl Atmosphere {
    /// Standard atmosphere at sea level (15 C ground temperature).
    pub fn standard() -> Self {
        Self { site_altitude: 0.0, temperature_offset: 0.0 }
    }

    /// Atmosphere for a site at `site_altitude` (m above sea level) whose ground temperature is
    /// `ground_temperature` (K).
    pub fn new(site_altitude: f64, ground_temperature: f64) -> Self {
        let (t_isa, _) = isa(site_altitude);
        Self { site_altitude, temperature_offset: ground_temperature - t_isa }
    }

    /// State at altitude `h` metres above sea level.
    pub fn at_msl(&self, h: f64) -> AirState {
        let (t_isa, p) = isa(h);
        let temperature = (t_isa + self.temperature_offset).max(100.0);
        let density = p / (R_AIR * temperature);
        let speed_of_sound = (GAMMA * R_AIR * temperature).sqrt();
        let viscosity = 1.458e-6 * temperature.powf(1.5) / (temperature + 110.4);
        AirState { temperature, pressure: p, density, speed_of_sound, viscosity }
    }

    /// State at `h` metres above the launch site.
    pub fn at_agl(&self, h: f64) -> AirState {
        self.at_msl(self.site_altitude + h)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, rel: f64) {
        assert!((a - b).abs() <= rel * b.abs(), "{a} vs {b}");
    }

    #[test]
    fn isa_sea_level_and_aloft() {
        // ISA 1976 tables.
        let a = Atmosphere::standard();
        let s = a.at_msl(0.0);
        close(s.pressure, 101_325.0, 1e-6);
        close(s.density, 1.2250, 1e-3);
        close(s.speed_of_sound, 340.294, 1e-4);
        close(s.viscosity, 1.7894e-5, 2e-3);
        // Geometric altitudes of the 11 km and 20 km geopotential levels.
        let s = a.at_msl(11_019.0);
        close(s.temperature, 216.65, 1e-3);
        close(s.pressure, 22_632.0, 2e-3);
        let s = a.at_msl(20_000.0);
        close(s.pressure, 5529.3, 3e-3);
        close(s.density, 0.08891, 5e-3);
    }

    #[test]
    fn site_offset() {
        let a = Atmosphere::new(1500.0, 303.15);
        let s = a.at_agl(0.0);
        close(s.temperature, 303.15, 1e-9);
        // Hotter than standard means thinner air than the ISA profile.
        assert!(s.density < Atmosphere::standard().at_msl(1500.0).density);
    }
}
