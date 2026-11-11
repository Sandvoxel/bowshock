//! International Standard Atmosphere (0-20 km) and explicit flight conditions: static state of the air for viscous analyses.

use crate::error::Error;

/// Specific gas constant of dry air [J/(kg K)].
const GAS_CONSTANT: f64 = 287.05287;
/// Ratio of specific heats of air [-].
const GAMMA: f64 = 1.4;
/// Sea-level standard temperature [K].
const T0: f64 = 288.15;
/// Sea-level standard pressure [Pa].
const P0: f64 = 101_325.0;
/// Temperature lapse rate of the troposphere [K/m].
const LAPSE: f64 = 0.0065;
/// Top of the troposphere, the lower bound of the isothermal layer [m].
const TROPOPAUSE_M: f64 = 11_000.0;
/// Top of the supported altitude range [m].
const CEILING_M: f64 = 20_000.0;
/// Isothermal temperature of the lower stratosphere [K].
const T_STRATOSPHERE: f64 = 216.65;
/// Pressure at the tropopause, the reference of the stratosphere exponential [Pa].
const P_TROPOPAUSE: f64 = 22_632.06;
/// Scale height of the lower stratosphere [m].
const SCALE_HEIGHT_M: f64 = 6_341.62;
/// Exponent of the troposphere pressure law [-].
const PRESSURE_EXPONENT: f64 = 5.255_877;
/// Sutherland reference viscosity [Pa s].
const MU_REF: f64 = 1.716e-5;
/// Sutherland reference temperature [K].
const T_REF: f64 = 273.15;
/// Sutherland constant of air [K].
pub(crate) const SUTHERLAND_S: f64 = 110.4;
/// Lowest explicit temperature accepted [K].
const T_MIN: f64 = 150.0;
/// Highest explicit temperature accepted [K].
const T_MAX: f64 = 400.0;

/// Source of the static state of the air.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Atmosphere {
    /// International Standard Atmosphere at a geometric altitude in metres (0 to 20 000 m).
    Isa {
        /// Geometric altitude [m].
        altitude_m: f64,
    },
    /// Static temperature and pressure given directly.
    Explicit {
        /// Static temperature [K], within 150 to 400 K.
        temperature_k: f64,
        /// Static pressure [Pa], finite and positive.
        pressure_pa: f64,
    },
}

/// Static state of the air.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirState {
    /// Static temperature [K].
    pub temperature_k: f64,
    /// Static pressure [Pa].
    pub pressure_pa: f64,
    /// Density [kg/m^3].
    pub density: f64,
    /// Speed of sound [m/s].
    pub sound_speed: f64,
    /// Dynamic viscosity from Sutherland's law [Pa s].
    pub viscosity: f64,
}

/// Flight conditions of a viscous analysis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightConditions {
    /// Static state of the air the vehicle flies through.
    pub atmosphere: Atmosphere,
}

impl Atmosphere {
    /// Static state of the air for this atmosphere.
    ///
    /// Returns `Error::InvalidInput` when the altitude is outside 0 to 20 000 m or the explicit temperature or pressure is
    /// out of range.
    pub fn state(&self) -> Result<AirState, Error> {
        let (temperature_k, pressure_pa) = match *self {
            Atmosphere::Isa { altitude_m: h } => {
                if (0.0..=TROPOPAUSE_M).contains(&h) {
                    let t = T0 - LAPSE * h;
                    (t, P0 * (t / T0).powf(PRESSURE_EXPONENT))
                } else if h > TROPOPAUSE_M && h <= CEILING_M {
                    (T_STRATOSPHERE, P_TROPOPAUSE * (-(h - TROPOPAUSE_M) / SCALE_HEIGHT_M).exp())
                } else {
                    return Err(Error::InvalidInput(format!("altitude_m must be within 0 to {CEILING_M} m, got {h}")));
                }
            }
            Atmosphere::Explicit { temperature_k, pressure_pa } => {
                if !(T_MIN..=T_MAX).contains(&temperature_k) {
                    return Err(Error::InvalidInput(format!("temperature_k must be within {T_MIN} to {T_MAX} K, got {temperature_k}")));
                }
                if !(pressure_pa.is_finite() && pressure_pa > 0.0) {
                    return Err(Error::InvalidInput(format!("pressure_pa must be finite and positive, got {pressure_pa}")));
                }
                (temperature_k, pressure_pa)
            }
        };
        let t = temperature_k;
        Ok(AirState {
            temperature_k: t,
            pressure_pa,
            density: pressure_pa / (GAS_CONSTANT * t),
            sound_speed: (GAMMA * GAS_CONSTANT * t).sqrt(),
            viscosity: MU_REF * (t / T_REF).powf(1.5) * (T_REF + SUTHERLAND_S) / (t + SUTHERLAND_S),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_matches_isa() {
        let s = Atmosphere::Isa { altitude_m: 0.0 }.state().expect("sea level is valid");
        assert_eq!(s.temperature_k, 288.15);
        assert!((s.pressure_pa - 101_325.0).abs() < 1e-6);
        assert!((s.density - 1.2250).abs() < 1e-4, "density {}", s.density);
        assert!((s.sound_speed - 340.29).abs() < 0.01, "sound speed {}", s.sound_speed);
        let rel = (s.viscosity - 1.789e-5).abs() / 1.789e-5;
        assert!(rel < 0.005, "viscosity {}", s.viscosity);
    }

    #[test]
    fn eleven_km_pressure() {
        let s = Atmosphere::Isa { altitude_m: 11_000.0 }.state().expect("11 km is valid");
        assert!((s.temperature_k - 216.65).abs() < 1e-9);
        assert!((s.pressure_pa - 22_632.0).abs() < 5.0, "pressure {}", s.pressure_pa);
    }

    #[test]
    fn isa_out_of_range_is_invalid() {
        assert!(matches!(Atmosphere::Isa { altitude_m: -1.0 }.state(), Err(Error::InvalidInput(_))));
        assert!(matches!(Atmosphere::Isa { altitude_m: 25_000.0 }.state(), Err(Error::InvalidInput(_))));
        assert!(matches!(Atmosphere::Isa { altitude_m: f64::NAN }.state(), Err(Error::InvalidInput(_))));
    }

    #[test]
    fn explicit_state_and_validation() {
        assert!(matches!(Atmosphere::Explicit { temperature_k: 100.0, pressure_pa: 1e5 }.state(), Err(Error::InvalidInput(_))));
        assert!(matches!(Atmosphere::Explicit { temperature_k: 300.0, pressure_pa: 0.0 }.state(), Err(Error::InvalidInput(_))));
        let s = Atmosphere::Explicit { temperature_k: 300.0, pressure_pa: 101_325.0 }.state().expect("300 K is valid");
        assert!((s.density - 1.1766).abs() < 1e-3, "density {}", s.density);
    }
}
