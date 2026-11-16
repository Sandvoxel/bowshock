//! Roll–pitch coupling: the pitch natural frequency of the rocket and the check of the spin rate against it.
//!
//! Canted fins spin the rocket up ([`crate::Aero::roll`]: Barrowman 1967, Niskanen 2009 section 3.2.3). A rocket that spins at
//! the rate of its own pitch oscillation couples the two motions: any small asymmetry (a bent fin, a CG off the axis, a thrust
//! misalignment) then feeds the pitch oscillation every turn, and the angle of attack can grow until the spin leaves the
//! resonance ("roll lock-in", "roll resonance"; Nielsen, *Missile Aerodynamics*, McGraw-Hill, 1960, the chapter on the coupling
//! of rolling with pitching and yawing). Bowshock's aerodynamics are axisymmetric, so the flight model itself carries no such
//! asymmetry and shows no growth: [`roll_resonance`] only reports *that and when* the spin rate meets the pitch frequency, so the
//! user can judge the risk of a real, imperfect rocket.

use std::f64::consts::TAU;

/// How near the spin rate must come to the pitch frequency, as a fraction of it, to count as approaching resonance. A heuristic
/// of Bowshock (the width of the resonance depends on the damping and on the asymmetry, which the model does not have), wide
/// enough that a spin that settles beside the pitch frequency is flagged and a spin far above or below it is not.
pub const RESONANCE_BAND: f64 = 0.15;

/// Spin below this rate (rad/s, a tenth of a turn per second) is not a spin: only a crossing is reported then, never an approach.
pub const MIN_SPIN_RATE: f64 = 0.1 * TAU;

/// A pitch oscillation slower than this (rad/s, a period of two seconds) is not resonance material: near apogee the dynamic
/// pressure, and with it the pitch frequency, falls towards zero while the spin hangs on, so the spin always "crosses" it there
/// with no air load to couple to. Samples below it break the run like samples without a pitch frequency. A heuristic of Bowshock.
pub const MIN_PITCH_FREQUENCY: f64 = std::f64::consts::PI;

/// The natural frequency (rad/s) of the rocket's pitch oscillation, `sqrt(q S C_Na (x_cp - x_cg) / I)`: the restoring moment of
/// the normal force `q S C_Na alpha` at the lever arm `x_cp - x_cg` over the pitch inertia `I` (the undamped short-period
/// frequency of a slender body; Zipfel 2014, *Modeling and Simulation of Aerospace Vehicle Dynamics*).
///
/// `q` is the dynamic pressure (Pa), `a_ref` the reference area (m^2), `cn_alpha` the normal-force slope (per rad),
/// `cp_minus_cg` the distance of the centre of pressure aft of the centre of gravity (m) and `i_long` the pitch moment of
/// inertia (kg m^2). Zero for a rocket that has no restoring moment (the CP at or ahead of the CG: it diverges rather than
/// oscillates) or no airflow.
pub fn pitch_frequency(q: f64, a_ref: f64, cn_alpha: f64, cp_minus_cg: f64, i_long: f64) -> f64 {
    if cp_minus_cg <= 0.0 || i_long <= 0.0 || q <= 0.0 || cn_alpha <= 0.0 {
        return 0.0;
    }
    (q * a_ref * cn_alpha * cp_minus_cg / i_long).sqrt()
}

/// The moment the spin rate meets the pitch frequency (see [`roll_resonance`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RollResonance {
    /// Time of the crossing, or of the closest approach (s).
    pub time: f64,
    /// Magnitude of the roll rate then (rad/s).
    pub roll_rate: f64,
    /// Pitch natural frequency then (rad/s).
    pub pitch_frequency: f64,
    /// Whether the spin rate crossed the pitch frequency (`true`) or only came within [`RESONANCE_BAND`] of it.
    pub crossed: bool,
}

/// Where the spin rate meets the pitch frequency over a flight: `(time s, roll rate rad/s, pitch frequency rad/s)` per sample, in
/// time order. A sample with no pitch frequency (point-mass flight, on the rail, under a parachute, an unstable rocket) or one
/// slower than [`MIN_PITCH_FREQUENCY`] breaks the run. The first crossing of `|roll rate| = pitch frequency` is returned; without
/// one, the closest approach within [`RESONANCE_BAND`] (while spinning faster than [`MIN_SPIN_RATE`]); `None` when the spin keeps
/// clear of the pitch frequency.
pub fn roll_resonance(samples: impl IntoIterator<Item = (f64, f64, f64)>) -> Option<RollResonance> {
    let mut previous: Option<f64> = None;
    let mut closest: Option<(f64, RollResonance)> = None;
    for (time, roll, wn) in samples {
        if !wn.is_finite() || wn < MIN_PITCH_FREQUENCY {
            previous = None;
            continue;
        }
        let (spin, offset) = (roll.abs(), roll.abs() / wn - 1.0);
        let here = RollResonance { time, roll_rate: spin, pitch_frequency: wn, crossed: false };
        if previous.is_some_and(|p| p * offset <= 0.0 && p != offset) {
            return Some(RollResonance { crossed: true, ..here });
        }
        previous = Some(offset);
        if spin >= MIN_SPIN_RATE && offset.abs() < RESONANCE_BAND && closest.is_none_or(|(best, _)| offset.abs() < best) {
            closest = Some((offset.abs(), here));
        }
    }
    closest.map(|(_, c)| c)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_pitch_frequency_is_the_square_root_of_the_restoring_moment_over_the_inertia() {
        // q = 2 kPa, S = 0.005 m^2, C_Na = 12, CP 0.15 m behind the CG, I = 0.2 kg m^2: sqrt(2000 * 0.005 * 12 * 0.15 / 0.2).
        let w = pitch_frequency(2000.0, 0.005, 12.0, 0.15, 0.2);
        assert!((w - (2000.0_f64 * 0.005 * 12.0 * 0.15 / 0.2).sqrt()).abs() < 1e-12, "{w}");
        // Four times the dynamic pressure doubles it.
        assert!((pitch_frequency(8000.0, 0.005, 12.0, 0.15, 0.2) / w - 2.0).abs() < 1e-12);
        // No restoring moment, no oscillation.
        assert_eq!(pitch_frequency(2000.0, 0.005, 12.0, 0.0, 0.2), 0.0);
        assert_eq!(pitch_frequency(2000.0, 0.005, 12.0, -0.05, 0.2), 0.0);
        assert_eq!(pitch_frequency(0.0, 0.005, 12.0, 0.15, 0.2), 0.0);
    }

    /// A spin that rises linearly from 0 to `top` rad/s over 4 s beside a pitch frequency of `wn` rad/s, sampled every 10 ms.
    fn ramp(top: f64, wn: f64) -> Vec<(f64, f64, f64)> {
        (0..=400).map(|i| (f64::from(i) * 0.01, top * f64::from(i) / 400.0, wn)).collect()
    }

    #[test]
    fn a_spin_up_through_the_pitch_frequency_is_a_crossing_at_that_moment() {
        // The ramp reaches the 20 rad/s pitch frequency half way through, at 2 s.
        let r = roll_resonance(ramp(40.0, 20.0)).expect("crosses");
        assert!(r.crossed && (r.time - 2.0).abs() < 0.011, "{r:?}");
        assert!((r.pitch_frequency - 20.0).abs() < 1e-12 && (r.roll_rate - 20.0).abs() < 0.2);
        // Spinning the other way crosses all the same: it is the magnitude that counts.
        let back: Vec<_> = ramp(40.0, 20.0).into_iter().map(|(t, p, w)| (t, -p, w)).collect();
        assert_eq!(roll_resonance(back).map(|r| r.crossed), Some(true));
    }

    #[test]
    fn a_spin_that_stays_clear_is_no_resonance_and_one_that_settles_beside_it_is_an_approach() {
        // 10 rad/s against 20 rad/s: half the pitch frequency, outside the band.
        assert_eq!(roll_resonance(ramp(10.0, 20.0)), None);
        // 18 rad/s against 20 rad/s: within 15 %, but never across it.
        let r = roll_resonance(ramp(18.0, 20.0)).expect("approaches");
        assert!(!r.crossed && (r.roll_rate - 18.0).abs() < 1e-9 && (r.time - 4.0).abs() < 1e-9, "{r:?}");
        // No spin at all is no resonance, however close its (zero) rate is to a tiny pitch frequency.
        assert_eq!(roll_resonance((0..100).map(|i| (f64::from(i) * 0.01, 0.0, 0.2))), None);
    }

    #[test]
    fn samples_without_a_pitch_frequency_break_the_run() {
        // Above the frequency, then a gap (a parachute), then below: no crossing, the gap hides it.
        let up = (0..10).map(|i| (f64::from(i) * 0.1, 30.0, 20.0));
        let gap = (10..20).map(|i| (f64::from(i) * 0.1, 30.0, 0.0));
        let down = (20..30).map(|i| (f64::from(i) * 0.1, 5.0, 20.0));
        assert_eq!(roll_resonance(up.chain(gap).chain(down)), None);
        assert_eq!(roll_resonance(std::iter::empty()), None);
        // Near apogee the pitch frequency falls to nothing while a slow spin hangs on: it "crosses" the spin below the two-second
        // period floor, which is no resonance.
        let coast = (0..100).map(|i| (f64::from(i) * 0.1, 2.0, 20.0 * (1.0 - f64::from(i) / 100.0)));
        assert_eq!(roll_resonance(coast), None);
    }
}
