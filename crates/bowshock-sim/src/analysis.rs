//! Parametric analyses: sweeps vs Mach and angle of attack, Monte Carlo dispersion. Fin flutter is [`crate::flutter`].

use crate::aero::{Aero, DragBreakdown};
use crate::atmosphere::Atmosphere;
use crate::flight::{build_aeros, run, Launch, SimInput, Tweaks, WindPoint};
use crate::rng::Rng;
use crate::vehicle::Vehicle;

/// One point of a Mach sweep.
#[derive(Clone, Debug, PartialEq)]
pub struct MachPoint {
    /// Mach number.
    pub mach: f64,
    /// Static margin (cal) for the supplied CG.
    pub margin_cal: f64,
    /// CP station from the tip (m).
    pub cp: f64,
    /// Normal force slope per radian.
    pub cn_alpha: f64,
    /// Drag build-up at this Mach (motor off).
    pub drag: DragBreakdown,
}

fn lerp_range(range: (f64, f64), i: usize, n: usize) -> f64 {
    if n < 2 {
        range.0
    } else {
        range.0 + (range.1 - range.0) * i as f64 / (n - 1) as f64
    }
}

/// Aerodynamics against Mach: `n` evenly spaced points over `mach_range` at angle of attack
/// `alpha` (rad). Reynolds number follows from the Mach number's velocity and the air at
/// `altitude` metres above the site of `atmosphere`. Drag is for motor-off (full base area).
pub fn vs_mach(vehicle: &Vehicle, cg: f64, mach_range: (f64, f64), n: usize, alpha: f64, atmosphere: &Atmosphere, altitude: f64) -> Vec<MachPoint> {
    let aero = Aero::new(vehicle);
    let air = atmosphere.at_agl(altitude);
    (0..n)
        .map(|i| {
            let mach = lerp_range(mach_range, i, n).max(0.0);
            let re = mach * air.speed_of_sound / air.kinematic_viscosity();
            let st = aero.stability(mach, alpha);
            MachPoint {
                mach,
                margin_cal: (st.cp - cg) / aero.reference_diameter(),
                cp: st.cp,
                cn_alpha: st.cn_alpha,
                drag: aero.drag_at_alpha(mach, re, false, alpha),
            }
        })
        .collect()
}

/// One point of an angle-of-attack sweep.
#[derive(Clone, Debug, PartialEq)]
pub struct AlphaPoint {
    /// Angle of attack (rad).
    pub alpha: f64,
    /// Static margin (cal).
    pub margin_cal: f64,
    /// CP station (m).
    pub cp: f64,
    /// `CN/alpha` per radian at this angle (includes nonlinear body lift).
    pub cn_alpha: f64,
    /// Normal force coefficient `CN = cn_alpha * alpha`.
    pub cn: f64,
    /// Drag including the induced term.
    pub drag: DragBreakdown,
}

/// Aerodynamics against angle of attack at fixed `mach`: `n` points over `alpha_range` (rad),
/// Reynolds number from the standard atmosphere at sea level.
pub fn vs_alpha(vehicle: &Vehicle, cg: f64, mach: f64, alpha_range: (f64, f64), n: usize) -> Vec<AlphaPoint> {
    let aero = Aero::new(vehicle);
    let air = Atmosphere::standard().at_agl(0.0);
    let re = mach * air.speed_of_sound / air.kinematic_viscosity();
    (0..n)
        .map(|i| {
            let alpha = lerp_range(alpha_range, i, n);
            let st = aero.stability(mach, alpha);
            AlphaPoint {
                alpha,
                margin_cal: (st.cp - cg) / aero.reference_diameter(),
                cp: st.cp,
                cn_alpha: st.cn_alpha,
                cn: st.cn_alpha * alpha,
                drag: aero.drag_at_alpha(mach, re, false, alpha),
            }
        })
        .collect()
}

/// One-sigma dispersions applied by [`monte_carlo`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Dispersion {
    /// Wind speed standard deviation (m/s).
    pub wind_speed: f64,
    /// Wind direction standard deviation (rad).
    pub wind_dir: f64,
    /// Launch rail angle standard deviation (rad), applied in both horizontal tilt components.
    pub launch_angle: f64,
    /// Fractional thrust (impulse) standard deviation.
    pub thrust_scale: f64,
    /// Fractional drag coefficient standard deviation.
    pub cd_scale: f64,
    /// Fractional dry-mass standard deviation.
    pub mass: f64,
}

/// Outcome of one dispersed flight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MonteCarloRun {
    /// Landing point `[east, north]` (m).
    pub landing: [f64; 2],
    /// Apogee altitude (m).
    pub apogee: f64,
    /// Maximum speed (m/s).
    pub max_velocity: f64,
    /// Flight time (s).
    pub flight_time: f64,
    /// Ground-hit speed (m/s).
    pub ground_hit_velocity: f64,
}

/// Aggregate statistics.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MonteCarloStats {
    /// Mean apogee (m).
    pub apogee_mean: f64,
    /// Apogee standard deviation (m).
    pub apogee_std: f64,
    /// Minimum apogee (m).
    pub apogee_min: f64,
    /// Maximum apogee (m).
    pub apogee_max: f64,
    /// Mean landing point `[east, north]` (m).
    pub landing_mean: [f64; 2],
    /// Landing standard deviations `[east, north]` (m).
    pub landing_std: [f64; 2],
    /// 1-sigma landing ellipse semi-major axis (m).
    pub ellipse_major: f64,
    /// 1-sigma landing ellipse semi-minor axis (m).
    pub ellipse_minor: f64,
    /// Angle of the major axis, counter-clockwise from east (rad).
    pub ellipse_angle: f64,
    /// Median distance of the landings from their mean (m), the circular error probable.
    pub cep50: f64,
}

/// Monte Carlo result.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MonteCarlo {
    /// Every run, in order.
    pub runs: Vec<MonteCarloRun>,
    /// Statistics over the runs.
    pub stats: MonteCarloStats,
}

/// Flies `input` `runs` times with Gaussian dispersions `sigma` around the nominal conditions.
/// The random stream derives from `input.launch.seed`, so results are reproducible; the runs
/// execute on all available cores.
pub fn monte_carlo(input: &SimInput, runs: usize, sigma: &Dispersion) -> MonteCarlo {
    let aeros = build_aeros(input);
    let base = &input.launch;
    let mut rng = Rng::new(base.seed ^ 0x5EED_0000_0000_0001);
    let nominal_tilt = [base.angle_from_vertical * base.direction.sin(), base.angle_from_vertical * base.direction.cos()];
    let cases: Vec<(Launch, Tweaks)> = (0..runs)
        .map(|i| {
            let tx = nominal_tilt[0] + sigma.launch_angle * rng.normal();
            let ty = nominal_tilt[1] + sigma.launch_angle * rng.normal();
            let wind_speed = (base.wind_speed + sigma.wind_speed * rng.normal()).max(0.0);
            let wind_from = base.wind_from + sigma.wind_dir * rng.normal();
            // A wind profile moves with the ground wind: every level scaled by the same factor and turned by the same angle.
            let (scale, turn) = (if base.wind_speed < 1e-9 { None } else { Some(wind_speed / base.wind_speed) }, wind_from - base.wind_from);
            let wind_levels =
                base.wind_levels.iter().map(|l| WindPoint { speed: scale.map_or(wind_speed, |k| l.speed * k), from: l.from + turn, ..*l }).collect();
            let launch = Launch {
                wind_speed,
                wind_from,
                wind_levels,
                angle_from_vertical: tx.hypot(ty).min(1.2),
                direction: if tx == 0.0 && ty == 0.0 { base.direction } else { tx.atan2(ty) },
                seed: base.seed.wrapping_add(i as u64 + 1),
                ..base.clone()
            };
            let tw = Tweaks {
                thrust: (1.0 + sigma.thrust_scale * rng.normal()).max(0.1),
                cd: (1.0 + sigma.cd_scale * rng.normal()).max(0.1),
                mass: (1.0 + sigma.mass * rng.normal()).max(0.1),
            };
            (launch, tw)
        })
        .collect();

    let threads = std::thread::available_parallelism().map_or(1, usize::from).min(cases.len().max(1));
    let chunk = cases.len().div_ceil(threads).max(1);
    let results: Vec<MonteCarloRun> = std::thread::scope(|s| {
        let handles: Vec<_> = cases
            .chunks(chunk)
            .map(|part| {
                let aeros = &aeros;
                s.spawn(move || {
                    part.iter()
                        .map(|(launch, tw)| {
                            let r = run(input, launch, *tw, aeros);
                            MonteCarloRun {
                                landing: r.summary.landing_position,
                                apogee: r.summary.apogee,
                                max_velocity: r.summary.max_velocity,
                                flight_time: r.summary.flight_time,
                                ground_hit_velocity: r.summary.ground_hit_velocity,
                            }
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().expect("monte carlo worker panicked")).collect()
    });
    let stats = statistics(&results);
    MonteCarlo { runs: results, stats }
}

fn statistics(runs: &[MonteCarloRun]) -> MonteCarloStats {
    let n = runs.len() as f64;
    if runs.is_empty() {
        return MonteCarloStats::default();
    }
    let mean = |f: &dyn Fn(&MonteCarloRun) -> f64| runs.iter().map(f).sum::<f64>() / n;
    let apogee_mean = mean(&|r| r.apogee);
    let apogee_std = (mean(&|r| (r.apogee - apogee_mean).powi(2))).sqrt();
    let mx = mean(&|r| r.landing[0]);
    let my = mean(&|r| r.landing[1]);
    let sxx = mean(&|r| (r.landing[0] - mx).powi(2));
    let syy = mean(&|r| (r.landing[1] - my).powi(2));
    let sxy = mean(&|r| (r.landing[0] - mx) * (r.landing[1] - my));
    let half = 0.5 * (sxx + syy);
    let rad = (0.25 * (sxx - syy).powi(2) + sxy * sxy).sqrt();
    let mut dists: Vec<f64> = runs.iter().map(|r| (r.landing[0] - mx).hypot(r.landing[1] - my)).collect();
    dists.sort_by(f64::total_cmp);
    MonteCarloStats {
        apogee_mean,
        apogee_std,
        apogee_min: runs.iter().map(|r| r.apogee).fold(f64::INFINITY, f64::min),
        apogee_max: runs.iter().map(|r| r.apogee).fold(f64::NEG_INFINITY, f64::max),
        landing_mean: [mx, my],
        landing_std: [sxx.sqrt(), syy.sqrt()],
        ellipse_major: (half + rad).max(0.0).sqrt(),
        ellipse_minor: (half - rad).max(0.0).sqrt(),
        ellipse_angle: 0.5 * (2.0 * sxy).atan2(sxx - syy),
        cep50: dists[dists.len() / 2],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::*;

    fn simple() -> Vehicle {
        let r = 0.03;
        Vehicle {
            reference_diameter: 2.0 * r,
            body: vec![
                BodySegment { station: 0.0, length: 0.2, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::VonKarman), roughness: 0.0 },
                BodySegment { station: 0.2, length: 0.6, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 0.0 },
            ],
            tube_fins: vec![],
            fin_sets: vec![FinSet {
                count: 4,
                station: 0.65,
                body_radius: r,
                root_chord: 0.15,
                tip_chord: 0.06,
                span: 0.09,
                sweep: 0.07,
                thickness: 0.003,
                cross_section: FinCrossSection::Square,
                cant: 0.0,
                roughness: 0.0,
                planform: FinPlanform::Trapezoidal,
                angles: Vec::new(),
                parent: 0,
                bodies: 1,
                mid_chord_cos: None,
                root_extent: None,
            }],
            protuberances: vec![],
            stages: vec![],
            labels: Default::default(),
        }
    }

    #[test]
    fn sweeps_have_requested_shape() {
        let v = simple();
        let m = vs_mach(&v, 0.5, (0.1, 3.0), 30, 0.0, &Atmosphere::standard(), 0.0);
        assert_eq!(m.len(), 30);
        assert!(m.iter().all(|p| p.margin_cal > 0.0 && p.drag.total > 0.0));
        let a = vs_alpha(&v, 0.5, 0.3, (0.0, 0.3), 10);
        assert_eq!(a.len(), 10);
        // Body lift grows with alpha, pulling the CP forward of the alpha=0 value.
        assert!(a[9].cp < a[0].cp);
        assert!(a[9].drag.total > a[0].drag.total);
    }

    #[test]
    fn ellipse_of_known_covariance() {
        let runs: Vec<MonteCarloRun> = [[1.0, 0.0], [-1.0, 0.0], [0.0, 3.0], [0.0, -3.0]]
            .iter()
            .map(|&landing| MonteCarloRun { landing, apogee: 100.0, max_velocity: 0.0, flight_time: 0.0, ground_hit_velocity: 0.0 })
            .collect();
        let s = statistics(&runs);
        assert!((s.ellipse_major - 4.5f64.sqrt()).abs() < 1e-9);
        assert!((s.ellipse_minor - 0.5f64.sqrt()).abs() < 1e-9);
    }
}
