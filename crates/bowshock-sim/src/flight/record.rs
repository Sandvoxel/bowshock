use super::{Raw, Sample};
use crate::aero::Aero;
use crate::attitude::dot;

/// The recorded samples of a flight: `raws` decimated to about 2000 (always the first, the last and the instants `event_idx`
/// indexes), each turned into a [`Sample`]. The aerodynamic model of a raw state is `aeros[nsep]`, `a_ref` is the reference
/// area the recovery drag is quoted against and `site_altitude` turns altitudes above ground into altitudes above sea level.
pub(super) fn samples(raws: &[Raw], event_idx: &[usize], aeros: &[Aero], a_ref: f64, site_altitude: f64) -> Vec<Sample> {
    let len = raws.len();
    let stride = len.div_ceil(2000).max(1);
    let mut keep = vec![false; len];
    for (i, k) in keep.iter_mut().enumerate() {
        *k = i % stride == 0 || i + 1 == len;
    }
    for &i in event_idx {
        if i < len {
            keep[i] = true;
        }
    }
    raws.iter()
        .zip(&keep)
        .filter(|(_, &k)| k)
        .map(|(r, _)| {
            let d = &r.d;
            let speed = dot(r.vel, r.vel).sqrt();
            let horizontal = r.vel[0].hypot(r.vel[1]);
            let aero = &aeros[r.nsep.min(aeros.len() - 1)];
            let cp = aero.stability(d.mach, 0.0).cp;
            let recovery = d.open / a_ref;
            Sample {
                t: r.t,
                position: r.pos,
                velocity: r.vel,
                altitude: r.pos[2],
                altitude_msl: site_altitude + r.pos[2],
                vertical_velocity: r.vel[2],
                horizontal_velocity: horizontal,
                speed,
                air_speed: d.air_speed,
                ground_distance: r.pos[0].hypot(r.pos[1]),
                flight_path_angle: r.vel[2].atan2(horizontal),
                mach: d.mach,
                acceleration: dot(d.acc, d.acc).sqrt(),
                vertical_acceleration: d.acc[2],
                thrust: d.thrust,
                drag: d.drag,
                cd: d.cd.total,
                cd_friction: d.cd.friction,
                cd_pressure: d.cd.pressure,
                cd_base: d.cd.base,
                cd_fins: d.cd.fins,
                cd_protuberances: d.cd.protuberances,
                cd_recovery: recovery,
                recovery_cd_area: d.open,
                mass: d.mass,
                propellant_mass: d.propellant,
                cg: d.cg,
                cp,
                stability_margin: (cp - d.cg) / aero.reference_diameter(),
                angle_of_attack: d.alpha,
                wind_speed: d.wind,
                dynamic_pressure: 0.5 * d.air.density * d.air_speed * d.air_speed,
                air_density: d.air.density,
                air_temperature: d.air.temperature,
                air_pressure: d.air.pressure,
                speed_of_sound: d.air.speed_of_sound,
                reynolds: d.reynolds,
                pitch_rate: d.omega[1].hypot(d.omega[2]),
                roll_rate: d.omega[0],
                pitch_frequency: d.pitch_freq,
                attitude_angle: d.axis[2].atan2(d.axis[0].hypot(d.axis[1])),
            }
        })
        .collect()
}
