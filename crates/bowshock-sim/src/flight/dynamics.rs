use super::input::wind_in;
use super::{Chute, Ctx, Diag, Flow, Mode, Props, State};
use crate::aero::{axial_drag_multiplier, DragBreakdown, BODY_CROSSFLOW_CD};
use crate::attitude::{dot, rotate, rotate_inv};
use crate::roll::pitch_frequency;
use std::f64::consts::{FRAC_PI_2, PI};

impl Ctx<'_> {
    /// The mean wind `[east, north]` at `h` above the site.
    pub(super) fn wind_at(&self, h: f64) -> [f64; 2] {
        wind_in(&self.wind, h)
    }

    /// The Coriolis acceleration `−2 Ω × v` of velocity `v` in the local east-north-up frame, zero without a latitude.
    pub(super) fn coriolis(&self, v: [f64; 3]) -> [f64; 3] {
        let Some(w) = self.omega else { return [0.0; 3] };
        let c = [w[1] * v[2] - w[2] * v[1], w[2] * v[0] - w[0] * v[2], w[0] * v[1] - w[1] * v[0]];
        [-2.0 * c[0], -2.0 * c[1], -2.0 * c[2]]
    }
}

impl Ctx<'_> {
    /// Mass, CG station, inertias, thrust and jet damping of the stages still flying at time `t` with `nsep` stages dropped.
    ///
    /// Dry parts carry the inertias of the model (about their own CG). A motor is a thin-walled casing plus the propellant still
    /// in it as a solid cylinder, both as long as the motor and as wide as its diameter, so the propellant's inertia falls
    /// as it burns. Everything is moved to the common CG with the parallel-axis theorem.
    pub(super) fn mass_props(&self, t: f64, nsep: usize) -> Props {
        let (mut m, mut mx, mut mx2, mut i_long, mut i_rot) = (0.0, 0.0, 0.0, 0.0, 0.0);
        let (mut thrust, mut burning) = (0.0, false);
        // Propellant flow `mdot` and its moments about the nozzle exit stations: sums of mdot, mdot x, mdot x^2.
        let (mut d0, mut d1, mut d2) = (0.0, 0.0, 0.0);
        for (k, st) in self.input.vehicle.stages.iter().enumerate().skip(nsep) {
            let dm = st.dry.mass * self.tw.mass;
            m += dm;
            mx += dm * st.dry.cg;
            mx2 += dm * st.dry.cg * st.dry.cg;
            i_long += st.dry.i_long * self.tw.mass;
            i_rot += st.dry.i_rot * self.tw.mass;
            if let Some(mo) = &st.motor {
                // A motor that has not lit (yet, or ever) is a full dead weight.
                let tau = self.sched.ignition[k].map_or(-1.0, |ign| t - ign);
                let c = &mo.curve;
                let mm = c.mass(tau);
                let xm = mo.cg_station();
                m += mm;
                mx += mm * xm;
                mx2 += mm * xm * xm;
                let (r, l) = (0.5 * c.diameter, c.length);
                let casing = (c.total_mass - c.propellant_mass).max(0.0);
                let propellant = (mm - casing).max(0.0);
                i_long += casing * (0.5 * r * r + l * l / 12.0) + propellant * (0.25 * r * r + l * l / 12.0);
                i_rot += casing * r * r + 0.5 * propellant * r * r;
                let raw = c.thrust(tau);
                if raw * self.tw.thrust > 0.0 {
                    burning = true;
                    thrust += raw * self.tw.thrust;
                }
                let impulse = c.total_impulse();
                if raw > 0.0 && impulse > 0.0 {
                    // Propellant burns in proportion to the impulse delivered (see `ThrustCurve::mass`).
                    let mdot = c.propellant_mass * raw / impulse;
                    let xe = mo.mount_aft_station;
                    d0 += mdot;
                    d1 += mdot * xe;
                    d2 += mdot * xe * xe;
                }
            }
        }
        if m < 1e-3 {
            return Props { mass: 1e-3, cg: 0.0, thrust, burning, i_long: 1e-9, i_rot: 1e-12, jet: 0.0 };
        }
        let cg = mx / m;
        // Parts that carry no inertia data fall back to a uniform rod over the flying stack and a solid cylinder of the reference
        // diameter, so the rotational dynamics are never singular.
        let i_long = {
            let about_cg = i_long + mx2 - m * cg * cg;
            if about_cg > 1e-9 {
                about_cg
            } else {
                let l = self.stack_length(nsep).max(1e-3);
                m * l * l / 12.0
            }
        };
        let i_rot = if i_rot > 1e-12 { i_rot } else { 0.5 * m * (0.5 * self.input.vehicle.reference_diameter).powi(2) }.max(1e-12);
        Props { mass: m, cg, thrust, burning, i_long, i_rot, jet: (d2 - 2.0 * cg * d1 + cg * cg * d0).max(0.0) }
    }

    /// Propellant left in the motors of the stages still flying at time `t`.
    pub(super) fn propellant(&self, t: f64, nsep: usize) -> f64 {
        let mut left = 0.0;
        for (k, st) in self.input.vehicle.stages.iter().enumerate().skip(nsep) {
            if let Some(mo) = &st.motor {
                let tau = self.sched.ignition[k].map_or(-1.0, |ign| t - ign);
                left += (mo.curve.mass(tau) - (mo.curve.total_mass - mo.curve.propellant_mass)).max(0.0);
            }
        }
        left
    }

    /// Length of the stack that is still flying (m): the vehicle, cut at the aft end of stage `nsep` once stages are dropped.
    pub(super) fn stack_length(&self, nsep: usize) -> f64 {
        let v = &self.input.vehicle;
        match v.stages.get(nsep) {
            Some(st) if nsep > 0 => v.length().min(st.aft_station),
            _ => v.length(),
        }
    }

    /// Equations of motion at `(t, st)`.
    pub(super) fn eval(&self, t: f64, st: &State, gust: [f64; 3], nsep: usize, mode: Mode, chutes: &[Chute]) -> Diag {
        let p = self.mass_props(t, nsep);
        let air = self.atm.at_agl(st.pos[2].max(0.0));
        let mean = self.wind_at(st.pos[2].max(0.0));
        let wind = [mean[0] + gust[0], mean[1] + gust[1]];
        let rel = [st.vel[0] - wind[0], st.vel[1] - wind[1], st.vel[2] - gust[2]];
        let s = dot(rel, rel).sqrt();
        let flow = Flow {
            p,
            air,
            wind: wind[0].hypot(wind[1]),
            rel,
            s,
            mach: s / air.speed_of_sound,
            re_per_m: s * air.density / air.viscosity,
            g: self.g_surface * (6_371_000.0 / (6_371_000.0 + st.pos[2].max(0.0) + self.launch.site_altitude)).powi(2),
        };
        let mut d = if mode == Mode::Rigid { self.rigid_eval(st, &flow, nsep) } else { self.point_eval(t, st, &flow, nsep, mode == Mode::Rail, chutes) };
        // The rotating Earth (WGS 84, NIMA TR8350.2): the Coriolis acceleration in the local east-north-up frame, off the rail.
        if mode != Mode::Rail {
            let c = self.coriolis(st.vel);
            for (a, ci) in d.acc.iter_mut().zip(c) {
                *a += ci;
            }
        }
        Diag { propellant: self.propellant(t, nsep), reynolds: flow.re_per_m * self.stack_length(nsep), control_cant: self.control_cant, ..d }
    }

    /// Point mass: the axis follows the relative wind (or the rail), drag opposes the relative velocity.
    pub(super) fn point_eval(&self, t: f64, st: &State, f: &Flow, nsep: usize, rail: bool, chutes: &[Chute]) -> Diag {
        let (p, s, rel) = (&f.p, f.s, f.rel);
        let dir = if rail || s < 1.0 { self.axis } else { [rel[0] / s, rel[1] / s, rel[2] / s] };
        let q = 0.5 * f.air.density * s * s;
        // Deployed devices: canopy area ramps up (see `Chute`); the airframe's own drag fades out
        // as the canopies fill.
        let full: f64 = chutes.iter().map(|c| c.cd_area).sum();
        let open: f64 = chutes.iter().map(|c| c.area_at(t)).sum();
        let frac = if full > 0.0 { (open / full).min(1.0) } else { 0.0 };
        let mut drag = q * open;
        let mut cd = DragBreakdown::default();
        if frac < 1.0 {
            let b = self.aeros[nsep.min(self.aeros.len() - 1)].drag(f.mach, f.re_per_m, p.burning);
            let w = (1.0 - frac) * self.tw.cd;
            cd = DragBreakdown {
                friction: w * b.friction,
                pressure: w * b.pressure,
                base: w * b.base,
                fins: w * b.fins,
                protuberances: w * b.protuberances,
                total: w * b.total,
            };
            drag += q * cd.total * self.a_ref;
        }
        let rd = if s > 1e-9 { [rel[0] / s, rel[1] / s, rel[2] / s] } else { [0.0; 3] };
        let alpha = if s > 1e-9 { dot(dir, rd).clamp(-1.0, 1.0).acos() } else { 0.0 };
        let mut acc = [0.0; 3];
        for i in 0..3 {
            acc[i] = (p.thrust * dir[i] - drag * rd[i]) / p.mass;
        }
        acc[2] -= f.g;
        if rail {
            let mut au = dot(acc, self.axis);
            if dot(st.vel, self.axis) <= 0.0 && au < 0.0 {
                au = 0.0;
            }
            acc = [au * self.axis[0], au * self.axis[1], au * self.axis[2]];
        }
        Diag {
            acc,
            ang_acc: [0.0; 3],
            thrust: p.thrust,
            drag,
            mass: p.mass,
            cg: p.cg,
            mach: f.mach,
            air: f.air,
            air_speed: s,
            alpha,
            cd,
            open,
            wind: f.wind,
            propellant: 0.0,
            reynolds: 0.0,
            axis: dir,
            omega: [0.0; 3],
            stiff: 0.0,
            pitch_freq: 0.0,
            control_cant: 0.0,
        }
    }

    /// Rigid body: forces and moments in the body frame (x along the nose, y and z transverse).
    ///
    /// With the lateral air velocity `v_l` and the axial one `u` the angle of attack is `alpha = atan2(|v_l|, u)`; flying tail
    /// first (`alpha > pi/2`) the coefficients are those of the mirror angle `pi - alpha`. The axial force is
    /// `-sign(u) q A_ref Cd k(alpha)` and the normal force `q A_ref CNalpha alpha` opposes `v_l`, acting at the centre of pressure
    /// `x_cp`, which is `x_cg - x_cp` ahead of the centre of gravity (aft when negative): the moment is that lever arm times the
    /// normal force, stabilising when the CP is aft of the CG.
    pub(super) fn rigid_eval(&self, st: &State, f: &Flow, nsep: usize) -> Diag {
        let aero = &self.aeros[nsep.min(self.aeros.len() - 1)];
        let (p, s) = (&f.p, f.s);
        let vb = rotate_inv(st.q, f.rel);
        let lateral = vb[1].hypot(vb[2]);
        let (alpha, alpha_eff) = if s > 1e-9 {
            let a = lateral.atan2(vb[0]);
            (a, if a > FRAC_PI_2 { PI - a } else { a })
        } else {
            (0.0, 0.0)
        };
        let rho = f.air.density;
        let q = 0.5 * rho * s * s;

        // Axial drag.
        let b = aero.drag(f.mach, f.re_per_m, p.burning);
        let w = self.tw.cd;
        let cd = DragBreakdown {
            friction: w * b.friction,
            pressure: w * b.pressure,
            base: w * b.base,
            fins: w * b.fins,
            protuberances: w * b.protuberances,
            total: w * b.total,
        };
        let axial = -vb[0].signum() * q * cd.total * axial_drag_multiplier(alpha_eff) * self.a_ref;

        // Normal force against the lateral velocity.
        let nf = aero.normal_force(f.mach, alpha_eff);
        let normal = q * self.a_ref * nf.cn_alpha * alpha_eff;
        let (fy, fz) = if lateral > 1e-9 { (-normal * vb[1] / lateral, -normal * vb[2] / lateral) } else { (0.0, 0.0) };

        // Moments about the CG: the normal force at the CP, then damping (fins and bodies at their CPs, crossflow drag on the
        // turning body, the exhaust leaving at the nozzle) and the roll of the canted fins.
        let arm = p.cg - nf.cp;
        let wl = st.w[1].hypot(st.w[2]);
        let linear = 0.5 * rho * s * self.a_ref * nf.damping_sum(p.cg) + p.jet;
        let crossflow = 0.5 * rho * BODY_CROSSFLOW_CD * aero.crossflow_damping(p.cg);
        let damp = linear + crossflow * wl;
        let roll = aero.roll(f.mach);
        // Actively controlled fins add the forcing of their cant away from their own (see `RollControl`).
        let controlled: f64 = self.controlled[nsep.min(self.controlled.len() - 1)]
            .iter()
            .map(|&(set, part)| aero.roll_forcing_per_rad(f.mach, set) * self.deflection.iter().find(|d| d.0 == part).map_or(0.0, |d| d.1))
            .sum();
        // Roll moment about the nose direction (right-handed, so clockwise seen from behind, the way a fin's angle increases).
        let roll_moment = if s > 1e-9 { q * (roll.forcing + controlled + roll.damping * st.w[0] / s) } else { 0.0 };
        let (mx, my, mz) = (roll_moment, -arm * fz - damp * st.w[1], arm * fy - damp * st.w[2]);

        // Euler's equations of an axisymmetric body (I_y = I_z = i_long, I_x = i_rot).
        let (i, ix) = (p.i_long, p.i_rot);
        let ang_acc = [mx / ix, (my + (i - ix) * st.w[2] * st.w[0]) / i, (mz + (ix - i) * st.w[0] * st.w[1]) / i];

        // Translation: body forces into the inertial frame.
        let force = rotate(st.q, [p.thrust + axial, fy, fz]);
        let aero_force = rotate(st.q, [axial, fy, fz]);
        let acc = [force[0] / p.mass, force[1] / p.mass, force[2] / p.mass - f.g];
        let rd = if s > 1e-9 { [f.rel[0] / s, f.rel[1] / s, f.rel[2] / s] } else { [0.0; 3] };

        // The fastest rotational rates (pitch frequency, damping, roll damping) bound the RK4 step.
        let wn = (q * self.a_ref * nf.cn_alpha.abs() * (nf.cp - p.cg).abs() / i).sqrt();
        let stiff = wn.max(damp / i).max(0.5 * rho * s.max(1.0) * roll.damping.abs() / ix);
        let pitch_freq = pitch_frequency(q, self.a_ref, nf.cn_alpha, nf.cp - p.cg, i);
        Diag {
            acc,
            ang_acc,
            thrust: p.thrust,
            drag: -dot(aero_force, rd),
            mass: p.mass,
            cg: p.cg,
            mach: f.mach,
            air: f.air,
            air_speed: s,
            alpha,
            cd,
            open: 0.0,
            wind: f.wind,
            propellant: 0.0,
            reynolds: 0.0,
            axis: rotate(st.q, [1.0, 0.0, 0.0]),
            omega: st.w,
            stiff,
            pitch_freq,
            control_cant: 0.0,
        }
    }
}
