use super::drag::{body_calc, button_drag, fin_calc};
use super::tables::{base_cd, blunt_nose_cd, cap_cd, roughness_correction, skin_cf, stagnation_cd};
use super::{
    axial_drag_multiplier, fin_fin_factor, lug_drag, Aero, Comp, ComponentAero, DragBreakdown, FinCalc, NormalForce, Pressure, ProtCalc, RollAero, Stability,
    LUG_ROUGHNESS,
};
use crate::atmosphere::Atmosphere;
use crate::drag_table::{DragTable, DragTablePoint, PowerRatio};
use crate::tube_fins::TubeFinCalc;
use crate::vehicle::{Protuberance, Vehicle};
use std::f64::consts::PI;
use std::sync::Arc;
impl Aero {
    /// Precomputes geometry-dependent quantities for `v`. Surface roughness uses the
    /// rough-surface model (fully turbulent, roughness-limited; the default); see [`Aero::perfect_finish`].
    pub fn new(v: &Vehicle) -> Self {
        let a_ref = v.reference_area().max(1e-12);
        let length = v.length();
        let mut bodies = Vec::with_capacity(v.body.len());
        for (i, s) in v.body.iter().enumerate() {
            let prev_aft = if i == 0 { 0.0 } else { v.body[i - 1].aft_radius };
            let next_fore = v.body.get(i + 1).map(|n| n.fore_radius);
            bodies.push(body_calc(s, prev_aft, next_fore, a_ref));
        }
        let max_r = v.body.iter().map(|s| s.fore_radius.max(s.aft_radius)).fold(0.0, f64::max);
        let min_x = v.body.iter().map(|s| s.station).fold(f64::INFINITY, f64::min);
        let body_length = if v.body.is_empty() { 0.0 } else { length - min_x + 0.0001 };
        let mut fins: Vec<FinCalc> = v
            .fin_sets
            .iter()
            .enumerate()
            .map(|(i, f)| fin_calc(f, if v.fin_sets.len() > 1 { format!("Fin set {}", i + 1) } else { "Fins".to_string() }))
            .collect();
        // Fin-fin interference: the fins of every set on the same body whose root chords overlap act together.
        let together: Vec<f64> = fins
            .iter()
            .map(|f| {
                fins.iter().filter(|g| g.parent == f.parent && g.root.0 < f.root.1 && g.root.1 > f.root.0).map(|g| g.per_body).sum::<f64>().max(f.per_body)
            })
            .collect();
        for (f, n) in fins.iter_mut().zip(together) {
            let ff = fin_fin_factor(n.round() as u32);
            f.lift *= ff;
            f.unit_lift *= ff;
        }
        let prots = v
            .protuberances
            .iter()
            .map(|p| match *p {
                Protuberance::LaunchLug { length, outer_diameter, inner_diameter, .. } => ProtCalc::Lug {
                    inner_diameter,
                    inner_area: 0.25 * PI * inner_diameter * inner_diameter,
                    frontal: 0.25 * PI * (outer_diameter * outer_diameter - inner_diameter * inner_diameter).max(0.0),
                    length,
                    roughness: LUG_ROUGHNESS,
                },
                Protuberance::RailButton { station, height, diameter } => ProtCalc::Button { station, height, diameter },
            })
            .collect();
        let motor_d = v
            .stages
            .iter()
            .filter_map(|s| s.motor.as_ref())
            .filter(|m| m.mount_aft_station - m.curve.length < length)
            .map(|m| m.curve.diameter)
            .fold(0.0, f64::max);
        Self {
            d_ref: v.reference_diameter,
            a_ref,
            length,
            body_length,
            max_diameter: 2.0 * max_r,
            motor_exit_area: 0.25 * PI * motor_d * motor_d,
            perfect_finish: false,
            bodies,
            fins,
            tube_fins: v.tube_fins.iter().map(TubeFinCalc::new).collect(),
            prots,
            labels: v.labels.clone(),
            table: None,
        }
    }

    /// Selects the smooth-surface skin friction (laminar-to-turbulent transition,
    /// roughness only limiting above Re 1e6) instead of the default fully turbulent,
    /// roughness-limited model.
    pub fn perfect_finish(mut self, on: bool) -> Self {
        self.perfect_finish = on;
        self
    }

    /// Reference diameter (m).
    pub fn reference_diameter(&self) -> f64 {
        self.d_ref
    }

    /// Normal-force slope and CP at `mach` and angle of attack `alpha` (rad).
    ///
    /// The slope includes the nonlinear Galejs body lift evaluated at `alpha`, so
    /// `cn_alpha * alpha` is the normal force coefficient `CN` there.
    pub fn stability(&self, mach: f64, alpha: f64) -> Stability {
        let mut comps: Vec<(String, f64, f64)> = Vec::new();
        self.each_component(mach, alpha, |_, name, cna, cp| comps.push((name(), cna, cp)));
        let cn_alpha: f64 = comps.iter().map(|c| c.1).sum();
        let cp = if cn_alpha.abs() > 1e-9 { comps.iter().map(|c| c.1 * c.2).sum::<f64>() / cn_alpha } else { 0.5 * self.length };
        Stability { cn_alpha, cp, components: comps }
    }

    /// Calls `f(component, name, CN_alpha per rad, CP station)` for every component with lift at `mach` and `alpha`; the name
    /// is only built when `f` asks for it, so the flight code can sum the components without allocating.
    fn each_component(&self, mach: f64, alpha: f64, mut f: impl FnMut(Comp, &dyn Fn() -> String, f64, f64)) {
        let sin_a = alpha.abs().sin();
        let sinc = if alpha.abs() > 1e-9 { sin_a / alpha.abs() } else { 1.0 };
        for (i, b) in self.bodies.iter().enumerate() {
            let mut cna = b.cna * sinc;
            let mut moment = cna * b.cp;
            if alpha.abs() > 1e-9 {
                // Galejs body lift: CN = 1.1 A_plan / A_ref sin^2(alpha), at the planform centroid.
                let extra = 1.1 * b.plan_area / self.a_ref * sin_a * sin_a / alpha.abs();
                cna += extra;
                moment += extra * b.plan_cp;
            }
            if cna.abs() > 1e-12 {
                f(Comp::Body(i), &|| b.name.clone(), cna, moment / cna);
            }
        }
        for (i, fin) in self.fins.iter().enumerate() {
            f(Comp::Fin(i), &|| fin.name.clone(), fin.cna(mach, alpha, self.a_ref), fin.cp(mach));
        }
        for (i, t) in self.tube_fins.iter().enumerate() {
            let cna = t.cna(alpha, self.a_ref);
            if cna.abs() > 1e-12 {
                f(Comp::TubeFin(i), &|| if self.tube_fins.len() > 1 { format!("Tube fins {}", i + 1) } else { "Tube fins".to_string() }, cna, t.cp(mach));
            }
        }
    }

    /// Index of `c` in the list [`Aero::components`] returns: bodies, fin sets, tube fin sets, protuberances.
    fn flat_index(&self, c: Comp) -> usize {
        let (nb, nf, nt) = (self.bodies.len(), self.fins.len(), self.tube_fins.len());
        match c {
            Comp::Body(i) => i,
            Comp::Fin(i) => nb + i,
            Comp::TubeFin(i) => nb + nf + i,
            Comp::Prot(i) => nb + nf + nt + i,
        }
    }

    /// The drag override of component `c`, if its label carries one.
    fn cd_override(&self, c: Comp) -> Option<f64> {
        let l = &self.labels;
        let list = match c {
            Comp::Body(i) => l.body.get(i),
            Comp::Fin(i) => l.fin_sets.get(i),
            Comp::TubeFin(i) => l.tube_fins.get(i),
            Comp::Prot(i) => l.protuberances.get(i),
        };
        list.and_then(|l| l.cd_override)
    }

    /// Normal force and drag of every component at `mach`, `re_per_m` (Reynolds number per metre) and angle of attack
    /// `alpha`, from the build-up (a CFD table is not split by component): the bodies, fin sets, tube fin sets and
    /// protuberances in the vehicle's order, named by the vehicle's labels. The drags sum to [`Aero::drag_at_alpha`]
    /// without a table.
    pub fn components(&self, mach: f64, re_per_m: f64, motor_on: bool, alpha: f64) -> Vec<ComponentAero> {
        let k = axial_drag_multiplier(alpha);
        let mut out = Vec::with_capacity(self.bodies.len() + self.fins.len() + self.tube_fins.len() + self.prots.len());
        self.each_drag(mach, re_per_m, motor_on, |c, d| {
            let label = match c {
                Comp::Body(i) => self.labels.body(i),
                Comp::Fin(i) => self.labels.fin_set(i),
                Comp::TubeFin(i) => self.labels.tube_fin(i),
                Comp::Prot(i) => self.labels.protuberance(i),
            };
            let overridden = label.cd_override.is_some();
            out.push(ComponentAero { name: label.name, part: label.part, cn_alpha: 0.0, cp: f64::NAN, drag: d.scaled(k), overridden });
        });
        self.each_component(mach, alpha, |c, _, cna, cp| {
            let i = self.flat_index(c);
            out[i].cn_alpha = cna;
            out[i].cp = cp;
        });
        out
    }

    /// Normal-force slope and CP at `mach` and angle of attack `alpha` (rad) together with the sums behind the pitch damping
    /// moment ([`NormalForce::damping_sum`]), without the per-component list of [`Aero::stability`] (no allocation): what the
    /// rigid-body flight model evaluates every step.
    pub fn normal_force(&self, mach: f64, alpha: f64) -> NormalForce {
        let (mut s0, mut s1, mut s2) = (0.0, 0.0, 0.0);
        self.each_component(mach, alpha, |_, _, cna, cp| {
            s0 += cna;
            s1 += cna * cp;
            s2 += cna * cp * cp;
        });
        NormalForce { cn_alpha: s0, cp: if s0.abs() > 1e-9 { s1 / s0 } else { 0.5 * self.length }, s1, s2 }
    }

    /// Static margin in calibers (positive = stable) for centre of gravity station `cg`.
    pub fn margin_cal(&self, mach: f64, alpha: f64, cg: f64) -> f64 {
        (self.stability(mach, alpha).cp - cg) / self.d_ref
    }

    /// Body crossflow damping factor about a centre of gravity at station `cg`: `sum A_strip |x - cg|^3` (m^5) over strips of
    /// the body planform. A body turning at rate `w` about its CG meets a crossflow `w (x - cg)` at each station and the
    /// Hoerner crossflow drag (coefficient [`BODY_CROSSFLOW_CD`], the one behind the body lift of [`Aero::stability`]) gives the
    /// restoring moment `-1/2 rho BODY_CROSSFLOW_CD K w |w|` (Zipfel 2014, *Modeling and Simulation of Aerospace Vehicle
    /// Dynamics*, the strip-theory damping of slender bodies).
    pub fn crossflow_damping(&self, cg: f64) -> f64 {
        self.bodies.iter().flat_map(|b| b.strips.iter()).map(|&(x, area)| area * (x - cg).abs().powi(3)).sum()
    }

    /// Fin roll forcing and damping at `mach` (see [`RollAero`]): the fins of the core body only.
    pub fn roll(&self, mach: f64) -> RollAero {
        let (mut forcing, mut damping) = (0.0, 0.0);
        for f in self.fins.iter().filter(|f| f.on_core) {
            let lift = f.n * f.unit_lift * f.cna1(mach, 0.0, self.a_ref) * self.a_ref;
            forcing += lift * f.cant * f.roll_arm;
            damping -= lift * f.roll_arm2;
        }
        RollAero { forcing, damping }
    }

    /// Zero-angle-of-attack drag at `mach` with `re_per_m = v/nu` (Reynolds number per metre).
    /// With `motor_on` the base area covered by the motor exhaust carries no base drag.
    pub fn drag(&self, mach: f64, re_per_m: f64, motor_on: bool) -> DragBreakdown {
        match &self.table {
            None => self.buildup(mach, re_per_m, motor_on),
            Some(table) => self.tabulated(table, mach, re_per_m, motor_on),
        }
    }

    /// Whether any component carries a drag override (which only the build-up applies, not a CFD table).
    pub fn has_cd_override(&self) -> bool {
        self.labels.any_cd_override()
    }

    /// This model flying on `table`: its parts at the table's Reynolds number, corrected by the build-up for the
    /// Reynolds number and motor exhaust of the actual condition. The table is the whole vehicle's drag, so the components'
    /// drag overrides no longer apply.
    pub fn with_table(mut self, table: DragTable) -> Aero {
        self.table = Some(Arc::new(table));
        self.labels = self.labels.without_cd_overrides();
        self
    }

    /// Drag from the CFD table; see [`Aero::with_table`]. With the motor on and a table solved with the jet, each part is the table's times the jet's effect at this Mach number
    /// ([`DragTable::power_ratio`]: the base annulus around the exit with the jet on over the whole base without it); without one the build-up takes the exit area out of the base
    /// drag.
    fn tabulated(&self, table: &DragTable, mach: f64, re_per_m: f64, motor_on: bool) -> DragBreakdown {
        let m = mach.max(0.0);
        let air = Atmosphere::standard().at_msl(table.altitude);
        let re_tab = |m: f64| m * air.speed_of_sound * air.density / air.viscosity;
        let jet = if motor_on { table.power_ratio(m) } else { None };
        let now = self.buildup(m, re_per_m, motor_on && jet.is_none());
        let tab = self.buildup(m, re_tab(m), false);
        let t = table.inside(m).unwrap_or_else(|| {
            let (first, last) = (&table.points[0], &table.points[table.points.len() - 1]);
            let end = if m > last.mach { last } else { first };
            let at_end = self.buildup(end.mach, re_tab(end.mach), false);
            let scale = |end: f64, here: f64, there: f64| {
                let r = here / there;
                if there.abs() < 1e-12 || !r.is_finite() {
                    end
                } else {
                    end * r
                }
            };
            DragTablePoint {
                mach: m,
                friction: scale(end.friction, tab.friction, at_end.friction),
                pressure: scale(end.pressure, tab.pressure, at_end.pressure),
                base: scale(end.base, tab.base, at_end.base),
                fins: scale(end.fins, tab.fins, at_end.fins),
                other: scale(end.other, tab.protuberances, at_end.protuberances),
            }
        });
        let jet = jet.unwrap_or(PowerRatio { mach: m, friction: 1.0, pressure: 1.0, base: 1.0 });
        let friction = (t.friction * jet.friction + now.friction - tab.friction).max(0.0);
        let pressure = (t.pressure * jet.pressure + now.pressure - tab.pressure).max(0.0);
        let fins = (t.fins + now.fins - tab.fins).max(0.0);
        let protuberances = (t.other + now.protuberances - tab.protuberances).max(0.0);
        let base = if tab.base < 1e-12 { t.base * jet.base } else { t.base * jet.base * now.base / tab.base }.max(0.0);
        DragBreakdown { friction, pressure, base, fins, protuberances, total: friction + pressure + base + fins + protuberances }
    }

    /// The Barrowman build-up: the sum of [`Aero::each_drag`] over the components.
    fn buildup(&self, mach: f64, re_per_m: f64, motor_on: bool) -> DragBreakdown {
        let mut sum = DragBreakdown::default();
        self.each_drag(mach, re_per_m, motor_on, |_, d| {
            sum.friction += d.friction;
            sum.pressure += d.pressure;
            sum.base += d.base;
            sum.fins += d.fins;
            sum.protuberances += d.protuberances;
        });
        sum.summed()
    }

    /// Calls `f(component, drag)` for every component in the order of [`Aero::flat_index`] with its zero-alpha build-up
    /// drag: bodies carry friction, pressure and base drag, fin and tube fin sets `fins`, lugs and buttons `protuberances`.
    /// A component whose label has a drag override gets that value instead, all in its own category (`pressure` for a body).
    fn each_drag(&self, mach: f64, re_per_m: f64, motor_on: bool, mut f: impl FnMut(Comp, DragBreakdown)) {
        let m = mach.max(0.0);
        let ar = self.a_ref;
        let re_per_m = re_per_m.max(0.0);
        let re_len = (re_per_m * self.length).max(1.0);
        let stag = stagnation_cd(m);
        let base = base_cd(m);
        let mut emit = |c: Comp, d: DragBreakdown| {
            let d = match self.cd_override(c) {
                None => d,
                Some(v) => match c {
                    Comp::Body(_) => DragBreakdown { pressure: v, ..DragBreakdown::default() },
                    Comp::Fin(_) | Comp::TubeFin(_) => DragBreakdown { fins: v, ..DragBreakdown::default() },
                    Comp::Prot(_) => DragBreakdown { protuberances: v, ..DragBreakdown::default() },
                },
            };
            f(c, d.summed());
        };

        // Skin friction: one flat-plate Cf from the whole-rocket length, floored per finish by the
        // roughness-limited value 0.032 (eps/L)^0.2 (Schlichting; Niskanen 2009 eq. 3.80).
        let cf = skin_cf(re_len, m, self.perfect_finish);
        let rough_corr = roughness_correction(m);
        let comp_cf = |eps: f64| {
            let limited = 0.032 * (eps / self.length.max(1e-9)).powf(0.2) * rough_corr;
            if self.perfect_finish {
                if re_len > 1e6 && limited > cf {
                    limited
                } else {
                    cf
                }
            } else {
                cf.max(limited)
            }
        };
        // Cylindrical-body correction 1 + 1/(2 fB), fB = body length / max diameter (tech doc eq. 3.85).
        let f_b = self.body_length / self.max_diameter.max(1e-9);
        let body_friction = (1.0 + 1.0 / (2.0 * f_b.max(1e-9))) / ar;

        for (i, b) in self.bodies.iter().enumerate() {
            let friction = comp_cf(b.roughness) * b.wet_area * body_friction;
            // Pressure drag of bodies: nose/shoulder tables plus stagnation on forward-facing steps.
            let pressure = match &b.pressure {
                Pressure::None => 0.0,
                Pressure::Stagnation { area } => stag * area / ar,
                Pressure::Base { area, scale } => base * area * scale / ar,
                Pressure::Table { table, area } => table.at(m) * area / ar,
                Pressure::BluntTable { table, hemisphere, tip_ratio, area } => blunt_nose_cd(table.at(m), cap_cd(m, hemisphere), *tip_ratio) * area / ar,
            } + stag * b.front_face / ar;
            // Base drag; exhaust covers the motor exit area of the terminal base.
            let mut area = b.back_face;
            if motor_on && b.is_last {
                area = (area - self.motor_exit_area).max(0.0);
            }
            emit(Comp::Body(i), DragBreakdown { friction, pressure, base: base * area / ar, ..DragBreakdown::default() });
        }

        for (i, fin) in self.fins.iter().enumerate() {
            emit(Comp::Fin(i), DragBreakdown { fins: fin.drag(m, comp_cf(fin.roughness), stag, base, ar), ..DragBreakdown::default() });
        }
        // Tube fins: friction on the outer surface and the through-flow pressure drag, no base drag (`crate::tube_fins`).
        for (i, t) in self.tube_fins.iter().enumerate() {
            let fins = t.friction_cd(comp_cf(t.roughness()), ar) + t.pressure_cd(re_per_m, stag, base, ar);
            emit(Comp::TubeFin(i), DragBreakdown { fins, ..DragBreakdown::default() });
        }

        for (i, p) in self.prots.iter().enumerate() {
            let protuberances = match *p {
                ProtCalc::Lug { inner_diameter, inner_area, frontal, length, roughness } => {
                    lug_drag(re_per_m, inner_diameter, inner_area, frontal, length, roughness, stag, base, ar)
                }
                ProtCalc::Button { station, height, diameter } => button_drag(m, re_per_m, station, height, diameter, stag, ar),
            };
            emit(Comp::Prot(i), DragBreakdown { protuberances, ..DragBreakdown::default() });
        }
    }

    /// Drag at angle of attack `alpha`: the zero-alpha build-up scaled by the axial-drag
    /// multiplier (1 at 0 deg, 1.3 at 17 deg, falling to 0 at 90 deg), which is the drag
    /// component along the rocket axis.
    pub fn drag_at_alpha(&self, mach: f64, re_per_m: f64, motor_on: bool, alpha: f64) -> DragBreakdown {
        self.drag(mach, re_per_m, motor_on).scaled(axial_drag_multiplier(alpha))
    }
}
