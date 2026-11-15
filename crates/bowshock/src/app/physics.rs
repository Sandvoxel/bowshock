//! The live numbers: what the status strip, the config menu and the motor picker show, computed by `bowshock-sim` from the
//! document and the motor database. One [`Physics`] lives in the app context; [`Physics::sync`] recomputes it when the
//! document's revision or the phase changed, so views only ever read the cache.
//!
//! The numbers are the static ones of [`from_model::balance`]: Barrowman CP at Mach 0.3 and zero angle of attack, mass and CG
//! of the dry parts plus the loaded motors at liftoff (and the burned-out casings, [`Physics::burnout`]).

use bowshock_model::{ConfigId, MotorMount, MotorRef, PartId, Rocket, Stage};
use bowshock_motors::{Motor, MotorDb};
use bowshock_sim::from_model::{self, Balance, BridgeError, Level, Phase, Warning};

use crate::widgets::gauge::GaugeState;

/// Mach number of the builder's balance readout.
pub const BALANCE_MACH: f64 = 0.3;

/// Cached physics of the open document. See the module docs.
pub struct Physics {
    motors: MotorDb,
    /// What is flying: the whole stack or only the upper stages. A view setting, not part of the document.
    phase: Phase,
    /// The document revision and phase the cache was computed for.
    computed_for: Option<(u64, Phase)>,
    liftoff: Result<Balance, BridgeError>,
    burnout: Result<Balance, BridgeError>,
    warnings: Vec<Warning>,
    stages: Vec<Stage>,
    /// Margin of every configuration (whole stack, liftoff), in the document's order.
    config_margins: Vec<(ConfigId, Option<f64>)>,
    gauge: GaugeState,
}

impl Default for Physics {
    fn default() -> Self {
        Physics::new()
    }
}

impl Physics {
    /// An empty cache with the bundled motor database.
    pub fn new() -> Self {
        Physics {
            motors: MotorDb::bundled(),
            phase: Phase::Full,
            computed_for: None,
            liftoff: Err(BridgeError::EmptyRocket),
            burnout: Err(BridgeError::EmptyRocket),
            warnings: Vec::new(),
            stages: Vec::new(),
            config_margins: Vec::new(),
            gauge: GaugeState::new(0.0),
        }
    }

    /// The motor database.
    pub fn motors(&self) -> &MotorDb {
        &self.motors
    }

    /// The phase on show.
    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// Chooses the phase on show (the next [`sync`](Self::sync) recomputes).
    pub fn set_phase(&mut self, phase: Phase) {
        self.phase = phase;
    }

    /// Recomputes if `revision` or the phase changed since the last time. Returns whether it did.
    pub fn sync(&mut self, rocket: &Rocket, revision: u64) -> bool {
        if self.computed_for == Some((revision, self.phase)) {
            return false;
        }
        // A phase that no longer exists (stages were removed) falls back to the whole stack.
        let stages = rocket.stages();
        if self.phase.first_stage() > stages.len().max(1) {
            self.phase = Phase::Full;
        }
        self.compute(rocket, stages);
        self.computed_for = Some((revision, self.phase));
        true
    }

    fn compute(&mut self, rocket: &Rocket, stages: Vec<Stage>) {
        self.liftoff = from_model::balance(rocket, &self.motors, self.phase, BALANCE_MACH, 0.0, true);
        self.burnout = from_model::balance(rocket, &self.motors, self.phase, BALANCE_MACH, 0.0, false);
        self.warnings = match &self.liftoff {
            Ok(balance) => from_model::warnings(rocket, &self.motors, balance),
            Err(_) => from_model::warnings(
                rocket,
                &self.motors,
                &Balance { mass: 0.0, length: 0.0, reference_diameter: 0.0, cg: 0.0, cp: 0.0, margin_cal: 0.0, cn_alpha: 0.0, motor_mass: 0.0 },
            ),
        };
        self.stages = stages;
        self.config_margins = rocket.configurations().iter().map(|c| (c.id, self.margin_of(rocket, c.id))).collect();
        if let Ok(balance) = &self.liftoff {
            self.gauge.set(balance.margin_cal as f32);
        }
    }

    /// The margin the whole stack would have in configuration `id` at liftoff.
    fn margin_of(&self, rocket: &Rocket, id: ConfigId) -> Option<f64> {
        let mut copy = rocket.clone();
        copy.select_configuration(id);
        from_model::balance(&copy, &self.motors, Phase::Full, BALANCE_MACH, 0.0, true).ok().map(|b| b.margin_cal)
    }

    /// Mass, length, CG, CP and margin of the phase on show at liftoff; `None` for a rocket without parts.
    pub fn balance(&self) -> Option<&Balance> {
        self.liftoff.as_ref().ok()
    }

    /// The same with the motors burned out (casings only).
    #[allow(dead_code)] // For Analyze and the lanes (CG at burnout).
    pub fn burnout(&self) -> Option<&Balance> {
        self.burnout.as_ref().ok()
    }

    /// Why there is no balance (an empty rocket, a motor that is not in the database).
    #[allow(dead_code)]
    pub fn error(&self) -> Option<&BridgeError> {
        self.liftoff.as_ref().err()
    }

    /// The health list of the active configuration, worst first within each topic, including `Ok` readings.
    #[allow(dead_code)]
    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }

    /// The warnings that are not mere `Ok` readings: what the status strip's chip counts.
    pub fn issues(&self) -> impl Iterator<Item = &Warning> {
        self.warnings.iter().filter(|w| w.level != Level::Ok)
    }

    /// The worst level among [`issues`](Self::issues), `Ok` when there are none.
    pub fn worst(&self) -> Level {
        self.issues().map(|w| w.level).max().unwrap_or(Level::Ok)
    }

    /// The stages of the document, firing order (stage 1 first).
    pub fn stages(&self) -> &[Stage] {
        &self.stages
    }

    /// Whether the rocket has more than one stage (the Phase switch appears).
    pub fn is_staged(&self) -> bool {
        self.stages.len() > 1
    }

    /// The margin of configuration `id` (whole stack at liftoff) for the config menu, `None` when it cannot be computed.
    pub fn config_margin(&self, id: ConfigId) -> Option<f64> {
        self.config_margins.iter().find(|(c, _)| *c == id).and_then(|(_, m)| *m)
    }

    /// The margin the whole stack would have at liftoff with `motor` in `mount` of the active configuration (every other
    /// mount as configured). The motor picker's "With this motor" column.
    pub fn margin_with_motor(&self, rocket: &Rocket, mount: PartId, motor: &Motor) -> Option<f64> {
        let mut copy = rocket.clone();
        let config = copy.active_config_id();
        copy.set_motor(config, mount, Some(MotorRef::new(motor.designation.clone(), motor.manufacturer.clone()))).ok()?;
        from_model::balance(&copy, &self.motors, Phase::Full, BALANCE_MACH, 0.0, true).ok().map(|b| b.margin_cal)
    }

    /// The motor mounts of `rocket` that belong to model stage `stage` (from 1), fore to aft.
    pub fn mounts_of_stage(&self, rocket: &Rocket, stage: usize) -> Vec<MotorMount> {
        let Some(stage) = self.stages.iter().find(|s| s.index == stage) else {
            return Vec::new();
        };
        rocket.motor_mounts().into_iter().filter(|m| stage.mounts.contains(&m.id)).collect()
    }

    /// The margin gauge the status strip draws; animated, see [`GaugeState`].
    pub fn gauge(&self) -> &GaugeState {
        &self.gauge
    }

    /// Advances the gauge animation.
    pub fn tick(&mut self, now: std::time::Instant) {
        self.gauge.tick(now);
    }

    /// Whether the gauge needs frame ticks.
    pub fn animating(&self) -> bool {
        self.gauge.moving()
    }

    /// Jumps the gauge to the current margin without animating (after loading a document).
    pub fn snap_gauge(&mut self) {
        if let Ok(balance) = &self.liftoff {
            self.gauge.snap(balance.margin_cal as f32);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind, RadialKind};

    fn finned() -> Rocket {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.9, diameter: 0.0762 }).unwrap();
        r.attach_radial(tube, 0.78, 0.0, 3, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.003 }).unwrap();
        r
    }

    #[test]
    fn sync_recomputes_only_when_the_revision_or_phase_changes() {
        let mut physics = Physics::new();
        let rocket = finned();
        assert!(physics.sync(&Rocket::default(), 0));
        assert!(physics.balance().is_none() && physics.error() == Some(&BridgeError::EmptyRocket));
        assert!(physics.sync(&rocket, 1));
        assert!(!physics.sync(&rocket, 1));
        let balance = *physics.balance().unwrap();
        // A finned rocket: CP aft of CG, positive margin, length and diameter from the geometry.
        assert!(balance.cp > balance.cg && balance.margin_cal > 0.0);
        assert!((balance.length - 1.1286).abs() < 1e-9 && (balance.reference_diameter - 0.0762).abs() < 1e-9);
        // Burnout has no more mass than liftoff.
        assert!(physics.burnout().unwrap().mass <= balance.mass);
        assert!(physics.sync(&rocket, 2));
        // The only configuration's margin is the strip's.
        assert_eq!(physics.config_margin(rocket.active_config_id()), Some(balance.margin_cal));
        assert!(!physics.is_staged());
    }

    #[test]
    fn issues_are_the_warnings_that_are_not_ok_readings() {
        let mut physics = Physics::new();
        physics.sync(&finned(), 1);
        // No motor and no recovery: warnings, but no errors; the stability reading is Ok and is not counted.
        assert!(physics.warnings().iter().any(|w| w.level == Level::Ok));
        assert!(physics.issues().count() >= 2 && physics.issues().all(|w| w.level != Level::Ok));
        assert_eq!(physics.worst(), Level::Warn);
    }
}
