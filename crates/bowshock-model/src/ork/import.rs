use std::collections::HashMap;

use roxmltree::Node;

use super::xml::*;
use super::{Builder, IgnitionSpec, Import, MountRec, OrkSimulation, OrkStored, SeparationRec};
use crate::config::{ConfigId, EventKind, StageEvent, StageTrigger};
use crate::flights::{Conditions, FlightDef, FlightMotor, Turbulence};
use crate::internal::{InternalKind, InternalPart};

impl<'w> Import<'w> {
    pub(in crate::ork) fn new(w: &'w mut Vec<String>) -> Import<'w> {
        Import { b: Builder::new(w), config_ids: Vec::new(), config_map: HashMap::new(), simulations: Vec::new(), motors: Vec::new(), separations: Vec::new() }
    }

    pub(in crate::ork) fn rocket_node(&mut self, n: Node) {
        self.b.r.name = name_of(n).unwrap_or_default();
        self.b.r.designer = text(n, "designer").unwrap_or("").to_string();
        match text(n, "referencetype") {
            None | Some("maximum") => {}
            Some(other) => self.b.warn(format!("reference type \"{other}\" is not modelled; the largest body diameter is the reference")),
        }
        // Configurations, in file order; the default one is active.
        let mut default = None;
        for (i, c) in children(n, "motorconfiguration").enumerate() {
            let Some(id) = c.attribute("configid") else {
                continue;
            };
            let cid = if i == 0 { self.b.r.configs[0].id } else { self.b.r.add_configuration("") };
            self.b.r.rename_configuration(cid, text(c, "name").unwrap_or(""));
            self.config_ids.push(id.to_string());
            self.config_map.insert(id.to_string(), cid);
            if c.attribute("default") == Some("true") {
                default = Some(cid);
            }
        }
        if let Some(d) = default {
            self.b.r.select_configuration(d);
        }
        // The stack, top stage first.
        if let Some(subs) = child(n, "subcomponents") {
            let axial: Vec<Node> = children(subs, "stage")
                .flat_map(|s| child(s, "subcomponents").into_iter().flat_map(|c| c.children().filter(Node::is_element)))
                .filter(|c| matches!(c.tag_name().name(), "nosecone" | "bodytube" | "transition"))
                .collect();
            self.b.plan_radii(&axial);
            for (index, stage) in children(subs, "stage").enumerate() {
                self.b.stage = index;
                let mut rec = SeparationRec::default();
                if let Some(e) = text(stage, "separationevent") {
                    rec.default = Some((e.to_string(), num(stage, "separationdelay").unwrap_or(0.0)));
                }
                for c in children(stage, "separationconfiguration") {
                    rec.per_config.push((
                        c.attribute("configid").unwrap_or("").to_string(),
                        text(c, "separationevent").unwrap_or("burnout").to_string(),
                        num(c, "separationdelay").unwrap_or(0.0),
                    ));
                }
                self.separations.push(rec);
                let first = self.b.r.next_id;
                if let Some(s) = child(stage, "subcomponents") {
                    self.b.stack(s);
                }
                self.b.stage_override(stage, first);
            }
        }
        self.apply_configurations();
    }

    /// An upper stage's motor may stick out behind its airframe by a few millimetres (OpenRocket's default overhang is 3 mm, and
    /// the motor simply reaches into the next stage). The model wants the stage joint at or aft of the motor's aft end, so such a
    /// motor is pulled in (the mount's overhang shortened) until the joint is at its aft end; the shift is reported.
    pub(in crate::ork) fn tuck_in_overhangs(&mut self) {
        const REACH: f64 = 0.02;
        for _ in 0..self.b.r.internals.len() {
            let Some(station) = self
                .b
                .r
                .stage_issues()
                .into_iter()
                .map(|(_, i)| match i {
                    crate::StageIssue::NoJoint { station, .. } => station,
                })
                .next()
            else {
                return;
            };
            let stations = self.b.r.stations();
            let joint = stations[1..stations.len().saturating_sub(1)]
                .iter()
                .copied()
                .filter(|j| *j < station && station - j <= REACH)
                .fold(f64::NEG_INFINITY, f64::max);
            let mount = self.b.r.motor_mounts().into_iter().filter(|m| m.pod.is_none() && (m.motor_aft_station - station).abs() < 1e-9).map(|m| m.id).next();
            let (true, Some(mount)) = (joint.is_finite(), mount) else {
                return;
            };
            if let Some(part) = self.b.r.internals.iter_mut().find(|p| p.id == mount) {
                if let InternalKind::InnerTube { overhang, .. } = &mut part.kind {
                    *overhang -= station - joint;
                }
            }
            let name = self.b.r.internal(mount).map(InternalPart::display_name).unwrap_or_default();
            self.b.warn(format!(
                "{name}: its motor reaches {:.1} mm past a stage joint; the motor was moved that far forward so the stage separates at the joint",
                (station - joint) * 1e3
            ));
        }
    }

    /// Motors, ignition, separation and recovery timing of every configuration.
    pub(in crate::ork) fn apply_configurations(&mut self) {
        self.tuck_in_overhangs();
        let mounts = self.b.mounts.clone();
        let recovery = self.b.recovery.clone();
        // OpenRocket stages with a motor mount of the core, top first, are the model's stages from the last (index 1 = aft-most).
        let core_mounts: Vec<&MountRec> = mounts.iter().filter(|m| self.b.r.internals.iter().any(|i| i.id == m.part)).collect();
        let mut stages: Vec<usize> = core_mounts.iter().map(|m| m.stage).collect();
        stages.sort_unstable();
        stages.dedup();
        let model_stage = |or_stage: usize| stages.iter().position(|s| *s == or_stage).map(|i| stages.len() - i);

        for (or_id, &cid) in self.config_ids.clone().iter().zip(self.config_ids.clone().iter().map(|k| self.config_map[k]).collect::<Vec<_>>().iter()) {
            for m in &mounts {
                let Some(spec) = m.motors.iter().find(|s| &s.config == or_id) else {
                    continue;
                };
                let ignition = m.ignition.iter().find(|i| &i.config == or_id).or_else(|| m.ignition.iter().find(|i| i.config.is_empty()));
                let in_core = self.b.r.internals.iter().any(|i| i.id == m.part);
                let label = spec.motor.designation.clone();
                if matches!(ignition, Some(i) if i.event == "never") {
                    self.b.warn(format!("motor {label}: ignition \"never\" is not modelled; the mount is left empty in that configuration"));
                    continue;
                }
                if self.b.r.set_motor(cid, m.part, Some(spec.motor.clone())).is_err() {
                    self.b.warn(format!("motor {label}: its mount is not a motor mount of the model; left out"));
                    continue;
                }
                let _ = self.b.r.set_motor_delay(cid, m.part, spec.delay);
                if !self.motors.contains(&spec.motor) {
                    self.motors.push(spec.motor.clone());
                }
                if in_core {
                    if let (Some(stage), Some(ign)) = (model_stage(m.stage), ignition) {
                        self.ignition_event(cid, stage, ign, &label);
                    }
                }
            }
            for (index, rec) in self.separations.clone().iter().enumerate() {
                let Some(stage) = model_stage(index) else {
                    continue;
                };
                if stage == stages.len() {
                    continue; // the top stage has nothing above it to separate from
                }
                let event = rec.per_config.iter().find(|c| &c.0 == or_id).map(|c| (c.1.clone(), c.2)).or_else(|| rec.default.clone());
                if let Some((event, delay)) = event {
                    self.separation_event(cid, stage, &event, delay, &mounts, or_id, index);
                }
            }
            for rec in &recovery {
                if let Some((_, d)) = rec.per_config.iter().find(|c| &c.0 == or_id) {
                    let _ = self.b.r.set_deployment(cid, rec.part, *d);
                }
            }
        }
    }

    pub(in crate::ork) fn ignition_event(&mut self, cid: ConfigId, stage: usize, ign: &IgnitionSpec, label: &str) {
        let (trigger, delay) = match ign.event.as_str() {
            "launch" => (StageTrigger::TimeFromLaunch, ign.delay),
            "burnout" if stage > 1 => (StageTrigger::Burnout, ign.delay),
            "burnout" => (StageTrigger::TimeFromLaunch, ign.delay),
            "ejection" if stage > 1 => {
                self.b.warn(format!("motor {label}: ignition at the ejection charge of the stage below is approximated by that stage's burnout"));
                (StageTrigger::Burnout, ign.delay)
            }
            // `automatic`: launch for the first stage, the separation of the stage below for the others (the model's defaults).
            _ if stage > 1 => (StageTrigger::Separation, ign.delay),
            _ => (StageTrigger::TimeFromLaunch, ign.delay),
        };
        let _ = self.b.r.set_stage_event(cid, StageEvent { stage, kind: EventKind::Ignition, trigger, delay, tilt_lockout: None });
    }

    pub(in crate::ork) fn separation_event(&mut self, cid: ConfigId, stage: usize, event: &str, delay: f64, mounts: &[MountRec], or_id: &str, or_stage: usize) {
        let ejection_delay = mounts
            .iter()
            .filter(|m| m.stage == or_stage)
            .find_map(|m| m.motors.iter().find(|s| s.config == or_id).map(|s| s.delay))
            .filter(|d| d.is_finite())
            .unwrap_or(0.0);
        let (trigger, delay) = match event {
            "launch" => (StageTrigger::TimeFromLaunch, delay),
            "burnout" => (StageTrigger::Burnout, delay),
            "ejection" => (StageTrigger::Burnout, delay + ejection_delay),
            "upperignition" => {
                self.b.warn(format!("stage {stage} separation at the ignition of the upper stage is approximated by burnout"));
                (StageTrigger::Burnout, delay)
            }
            "never" => {
                self.b.warn(format!("stage {stage} separation \"never\" is not modelled; it separates at burnout"));
                (StageTrigger::Burnout, 0.0)
            }
            other => {
                self.b.warn(format!("stage {stage} separation event \"{other}\" is not modelled; it separates at burnout"));
                (StageTrigger::Burnout, 0.0)
            }
        };
        let _ = self.b.r.set_stage_event(cid, StageEvent { stage, kind: EventKind::Separation, trigger, delay, tilt_lockout: None });
    }

    pub(in crate::ork) fn simulations(&mut self, sims: Option<Node>) {
        let Some(sims) = sims else {
            return;
        };
        let mut flights = Vec::new();
        for (i, s) in children(sims, "simulation").enumerate() {
            let Some(c) = child(s, "conditions") else {
                continue;
            };
            let config = c.children().find(|x| x.tag_name().name() == "configid").and_then(|x| x.text()).and_then(|t| self.config_map.get(t.trim()).copied());
            let altitude = num(c, "launchaltitude").unwrap_or(0.0);
            let atmosphere = child(c, "atmosphere");
            let isa = atmosphere.and_then(|a| a.attribute("model")).is_none_or(|m| m == "isa");
            let temperature = match (isa, atmosphere.and_then(|a| a.attribute("temperature")).and_then(|t| t.parse::<f64>().ok())) {
                (false, Some(t)) => t,
                _ => 288.15 - 0.0065 * altitude,
            };
            let name = name_of(s).unwrap_or_else(|| format!("Simulation {}", i + 1));
            let sim = OrkSimulation {
                name: name.clone(),
                config,
                rod_length: num(c, "launchrodlength").unwrap_or(1.0),
                rod_angle: deg(num(c, "launchrodangle").unwrap_or(0.0)),
                rod_direction: deg(num(c, "launchroddirection").unwrap_or(90.0)),
                wind_average: num(c, "windaverage").unwrap_or(2.0),
                wind_turbulence: num(c, "windturbulence").unwrap_or(0.1),
                altitude,
                latitude: num(c, "launchlatitude").unwrap_or(45.0),
                isa,
                temperature,
                pressure: atmosphere.and_then(|a| a.attribute("pressure")).and_then(|t| t.parse().ok()),
                time_step: num(c, "timestep").unwrap_or(0.05),
                stored: child(s, "flightdata").map(stored),
            };
            let turbulence = match sim.wind_turbulence {
                t if t < 0.025 => Turbulence::Off,
                t if t < 0.075 => Turbulence::Low,
                t if t < 0.15 => Turbulence::Medium,
                _ => Turbulence::High,
            };
            let conditions = Conditions {
                rail_length: sim.rod_length,
                launch_angle: sim.rod_angle,
                wind_speed: sim.wind_average,
                // OpenRocket's default direction (and the app's launch) is into a wind from 90 degrees.
                wind_from: deg(num(c, "winddirection").unwrap_or(90.0)),
                turbulence,
                site_altitude: altitude,
                temperature,
                field_radius: 300.0,
                // OpenRocket stores a latitude (default 45°) for its Earth model; a stored one becomes the launch location.
                location: num(c, "launchlatitude").map(|latitude| crate::flights::Location { latitude, longitude: num(c, "launchlongitude").unwrap_or(0.0) }),
                wind_levels: Vec::new(),
                air_start: None,
            };
            flights.push(FlightDef {
                id: flights.len() as u64 + 1,
                name,
                motor: config.map_or(FlightMotor::Active, FlightMotor::Config),
                delay: None,
                conditions,
                drag: Default::default(),
            });
            self.simulations.push(sim);
        }
        if !flights.is_empty() {
            self.b.r.set_flights(flights);
        }
    }
}

fn stored(f: Node) -> OrkStored {
    let at = |name: &str| f.attribute(name).and_then(|v| v.parse::<f64>().ok());
    OrkStored {
        max_altitude: at("maxaltitude"),
        max_velocity: at("maxvelocity"),
        max_acceleration: at("maxacceleration"),
        max_mach: at("maxmach"),
        time_to_apogee: at("timetoapogee"),
        flight_time: at("flighttime"),
        ground_hit_velocity: at("groundhitvelocity"),
        launch_rod_velocity: at("launchrodvelocity"),
    }
}
