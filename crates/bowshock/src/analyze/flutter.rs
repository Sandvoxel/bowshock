//! The fin flutter estimate of a rocket as the app shows it: one [`FinStock`] per fin set the simulation flies (its panel and the
//! shear modulus of its stock, with where that number comes from) flown along the configuration's nominal flight by
//! `bowshock_sim::flutter`. The Analyze screen and the inspector's health chip both take their numbers from here, so they always
//! agree on the margin.

use bowshock_model::{PartId, Rocket};
use bowshock_sim::flutter::{FlutterFin, FlutterGeometry, FlutterInputs, FlutterTrace, FlutterWarning};
use bowshock_sim::from_model;
use bowshock_sim::{Atmosphere, FlightResult, Launch, Vehicle};

/// Birch plywood's shear modulus (Pa), for a fin set whose group the document cannot name (a fallback only).
const FALLBACK_G: f64 = 0.85e9;

/// One fin set of the simulated vehicle and what its stiffness is taken from.
#[derive(Debug, Clone, PartialEq)]
pub struct FinStock {
    /// The fin group of the document it comes from.
    pub source: PartId,
    /// What to call it (`Fin set`, or `Fin set · fins B, C` for the copies that fly apart from the group).
    pub name: String,
    /// The panel and the shear modulus the estimate uses.
    pub fin: FlutterFin,
    /// Material name and where its shear modulus comes from (`Fiberglass · Bennett 2023 · G10, 600 ksi`), or `Fin override`.
    pub g_source: String,
    /// Whether the shear modulus is the fin set's own (a measured value) and not the material's.
    pub overridden: bool,
    /// Whether the tip-to-tip allowance doubled the shear modulus.
    pub tip_to_tip: bool,
    /// Whether the fins have a through-the-wall tab (the estimate takes the root as clamped either way).
    pub tabbed: bool,
}

/// The stock of each fin set of `vehicle`, in order. `rocket` names the groups and their materials; when the vehicle has other fin
/// sets than the document's groups (a stage subset), every set takes the first group's stock.
pub fn stocks(rocket: &Rocket, vehicle: &Vehicle) -> Vec<FinStock> {
    let sources = from_model::fin_set_sources(rocket);
    let aligned = sources.len() == vehicle.fin_sets.len();
    let first = sources.first().map(|(id, _)| *id);
    vehicle
        .fin_sets
        .iter()
        .enumerate()
        .map(|(i, set)| {
            let (source, copies) = if aligned { (sources[i].0, sources[i].1.clone()) } else { (first.unwrap_or_default(), Vec::new()) };
            let radial = rocket.radial(source);
            let siblings = sources.iter().filter(|(id, _)| *id == source).count();
            let mut name = radial.map_or_else(|| "Fin set".to_string(), |r| r.display_name());
            if siblings > 1 && !copies.is_empty() {
                let letters = copies.iter().map(|&c| char::from(b'A' + (c.min(25)) as u8).to_string()).collect::<Vec<_>>().join(", ");
                name = format!("{name} · fin{} {letters}", if copies.len() > 1 { "s" } else { "" });
            }
            let (g, g_source, overridden, tip_to_tip, tabbed) = match radial {
                Some(r) => {
                    let (label, overridden) = match r.shear_modulus {
                        Some(_) => ("Fin's own value".to_string(), true),
                        None => {
                            let (_, source) = r.material_shear();
                            (format!("{} · {source}", r.material.name()), false)
                        }
                    };
                    (r.flutter_shear_modulus(), label, overridden, r.tip_to_tip, r.tab.is_some())
                }
                None => (FALLBACK_G, "Birch plywood estimate".to_string(), false, false, false),
            };
            FinStock { source, name, fin: FlutterFin { geometry: FlutterGeometry::of(set), shear_modulus: g }, g_source, overridden, tip_to_tip, tabbed }
        })
        .collect()
}

/// The estimate of a rocket's fins along a flight.
#[derive(Debug, Clone, PartialEq)]
pub struct FlutterAnalysis {
    /// Each fin set of the vehicle.
    pub stocks: Vec<FinStock>,
    /// The margin along the flight up to apogee.
    pub trace: FlutterTrace,
    /// The flight samples `(time s, altitude above the site m, airspeed m/s)` the trace was made from.
    samples: Vec<(f64, f64, f64)>,
    /// Apogee of the flight (m above the site).
    pub apogee: f64,
}

impl FlutterAnalysis {
    /// The estimate of `stocks` along `flight` (flown from `launch`); `None` without fins or samples.
    pub fn of_flight(stocks: Vec<FinStock>, launch: &Launch, flight: &FlightResult) -> Option<FlutterAnalysis> {
        if stocks.is_empty() {
            return None;
        }
        let apogee_time = flight.summary.time_to_apogee;
        let samples: Vec<(f64, f64, f64)> =
            flight.trajectory.samples.iter().take_while(|s| s.t <= apogee_time + 1e-9).map(|s| (s.t, s.altitude, s.air_speed)).collect();
        let atmosphere = Atmosphere::new(launch.site_altitude, launch.ground_temperature);
        let fins: Vec<FlutterFin> = stocks.iter().map(|s| s.fin).collect();
        let trace = FlutterTrace::along(&fins, &atmosphere, samples.iter().copied())?;
        Some(FlutterAnalysis { stocks, trace, samples, apogee: flight.summary.apogee })
    }

    /// The same flight judged for the fin sets of group `source` only (the inspector's chip of one fin group).
    pub fn for_group(&self, source: PartId) -> Option<FlutterAnalysis> {
        let stocks: Vec<FinStock> = self.stocks.iter().filter(|s| s.source == source).cloned().collect();
        let fins: Vec<FlutterFin> = stocks.iter().map(|s| s.fin).collect();
        let trace = FlutterTrace::along(&fins, &self.trace.atmosphere, self.samples.iter().copied())?;
        Some(FlutterAnalysis { stocks, trace, samples: self.samples.clone(), apogee: self.apogee })
    }

    /// The fins as the trace sees them.
    pub fn fins(&self) -> Vec<FlutterFin> {
        self.stocks.iter().map(|s| s.fin).collect()
    }

    /// The fin set that decides the margin at its worst point.
    pub fn governing(&self) -> &FinStock {
        &self.stocks[self.trace.worst().governing.min(self.stocks.len() - 1)]
    }

    /// Every input of the formula at the worst point.
    pub fn inputs(&self) -> Option<FlutterInputs> {
        self.trace.inputs_at_worst(&self.fins())
    }

    /// What makes the estimate less trustworthy.
    pub fn warnings(&self) -> Vec<FlutterWarning> {
        self.trace.warnings(&self.fins())
    }

    /// The lowest margin `V_f / V` of the flight.
    pub fn min_margin(&self) -> f64 {
        self.trace.min_margin()
    }

    /// `V_f` and `V` at time `t` (interpolated), `None` outside the flight.
    pub fn at_time(&self, t: f64) -> Option<(f64, f64)> {
        let p = &self.trace.points;
        let (first, last) = (p.first()?, p.last()?);
        if t < first.t || t > last.t {
            return None;
        }
        let i = p.partition_point(|q| q.t <= t).min(p.len() - 1).max(1);
        let (a, b) = (&p[i - 1], &p[i]);
        let f = if b.t > a.t { ((t - a.t) / (b.t - a.t)).clamp(0.0, 1.0) } else { 0.0 };
        Some((a.flutter_velocity + (b.flutter_velocity - a.flutter_velocity) * f, a.speed + (b.speed - a.speed) * f))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_motors::MotorDb;
    use bowshock_sim::from_model::Phase;

    const EXAMPLE: &[u8] = include_bytes!("../../assets/examples/hpr_l1_cert.bowshock");

    #[test]
    fn a_fin_group_with_its_own_shear_modulus_and_tip_to_tip_changes_only_its_stock() {
        let motors = MotorDb::bundled();
        let mut rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        let input = |r: &Rocket| from_model::sim_input(r, &motors, Phase::Full, bowshock_sim::Launch::default()).unwrap();
        let base = stocks(&rocket, &input(&rocket).vehicle)[0].clone();
        assert!(!base.overridden && base.g_source.contains(rocket.radials().iter().find(|r| r.kind.is_fin()).unwrap().material.material().name));
        let id = base.source;
        assert!(rocket.set_fin_tip_to_tip(id, true));
        let tip = stocks(&rocket, &input(&rocket).vehicle)[0].clone();
        assert!((tip.fin.shear_modulus / base.fin.shear_modulus - 2.0).abs() < 1e-12 && tip.tip_to_tip);
        assert!(rocket.set_fin_shear_modulus(id, Some(3.0e9)));
        let own = stocks(&rocket, &input(&rocket).vehicle)[0].clone();
        assert!((own.fin.shear_modulus - 6.0e9).abs() < 1.0 && own.overridden);
    }

    /// The freeform plywood fins of the HYPERLOC L2 `.ork` (fins `Aircraft plywood (LOC)`, 725 kg/m³, nearest to cardboard in the
    /// table; tubes and fillet cardboard): through the importer and the bridge they fly as plywood, with the tab noted, and the
    /// estimate at the sea-level start agrees with the hand calculation of Martin's formula (67.9 m/s, see the sim bridge test).
    #[test]
    fn an_imported_plywood_ttw_fin_set_flies_with_plywood_stiffness_and_the_hand_calculated_flutter_speed() {
        let xml = "<openrocket><rocket><name>t</name><motorconfiguration configid=\"c1\" default=\"true\"/><subcomponents><stage><name>s</name><subcomponents>\
             <nosecone><name>n</name><material type=\"bulk\" density=\"680.0\">Cardboard</material><length>0.3</length><thickness>0.002</thickness><shape>ogive</shape><shapeparameter>1.0</shapeparameter><aftradius>0.0508</aftradius></nosecone>\
             <bodytube><name>body</name><material type=\"bulk\" density=\"680.0\">Cardboard</material><length>0.86</length><radius>0.0508</radius><thickness>0.0013</thickness><subcomponents>\
             <innertube><name>mount</name><axialoffset method=\"bottom\">0.0</axialoffset><length>0.6</length><outerradius>0.028702</outerradius><thickness>0.0015</thickness>\
               <motormount><ignitionevent>automatic</ignitionevent><overhang>0.0</overhang><motor configid=\"c1\"><designation>H128W</designation><manufacturer>AeroTech</manufacturer><delay>6</delay></motor></motormount></innertube>\
             <freeformfinset><name>Fin set</name><fincount>3</fincount><axialoffset method=\"top\">0.52705</axialoffset>\
               <material type=\"bulk\" density=\"724.996\">Aircraft plywood (LOC)</material><thickness>0.00299974</thickness><crosssection>rounded</crosssection>\
               <tabheight>0.022098</tabheight><tablength>0.117475</tablength><tabposition relativeto=\"front\">0.200025</tabposition>\
               <filletradius>0.0</filletradius><filletmaterial type=\"bulk\" density=\"680.0\">Cardboard</filletmaterial>\
               <finpoints><point x=\"0.0\" y=\"0.0\"/><point x=\"0.23495\" y=\"0.10795\"/><point x=\"0.287655\" y=\"0.10795\"/><point x=\"0.3302\" y=\"0.0\"/></finpoints></freeformfinset>\
             </subcomponents></bodytube></subcomponents></stage></subcomponents></rocket></openrocket>";
        let rocket = bowshock_model::ork::import(xml.as_bytes()).unwrap().rocket;
        let vehicle = from_model::vehicle(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        let stock = stocks(&rocket, &vehicle)[0].clone();
        assert!(stock.g_source.contains("plywood") && stock.g_source.contains("Wood Handbook"), "{}", stock.g_source);
        assert!(stock.tabbed && !stock.overridden);
        assert!((stock.fin.shear_modulus - 0.85e9).abs() < 1.0, "{}", stock.fin.shear_modulus);
        let air = Atmosphere::standard().at_agl(0.0);
        let vf = bowshock_sim::flutter::flutter_velocity(&stock.fin, &air);
        assert!((vf - 67.92).abs() < 0.07, "{vf}");
        let trace = FlutterTrace::along(&[stock.fin], &Atmosphere::standard(), [(1.5, 136.0, 160.0)]).unwrap();
        assert!(trace.warnings(&[stock.fin]).is_empty() && trace.min_margin().is_finite());
    }
}
