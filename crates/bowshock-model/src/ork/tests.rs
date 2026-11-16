use roxmltree::Document;

use super::xml::fin_tab_of;
use super::{import, import_file, OrkError, OrkImport};
use crate::config::MotorRef;
use crate::geometry::{FinSection, FinTab, NoseShape, PartKind, RadialKind};
use crate::material::MaterialId;
use crate::rocket::Rocket;
use crate::PartId;

use crate::config::{EventKind, StageTrigger};
use crate::internal::{DeployEvent, InternalKind};

const MINI: &[u8] = include_bytes!("../../tests/data/mini_two_stage.ork");

fn mini() -> OrkImport {
    import(MINI).expect("the fixture imports")
}

fn part_id(r: &Rocket, name: &str) -> PartId {
    r.ids()
        .into_iter()
        .find(|&id| match r.kind_of(id) {
            Some(crate::PartRef::Axial(p)) => p.display_name() == name,
            Some(crate::PartRef::Radial(p)) => p.display_name() == name,
            Some(crate::PartRef::Internal(p)) => p.display_name() == name,
            None => false,
        })
        .unwrap_or_else(|| panic!("no part named {name}"))
}

fn close(a: f64, b: f64, rel: f64) {
    assert!((a - b).abs() <= rel * b.abs().max(1e-12), "{a} vs {b}");
}

/// A one-stage document around `body`, for tests of single features.
fn doc(body: &str) -> OrkImport {
    let xml = format!(
        "<openrocket version=\"1.8\"><rocket><name>t</name><subcomponents><stage><name>s</name><subcomponents>{body}</subcomponents></stage></subcomponents></rocket></openrocket>"
    );
    import(xml.as_bytes()).expect("a small document imports")
}

const NOSE: &str = "<nosecone><name>n</name><material type=\"bulk\" density=\"1050\">p</material><length>0.2</length><thickness>0.002</thickness><shape>ogive</shape><shapeparameter>1.0</shapeparameter><aftradius>0.025</aftradius></nosecone>";

#[test]
fn auto_radii_follow_the_neighbours_and_ignore_the_stale_number() {
    let imp = mini();
    let r = &imp.rocket;
    let d: Vec<(f64, f64)> = r.parts().iter().map(|p| (p.kind.fore_diameter(), p.kind.aft_diameter())).collect();
    // Nose: `auto` follows the tube behind it (0.027 m radius). Boattail: fore follows the tube ahead, aft is given.
    // The sustainer tube says `auto 0.1` but follows the boattail's aft radius; the booster tube follows it too.
    assert_eq!(d, vec![(0.0, 0.054), (0.054, 0.054), (0.054, 0.045), (0.045, 0.045), (0.045, 0.045)]);
    let stations = r.stations();
    close(stations[4], 0.2 + 0.4 + 0.05 + 0.25, 1e-12);
    close(r.length(), 1.4, 1e-12);
    assert_eq!(r.name, "Mini two-stage");
    assert_eq!(r.parts()[0].finish, crate::Finish::SmoothPaint);
    assert_eq!(r.parts()[4].finish, crate::Finish::Rough);
    assert!(r.parts()[0].shoulder_aft.is_some() && r.parts()[0].shoulder_aft.diameter == 0.049);
}

#[test]
fn masses_follow_openrocket_density_override_and_formulas() {
    let imp = mini();
    let r = &imp.rocket;
    // A tube of 1000 kg/m^3 stock (the table has no such entry): mass = pi (ro^2 - ri^2) L rho, within the override scaling.
    let tube = r.component_mass(part_id(r, "Upper tube")).unwrap().mass;
    close(tube, std::f64::consts::PI * (0.027f64.powi(2) - 0.025f64.powi(2)) * 0.4 * 1000.0, 5e-3);
    // A mass override replaces the computed mass of a whole fin group.
    close(r.component_mass(part_id(r, "Sustainer fins")).unwrap().mass, 0.2, 1e-12);
    // Parachute: canopy area x surface density + lines x line density (OpenRocket's formula), whatever the table says.
    close(r.component_mass(part_id(r, "Main")).unwrap().mass, std::f64::consts::FRAC_PI_4 * 0.36 * 0.067 + 6.0 * 0.5 * 0.001, 5e-3);
    close(r.component_mass(part_id(r, "Nose weight")).unwrap().mass, 0.1, 1e-12);
    close(r.component_mass(part_id(r, "Cord")).unwrap().mass, 1.0 * 0.0035, 5e-3);
    assert!(imp.warnings.iter().any(|w| w.contains("Odd cardboard")), "{:?}", imp.warnings);
}

#[test]
fn rail_button_instances_are_separate_groups_at_their_spacing() {
    let imp = mini();
    let r = &imp.rocket;
    let buttons: Vec<&crate::RadialPart> = r.radials().iter().filter(|p| p.name.as_deref() == Some("Buttons")).collect();
    assert_eq!(buttons.len(), 2);
    let (a, b) = (r.radial_station(buttons[0]).unwrap(), r.radial_station(buttons[1]).unwrap());
    close(b - a, 0.3, 1e-9);
    // The model's offset is the button's fore edge: its centre sits where OpenRocket puts the component.
    let top = r.stations()[4];
    close(a + 0.0097 / 2.0, top + 0.1, 1e-9);
    assert!((buttons[0].angle - std::f64::consts::PI).abs() < 1e-9);
    assert!(matches!(buttons[0].kind, RadialKind::RailButton { height, diameter } if height == 0.0097 && diameter == 0.0097));
}

#[test]
fn fins_carry_count_cant_cross_section_and_position() {
    let imp = mini();
    let r = &imp.rocket;
    let fins = r.radial(part_id(r, "Sustainer fins")).unwrap();
    assert_eq!(fins.count, 3);
    assert_eq!(fins.cross_section, crate::FinSection::Airfoil);
    close(fins.cant, 1.5f64.to_radians(), 1e-12);
    close(fins.angle, 10f64.to_radians(), 1e-12);
    assert_eq!(fins.kind, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.08, sweep: 0.07, thickness: 0.003 });
    // `bottom` 0: the root's trailing edge is at the aft end of the tube (stations 0.65 to 0.9).
    close(r.radial_station(fins).unwrap(), 0.9 - 0.12, 1e-9);
    let booster = r.radial(part_id(r, "Booster fins")).unwrap();
    assert_eq!(booster.count, 4);
    close(r.radial_station(booster).unwrap(), 1.4 - 0.15, 1e-9);
}

#[test]
fn internal_parts_sit_where_openrocket_puts_them() {
    let imp = mini();
    let r = &imp.rocket;
    let st = |name: &str| r.part_station(part_id(r, name)).unwrap();
    close(st("Main"), 0.2 + 0.02, 1e-12);
    // `bottom` 0.05: the coupler's aft end is 0.05 above the aft end of its tube (stations 0.2 to 0.6).
    close(st("Coupler"), 0.6 + 0.05 - 0.12, 1e-12);
    // The sustainer mount: bottom of the tube (0.65 to 0.9), 0.15 long.
    close(st("Sustainer mount"), 0.9 - 0.15, 1e-12);
    // A ring inside an inner tube has no sibling tube to follow: a solid disc, flush with the tube's bore.
    let ring = r.internal(part_id(r, "Retainer")).unwrap();
    assert!(matches!(ring.kind, InternalKind::CenteringRing { inner_diameter: Some(d), .. } if d == 0.0));
    // Auto outer diameters of a bulkhead in a coupler follow the coupler's bore.
    let bulkhead = r.internal(part_id(r, "Coupler bulkhead")).unwrap();
    assert!(matches!(bulkhead.kind, InternalKind::Bulkhead { diameter: Some(d), .. } if (d - (0.054 - 0.004 - 0.004)).abs() < 1e-9));
}

#[test]
fn configurations_carry_motors_delays_and_staging() {
    let imp = mini();
    let r = &imp.rocket;
    assert_eq!(r.configurations().len(), 2);
    assert_eq!(imp.config_ids.len(), 2);
    assert_eq!(r.active_config_id(), r.configurations()[0].id, "the default configuration is active");
    let mounts = r.motor_mounts();
    assert_eq!(mounts.len(), 2);
    let (sustainer, booster) = (mounts[0].id, mounts[1].id);
    let (c0, c1) = (&r.configurations()[0], &r.configurations()[1]);
    let slot = |c: &crate::Configuration, m: PartId| c.slot(m).cloned().unwrap();
    assert_eq!(slot(c0, sustainer).motor, Some(MotorRef::new("G64W", "AeroTech")));
    assert!(slot(c0, sustainer).is_plugged(), "delay `none` is a plugged motor");
    assert_eq!(slot(c1, sustainer).motor, Some(MotorRef::new("G80T", "AeroTech")));
    assert_eq!(slot(c1, sustainer).delay, 7.0);
    assert_eq!(slot(c0, booster).motor, Some(MotorRef::new("H128W", "AeroTech")));
    assert_eq!(imp.motors.len(), 3);
    assert_eq!((mounts[0].overhang, mounts[1].overhang), (0.0, 0.01));

    // Stage 2 (sustainer) lights 2 s after the booster burns out in configuration 0, at burnout in configuration 1.
    let ignite0 = c0.event(2, EventKind::Ignition);
    assert_eq!((ignite0.trigger, ignite0.delay), (StageTrigger::Burnout, 2.0));
    assert_eq!(c1.event(2, EventKind::Ignition).delay, 0.0);
    // Stage 1 (booster) lights at launch. Its separation: the stage's `ejection` is burnout plus the motor's delay (a plugged
    // motor adds none); configuration 1 says burnout + 1.5 s.
    assert_eq!(c0.event(1, EventKind::Ignition).trigger, StageTrigger::TimeFromLaunch);
    let sep0 = c0.event(1, EventKind::Separation);
    assert_eq!((sep0.trigger, sep0.delay), (StageTrigger::Burnout, 0.5));
    let sep1 = c1.event(1, EventKind::Separation);
    assert_eq!((sep1.trigger, sep1.delay), (StageTrigger::Burnout, 1.5));
    assert_eq!(r.stages().len(), 2);
    assert!(r.stage_issues().is_empty());
}

#[test]
fn recovery_deployment_is_per_configuration() {
    let imp = mini();
    let r = &imp.rocket;
    let chute = part_id(r, "Main");
    let (c0, c1) = (r.configurations()[0].id, r.configurations()[1].id);
    assert_eq!(r.deployment(c0, chute).map(|d| d.event), Some(DeployEvent::Ejection));
    let d = r.deployment(c1, chute).unwrap();
    assert_eq!((d.event, d.delay), (DeployEvent::Altitude(150.0), 1.0));
}

#[test]
fn simulations_become_flights() {
    let imp = mini();
    let flight = &imp.rocket.flights()[0];
    assert_eq!(flight.name, "Windy pad");
    assert_eq!(flight.motor, crate::FlightMotor::Config(imp.rocket.configurations()[1].id));
    let c = &flight.conditions;
    close(c.rail_length, 2.4384, 1e-12);
    close(c.launch_angle, 5f64.to_radians(), 1e-12);
    assert_eq!((c.wind_speed, c.turbulence, c.site_altitude), (4.0, crate::Turbulence::High, 1400.0));
    close(c.temperature, 288.15 - 0.0065 * 1400.0, 1e-9);
    let sim = &imp.simulations[0];
    assert_eq!(sim.config, Some(imp.rocket.configurations()[1].id));
    assert_eq!(sim.stored.as_ref().and_then(|s| s.max_altitude), Some(1234.5));
    close(sim.latitude, 32.9, 1e-12);
}

#[test]
fn the_imported_document_saves_and_loads() {
    let imp = mini();
    let back = Rocket::from_bytes(&imp.rocket.to_bytes()).expect("a saved import loads");
    assert_eq!(back, imp.rocket);
}

#[test]
fn unsupported_things_are_reported_not_dropped() {
    let body = "<nosecone><name>secant</name><length>0.2</length><shape>ogive</shape><shapeparameter>0.6</shapeparameter><aftradius>0.025</aftradius></nosecone>\
         <bodytube><name>t</name><length>0.5</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>\
           <innertube><name>cluster</name><length>0.1</length><outerradius>0.01</outerradius><thickness>0.001</thickness><clusterconfiguration>3-ring</clusterconfiguration></innertube>\
           <parachute><name>chute</name><packedlength>0.05</packedlength><diameter>0.3</diameter><deployevent>never</deployevent></parachute>\
           <thingamajig><name>gizmo</name></thingamajig>\
         </subcomponents></bodytube>\
         <transition><name>flare</name><length>0.05</length><shape>ogive</shape><foreradius>auto</foreradius><aftradius>0.03</aftradius></transition>".to_string();
    let imp = doc(&body);
    let has = |needle: &str| imp.warnings.iter().any(|w| w.contains(needle));
    assert!(has("secant ogive"), "{:?}", imp.warnings);
    assert!(has("cluster \"3-ring\""), "{:?}", imp.warnings);
    assert!(has("deployment \"never\""), "{:?}", imp.warnings);
    assert!(has("gizmo") && has("thingamajig"), "{:?}", imp.warnings);
    assert!(has("only conical transitions"), "{:?}", imp.warnings);
    // What could be imported still was.
    assert_eq!(imp.rocket.parts().len(), 3);
}

#[test]
fn a_stage_override_with_subcomponents_sets_the_stage_total() {
    let xml = format!(
        "<openrocket><rocket><name>t</name><subcomponents><stage><name>s</name><overridemass>1.5</overridemass><overridesubcomponentsmass>true</overridesubcomponentsmass><subcomponents>{NOSE}\
         <bodytube><name>t</name><material type=\"bulk\" density=\"1850\">g</material><length>0.5</length><radius>auto</radius><thickness>0.002</thickness>\
         <subcomponents><masscomponent><name>m</name><packedlength>0.05</packedlength><mass>0.3</mass></masscomponent></subcomponents></bodytube></subcomponents></stage></subcomponents></rocket></openrocket>"
    );
    let imp = import(xml.as_bytes()).unwrap();
    close(imp.rocket.dry_mass().mass, 1.5, 1e-9);
}

#[test]
fn a_drag_override_is_imported() {
    let xml = format!(
        "<openrocket><rocket><name>t</name><subcomponents><stage><name>s</name><subcomponents>{NOSE}\
         <bodytube><name>t</name><material type=\"bulk\" density=\"1850\">g</material><length>0.5</length><radius>auto</radius><thickness>0.002</thickness>\
         <overridecd>0.35</overridecd></bodytube></subcomponents></stage></subcomponents></rocket></openrocket>"
    );
    let imp = import(xml.as_bytes()).unwrap();
    assert_eq!(imp.rocket.parts()[1].cd_override, Some(0.35));
    assert!(!imp.warnings.iter().any(|w| w.contains("drag override")), "{:?}", imp.warnings);
}

#[test]
fn a_capped_shoulder_adds_its_disc() {
    let capped = |c: &str| {
        let nose = format!(
            "<nosecone><name>n</name><material type=\"bulk\" density=\"1050\">p</material><length>0.1</length><thickness>0.002</thickness><shape>ogive</shape><aftradius>0.0125</aftradius>\
             <aftshoulderradius>0.0115</aftshoulderradius><aftshoulderlength>0.02</aftshoulderlength><aftshoulderthickness>0.002</aftshoulderthickness><aftshouldercapped>{c}</aftshouldercapped></nosecone>"
        );
        let imp = doc(&nose);
        imp.rocket.component_mass(imp.rocket.parts()[0].id).unwrap()
    };
    let (open, closed) = (capped("false"), capped("true"));
    close(closed.mass - open.mass, std::f64::consts::PI * 0.0095f64.powi(2) * 0.002 * 1050.0, 1e-6);
    assert!(closed.cg > open.cg, "the disc is at the far end of the shoulder");
}

#[test]
fn nose_shapes_map_to_the_models_families() {
    let shape = |s: &str, p: f64| {
        let nose = format!(
            "<nosecone><name>n</name><length>0.2</length><shape>{s}</shape><shapeparameter>{p}</shapeparameter><aftradius>0.025</aftradius></nosecone>"
        );
        let imp = doc(&nose);
        match imp.rocket.parts()[0].kind {
            PartKind::NoseCone { shape, .. } => shape,
            _ => unreachable!(),
        }
    };
    assert_eq!(shape("conical", 0.0), NoseShape::Conical);
    assert_eq!(shape("ellipsoid", 0.0), NoseShape::Elliptical);
    assert_eq!(shape("power", 0.5), NoseShape::PowerSeries(0.5));
    assert_eq!(shape("haack", 0.0), NoseShape::VonKarman);
    assert_eq!(shape("haack", 1.0 / 3.0), NoseShape::Haack(1.0 / 3.0));
    assert_eq!(shape("parabolic", 1.0), NoseShape::Parabolic);
    assert!(matches!(shape("parabolic", 0.5), NoseShape::PowerSeries(n) if (0.3..1.0).contains(&n)));
}

#[test]
fn freeform_and_elliptical_fins() {
    let body = format!(
        "{NOSE}<bodytube><name>t</name><length>0.5</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>\
         <freeformfinset><name>ff</name><fincount>3</fincount><axialoffset method=\"bottom\">0.0</axialoffset><thickness>0.002</thickness>\
           <finpoints><point x=\"0.0\" y=\"0.0\"/><point x=\"0.025\" y=\"0.03\"/><point x=\"0.105\" y=\"0.04\"/><point x=\"0.08\" y=\"0.0\"/></finpoints></freeformfinset>\
         <ellipticalfinset><name>ef</name><fincount>4</fincount><axialoffset method=\"bottom\">0.0</axialoffset><thickness>0.002</thickness><rootchord>0.1</rootchord><height>0.05</height></ellipticalfinset>\
         </subcomponents></bodytube>"
    );
    let imp = doc(&body);
    let r = &imp.rocket;
    let ff = r.radial(part_id(r, "ff")).unwrap();
    assert!(matches!(&ff.kind, RadialKind::FreeformFin { points, .. } if points.len() == 4 && points[3] == (0.08, 0.0)));
    let ef = r.radial(part_id(r, "ef")).unwrap();
    assert_eq!((ef.count, &ef.kind), (4, &RadialKind::EllipticalFin { root_chord: 0.1, span: 0.05, thickness: 0.002 }));
}

#[test]
fn a_pod_set_becomes_a_pod_with_its_parts() {
    let body = format!(
        "{NOSE}<bodytube><name>t</name><length>0.5</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>\
         <podset><name>pods</name><instancecount>2</instancecount><radiusoffset method=\"relative\">0.003</radiusoffset><angleoffset method=\"relative\">90.0</angleoffset><axialoffset method=\"top\">0.1</axialoffset>\
           <subcomponents><nosecone><name>pn</name><length>0.05</length><shape>conical</shape><aftradius>0.01</aftradius></nosecone>\
           <bodytube><name>pt</name><length>0.2</length><radius>auto</radius><thickness>0.001</thickness></bodytube></subcomponents></podset>\
         </subcomponents></bodytube>"
    );
    let imp = doc(&body);
    let r = &imp.rocket;
    let pod = r.pods().next().expect("a pod set");
    assert_eq!(pod.count, 2);
    assert_eq!(pod.kind.pod_gap(), Some(0.003));
    assert_eq!(pod.kind.pod_body().map(|b| b.parts().len()), Some(2));
    close(pod.angle, std::f64::consts::FRAC_PI_2, 1e-12);
    close(pod.offset, 0.1, 1e-12);
}

#[test]
fn containers_zip_gzip_and_plain_xml_all_import() {
    use flate2::write::{DeflateEncoder, GzEncoder};
    use flate2::Compression;
    use std::io::Write;

    let plain = import(MINI).unwrap();
    let mut gz = GzEncoder::new(Vec::new(), Compression::default());
    gz.write_all(MINI).unwrap();
    assert_eq!(import(&gz.finish().unwrap()).unwrap().rocket, plain.rocket);

    for deflate in [true, false] {
        let payload = if deflate {
            let mut e = DeflateEncoder::new(Vec::new(), Compression::default());
            e.write_all(MINI).unwrap();
            e.finish().unwrap()
        } else {
            MINI.to_vec()
        };
        let name = b"rocket.ork";
        let method: u16 = if deflate { 8 } else { 0 };
        let mut zip = Vec::new();
        // A decoy entry first: the importer must pick rocket.ork.
        let mut entries: Vec<(&[u8], u16, Vec<u8>, usize)> = vec![(b"decals/readme.txt".as_slice(), 0, b"hi".to_vec(), 2)];
        entries.push((name.as_slice(), method, payload.clone(), MINI.len()));
        let mut central = Vec::new();
        for (n, m, data, size) in &entries {
            let offset = zip.len() as u32;
            for (bytes, header) in [(&mut zip, true), (&mut central, false)] {
                bytes.extend_from_slice(if header { &[0x50, 0x4b, 3, 4] } else { &[0x50, 0x4b, 1, 2] });
                if !header {
                    bytes.extend_from_slice(&[20, 0]);
                }
                bytes.extend_from_slice(&[20, 0, 0, 0]);
                bytes.extend_from_slice(&m.to_le_bytes());
                bytes.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0]); // time, date, crc (unchecked)
                bytes.extend_from_slice(&(data.len() as u32).to_le_bytes());
                bytes.extend_from_slice(&(*size as u32).to_le_bytes());
                bytes.extend_from_slice(&(n.len() as u16).to_le_bytes());
                bytes.extend_from_slice(&[0, 0]);
                if !header {
                    bytes.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]); // comment, disk, attributes
                    bytes.extend_from_slice(&offset.to_le_bytes());
                }
                bytes.extend_from_slice(n);
                if header {
                    bytes.extend_from_slice(data);
                }
            }
        }
        let start = zip.len() as u32;
        zip.extend_from_slice(&central);
        zip.extend_from_slice(&[0x50, 0x4b, 5, 6, 0, 0, 0, 0]);
        zip.extend_from_slice(&(entries.len() as u16).to_le_bytes());
        zip.extend_from_slice(&(entries.len() as u16).to_le_bytes());
        zip.extend_from_slice(&(central.len() as u32).to_le_bytes());
        zip.extend_from_slice(&start.to_le_bytes());
        zip.extend_from_slice(&[0, 0]);
        assert_eq!(import(&zip).unwrap().rocket, plain.rocket, "deflate = {deflate}");
    }
}

/// The freeform plywood fin of a 4 in L2 rocket (HYPERLOC) with a through-the-wall tab onto its 54 mm motor mount: `Aircraft plywood
/// (LOC)` is 725 kg/m^3, nearest in density to cardboard (680) in the table, but the fin is plywood and must keep a plywood's shear
/// modulus.
#[test]
fn a_ttw_plywood_fin_keeps_its_plywood_stiffness_and_its_tab() {
    let xml = format!(
        "<openrocket><rocket><name>t</name><motorconfiguration configid=\"c1\" default=\"true\"/><subcomponents><stage><name>s</name><subcomponents>{NOSE}\
         <bodytube><name>body</name><material type=\"bulk\" density=\"680.0\">Cardboard</material><length>0.86</length><radius>0.0508</radius><thickness>0.0013</thickness><subcomponents>\
         <innertube><name>mount</name><axialoffset method=\"bottom\">0.0</axialoffset><length>0.6</length><outerradius>0.028702</outerradius><thickness>0.0015</thickness>\
           <motormount><ignitionevent>automatic</ignitionevent><overhang>0.0</overhang><motor configid=\"c1\"><designation>J425R</designation><manufacturer>AeroTech</manufacturer><delay>14</delay></motor></motormount></innertube>\
         <freeformfinset><name>Fin set</name><fincount>3</fincount><axialoffset method=\"top\">0.52705</axialoffset>\
           <material type=\"bulk\" density=\"724.996\">Aircraft plywood (LOC)</material><thickness>0.00299974</thickness><crosssection>rounded</crosssection>\
           <tabheight>0.022098</tabheight><tablength>0.117475</tablength><tabposition relativeto=\"front\">0.200025</tabposition><tabposition relativeto=\"top\">0.200025</tabposition>\
           <filletradius>0.0</filletradius><filletmaterial type=\"bulk\" density=\"680.0\">Cardboard</filletmaterial>\
           <finpoints><point x=\"0.0\" y=\"0.0\"/><point x=\"0.23495\" y=\"0.10795\"/><point x=\"0.287655\" y=\"0.10795\"/><point x=\"0.3302\" y=\"0.0\"/></finpoints></freeformfinset>\
         </subcomponents></bodytube></subcomponents></stage></subcomponents></rocket></openrocket>"
    );
    let imp = import(xml.as_bytes()).unwrap();
    let r = &imp.rocket;
    let fins = r.radials().iter().find(|f| f.kind.is_fin()).unwrap();
    // Plywood, not the paperboard (0.30 GPa) the nearest density gave.
    assert!(fins.material.name().contains("plywood"), "{}", fins.material.name());
    close(fins.flutter_shear_modulus(), 0.85e9, 1e-12);
    assert!(!fins.material.material().shear_source.contains("paperboard"));
    // The tab, from the root's leading edge, and the height that reaches the mount (body radius less mount radius).
    assert_eq!(fins.tab, Some(FinTab { height: 0.022098, length: 0.117475, position: 0.200025 }));
    close(r.fin_tab_target(fins.id).unwrap().height, 0.0508 - 0.028702, 1e-9);
    assert!(r.fin_tab_issues(fins.id).is_empty());
    // OpenRocket's fin mass: plate (area x thickness x the rounded section's volume factor) plus the tab plate, at 725 kg/m^3.
    let area = 0.5 * (0.3302 + (0.287655 - 0.23495)) * 0.10795;
    let plate = area * 0.00299974 * FinSection::Rounded.volume_factor() + 0.022098 * 0.117475 * 0.00299974;
    close(r.component_mass(fins.id).unwrap().mass, 3.0 * 724.996 * plate, 1e-9);
}

#[test]
fn tab_positions_are_taken_from_the_front_the_centre_or_the_end_of_the_root() {
    let fin = |position: &str| {
        let xml = format!("<f><tabheight>0.02</tabheight><tablength>0.1</tablength>{position}</f>");
        let doc = Document::parse(&xml).unwrap();
        fin_tab_of(doc.root_element(), 0.3)
    };
    assert_eq!(fin("<tabposition relativeto=\"front\">0.05</tabposition>").unwrap().position, 0.05);
    // Centred and shifted 10 mm aft: (300 - 100)/2 + 10 from the front.
    close(fin("<tabposition relativeto=\"center\">0.01</tabposition>").unwrap().position, 0.11, 1e-12);
    // 20 mm ahead of the trailing edge: 300 - 100 - 20.
    close(fin("<tabposition relativeto=\"end\">0.02</tabposition>").unwrap().position, 0.18, 1e-12);
    assert!(fin("").is_some());
    let none = Document::parse("<f><tabheight>0.0</tabheight><tablength>0.1</tablength></f>").unwrap();
    assert!(fin_tab_of(none.root_element(), 0.3).is_none());
}

#[test]
fn an_unrecognised_fin_material_is_reported_with_the_shear_modulus_it_got() {
    let xml = |name: &str| {
        format!(
            "<openrocket><rocket><name>t</name><subcomponents><stage><name>s</name><subcomponents>{NOSE}\
             <bodytube><name>t</name><length>0.5</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>\
             <trapezoidfinset><name>f</name><fincount>3</fincount><axialoffset method=\"bottom\">0.0</axialoffset><material type=\"bulk\" density=\"1210.0\">{name}</material>\
             <thickness>0.002</thickness><rootchord>0.1</rootchord><tipchord>0.05</tipchord><height>0.05</height><sweeplength>0.03</sweeplength></trapezoidfinset>\
             </subcomponents></bodytube></subcomponents></stage></subcomponents></rocket></openrocket>"
        )
    };
    let known = import(xml("Fiberglass (G10)").as_bytes()).unwrap();
    assert_eq!(known.rocket.radials()[0].material, MaterialId::FIBERGLASS);
    assert!(!known.warnings.iter().any(|w| w.contains("shear modulus is unknown")));
    let unknown = import(xml("Unobtainium").as_bytes()).unwrap();
    assert!(
        unknown.warnings.iter().any(|w| w.contains("Unobtainium") && w.contains("shear modulus is unknown") && w.contains("GPa")),
        "{:?}",
        unknown.warnings
    );
    assert!(unknown.rocket.radials()[0].flutter_shear_modulus() > 0.0);
}

#[test]
fn a_motor_reaching_past_a_stage_joint_is_tucked_in() {
    // An upper-stage motor that sticks 3 mm out of its airframe into the next stage (OpenRocket's default overhang).
    let mount = |overhang: f64, config: &str, designation: &str| {
        format!(
            "<innertube><name>mount</name><axialoffset method=\"bottom\">0.0</axialoffset><length>0.1</length><outerradius>0.015</outerradius><thickness>0.001</thickness>\
             <motormount><ignitionevent>automatic</ignitionevent><overhang>{overhang}</overhang><motor configid=\"{config}\"><designation>{designation}</designation><manufacturer>X</manufacturer><delay>3</delay></motor></motormount></innertube>"
        )
    };
    let xml = format!(
        "<openrocket><rocket><name>t</name><motorconfiguration configid=\"c1\" default=\"true\"/><subcomponents>\
         <stage><name>upper</name><subcomponents>{NOSE}<bodytube><name>a</name><length>0.3</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>{}</subcomponents></bodytube></subcomponents></stage>\
         <stage><name>lower</name><subcomponents><bodytube><name>b</name><length>0.3</length><radius>auto</radius><thickness>0.001</thickness><subcomponents>{}</subcomponents></bodytube></subcomponents></stage>\
         </subcomponents></rocket></openrocket>",
        mount(0.003, "c1", "U100"),
        mount(0.0, "c1", "L100"),
    );
    let imp = import(xml.as_bytes()).unwrap();
    assert!(imp.rocket.stage_issues().is_empty(), "{:?}", imp.rocket.stage_issues());
    assert_eq!(imp.rocket.stages().len(), 2);
    let upper = imp.rocket.motor_mounts()[0];
    close(upper.motor_aft_station, imp.rocket.stations()[2], 1e-9);
    assert!(imp.warnings.iter().any(|w| w.contains("3.0 mm past a stage joint")), "{:?}", imp.warnings);
}

#[test]
fn bad_input_is_an_error() {
    assert!(matches!(import(b"<html/>"), Err(OrkError::NotOpenRocket)));
    assert!(matches!(import(b"<openrocket><oops></openrocket>"), Err(OrkError::Xml(_))));
    assert!(matches!(import(b"<openrocket/>"), Err(OrkError::NotOpenRocket)));
    assert!(matches!(import(b"PK\x03\x04 truncated"), Err(OrkError::Archive(_))));
    assert!(import_file("/no/such/file.ork").is_err());
}

#[test]
fn a_body_tube_can_be_its_own_motor_mount() {
    // A 24.1 mm bore body tube holding a D12 directly, 6.35 mm overhang, as in the *Tube fin rocket* example.
    let tube = |mount: &str| {
        format!(
            "<openrocket><rocket><name>t</name><motorconfiguration configid=\"c1\" default=\"true\"/><subcomponents><stage><name>s</name><subcomponents>{NOSE}\
             <bodytube><name>body</name><material type=\"bulk\" density=\"894.4\">paper</material><length>0.4572</length><radius>0.0124</radius><thickness>0.00033</thickness>{mount}</bodytube>\
             </subcomponents></stage></subcomponents></rocket></openrocket>"
        )
    };
    let mount = "<motormount><ignitionevent>automatic</ignitionevent><ignitiondelay>0.0</ignitiondelay><overhang>0.00635</overhang>\
                 <motor configid=\"c1\"><type>single</type><manufacturer>Estes</manufacturer><designation>D12</designation><delay>7.0</delay></motor></motormount>";
    let plain = import(tube("").as_bytes()).unwrap();
    let imp = import(tube(mount).as_bytes()).unwrap();
    let r = &imp.rocket;
    let mounts = r.motor_mounts();
    assert_eq!(mounts.len(), 1);
    let m = mounts[0];
    close(m.inner_diameter, 2.0 * (0.0124 - 0.00033), 1e-9);
    close(m.length, 0.4572, 1e-12);
    close(m.fore_station, 0.2, 1e-12);
    close(m.motor_aft_station, 0.2 + 0.4572 + 0.00635, 1e-12);
    let slot = r.active_configuration().slot(m.id).cloned().unwrap();
    assert_eq!(slot.motor, Some(MotorRef::new("D12", "Estes")));
    assert_eq!(slot.delay, 7.0);
    assert_eq!(imp.motors, vec![MotorRef::new("D12", "Estes")]);
    // The stand-in tube adds no mass: the body tube already carries the wall.
    close(r.dry_mass().mass, plain.rocket.dry_mass().mass, 1e-12);
    close(r.dry_mass().cg, plain.rocket.dry_mass().cg, 1e-12);
    assert!(imp.warnings.iter().any(|w| w.contains("body: the body tube is its own motor mount")), "{:?}", imp.warnings);
}
