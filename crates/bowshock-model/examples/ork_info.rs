//! Imports an OpenRocket design and prints what came out: `cargo run -p bowshock-model --example ork_info -- file.ork`.

use bowshock_model::ork;

fn main() {
    let path = std::env::args().nth(1).expect("usage: ork_info <file.ork>");
    let imp = ork::import_file(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let r = &imp.rocket;
    println!("{} ({} parts, {} radial, {} internal)", r.name, r.parts().len(), r.radials().len(), r.internals().len());
    let stations = r.stations();
    for (i, p) in r.parts().iter().enumerate() {
        println!("  axial {:>8.4} {:<24} {:?}", stations[i], p.display_name(), p.kind);
    }
    for id in r.ids() {
        if let Some(m) = r.component_mass(id) {
            let station = r.part_station(id).unwrap_or(0.0);
            let name = match r.kind_of(id) {
                Some(bowshock_model::PartRef::Axial(p)) => p.display_name(),
                Some(bowshock_model::PartRef::Radial(p)) => p.display_name(),
                Some(bowshock_model::PartRef::Internal(p)) => p.display_name(),
                None => String::new(),
            };
            println!("  mass id={id:<3} station={station:>8.4} m={:>8.4} cg={:>8.4} {name}", m.mass, m.cg);
        }
    }
    let dry = r.dry_mass();
    println!("dry mass {:.4} kg, cg {:.4} m", dry.mass, dry.cg);
    for c in r.configurations() {
        println!(
            "config {} {}: {:?}",
            c.id,
            r.configuration_name(c.id).unwrap_or_default(),
            c.motors.iter().map(|s| (s.mount, s.motor.as_ref().map(|m| m.designation.clone()), s.delay)).collect::<Vec<_>>()
        );
        println!("   events {:?}", c.events);
    }
    println!("stages {:?}", r.stages().iter().map(|s| (s.index, s.fore_station, s.aft_station, s.issue)).collect::<Vec<_>>());
    for s in &imp.simulations {
        println!("sim {:?}", s);
    }
    for w in &imp.warnings {
        println!("WARNING: {w}");
    }
}
