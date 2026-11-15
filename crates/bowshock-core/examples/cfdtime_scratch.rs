use bowshock_core::{AftSpec, Analysis, Atmosphere, Backend, BodySpec, FinProfile, FinSpec, FlightConditions, NoseShape, NoseSpec, Resolution};
use std::time::Instant;

fn main() {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 },
        aft: vec![AftSpec::Tube { length: 0.52 }, AftSpec::Tube { length: 0.3934 }],
    };
    let fins = FinSpec { count: 4, root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.00318, profile: FinProfile::Rounded };
    let cap: Option<usize> = std::env::var("CAP").ok().and_then(|c| c.parse().ok());
    for m in std::env::args().skip(1).filter_map(|s| s.parse::<f64>().ok()) {
        let mut a = Analysis::new(&spec)
            .unwrap()
            .fins(fins.clone())
            .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 120.0 } })
            .resolution(Resolution::Coarse)
            .backend(Backend::Auto);
        if let Some(c) = cap {
            a = a.max_iterations(c);
        }
        let t = Instant::now();
        let p = a.run(&[m]).unwrap();
        let q = &p.points[0];
        println!(
            "M {m}: {:.1} s iters {} {:?} stop {:?} cd {:.4} (fric {:.4} base {:.4} fins {:?}) warn {:?}",
            t.elapsed().as_secs_f64(),
            q.detail.iterations,
            q.quality,
            q.detail.stop_reason,
            q.drag.total,
            q.drag.friction.unwrap_or(0.0),
            q.drag.base,
            q.drag.fins.map(|f| f.friction + f.pressure),
            q.warnings
        );
    }
}
