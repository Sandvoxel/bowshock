//! Freezes the validated axisymmetric (2D) results into `crates/bowshock-cfd/fixtures/ref2d.json`, the reference the 3D one-cell wedge has
//! to reproduce before the 2D solver is deleted (DESIGN3D sections F3 and G, stage S0). The same file is the timing baseline of the 2D
//! solver: every case carries its wall time.
//!
//! One process per group (the NVIDIA exit hang of ISSUES #1 can kill a long process; every group writes its file before it exits):
//!
//! ```text
//! cargo run --release -p bowshock-validate --example ref2d -- --list
//! cargo run --release -p bowshock-validate --example ref2d -- --group g3 --out out/ref2d/g3.json
//! cargo run --release -p bowshock-validate --example ref2d -- --merge out/ref2d --out crates/bowshock-cfd/fixtures/ref2d.json
//! ```
//!
//! The GPU groups run through `tools/gpuq/gpuq submit --host local` (see `perf/ref2d-jobs.sh`). The git sha of the code that produced a
//! group comes from `REF2D_SHA` (the `gpuq` snapshot has no `.git`), else from `git rev-parse HEAD`.
//!
//! Facade cases (`kind: "facade"`) go through `bowshock_core::Analysis` exactly as the app and the validation tests do: one `Analysis`
//! per Mach number, cold start. Their `numbers` are in nose-base-area drag coefficients of the nose diameter (`Analysis` reference
//! diameter = nose diameter). `cd_solver_total` is what the solver itself computes (nose + aft pressure + friction + solved base) with
//! no empirical fin or build-up term and no modelled base: it is the number a finless 3D wedge on the base-wake grid must reproduce.
//! Plate and cone cases (`kind: "plate" | "cone" | "ogive"`) drive the 2D `Solver` directly, as the unit tests of `bowshock-cfd` and
//! the bins of `bowshock-validate` do.

use bowshock_cases::cone::{run_cone, ConeCase};
use bowshock_cases::ogive::{perkins_jorgensen as pj, run_ogive, wall_cp_at, OgiveCase};
use bowshock_cases::subsonic::subsonic_case;
use bowshock_cases::transonic::transonic_case;
use bowshock_cfd::gas::freestream;
use bowshock_cfd::grid::{plate_grid, PlateGridParams};
use bowshock_cfd::implicit_params::ImplicitParams;
use bowshock_cfd::limiter::Limiter;
use bowshock_cfd::solver::{Bc, Config, Scheme, Solver};
use bowshock_cfd::sst::{aupoix_wall, BETA1};
use bowshock_cfd::sst::{Turbulence, Viscous};
use bowshock_cfd::viscous::temperature;
use bowshock_core::engine::body_grid::{Body, Nose};
use bowshock_core::engine::case::Case;
use bowshock_core::engine::run::run_cold;
use bowshock_core::{
    AftSpec, Analysis, Atmosphere, Backend, BackendUsed, BodySpec, FinProfile, FinSpec, FlightConditions, MotorSpec, NoseShape, NoseSpec, Resolution, Wake,
};
use bowshock_reference::taylor_maccoll::solve_cone;
use bowshock_reference::{arcas, tp2005};
use serde_json::{json, Map, Value};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Instant;

const R_AIR: f64 = 287.052_87;
/// Schema version of the fixture; bump when a key changes meaning.
const SCHEMA: u64 = 1;
const GROUPS: [&str; 10] = ["g3", "table-rans", "table-euler", "euler-base", "arcas", "tp2005", "plates", "cones", "ogive", "w4-tables"];

// ---------------------------------------------------------------------------------------------------------------------------------
// The example rocket of the app (`assets/examples/hpr_l1_cert.bowshock`) as `analyze/cfd/plan.rs::plan` maps it (Debug of `Plan::body`,
// `Plan::fins`, `Plan::body_roughness`, read 2026-11-17 from a throwaway test in the app crate; the app crate is not a dependency here).

fn nose_json(n: &NoseSpec) -> Value {
    let shape = match n.shape {
        NoseShape::Cone => json!({"Cone": {}}),
        NoseShape::TangentOgive => json!({"TangentOgive": {}}),
        NoseShape::Power { n } => json!({"Power": {"n": n}}),
        NoseShape::Parabolic { k } => json!({"Parabolic": {"k": k}}),
        NoseShape::Haack { c } => json!({"Haack": {"c": c}}),
        other => json!({ "Other": format!("{other:?}") }),
    };
    json!({"shape": shape, "length": n.length, "diameter": n.diameter})
}

fn body_json(b: &BodySpec) -> Value {
    let aft: Vec<Value> = b
        .aft
        .iter()
        .map(|a| match a {
            AftSpec::Tube { length } => json!({"Tube": {"length": length}}),
            AftSpec::Transition { length, aft_diameter } => json!({"Transition": {"length": length, "aft_diameter": aft_diameter}}),
        })
        .collect();
    json!({"nose": nose_json(&b.nose), "aft": aft})
}

fn fins_json(f: &FinSpec) -> Value {
    json!({
        "count": f.count, "root_chord": f.root_chord, "tip_chord": f.tip_chord, "span": f.span, "sweep": f.sweep, "thickness": f.thickness,
        "roughness": f.roughness, "root_te_to_base": f.root_te_to_base, "profile": format!("{:?}", f.profile),
    })
}

fn atmosphere_json(a: &Atmosphere) -> Value {
    match *a {
        Atmosphere::Isa { altitude_m } => json!({"isa_altitude_m": altitude_m}),
        Atmosphere::Explicit { temperature_k, pressure_pa } => json!({"temperature_k": temperature_k, "pressure_pa": pressure_pa}),
    }
}

fn motor_json(m: &MotorSpec) -> Value {
    json!({"exit_diameter": m.exit_diameter, "exit_mach": m.exit_mach, "exit_pressure_pa": m.exit_pressure_pa, "total_temperature_k": m.total_temperature_k})
}

fn tube_body(nose: f64, d: f64, tube: f64) -> BodySpec {
    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: nose, diameter: d }, aft: vec![AftSpec::Tube { length: tube }] }
}

/// G3 body: tangent ogive 0.30 x 0.10 m and a 0.60 m tube (`tests/viscous.rs::g3_spec`).
fn g3_body() -> BodySpec {
    tube_body(0.30, 0.10, 0.60)
}

/// `tests/viscous.rs::spec`: G3 body + 0.10 m transition to 80 mm.
fn transition_body() -> BodySpec {
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    }
}

fn arcas_body() -> BodySpec {
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: arcas::NOSE_LENGTH, diameter: arcas::BODY_DIAMETER },
        aft: vec![AftSpec::Tube { length: arcas::TUBE_LENGTH }, AftSpec::Transition { length: arcas::BOATTAIL_LENGTH, aft_diameter: arcas::BASE_DIAMETER }],
    }
}

/// `tests/long_body.rs::two_stage(46.0)`: 38.1 mm sustainer on a 54 mm booster, 46 diameters.
fn two_stage_body() -> BodySpec {
    const D: f64 = 0.0381;
    let (nose, tube, transition, total) = (5.0, 14.0, 1.3, 46.0);
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: nose * D, diameter: D },
        aft: vec![
            AftSpec::Tube { length: tube * D },
            AftSpec::Transition { length: transition * D, aft_diameter: 0.054 },
            AftSpec::Tube { length: (total - nose - tube - transition) * D },
        ],
    }
}

/// Tunnel flow at `mach` from the stagnation temperature and the Reynolds number per metre (`tests/validation.rs::tunnel`).
fn tunnel(mach: f64, t0: f64, re_per_m: f64) -> Atmosphere {
    let t = t0 / (1.0 + 0.2 * mach * mach);
    let a = (1.4 * R_AIR * t).sqrt();
    let mu = 1.716e-5 * (t / 273.15).powf(1.5) * (273.15 + 110.4) / (t + 110.4);
    let rho = re_per_m * mu / (mach * a);
    Atmosphere::Explicit { temperature_k: t, pressure_pa: rho * R_AIR * t }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Facade cases

struct Facade {
    id: String,
    group: &'static str,
    gates: &'static [&'static str],
    body: BodySpec,
    atmosphere: Option<Atmosphere>,
    wake: Option<Wake>,
    res: Resolution,
    mach: f64,
    roughness: f64,
    tolerance: Option<f64>,
    motor: Option<MotorSpec>,
    fins: Option<FinSpec>,
    backend: Backend,
    keep_wall: bool,
    /// Experimental or analytic data the case is compared with elsewhere (kept next to the inputs for the reader).
    data: Value,
}

fn round6(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}

fn finite(x: f64) -> Value {
    if x.is_finite() {
        json!(x)
    } else {
        Value::Null
    }
}

fn wake_name(w: Option<Wake>, rans: bool) -> &'static str {
    match w.unwrap_or(if rans { Wake::Base } else { Wake::Sting }) {
        Wake::Base => "base",
        Wake::Sting => "sting",
    }
}

fn resolution_name(r: Resolution) -> String {
    format!("{r:?}").to_lowercase()
}

impl Facade {
    fn rans(&self) -> bool {
        self.atmosphere.is_some()
    }

    fn inputs(&self) -> Value {
        json!({
            "kind": "facade",
            "equations": if self.rans() { "rans" } else { "euler" },
            "body": body_json(&self.body),
            "atmosphere": self.atmosphere.as_ref().map(atmosphere_json),
            "wake": wake_name(self.wake, self.rans()),
            "resolution": resolution_name(self.res),
            "mach": self.mach,
            "roughness_m": self.roughness,
            "drag_tolerance": self.tolerance,
            "motor": self.motor.as_ref().map(motor_json),
            "fins": self.fins.as_ref().map(fins_json),
            "grid_mach": null,
        })
    }

    /// Runs the case; the error text of a failed run goes into the record, it does not stop the generator.
    fn run(&self) -> Result<Value, String> {
        let mut a = Analysis::new(&self.body).map_err(|e| e.to_string())?.resolution(self.res).backend(self.backend);
        if let Some(atmosphere) = self.atmosphere {
            a = a.viscous(FlightConditions { atmosphere });
        }
        if let Some(w) = self.wake {
            a = a.wake(w);
        }
        if self.roughness > 0.0 {
            a = a.roughness(self.roughness);
        }
        if let Some(t) = self.tolerance {
            a = a.drag_tolerance(t);
        }
        if let Some(m) = self.motor {
            a = a.motor(m);
        }
        if let Some(f) = &self.fins {
            a = a.fins(f.clone());
        }
        let t0 = Instant::now();
        let polar = a.run(&[self.mach]).map_err(|e| e.to_string())?;
        let wall_s = t0.elapsed().as_secs_f64();
        let want_gpu = self.backend == Backend::Gpu;
        if want_gpu && polar.backend != BackendUsed::Gpu {
            return Err(format!("GPU requested, ran on {:?}: {:?}", polar.backend, polar.fallback));
        }
        let p = &polar.points[0];
        let (d, r) = (&p.drag, &p.detail);
        let cd_aft: f64 = r.cd_aft.iter().sum();
        let cd_solver_total = r.cd_nose + cd_aft + r.cd_friction + if r.cd_base_solver.is_finite() { r.cd_base_solver } else { 0.0 };
        let mut numbers = Map::new();
        numbers.insert("cd_total_facade".into(), json!(d.total));
        numbers.insert("cd_solver_total".into(), json!(cd_solver_total));
        numbers.insert("cd_pressure_forebody".into(), json!(d.forebody_pressure));
        numbers.insert("cd_nose".into(), json!(r.cd_nose));
        numbers.insert("cd_aft".into(), json!(r.cd_aft));
        numbers.insert("cd_friction".into(), if self.rans() { json!(r.cd_friction) } else { Value::Null });
        numbers.insert("cd_base_solver".into(), finite(r.cd_base_solver));
        numbers.insert("cd_base_facade".into(), json!(d.base));
        numbers.insert("base_modelled".into(), json!(d.base_modelled));
        numbers.insert("cp_base".into(), finite(r.cp_base));
        numbers.insert("y_plus_max".into(), finite(r.y_plus_max));
        numbers.insert("fins_empirical".into(), d.fins.map_or(Value::Null, |f| json!({"friction": f.friction, "pressure": f.pressure})));
        numbers.insert("iterations".into(), json!(r.iterations));
        numbers.insert("residual_drop_orders".into(), finite(r.residual_drop_orders));
        numbers.insert("stop_reason".into(), json!(format!("{:?}", r.stop_reason)));
        numbers.insert("stop_reason_text".into(), json!(r.stop_reason.to_string()));
        numbers.insert("quality".into(), json!(format!("{:?}", p.quality)));
        numbers.insert("warnings".into(), json!(p.warnings.iter().map(|w| format!("{w:?}")).collect::<Vec<_>>()));
        numbers.insert("farfield_dp".into(), finite(r.farfield_dp));
        if self.keep_wall {
            let wall: Vec<Value> = r.wall.iter().map(|w| json!([round6(w.x), round6(w.r), round6(w.cp)])).collect();
            numbers.insert("wall_x_r_cp".into(), Value::Array(wall));
        }
        let gpu = bowshock_core::gpu_name();
        Ok(json!({
            "id": self.id,
            "group": self.group,
            "gates": self.gates,
            "inputs": self.inputs(),
            "data": self.data,
            "backend": format!("{:?}", polar.backend),
            "precision": match polar.backend { BackendUsed::Cpu => "f64", BackendUsed::Gpu => "df32" },
            "gpu": if polar.backend == BackendUsed::Gpu { json!(gpu) } else { Value::Null },
            "numbers": Value::Object(numbers),
            "wall_s": wall_s,
        }))
    }
}

fn error_record(id: &str, group: &str, inputs: Value, err: &str) -> Value {
    json!({"id": id, "group": group, "inputs": inputs, "error": err})
}

fn run_facades(cases: Vec<Facade>, errors: &mut usize) -> Vec<Value> {
    let mut out = Vec::new();
    for c in cases {
        println!("ref2d: {} ...", c.id);
        match c.run() {
            Ok(v) => {
                println!(
                    "ref2d: {}  backend {}  cd_solver_total {:.5}  iters {}  stop {}  wall {:.1} s",
                    c.id,
                    v["backend"],
                    v["numbers"]["cd_solver_total"].as_f64().unwrap_or(f64::NAN),
                    v["numbers"]["iterations"],
                    v["numbers"]["stop_reason"],
                    v["wall_s"].as_f64().unwrap_or(f64::NAN)
                );
                out.push(v);
            }
            Err(e) => {
                println!("ref2d: {} FAILED: {e}", c.id);
                *errors += 1;
                out.push(error_record(&c.id, c.group, c.inputs(), &e));
            }
        }
    }
    out
}

fn isa(h: f64) -> Atmosphere {
    Atmosphere::Isa { altitude_m: h }
}

fn facade(id: String, group: &'static str, gates: &'static [&'static str], body: BodySpec, mach: f64, backend: Backend) -> Facade {
    Facade {
        id,
        group,
        gates,
        body,
        atmosphere: None,
        wake: None,
        res: Resolution::Coarse,
        mach,
        roughness: 0.0,
        tolerance: None,
        motor: None,
        fins: None,
        backend,
        keep_wall: false,
        data: Value::Null,
    }
}

fn g3_cases() -> Vec<Facade> {
    [(Backend::Cpu, "cpu"), (Backend::Gpu, "gpu")]
        .into_iter()
        .map(|(backend, tag)| Facade {
            atmosphere: Some(isa(0.0)),
            keep_wall: true,
            data: json!({"README_G3": {"cd_total": 0.21300, "friction": 0.09003, "base": 0.12257, "cp_base": -0.1226, "y_plus_max": 0.648}}),
            ..facade(format!("g3-{tag}"), "g3", &["G-V3"], g3_body(), 0.5, backend)
        })
        .collect()
}

/// The 10 Mach numbers of the app's drag table over [0.1, 3.0] (`analyze/cfd/plan.rs::nodes`).
const TABLE_NODES: [f64; 10] = [0.1, 0.5, 0.8, 0.9, 1.0, 1.1, 1.3, 1.7, 2.3, 3.0];
/// Altitude of the app's table solve (m).
const TABLE_ALTITUDE_M: f64 = 120.0;
/// `DRAG_TOLERANCE` of `analyze/cfd/plan.rs`.
const TABLE_DRAG_TOLERANCE: f64 = 0.002;

/// Body, fin set and roughness of the example rocket as the app's plan maps them.
fn example_plan() -> (BodySpec, FinSpec, f64) {
    let body = tube_body(0.2286, 0.0762, 0.9134);
    let fins = FinSpec {
        count: 3,
        root_chord: 0.12,
        tip_chord: 0.05,
        span: 0.0762,
        sweep: 0.06,
        thickness: 0.00318,
        roughness: 6e-5,
        root_te_to_base: 0.0,
        profile: FinProfile::Square,
    };
    (body, fins, 6e-5)
}

fn table_cases(rans: bool) -> Vec<Facade> {
    let (body, fins, roughness) = example_plan();
    TABLE_NODES
        .iter()
        .map(|&m| {
            let id = format!("table-{}-M{m:.2}", if rans { "rans" } else { "euler" });
            if rans {
                Facade {
                    atmosphere: Some(isa(TABLE_ALTITUDE_M)),
                    roughness,
                    tolerance: Some(TABLE_DRAG_TOLERANCE),
                    fins: Some(fins.clone()),
                    keep_wall: true,
                    ..facade(id, "table-rans", &["G-V4"], body.clone(), m, Backend::Gpu)
                }
            } else {
                Facade { wake: Some(Wake::Base), keep_wall: true, ..facade(id, "table-euler", &["G-V4", "G-W1"], body.clone(), m, Backend::Gpu) }
            }
        })
        .collect()
}

fn euler_base_cases() -> Vec<Facade> {
    let bodies: [(&str, BodySpec); 3] = [("g3-transition", transition_body()), ("arcas", arcas_body()), ("two-stage-46d", two_stage_body())];
    let mut out = Vec::new();
    for (name, body) in bodies {
        for m in [0.5, 0.9, 1.3, 2.0] {
            out.push(Facade {
                wake: Some(Wake::Base),
                keep_wall: true,
                ..facade(format!("euler-base-{name}-M{m:.2}"), "euler-base", &["G-W1"], body.clone(), m, Backend::Gpu)
            });
        }
    }
    out
}

fn arcas_cases() -> Vec<Facade> {
    let mut out = Vec::new();
    let rows = arcas::TRANSONIC
        .iter()
        .map(|&(m, off, _)| (m, off, "D4013", arcas::TRANSONIC_T0_K, arcas::TRANSONIC_RE_PER_M))
        .chain(arcas::SUPERSONIC.iter().map(|&(m, off, _)| (m, off, "D4014", arcas::SUPERSONIC_T0_K, arcas::SUPERSONIC_RE_PER_M)));
    for (m, data_off, report, t0, re) in rows {
        out.push(Facade {
            atmosphere: Some(tunnel(m, t0, re)),
            res: Resolution::Medium,
            data: json!({"report": report, "ca_fins_off": data_off, "t0_k": t0, "re_per_m": re}),
            ..facade(format!("arcas-off-M{m:.2}"), "arcas", &["G-V4"], arcas_body(), m, Backend::Gpu)
        });
    }
    out
}

fn tp2005_cases() -> Vec<Facade> {
    let body = tube_body(tp2005::NOSE_LENGTH, tp2005::BODY_DIAMETER, tp2005::BASE_STATION - tp2005::NOSE_LENGTH);
    tp2005::TABLE_II
        .iter()
        .map(|&row| {
            let (minf, me, _, ptj_pinf) = row;
            let atmosphere = tunnel(minf, tp2005::T0_K, tp2005::RE_PER_M);
            let p_inf = atmosphere.state().expect("tunnel state").pressure_pa;
            let motor = MotorSpec {
                exit_diameter: tp2005::NOZZLE_EXIT_DIAMETER,
                exit_mach: me,
                exit_pressure_pa: ptj_pinf / (1.0 + 0.2 * me * me).powf(3.5) * p_inf,
                total_temperature_k: tp2005::JET_TOTAL_TEMPERATURE_K,
            };
            Facade {
                atmosphere: Some(atmosphere),
                res: Resolution::Medium,
                motor: Some(motor),
                data: json!({"pb_over_pinf_data": tp2005::base_pressure_ratio(row), "ptj_over_pinf": ptj_pinf, "gate_band_abs": 0.15}),
                ..facade(format!("tp2005-M{minf:.2}-Me{me:.2}"), "tp2005", &["G-V4"], body.clone(), minf, Backend::Gpu)
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Plates (G1, G2, G2r): the 2D `Solver` in the annulus formulation, copied from the tests of `bowshock-cfd` (`viscous.rs`, `turbulence.rs`)

struct Plate {
    solver: Solver,
    visc: Viscous,
    mach: f64,
}

fn plate(laminar: bool, ks: f64) -> Plate {
    let mach = 0.2;
    let (grid, re_c, turbulence, wall_i_start) = if laminar {
        (
            plate_grid(&PlateGridParams { r0: 20.0, x_up: 0.5, length: 1.0, height: 0.6, n_up: 20, n_plate: 80, nj: 64, dx_le: 0.005, dr_wall: 2e-4 }),
            1e6,
            Turbulence::Laminar,
            20,
        )
    } else {
        (
            plate_grid(&PlateGridParams { r0: 20.0, x_up: 0.33, length: 2.0, height: 1.0, n_up: 24, n_plate: 112, nj: 72, dx_le: 0.002, dr_wall: 4e-6 }),
            2.5e7,
            Turbulence::Sst,
            24,
        )
    };
    let cfg = Config {
        cfl: 0.5,
        limiter: Limiter::VanAlbada,
        freestream: freestream(mach),
        bc: [Bc::Characteristic, Bc::PressureOutflow, Bc::SlipWall, Bc::Characteristic],
        first_order_iters: 50,
    };
    let mut solver = Solver::new(grid, cfg);
    solver.low_mach = true;
    solver.set_scheme(Scheme::Implicit(ImplicitParams::VISCOUS));
    let visc = Viscous { re_c, sutherland: 0.368, turbulence, wall_i_start, roughness: ks };
    solver.set_viscous(Some(visc));
    Plate { solver, visc, mach }
}

impl Plate {
    /// Wall face (j = 0, i >= i0) nearest to `x_target`: `(x of the face, local skin friction cf = (mu + mu_t,w) u_t / d_n / (0.5 M^2))`
    /// (`bowshock-cfd::viscous::tests::cf_at`).
    fn cf_at(&self, x_target: f64) -> (f64, f64) {
        let (s, v, mach) = (&self.solver, &self.visc, self.mach);
        let g = &s.grid;
        let i = (v.wall_i_start..g.ni).min_by(|&a, &b| face_dx(g, a, x_target).total_cmp(&face_dx(g, b, x_target))).expect("plate faces");
        let f = &g.jface[i];
        let (xf, rf) = (0.5 * (g.node(i, 0).0 + g.node(i + 1, 0).0), 0.5 * (g.node(i, 0).1 + g.node(i + 1, 0).1));
        let w = s.prim(i, 0);
        let (tx, tr) = (f.nr, -f.nx);
        let ut = w[1] * tx + w[2] * tr;
        let dn = ((g.xc[i] - xf) * f.nx + (g.rc[i] - rf) * f.nr).abs();
        let mu = v.mu(temperature(&w));
        let mu_tw = if v.rough() { w[0] * aupoix_wall(mu / w[0], w[1].hypot(w[2]), dn, v.roughness)[2] } else { 0.0 };
        (xf, (mu + mu_tw) * ut / dn / (0.5 * mach * mach))
    }

    /// First-cell `y+` and wall-unit quantities at the cell column nearest to `x_target`: `(y+, ks+, k_w+, omega_w+, nu_t,w/nu, max mu_t/mu_inf)`.
    fn wall_units(&self, x_target: f64, cf: f64) -> (f64, f64, f64, f64, f64, f64) {
        let (s, v, mach) = (&self.solver, &self.visc, self.mach);
        let g = &s.grid;
        let i = (v.wall_i_start..g.ni).min_by(|&a, &b| (g.xc[a] - x_target).abs().total_cmp(&(g.xc[b] - x_target).abs())).expect("plate cells");
        let f = &g.jface[i];
        let (xf, rf) = (0.5 * (g.node(i, 0).0 + g.node(i + 1, 0).0), 0.5 * (g.node(i, 0).1 + g.node(i + 1, 0).1));
        let dn = ((g.xc[i] - xf) * f.nx + (g.rc[i] - rf) * f.nr).abs();
        let w = s.prim(i, 0);
        let nu = v.mu(temperature(&w)) / w[0];
        let u2 = 0.5 * mach * mach * cf / w[0];
        let (ks_plus, y_plus) = (u2.sqrt() * v.roughness / nu, dn * u2.sqrt() / nu);
        let [k_w, om_w, nut] = if v.roughness > 0.0 { aupoix_wall(nu, w[1].hypot(w[2]), dn, v.roughness) } else { [0.0, 60.0 * nu / (BETA1 * dn * dn), 0.0] };
        let mu_t_max = if v.turbulence == Turbulence::Sst { (0..g.nj).map(|j| s.mu_t[j * g.ni + i]).fold(0.0f64, f64::max) * v.re_c } else { 0.0 };
        (y_plus, ks_plus, k_w / u2, om_w * nu / u2, nut / nu, mu_t_max)
    }
}

fn face_dx(g: &bowshock_cfd::grid::Grid, i: usize, x_target: f64) -> f64 {
    (0.5 * (g.node(i, 0).0 + g.node(i + 1, 0).0) - x_target).abs()
}

/// Iterates a plate as the unit tests do: until the density residual dropped `orders` from its peak (after iteration 20) or `max_iter`
/// (G1 also stops when both checked cf values changed by < 1e-5 in 250 iterations).
fn iterate_plate(p: &mut Plate, orders: f64, max_iter: usize, g1_stations: Option<[f64; 2]>) -> (usize, f64, f64, bool) {
    let (mut peak, mut last, mut iters, mut converged) = (0.0f64, f64::INFINITY, 0, false);
    let mut cf_prev = [f64::NAN; 2];
    for it in 0..max_iter {
        let r = p.solver.step();
        iters = it + 1;
        last = r;
        if it > 20 {
            peak = peak.max(r);
            if peak / r >= 10f64.powf(orders) {
                converged = true;
                break;
            }
        }
        if let Some(st) = g1_stations {
            if it % 250 == 249 {
                let cf = [p.cf_at(st[0]).1, p.cf_at(st[1]).1];
                let settled = cf.iter().zip(&cf_prev).all(|(c, q)| (c / q - 1.0).abs() < 1e-5);
                cf_prev = cf;
                if settled {
                    converged = true;
                    break;
                }
            }
        }
    }
    (iters, peak, last, converged)
}

fn plate_record(id: &str, gates: &[&str], inputs: Value, iters: (usize, f64, f64, bool), stations: Vec<Value>, wall_s: f64) -> Value {
    let (n, peak, last, converged) = iters;
    json!({
        "id": id, "group": "plates", "gates": gates, "inputs": inputs,
        "backend": "Cpu", "precision": "f64", "gpu": null,
        "numbers": {"iterations": n, "density_residual_peak": peak, "density_residual_last": last, "drop_orders": (peak / last).log10(),
                    "converged": converged, "stations": stations},
        "wall_s": wall_s,
    })
}

fn plate_cases() -> Vec<Value> {
    let mut out = Vec::new();
    // G1: laminar flat plate, Blasius, Re_x = re_c M x
    {
        println!("ref2d: plate-g1 ...");
        let t0 = Instant::now();
        let mut p = plate(true, 0.0);
        let it = iterate_plate(&mut p, 5.0, 6000, Some([0.5, 0.9]));
        let stations: Vec<Value> = [0.5, 0.9]
            .iter()
            .map(|&xt| {
                let (x, cf) = p.cf_at(xt);
                let reference = 0.664 / (p.visc.re_c * p.mach * x).sqrt();
                json!({"x": x, "cf": cf, "reference": reference, "reference_name": "Blasius 0.664/sqrt(Re_x)", "error": cf / reference - 1.0, "gated": true})
            })
            .collect();
        let inputs = json!({"kind": "plate", "case": "G1", "mach": 0.2, "re_c": 1e6, "turbulence": "laminar", "ks": 0.0,
            "grid": {"r0": 20.0, "x_up": 0.5, "length": 1.0, "height": 0.6, "n_up": 20, "n_plate": 80, "nj": 64, "dx_le": 0.005, "dr_wall": 2e-4},
            "stop": "density residual 5 orders or cf stationary to 1e-5 per 250 iterations, 6000 max"});
        out.push(plate_record("plate-g1", &["G-V1"], inputs, it, stations, t0.elapsed().as_secs_f64()));
    }
    // G2 and G2r: fully turbulent plate (SST), x = 0.97 gate station; correlation 0.37 / log10(5e6 x)^2.584, rough: Mills-Hang
    for (id, ks, gates) in [("plate-g2", 0.0, &["G-V2"][..]), ("plate-g2r", 1e-3 * 0.97, &["G-V2"][..])] {
        println!("ref2d: {id} ...");
        let t0 = Instant::now();
        let mut p = plate(false, ks);
        let it = iterate_plate(&mut p, 4.0, 20_000, None);
        let targets: &[f64] = if ks > 0.0 { &[0.97] } else { &[0.25, 0.5, 0.75, 0.97] };
        let stations: Vec<Value> = targets
            .iter()
            .map(|&xt| {
                let (x, cf) = p.cf_at(xt);
                let (y_plus, ks_plus, k_plus, om_plus, nut_ratio, mu_t_max) = p.wall_units(xt, cf);
                let (reference, name) = if ks > 0.0 {
                    ((3.476 + 0.707 * (x / ks).ln()).powf(-2.46), "Mills-Hang (3.476 + 0.707 ln(x/ks))^-2.46")
                } else {
                    (0.37 / (5e6 * x).log10().powf(2.584), "0.37 / log10(5e6 x)^2.584")
                };
                json!({"x": x, "cf": cf, "reference": reference, "reference_name": name, "error": cf / reference - 1.0, "gated": xt == 0.97,
                       "y_plus": y_plus, "ks_plus": ks_plus, "k_w_plus": k_plus, "omega_w_plus": om_plus, "nu_t_w_over_nu": nut_ratio, "max_mu_t_over_mu_inf": mu_t_max})
            })
            .collect();
        let inputs = json!({"kind": "plate", "case": if ks > 0.0 { "G2r" } else { "G2" }, "mach": 0.2, "re_c": 2.5e7, "turbulence": "sst", "ks": ks,
            "grid": {"r0": 20.0, "x_up": 0.33, "length": 2.0, "height": 1.0, "n_up": 24, "n_plate": 112, "nj": 72, "dx_le": 0.002, "dr_wall": 4e-6},
            "gate_station_x": 0.97, "stop": "density residual 4 orders, 20000 max"});
        out.push(plate_record(id, gates, inputs, it, stations, t0.elapsed().as_secs_f64()));
    }
    out
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Cone (Taylor-Maccoll) and ogive (Perkins-Jorgensen) Euler cases of `bowshock-cases`, the 2D CPU solver on its own grids

fn cone_cases() -> Vec<Value> {
    let mut out = Vec::new();
    for &theta in &[10.0f64, 15.0] {
        let tm = solve_cone(2.0, theta.to_radians()).expect("attached conical shock");
        for (name, ni, nj) in [("coarse", 100usize, 50usize), ("medium", 200, 100)] {
            println!("ref2d: cone-{theta}deg-{name} ...");
            let case = ConeCase::new(2.0, theta, ni, nj);
            let t0 = Instant::now();
            let res = run_cone(&case, &tm);
            let wall_s = t0.elapsed().as_secs_f64();
            out.push(json!({
                "id": format!("cone-{theta}deg-M2.0-{name}"), "group": "cones", "gates": ["G-W4"],
                "inputs": {"kind": "cone", "mach": 2.0, "theta_deg": theta, "ni": ni, "nj": nj, "limiter": "Minmod", "cfl": 0.5, "drop_orders": 6.0,
                           "window": [0.4, 0.9], "first_order_iters": 300},
                "backend": "Cpu", "precision": "f64", "gpu": null,
                "theory": {"cp": tm.cp_surface, "beta_deg": tm.beta.to_degrees()},
                "numbers": {"cp_avg": res.cp_avg, "cp_cell_avg": res.cp_cell_avg, "beta_deg": res.beta_num.to_degrees(), "iterations": res.iters,
                            "drop_orders": res.drop_orders, "stop_reason": format!("{:?}", res.stop_reason)},
                "wall_s": wall_s,
            }));
        }
    }
    out
}

fn ogive_cases() -> Vec<Value> {
    let mut out = Vec::new();
    for level in 0..=1u32 {
        println!("ref2d: ogive-M1.98-level{level} ...");
        let case = OgiveCase::level(pj::MACH, level);
        let (ni, nj) = case.cells();
        let t0 = Instant::now();
        let res = run_ogive(&case);
        let wall_s = t0.elapsed().as_secs_f64();
        let cp_at: Vec<Value> = pj::X_OVER_D.iter().map(|&x| finite(wall_cp_at(&res.wall, x))).collect();
        out.push(json!({
            "id": format!("ogive-M{}-level{level}", pj::MACH), "group": "ogive", "gates": ["G-W4"],
            "inputs": {"kind": "ogive", "mach": pj::MACH, "fineness": pj::FINENESS, "level": level, "ni": ni, "nj": nj, "limiter": "Minmod", "cfl": 0.5,
                       "drop_orders": 6.0, "cyl_length": 6.0},
            "backend": "Cpu", "precision": "f64", "gpu": null,
            "data": {"x_over_d": pj::X_OVER_D, "cp_experiment": pj::CP, "cp_uncertainty": pj::CP_UNCERTAINTY},
            "numbers": {"cd_forebody": res.cd_nose, "cd_forebody_first_cell": res.cd_nose_cell, "cp_at_x_over_d": cp_at, "iterations": res.iters,
                        "drop_orders": res.drop_orders, "stop_reason": format!("{:?}", res.stop_reason), "shock_margin": res.shock_margin},
            "wall_s": wall_s,
        }));
    }
    out
}

// ---------------------------------------------------------------------------------------------------------------------------------
// The transonic and subsonic smoke cases of `bowshock-cases` (`transonic_smoke_coarse_m09`, `subsonic_smoke_no_base`, `subsonic_smoke_with_base`): the
// 2D engine `Case` built exactly as those tests build it, run cold on the CPU. The `checks` of a record are the asserts of the test (strict bounds on a
// key of `numbers`); the tests are the numeric tolerance of these tables (director decision, S0 follow-up). Not carried: the identities of the 2D
// post-processing the tests also assert (sum of the parts to 1e-14, modelled base drag, `base_wall.len() == n_gap`): they test 2D arithmetic, not physics.

/// The case of `bowshock-cases::subsonic` test `small(base, mach)`: coarse grids, the Euler base-wake or the sting grid.
fn subsonic_small(base: bool, mach: f64) -> Case {
    let mut c = subsonic_case(mach, Nose::TangentOgive { fineness: 3.0 }, 0, base).with_domain(5.0, 2.5);
    c.grid_base.n_up = 12;
    c.grid_base.n_nose = 24;
    c.grid_base.n_seg[0] = 20;
    c.grid_base.n_wake = 16;
    c.grid_base.n_gap = 8;
    c.grid_base.nj_out = 24;
    c.grid_base.dx_tip = 0.08;
    c.grid_base.dr_wall = 0.08;
    c.grid_sting.n_up = 12;
    c.grid_sting.n_nose = 24;
    c.grid_sting.n_seg[0] = 20;
    c.grid_sting.n_ext = 12;
    c.grid_sting.nj = 32;
    c.grid_sting.dx_tip = 0.08;
    c.grid_sting.dr_wall = 0.08;
    c.numerics.first_order_iters = 100;
    c.numerics.max_iter = 3000;
    c
}

/// The case of `bowshock-cases::transonic` test `transonic_smoke_coarse_m09`.
fn transonic_small() -> Case {
    let mut case = transonic_case(0.9, 0, Body { nose: Nose::cone_fineness(3.0) }, 6.0);
    case.grid_sting.n_up = 16;
    case.grid_sting.n_nose = 32;
    case.grid_sting.n_seg[0] = 16;
    case.grid_sting.n_ext = 14;
    case.grid_sting.nj = 40;
    case.grid_sting.dx_tip = 0.04;
    case.grid_sting.dr_wall = 0.04;
    case.numerics.max_iter = 1500;
    case.numerics.first_order_iters = 150;
    case
}

fn smoke_record(id: &str, inputs: Value, checks: Value, case: &Case) -> Value {
    println!("ref2d: {id} ...");
    let t0 = Instant::now();
    let r = run_cold(case);
    let wall_s = t0.elapsed().as_secs_f64();
    let (ni, nj) = case.cells();
    let mut inputs = inputs;
    inputs["ni"] = json!(ni);
    inputs["nj"] = json!(nj);
    json!({
        "id": id, "group": "w4-tables", "gates": ["G-W4"],
        "inputs": inputs,
        "backend": "Cpu", "precision": "f64", "gpu": null,
        "checks": checks,
        "numbers": {
            "iterations": r.iters,
            "residual_finite": r.residual.iter().all(|x| x.is_finite()),
            "drop_orders": r.drop_orders,
            "stop_reason": format!("{:?}", r.stop_reason),
            "min_rho": r.min_rho,
            "min_p": r.min_p,
            "cd_nose": r.cd_nose,
            "cd_aft": r.cd_aft,
            "cd_base": finite(r.cd_base),
            "cd_total": r.cd_total,
            "cp_tip": r.wall.first().map_or(Value::Null, |w| json!(w.cp)),
            "cp_min": r.cp_min,
            "cp_base": finite(r.cp_base),
            "cd_base_plus_cp_base": finite(r.cd_base + r.cp_base),
            // the base drag dominates the nose drag five times over (a check of the subsonic base smoke test)
            "cd_base_minus_5_abs_cd_nose": finite(r.cd_base - 5.0 * r.cd_nose.abs()),
        },
        "wall_s": wall_s,
    })
}

fn smoke_cases() -> Vec<Value> {
    let check = |key: &str, lo: Option<f64>, hi: Option<f64>| json!({"key": key, "gt": lo, "lt": hi});
    let sting_overrides = |c: &Case, nose: Value, mach: f64, builder: &str| {
        let g = &c.grid_sting;
        json!({
            "kind": "case2d", "builder": builder, "mach": mach, "level": 0, "wake": "sting", "nose": nose, "cyl_length": 6.0,
            "grid_sting": {"n_up": g.n_up, "n_nose": g.n_nose, "n_seg": g.n_seg, "n_ext": g.n_ext, "nj": g.nj, "dx_tip": g.dx_tip, "dr_wall": g.dr_wall},
            "first_order_iters": c.numerics.first_order_iters, "max_iter": c.numerics.max_iter, "numerics": builder,
        })
    };
    let mut out = Vec::new();
    // transonic: finite residual falling more than one order, positive state, a plausible nose drag, the tip stagnation and the shoulder suction
    let c = transonic_small();
    out.push(smoke_record(
        "w4-transonic-smoke-M0.90",
        sting_overrides(&c, json!({"cone_fineness": 3.0}), 0.9, "transonic_case"),
        json!([
            check("drop_orders", Some(1.0), None),
            check("min_rho", Some(0.0), None),
            check("min_p", Some(0.0), None),
            check("cd_nose", Some(0.0), Some(0.12)),
            check("cp_tip", Some(0.0), None),
            check("cp_min", None, Some(0.0)),
            {"key": "residual_finite", "is": true}
        ]),
        &c,
    ));
    // subsonic, no base: converged in < 1500 iterations, slender-body signature, nose drag near d'Alembert's zero
    let c = subsonic_small(false, 0.3);
    let mut inputs = sting_overrides(&c, json!({"tangent_ogive_fineness": 3.0}), 0.3, "subsonic_case");
    inputs["domain"] = json!([5.0, 2.5]);
    inputs["base"] = json!(false);
    out.push(smoke_record(
        "w4-subsonic-smoke-M0.30-sting",
        inputs,
        json!([check("iterations", None, Some(1500.0)), check("min_rho", Some(0.0), None), check("min_p", Some(0.0), None), check("cp_tip", Some(0.1), None),
               check("cp_min", Some(-0.3), Some(-0.05)), check("cd_nose", Some(-0.02), Some(0.02)), {"key": "stop_reason", "in": ["ResidualDrop"]},
               {"key": "residual_finite", "is": true}]),
        &c,
    ));
    // subsonic with a base: the standard level-0 base-wake grid, 6000 iterations at most; stationary base pressure, base suction, base drag = -Cp_b
    let mut c = subsonic_case(0.2, Nose::TangentOgive { fineness: 3.0 }, 0, true);
    c.numerics.max_iter = 6000;
    let g = &c.grid_base;
    let inputs = json!({
        "kind": "case2d", "builder": "subsonic_case", "mach": 0.2, "level": 0, "wake": "base", "nose": {"tangent_ogive_fineness": 3.0}, "cyl_length": 6.0, "base": true,
        "grid_base": {"n_up": g.n_up, "n_nose": g.n_nose, "n_seg": g.n_seg, "n_wake": g.n_wake, "n_gap": g.n_gap, "nj_out": g.nj_out, "dx_tip": g.dx_tip, "dr_wall": g.dr_wall},
        "first_order_iters": c.numerics.first_order_iters, "max_iter": 6000, "numerics": "subsonic_case",
    });
    out.push(smoke_record(
        "w4-subsonic-smoke-M0.20-base",
        inputs,
        json!([check("min_rho", Some(0.0), None), check("min_p", Some(0.0), None), check("cp_base", Some(-0.5), Some(-0.01)), check("cd_base", Some(0.0), None),
               // base CD = -Cp_b: the 2D test asserts 1e-12 (post-processing identity); the gate band is chosen, a flat-face wedge has a 1e-4 chord error
               check("cd_base_plus_cp_base", Some(-1e-3), Some(1e-3)), check("cd_base_minus_5_abs_cd_nose", Some(0.0), None),
               {"key": "stop_reason", "in": ["ResidualDrop", "BasePressureStationary"]}]),
        &c,
    ));
    out
}

// ---------------------------------------------------------------------------------------------------------------------------------

fn git_sha() -> (String, bool) {
    if let Ok(sha) = std::env::var("REF2D_SHA") {
        return (sha, std::env::var("REF2D_DIRTY").is_ok_and(|d| d == "1"));
    }
    let run = |args: &[&str]| {
        Command::new("git").args(args).output().ok().filter(|o| o.status.success()).map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    };
    let sha = run(&["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".into());
    let dirty = run(&["status", "--porcelain"]).is_some_and(|s| !s.is_empty());
    (sha, dirty)
}

fn host() -> String {
    Command::new("hostname").output().ok().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string()).unwrap_or_default()
}

fn date() -> String {
    Command::new("date").arg("-Is").output().ok().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string()).unwrap_or_default()
}

fn run_group(group: &str, errors: &mut usize) -> Vec<Value> {
    match group {
        "g3" => run_facades(g3_cases(), errors),
        "table-rans" => run_facades(table_cases(true), errors),
        "table-euler" => run_facades(table_cases(false), errors),
        "euler-base" => run_facades(euler_base_cases(), errors),
        "arcas" => run_facades(arcas_cases(), errors),
        "tp2005" => run_facades(tp2005_cases(), errors),
        "plates" => plate_cases(),
        "cones" => cone_cases(),
        "ogive" => ogive_cases(),
        "w4-tables" => smoke_cases(),
        other => panic!("unknown group {other}; known: {GROUPS:?}"),
    }
}

fn write_json(path: &Path, v: &Value) {
    if let Some(dir) = path.parent() {
        fs::create_dir_all(dir).expect("create output directory");
    }
    fs::write(path, serde_json::to_string_pretty(v).expect("serialise") + "\n").expect("write output");
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let arg = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    if args.iter().any(|a| a == "--list") {
        println!("{}", GROUPS.join("\n"));
        return;
    }
    if let Some(dir) = arg("--merge") {
        let out = PathBuf::from(arg("--out").expect("--merge needs --out"));
        let mut cases = Vec::new();
        let mut generated = Vec::new();
        for g in GROUPS {
            let path = Path::new(&dir).join(format!("{g}.json"));
            let text = fs::read_to_string(&path).unwrap_or_else(|e| panic!("group {g}: {}: {e}", path.display()));
            let v: Value = serde_json::from_str(&text).expect("group file is JSON");
            generated.push(json!({"group": g, "generated": v["generated"], "group_wall_s": v["group_wall_s"]}));
            cases.extend(v["cases"].as_array().expect("cases").iter().cloned());
        }
        let failed: Vec<&str> = cases.iter().filter(|c| c.get("error").is_some()).filter_map(|c| c["id"].as_str()).collect();
        assert!(failed.is_empty(), "failed cases in the groups: {failed:?}");
        let doc = json!({
            "schema": SCHEMA,
            "what": "Frozen results of the axisymmetric (2D) solver: the reference the 3D one-cell wedge must reproduce (DESIGN3D F3, G). Regenerate with examples/ref2d.rs.",
            "units": "drag coefficients on the nose-base area of the case; lengths in m (wall_x_r_cp) or nose diameters (plate x)",
            "groups": generated,
            "cases": cases,
        });
        write_json(&out, &doc);
        println!("ref2d: merged {} cases from {dir} into {}", doc["cases"].as_array().map_or(0, Vec::len), out.display());
        return;
    }
    let group = arg("--group").expect("usage: ref2d --list | --group NAME --out FILE | --merge DIR --out FILE");
    let out = PathBuf::from(arg("--out").unwrap_or_else(|| format!("out/ref2d/{group}.json")));
    let (sha, dirty) = git_sha();
    let t0 = Instant::now();
    let mut errors = 0;
    let cases = run_group(&group, &mut errors);
    let group_wall_s = t0.elapsed().as_secs_f64();
    let doc = json!({
        "generated": {"git_sha": sha, "dirty": dirty, "date": date(), "host": host(), "gpu": bowshock_core::gpu_name()},
        "group": group,
        "group_wall_s": group_wall_s,
        "cases": cases,
    });
    write_json(&out, &doc);
    println!("ref2d: group {group}: {} cases, {errors} failed, {group_wall_s:.1} s, written to {}", doc["cases"].as_array().map_or(0, Vec::len), out.display());
    println!("RESULT {}", json!({"ref2d_group": group, "cases": doc["cases"].as_array().map_or(0, Vec::len), "failed": errors, "wall_s": group_wall_s}));
    if group.starts_with("table-") {
        // one ledger line per table (the same shape as the app's `2d-table-example` line: wall time of the 10 points, iterations summed)
        let sum = |key: &str| doc["cases"].as_array().map_or(0.0, |cs| cs.iter().filter_map(|c| c["numbers"][key].as_f64()).sum::<f64>());
        let iters = sum("iterations");
        let wall = doc["cases"].as_array().map_or(0.0, |cs| cs.iter().filter_map(|c| c["wall_s"].as_f64()).sum::<f64>());
        let precision = "df32";
        println!(
            "PERF {}",
            json!({"case": format!("2d-{group}-ref2d"), "precision": precision, "cells": 21840, "iters": iters, "us_per_iter": 1e6 * wall / iters.max(1.0),
                   "launches_per_iter": null, "gpu_us_per_iter": null, "wall_s": wall})
        );
    }
    if errors > 0 {
        std::process::exit(1);
    }
}
