//! The wedge runner THROUGH THE FACADE: the same gates as `gpu.rs` (`gpu3d`'s direct GPU runner) and `cpu_wedge` (`solver3d`'s direct CPU runner), with the numbers
//! coming from `Analysis::engine(Engine::Solver3).run(..)`: the route of `analysis/solver3.rs` (the base-wake wedge, the class forces, the reference-area factor,
//! the guards). Selected by `BOWSHOCK_WEDGE_BACKEND=facade`; the backend of the facade runs is `BOWSHOCK_FACADE_BACKEND=cpu|gpu` (default: the GPU when the
//! `gpu` feature is on). GPU solver runs go through `tools/gpuq`:
//!
//! ```text
//! tools/gpuq/gpuq submit --host local --name wedge-gates-facade -- env BOWSHOCK_WEDGE_BACKEND=facade \
//!     cargo test --release -p bowshock-core --test wedge_gates -- --ignored --nocapture --test-threads=1 gate_w1 gate_w4
//! ```
//!
//! * `facade_case` (G-W1): the fixture case as an `Analysis` of its body (default reference diameter), Mach number and resolution on the base-wake grid (the sting
//!   grid when the fixture case has one). The facade's coefficients are on the largest cross-section, the unit of the fixture.
//! * `ogive_forebody_cd` (G-W4): the fineness-3 tangent ogive and 6 d cylinder as an `Analysis` at `level`.
//! * `case2d` (G-W4 smoke cases): the body and Mach number as an `Analysis`, the grid counts and iteration limits of the `bowshock-cases` builder given to the
//!   route through `case3_hook` (a function pointer: the parameters are in a thread-local). The facade does not expose the extremes of the state: a point that is
//!   not `Quality::Failed` has a positive density and pressure everywhere, which is what `min_rho` and `min_p` gate.
//! * `cone`: a bare meridian grid revolved into the wedge, not an `Analysis`: it delegates to the direct runner (`gpu.rs` on the GPU, `cpu_wedge` on the CPU).

use super::*;
use bowshock_core::engine::body_grid::{BaseGridParams, LargeGridParams};
use bowshock_core::engine::case3::Case3;
use bowshock_core::{AftSpec, Analysis, Backend, Engine, NoseShape, NoseSpec, Wake};
use serde_json::json;
use std::cell::RefCell;

pub struct FacadeWedge {
    pub backend: Backend,
    direct: Box<dyn WedgeRunner>,
}

impl FacadeWedge {
    pub fn new(direct: Box<dyn WedgeRunner>) -> FacadeWedge {
        let backend = match std::env::var("BOWSHOCK_FACADE_BACKEND").as_deref() {
            Ok("cpu") => Backend::Cpu,
            Ok("gpu") => Backend::Gpu,
            _ if cfg!(feature = "gpu") => Backend::Gpu,
            _ => Backend::Cpu,
        };
        FacadeWedge { backend, direct }
    }

    fn analysis(&self, body: &BodySpec, resolution: Resolution) -> Result<Analysis, String> {
        Ok(Analysis::new(body).map_err(|e| e.to_string())?.engine(Engine::Solver3).resolution(resolution).backend(self.backend))
    }
}

fn resolution_of(level: u32) -> Resolution {
    match level {
        0 => Resolution::Coarse,
        1 => Resolution::Medium,
        _ => Resolution::Fine,
    }
}

/// The grid counts and iteration limits of a smoke case, for the hook of the route.
#[derive(Clone)]
struct Smoke {
    base: Option<BaseGridParams>,
    sting: LargeGridParams,
    first_order_iters: usize,
    max_iter: usize,
}

thread_local! {
    static SMOKE: RefCell<Option<Smoke>> = const { RefCell::new(None) };
}

fn smoke_hook(c: &mut Case3) {
    SMOKE.with(|s| {
        if let Some(s) = s.borrow().as_ref() {
            c.grid.base = s.base.clone();
            c.grid.sting = s.sting.clone();
            c.numerics.first_order_iters = s.first_order_iters;
            c.numerics.max_iter = s.max_iter;
        }
    });
}

impl WedgeRunner for FacadeWedge {
    fn facade_case(&self, c: &FacadeCase) -> Result<Numbers, String> {
        if c.rans {
            return Err("RANS: Engine::Solver3 fails loudly on a viscous request (the engine is Euler)".into());
        }
        let mut a = self.analysis(&c.body, c.resolution)?;
        if !c.wake_base {
            a = a.wake(Wake::Sting);
        }
        let polar = a.run(&[c.mach]).map_err(|e| e.to_string())?;
        let p = &polar.points[0];
        // the facade's coefficients are on the reference area (the largest cross-section), which is also the fixture's unit (the direct runners scale the engine's
        // nose-base coefficients to it): nothing to convert here, that factor is the facade's
        Ok(Numbers {
            cd_total: p.drag.total,
            cd_nose: p.detail.cd_nose,
            cd_aft: p.detail.cd_aft.clone(),
            cd_friction: None,
            cd_base: c.wake_base.then_some(p.drag.base),
            cp_base: None,
            y_plus_max: None,
            iterations: p.detail.iterations,
            stop_reason: format!("{:?}", p.detail.stop_reason),
        })
    }

    fn cone(&self, mach: f64, theta_deg: f64, ni: usize, nj: usize) -> Result<ConeNumbers, String> {
        self.direct.cone(mach, theta_deg, ni, nj)
    }

    fn ogive_forebody_cd(&self, mach: f64, level: u32) -> Result<f64, String> {
        // the fineness-3 tangent ogive and 6 d cylinder of `bowshock-cases::ogive`, nose diameter 1: the reference area is the nose base
        let body = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 3.0, diameter: 1.0 }, aft: vec![AftSpec::Tube { length: 6.0 }] };
        let polar = self.analysis(&body, resolution_of(level))?.run(&[mach]).map_err(|e| e.to_string())?;
        let p = &polar.points[0];
        println!(
            "INFO ogive M {mach} level {level}: {} iterations, {:?}; forebody {:+.5} base {:+.5}",
            p.detail.iterations, p.detail.stop_reason, p.drag.forebody_pressure, p.drag.base
        );
        Ok(p.drag.forebody_pressure)
    }

    fn case2d(&self, c: &Case2d) -> Result<Value, String> {
        let i = &c.inputs;
        let mach = num(&i["mach"]);
        let level = i["level"].as_u64().unwrap_or(0) as u32;
        let cyl = num(&i["cyl_length"]);
        // the 2D case the builder makes: its body and its base-wake grid
        let (case2, body) = match s(&i["builder"]).as_str() {
            "transonic_case" => {
                let fineness = num(&i["nose"]["cone_fineness"]);
                let nose =
                    bowshock_core::engine::body_grid::Body { nose: bowshock_core::engine::body_grid::Nose::Cone { half_angle: (0.5 / fineness).atan() } };
                let case2 = bowshock_cases::transonic::transonic_case(mach, level, nose, cyl);
                (case2, BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: fineness, diameter: 1.0 }, aft: vec![AftSpec::Tube { length: cyl }] })
            }
            "subsonic_case" => {
                let fineness = num(&i["nose"]["tangent_ogive_fineness"]);
                let nose = bowshock_core::engine::body_grid::Nose::TangentOgive { fineness };
                let case2 = bowshock_cases::subsonic::subsonic_case(mach, nose, level, i["base"].as_bool().unwrap_or(false));
                (
                    case2,
                    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: fineness, diameter: 1.0 }, aft: vec![AftSpec::Tube { length: cyl }] },
                )
            }
            other => return Err(format!("case2d builder `{other}` is not known to the facade runner")),
        };
        let base = case2.wake == Wake::Base;
        SMOKE.with(|s| {
            *s.borrow_mut() = Some(Smoke {
                base: base.then(|| case2.grid_base.clone()),
                sting: case2.grid_sting.clone(),
                first_order_iters: i["first_order_iters"].as_u64().unwrap_or(0) as usize,
                max_iter: i["max_iter"].as_u64().unwrap_or(0) as usize,
            })
        });
        let mut a = self.analysis(&body, resolution_of(level))?.case3_hook(smoke_hook);
        if !base {
            a = a.wake(Wake::Sting);
        }
        let polar = a.run(&[mach]);
        SMOKE.with(|s| *s.borrow_mut() = None);
        let polar = polar.map_err(|e| e.to_string())?;
        let p = &polar.points[0];
        let d = &p.detail;
        let finite = |x: f64| if x.is_finite() { json!(x) } else { Value::Null };
        let cp_tip = d.wall.first().map(|w| w.cp);
        let cp_min = d.wall.iter().map(|w| w.cp).fold(f64::INFINITY, f64::min);
        let ok = p.quality != bowshock_core::Quality::Failed;
        let cd_base = if base { d.cd_base_solver } else { 0.0 };
        Ok(json!({
            "iterations": d.iterations,
            "residual_finite": d.residual_drop_orders.is_finite() && format!("{:?}", d.stop_reason) != "NonFinite",
            "drop_orders": d.residual_drop_orders,
            "stop_reason": format!("{:?}", d.stop_reason),
            "min_rho": if ok { 1.0 } else { 0.0 },
            "min_p": if ok { 1.0 } else { 0.0 },
            "cd_nose": d.cd_nose,
            "cp_tip": cp_tip.map_or(Value::Null, finite),
            "cp_min": cp_min,
            "cp_base": if base { finite(d.cp_base) } else { Value::Null },
            "cd_base": cd_base,
            "cd_base_plus_cp_base": if base { finite(d.cd_base_solver + d.cp_base) } else { Value::Null },
            "cd_base_minus_5_abs_cd_nose": if base { json!(d.cd_base_solver - 5.0 * d.cd_nose.abs()) } else { Value::Null },
        }))
    }
}
