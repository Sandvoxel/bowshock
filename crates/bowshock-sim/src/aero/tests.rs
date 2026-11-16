use super::fin::{round_edge_cd, supersonic_k};
use super::tables::{base_cd, skin_cf, stagnation_cd};
use super::*;
use crate::vehicle::*;
use std::f64::consts::PI;

#[test]
fn component_drags_sum_to_the_total() {
    let a = Aero::new(&alpha_like());
    for mach in [0.3, 1.2] {
        let re = mach * 340.0 / 1.5e-5;
        let total = a.drag(mach, re, false).total;
        let comps = a.components(mach, re, false, 0.0);
        assert_eq!(comps.len(), 4);
        let sum: f64 = comps.iter().map(|c| c.drag.total).sum();
        assert!((sum - total).abs() <= 1e-12 * total, "{sum} vs {total} at M{mach}");
        let cn: f64 = comps.iter().map(|c| c.cn_alpha).sum();
        assert!((cn - a.stability(mach, 0.0).cn_alpha).abs() < 1e-12);
        assert!(comps[3].cp.is_nan() && comps[3].cn_alpha == 0.0, "a lug has no lift");
    }
}

#[test]
fn a_drag_override_replaces_the_component() {
    let mut v = alpha_like();
    let (mach, re) = (0.3, 0.3 * 340.0 / 1.5e-5);
    let before = Aero::new(&v);
    let old = before.drag(mach, re, false);
    v.labels.fin_sets = vec![Label { cd_override: Some(0.2), ..Label::default() }];
    let after = Aero::new(&v);
    let fins = &after.components(mach, re, false, 0.0)[2];
    assert_eq!((fins.drag.fins, fins.drag.total, fins.drag.friction), (0.2, 0.2, 0.0));
    let new = after.drag(mach, re, false);
    assert!((new.total - (old.total + 0.2 - old.fins)).abs() < 1e-12);
    assert!(after.has_cd_override() && !before.has_cd_override());
}

/// Estes Alpha III-like: 18.7 mm body, 0.3 m long (nose 0.1 m), 3 trapezoidal fins.
fn alpha_like() -> Vehicle {
    let r = 0.0093;
    Vehicle {
        reference_diameter: 2.0 * r,
        body: vec![
            BodySegment { station: 0.0, length: 0.07, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::TangentOgive), roughness: 20e-6 },
            BodySegment { station: 0.07, length: 0.26, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 20e-6 },
        ],
        tube_fins: vec![],
        fin_sets: vec![FinSet {
            count: 3,
            station: 0.26,
            body_radius: r,
            root_chord: 0.07,
            tip_chord: 0.03,
            span: 0.045,
            sweep: 0.03,
            thickness: 0.0016,
            cross_section: FinCrossSection::Rounded,
            cant: 0.0,
            roughness: 20e-6,
            planform: FinPlanform::Trapezoidal,
            angles: Vec::new(),
            parent: 0,
            bodies: 1,
            mid_chord_cos: None,
            root_extent: None,
        }],
        protuberances: vec![Protuberance::LaunchLug { station: 0.15, length: 0.02, outer_diameter: 0.005, inner_diameter: 0.004 }],
        stages: vec![],
        labels: Labels::default(),
    }
}

fn nose_only(shape: NoseShape, r: f64, fineness: f64) -> Vehicle {
    Vehicle {
        reference_diameter: 2.0 * r,
        body: vec![BodySegment { station: 0.0, length: 2.0 * r * fineness, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(shape), roughness: 0.0 }],
        tube_fins: vec![],
        fin_sets: vec![],
        protuberances: vec![],
        stages: vec![],
        labels: Labels::default(),
    }
}

#[test]
fn nose_cna_is_two_and_cp_matches_barrowman() {
    for (shape, cp_frac, tol) in [(NoseShape::Conical, 2.0 / 3.0, 1e-3), (NoseShape::TangentOgive, 0.466, 0.02)] {
        let r = 0.03;
        let v = Vehicle {
            reference_diameter: 2.0 * r,
            body: vec![BodySegment { station: 0.0, length: 0.18, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(shape), roughness: 0.0 }],
            tube_fins: vec![],
            fin_sets: vec![],
            protuberances: vec![],
            stages: vec![],
            labels: Labels::default(),
        };
        let st = Aero::new(&v).stability(0.3, 0.0);
        assert!((st.cn_alpha - 2.0).abs() < 1e-9);
        assert!((st.cp / 0.18 - cp_frac).abs() < tol, "{shape:?} {}", st.cp / 0.18);
    }
}

#[test]
fn transition_cp_matches_barrowman_formula() {
    // Frustum with R_f/R_a = 0.5: x_cp = L/3 (1 + (1 - 0.5)/(1 - 0.25)) from the fore face.
    let v = Vehicle {
        reference_diameter: 0.06,
        body: vec![BodySegment { station: 0.0, length: 0.1, fore_radius: 0.015, aft_radius: 0.03, kind: BodyKind::Transition, roughness: 0.0 }],
        tube_fins: vec![],
        fin_sets: vec![],
        protuberances: vec![],
        stages: vec![],
        labels: Labels::default(),
    };
    let st = Aero::new(&v).stability(0.3, 0.0);
    let expect = 0.1 / 3.0 * (1.0 + 0.5 / 0.75);
    assert!((st.cp - expect).abs() < 1e-4, "{} vs {expect}", st.cp);
    assert!((st.cn_alpha - 2.0 * (1.0 - 0.25)).abs() < 1e-9);
}

#[test]
fn stock_rocket_is_stable_with_cp_behind_cg() {
    let a = Aero::new(&alpha_like());
    let st = a.stability(0.1, 0.0);
    let margin = (st.cp - 0.21) / 0.0186;
    assert!((1.0..3.5).contains(&margin), "margin {margin} cp {}", st.cp);
    assert!(st.cn_alpha > 2.0 && st.cn_alpha < 60.0, "{}", st.cn_alpha);
    let st15 = a.stability(0.1, 15f64.to_radians());
    assert!(st15.cn_alpha > st.cn_alpha);
}

/// 3:1 ogive, 10 cal body, four trapezoidal fins on a 50 mm airframe.
fn typical() -> Vehicle {
    let r = 0.025;
    let mut v = alpha_like();
    v.reference_diameter = 2.0 * r;
    v.body = vec![
        BodySegment { station: 0.0, length: 0.15, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::TangentOgive), roughness: 10e-6 },
        BodySegment { station: 0.15, length: 0.5, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 10e-6 },
    ];
    v.fin_sets = vec![FinSet {
        count: 4,
        station: 0.55,
        body_radius: r,
        root_chord: 0.1,
        tip_chord: 0.05,
        span: 0.06,
        sweep: 0.04,
        thickness: 0.002,
        cross_section: FinCrossSection::Rounded,
        cant: 0.0,
        roughness: 10e-6,
        planform: FinPlanform::Trapezoidal,
        angles: Vec::new(),
        parent: 0,
        bodies: 1,
        mid_chord_cos: None,
        root_extent: None,
    }];
    v.protuberances = vec![];
    v
}

#[test]
fn drag_levels_and_transonic_peak() {
    let a = Aero::new(&typical());
    let nu = 1.46e-5;
    let d = a.drag(0.3, 0.3 * 340.0 / nu, false);
    assert!((0.3..0.7).contains(&d.total), "{d:?}");
    assert!((d.friction + d.pressure + d.base + d.fins + d.protuberances - d.total).abs() < 1e-12);
    let cd = |m: f64| a.drag(m, m * 340.0 / nu, false).total;
    let peak_m = (50..=300).max_by(|&i, &j| cd(i as f64 / 100.0).total_cmp(&cd(j as f64 / 100.0))).unwrap() as f64 / 100.0;
    assert!((0.95..1.25).contains(&peak_m), "peak at {peak_m}");
    assert!(cd(peak_m) > cd(0.3) * 1.3 && cd(peak_m) > cd(2.5), "peak {} sub {} sup {}", cd(peak_m), cd(0.3), cd(2.5));
    // Motor exhaust removes base drag.
    let mut v = alpha_like();
    v.stages.push(StageMass {
        dry: MassProps { mass: 0.05, cg: 0.2, i_long: 0.0, i_rot: 0.0 },
        fore_station: 0.0,
        aft_station: 0.33,
        motor: Some(StageMotor {
            curve: ThrustCurve { points: vec![(0.0, 5.0), (1.0, 5.0)], propellant_mass: 0.01, total_mass: 0.02, length: 0.07, diameter: 0.018 },
            mount_aft_station: 0.33,
            ignition: Timing::LAUNCH,
            ejection_delay: None,
            tilt_lockout: None,
        }),
        separation: Timing::BURNOUT,
    });
    let a2 = Aero::new(&v);
    assert!(a2.drag(0.3, 1e6, true).base < a2.drag(0.3, 1e6, false).base);
}

#[test]
fn fin_set_lift_compares_with_barrowman() {
    // Barrowman: CNa = 4N (s/d)^2 / (1 + sqrt(1 + (2 lm/(Cr+Ct))^2)) * K_fb, incompressible.
    let v = alpha_like();
    let f = &v.fin_sets[0];
    let d = v.reference_diameter;
    let lm = (f.span * f.span + (f.sweep + 0.5 * (f.tip_chord - f.root_chord)).powi(2)).sqrt();
    let n = 3.0;
    let kfb = 1.0 + f.body_radius / (f.span + f.body_radius);
    let expect = 4.0 * n * (f.span / d).powi(2) / (1.0 + (1.0 + (2.0 * lm / (f.root_chord + f.tip_chord)).powi(2)).sqrt()) * kfb;
    let st = Aero::new(&v).stability(0.0, 0.0);
    let fins = st.components.iter().find(|c| c.0 == "Fins").unwrap();
    assert!((fins.1 - expect).abs() / expect < 1e-9, "{} vs {expect}", fins.1);
}

fn fin_cna(v: &Vehicle, mach: f64) -> Vec<f64> {
    Aero::new(v).stability(mach, 0.0).components.iter().filter(|c| c.0.starts_with("Fin")).map(|c| c.1).collect()
}

#[test]
fn fin_lift_follows_sin_squared_of_roll() {
    // Niskanen 2009 §3.2.2: each fin's CNa is scaled by sin^2(theta - phi); theta = 0 (crossflow along +Y).
    let deg = PI / 180.0;
    assert!(roll_factor(&[0.0], 1).abs() < 1e-15);
    assert!((roll_factor(&[90.0 * deg], 1) - 1.0).abs() < 1e-12);
    assert!((roll_factor(&[150.0 * deg], 1) - 0.25).abs() < 1e-12);
    assert!((roll_factor(&[90.0 * deg, 270.0 * deg], 2) - 2.0).abs() < 1e-12);
    assert!(roll_factor(&[], 2).abs() < 1e-12);
    // 3+ evenly spaced fins: N/2 at any roll.
    for n in 3..=8 {
        assert!((roll_factor(&[], n) - f64::from(n) / 2.0).abs() < 1e-12);
        let turned: Vec<f64> = (0..n).map(|i| 0.3 + 2.0 * PI * f64::from(i) / f64::from(n)).collect();
        assert!((roll_factor(&turned, n) - f64::from(n) / 2.0).abs() < 1e-12);
    }
    // On the vehicle: one fin of the 3-fin set's slope is CNa(set)/1.5; 2 fins across the flow give twice that.
    let mut v = alpha_like();
    let three = fin_cna(&v, 0.3)[0];
    v.fin_sets[0].count = 2;
    v.fin_sets[0].angles = vec![0.0, PI];
    assert!(fin_cna(&v, 0.3).first().is_none_or(|c| c.abs() < 1e-12));
    v.fin_sets[0].angles = vec![PI / 2.0, 1.5 * PI];
    for m in [0.3, 1.2, 2.0] {
        v.fin_sets[0].count = 3;
        v.fin_sets[0].angles = Vec::new();
        let base = fin_cna(&v, m)[0] / 1.5;
        v.fin_sets[0].count = 2;
        v.fin_sets[0].angles = vec![PI / 2.0, 1.5 * PI];
        assert!((fin_cna(&v, m)[0] - 2.0 * base).abs() < 1e-12 * base.abs().max(1.0));
    }
    assert!((fin_cna(&v, 0.3)[0] - three / 1.5 * 2.0).abs() < 1e-12);
}

#[test]
fn many_fins_lose_lift_to_fin_fin_interference() {
    // Niskanen 2009 §3.2.2 table: 5..8 fins 0.948 / 0.913 / 0.854 / 0.81, then 0.75.
    assert_eq!([1, 2, 3, 4, 5, 6, 7, 8, 9, 12].map(fin_fin_factor), [1.0, 1.0, 1.0, 1.0, 0.948, 0.913, 0.854, 0.81, 0.75, 0.75]);
    let mut v = alpha_like();
    let one = fin_cna(&v, 0.5)[0] / 1.5;
    v.fin_sets[0].count = 6;
    assert!((fin_cna(&v, 0.5)[0] - 3.0 * 0.913 * one).abs() < 1e-12);
    // Two overlapping 4-fin sets on one body act as 8 fins; on different bodies (or apart) they do not interfere.
    v.fin_sets[0].count = 4;
    let mut second = v.fin_sets[0].clone();
    second.angles = (0..4).map(|i| PI / 4.0 + PI / 2.0 * f64::from(i)).collect();
    v.fin_sets.push(second);
    let both = fin_cna(&v, 0.5);
    assert!(both.iter().all(|c| (c - 2.0 * 0.81 * one).abs() < 1e-12), "{both:?}");
    v.fin_sets[1].parent = 7;
    assert!(fin_cna(&v, 0.5).iter().all(|c| (c - 2.0 * one).abs() < 1e-12));
    v.fin_sets[1].parent = 0;
    v.fin_sets[1].station -= v.fin_sets[1].root_chord;
    assert!(fin_cna(&v, 0.5).iter().all(|c| (c - 2.0 * one).abs() < 1e-12));
    // A pod set spread over 3 pods with 2 fins each counts 2 fins per body, not 6.
    let mut v = alpha_like();
    v.fin_sets[0].count = 6;
    v.fin_sets[0].bodies = 3;
    v.fin_sets[0].angles = vec![PI / 2.0, 1.5 * PI, PI / 2.0, 1.5 * PI, PI / 2.0, 1.5 * PI];
    assert!((fin_cna(&v, 0.5)[0] - 6.0 * one).abs() < 1e-12);
}

// ---- Hand-computed checks of the closed-form formulas ----

#[test]
fn stagnation_and_base_match_hand_calc() {
    // Stagnation (eq. B.1-B.2) and base (eq. 3.94) drag coefficients.
    assert!((stagnation_cd(0.0) - 0.85).abs() < 1e-12);
    assert!((stagnation_cd(1.0) - 0.85 * 1.275).abs() < 1e-12);
    assert!((stagnation_cd(2.0) - 0.85 * (1.84 - 0.19 + 0.010_375 + 0.000_546_875)).abs() < 1e-9);
    assert!((base_cd(0.5) - 0.1525).abs() < 1e-12);
    assert!((base_cd(1.0) - 0.25).abs() < 1e-12 && (base_cd(2.0) - 0.125).abs() < 1e-12);
}

#[test]
fn skin_friction_matches_flat_plate_theory() {
    // Fully turbulent: 1/(1.5 ln Re - 5.6)^2, subsonic compressibility (1 - 0.1 M^2).
    let t = 1.0 / (1.5 * 1e7f64.ln() - 5.6).powi(2);
    assert!((skin_cf(1e7, 0.0, false) - t).abs() < 1e-12);
    assert!((skin_cf(1e7, 0.5, false) - t * 0.975).abs() < 1e-12);
    assert!((skin_cf(1e3, 0.0, false) - 1.48e-2).abs() < 1e-12);
    // Supersonic: / (1 + 0.15 M^2)^0.58.
    assert!((skin_cf(1e7, 2.0, false) - t / 1.6f64.powf(0.58)).abs() < 1e-12);
    // Perfect finish: laminar below Re 5.39e5, turbulent minus 1700/Re above.
    assert!((skin_cf(1e5, 0.0, true) - 1.328 / 1e5f64.sqrt()).abs() < 1e-12);
    assert!((skin_cf(1e6, 0.0, true) - (1.0 / (1.5 * 1e6f64.ln() - 5.6).powi(2) - 1.7e-3)).abs() < 1e-12);
}

#[test]
fn nose_pressure_drag_matches_tr_r100_tables() {
    let nu = 1.46e-5;
    let press = |v: &Vehicle, m: f64| Aero::new(v).drag(m, m * 340.0 / nu, false).pressure;
    // Fineness-3 tangent ogive: Cd = 2.1 sin^2 + 0.5 sin / sqrt(M^2 - 1), sin = 1/sqrt(1 + 4 f^2), on the base area.
    let s = 1.0 / 37f64.sqrt();
    let v = nose_only(NoseShape::TangentOgive, 0.03, 3.0);
    assert!((press(&v, 2.0) - (2.1 * s * s + 0.5 * s / 3f64.sqrt())).abs() < 1e-4, "{}", press(&v, 2.0));
    // Conical at Mach 1: Cd = sin(phi) = R / sqrt(R^2 + L^2).
    let v = nose_only(NoseShape::Conical, 0.03, 3.0);
    assert!((press(&v, 1.0) - s).abs() < 1e-3, "{}", press(&v, 1.0));
    // Von Karman fineness 3 is the tabulated value (0.091 at Mach 2).
    let v = nose_only(NoseShape::VonKarman, 0.03, 3.0);
    assert!((press(&v, 2.0) - 0.091).abs() < 1e-6, "{}", press(&v, 2.0));
    // The data are extrapolated with stag*(Cd/stag)^(ln(f+1)/ln 4): fineness 7 lowers it.
    let v7 = nose_only(NoseShape::VonKarman, 0.03, 7.0);
    assert!(press(&v7, 2.0) < 0.091 * 0.8);
    // Subsonic nose pressure drag vanishes and rises towards Mach 1.
    assert!(press(&v, 0.3) < 1e-3 && press(&v, 0.95) >= press(&v, 0.6));
}

#[test]
fn rounded_fin_edge_matches_closed_form() {
    assert!((round_edge_cd(0.5) - ((1.0f64 - 0.25).powf(-0.417) - 1.0)).abs() < 1e-12);
    assert!((round_edge_cd(0.95) - (1.0 - 1.785 * 0.05)).abs() < 1e-12);
    assert!((round_edge_cd(2.0) - (1.214 - 0.1255 + 0.1095 / 16.0)).abs() < 1e-12);
}

#[test]
fn normal_force_is_the_component_sum_and_damping_sum_the_barrowman_strip_sum() {
    let a = Aero::new(&typical());
    for (mach, alpha) in [(0.1, 0.0), (0.3, 0.05), (0.8, 0.2), (2.0, 0.1)] {
        let st = a.stability(mach, alpha);
        let nf = a.normal_force(mach, alpha);
        assert!((nf.cn_alpha - st.cn_alpha).abs() < 1e-12 && (nf.cp - st.cp).abs() < 1e-12);
        for cg in [0.3, 0.55, 0.8] {
            let hand: f64 = st.components.iter().map(|(_, cna, x)| cna * (x - cg) * (x - cg)).sum();
            assert!((nf.damping_sum(cg) - hand).abs() < 1e-9 * hand.max(1.0), "M {mach} cg {cg}: {} vs {hand}", nf.damping_sum(cg));
        }
    }
    // Damping about the CG is smallest when the lifting components cluster at the CG and grows with the lever arm.
    let nf = a.normal_force(0.3, 0.0);
    assert!(nf.damping_sum(nf.cp) < nf.damping_sum(nf.cp - 0.3));
}

#[test]
fn body_crossflow_damping_is_the_planform_third_moment() {
    // A uniform tube of length L and radius r about its own mid-point: sum A |x - cg|^3 = 2 r * 2 (L/2)^4 / 4 = r L^4 / 16.
    let (r, l) = (0.02, 0.5);
    let v = Vehicle {
        reference_diameter: 2.0 * r,
        body: vec![BodySegment { station: 0.0, length: l, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 0.0 }],
        fin_sets: vec![],
        tube_fins: vec![],
        protuberances: vec![],
        stages: vec![],
        labels: Labels::default(),
    };
    let k = Aero::new(&v).crossflow_damping(0.5 * l);
    assert!((k - r * l.powi(4) / 16.0).abs() < 1e-3 * r * l.powi(4) / 16.0, "{k}");
}

#[test]
fn fin_roll_forcing_follows_cant_and_damping_is_the_strip_sum() {
    let mut v = alpha_like();
    let a0 = Aero::new(&v);
    assert_eq!(a0.roll(0.3).forcing, 0.0, "no cant, no roll forcing");
    let f = v.fin_sets[0].clone();
    v.fin_sets[0].cant = 0.02;
    let a = Aero::new(&v);
    let r = a.roll(0.3);
    assert!(r.forcing > 0.0 && r.damping < 0.0);
    // 3 fins at 120 degrees: the crossflow lift multiplier is N/2 = 1.5, so each fin's face-on lift is the set's / 1.5.
    let set = a.stability(0.3, 0.0).components.iter().find(|c| c.0 == "Fins").unwrap().1;
    let (a_ref, one_fin_lift_times_n) = (std::f64::consts::FRAC_PI_4 * v.reference_diameter.powi(2), set / 1.5 * 3.0);
    // The arms are chord-weighted averages of r = R + y over the span, so they lie between R and R + span.
    let arm2 = -r.damping / (one_fin_lift_times_n * a_ref);
    let arm = r.forcing / (one_fin_lift_times_n * a_ref * 0.02);
    assert!((f.body_radius..f.body_radius + f.span).contains(&arm), "arm {arm}");
    assert!(arm2 >= arm * arm * 0.999 && arm2 <= (f.body_radius + f.span).powi(2), "arm2 {arm2} arm {arm}");
    // Doubling the cant doubles the forcing and leaves the damping alone.
    v.fin_sets[0].cant = 0.04;
    let r2 = Aero::new(&v).roll(0.3);
    assert!((r2.forcing / r.forcing - 2.0).abs() < 1e-9 && (r2.damping / r.damping - 1.0).abs() < 1e-9);
}

#[test]
fn axial_drag_multiplier_shape() {
    assert!((axial_drag_multiplier(0.0) - 1.0).abs() < 1e-12);
    assert!((axial_drag_multiplier(17f64.to_radians()) - 1.3).abs() < 1e-9);
    assert!(axial_drag_multiplier(PI / 2.0).abs() < 1e-9);
    let a = 17f64.to_radians();
    assert!((axial_drag_multiplier(a - 1e-9) - axial_drag_multiplier(a + 1e-9)).abs() < 1e-6);
}

#[test]
fn perfect_finish_reduces_low_speed_drag() {
    let v = typical();
    let rough = Aero::new(&v).drag(0.2, 2e6, false).total;
    let perfect = Aero::new(&v).perfect_finish(true).drag(0.2, 2e6, false).total;
    assert!(perfect <= rough);
}

/// Vehicles with different fin shapes/aspect ratios for the transonic continuity checks.
fn fin_variants() -> Vec<Vehicle> {
    let mut out = vec![alpha_like(), typical()];
    let mut elliptical = typical();
    elliptical.fin_sets[0].planform = FinPlanform::Elliptical;
    out.push(elliptical);
    let mut low_ar = typical();
    low_ar.fin_sets[0].span = 0.03;
    low_ar.fin_sets[0].root_chord = 0.14;
    low_ar.fin_sets[0].tip_chord = 0.1;
    out.push(low_ar);
    let mut high_ar = typical();
    high_ar.fin_sets[0].span = 0.15;
    high_ar.fin_sets[0].root_chord = 0.06;
    high_ar.fin_sets[0].tip_chord = 0.03;
    high_ar.fin_sets[0].sweep = 0.05;
    out.push(high_ar);
    out
}

#[test]
fn cna_and_cp_are_c1_continuous_through_transonic() {
    // Fin CNa is blended with a quartic over Mach 0.9..1.5 and CP with a quintic over
    // 0.5..2, both matching value and slope at the ends, so no corners may remain.
    for v in fin_variants() {
        let a = Aero::new(&v);
        let len = v.length();
        let step = 0.005;
        let n = ((3.0 - 0.5) / step) as usize;
        let samples: Vec<(f64, f64)> = (0..=n)
            .map(|i| {
                let st = a.stability(0.5 + step * i as f64, 0.0);
                (st.cn_alpha, st.cp)
            })
            .collect();
        for w in samples.windows(3) {
            // First differences (jumps) and second differences (slope corners).
            assert!((w[1].0 - w[0].0).abs() < 0.06 * w[0].0.abs().max(1.0), "CNa jump {w:?}");
            assert!((w[1].1 - w[0].1).abs() < 8e-3 * len, "CP jump {w:?}");
            let d2n = w[2].0 - 2.0 * w[1].0 + w[0].0;
            let d2p = w[2].1 - 2.0 * w[1].1 + w[0].1;
            assert!(d2n.abs() < 4e-3 * w[1].0.abs().max(1.0), "CNa corner {w:?} d2 {d2n}");
            assert!(d2p.abs() < 4e-4 * len, "CP corner {w:?} d2 {d2p}");
        }
    }
}

/// Regression (results comparison report under `docs/`): the transonic fin CNa is ONE quartic in Mach through
/// 0.9..1.5 with the derivative at Mach 0.9 taken at 0.9, as Niskanen (2009, section 3.2.2) describes.
/// A quartic has a vanishing fifth finite difference.
#[test]
fn transonic_fin_cna_is_a_single_quartic_with_the_slope_taken_at_mach_0_9() {
    for v in fin_variants() {
        let a = Aero::new(&v);
        let a_ref = v.reference_area();
        for fin in &a.fins {
            let step = 0.04;
            let f: Vec<f64> = (0..6).map(|i| fin.cna1(0.95 + step * i as f64, 0.0, a_ref)).collect();
            let fifth = f[5] - 5.0 * f[4] + 10.0 * f[3] - 10.0 * f[2] + 5.0 * f[1] - f[0];
            assert!(fifth.abs() < 1e-9 * f[0].abs().max(1.0), "fifth difference {fifth} of {f:?}");
            // The slope leaving Mach 0.9 is the Diederich slope there, not the one at the Mach being evaluated.
            let h = 1e-5;
            let left = (fin.cna1(0.9, 0.0, a_ref) - fin.cna1(0.9 - h, 0.0, a_ref)) / h;
            let right = (fin.cna1(0.9 + h, 0.0, a_ref) - fin.cna1(0.9, 0.0, a_ref)) / h;
            assert!((left - right).abs() < 2e-3 * left.abs().max(1.0), "slope {left} vs {right}");
        }
    }
}

#[test]
fn supersonic_margin_shrinks_but_stays_positive() {
    // Supersonic fin lift (K1 = 2/beta, per fin) falls well below the subsonic value, so
    // the CP moves forward and the margin shrinks to a fraction of its subsonic value, but stays positive.
    let a = Aero::new(&typical());
    let cg = 0.42;
    let m03 = a.margin_cal(0.3, 0.0, cg);
    let m3 = a.margin_cal(3.0, 0.0, cg);
    assert!(m3 > 0.2 && m3 < 0.7 * m03, "margin M0.3 {m03} M3 {m3}");
    // At Mach 1.5 the fin CNa is the K1 term exactly: A * 2/beta / A_ref per fin.
    let v = typical();
    let f = &v.fin_sets[0];
    let beta = (1.5f64 * 1.5 - 1.0).sqrt();
    let (_, k2, k3) = supersonic_k(1.5);
    assert!(k2 > 0.0 && k3 > 0.0);
    let expect = 0.5 * 4.0 * f.fin_area() * 2.0 / beta / v.reference_area() * (1.0 + f.body_radius / (f.span + f.body_radius));
    let st = a.stability(1.5, 0.0);
    let fins = st.components.iter().find(|c| c.0 == "Fins").unwrap();
    assert!((fins.1 - expect).abs() / expect < 1e-9, "{} vs {expect}", fins.1);
}

#[test]
fn fin_cp_fraction_limits() {
    assert_eq!(fin_cp_fraction(0.3, 1.5), 0.25);
    // Supersonic CP at high AR*beta tends to mid-chord from ahead of it.
    let hi = fin_cp_fraction(3.0, 4.0);
    assert!(hi > 0.25 && hi < 0.5, "{hi}");
    // Value and slope agree on both sides of Mach 2.
    let (a, b) = (fin_cp_fraction(2.0 - 1e-6, 1.5), fin_cp_fraction(2.0 + 1e-6, 1.5));
    assert!((a - b).abs() < 1e-5);
}
