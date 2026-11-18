use bowshock_sim::{vs_mach, Atmosphere, DragBreakdown, MachPoint};

use super::*;

#[test]
fn the_mach_sweep_interpolates_the_3d_solve_and_holds_it_above_the_last_node() {
    let lift = lift_table(1);
    let two = 2f64.to_radians();
    let at = lift_at(&lift, 1.4, &no_ext).unwrap(); // halfway between 0.8 and 2.0
    assert!((at.cn_alpha() - 0.15 / two).abs() < 1e-12);
    assert!((at.cp() - 1.2).abs() < 1e-12);
    let top = lift_at(&lift, 2.7, &no_ext).unwrap();
    assert!((top.cn_alpha() - 0.2 / two).abs() < 1e-12 && (top.cp() - 1.4).abs() < 1e-12);
}

#[test]
fn below_the_first_node_the_3d_solve_is_scaled_by_extended() {
    let lift = lift_table(1);
    // Extended: CNα 10 at M 0.8 and 8 at M 0.5; CP 0.9 and 0.7 m
    let ext = |m: f64| Some(if m < 0.6 { ExtStab { cn_alpha: 8.0, cp: 0.7 } } else { ExtStab { cn_alpha: 10.0, cp: 0.9 } });
    let at = lift_at(&lift, 0.5, &ext).unwrap();
    assert!((at.a2.0 - 0.1 * 0.8).abs() < 1e-12, "{at:?}");
    assert!((at.a2.1 - (1.0 - 0.2)).abs() < 1e-12);
}

#[test]
fn the_angle_rules_of_the_3d_solve() {
    let at = lift_at(&lift_table(1), 0.8, &no_ext).unwrap(); // CN 0.1 @ 2°, CP 1.0
    let deg = f64::to_radians;
    let (cn, cp) = at.at_alpha(deg(1.0));
    assert!((cn - 0.05).abs() < 1e-12 && (cp - 1.0).abs() < 1e-12, "linear to the origin, CP of 2°");
    let (cn, cp) = at.at_alpha(deg(10.0));
    assert!((cn - 0.5).abs() < 1e-12 && (cp - 1.0).abs() < 1e-12, "CNα·α beyond the angle, CP held");
    let (cn, cp) = at.at_alpha(deg(-4.0));
    assert!((cn + 0.2).abs() < 1e-12 && (cp - 1.0).abs() < 1e-12, "negative angles mirror");
    assert_eq!(at.at_alpha(0.0).0, 0.0);
}

#[test]
fn the_margin_from_the_3d_cp_is_cp_minus_cg_over_the_reference_diameter() {
    let (_, vehicle) = example();
    let cg = 0.6 * vehicle.length();
    let d = vehicle.reference_diameter;
    let lift = lift_table(1);
    let sweep = vs_mach(&vehicle, cg, (0.8, 2.0), 7, 2f64.to_radians(), &Atmosphere::standard(), 0.0);
    for p in lift_mach_points(&lift, sweep, cg, d, &no_ext) {
        assert!((p.margin_cal - (p.cp - cg) / d).abs() < 1e-12);
        let at = lift_at(&lift, p.mach, &no_ext).unwrap();
        assert!((p.cn_alpha - at.cn_alpha()).abs() < 1e-12 && (p.cp - at.cp()).abs() < 1e-12);
    }
    let alphas = bowshock_sim::vs_alpha(&vehicle, cg, 0.8, (0.0, 10f64.to_radians()), 6);
    let out = lift_alpha_points(&lift, 0.8, alphas, cg, d, &no_ext);
    for p in &out {
        assert!((p.margin_cal - (p.cp - cg) / d).abs() < 1e-12);
    }
    assert!((out[0].cn_alpha - 0.1 / 2f64.to_radians()).abs() < 1e-12, "zero angle: the small-angle slope");
    assert!(out.iter().skip(1).all(|p| (p.cn - p.cn_alpha * p.alpha).abs() < 1e-12));
}

#[test]
fn the_table_interpolates_and_holds_its_ends() {
    let t = table();
    assert!((at(&t, 0.75).cd - 0.6).abs() < 1e-12);
    assert!((at(&t, 1.5).cd - 0.7).abs() < 1e-12);
    assert_eq!(at(&t, 0.1).cd, 0.4);
    assert_eq!(at(&t, 3.0).cd, 0.6);
    let a = at(&t, 0.75);
    assert!((a.friction + a.pressure + a.base + a.fins + a.other - a.cd).abs() < 1e-12, "the parts add up");
    assert!(covers(&t, (0.1, 3.0)) && covers(&t, (1.0, 1.5)) && !covers(&t, (2.5, 3.0)));
}

#[test]
fn the_table_drag_replaces_the_extended_drag_and_keeps_its_stability() {
    let t = table();
    let (_, vehicle) = example();
    let atm = Atmosphere::standard();
    let sweep =
        |alpha: f64| -> Vec<MachPoint> { [0.4, 0.5, 1.0, 1.5, 2.0, 2.5].iter().flat_map(|&m| vs_mach(&vehicle, 1.0, (m, m), 1, alpha, &atm, 120.0)).collect() };
    let (ext, zero) = (sweep(0.0), sweep(0.0));
    let points = mach_points(&t, ext.clone(), Some(&zero));
    assert_eq!(points.len(), 5, "the table's Mach numbers (0.5 to 2.0) and the one below its first node");
    assert!((points[1].drag.total - 0.4).abs() < 1e-9 && (points[3].drag.total - 0.7).abs() < 1e-9);
    assert_eq!(points[2].margin_cal, ext[2].margin_cal, "margin, CP and CNα stay Extended's");
    // At an angle, Extended's change of drag is added on top of the solved zero-lift drag.
    let angled = sweep(8f64.to_radians());
    let with = mach_points(&t, angled.clone(), Some(&zero));
    assert!(with[2].drag.total > points[2].drag.total);
    let induced = angled[2].drag.total - zero[2].drag.total;
    assert!((with[2].drag.total - points[2].drag.total - induced).abs() < 1e-9);
}

/// An Extended-like sweep at zero angle over `machs` with parts that grow with Mach (`other` is zero when `no_other`).
fn synthetic(machs: &[f64], no_other: bool) -> Vec<MachPoint> {
    let (_, vehicle) = example();
    let atm = Atmosphere::standard();
    machs
        .iter()
        .flat_map(|&m| vs_mach(&vehicle, 1.0, (m, m), 1, 0.0, &atm, 120.0))
        .map(|mut p| {
            let m = p.mach;
            p.drag = DragBreakdown {
                friction: 0.02 * (1.0 + m),
                pressure: 0.05 * (1.0 + 2.0 * m),
                base: 0.03 * (1.0 + 3.0 * m),
                fins: 0.04 * (1.0 + 4.0 * m),
                protuberances: if no_other { 0.0 } else { 0.01 * (1.0 + 5.0 * m) },
                total: 0.0,
            };
            p
        })
        .collect()
}

fn table_from(first: f64) -> CfdTable {
    let mut t = table();
    t.points[0].mach = first;
    t
}

#[test]
fn below_the_first_node_the_first_point_is_scaled_by_extendeds_trend() {
    let t = table_from(0.3);
    let ext = synthetic(&[0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 2.5], false);
    let points = mach_points(&t, ext.clone(), None);
    assert_eq!(points.len(), 6, "below the first node is kept, above the last is not");
    let first = t.points[0];
    let at_lo = points.iter().find(|p| (p.mach - 0.3).abs() < 1e-9).unwrap();
    assert!((at_lo.drag.friction - first.friction).abs() < 1e-12 && (at_lo.drag.total - first.cd).abs() < 1e-12);
    let (m, lo) = (&ext[0].drag, &ext[2].drag);
    let p = &points[0].drag;
    assert!((p.friction - first.friction * m.friction / lo.friction).abs() < 1e-12);
    assert!((p.pressure - first.pressure * m.pressure / lo.pressure).abs() < 1e-12);
    assert!((p.base - first.base * m.base / lo.base).abs() < 1e-12);
    assert!((p.fins - first.fins * m.fins / lo.fins).abs() < 1e-12);
    assert!((p.protuberances - first.other * m.protuberances / lo.protuberances).abs() < 1e-12);
    assert!((p.total - (p.friction + p.pressure + p.base + p.fins + p.protuberances)).abs() < 1e-12);
}

#[test]
fn a_part_without_extended_drag_at_the_first_node_is_held() {
    let t = table_from(0.3);
    let ext = synthetic(&[0.1, 0.3, 0.5], true);
    let points = mach_points(&t, ext, None);
    let p = &points[0].drag;
    assert!(p.total.is_finite() && p.protuberances.is_finite());
    assert_eq!(p.protuberances, t.points[0].other);
}
