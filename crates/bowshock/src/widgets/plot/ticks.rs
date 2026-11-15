//! Axis arithmetic: "nice" tick generation (1-2-5 steps) and number formatting shared by the plot canvases.

/// The 1-2-5 step closest to `span / target`, so that about `target` ticks fit in `span`.
///
/// Heckbert, "Nice Numbers for Graph Labels", Graphics Gems I (1990): the mantissa of the raw step is rounded to
/// 1, 2, 5 or 10.
pub fn nice_step(span: f64, target: usize) -> f64 {
    if !(span.is_finite() && span > 0.0) {
        return 1.0;
    }
    let raw = span / target.max(1) as f64;
    let mag = 10f64.powf(raw.log10().floor());
    let norm = raw / mag;
    let mantissa = if norm < 1.5 {
        1.0
    } else if norm < 3.5 {
        2.0
    } else if norm < 7.5 {
        5.0
    } else {
        10.0
    };
    mantissa * mag
}

/// The multiples of a [`nice_step`] that lie within `[lo, hi]`, aiming at roughly `target` ticks.
pub fn nice_ticks(lo: f64, hi: f64, target: usize) -> Vec<f64> {
    if !(lo.is_finite() && hi.is_finite()) || hi <= lo {
        return Vec::new();
    }
    let step = nice_step(hi - lo, target);
    let eps = step * 1e-9;
    let first = (lo / step - 1e-9).ceil() as i64;
    let last = (hi / step + 1e-9).floor() as i64;
    (first..=last).map(|k| k as f64 * step).filter(|v| *v >= lo - eps && *v <= hi + eps).map(|v| if v.abs() < eps { 0.0 } else { v }).collect()
}

/// `n` evenly spaced ticks from `lo` to `hi` inclusive, with the decimals that tell them apart: two significant digits for
/// a step that starts with 1 or 2 (`0.19`), otherwise just enough to print it (`0.6`, `67`).
pub fn even_ticks(lo: f64, hi: f64, n: usize) -> (Vec<f64>, usize) {
    if !(lo.is_finite() && hi.is_finite()) || hi <= lo || n < 2 {
        return (Vec::new(), 0);
    }
    let step = (hi - lo) / (n - 1) as f64;
    let mantissa = step / 10f64.powf(step.log10().floor());
    let decimals = decimals_for(step) + usize::from(mantissa < 3.0);
    ((0..n).map(|i| if i == n - 1 { hi } else { lo + step * i as f64 }).collect(), decimals)
}

/// Decimal places needed to print multiples of `step` exactly (0 for 1, 10, 100; 1 for 0.5, 0.2; 2 for 0.05).
pub fn decimals_for(step: f64) -> usize {
    if !(step.is_finite() && step > 0.0) {
        return 0;
    }
    (-step.log10().floor()).max(0.0) as usize
}

/// `v` with `decimals` places, never `-0`.
pub fn fmt_fixed(v: f64, decimals: usize) -> String {
    let s = format!("{v:.decimals$}");
    if s.starts_with('-') && s[1..].chars().all(|c| c == '0' || c == '.') {
        s[1..].to_string()
    } else {
        s
    }
}

/// Places for printing the ends of a range `span` wide: whole numbers for wide ranges, 2 for ordinary ones, more when
/// zoomed far in (`0 – 24`, `2.10 – 4.80`, `3.000 – 3.040`).
pub fn range_decimals(span: f64) -> usize {
    if !(span.is_finite() && span > 0.0) || span >= 10.0 {
        0
    } else if span >= 0.1 {
        2
    } else {
        (-span.log10().floor()) as usize + 2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_are_one_two_five() {
        assert_eq!(nice_step(100.0, 5), 20.0);
        assert_eq!(nice_step(10.0, 5), 2.0);
        assert_eq!(nice_step(3.0, 6), 0.5);
        assert_eq!(nice_step(0.9, 4), 0.2);
        assert_eq!(nice_step(0.0, 4), 1.0);
        assert_eq!(nice_step(f64::NAN, 4), 1.0);
    }

    #[test]
    fn ticks_lie_inside_the_range_on_multiples_of_the_step() {
        let t = nice_ticks(0.0, 96.0, 6);
        assert_eq!(t, vec![0.0, 20.0, 40.0, 60.0, 80.0]);
        let t = nice_ticks(-34.0, 140.0, 5);
        assert_eq!(t, vec![-20.0, 0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0]);
        let t = nice_ticks(0.8, 3.2, 5);
        assert_eq!(t, vec![1.0, 1.5, 2.0, 2.5, 3.0]);
        for t in nice_ticks(0.137, 0.482, 4) {
            assert!((0.137..=0.482).contains(&t));
        }
    }

    #[test]
    fn ticks_have_no_negative_zero_or_drift() {
        let t = nice_ticks(-0.3, 0.3, 6);
        assert!(t.iter().all(|v| !(*v == 0.0 && v.is_sign_negative())));
        assert!(t.contains(&0.0));
        // Multiples are computed as k * step, so a long run does not accumulate error.
        let t = nice_ticks(0.0, 1.0, 100);
        assert_eq!(t.len(), 101);
        assert!((t[30] - 0.3).abs() < 1e-12);
    }

    #[test]
    fn degenerate_ranges_have_no_ticks() {
        assert!(nice_ticks(1.0, 1.0, 5).is_empty());
        assert!(nice_ticks(2.0, 1.0, 5).is_empty());
        assert!(nice_ticks(f64::NAN, 1.0, 5).is_empty());
    }

    #[test]
    fn decimals_follow_the_step() {
        assert_eq!(decimals_for(100.0), 0);
        assert_eq!(decimals_for(1.0), 0);
        assert_eq!(decimals_for(0.5), 1);
        assert_eq!(decimals_for(0.2), 1);
        assert_eq!(decimals_for(0.05), 2);
        assert_eq!(decimals_for(0.0), 0);
    }

    #[test]
    fn formatting() {
        assert_eq!(fmt_fixed(-0.0001, 2), "0.00");
        assert_eq!(fmt_fixed(-1.5, 1), "-1.5");
        assert_eq!(range_decimals(96.0), 0);
        assert_eq!(range_decimals(2.7), 2);
        assert_eq!(range_decimals(0.04), 4);
    }
}
