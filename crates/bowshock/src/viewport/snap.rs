//! Snapping of a handle drag (design kit, Snapping): the grid step of a plain drag, the coarse step of Ctrl/Cmd, and with
//! Shift the catch of the nearest target within 8 screen px. The targets come from the model once at drag start
//! ([`Rocket::snap_targets`](bowshock_model::Rocket::snap_targets)); this module only decides which one catches.

use bowshock_model::{Dim, Target};

/// A target catches when the handle is within this many screen px of it.
pub const CATCH_PX: f32 = 8.0;
/// Ticks are drawn for targets within this many screen px of the handle.
pub const TICK_RANGE_PX: f32 = 220.0;

/// The grid step of `dim` in its own unit (metres, radians, or the shape parameter): `grid` is the toolbar's step in metres.
/// Lengths and positions take the toolbar step (5 mm coarse); diameters and thicknesses a tenth of it (1 mm coarse); angles
/// 1 degree (15 coarse); the shape parameter 0.01 (0.05 coarse).
pub fn step(dim: Dim, grid: f64, coarse: bool) -> f64 {
    match dim {
        Dim::Cant | Dim::Angle => (if coarse { 15.0_f64 } else { 1.0 }).to_radians(),
        Dim::Shape => {
            if coarse {
                0.05
            } else {
                0.01
            }
        }
        Dim::Diameter | Dim::ForeDiameter | Dim::InnerDiameter | Dim::Thickness | Dim::Wall => {
            if coarse {
                0.001
            } else {
                (grid / 10.0).max(0.00005)
            }
        }
        _ => {
            if coarse {
                0.005
            } else {
                grid
            }
        }
    }
}

/// `value` rounded to a multiple of `step`.
pub fn quantize(value: f64, step: f64) -> f64 {
    if step > 0.0 {
        (value / step).round() * step
    } else {
        value
    }
}

/// The index of the target that catches `raw`: the nearest within [`CATCH_PX`], where `px_per_unit` converts a value
/// difference to screen px; ties (within half a pixel) go to the higher priority.
pub fn catch(targets: &[Target], raw: f64, px_per_unit: f32) -> Option<usize> {
    let px = |t: &Target| ((t.value - raw).abs() * f64::from(px_per_unit.abs())) as f32;
    targets
        .iter()
        .enumerate()
        .filter(|(_, t)| px(t) <= CATCH_PX)
        .min_by(|(_, a), (_, b)| {
            let (pa, pb) = (px(a), px(b));
            if (pa - pb).abs() < 0.5 {
                a.priority.cmp(&b.priority)
            } else {
                pa.total_cmp(&pb)
            }
        })
        .map(|(i, _)| i)
}

/// The targets to draw ticks for: within [`TICK_RANGE_PX`] of `raw`, nearest first, at most `max`.
pub fn ticks(targets: &[Target], raw: f64, px_per_unit: f32, max: usize) -> Vec<usize> {
    let px = |t: &Target| ((t.value - raw).abs() * f64::from(px_per_unit.abs())) as f32;
    let mut near: Vec<usize> = (0..targets.len()).filter(|&i| px(&targets[i]) <= TICK_RANGE_PX).collect();
    near.sort_by(|&a, &b| px(&targets[a]).total_cmp(&px(&targets[b])));
    near.truncate(max);
    near
}

/// The outcome of snapping a raw value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Snapped {
    /// The value to set.
    pub value: f64,
    /// The target that caught it, by index into the list given to [`snap`].
    pub caught: Option<usize>,
}

/// Snaps `raw`: with `shift` to a catching target, else (or with none near) to the grid (`coarse` with Ctrl/Cmd).
pub fn snap(dim: Dim, raw: f64, targets: &[Target], px_per_unit: f32, shift: bool, coarse: bool, grid: f64) -> Snapped {
    if shift {
        if let Some(i) = catch(targets, raw, px_per_unit) {
            return Snapped { value: targets[i].value, caught: Some(i) };
        }
    }
    Snapped { value: quantize(raw, step(dim, grid, coarse)), caught: None }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(value: f64, priority: u8) -> Target {
        Target { value, label: String::new(), source: None, priority }
    }

    #[test]
    fn plain_drags_use_the_grid_ctrl_the_coarse_step() {
        let s = |dim, raw, coarse| snap(dim, raw, &[], 1000.0, false, coarse, 0.001).value;
        assert!((s(Dim::Length, 0.60049, false) - 0.600).abs() < 1e-12);
        assert!((s(Dim::Length, 0.6026, true) - 0.605).abs() < 1e-12);
        assert!((s(Dim::Diameter, 0.07623, false) - 0.0762).abs() < 1e-12);
        assert!((s(Dim::Angle, 0.3, false).to_degrees() - 17.0).abs() < 1e-9);
        assert!((s(Dim::Angle, 0.3, true).to_degrees() - 15.0).abs() < 1e-9);
    }

    #[test]
    fn shift_catches_within_eight_px_and_falls_back_to_the_grid() {
        let targets = [t(0.600, 1), t(0.6096, 3)];
        // At 1000 px/m 8 px is 8 mm: 0.5985 is 1.5 px from 0.600.
        let caught = snap(Dim::Length, 0.5985, &targets, 1000.0, true, false, 0.001);
        assert_eq!((caught.value, caught.caught), (0.600, Some(0)));
        // 0.58 is 20 px from the nearest: no catch, the grid takes it.
        let free = snap(Dim::Length, 0.5804, &targets, 1000.0, true, false, 0.001);
        assert_eq!(free.caught, None);
        assert!((free.value - 0.580).abs() < 1e-12);
        // Without Shift nothing catches.
        assert_eq!(snap(Dim::Length, 0.5985, &targets, 1000.0, false, false, 0.001).caught, None);
    }

    #[test]
    fn a_tie_goes_to_the_higher_priority() {
        let targets = [t(0.6000, 3), t(0.6002, 1)];
        assert_eq!(catch(&targets, 0.6001, 1000.0), Some(1));
        // A clearly nearer target of lower priority still wins.
        let targets = [t(0.6000, 3), t(0.6060, 1)];
        assert_eq!(catch(&targets, 0.6001, 1000.0), Some(0));
    }

    #[test]
    fn ticks_are_the_nearby_targets_nearest_first() {
        let targets = [t(0.9, 1), t(0.61, 1), t(0.6, 2)];
        assert_eq!(ticks(&targets, 0.6, 1000.0, 5), vec![2, 1]);
        assert_eq!(ticks(&targets, 0.6, 1000.0, 1), vec![2]);
    }
}
