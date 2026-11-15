//! The comparison drawn over the analysis: a second set of curves from the other built-in method or an imported file, in SI,
//! with the arithmetic the plots and the "values at cursor" share.

use bowshock_model::ConfigId;

use super::compute::{Analysis, Data, Output};
use super::readout::active_run;

/// One point of a comparison curve; fields the source does not have are `None`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Mach number or angle of attack (rad), by the curve.
    pub x: f64,
    /// Static margin (cal).
    pub margin: Option<f64>,
    /// CP from the nose (m).
    pub cp: Option<f64>,
    /// Drag coefficient.
    pub cd: Option<f64>,
    /// Normal force slope (/rad).
    pub cn_alpha: Option<f64>,
    /// Normal force coefficient.
    pub cn: Option<f64>,
}

/// The curves of a comparison.
#[derive(Debug, Clone, PartialEq)]
pub struct Overlay {
    /// Source name drawn on the series (`OpenRocket`, `Extended`).
    pub label: String,
    /// Against Mach, ascending.
    pub mach: Vec<Sample>,
    /// Against angle of attack (rad), ascending.
    pub alpha: Vec<Sample>,
}

/// A quantity of a [`Sample`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// Static margin.
    Margin,
    /// Centre of pressure.
    Cp,
    /// Drag coefficient.
    Cd,
    /// Normal force coefficient.
    Cn,
}

impl Field {
    fn of(self, s: &Sample) -> Option<f64> {
        match self {
            Field::Margin => s.margin,
            Field::Cp => s.cp,
            Field::Cd => s.cd,
            Field::Cn => s.cn,
        }
    }
}

impl Overlay {
    /// The curve of `analysis`: Mach for the aerodynamics and drag analyses, angle for stability.
    pub fn curve(&self, analysis: Analysis) -> &[Sample] {
        match analysis {
            Analysis::Aerodynamics | Analysis::Drag => &self.mach,
            Analysis::Stability => &self.alpha,
            _ => &[],
        }
    }

    /// The points of `field` along the curve of `analysis` (x as stored: Mach, or radians), only where it exists.
    pub fn points(&self, analysis: Analysis, field: Field) -> Vec<(f64, f64)> {
        self.curve(analysis).iter().filter_map(|s| Some((s.x, field.of(s)?))).collect()
    }

    /// The linearly interpolated `field` at `x`; `None` outside the data's range (never extrapolated) or without any value.
    pub fn at(&self, analysis: Analysis, field: Field, x: f64) -> Option<f64> {
        let pts = self.points(analysis, field);
        let (first, last) = (pts.first()?, pts.last()?);
        if x < first.0 || x > last.0 {
            return None;
        }
        let i = pts.partition_point(|p| p.0 <= x);
        let Some(q) = pts.get(i) else { return Some(last.1) };
        let p = pts[i - 1];
        Some(if q.0 == p.0 { q.1 } else { p.1 + (q.1 - p.1) * (x - p.0) / (q.0 - p.0) })
    }

    /// The overlay of another computed result (the other method): the active configuration's run.
    pub fn from_output(out: &Output, active: ConfigId, label: &str) -> Option<Overlay> {
        let run = active_run(out, active)?;
        let mut o = Overlay { label: label.to_string(), mach: Vec::new(), alpha: Vec::new() };
        match &run.data {
            Data::Mach(points) => {
                o.mach = points
                    .iter()
                    .map(|p| Sample { x: p.mach, margin: Some(p.margin_cal), cp: Some(p.cp), cd: Some(p.drag.total), cn_alpha: Some(p.cn_alpha), cn: None })
                    .collect();
            }
            Data::Alpha(points) => {
                o.alpha = points
                    .iter()
                    .map(|p| Sample {
                        x: p.alpha,
                        margin: Some(p.margin_cal),
                        cp: Some(p.cp),
                        cd: Some(p.drag.total),
                        cn_alpha: Some(p.cn_alpha),
                        cn: Some(p.cn),
                    })
                    .collect();
            }
            _ => return None,
        }
        Some(o)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn overlay() -> Overlay {
        let s = |x: f64, cd: f64| Sample { x, margin: None, cp: None, cd: Some(cd), cn_alpha: None, cn: None };
        Overlay { label: "T".into(), mach: vec![s(0.0, 1.0), s(1.0, 3.0)], alpha: Vec::new() }
    }

    #[test]
    fn interpolates_and_holds_the_ends() {
        let o = overlay();
        assert_eq!(o.at(Analysis::Aerodynamics, Field::Cd, 0.25), Some(1.5));
        assert_eq!(o.at(Analysis::Aerodynamics, Field::Cd, 1.0), Some(3.0));
        assert_eq!(o.at(Analysis::Aerodynamics, Field::Cd, 5.0), None);
        assert_eq!(o.at(Analysis::Aerodynamics, Field::Margin, 0.5), None);
        assert!(o.points(Analysis::Stability, Field::Cd).is_empty());
    }
}
