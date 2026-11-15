//! The [`Motor`] type: a thrust curve plus mass and dimension data.

/// Impulse-class upper bounds start at 2.5 N·s (class A) and double per letter, up to class O.
const CLASS_A_MAX_NS: f64 = 2.5;

/// A single rocket motor as described by one RASP `.eng` entry. All quantities are SI (m, kg, s, N).
#[derive(Debug, Clone, PartialEq)]
pub struct Motor {
    /// Designation exactly as in the `.eng` header, e.g. `H128W`, `B6-0`, `163H133-14A`.
    pub designation: String,
    /// Manufacturer, with the common RASP abbreviations expanded (`AT` → `AeroTech`, `CTI` → `Cesaroni`, `E` → `Estes`).
    pub manufacturer: String,
    /// Designation without a delay suffix and without Cesaroni's leading total-impulse number, e.g. `H128W`, `H133`, `B6`.
    pub common_name: String,
    /// Motor outer diameter, m.
    pub diameter: f64,
    /// Motor length, m.
    pub length: f64,
    /// Available ejection delays, s. Empty means the motor is plugged (no ejection charge, header delay `P`).
    pub delays: Vec<f64>,
    /// Propellant mass, kg.
    pub propellant_mass: f64,
    /// Total (loaded) motor mass, kg.
    pub total_mass: f64,
    /// Thrust curve as `(time s, thrust N)` points with non-decreasing time. Thrust is taken as rising linearly from zero at
    /// `t = 0` to the first point when that point is later than `t = 0`.
    pub points: Vec<(f64, f64)>,
    /// `true` when the curve was hand-entered from manufacturer totals rather than measured data (file comment `APPROXIMATED`).
    pub approximated: bool,
    /// Comment lines that preceded the motor in the file (source and attribution), without the leading `;`.
    pub comments: Vec<String>,
}

impl Motor {
    /// Time of the last curve point, s (zero for an empty curve).
    pub fn burn_time(&self) -> f64 {
        self.points.last().map_or(0.0, |p| p.0)
    }

    /// Total impulse, N·s: the trapezoidal integral of the thrust curve from `t = 0`.
    pub fn total_impulse(&self) -> f64 {
        self.impulse_to(f64::INFINITY)
    }

    /// Average thrust over the burn, N (`total_impulse / burn_time`).
    pub fn average_thrust(&self) -> f64 {
        let tb = self.burn_time();
        if tb > 0.0 {
            self.total_impulse() / tb
        } else {
            0.0
        }
    }

    /// Peak thrust, N.
    pub fn max_thrust(&self) -> f64 {
        self.points.iter().map(|p| p.1).fold(0.0, f64::max)
    }

    /// NAR/Tripoli impulse class letter, `'A'` (≤ 2.5 N·s, including the fractional A classes) through `'O'`.
    /// Each class spans a doubling of total impulse: B ≤ 5 N·s, C ≤ 10 N·s, D ≤ 20 N·s, ...
    pub fn impulse_class(&self) -> char {
        let impulse = self.total_impulse();
        let mut upper = CLASS_A_MAX_NS;
        for letter in 'A'..='N' {
            if impulse <= upper {
                return letter;
            }
            upper *= 2.0;
        }
        'O'
    }

    /// Thrust at time `t` (s since ignition) by linear interpolation, N. Zero before ignition and after burnout.
    pub fn thrust(&self, t: f64) -> f64 {
        if t < 0.0 || t > self.burn_time() {
            return 0.0;
        }
        let idx = self.points.partition_point(|p| p.0 < t);
        let (t0, f0) = if idx == 0 { (0.0, 0.0) } else { self.points[idx - 1] };
        let (t1, f1) = self.points[idx];
        if t1 <= t0 {
            return f1;
        }
        f0 + (f1 - f0) * (t - t0) / (t1 - t0)
    }

    /// Impulse delivered between ignition and time `t`, N·s (piecewise-quadratic, exact for the linear thrust model).
    pub fn impulse_to(&self, t: f64) -> f64 {
        let mut total = 0.0;
        let (mut t0, mut f0) = (0.0, 0.0);
        for &(t1, f1) in &self.points {
            if t <= t0 {
                break;
            }
            if t1 > t0 {
                let te = t.min(t1);
                let fe = f0 + (f1 - f0) * (te - t0) / (t1 - t0);
                total += 0.5 * (f0 + fe) * (te - t0);
            }
            (t0, f0) = (t1, f1);
        }
        total
    }

    /// Propellant mass still in the motor at time `t`, kg. Burn-off is proportional to impulse delivered.
    pub fn propellant_remaining(&self, t: f64) -> f64 {
        let total = self.total_impulse();
        if total <= 0.0 {
            return self.propellant_mass;
        }
        let burned = (self.impulse_to(t) / total).clamp(0.0, 1.0);
        self.propellant_mass * (1.0 - burned)
    }

    /// Motor mass at time `t`, kg: the empty case plus the remaining propellant.
    pub fn mass(&self, t: f64) -> f64 {
        self.total_mass - self.propellant_mass + self.propellant_remaining(t)
    }

    /// `true` if `query` names this motor, ignoring case and any delay suffix: `H128W`, `h128w-14a`, `B6-4` and `163H133-14A` /
    /// `H133` all match their motors.
    pub fn matches(&self, query: &str) -> bool {
        let q = query.trim();
        if self.designation.eq_ignore_ascii_case(q) {
            return true;
        }
        let q = strip_delay(q);
        strip_delay(&self.designation).eq_ignore_ascii_case(q) || self.common_name.eq_ignore_ascii_case(q)
    }
}

/// Designation up to (not including) the first `-` (the delay suffix).
fn strip_delay(s: &str) -> &str {
    s.split('-').next().unwrap_or(s)
}

/// Common name of a designation: delay suffix and leading digits (Cesaroni's total impulse prefix) removed.
pub(crate) fn common_name(designation: &str) -> String {
    strip_delay(designation).trim_start_matches(|c: char| c.is_ascii_digit()).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(thrust: f64, burn: f64) -> Motor {
        Motor {
            designation: "X10-3".into(),
            manufacturer: "Test".into(),
            common_name: "X10".into(),
            diameter: 0.029,
            length: 0.1,
            delays: vec![3.0],
            propellant_mass: 0.02,
            total_mass: 0.05,
            points: vec![(0.0, thrust), (burn, thrust), (burn, 0.0)],
            approximated: false,
            comments: vec![],
        }
    }

    #[test]
    fn constant_thrust_integrates_exactly() {
        let m = square(10.0, 2.0);
        assert!((m.total_impulse() - 20.0).abs() < 1e-12);
        assert!((m.average_thrust() - 10.0).abs() < 1e-12);
        assert_eq!(m.max_thrust(), 10.0);
        assert_eq!(m.impulse_class(), 'D');
        assert!((m.impulse_to(0.5) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn interpolation_and_edges() {
        let m = Motor { points: vec![(0.1, 10.0), (0.3, 30.0), (0.5, 0.0)], ..square(1.0, 1.0) };
        // Ramp from (0, 0) to the first point.
        assert!((m.thrust(0.05) - 5.0).abs() < 1e-12);
        assert!((m.thrust(0.2) - 20.0).abs() < 1e-12);
        assert!((m.thrust(0.4) - 15.0).abs() < 1e-12);
        assert_eq!(m.thrust(-1.0), 0.0);
        assert_eq!(m.thrust(0.6), 0.0);
        // 0.5*0.1*10 + 0.5*(10+30)*0.2 + 0.5*30*0.2
        assert!((m.total_impulse() - (0.5 + 4.0 + 3.0)).abs() < 1e-12);
    }

    #[test]
    fn mass_burns_with_impulse() {
        let m = square(10.0, 2.0);
        assert!((m.mass(-1.0) - 0.05).abs() < 1e-12);
        assert!((m.mass(1.0) - 0.04).abs() < 1e-12);
        assert!((m.mass(2.0) - 0.03).abs() < 1e-12);
        assert!((m.mass(99.0) - 0.03).abs() < 1e-12);
        assert!((m.propellant_remaining(0.5) - 0.015).abs() < 1e-12);
    }

    #[test]
    fn impulse_class_boundaries() {
        let class_of = |ns: f64| square(ns, 1.0).impulse_class();
        assert_eq!(class_of(1.0), 'A');
        assert_eq!(class_of(2.5), 'A');
        assert_eq!(class_of(2.6), 'B');
        assert_eq!(class_of(10.0), 'C');
        assert_eq!(class_of(172.9), 'H');
        assert_eq!(class_of(640.0), 'I');
        assert_eq!(class_of(700.0), 'J');
        assert_eq!(class_of(15000.0), 'N');
        assert_eq!(class_of(30000.0), 'O');
    }

    #[test]
    fn designation_matching() {
        let h = Motor { designation: "163H133-14A".into(), common_name: "H133".into(), ..square(1.0, 1.0) };
        assert!(h.matches("163h133-14a") && h.matches("H133") && h.matches("H133-14A") && h.matches("163H133"));
        assert!(!h.matches("H13"));
        let b = Motor { designation: "B6-0".into(), common_name: "B6".into(), ..square(1.0, 1.0) };
        assert!(b.matches("B6-4") && b.matches("b6"));
        assert_eq!(common_name("163H133-14A"), "H133");
        assert_eq!(common_name("H128W"), "H128W");
        assert_eq!(common_name("J350W-L"), "J350W");
    }
}
