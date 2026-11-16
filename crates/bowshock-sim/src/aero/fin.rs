use super::FinCalc;
use crate::vehicle::FinCrossSection;
use std::f64::consts::PI;
/// Roll angle (rad) of the crossflow Bowshock assumes: the lateral wind lies along +Y, the angle
/// a fin at roll angle 0 points to (a fin's roll angle is measured from +Y towards +X, see
/// [`crate::vehicle::FinSet::angles`]). A 1- or 2-fin set at angle 0 therefore sees the flow edge-on and gives no
/// lift; turn the set by 90 deg for full lift. The aerodynamic model is axisymmetric: the rigid-body flight model rolls (fin
/// cant) but the crossflow azimuth of the fin lift stays fixed here, so the roll angle only enters through this constant.
pub const CROSSFLOW_ROLL: f64 = 0.0;

/// `sum sin^2(CROSSFLOW_ROLL - phi_i)` over the fins at roll angles `angles` (`count` fins evenly
/// spaced from 0 when empty): a fin's normal force is proportional to the square of the sine of the
/// angle between its plane and the crossflow (Niskanen 2009, §3.2.2). For 3 or more evenly spaced
/// fins it is `N/2` whatever the roll.
pub fn roll_factor(angles: &[f64], count: u32) -> f64 {
    let s = |phi: f64| (CROSSFLOW_ROLL - phi).sin().powi(2);
    if angles.is_empty() {
        let n = f64::from(count);
        (0..count).map(|i| s(2.0 * PI * f64::from(i) / n)).sum()
    } else {
        angles.iter().map(|&a| s(a)).sum()
    }
}

/// Lift efficiency of `n` fins acting together on one body (fin-fin interference; Niskanen 2009,
/// §3.2.2): no loss up to 4 fins, then 0.948, 0.913, 0.854 and 0.81 for 5..8 fins, and 0.75 beyond.
pub fn fin_fin_factor(n: u32) -> f64 {
    match n {
        0..=4 => 1.0,
        5 => 0.948,
        6 => 0.913,
        7 => 0.854,
        8 => 0.81,
        _ => 0.75,
    }
}

// Fin normal force and CP vs Mach: Diederich subsonic lift, second-order supersonic theory, and a
// transonic polynomial bridge (Niskanen 2009, §3.2.2 and §3.2.4).

const CNA_SUBSONIC: f64 = 0.9;
const CNA_SUPERSONIC: f64 = 1.5;
const GAMMA: f64 = 1.4;
/// Fin stall angle (20 deg): the angle-of-attack terms of the fin lift saturate there.
pub(crate) const FIN_STALL: f64 = 20.0 * PI / 180.0;

/// Supersonic coefficients `(K1, K2, K3)` of `CN = A (K1 a + K2 a^2 + K3 a^3)` per unit angle
/// (Busemann second-order thin-airfoil theory; Pitts, Nielsen & Kaattari, NACA Report 1307;
/// Niskanen 2009 §3.2.2), evaluated in closed form.
pub(super) fn supersonic_k(mach: f64) -> (f64, f64, f64) {
    let m = mach.clamp(CNA_SUPERSONIC, 4.9);
    let b2 = m * m - 1.0;
    let beta = b2.sqrt();
    let k1 = 2.0 / beta;
    let k2 = ((GAMMA + 1.0) * m.powi(4) - 4.0 * b2) / (4.0 * b2 * b2);
    let k3 =
        ((GAMMA + 1.0) * m.powi(8) + (2.0 * GAMMA * GAMMA - 7.0 * GAMMA - 5.0) * m.powi(6) + 10.0 * (GAMMA + 1.0) * m.powi(4) + 8.0) / (6.0 * beta.powi(7));
    (k1, k2, k3)
}

pub(super) const SUBSONIC_CP_POS: f64 = 0.25;
const SUPERSONIC_CP_OFFSET: f64 = 0.67;
const LOW_AR_CP_BRIDGE_START: f64 = (SUPERSONIC_CP_OFFSET - SUBSONIC_CP_POS) / (1.0 - 2.0 * SUBSONIC_CP_POS);

fn source_supersonic_cp(x: f64) -> f64 {
    (x - SUPERSONIC_CP_OFFSET) / (2.0 * x - 1.0)
}

fn source_supersonic_cp_gradient(x: f64) -> f64 {
    let d = 2.0 * x - 1.0;
    (2.0 * SUPERSONIC_CP_OFFSET - 1.0) / (d * d)
}

/// Supersonic CP as a fraction of the MAC from its leading edge, `(x - 0.67)/(2x - 1)` with
/// `x = AR beta` (Fleeman 2001; Niskanen 2009 §3.2.2). The published rule falls to the quarter
/// chord at `x = 0.84` and runs into a pole at `x = 1/2` below that, so for `x < 1` it is replaced
/// by a cubic Hermite from the quarter chord at `x = 0.84` to the rule's value and slope at `x = 1`
/// (an engineering bridge, not a published curve): low-aspect-ratio fins keep a CP between the
/// subsonic and supersonic limits.
fn supersonic_cp(x: f64) -> f64 {
    if x <= LOW_AR_CP_BRIDGE_START {
        return SUBSONIC_CP_POS;
    }
    if x >= 1.0 {
        return source_supersonic_cp(x);
    }
    let w = 1.0 - LOW_AR_CP_BRIDGE_START;
    let t = (x - LOW_AR_CP_BRIDGE_START) / w;
    let (t2, t3) = (t * t, t * t * t);
    (2.0 * t3 - 3.0 * t2 + 1.0) * SUBSONIC_CP_POS + (-2.0 * t3 + 3.0 * t2) * source_supersonic_cp(1.0) + (t3 - t2) * w * source_supersonic_cp_gradient(1.0)
}

fn supersonic_cp_gradient(x: f64) -> f64 {
    if x <= LOW_AR_CP_BRIDGE_START {
        return 0.0;
    }
    if x >= 1.0 {
        return source_supersonic_cp_gradient(x);
    }
    let w = 1.0 - LOW_AR_CP_BRIDGE_START;
    let t = (x - LOW_AR_CP_BRIDGE_START) / w;
    let t2 = t * t;
    let sb = 6.0 * t2 - 6.0 * t;
    (sb * SUBSONIC_CP_POS - sb * source_supersonic_cp(1.0)) / w + (3.0 * t2 - 2.0 * t) * source_supersonic_cp_gradient(1.0)
}

/// Fin CP fraction of the MAC: quarter chord up to Mach 0.5, a quintic in Mach up to Mach 2 that
/// meets the supersonic value and slope there (the transonic interpolation of Niskanen 2009
/// §3.2.2, with a monotone variant when the quintic would overshoot), then the supersonic rule.
pub(crate) fn fin_cp_fraction(mach: f64, ar: f64) -> f64 {
    if mach <= 0.5 {
        return SUBSONIC_CP_POS;
    }
    if mach >= 2.0 {
        return supersonic_cp(ar * (mach * mach - 1.0).sqrt());
    }
    let t = (mach - 0.5) / 1.5;
    let x2 = ar * 3f64.sqrt();
    let delta = supersonic_cp(x2) - SUBSONIC_CP_POS;
    if delta <= 0.0 {
        return SUBSONIC_CP_POS;
    }
    let slope = x2 * supersonic_cp_gradient(x2);
    let ratio = slope / delta;
    let (t2, t3) = (t * t, t * t * t);
    if ratio <= 5.0 / 3.0 {
        let (t4, t5) = (t3 * t, t3 * t * t);
        SUBSONIC_CP_POS
            + (10.0 * delta - 6.0 * slope) * t2
            + (-20.0 * delta + 14.0 * slope) * t3
            + (15.0 * delta - 11.0 * slope) * t4
            + (3.0 * slope - 4.0 * delta) * t5
    } else {
        // Monotone continuation of the limiting quintic (slope ratio 5/3), raised to ratio/(5/3).
        let limiting = t3 * (10.0 / 3.0 - 10.0 / 3.0 * t + t2);
        SUBSONIC_CP_POS + delta * limiting.powf(ratio / (5.0 / 3.0))
    }
}

impl FinCalc {
    /// Normal-force slope of one fin on the reference area (Niskanen 2009 §3.2.2):
    /// Diederich below Mach 0.9, `A (K1 + K2 a + K3 a^2)/A_ref` above Mach 1.5, and a quartic
    /// through value and slope at both ends (zero curvature at 0.9) in between.
    pub(super) fn cna1(&self, mach: f64, alpha: f64, a_ref: f64) -> f64 {
        if self.area < 1e-12 || self.span < 1e-9 || self.cos_mid < 1e-9 {
            return 0.0;
        }
        let alpha = alpha.abs().min(PI - alpha.abs()).min(FIN_STALL);
        let q = self.span * self.span / (self.area * self.cos_mid);
        let sub = |m: f64| 2.0 * PI * self.span * self.span / (1.0 + (1.0 + (1.0 - m * m) * q * q).sqrt()) / a_ref;
        let sup = |m: f64, a: f64| {
            let (k1, k2, k3) = supersonic_k(m);
            self.area * (k1 + k2 * a + k3 * a * a) / a_ref
        };
        if mach <= CNA_SUBSONIC {
            return sub(mach);
        }
        if mach >= CNA_SUPERSONIC {
            return sup(mach, alpha);
        }
        let sq = (1.0 + (1.0 - CNA_SUBSONIC * CNA_SUBSONIC) * q * q).sqrt();
        let sub_v = sub(CNA_SUBSONIC);
        let sub_d = 2.0 * CNA_SUBSONIC * PI * self.span.powi(6) / ((self.area * self.cos_mid).powi(2) * a_ref * sq * (1.0 + sq).powi(2));
        let sup_v = sup(CNA_SUPERSONIC, alpha);
        let sup_d = -self.area / a_ref * 2.0 * CNA_SUPERSONIC / (CNA_SUPERSONIC * CNA_SUPERSONIC - 1.0).powf(1.5);
        // Quartic p(t), t = M - 0.9: p(0) = sub_v, p'(0) = sub_d, p''(0) = 0, p(h) = sup_v, p'(h) = sup_d.
        let h = CNA_SUPERSONIC - CNA_SUBSONIC;
        let t = mach - CNA_SUBSONIC;
        let r1 = sup_v - sub_v - sub_d * h;
        let r2 = sup_d - sub_d;
        let a4 = (r2 - 3.0 * r1 / h) / h.powi(3);
        let a3 = (r1 - a4 * h.powi(4)) / h.powi(3);
        sub_v + sub_d * t + a3 * t.powi(3) + a4 * t.powi(4)
    }

    /// Total CN slope of the fin set: one fin's slope times the set's lift multiplier (roll
    /// dependence, body-fin interference `K_fb = 1 + r/(s + r)` and fin-fin interference).
    pub(super) fn cna(&self, mach: f64, alpha: f64, a_ref: f64) -> f64 {
        self.cna1(mach, alpha, a_ref) * self.lift
    }

    /// CP station of the fin set at `mach`: MAC leading edge plus the Mach-dependent fraction of the MAC.
    pub(super) fn cp(&self, mach: f64) -> f64 {
        self.mac_lead + fin_cp_fraction(mach, self.ar) * self.mac
    }

    /// Fin drag (Niskanen 2009 section 3.4.4): friction `Cf (1 + 2 t/MAC) 2 A`, leading-edge pressure
    /// drag (stagnation for square edges, the rounded-edge fit for rounded/airfoil) scaled by
    /// `cos^2` of the leading-edge sweep, and trailing-edge base drag (full for square, half for
    /// rounded, none for airfoil), all times the fin count.
    pub(super) fn drag(&self, mach: f64, cf: f64, stag: f64, base: f64, a_ref: f64) -> f64 {
        let friction = cf * (1.0 + 2.0 * self.thickness / self.mac.max(1e-9)) * 2.0 * self.area * self.n / a_ref;
        let a_edge = self.n * self.span * self.thickness / a_ref;
        let le_cd = match self.cross {
            FinCrossSection::Square => stag,
            FinCrossSection::Rounded | FinCrossSection::Airfoil => round_edge_cd(mach),
        };
        let te_cd = match self.cross {
            FinCrossSection::Square => base,
            FinCrossSection::Rounded => 0.5 * base,
            FinCrossSection::Airfoil => 0.0,
        };
        friction + (le_cd * self.cos_lead * self.cos_lead + te_cd) * a_edge
    }
}

/// Leading-edge pressure drag of a rounded/airfoil fin edge (Niskanen 2009 eq. 3.89).
pub(super) fn round_edge_cd(m: f64) -> f64 {
    if m < 0.9 {
        (1.0 - m * m).powf(-0.417) - 1.0
    } else if m < 1.0 {
        1.0 - 1.785 * (m - 0.9)
    } else {
        1.214 - 0.502 / (m * m) + 0.1095 / m.powi(4)
    }
}
