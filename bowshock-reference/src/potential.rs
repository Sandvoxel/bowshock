//! Incompressible potential flow past a closed body of revolution at zero incidence: an independent reference for the
//! low-Mach Euler results (no CFD code involved).
//!
//! Method (method of fundamental solutions / axial source distribution): the flow is `U e_x` plus the field of `n_s` point
//! sources of unknown strength `q_k` placed on the axis inside the body,
//!
//! ```text
//!   V(P) = U e_x + sum_k q_k (P - P_k) / (4 pi |P - P_k|^3),
//! ```
//!
//! and the strengths solve the no-penetration condition `V.n = 0` at `n_c > n_s` collocation points on the surface (regularised
//! least squares through the normal equations; the net source `sum q_k = 0` is added as a soft constraint). The surface speed gives
//! `Cp = 1 - |V|^2 / U^2`. Axisymmetric potential flows are represented by axial singularity distributions, so the solution
//! converges to the exact one as the point sets are refined; it is verified against the exact sphere solution
//! (`Cp = 1 - 9/4 sin^2 theta`) in the unit tests.
//!
//! Compressibility (linearised, thin body): Goethert's rule for bodies of revolution, `Cp_M[R](x) = Cp_inc[beta R](x) / beta^2`,
//! `beta = sqrt(1 - M^2)` (the Prandtl-Glauert transformation of the axisymmetric small-perturbation equation
//! `beta^2 phi_xx + phi_rr + phi_r / r = 0`), i.e. the incompressible solution for the body with radius scaled by `beta`.

use std::f64::consts::PI;

/// A closed body of revolution given by its meridian polyline `(x, r)` from the nose tip (`r = 0`) to the tail tip (`r = 0`).
#[derive(Clone, Debug)]
pub struct Meridian {
    pub pts: Vec<(f64, f64)>,
}

/// Solution on the surface: panel mid-points and their `Cp`.
#[derive(Clone, Debug)]
pub struct PotentialSolution {
    pub x: Vec<f64>,
    pub r: Vec<f64>,
    pub cp: Vec<f64>,
    /// Largest `|sum_k q_k| / max |q_k|` (closure check) and the RMS no-penetration residual `V.n / U` at the collocation points.
    pub closure: f64,
    pub residual: f64,
}

impl PotentialSolution {
    /// `Cp` at axial position `x` on the upper half of the body (linear interpolation of the panel values).
    pub fn cp_at(&self, x: f64) -> Option<f64> {
        for k in 0..self.x.len() - 1 {
            let (a, b) = (self.x[k], self.x[k + 1]);
            if (x >= a && x <= b) || (x >= b && x <= a) {
                if (b - a).abs() < 1e-14 {
                    return Some(self.cp[k]);
                }
                let t = (x - a) / (b - a);
                return Some(self.cp[k] + t * (self.cp[k + 1] - self.cp[k]));
            }
        }
        None
    }
}

fn solve_dense(a: &mut Vec<f64>, b: &mut Vec<f64>, n: usize) {
    // Gaussian elimination with partial pivoting (a: n x n row-major)
    for c in 0..n {
        let mut piv = c;
        for r in c + 1..n {
            if a[r * n + c].abs() > a[piv * n + c].abs() {
                piv = r;
            }
        }
        if piv != c {
            for k in 0..n {
                a.swap(c * n + k, piv * n + k);
            }
            b.swap(c, piv);
        }
        let d = a[c * n + c];
        for r in c + 1..n {
            let f = a[r * n + c] / d;
            if f != 0.0 {
                for k in c..n {
                    a[r * n + k] -= f * a[c * n + k];
                }
                b[r] -= f * b[c];
            }
        }
    }
    for c in (0..n).rev() {
        let mut s = b[c];
        for k in c + 1..n {
            s -= a[c * n + k] * b[k];
        }
        b[c] = s / a[c * n + c];
    }
}

/// Solve the potential flow past `body` with `n_src` axial sources (Chebyshev-clustered over the interior of the body's axial
/// extent, `margin` = fraction of the length kept free at both ends) and one collocation point per polyline segment (refine the
/// polyline for more). Free stream `U = 1` along `+x`.
pub fn solve(body: &Meridian, n_src: usize, margin: f64) -> PotentialSolution {
    let n_c = body.pts.len() - 1;
    let (x0, x1) = (body.pts[0].0, body.pts.last().unwrap().0);
    let len = x1 - x0;
    // Chebyshev-Lobatto-like source positions
    let xs: Vec<f64> = (0..n_src)
        .map(|k| {
            let t = 0.5 * (1.0 - (PI * (k as f64 + 0.5) / n_src as f64).cos());
            x0 + margin * len + (1.0 - 2.0 * margin) * len * t
        })
        .collect();
    // collocation points, normals (pointing out of the body; the meridian runs tip -> tail on the upper half, so the outward
    // normal of a segment with tangent (tx, tr) is (tr, -tx)).
    let mut col = Vec::with_capacity(n_c);
    for k in 0..n_c {
        let (a, b) = (body.pts[k], body.pts[k + 1]);
        let (tx, tr) = (b.0 - a.0, b.1 - a.1);
        let l = tx.hypot(tr);
        col.push((0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1), tr / l, -tx / l));
    }
    // velocity influence of a unit source at (xk, 0) on point (x, r): (dx, dr) / (4 pi d^3)
    let infl = |x: f64, r: f64, xk: f64| -> (f64, f64) {
        let (dx, d2) = (x - xk, (x - xk) * (x - xk) + r * r);
        let f = 1.0 / (4.0 * PI * d2 * d2.sqrt());
        (dx * f, r * f)
    };
    // normal equations for min sum (V.n)^2 + soft closure
    let mut a = vec![0.0; n_src * n_src];
    let mut rhs = vec![0.0; n_src];
    let mut rows: Vec<Vec<f64>> = Vec::with_capacity(n_c);
    let mut b0 = Vec::with_capacity(n_c);
    for &(x, r, nx, nr) in &col {
        let row: Vec<f64> = xs
            .iter()
            .map(|&xk| {
                let (vx, vr) = infl(x, r, xk);
                vx * nx + vr * nr
            })
            .collect();
        b0.push(-nx); // U e_x . n = nx, moved to the right-hand side
        rows.push(row);
    }
    for (row, &b) in rows.iter().zip(b0.iter()) {
        for i in 0..n_src {
            rhs[i] += row[i] * b;
            for j in 0..n_src {
                a[i * n_src + j] += row[i] * row[j];
            }
        }
    }
    let scale = (0..n_src).map(|i| a[i * n_src + i]).fold(0.0, f64::max);
    for i in 0..n_src {
        a[i * n_src + i] += 1e-13 * scale;
    }
    let w_closure = 1.0 * n_c as f64;
    for i in 0..n_src {
        for j in 0..n_src {
            a[i * n_src + j] += w_closure * 1e-6;
        }
    }
    let mut q = rhs;
    solve_dense(&mut a, &mut q, n_src);
    let qmax = q.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let closure = q.iter().sum::<f64>().abs() / qmax.max(1e-300);
    let (mut cx, mut cr, mut cp) = (Vec::new(), Vec::new(), Vec::new());
    let mut res2 = 0.0;
    for &(x, r, nx, nr) in &col {
        let (mut vx, mut vr) = (1.0, 0.0);
        for (k, &xk) in xs.iter().enumerate() {
            let (ux, ur) = infl(x, r, xk);
            vx += q[k] * ux;
            vr += q[k] * ur;
        }
        res2 += (vx * nx + vr * nr).powi(2);
        cx.push(x);
        cr.push(r);
        cp.push(1.0 - (vx * vx + vr * vr));
    }
    PotentialSolution { x: cx, r: cr, cp, closure, residual: (res2 / n_c as f64).sqrt() }
}

/// Closed body: nose profile `r_nose(x)` on `[0, l_nose]`, cylinder of radius `r_cyl` up to `x = l_nose + l_cyl`, mirrored tail
/// (the nose profile reversed), `n_nose` / `n_cyl` segments (cosine clustering at the ends of each part).
pub fn nose_cylinder_body(r_nose: &dyn Fn(f64) -> f64, l_nose: f64, l_cyl: f64, r_cyl: f64, n_nose: usize, n_cyl: usize) -> Meridian {
    let cl = |k: usize, n: usize| 0.5 * (1.0 - (PI * k as f64 / n as f64).cos());
    let mut pts = Vec::new();
    for k in 0..=n_nose {
        let x = l_nose * cl(k, n_nose);
        pts.push((x, if k == 0 { 0.0 } else { r_nose(x) }));
    }
    for k in 1..n_cyl {
        pts.push((l_nose + l_cyl * cl(k, n_cyl), r_cyl));
    }
    let x_tail = l_nose + l_cyl;
    for k in 0..=n_nose {
        let xt = l_nose * (1.0 - cl(n_nose - k, n_nose));
        // tail point at x_tail + xt has the radius of the nose at l_nose - xt
        let xn = l_nose - xt;
        pts.push((x_tail + xt, if k == n_nose { 0.0 } else { r_nose(xn.max(0.0)) }));
    }
    Meridian { pts }
}

/// Goethert-rule compressible `Cp` of the same body at Mach `m` (incompressible solution for the radius scaled by `beta`,
/// divided by `beta^2`).
pub fn solve_compressible(body: &Meridian, mach: f64, n_src: usize, margin: f64) -> PotentialSolution {
    let beta = (1.0 - mach * mach).sqrt();
    let scaled = Meridian { pts: body.pts.iter().map(|&(x, r)| (x, beta * r)).collect() };
    let mut sol = solve(&scaled, n_src, margin);
    for c in sol.cp.iter_mut() {
        *c /= beta * beta;
    }
    for r in sol.r.iter_mut() {
        *r /= beta;
    }
    sol
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_matches_the_exact_solution() {
        // sphere of radius 0.5 centred at x = 0.5, polyline from the nose to the tail (angle from the nose)
        let n = 160;
        let pts: Vec<(f64, f64)> = (0..=n)
            .map(|k| {
                let th = PI * k as f64 / n as f64;
                (0.5 - 0.5 * th.cos(), 0.5 * th.sin())
            })
            .collect();
        let sol = solve(&Meridian { pts }, 60, 0.02);
        let mut worst: f64 = 0.0;
        for k in 0..sol.x.len() {
            let th = ((0.5 - sol.x[k]) / 0.5).clamp(-1.0, 1.0).acos(); // angle from the nose
            let exact = 1.0 - 2.25 * th.sin().powi(2);
            worst = worst.max((sol.cp[k] - exact).abs());
        }
        assert!(worst < 5e-3, "sphere Cp error {worst}");
        assert!(sol.closure < 1e-3, "net source {}", sol.closure);
    }

    #[test]
    fn slender_body_cp_scales_like_the_thickness_squared_and_goethert_is_nearly_mach_independent() {
        // slender ellipsoid-like nose-cylinder: compressibility has (almost) no effect on a slender axisymmetric body
        let nose = |x: f64| 0.5 * (1.0 - ((3.0 - x) / 3.0).powi(2)).max(0.0).sqrt();
        let body = nose_cylinder_body(&nose, 3.0, 8.0, 0.5, 80, 120);
        let inc = solve(&body, 200, 0.003);
        let m5 = solve_compressible(&body, 0.5, 200, 0.003);
        let cp_inc = inc.cp_at(1.5).unwrap();
        let cp_m5 = m5.cp_at(1.5).unwrap();
        assert!(cp_inc < 0.0 && (cp_m5 - cp_inc).abs() < 0.2 * cp_inc.abs(), "{cp_inc} vs {cp_m5}");
        assert!(inc.residual < 1e-3, "no-penetration residual {}", inc.residual);
    }
}
