//! Structured body-fitted grid in the (x, r) meridian plane and its finite-volume metrics.
//!
//! Node `(i, j)`, `0<=i<=ni`, `0<=j<=nj`, stored at `j*(ni+1)+i`. Cell `(i, j)` is the quad
//! `(i,j) (i+1,j) (i+1,j+1) (i,j+1)` (counter-clockwise in the (x, r) plane), stored at `j*ni+i`.
//!
//! Axisymmetric weighting. Integrating `d(r U)/dt + d(r F)/dx + d(r G)/dr = S` over a planar
//! cell of area `A` gives
//!
//! ```text
//!   V dU/dt = - sum_faces  (r_mid * len) * (F.n)  +  (0, 0, p*A, 0),      V = integral r dA
//! ```
//!
//! where every face carries the weight `w = len * r_mid` (r_mid = mean radius of its end nodes,
//! exact for a straight edge), and `V` is the exact integral of `r` over the polygon. The
//! pressure source `p*A` is the exact counterpart of `sum r_mid * p * n_r * len = p*A` (Green's
//! theorem on the polygon), so a uniform pressure field is preserved to round-off. Faces lying
//! on the axis have `r_mid = 0` and therefore contribute no flux (no singularity).

/// Geometry of one face. `(nx, nr)` is the unit normal pointing towards increasing
/// i (for i-faces) or increasing j (for j-faces).
#[derive(Clone, Copy, Debug, Default)]
pub struct Face {
    pub nx: f64,
    pub nr: f64,
    /// Planar length in the (x, r) plane.
    pub len: f64,
    /// Mean radius of the end nodes.
    pub rm: f64,
}

impl Face {
    /// Flux weight `len * r_mid`.
    #[inline]
    pub fn w(&self) -> f64 {
        self.len * self.rm
    }
}

pub struct Grid {
    pub ni: usize,
    pub nj: usize,
    /// Node coordinates, `(ni+1)*(nj+1)`.
    pub x: Vec<f64>,
    pub r: Vec<f64>,
    /// i-faces (between cells i-1 and i): `(ni+1)*nj`, index `j*(ni+1)+i`.
    pub iface: Vec<Face>,
    /// j-faces (between cells j-1 and j): `ni*(nj+1)`, index `j*ni+i`.
    pub jface: Vec<Face>,
    /// Planar cell area `A`.
    pub area: Vec<f64>,
    /// Axisymmetric volume `V = integral r dA` (per radian).
    pub vol: Vec<f64>,
    /// Cell centroid.
    pub xc: Vec<f64>,
    pub rc: Vec<f64>,
}

impl Grid {
    /// Build the grid from node coordinates, panicking on a degenerate cell (see [`Grid::try_from_nodes`]).
    pub fn from_nodes(ni: usize, nj: usize, x: Vec<f64>, r: Vec<f64>) -> Grid {
        Grid::try_from_nodes(ni, nj, x, r).unwrap_or_else(|e| panic!("{e}"))
    }

    /// Build the grid from node coordinates (`(ni + 1) * (nj + 1)`, row `j` major); `Err` if a cell has non-positive area.
    pub fn try_from_nodes(ni: usize, nj: usize, x: Vec<f64>, r: Vec<f64>) -> Result<Grid, String> {
        assert_eq!(x.len(), (ni + 1) * (nj + 1));
        assert_eq!(r.len(), x.len());
        let nn = |i: usize, j: usize| j * (ni + 1) + i;

        let mut iface = vec![Face::default(); (ni + 1) * nj];
        for j in 0..nj {
            for i in 0..=ni {
                let (a, b) = (nn(i, j), nn(i, j + 1));
                let (dx, dr) = (x[b] - x[a], r[b] - r[a]);
                let len = dx.hypot(dr);
                iface[j * (ni + 1) + i] = Face { nx: dr / len, nr: -dx / len, len, rm: 0.5 * (r[a] + r[b]) };
            }
        }
        let mut jface = vec![Face::default(); ni * (nj + 1)];
        for j in 0..=nj {
            for i in 0..ni {
                let (a, b) = (nn(i, j), nn(i + 1, j));
                let (dx, dr) = (x[b] - x[a], r[b] - r[a]);
                let len = dx.hypot(dr);
                jface[j * ni + i] = Face { nx: -dr / len, nr: dx / len, len, rm: 0.5 * (r[a] + r[b]) };
            }
        }

        let mut area = vec![0.0; ni * nj];
        let mut vol = vec![0.0; ni * nj];
        let mut xc = vec![0.0; ni * nj];
        let mut rc = vec![0.0; ni * nj];
        for j in 0..nj {
            for i in 0..ni {
                let p = [nn(i, j), nn(i + 1, j), nn(i + 1, j + 1), nn(i, j + 1)];
                let (mut a, mut v, mut sx, mut sr) = (0.0, 0.0, 0.0, 0.0);
                // two triangles (0,1,2) and (0,2,3)
                for t in [[0usize, 1, 2], [0, 2, 3]] {
                    let (k0, k1, k2) = (p[t[0]], p[t[1]], p[t[2]]);
                    let at = 0.5 * ((x[k1] - x[k0]) * (r[k2] - r[k0]) - (x[k2] - x[k0]) * (r[k1] - r[k0]));
                    let cx = (x[k0] + x[k1] + x[k2]) / 3.0;
                    let cr = (r[k0] + r[k1] + r[k2]) / 3.0;
                    a += at;
                    v += at * cr;
                    sx += at * cx;
                    sr += at * cr;
                }
                if a <= 0.0 || a.is_nan() {
                    return Err(format!("non-positive cell area at ({i},{j})"));
                }
                let c = j * ni + i;
                area[c] = a;
                vol[c] = v;
                xc[c] = sx / a;
                rc[c] = sr / a;
            }
        }
        Ok(Grid { ni, nj, x, r, iface, jface, area, vol, xc, rc })
    }

    #[inline]
    pub fn node(&self, i: usize, j: usize) -> (f64, f64) {
        let k = j * (self.ni + 1) + i;
        (self.x[k], self.r[k])
    }

    /// Grid with every second node line removed in both directions (2x2 cells agglomerated into one). `None` if `ni` or
    /// `nj` is odd. Used for the multigrid hierarchy: coarse cell `(I, J)` is the union of fine cells `(2I..2I+2, 2J..2J+2)`.
    pub fn coarsen(&self) -> Option<Grid> {
        if self.ni % 2 != 0 || self.nj % 2 != 0 {
            return None;
        }
        let (ci, cj) = (self.ni / 2, self.nj / 2);
        let mut x = Vec::with_capacity((ci + 1) * (cj + 1));
        let mut r = Vec::with_capacity((ci + 1) * (cj + 1));
        for j in 0..=cj {
            for i in 0..=ci {
                let k = 2 * j * (self.ni + 1) + 2 * i;
                x.push(self.x[k]);
                r.push(self.r[k]);
            }
        }
        Some(Grid::from_nodes(ci, cj, x, r))
    }
}

/// Parameters of the cone grid.
#[derive(Clone, Debug)]
pub struct ConeGridParams {
    pub ni: usize,
    pub nj: usize,
    /// Cone half angle [rad]; nose at the origin, surface `r = x tan(theta)`.
    pub theta: f64,
    /// Downstream end of the domain (cone length).
    pub length: f64,
    /// Upstream end of the domain (negative: upstream of the tip).
    pub x_up: f64,
    /// Radius of the straight outer boundary at `x = x_up` and `x = length`.
    pub h_start: f64,
    pub h_end: f64,
    /// Wall clustering strength (0 = uniform; ~1-3 = tanh clustering towards j=0).
    pub cluster: f64,
}

/// Algebraic body-fitted grid. Grid lines of constant i are the vertical lines `x = x_i`;
/// the lower boundary is the axis `r=0` for `x<0` and the cone surface for `x>=0`; the upper
/// boundary is the straight line from `(x_up, h_start)` to `(length, h_end)`. A grid node sits
/// exactly at the tip `x=0`.
pub fn cone_grid(p: &ConeGridParams) -> Grid {
    let (ni, nj) = (p.ni, p.nj);
    assert!(p.x_up < 0.0 && p.length > 0.0 && ni >= 4 && nj >= 2);
    let n_up = (((ni as f64) * (-p.x_up) / (p.length - p.x_up)).round() as usize).clamp(1, ni - 1);
    let n_dn = ni - n_up;
    let mut xs = Vec::with_capacity(ni + 1);
    for i in 0..n_up {
        xs.push(p.x_up * (1.0 - i as f64 / n_up as f64));
    }
    for i in 0..=n_dn {
        xs.push(p.length * i as f64 / n_dn as f64);
    }
    let tan_c = p.theta.tan();
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = vec![0.0; (ni + 1) * (nj + 1)];
    for i in 0..=ni {
        let xi = xs[i];
        let r_in = if xi > 0.0 { xi * tan_c } else { 0.0 };
        let r_out = p.h_start + (p.h_end - p.h_start) * (xi - p.x_up) / (p.length - p.x_up);
        for j in 0..=nj {
            let xi_j = j as f64 / nj as f64;
            let s = if p.cluster > 0.0 { 1.0 + ((p.cluster * (xi_j - 1.0)).tanh()) / p.cluster.tanh() } else { xi_j };
            x[j * (ni + 1) + i] = xi;
            r[j * (ni + 1) + i] = r_in + s * (r_out - r_in);
        }
    }
    Grid::from_nodes(ni, nj, x, r)
}

/// Node offsets `0 = s_0 < s_1 < ... < s_n = length` with spacing `first * q^k`, `q` solved so that they sum to `length`.
pub fn geometric_offsets(first: f64, length: f64, n: usize) -> Vec<f64> {
    assert!(n >= 1 && first > 0.0 && length > 0.0);
    let sum = |q: f64| {
        let (mut s, mut d) = (0.0, first);
        for _ in 0..n {
            s += d;
            d *= q;
        }
        s
    };
    let (mut lo, mut hi) = (0.2f64, 4.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if sum(mid) < length {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let q = 0.5 * (lo + hi);
    let mut v = Vec::with_capacity(n + 1);
    let (mut s, mut d) = (0.0, first);
    for _ in 0..n {
        v.push(s);
        s += d;
        d *= q;
    }
    v.push(length); // closes any residual of the bisection exactly
    v
}

/// Parameters of the flat-plate grid ([`plate_grid`]).
#[derive(Clone, Debug)]
pub struct PlateGridParams {
    /// Radius of the plate surface (a large `r0` makes the axisymmetric annulus a flat plate).
    pub r0: f64,
    /// Length of the upstream approach region (`x < 0`).
    pub x_up: f64,
    /// Plate length (`x` in `[0, length]`).
    pub length: f64,
    /// Height of the domain above the plate surface.
    pub height: f64,
    /// Number of cells upstream of the leading edge and along the plate.
    pub n_up: usize,
    pub n_plate: usize,
    /// Number of cells normal to the plate.
    pub nj: usize,
    /// Cell size at the leading edge (geometric growth away from it in both directions).
    pub dx_le: f64,
    /// First cell height at the plate.
    pub dr_wall: f64,
}

/// Flat plate as the inner wall of an annulus: x nodes `-geometric_offsets(dx_le, x_up, n_up)` (reversed) then
/// `geometric_offsets(dx_le, length, n_plate)`; r nodes `r0 + geometric_offsets(dr_wall, height, nj)` in every column.
/// The leading edge is the node line `i = n_up`.
pub fn plate_grid(p: &PlateGridParams) -> Grid {
    let up = geometric_offsets(p.dx_le, p.x_up, p.n_up);
    let plate = geometric_offsets(p.dx_le, p.length, p.n_plate);
    let mut xs = Vec::with_capacity(p.n_up + p.n_plate + 1);
    for k in (1..=p.n_up).rev() {
        xs.push(-up[k]);
    }
    xs.extend_from_slice(&plate);
    let (ni, nj) = (p.n_up + p.n_plate, p.nj);
    let rs = geometric_offsets(p.dr_wall, p.height, nj);
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = vec![0.0; (ni + 1) * (nj + 1)];
    for j in 0..=nj {
        for i in 0..=ni {
            x[j * (ni + 1) + i] = xs[i];
            r[j * (ni + 1) + i] = p.r0 + rs[j];
        }
    }
    Grid::from_nodes(ni, nj, x, r)
}
