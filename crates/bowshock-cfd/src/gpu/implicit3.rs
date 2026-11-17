// The three-factor line-implicit scheme `Scheme3::Implicit` in array ops. This file is `include!`d by `gpu/mod.rs` after `body3.rs` and `implicit.rs`
// into the same per-precision module, so it sees `S`, `A`, `Rec3`, `State3`, `LazySolver3`, `lhs` / `from_lhs`, the f32 helpers of `implicit.rs`
// (`F`, `cf`, `up_lhs`, the block entry `E`, `shock_weight_f`) and everything `body3.rs` imports.
//
// It mirrors `crate::implicit3` (`flux_jacobian3`, `abs_jacobian3`, `cell_blocks3`, `assemble3`, `solve_lines3`, `apply_factor3`, `step`) the way
// `implicit.rs` mirrors `crate::implicit`: one iteration solves the backward-Euler pseudo-time step with the first-order flux Jacobians on the left,
//
//     (I - dt J_i) X = dt R,   (I - dt J_j) Y = X,   (I - dt J_k) dU = Y,   U <- U + alpha dU,
//
// `R` being the residual of the full second-order scheme (`LazySolver3::eval_residual`, in the precision `S`). The left-hand side (Jacobians,
// dissipation, ghost Jacobians, dt, line solves, defect correction) is plain f32 in both precisions: it only steers the iteration, the fixed point is
// that of the `S`-precision `R`.
//
// A 5x5 block is `Blk5`: twenty-five entries, row-major like the CPU `M5`, each an array over the cells or faces (`E::A`) or the constants 0 / 1
// (`E::Z`, `E::O`), so that the structurally zero entries of the flux Jacobian cost nothing. The line solves are Luma's
// `solve_block_tridiagonal_parts::<25, 5>` on the entries of each block (no stacking); a vector field is five `[ni, nj, nk]` arrays.
//
// Solid cells (fins) are identity rows (`d = I`, `lo = up = 0`, `dt = 0`, so the right-hand side is zero), which cuts every line through a fin
// into independent segments; the fluid cell next to a solid cell drops that off-diagonal block and adds `B T` to its diagonal with `T` the
// reflection about the wall face, exactly as the CPU does.

/// Five f32 fields (primitive state, or a conserved vector).
type V5 = [F; 5];
/// Three f32 fields (a face normal).
type N3 = [F; 3];

/// `[b0, b1, b2, b3, b4]` of a vector of five fields.
fn v5(f: impl FnMut(usize) -> F) -> V5 {
    std::array::from_fn(f)
}

// ---------------------------------------------------------------------------------------------------------------------
// 5x5 blocks
// ---------------------------------------------------------------------------------------------------------------------

/// A 5x5 block, row-major (`m[5 r + c]`, the CPU `M5`), one entry per row / column pair.
#[derive(Clone)]
struct Blk5([E; 25]);

impl Blk5 {
    fn zero() -> Blk5 {
        Blk5(std::array::from_fn(|_| E::Z))
    }

    fn eye() -> Blk5 {
        Blk5(std::array::from_fn(|k| if k % 6 == 0 { E::O } else { E::Z }))
    }

    fn zip(&self, o: &Blk5, f: impl Fn(&E, &E) -> E) -> Blk5 {
        Blk5(std::array::from_fn(|k| f(&self.0[k], &o.0[k])))
    }

    fn add(&self, o: &Blk5) -> Blk5 {
        self.zip(o, E::add)
    }

    fn sub(&self, o: &Blk5) -> Blk5 {
        self.zip(o, E::sub)
    }

    /// Every entry times the array `s`.
    fn scale(&self, s: &F) -> Blk5 {
        Blk5(std::array::from_fn(|k| self.0[k].scale(s)))
    }

    /// `self + d I`.
    fn add_diag(&self, d: &F) -> Blk5 {
        Blk5(std::array::from_fn(|k| if k % 6 == 0 { self.0[k].add(&E::A(d.clone())) } else { self.0[k].clone() }))
    }

    /// `self - d I`.
    fn sub_diag(&self, d: &F) -> Blk5 {
        Blk5(std::array::from_fn(|k| if k % 6 == 0 { self.0[k].sub(&E::A(d.clone())) } else { self.0[k].clone() }))
    }

    /// `self o`: `c[5 i + j] = sum_k a[5 i + k] b[5 k + j]`, `k` ascending like the CPU `mm`.
    fn mm(&self, o: &Blk5) -> Blk5 {
        Blk5(std::array::from_fn(|n| {
            let (i, j) = (n / 5, n % 5);
            (0..5).fold(E::Z, |acc, k| acc.add(&self.0[5 * i + k].mul(&o.0[5 * k + j])))
        }))
    }

    /// The part `range` of every array entry along `axis`.
    fn slice(&self, axis: usize, range: std::ops::Range<usize>) -> Blk5 {
        Blk5(std::array::from_fn(|k| match &self.0[k] {
            E::A(a) => E::A(a.slice(axis, range.clone())),
            e => e.clone(),
        }))
    }

    /// All twenty-five entries as arrays (`z` and `o` the arrays of zeros and ones of the shape).
    fn dense(&self, z: &F, o: &F) -> [F; 25] {
        std::array::from_fn(|k| match &self.0[k] {
            E::Z => z.clone(),
            E::O => o.clone(),
            E::A(a) => a.clone(),
        })
    }

    fn of(dense: &[F; 25]) -> Blk5 {
        Blk5(std::array::from_fn(|k| E::A(dense[k].clone())))
    }
}

/// `y = a x` for a block of arrays: `y_i = sum_j a_ij x_j`.
fn mv5(a: &[F; 25], x: &V5) -> V5 {
    v5(|i| &a[5 * i] * &x[0] + &a[5 * i + 1] * &x[1] + &a[5 * i + 2] * &x[2] + &a[5 * i + 3] * &x[3] + &a[5 * i + 4] * &x[4])
}

// ---------------------------------------------------------------------------------------------------------------------
// gas, flux Jacobian, dissipation
// ---------------------------------------------------------------------------------------------------------------------

/// `rho = q0, u = q1 / rho, v = q2 / rho, w = q3 / rho, p = (g - 1) (E - rho |u|^2 / 2)` in f32.
fn cons_to_prim_f5(q: &V5) -> V5 {
    let u = &q[1] / &q[0];
    let v = &q[2] / &q[0];
    let w = &q[3] / &q[0];
    let p = cf(G1) * (&q[4] - 0.5f32 * &q[0] * (&u * &u + &v * &v + &w * &w));
    [q[0].clone(), u, v, w, p]
}

/// `A = d(F.n)/dU` of the primitive state `w` for the unit normal `n` (conservative variables `(rho, rho u, rho v, rho w, E)`):
/// `h = g p / ((g - 1) rho) + |u|^2 / 2` and the rows of [`flux_jacobian_h5`].
fn flux_jacobian_f5(w: &V5, n: &N3) -> Blk5 {
    let [rho, u, v, ww, p] = w;
    let h = cf(GAMMA) * p / (cf(G1) * rho) + 0.5f32 * (u * u + v * v + ww * ww);
    flux_jacobian_h5(u, v, ww, &h, n)
}

/// [`flux_jacobian_f5`] from the velocity and the total enthalpy `h` (the density does not enter `A(u, v, w, H)`):
///
/// ```text
/// th = u.n,  phi = (g - 1) |u|^2 / 2
/// [ 0                  n_x                     n_y                     n_z                     0        ]
/// [ phi n_x - u th     th + (2 - g) u n_x      u n_y - (g - 1) v n_x   u n_z - (g - 1) w n_x   (g-1) n_x ]
/// [ phi n_y - v th     v n_x - (g - 1) u n_y   th + (2 - g) v n_y      v n_z - (g - 1) w n_y   (g-1) n_y ]
/// [ phi n_z - w th     w n_x - (g - 1) u n_z   w n_y - (g - 1) v n_z   th + (2 - g) w n_z      (g-1) n_z ]
/// [ th (phi - h)       h n_x - (g - 1) u th    h n_y - (g - 1) v th    h n_z - (g - 1) w th    g th     ]
/// ```
fn flux_jacobian_h5(u: &F, v: &F, w: &F, h: &F, n: &N3) -> Blk5 {
    let g1 = cf(G1);
    let (nx, ny, nz) = (&n[0], &n[1], &n[2]);
    let q2 = u * u + v * v + w * w;
    let th = u * nx + v * ny + w * nz;
    let phi = (0.5f32 * g1) * &q2;
    let a = E::A;
    let tg = cf(2.0 - GAMMA);
    Blk5([
        E::Z,
        a(nx.clone()),
        a(ny.clone()),
        a(nz.clone()),
        E::Z,
        a(&phi * nx - u * &th),
        a(&th + (tg * u) * nx),
        a(u * ny - (g1 * v) * nx),
        a(u * nz - (g1 * w) * nx),
        a(g1 * nx),
        a(&phi * ny - v * &th),
        a(v * nx - (g1 * u) * ny),
        a(&th + (tg * v) * ny),
        a(v * nz - (g1 * w) * ny),
        a(g1 * ny),
        a(&phi * nz - w * &th),
        a(w * nx - (g1 * u) * nz),
        a(w * ny - (g1 * v) * nz),
        a(&th + (tg * w) * nz),
        a(g1 * nz),
        a(&th * (&phi - h)),
        a(h * nx - (g1 * u) * &th),
        a(h * ny - (g1 * v) * &th),
        a(h * nz - (g1 * w) * &th),
        a(cf(GAMMA) * &th),
    ])
}

/// `|u_n| + c` of the primitive state `w` for the unit normal `n`.
fn spectral_f5(w: &V5, n: &N3) -> F {
    (&w[1] * &n[0] + &w[2] * &n[1] + &w[3] * &n[2]).abs() + (cf(GAMMA) * &w[4] / &w[0]).sqrt()
}

/// `|A|` of the Roe-averaged state of `wl`, `wr` for the unit normal `n`: `|A| = d1 (A - a I) + d2 (A - a I)(A - (a + c) I) + e1 I` with `a = u~_n`, the
/// entropy-fixed eigenvalue moduli `e1 = fix(a)`, `e2 = fix(a + c)`, `e3 = fix(a - c)` (`fix(l) = |l|` for `|l| >= eps_fix c`, else
/// `(l^2 / (eps_fix c) + eps_fix c) / 2`), `d1 = (e2 - e1) / c`, `d2 = (e2 + e3 - 2 e1) / (2 c^2)`.
fn abs_jacobian_f5(wl: &V5, wr: &V5, n: &N3, eps_fix: f32) -> Blk5 {
    let (sl, sr) = (wl[0].sqrt(), wr[0].sqrt());
    let inv = 1.0f32 / (&sl + &sr);
    let u = (&sl * &wl[1] + &sr * &wr[1]) * &inv;
    let v = (&sl * &wl[2] + &sr * &wr[2]) * &inv;
    let w = (&sl * &wl[3] + &sr * &wr[3]) * &inv;
    let ke = |q: &V5| 0.5f32 * (&q[1] * &q[1] + &q[2] * &q[2] + &q[3] * &q[3]);
    let hl = cf(GAMMA) * &wl[4] / (cf(G1) * &wl[0]) + ke(wl);
    let hr = cf(GAMMA) * &wr[4] / (cf(G1) * &wr[0]) + ke(wr);
    let h = (&sl * &hl + &sr * &hr) * &inv;
    let c = (cf(G1) * (&h - 0.5f32 * (&u * &u + &v * &v + &w * &w))).maximum(1e-30f32).sqrt();
    let a = &u * &n[0] + &v * &n[1] + &w * &n[2];
    let eps = eps_fix * &c;
    let fix = |l: &F| -> F {
        let al = l.abs();
        select(&al.ge(&eps), &al, 0.5f32 * (l * l / &eps + &eps))
    };
    let (e1, e2, e3) = (fix(&a), fix(&(&a + &c)), fix(&(&a - &c)));
    let d1 = (&e2 - &e1) / &c;
    let d2 = (&e2 + &e3 - 2.0f32 * &e1) / (2.0f32 * &c * &c);
    let am = flux_jacobian_h5(&u, &v, &w, &h, n);
    let b1 = am.sub_diag(&a);
    let b2 = am.sub_diag(&(&a + &c));
    let prod = b1.mm(&b2);
    b1.scale(&d1).add(&prod.scale(&d2)).add_diag(&e1)
}

/// Dissipation matrix of the face between the states `l` and `r`: Roe-type `|A|` blended with the scalar Rusanov `lambda I`, `lambda = max(|u_n| + c)`
/// of the two states, by the shock weight `t` (`(1 - t) |A| + t lambda I`); `roe = false`: `lambda I` alone.
fn dissipation_f5(l: &V5, r: &V5, n: &N3, p: &ImplicitParams) -> Blk5 {
    let lam = spectral_f5(l, n).maximum(spectral_f5(r, n));
    if p.roe {
        let t = shock_weight_f(&l[4], &r[4]);
        abs_jacobian_f5(l, r, n, cf(p.eps_fix)).scale(&(1.0f32 - &t)).add_diag(&(&t * &lam))
    } else {
        Blk5::zero().add_diag(&lam)
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// ghost-state Jacobians (f32 copies of body3.rs's ghost states: only the left-hand side uses them)
// ---------------------------------------------------------------------------------------------------------------------

/// `w` reflected about a wall of unit normal `n`: `v - 2 (v.n) n` (f32).
fn reflect_f3(w: &V5, n: &N3) -> V5 {
    let un = &w[1] * &n[0] + &w[2] * &n[1] + &w[3] * &n[2];
    [w[0].clone(), &w[1] - 2.0f32 * &un * &n[0], &w[2] - 2.0f32 * &un * &n[1], &w[3] - 2.0f32 * &un * &n[2], w[4].clone()]
}

/// Reflection matrix of the conserved variables about the wall normal `n`: momentum mirrored, scalars kept (`I - 2 n n^T` on rows / columns 1..3).
fn reflection_f3(n: &N3) -> Blk5 {
    let mut m = Blk5::eye();
    for a in 0..3 {
        for b in 0..3 {
            m.0[5 * (1 + a) + 1 + b] = E::A(if a == b { 1.0f32 - 2.0f32 * &n[a] * &n[b] } else { -2.0f32 * &n[a] * &n[b] });
        }
    }
    m
}

/// `T = dU_ghost / dU_interior` of the first ghost layer of a line end.
#[allow(clippy::large_enum_variant)]
enum Tj5 {
    /// Frozen ghost state (`implicit_bc = false`).
    Zero,
    Mat(Blk5),
}

/// `m T`.
fn times_t5(m: &Blk5, t: &Tj5) -> Option<Blk5> {
    match t {
        Tj5::Zero => None,
        Tj5::Mat(t) => Some(m.mm(t)),
    }
}

/// `a` extended along `dir` by `before` / `after` copies of its edge cells.
fn pad_dir(a: &F, dir: usize, before: usize, after: usize) -> F {
    match dir {
        0 => a.pad([(before, after)], PadMode::Edge),
        1 => a.pad([(0, 0), (before, after)], PadMode::Edge),
        _ => a.pad([(0, 0), (0, 0), (before, after)], PadMode::Edge),
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// geometry of the left-hand side
// ---------------------------------------------------------------------------------------------------------------------

/// The geometry of one line end (the boundary faces of one side of direction `dir`) on its edge cells (f32): the unit normal of the boundary face
/// (`raw`, for the wall reflections);  arrays of the edge slice (`[1, nj, nk]`
/// for the i ends, `[ni, 1, nk]`, `[ni, nj, 1]`).
struct EdgeGeo3 {
    raw: N3,
}

/// Uploaded f32 geometry of one direction: unit normal and area of its faces, zeros and ones of the face shape, and the masks of the cells
/// `[ni, nj, nk]` that decide how the line ends and the solid cells fold into the blocks.
struct LDir3 {
    n: N3,
    area: F,
    zero: F,
    one: F,
    /// First / last cell of its line (`m = 0`, `m = n - 1`): the ghost Jacobian folds in.
    first: Array<bool>,
    last: Array<bool>,
    /// The lower / upper neighbour along the line is solid: the wall reflection folds in (`None` without solid boxes).
    lower_solid: Option<Array<bool>>,
    upper_solid: Option<Array<bool>>,
    /// The `lo` (`up`) block is dropped: the first (last) cell of the line, or the lower (upper) neighbour is solid.
    cut_lo: Array<bool>,
    cut_up: Array<bool>,
    /// Faces with a solid cell on one side only: the state of the solid side is the reflection of the fluid side. `wall_l`: the lower cell is solid,
    /// `wall_r`: the upper one (`None` without solid boxes).
    wall_l: Option<Array<bool>>,
    wall_r: Option<Array<bool>>,
}

/// The f32 geometry of the left-hand side, uploaded once from the grid (the arrays of `Geo3` are of the precision `S`).
struct LGeo3 {
    dir: [LDir3; 3],
    /// Cell volume `V`.
    vol: F,
    /// Zeros and ones `[ni, nj, nk]`.
    zero: F,
    one: F,
    /// Solid cells (`None` without solid boxes).
    solid: Option<Array<bool>>,
    /// `A1 n1 - A0 n0` of the two k faces of every cell (the wedge's pressure source), `None` except for a wedge.
    ksrc: Option<[F; 3]>,
    /// `edge[dir][side]`, side 0 = lower end (`m = 0`), 1 = upper end.
    edge: [[EdgeGeo3; 2]; 3],
}

impl LGeo3 {
    fn new(ctx: &Context, grid: &Grid3, solid: &[bool]) -> Result<LGeo3> {
        let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
        let dims = [ni, nj, nk];
        let has_solid = solid.iter().any(|s| *s);
        let is_solid = |c: [isize; 3]| {
            c[0] >= 0
                && c[1] >= 0
                && c[2] >= 0
                && (c[0] as usize) < ni
                && (c[1] as usize) < nj
                && (c[2] as usize) < nk
                && solid[(c[2] as usize * nj + c[1] as usize) * ni + c[0] as usize]
        };
        let mask = |shape: [usize; 3], f: &dyn Fn(usize, usize, usize) -> bool| -> Result<Array<bool>> {
            let mut v = Vec::with_capacity(shape[0] * shape[1] * shape[2]);
            for k in 0..shape[2] {
                for j in 0..shape[1] {
                    for i in 0..shape[0] {
                        v.push(f(i, j, k));
                    }
                }
            }
            Array::from_slice_on(ctx, &v, shape)
        };
        let ones = |shape: [usize; 3], x: f64| up_lhs(ctx, &vec![x; shape[0] * shape[1] * shape[2]], shape);
        let direction = |dir: usize, faces: &[Face3]| -> Result<LDir3> {
            let nf = [ni + (dir == 0) as usize, nj + (dir == 1) as usize, nk + (dir == 2) as usize];
            let get = |g: &dyn Fn(&Face3) -> f64| up_lhs(ctx, &faces.iter().map(g).collect::<Vec<_>>(), nf);
            let mut e = [0isize; 3];
            e[dir] = 1;
            let at = |i: usize, j: usize, k: usize| [i, j, k][dir];
            let nl = dims[dir];
            let cell = |i: usize, j: usize, k: usize, s: isize| [i as isize + s * e[0], j as isize + s * e[1], k as isize + s * e[2]];
            let lower = |i, j, k| is_solid(cell(i, j, k, -1));
            let upper = |i, j, k| is_solid(cell(i, j, k, 1));
            // a neighbour that does not exist is not solid (the ghost state stands there): `is_solid` is false out of range
            Ok(LDir3 {
                n: [get(&|f| f.n[0])?, get(&|f| f.n[1])?, get(&|f| f.n[2])?],
                area: get(&|f| f.area)?,
                zero: ones(nf, 0.0)?,
                one: ones(nf, 1.0)?,
                first: mask(dims, &|i, j, k| at(i, j, k) == 0)?,
                last: mask(dims, &|i, j, k| at(i, j, k) + 1 == nl)?,
                lower_solid: if has_solid { Some(mask(dims, &|i, j, k| lower(i, j, k))?) } else { None },
                upper_solid: if has_solid { Some(mask(dims, &|i, j, k| upper(i, j, k))?) } else { None },
                cut_lo: mask(dims, &|i, j, k| at(i, j, k) == 0 || lower(i, j, k))?,
                cut_up: mask(dims, &|i, j, k| at(i, j, k) + 1 == nl || upper(i, j, k))?,
                wall_l: if has_solid { Some(mask(nf, &|i, j, k| lower(i, j, k) && !is_solid(cell(i, j, k, 0)))?) } else { None },
                wall_r: if has_solid { Some(mask(nf, &|i, j, k| is_solid(cell(i, j, k, 0)) && !lower(i, j, k))?) } else { None },
            })
        };
        let edge = |faces: Vec<&Face3>, shape: [usize; 3]| -> Result<EdgeGeo3> {
            let get = |q: usize, s: f64| up_lhs(ctx, &faces.iter().map(|f| s * f.n[q]).collect::<Vec<_>>(), shape);
            Ok(EdgeGeo3 { raw: [get(0, 1.0)?, get(1, 1.0)?, get(2, 1.0)?] })
        };
        // i ends `[1, nj, nk]`, j ends `[ni, 1, nk]`, k ends `[ni, nj, 1]`
        let iend = |i: usize| -> Vec<&Face3> { (0..nk * nj).map(|q| &grid.iface[q * (ni + 1) + i]).collect() };
        let jend = |j: usize| -> Vec<&Face3> { (0..nk).flat_map(|k| (0..ni).map(move |i| (k * (nj + 1) + j) * ni + i)).map(|q| &grid.jface[q]).collect() };
        let kend = |k: usize| -> Vec<&Face3> { (0..nj * ni).map(|q| &grid.kface[k * nj * ni + q]).collect() };
        Ok(LGeo3 {
            dir: [direction(0, &grid.iface)?, direction(1, &grid.jface)?, direction(2, &grid.kface)?],
            vol: up_lhs(ctx, &grid.vol, dims)?,
            zero: ones(dims, 0.0)?,
            one: ones(dims, 1.0)?,
            solid: if has_solid { Some(Array::from_slice_on(ctx, solid, dims)?) } else { None },
            ksrc: if grid.sym == crate::grid3::Symmetry::Wedge {
                let plane = ni * nj;
                let comp = |r: usize| {
                    let v: Vec<f64> = (0..ni * nj * nk)
                        .map(|c| {
                            let (a0, a1) = (&grid.kface[c], &grid.kface[c + plane]);
                            a1.area * a1.n[r] - a0.area * a0.n[r]
                        })
                        .collect();
                    up_lhs(ctx, &v, dims)
                };
                Some([comp(0)?, comp(1)?, comp(2)?])
            } else {
                None
            },
            edge: [
                [edge(iend(0), [1, nj, nk])?, edge(iend(ni), [1, nj, nk])?],
                [edge(jend(0), [ni, 1, nk])?, edge(jend(nj), [ni, 1, nk])?],
                [edge(kend(0), [ni, nj, 1])?, edge(kend(nk), [ni, nj, 1])?],
            ],
        })
    }
}

/// The blocks `lo`, `d`, `up` of `I - dt J_dir` of every cell (`[ni, nj, nk]` arrays, lazy expressions of the face matrices below), and the face matrices.
struct Factor3 {
    /// The wedge's pressure-source block of the j factor: `(a_r = dt/V (A1 n1[r] - A0 n0[r]), dp)` (see `factor3`), `None` elsewhere.
    src: Option<([F; 3], [F; 5])>,
    lo: [F; 25],
    d: [F; 25],
    up: [F; 25],
    /// The face matrices `FL`, `FR` of the direction (`[.., n + 1, ..]` arrays, evaluated) and the cell coefficients `s_l`, `s_u` they enter with.
    fl: [F; 25],
    fr: [F; 25],
    s_l: F,
    s_u: F,
    /// The states of the two end cells of the lines (`m = 0`, `m = n - 1`, `[1, ..]` slices): the ghost Jacobians of the line ends depend on them (the outflow).
    edge: [V5; 2],
}

/// Roe-averaged entropy-fixed `|u_n|, |u_n + c|, |u_n - c|` of the face states `wl`, `wr` (the eigenvalues of [`abs_jacobian_f5`]).
fn roe_eigs_f5(wl: &V5, wr: &V5, n: &N3, eps_fix: f32) -> [F; 3] {
    let (sl, sr) = (wl[0].sqrt(), wr[0].sqrt());
    let inv = 1.0f32 / (&sl + &sr);
    let u = (&sl * &wl[1] + &sr * &wr[1]) * &inv;
    let v = (&sl * &wl[2] + &sr * &wr[2]) * &inv;
    let w = (&sl * &wl[3] + &sr * &wr[3]) * &inv;
    let ke = |q: &V5| 0.5f32 * (&q[1] * &q[1] + &q[2] * &q[2] + &q[3] * &q[3]);
    let hl = cf(GAMMA) * &wl[4] / (cf(G1) * &wl[0]) + ke(wl);
    let hr = cf(GAMMA) * &wr[4] / (cf(G1) * &wr[0]) + ke(wr);
    let h = (&sl * &hl + &sr * &hr) * &inv;
    let c = (cf(G1) * (&h - 0.5f32 * (&u * &u + &v * &v + &w * &w))).maximum(1e-30f32).sqrt();
    let a = &u * &n[0] + &v * &n[1] + &w * &n[2];
    let eps = eps_fix * &c;
    let fix = |l: &F| -> F {
        let al = l.abs();
        select(&al.ge(&eps), &al, 0.5f32 * (l * l / &eps + &eps))
    };
    [fix(&a), fix(&(&a + &c)), fix(&(&a - &c))]
}

/// The eigensystem `A(w, n) = R diag(u_n, u_n, u_n, u_n + c, u_n - c) L` in conservative variables (the CPU `diag3::eig3`), as arrays over the cells; `R`
/// and `L` are applied by formula, never formed.
struct Eig {
    n: N3,
    t1: N3,
    t2: N3,
    vel: N3,
    c: F,
    k: F,
    h: F,
    th: F,
    b2: F,
    vt1: F,
    vt2: F,
}

impl Eig {
    /// From the primitive cell state `w` and the unit normal `n`; the tangents `t1 = e x n` (normalised) and `t2 = n x t1`, `e` the axis of the smallest `|n|`.
    fn new(w: &V5, n: &N3) -> Eig {
        let (a0, a1, a2) = (n[0].abs(), n[1].abs(), n[2].abs());
        // ties go to the lower axis within 1e-4, as on the CPU (`diag3::tangents`): the choice of the pair changes the scheme, so it must not depend on rounding
        let c0 = a0.le(&a1 + 1e-4f32) & a0.le(&a2 + 1e-4f32);
        let c1 = a1.le(&a2 + 1e-4f32);
        let ex = select(&c0, 1.0f32, 0.0f32);
        let ey = select(&c0, 0.0f32, select(&c1, 1.0f32, 0.0f32));
        let ez = 1.0f32 - &ex - &ey;
        let t1: N3 = [&ey * &n[2] - &ez * &n[1], &ez * &n[0] - &ex * &n[2], &ex * &n[1] - &ey * &n[0]];
        let len = (&t1[0] * &t1[0] + &t1[1] * &t1[1] + &t1[2] * &t1[2]).sqrt();
        let t1: N3 = std::array::from_fn(|q| &t1[q] / &len);
        let t2: N3 = [&n[1] * &t1[2] - &n[2] * &t1[1], &n[2] * &t1[0] - &n[0] * &t1[2], &n[0] * &t1[1] - &n[1] * &t1[0]];
        let n = n.clone();
        let vel: N3 = [w[1].clone(), w[2].clone(), w[3].clone()];
        let c = (cf(GAMMA) * &w[4] / &w[0]).sqrt();
        let k = 0.5f32 * (&vel[0] * &vel[0] + &vel[1] * &vel[1] + &vel[2] * &vel[2]);
        let h = &c * &c / cf(G1) + &k;
        let th = &vel[0] * &n[0] + &vel[1] * &n[1] + &vel[2] * &n[2];
        let b2 = cf(G1) / (&c * &c);
        let vt1 = &vel[0] * &t1[0] + &vel[1] * &t1[1] + &vel[2] * &t1[2];
        let vt2 = &vel[0] * &t2[0] + &vel[1] * &t2[1] + &vel[2] * &t2[2];
        Eig { n, t1, t2, vel, c, k, h, th, b2, vt1, vt2 }
    }

    /// `y = L x`: with `a = b1 x0 - b2 u.x + b2 x4` (`b1 = b2 k`) and `q = (n.x - u_n x0) / c`: `y = (x0 - a, t1.x - vt1 x0, t2.x - vt2 x0, (a + q) / 2, (a - q) / 2)`.
    fn cons_to_char(&self, x: &V5) -> V5 {
        let ur = &self.vel[0] * &x[1] + &self.vel[1] * &x[2] + &self.vel[2] * &x[3];
        let nr = &self.n[0] * &x[1] + &self.n[1] * &x[2] + &self.n[2] * &x[3];
        let a = &self.b2 * &self.k * &x[0] - &self.b2 * &ur + &self.b2 * &x[4];
        let q = (&nr - &self.th * &x[0]) / &self.c;
        let tx = |t: &N3, vt: &F| &t[0] * &x[1] + &t[1] * &x[2] + &t[2] * &x[3] - vt * &x[0];
        [&x[0] - &a, tx(&self.t1, &self.vt1), tx(&self.t2, &self.vt2), 0.5f32 * (&a + &q), 0.5f32 * (&a - &q)]
    }

    /// `x = R y`: with `s = y3 + y4`, `d = y3 - y4`: `x = (y0 + s, vel (y0 + s) + t1 y1 + t2 y2 + c n d, k y0 + vt1 y1 + vt2 y2 + h s + c u_n d)`.
    fn char_to_cons(&self, y: &V5) -> V5 {
        let s = &y[3] + &y[4];
        let d = &y[3] - &y[4];
        let m = &y[0] + &s;
        let cd = &self.c * &d;
        let mom = |q: usize| &self.vel[q] * &m + &self.t1[q] * &y[1] + &self.t2[q] * &y[2] + &self.n[q] * &cd;
        [m.clone(), mom(0), mom(1), mom(2), &self.k * &y[0] + &self.vt1 * &y[1] + &self.vt2 * &y[2] + &self.h * &s + &self.th * &cd]
    }
}

/// The diagonal factor of one direction: the eigensystem and the scalar coefficients of the three distinct eigenvalues (`u_n`, `u_n + c`, `u_n - c`).
struct DiagFac3 {
    eig: Eig,
    lo: [F; 3],
    d: [F; 3],
    up: [F; 3],
}

impl LazySolver3 {
    /// The f32 view of the state `a` (the left-hand side runs in f32; see the module comment).
    fn lhs_rec3(r: &Rec3) -> V5 {
        v5(|k| lhs(&r[k]))
    }

    fn dims3(&self) -> [usize; 3] {
        [self.ni, self.nj, self.nk]
    }

    /// `T = dU_ghost / dU_interior` of the first ghost layer at the `lower` / upper end of the lines of direction `dir`: the slip wall `j = 0` and the
    /// symmetry planes the reflection about the boundary face (at the axis cells the face has no area, so `lo` is zero there and `T` does not
    /// matter); the characteristic far field (the i ends and the `j = nj` end) a frozen ghost, `T = 0`, as on the CPU: the f32 finite-difference
    /// Jacobian at the kink of the characteristic branch switch destabilised the left-hand side at CFL ~ 6 (ANF M 2.48 nk 48 froze).
    /// `ImplicitParams::implicit_bc = false` freezes the wall ghosts too.
    fn ghost_jacobian3(&self, dir: usize, lower: bool, edge: &V5, p: &ImplicitParams) -> Tj5 {
        if !p.implicit_bc {
            return Tj5::Zero;
        }
        let e = &self.lg.edge[dir][!lower as usize];
        // a viscous run: the static-pressure outflow at `i = ni` (`implicit3::pressure_outflow_t`): the ghost of a subsonic face is
        // `(rho, rho u, rho v, rho w, p_b / (gamma - 1) + rho |u|^2 / 2)`, the cell itself on a supersonic face
        if dir == 0 && !lower && self.p_out.is_some() {
            let un = &edge[1] * &e.raw[0] + &edge[2] * &e.raw[1] + &edge[3] * &e.raw[2];
            let sup = un.ge((cf(GAMMA) * &edge[4] / &edge[0]).sqrt());
            let mut m = Blk5::eye();
            let q2 = &edge[1] * &edge[1] + &edge[2] * &edge[2] + &edge[3] * &edge[3];
            let (z, one) = (0.0f32 * &q2, 1.0f32 + 0.0f32 * &q2);
            m.0[24] = E::A(select(&sup, &one, &z));
            m.0[20] = E::A(select(&sup, &z, -0.5f32 * &q2));
            for a in 0..3 {
                m.0[21 + a] = E::A(select(&sup, &z, &edge[1 + a]));
            }
            return Tj5::Mat(m);
        }
        if (dir == 1 && lower) || dir == 2 {
            return Tj5::Mat(reflection_f3(&e.raw));
        }
        Tj5::Zero
    }

    /// Number of line-implicit directions: 3, or 2 without the k factor (`skip_k`, see [`LazySolver3::skip_k`]).
    fn n_dirs3(&self) -> usize {
        if self.skip_k {
            2
        } else {
            3
        }
    }

    /// Local pseudo-time step from the cell primitives `wk` (f32): `dt = cfl V / (L_i + L_j + L_k)` (`DtMode::Sum`) or `cfl V / (3 min(L_i, L_j, L_k))`
    /// (`DtMode::MinDirection`), `L_d` = half the sum of `(|u.n| + c) S` over the two faces of direction `d`. `cfl` is the one-element array of the
    /// loop state. Zero in solid cells. `no_dt_k`: `L_k` is left out (`L_i + L_j`, or `2 min(L_i, L_j)`).
    fn pseudo_dt3(&self, wk: &V5, cfl: &F, mode: DtMode, diff: Option<&[F; 3]>) -> F {
        let g = &self.lg;
        let dims = self.dims3();
        let cs = (cf(GAMMA) * &wk[4] / &wk[0]).sqrt();
        let lam = |dir: usize, upper: bool| -> F {
            let d = &g.dir[dir];
            let r = if upper { 1..dims[dir] + 1 } else { 0..dims[dir] };
            let s = |a: &F| a.slice(dir, r.clone());
            ((&wk[1] * s(&d.n[0]) + &wk[2] * s(&d.n[1]) + &wk[3] * s(&d.n[2])).abs() + &cs) * s(&d.area)
        };
        let l: Vec<F> = (0..if self.no_dt_k { 2 } else { 3 }).map(|dir| 0.5f32 * (lam(dir, false) + lam(dir, true)) + diff.map_or_else(|| 0.0f32 * &g.one, |d| d[dir].clone())).collect();
        let dt = match (mode, l.as_slice()) {
            (DtMode::Sum, [a, b, c]) => cfl * &g.vol / (a + b + c),
            (DtMode::Sum, [a, b]) => cfl * &g.vol / (a + b),
            (DtMode::MinDirection, [a, b, c]) => cfl * &g.vol / (3.0f32 * a.minimum(b).minimum(c)),
            (DtMode::MinDirection, [a, b]) => cfl * &g.vol / (2.0f32 * a.minimum(b)),
            _ => unreachable!("two or three directions"),
        };
        match &g.solid {
            Some(solid) => select(solid, 0.0f32, &dt),
            None => dt,
        }
    }

    /// The states below / above every face of direction `dir` (`[.., n + 1, ..]` arrays), from the padded primitives `wf` (two ghost layers; ghost states
    /// at the line ends); at a face of a solid box the solid side is the reflection of the fluid side.
    fn face_states3(&self, wf: &V5, dir: usize) -> (V5, V5) {
        let dg = &self.lg.dir[dir];
        let dims = self.dims3();
        let n = dims[dir];
        let line: V5 = v5(|k| (0..3).filter(|&a| a != dir).fold(wf[k].clone(), |a, ax| a.slice(ax, 2..dims[ax] + 2)));
        let (l, r): (V5, V5) = (v5(|k| line[k].slice(dir, 1..n + 2)), v5(|k| line[k].slice(dir, 2..n + 3)));
        match (&dg.wall_l, &dg.wall_r) {
            (Some(wl), Some(wr)) => {
                let (r_of_r, r_of_l) = (self.wall_ghost_f3(&r, &dg.n), self.wall_ghost_f3(&l, &dg.n));
                (v5(|k| select(wl, &r_of_r[k], &l[k])), v5(|k| select(wr, &r_of_l[k], &r[k])))
            }
            _ => (l, r),
        }
    }

    /// Blocks `(lo, d, up)` of `I - dt J_dir` for every cell (`assemble3`, `cell_blocks3`). With the face matrices `FL = A(U_L) + D`, `FR = A(U_R) - D` of
    /// every face (`D` the dissipation matrix of the face, `L` / `R` its lower / upper state, ghost states at the ends of a line, the solid side of a wall
    /// face the reflection of its fluid side, `A` evaluated with the face normal) and the cell coefficients `s_l = dt S_l / (2 V)`, `s_u = dt S_u / (2 V)`
    /// (`S` the areas of the lower / upper face):
    ///
    /// ```text
    /// lo = -s_l FL_lower,   d = I + s_u FL_upper - s_l FR_lower,   up = s_u FR_upper
    /// ```
    ///
    /// The first / last cell of a line folds `lo T` / `up T` into `d` (`T` from [`LazySolver3::ghost_jacobian3`]), a fluid cell next to a solid cell folds
    /// `lo R` / `up R` (`R` the reflection about that face), and both drop `lo` / `up`. Solid cells: `d = I`, `lo = up = 0`. `lines = false`: point-implicit.
    fn factor3(&self, wf: &V5, wk: &V5, dt: &F, p: &ImplicitParams, dir: usize, lam: Option<&[F; 3]>) -> Factor3 {
        let edge = |at: std::ops::Range<usize>| -> V5 { v5(|k| wk[k].slice(dir, at.clone())) };
        let g = &self.lg;
        let dg = &g.dir[dir];
        let dims = self.dims3();
        let n = dims[dir];

        let (l, r) = self.face_states3(wf, dir);
        let dis = dissipation_f5(&l, &r, &dg.n, p);
        // a viscous run: `2 lam_f I` on the dissipation of every face (`lam_f` the diffusive spectral radius of the face)
        let dis = match lam {
            Some(l) => dis.add_diag(&(2.0f32 * &l[dir])),
            None => dis,
        };
        let fl = flux_jacobian_f5(&l, &dg.n).add(&dis);
        let fr = flux_jacobian_f5(&r, &dg.n).sub(&dis);
        // evaluate the face matrices once: the cell blocks read each of them at two cells
        let [fl_faces, fr_faces]: [[F; 25]; 2] = [fl.dense(&dg.zero, &dg.one), fr.dense(&dg.zero, &dg.one)].cache();
        let (fl, fr) = (Blk5::of(&fl_faces), Blk5::of(&fr_faces));

        // cell blocks
        let s = 0.5f32 * dt / &g.vol;
        let (s_l, s_u) = (&s * dg.area.slice(dir, 0..n), &s * dg.area.slice(dir, 1..n + 1));
        let mut d = Blk5::eye().add(&fl.slice(dir, 1..n + 1).scale(&s_u)).sub(&fr.slice(dir, 0..n).scale(&s_l));
        // the wedge's point-implicit pressure Jacobian (`implicit3::assemble3`, `if wedge && dir == 1`; the 2D `cell_blocks` `src`): the pressure of the k faces is the
        // radial-momentum source `-(1/V) (A1 n1 - A0 n0) p`, so `d_{(r + 1) q} += dt/V (A1 n1[r] - A0 n0[r]) dp[q]`, `dp = (g - 1) (|u|^2 / 2, -u, -v, -w, 1)`
        let src = if dir == 1 && self.k_src {
            g.ksrc.as_ref().map(|ks| {
                let f = dt / &g.vol;
                let ar: [F; 3] = std::array::from_fn(|r| &f * &ks[r]);
                let (u, v, ww) = (&wk[1], &wk[2], &wk[3]);
                let dp: [F; 5] = [cf(G1) * (0.5f32 * (u * u + v * v + ww * ww)), -(cf(G1) * u), -(cf(G1) * v), -(cf(G1) * ww), cf(G1) * &g.one];
                for r in 0..3 {
                    for q in 0..5 {
                        d.0[5 * (r + 1) + q] = d.0[5 * (r + 1) + q].add(&E::A(&ar[r] * &dp[q]));
                    }
                }
                (ar, dp)
            })
        } else {
            None
        };
        let (mut lo, mut up) = if p.lines { (fl.slice(dir, 0..n).scale(&(-&s_l)), fr.slice(dir, 1..n + 1).scale(&s_u)) } else { (Blk5::zero(), Blk5::zero()) };

        if p.lines {
            // ends of the lines: fold `lo T` / `up T` into `d`
            let (t_lo, t_hi) = (self.ghost_jacobian3(dir, true, &edge(0..1), p), self.ghost_jacobian3(dir, false, &edge(n - 1..n), p));
            let folds = [times_t5(&lo.slice(dir, 0..1), &t_lo), times_t5(&up.slice(dir, n - 1..n), &t_hi)];
            let (z, o) = (g.zero.slice(dir, 0..1), g.one.slice(dir, 0..1));
            // a fluid cell next to a solid cell sees the wall: `T` = the reflection about the face towards the solid cell
            let walls = [
                dg.lower_solid.as_ref().map(|m| (m, lo.mm(&self.wall_jac_f3(&std::array::from_fn(|q| dg.n[q].slice(dir, 0..n)), &dg.one.slice(dir, 0..n))))),
                dg.upper_solid.as_ref().map(|m| (m, up.mm(&self.wall_jac_f3(&std::array::from_fn(|q| dg.n[q].slice(dir, 1..n + 1)), &dg.one.slice(dir, 1..n + 1))))),
            ];
            let present: [[bool; 25]; 2] = std::array::from_fn(|s| std::array::from_fn(|k| folds[s].as_ref().is_some_and(|b| !matches!(b.0[k], E::Z))));
            let dense: [[F; 25]; 2] = std::array::from_fn(|s| folds[s].as_ref().map_or_else(|| std::array::from_fn(|_| z.clone()), |b| b.dense(&z, &o)));
            let dense = dense.cache();
            let spread = |a: &F, low: bool| -> F {
                if low {
                    pad_dir(a, dir, 0, n - 1)
                } else {
                    pad_dir(a, dir, n - 1, 0)
                }
            };
            for k in 0..25 {
                for (s, mask) in [(0, &dg.first), (1, &dg.last)] {
                    if present[s][k] {
                        let add = select(mask, spread(&dense[s][k], s == 0), 0.0f32);
                        d.0[k] = d.0[k].add(&E::A(add));
                    }
                }
                for (m, blk) in walls.iter().flatten() {
                    if let E::A(x) = &blk.0[k] {
                        d.0[k] = d.0[k].add(&E::A(select(*m, x, 0.0f32)));
                    }
                }
                if let (E::A(a), E::A(b)) = (&lo.0[k], &up.0[k]) {
                    lo.0[k] = E::A(select(&dg.cut_lo, 0.0f32, a));
                    up.0[k] = E::A(select(&dg.cut_up, 0.0f32, b));
                }
            }
        }

        // solid cells: `d = I`, `lo = up = 0` (selects, not multiplies: nothing computed inside the solid reaches the blocks)
        if let Some(solid) = &g.solid {
            let mask = |b: &Blk5, diag: bool| -> Blk5 {
                Blk5(std::array::from_fn(|k| match &b.0[k] {
                    E::A(a) => E::A(select(solid, if diag && k % 6 == 0 { 1.0f32 } else { 0.0f32 }, a)),
                    e => e.clone(),
                }))
            };
            (lo, d, up) = (mask(&lo, false), mask(&d, true), mask(&up, false));
        }
        // the blocks stay lazy: the line solve computes them from the (evaluated) face matrices where it reads the coefficients of a cell, so the 75 block
        // arrays of a direction are never written and read back (the solves cost 7 % more, the three assembly kernels, 0.84 ms, are gone)
        let (z, o) = (&g.zero, &g.one);
        let [lo, d, up] = [lo.dense(z, o), d.dense(z, o), up.dense(z, o)];
        Factor3 { lo, d, up, fl: fl_faces, fr: fr_faces, s_l, s_u, src, edge: [edge(0..1), edge(n - 1..n)] }
    }

    /// `x <- (I - dt J_dir)^{-1} x` for every line of direction `dir` (block-tridiagonal solve, `solve_lines3`); the twenty-five block entries and the
    /// five components are passed as parts (no stacking).
    fn solve_lines3(&self, fac: &Factor3, dir: usize, x: &V5) -> V5 {
        let lo: [&F; 25] = std::array::from_fn(|k| &fac.lo[k]);
        let d: [&F; 25] = std::array::from_fn(|k| &fac.d[k]);
        let up: [&F; 25] = std::array::from_fn(|k| &fac.up[k]);
        let rhs: [&F; 5] = std::array::from_fn(|k| &x[k]);
        luma::solve_block_tridiagonal_parts(dir, lo, d, up, rhs)
    }

    /// The approximate factorization `(I - dt J_k)^{-1} (I - dt J_j)^{-1} (I - dt J_i)^{-1} rhs` (wedge mode: without the k factor).
    fn approx_factor3(&self, fac: &[Factor3], rhs: &V5) -> V5 {
        self.approx_factor3_parts(fac, rhs).2
    }

    /// [`LazySolver3::approx_factor3`] with the intermediates: `(X, Y, dU)`, `X = A_i^{-1} rhs`, `Y = A_j^{-1} X`, `dU = A_k^{-1} Y` (`A_d = I - dt J_d`);
    /// in wedge mode (`fac` has two factors) `dU = Y`.
    fn approx_factor3_parts(&self, fac: &[Factor3], rhs: &V5) -> (V5, V5, V5) {
        let x = self.solve_lines3(&fac[0], 0, rhs);
        let y = self.solve_lines3(&fac[1], 1, &x);
        let du = match fac.get(2) {
            Some(f) => self.solve_lines3(f, 2, &y),
            None => y.clone(),
        };
        (x, y, du)
    }

    /// `N_dir z = (A_dir - I) z = dt (-J_dir) z` of a vector `z` (five `[ni, nj, nk]` arrays) in flux-difference form, the same operator as the blocks of
    /// [`LazySolver3::factor3`] (`lines = true`): with the linearised numerical flux of face `f` `G_f = FL_f zl_f + FR_f zr_f` (`zl_f` / `zr_f` the vector
    /// below / above the face: the neighbours, at a line end the ghost `T z`, at a wall face the reflection of the fluid side), the cell `m` takes
    /// `s_u G_{m+1} - s_l G_m`; solid cells 0. Costs one pass over the face matrices instead of three block products.
    ///
    /// The first defect-correction pass needs only `N_i` and `N_j`: with `X = A_i^{-1} b`, `Y = A_j^{-1} X`, `dU = A_k^{-1} Y` the residual
    /// `b - (A_i + A_j + A_k - 2 I) dU` of the approximate factorisation `A_i A_j A_k` is `N_i (X - dU) + N_j (Y - dU)` (expand `A = I + N`).
    fn n_apply3(&self, fac: &Factor3, dir: usize, z: &V5, p: &ImplicitParams) -> V5 {
        let g = &self.lg;
        let dg = &g.dir[dir];
        let n = self.dims3()[dir];
        let (zero, one) = (g.zero.slice(dir, 0..1), g.one.slice(dir, 0..1));
        let end = |lower: bool| -> V5 {
            match self.ghost_jacobian3(dir, lower, &fac.edge[!lower as usize], p) {
                Tj5::Mat(t) => {
                    let e: V5 = v5(|k| if lower { z[k].slice(dir, 0..1) } else { z[k].slice(dir, n - 1..n) });
                    mv5(&t.dense(&zero, &one), &e)
                }
                Tj5::Zero => v5(|_| zero.clone()),
            }
        };
        let (lo_end, hi_end) = (end(true), end(false));
        let zl: V5 = v5(|k| luma::concat(dir, [&lo_end[k], &z[k]]));
        let zr: V5 = v5(|k| luma::concat(dir, [&z[k], &hi_end[k]]));
        let (zl, zr) = match (&dg.wall_l, &dg.wall_r) {
            (Some(wl), Some(wr)) => {
                let (r_of_r, r_of_l) = (self.wall_ghost_f3(&zr, &dg.n), self.wall_ghost_f3(&zl, &dg.n));
                (v5(|k| select(wl, &r_of_r[k], &zl[k])), v5(|k| select(wr, &r_of_l[k], &zr[k])))
            }
            _ => (zl, zr),
        };
        let (a, b) = (mv5(&fac.fl, &zl), mv5(&fac.fr, &zr));
        let gf: V5 = v5(|q| &a[q] + &b[q]).cache();
        // the wedge's pressure-source block (the diagonal of the j factor, `factor3`): rows 1..=3 get `a_r dp . z`
        let dpz = fac.src.as_ref().filter(|_| dir == 1).map(|(_, dp)| &dp[0] * &z[0] + &dp[1] * &z[1] + &dp[2] * &z[2] + &dp[3] * &z[3] + &dp[4] * &z[4]);
        v5(|q| {
            let mut nz = &fac.s_u * gf[q].slice(dir, 1..n + 1) - &fac.s_l * gf[q].slice(dir, 0..n);
            if let (Some((ar, _)), Some(dpz), 1..=3) = (fac.src.as_ref().filter(|_| dir == 1), dpz.as_ref(), q) {
                nz = nz + &ar[q - 1] * dpz;
            }
            match &g.solid {
                Some(solid) => select(solid, 0.0f32, &nz),
                None => nz,
            }
        })
    }

    /// `(I - dt J_dir) x` (`apply_factor3`): `d x + lo x_{m-1} + up x_{m+1}`, the missing neighbours at the ends of a line being 0 (their blocks are
    /// zero there).
    fn apply_factor3(&self, fac: &Factor3, dir: usize, x: &V5) -> V5 {
        let mut lower = [0isize; 3];
        lower[dir] = 1;
        let upper = lower.map(|s| -s);
        let xm: V5 = v5(|k| x[k].shift_fill(lower, 0.0));
        let xp: V5 = v5(|k| x[k].shift_fill(upper, 0.0));
        let (dx, lx, ux) = (mv5(&fac.d, x), mv5(&fac.lo, &xm), mv5(&fac.up, &xp));
        v5(|q| &dx[q] + &lx[q] + &ux[q])
    }

    /// The diagonal factor of direction `dir` (`LineSolve3::DiagonalCorrected`, the CPU `diag3::assemble_d3`): the eigensystem of `A(w_k, n_k)`
    /// with `n_k` the normalised mean of the two face normals, and the scalar coefficients of the three distinct eigenvalues `u_n`, `u_n + c`, `u_n - c`
    /// (`lo_q = -s_l FL_q(lower face)`, `d_q = 1 + s_u FL_q(upper face) - s_l FR_q(lower face)`, `up_q = s_u FR_q(upper face)`, with the face
    /// quantities `FL_q = lam_q(l) + D_q`, `FR_q = lam_q(r) - D_q` of the face states `l`, `r`), the line ends and the neighbours of solid cells frozen
    /// (`lo` / `up` dropped), solid cells the identity.
    fn diag_factor3(&self, wf: &V5, wk: &V5, dt: &F, p: &ImplicitParams, dir: usize) -> DiagFac3 {
        let g = &self.lg;
        let dg = &g.dir[dir];
        let n = self.dims3()[dir];
        let (l, r) = self.face_states3(wf, dir);
        let cs = |w: &V5| (cf(GAMMA) * &w[4] / &w[0]).sqrt();
        let th = |w: &V5| &w[1] * &dg.n[0] + &w[2] * &dg.n[1] + &w[3] * &dg.n[2];
        let (thl, cl, thr, cr) = (th(&l), cs(&l), th(&r), cs(&r));
        let lamf = spectral_f5(&l, &dg.n).maximum(spectral_f5(&r, &dg.n));
        let [d1, d2, d3]: [F; 3] = if p.roe {
            let [e1, e2, e3] = roe_eigs_f5(&l, &r, &dg.n, cf(p.eps_fix));
            let t = shock_weight_f(&l[4], &r[4]);
            let blend = |e: F| (1.0f32 - &t) * e + &t * &lamf;
            [blend(e1), blend(e2), blend(e3)]
        } else {
            [lamf.clone(), lamf.clone(), lamf.clone()]
        };
        let fl = [&thl + &d1, &thl + &cl + &d2, &thl - &cl + &d3];
        let fr = [&thr - &d1, &thr + &cr - &d2, &thr - &cr - &d3];
        // evaluated once: the cells read each face at two cells
        let [fl, fr]: [[F; 3]; 2] = [fl, fr].cache();

        let s = 0.5f32 * dt / &g.vol;
        let (s_l, s_u) = (&s * dg.area.slice(dir, 0..n), &s * dg.area.slice(dir, 1..n + 1));
        let solid = |a: F, diag: bool| -> F {
            match &g.solid {
                Some(solid) => select(solid, if diag { 1.0f32 } else { 0.0f32 }, &a),
                None => a,
            }
        };
        let lo: [F; 3] = std::array::from_fn(|q| {
            if p.lines { solid(select(&dg.cut_lo, 0.0f32, -&s_l * fl[q].slice(dir, 0..n)), false) } else { g.zero.clone() }
        });
        let up: [F; 3] = std::array::from_fn(|q| {
            if p.lines { solid(select(&dg.cut_up, 0.0f32, &s_u * fr[q].slice(dir, 1..n + 1)), false) } else { g.zero.clone() }
        });
        let d: [F; 3] = std::array::from_fn(|q| solid(1.0f32 + &s_u * fl[q].slice(dir, 1..n + 1) - &s_l * fr[q].slice(dir, 0..n), true));

        // the eigensystem of the cell: normalised mean of the face normals (the lower face normal if that is zero)
        let nl: N3 = std::array::from_fn(|q| dg.n[q].slice(dir, 0..n));
        let nm: N3 = std::array::from_fn(|q| &nl[q] + dg.n[q].slice(dir, 1..n + 1));
        let nn = (&nm[0] * &nm[0] + &nm[1] * &nm[1] + &nm[2] * &nm[2]).sqrt();
        let big = nn.gt(1e-12f32);
        let nc: N3 = std::array::from_fn(|q| select(&big, &nm[q] / &nn, &nl[q]));
        DiagFac3 { eig: Eig::new(wk, &nc), lo, d, up }
    }

    /// `x <- T diag(...)^{-1} T^{-1} x` of direction `dir`: transform to the characteristic variables, the five scalar tridiagonal systems in one call
    /// (fields 0-2 share their coefficients), transform back. Solid cells keep `x`.
    fn diag_dir3(&self, fac: &DiagFac3, dir: usize, x: &V5) -> V5 {
        let solid = &self.lg.solid;
        let pick = |keep: &V5, a: V5| -> V5 {
            match solid {
                Some(sol) => v5(|k| select(sol, &keep[k], &a[k])),
                None => a,
            }
        };
        let y = pick(x, fac.eig.cons_to_char(x));
        // evaluated before the solve: the five systems and the neighbours of a line read it several times (measured: 1.0 ms -> 0.74 ms a step)
        let y = y.cache();
        let sys = |q: usize, g: usize| (&fac.lo[g], &fac.d[g], &fac.up[g], &y[q]);
        let sol = luma::solve_tridiagonal_parts(dir, [sys(0, 0), sys(1, 0), sys(2, 0), sys(3, 1), sys(4, 2)]);
        let sol: V5 = v5(|k| sol[k].clone());
        pick(&sol, fac.eig.char_to_cons(&sol))
    }

    /// `D_2^{-1} D_1^{-1} D_0^{-1} rhs`: the diagonal factorisation (wedge mode: `D_1^{-1} D_0^{-1} rhs`).
    fn diag_af3(&self, fac: &[DiagFac3], rhs: &V5) -> V5 {
        let x = self.diag_dir3(&fac[0], 0, rhs);
        let y = self.diag_dir3(&fac[1], 1, &x);
        match fac.get(2) {
            Some(f) => self.diag_dir3(f, 2, &y),
            None => y,
        }
    }

    /// The correction of the diagonal scheme for `b = dt R` (`LineSolve3::DiagonalCorrected`, the CPU `diag3::du3`): the diagonal solve `dU0 = D^{-1} b`
    /// and one defect-correction pass against the unfactored BLOCK operator `I + N_i + N_j + N_k` (`n_apply3`, no block solves):
    /// `dU = dU0 + D^{-1} (b - (I + N_i + N_j + N_k) dU0)` (wedge mode: without `N_k` and the k factor).
    fn diag_du3(&self, wf: &V5, wk: &V5, dt: &F, p: &ImplicitParams, b: &V5) -> V5 {
        let nd = self.n_dirs3();
        let dfac: Vec<DiagFac3> = (0..nd).map(|dir| self.diag_factor3(wf, wk, dt, p, dir)).collect();
        let du0 = self.diag_af3(&dfac, b);
        let du0 = v5(|k| du0[k].clone()).cache();
        let fac: Vec<Factor3> = (0..nd).map(|dir| self.factor3(wf, wk, dt, p, dir, None)).collect();
        let nz: Vec<V5> = (0..nd).map(|dir| self.n_apply3(&fac[dir], dir, &du0, p)).collect();
        let r: V5 = v5(|k| nz.iter().fold(&b[k] - &du0[k], |acc, z| acc - &z[k]));
        let corr = self.diag_af3(&dfac, &r.cache());
        v5(|k| &du0[k] + &corr[k])
    }

    /// Positivity damping: `alpha` is the first of `1, 1/2, .., 1/128` for which `U + alpha dU` has `rho > 0`, `p > 0` and five finite components, `0` if
    /// there is none. Decided in f32 from the f32 view of `U`.
    fn damping3(&self, u: &V5, du: &V5) -> F {
        let ok = |a: f32| -> Array<bool> {
            let q: V5 = v5(|k| &u[k] + a * &du[k]);
            let w = cons_to_prim_f5(&q);
            let finite = q.iter().map(|x| x.is_finite()).reduce(|a, b| a & b).expect("the conserved state has five components");
            w[0].gt(0.0f32) & w[4].gt(0.0f32) & finite
        };
        let last = 1.0f32 / 128.0;
        let mut alpha = select(&ok(last), last, 0.0f32);
        for k in (0..7).rev() {
            let a = 0.5f32.powi(k);
            alpha = select(&ok(a), a, &alpha);
        }
        alpha
    }

    /// One implicit iteration of the loop state `s` (`implicit3::step`): the new state and the density residual of `R` at the start of the iteration, the
    /// CFL ramp advanced: `cfl <- min(growth cfl, cfl_max)`, `cfl` being `s[6]` (it starts at `cfl0` at [`LazySolver3::restart_ramp`]).
    ///
    /// ```text
    /// X = AF^{-1} (dt R),  r = dt R - (A_i + A_j + A_k - 2 I) X,  X += AF^{-1} r      (sub_iters - 1 times),   U += alpha X
    /// ```
    ///
    /// with `AF = (I - dt J_i)(I - dt J_j)(I - dt J_k)` (factored) and `A_d = I - dt J_d`. The first defect `r` is not formed from the three block
    /// products but from the intermediates of the solve, `r = N_i (X - dU) + N_j (Y - dU)` (see [`LazySolver3::n_apply3`]).
    fn implicit_step3(&self, s: &State3, p: &ImplicitParams, lim: Limiter) -> State3 {
        let u = unpack3(s);
        let mu_t = self.turb.as_ref().map(|_| s[9].clone());
        let rs = self.residual3(&u, mu_t.as_ref(), lim, false);
        let (prim, w, r0) = (&rs.p, &rs.w, &rs.res);
        let rn = self.residual_norm3(&r0[0]);

        // the f32 left-hand side: primitives with ghosts, pseudo-time step, factors
        let wf: V5 = Self::lhs_rec3(w).cache();
        let wk: V5 = v5(|k| wf[k].slice(0, 2..self.ni + 2).slice(1, 2..self.nj + 2).slice(2, 2..self.nk + 2));
        let cfl = lhs(&s[6]);
        // a viscous run: the diffusive rates of the directions in the time step and the diffusive spectral radius of every face in the dissipation
        let (diff, lam): (Option<[F; 3]>, Option<[F; 3]>) = match (&self.visc, &rs.vis) {
            (Some(vg), Some(vis)) => {
                let d = self.diffusion_rates3(&vis.cell, &vis.mu_t, &prim[0]);
                let lam = self.lambda_pass3(vg, w, &vis.cell, &vis.mu_t, None);
                (Some(std::array::from_fn(|k| lhs(&d[k]).cache())), Some(std::array::from_fn(|k| lhs(&lam[k]).cache())))
            }
            _ => (None, None),
        };
        let dt = self.pseudo_dt3(&wk, &cfl, p.dt_mode, diff.as_ref()).cache();
        let b: V5 = v5(|k| &lhs(&r0[k]) * &dt);
        let lam = lam.as_ref();
        let du = match p.line_solve {
            LineSolve3::DiagonalCorrected => self.diag_du3(&wf, &wk, &dt, p, &b),
            LineSolve3::Block => {
                let nd = self.n_dirs3();
                let fac: Vec<Factor3> = (0..nd).map(|dir| self.factor3(&wf, &wk, &dt, p, dir, lam)).collect();
                // right-hand side and defect correction
                let (x, y, mut du) = self.approx_factor3_parts(&fac, &b);
                let mut passes = 1;
                if p.lines && p.sub_iters >= 2 {
                    // the first defect, from the intermediates of the solve: `r = N_i (X - dU) + N_j (Y - dU)` (wedge mode: `Y = dU`, so `r = N_i (X - dU)`)
                    let ni_ = self.n_apply3(&fac[0], 0, &v5(|k| &x[k] - &du[k]), p);
                    let r: V5 = if nd == 3 {
                        let nj_ = self.n_apply3(&fac[1], 1, &v5(|k| &y[k] - &du[k]), p);
                        v5(|k| &ni_[k] + &nj_[k])
                    } else {
                        ni_
                    };
                    let corr = self.approx_factor3(&fac, &r.cache());
                    du = v5(|k| &du[k] + &corr[k]);
                    passes = 2;
                }
                for _ in passes..p.sub_iters.max(1) {
                    // `r = b - (A_i + A_j + A_k - 2 I) du` (two factors: `A_i + A_j - I`)
                    let a: Vec<V5> = (0..nd).map(|dir| self.apply_factor3(&fac[dir], dir, &du)).collect();
                    let r: V5 = v5(|k| &b[k] - (a.iter().fold(&du[k] * -((nd - 1) as f32), |acc, x| acc + &x[k])));
                    let corr = self.approx_factor3(&fac, &r.cache());
                    du = v5(|k| &du[k] + &corr[k]);
                }
                du
            }
        };

        // update with positivity damping
        let alpha = self.damping3(&Self::lhs_rec3(&u), &du);
        let moved = match &self.geo.fluid {
            Some(fluid) => alpha.gt(0.0f32) & fluid,
            None => alpha.gt(0.0f32),
        };
        let alpha = from_lhs(&alpha);
        let un: Rec3 = rec3(|k| select(&moved, &u[k] + &alpha * from_lhs(&du[k]), &u[k]));
        let cfl_next = (&s[6] * c(p.growth)).minimum(c(p.cfl_max));
        let turb = self.turb_step3(s, &from_lhs(&dt), &rs);
        pack3(un, rn, cfl_next, turb)
    }
}
