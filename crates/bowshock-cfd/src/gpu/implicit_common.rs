// What the 2D and 3D line-implicit array-op schemes share: `include!`d by `gpu/mod.rs` before `implicit3.rs` and `implicit.rs`, once per precision. The f32 left-hand side
// types and helpers (`F`, `cf`, `up_lhs`), the entries `E` of a block of arrays and the shock weight on f32 arrays.

use crate::implicit_params::{DtMode, ImplicitParams, LineSolve3};

/// Array of the f32 left-hand side.
type F = Array<f32>;

fn cf(x: f64) -> f32 {
    x as f32
}

/// Host f64 data (row-major, `i` fastest) as an f32 array of `shape`.
fn up_lhs(ctx: &Context, data: &[f64], shape: impl Into<luma::Shape>) -> Result<F> {
    let v: Vec<f32> = data.iter().map(|&x| x as f32).collect();
    Array::from_slice_on(ctx, &v, shape)
}

// ---------------------------------------------------------------------------------------------------------------------
// 4x4 blocks
// ---------------------------------------------------------------------------------------------------------------------

/// One entry of a block: the constant 0, the constant 1, or an array.
#[derive(Clone)]
enum E {
    Z,
    O,
    A(F),
}

impl E {
    fn add(&self, o: &E) -> E {
        match (self, o) {
            (E::Z, x) | (x, E::Z) => x.clone(),
            (E::A(a), E::A(b)) => E::A(a + b),
            (E::O, E::A(b)) | (E::A(b), E::O) => E::A(1.0f32 + b),
            (E::O, E::O) => unreachable!("1 + 1 is not an entry of any block of the scheme"),
        }
    }

    fn sub(&self, o: &E) -> E {
        match (self, o) {
            (x, E::Z) => x.clone(),
            (E::Z, E::A(b)) => E::A(-b),
            (E::A(a), E::A(b)) => E::A(a - b),
            (E::O, E::A(b)) => E::A(1.0f32 - b),
            (E::A(a), E::O) => E::A(a - 1.0f32),
            (E::Z, E::O) | (E::O, E::O) => unreachable!("-1 and 1 - 1 are not entries of any block of the scheme"),
        }
    }

    /// Every array entry times the constant `s` (no entry of the blocks it is used on is the constant 1).
    fn muls(&self, s: f32) -> E {
        match self {
            E::Z => E::Z,
            E::A(a) => E::A(a * s),
            E::O => unreachable!("a constant 1 entry times a constant is not an entry of any block of the scheme"),
        }
    }

    fn mul(&self, o: &E) -> E {
        match (self, o) {
            (E::Z, _) | (_, E::Z) => E::Z,
            (E::O, x) | (x, E::O) => x.clone(),
            (E::A(a), E::A(b)) => E::A(a * b),
        }
    }

    fn scale(&self, s: &F) -> E {
        match self {
            E::Z => E::Z,
            E::O => E::A(s.clone()),
            E::A(a) => E::A(a * s),
        }
    }
}

/// Weight of the Rusanov-like dissipation (the shock sensor of the face flux): `s = |p_R - p_L| / min(p_L, p_R)`,
/// `t = clamp((s - lo) / (hi - lo), 0, 1)`, `theta = t^2 (3 - 2 t)`.
fn shock_weight_f(pl: &F, pr: &F) -> F {
    let s = (pr - pl).abs() / pl.minimum(pr);
    let t = ((s - cf(SHOCK_SENSOR_LO)) / cf(SHOCK_SENSOR_HI - SHOCK_SENSOR_LO)).maximum(0.0f32).minimum(1.0f32);
    &t * &t * (3.0f32 - 2.0f32 * &t)
}


/// `kappa = 1 / U_r^2 - 1 / c^2` (exactly 0 for `U_r = c`).
fn kappa_f(ur2: &F, c2: &F) -> F {
    select(&ur2.ge(c2), 0.0f32, 1.0f32 / ur2 - 1.0f32 / c2)
}
