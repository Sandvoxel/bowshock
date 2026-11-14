// Run control of the solver: the configuration of a sweep point, the reads `GpuSolver` forwards. This file is `include!`d by `lib.rs`
// after `body.rs` and `implicit.rs` into the same per-precision module, so it sees `LazySolver` with its private fields.

/// The constant ghost layers of boundary `side` (`[imin, imax, jmin, jmax]`, `shape` the two layers) for `Bc::Freestream`, else `None`.
fn fixed_ghost(ctx: &Context, cfg: &Config, side: usize, shape: [usize; 2]) -> Result<Option<Rec>> {
    if cfg.bc[side] != Bc::Freestream {
        return Ok(None);
    }
    let fs = cfg.freestream;
    let mut v = Vec::new();
    for k in 0..4 {
        v.push(up(ctx, &vec![fs[k]; shape[0] * shape[1]], shape)?);
    }
    let mut it = v.into_iter();
    Ok(Some(rec(|_| it.next().unwrap())))
}

impl LazySolver {
    pub fn config(&self) -> &Config {
        &self.cfg
    }

    /// Replaces the configuration (free stream, CFL, limiter, `first_order_iters`, boundary conditions) without touching the state or the
    /// geometry: the fixed ghost states are uploaded again, everything else is read from the configuration when the next run traces its
    /// step. Does not restart the CFL ramp of the implicit scheme ([`LazySolver::restart_ramp`]).
    pub fn set_config(&mut self, cfg: &Config) -> Result<()> {
        for side in 0..4 {
            let shape = if side < 2 { [2, self.nj] } else { [self.ni, 2] };
            self.geo.bnd[side].fixed = fixed_ghost(&self.ctx, cfg, side, shape)?;
        }
        self.cfg = cfg.clone();
        Ok(())
    }

    pub fn block(&self) -> Option<Block> {
        self.block
    }

    pub fn scheme(&self) -> Scheme {
        self.scheme
    }

    /// The conserved states of the two first cells behind the base face, `[(i_end, j), (i_end + 1, j)]` for every row
    /// `j_sting..j_end` of the solid block; only these cells are read back.
    pub fn base_cells(&self) -> Result<Vec<[[f64; 4]; 2]>> {
        let b = self.block.ok_or_else(|| luma::LumaError::Unsupported("base_cells needs a solid block".into()))?;
        let s = self.state.as_ref().expect("state");
        let cols: Vec<Vec<f64>> = (0..4).map(|k| to_f64(&s[k].slice(0, b.i_end..b.i_end + 2).slice(1, b.j_sting..b.j_end))).collect::<Result<_>>()?;
        Ok((0..b.j_end - b.j_sting).map(|r| [0, 1].map(|n| [0, 1, 2, 3].map(|k| cols[k][2 * r + n]))).collect())
    }
}
