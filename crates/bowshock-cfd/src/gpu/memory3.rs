// The device memory ledger of `LazySolver3` (included into the precision modules like `body3.rs`): the bytes of the persistent arrays by group, from the element
// count and the element size of every array the solver holds (a `bool` array is stored as `u32`, four bytes). The loop's transients (the face fluxes, gradients,
// blocks and line-solve scratch the compiled step materialises) are not arrays the solver holds: the peak of a process minus [`LazySolver3::memory_report`] is them.

/// Device bytes of what a value holds.
trait DevBytes {
    fn dev_bytes(&self) -> usize;
}

impl<T: luma::GpuType> DevBytes for Array<T> {
    fn dev_bytes(&self) -> usize {
        self.len() * match std::mem::size_of::<T>() {
            1 => 4,
            n => n,
        }
    }
}

impl<T: DevBytes> DevBytes for Option<T> {
    fn dev_bytes(&self) -> usize {
        self.as_ref().map_or(0, DevBytes::dev_bytes)
    }
}

impl<T: DevBytes> DevBytes for Vec<T> {
    fn dev_bytes(&self) -> usize {
        self.iter().map(DevBytes::dev_bytes).sum()
    }
}

impl<T: DevBytes, const N: usize> DevBytes for [T; N] {
    fn dev_bytes(&self) -> usize {
        self.iter().map(DevBytes::dev_bytes).sum()
    }
}

/// `impl DevBytes for $t` as the sum over the listed fields.
macro_rules! dev_bytes_of {
    ($t:ty; $($f:ident),+) => {
        impl DevBytes for $t {
            fn dev_bytes(&self) -> usize {
                0 $(+ self.$f.dev_bytes())+
            }
        }
    };
}

dev_bytes_of!(FaceGeo3; n, area, rot);
dev_bytes_of!(SolidFaces3; z, ma, mb, mc, md, m2s, m1s, ja, jb);
dev_bytes_of!(AxisGeo3; idx, w, m);
dev_bytes_of!(LDir3; n, area, zero, one, first, last, lower_solid, upper_solid, cut_lo, cut_up, wall_l, wall_r);
dev_bytes_of!(EdgeGeo3; raw);
dev_bytes_of!(LGeo3; dir, vol, zero, one, solid, ksrc, edge);
dev_bytes_of!(VFace3; e, inv_d, flux, fc, noslip, ghost, wall);
dev_bytes_of!(VGeo3; dir);
dev_bytes_of!(TFam3; inf, sym, coupled, follows);
dev_bytes_of!(TGeo3; dir, fluid, wall_idx, wall_pos, wall_cell, wall_ns, wall_e, wall_counted, inv_v, vol);
dev_bytes_of!(TurbRun3; tg, dwall, ks);
dev_bytes_of!(TractionFam3; idx, wa, cx, class);
dev_bytes_of!(TractionGeo3; fam);
dev_bytes_of!(ForceGeo3; ia, ib, k, cx, cz, cm, class, cut_class);
dev_bytes_of!(MonitorFam3; idx, w, wm);
dev_bytes_of!(MonitorGeo3; ia, ib, k, wx, wz, wm, fam);
dev_bytes_of!(JetGpu; mask, behind);
dev_bytes_of!(AxisDt3; mask, cap, cap_f);

impl LazySolver3 {
    /// The device bytes of the arrays the solver holds, by group (the order is the order of the groups in memory): `(group, bytes)`. The solver's own transients are
    /// not in it (see the top of this file); the loop state is counted as one set of arrays (the compiled loop keeps a second one).
    pub fn memory_report(&self) -> Vec<(&'static str, usize)> {
        let g = &self.geo;
        let slip: usize = g.slip.iter().map(|(_, _, n)| n.dev_bytes()).sum();
        vec![
            ("state: U, residual norm, CFL, k, omega, mu_t, force monitor (one set)", self.state.as_ref().map_or(0, |s| s.dev_bytes())),
            ("Geo3 faces: normals, areas, azimuth of the three families", g.f.dev_bytes()),
            ("Geo3 cell metrics: V, 1/V, fluid mask", g.inv_v.dev_bytes() + g.vol.dev_bytes() + g.fluid.dev_bytes()),
            (
                "Geo3 boundaries: far-field normals, slip walls, axis maps, symmetry planes",
                g.imin.dev_bytes() + g.imax.dev_bytes() + g.jmax.dev_bytes() + slip + g.axis.dev_bytes() + g.ksym.dev_bytes(),
            ),
            ("Geo3 solid-face masks (SolidFaces3)", g.solid_faces.dev_bytes()),
            ("LGeo3: f32 geometry of the implicit left-hand side", self.lg.dev_bytes()),
            ("VGeo3: viscous face data", self.visc.dev_bytes()),
            ("TurbRun3: SST wall data, wall distance, roughness", self.turb.dev_bytes()),
            ("forces: wall faces, traction faces, force monitor", self.forces.dev_bytes() + self.traction.dev_bytes() + self.monitor.dev_bytes()),
            ("jet masks, axis-cell masks, residual-norm mask", self.jet.dev_bytes() + self.norm.dev_bytes() + self.axis_dt.dev_bytes()),
        ]
    }
}
