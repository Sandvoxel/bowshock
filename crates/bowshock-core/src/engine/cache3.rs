//! Grid cache for per-call reuse (DESIGN3D E): the 3D grid of a case is a function of its [`Grid3Params`] alone (the body and the meridian grid, the fins,
//! the domain, the azimuthal resolution, and through the tier the first cell of the walls, the fin stack and the roughness, so the Mach number the
//! wall-resolved grid is sized for). A Mach table, a polar and the repeated calls of one session solve many cases on the same grid; it is built once.
//!
//! What is shared is the finished [`Grid3`] (the mesher and the geometry of every face). The solver's per-grid tables (the axis ghost map, the solid and
//! wall flags, the face rotations) are rebuilt by [`Solver3::new`] from a copy of the grid, which costs a fraction of the mesher; the wall distance joins the
//! entry when the turbulence model's distance function is a grid product. A GPU runner takes its host solver from [`GridCache::solver`] (or
//! [`Case3::build_solver_cached`]) in place of `Case3::build_solver`.

use crate::engine::case3::Case3;
use crate::engine::grid3::{body_grid3, Grid3Params};
use crate::error::Error;
use bowshock_cfd::axis3::AxisMap;
use bowshock_cfd::grid3::Grid3;
use bowshock_cfd::solver3::Solver3;
use std::sync::{Arc, LazyLock, Mutex};

/// What decides a grid: the exact text of its parameters (`Debug` prints every `f64` with the shortest digits that read back to the same number, so equal
/// keys are equal parameters).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GridKey(String);

impl GridKey {
    pub fn of(p: &Grid3Params) -> GridKey {
        GridKey(format!("{p:?}"))
    }
}

/// The most recently used grids, newest last.
pub struct GridCache {
    capacity: usize,
    entries: Vec<(GridKey, Arc<Grid3>, Arc<AxisMap>)>,
    /// Lookups answered from the cache.
    pub hits: usize,
    /// Grids built (lookups that missed).
    pub builds: usize,
}

impl GridCache {
    /// A cache of `capacity` grids (at least one).
    pub fn new(capacity: usize) -> GridCache {
        GridCache { capacity: capacity.max(1), entries: Vec::new(), hits: 0, builds: 0 }
    }

    /// The grid of `p`: the cached one, or built and cached (the least recently used grid goes when the cache is full).
    pub fn grid(&mut self, p: &Grid3Params) -> Result<Arc<Grid3>, Error> {
        Ok(self.entry(p)?.0)
    }

    /// The grid of `p` and its axis ghost map (`Solver3::with_axis`), cached together.
    pub fn entry(&mut self, p: &Grid3Params) -> Result<(Arc<Grid3>, Arc<AxisMap>), Error> {
        let key = GridKey::of(p);
        if let Some(i) = self.entries.iter().position(|(k, _, _)| *k == key) {
            let entry = self.entries.remove(i);
            let out = (entry.1.clone(), entry.2.clone());
            self.entries.push(entry);
            self.hits += 1;
            return Ok(out);
        }
        let grid = Arc::new(body_grid3(p)?);
        let axis = Arc::new(AxisMap::new(&grid));
        self.builds += 1;
        if self.entries.len() == self.capacity {
            self.entries.remove(0);
        }
        self.entries.push((key, grid.clone(), axis.clone()));
        Ok((grid, axis))
    }

    /// The implicit CPU solver of `case` on its (cached) grid and axis map: [`Case3::build_solver`] without the mesher and the axis map.
    pub fn solver(&mut self, case: &Case3) -> Result<Solver3, Error> {
        let (grid, axis) = self.entry(&case.grid)?;
        Ok(case.solver_with_axis(Grid3::clone(&grid), AxisMap::clone(&axis)))
    }

    /// Number of grids held.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

static CACHE: LazyLock<Mutex<GridCache>> = LazyLock::new(|| Mutex::new(GridCache::new(3)));

/// The process-wide cache (three grids: a finned product grid is of the order of 100 MB).
pub fn shared() -> &'static Mutex<GridCache> {
    &CACHE
}

impl Case3 {
    /// [`Case3::build_solver`] on the grid of the process-wide cache ([`shared`]).
    pub fn build_solver_cached(&self) -> Result<Solver3, Error> {
        shared().lock().unwrap_or_else(|e| e.into_inner()).solver(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::case3::Tier;

    fn tiny(nk: usize) -> Case3 {
        let mut c = Case3::tiny_anf(2.0, 0.0);
        c.grid.nk = nk;
        c
    }

    #[test]
    fn the_same_parameters_share_one_grid_and_any_other_build_another() {
        let mut cache = GridCache::new(4);
        let a = cache.grid(&tiny(8).grid).unwrap();
        let b = cache.grid(&tiny(8).grid).unwrap();
        assert!(Arc::ptr_eq(&a, &b));
        assert_eq!((cache.builds, cache.hits), (1, 1));
        // the Mach number of the case does not enter (the grid is sized for the tier's), the azimuthal resolution does
        let mut c = tiny(8);
        c.mach = 0.8;
        assert!(Arc::ptr_eq(&a, &cache.grid(&c.grid).unwrap()));
        let other = cache.grid(&tiny(9).grid).unwrap();
        assert!(!Arc::ptr_eq(&a, &other));
        assert_eq!(other.nk, 9);
        assert_eq!(cache.len(), 2);
    }

    #[test]
    fn the_grid_mach_and_the_domain_are_part_of_the_key() {
        use crate::engine::case3::WallFlow;
        let flow = |mach_max| WallFlow { re_c: 1e6, mach_max, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
        let wedge = |tier| Case3::wedge(2.0, Case3::anf_body(), 0).with_tier(tier);
        let (c1, c2, c3) = (wedge(Tier::Coarse(flow(2.0))), wedge(Tier::Coarse(flow(3.0))), wedge(Tier::Euler));
        let keys: Vec<GridKey> = [&c1, &c2, &c3].iter().map(|c| GridKey::of(&c.grid)).collect();
        assert!(keys[0] != keys[1] && keys[0] != keys[2] && keys[1] != keys[2]);
        let half = Case3::base_half(2.0, 0.0, Case3::anf_body(), None, Tier::Euler, 0);
        assert!(GridKey::of(&half.grid) != keys[2], "wedge and half domain differ");
        assert_eq!(GridKey::of(&Case3::wedge(2.0, Case3::anf_body(), 0).grid), keys[2]);
    }

    #[test]
    fn the_least_recently_used_grid_goes_first() {
        let mut cache = GridCache::new(2);
        let a = cache.grid(&tiny(8).grid).unwrap();
        cache.grid(&tiny(9).grid).unwrap();
        cache.grid(&tiny(8).grid).unwrap(); // refreshes the first
        cache.grid(&tiny(10).grid).unwrap(); // evicts the second
        assert_eq!(cache.builds, 3);
        assert!(Arc::ptr_eq(&a, &cache.grid(&tiny(8).grid).unwrap()), "the first stayed");
        cache.grid(&tiny(9).grid).unwrap();
        assert_eq!(cache.builds, 4, "the second was evicted and is built again");
    }

    #[test]
    fn a_cached_solver_is_the_uncached_one() {
        let mut cache = GridCache::new(2);
        let c = tiny(8);
        let fresh = c.build_solver().unwrap();
        let first = cache.solver(&c).unwrap();
        let again = cache.solver(&c).unwrap();
        assert_eq!(cache.builds, 1);
        for s in [&first, &again] {
            assert_eq!((s.grid.ni, s.grid.nj, s.grid.nk), (fresh.grid.ni, fresh.grid.nj, fresh.grid.nk));
            assert_eq!(s.grid.vol, fresh.grid.vol);
            assert_eq!(s.grid.part, fresh.grid.part);
            assert_eq!(s.u, fresh.u);
        }
    }

    /// What the cache saves on the product grid (ANF sector, wall-resolved `Coarse` tier, base-wake grid): the solver from the mesher against the solver from a
    /// cached grid. Prints the times (run with `--run-ignored only --no-capture`).
    #[test]
    #[ignore = "timing: prints the cost of the mesher against a cached grid"]
    fn a_cached_product_grid_saves_the_mesher() {
        use crate::engine::case3::WallFlow;
        use std::time::Instant;
        let flow = WallFlow { re_c: 5.0e5, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
        let c = Case3::sector(2.0, Case3::anf_body(), Case3::anf_fins(), Tier::Coarse(flow), 0).euler_equations();
        let mut cache = GridCache::new(1);
        let t = Instant::now();
        let a = cache.solver(&c).unwrap();
        let t_first = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let b = cache.solver(&c).unwrap();
        let t_second = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let grid = cache.grid(&c.grid).unwrap();
        let t_grid = t.elapsed().as_secs_f64();
        println!(
            "{}x{}x{} cells: first solver (mesher + solver) {t_first:.3} s, second (cached grid copy + solver) {t_second:.3} s, grid lookup {t_grid:.2e} s",
            grid.ni, grid.nj, grid.nk
        );
        assert_eq!(a.grid.vol, b.grid.vol);
        assert_eq!((cache.builds, cache.hits), (1, 2));
    }
}
