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
    entries: Vec<(GridKey, Arc<Grid3>, Arc<AxisMap>, Option<Arc<Vec<f64>>>)>,
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
        if let Some(i) = self.entries.iter().position(|(k, ..)| *k == key) {
            let entry = self.entries.remove(i);
            let out = (entry.1.clone(), entry.2.clone());
            self.entries.push(entry);
            self.hits += 1;
            return Ok(out);
        }
        let grid = Arc::new(bowshock_cfd::timing::timed("mesher", || body_grid3(p))?);
        let axis = Arc::new(bowshock_cfd::timing::timed("axis", || AxisMap::new(&grid)));
        self.builds += 1;
        if self.entries.len() == self.capacity {
            self.entries.remove(0);
        }
        self.entries.push((key, grid.clone(), axis.clone(), None));
        Ok((grid, axis))
    }

    /// The wall distance of every cell of the grid of `p` (`grid`, as [`GridCache::entry`] gave it): the cached one, or computed and cached with it.
    fn wall_distance(&mut self, p: &Grid3Params, grid: &Grid3) -> Arc<Vec<f64>> {
        let key = GridKey::of(p);
        let slot = self.entries.iter_mut().find(|(k, ..)| *k == key).map(|e| &mut e.3);
        match slot {
            Some(Some(d)) => d.clone(),
            Some(slot) => slot.insert(Arc::new(bowshock_cfd::timing::timed("walldist", || bowshock_cfd::solver3::grid_wall_distance(grid)))).clone(),
            None => Arc::new(bowshock_cfd::solver3::grid_wall_distance(grid)),
        }
    }

    /// The implicit CPU solver of `case` on its (cached) grid and axis map: [`Case3::build_solver`] without the mesher and the axis map.
    pub fn solver(&mut self, case: &Case3) -> Result<Solver3, Error> {
        let (grid, axis) = self.entry(&case.grid)?;
        // the exact wall distance of the SST run is a function of the grid: computed once for the cases that share it
        let dwall = case
            .viscous
            .filter(|v| !crate::engine::timing::no_fixedcost() && v.turbulence == bowshock_cfd::sst::Turbulence::Sst)
            .map(|_| self.wall_distance(&case.grid, &grid));
        let (grid, axis) = bowshock_cfd::timing::timed("grid_clone", || (Grid3::clone(&grid), AxisMap::clone(&axis)));
        Ok(case.solver_with_axis(grid, axis, dwall))
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

/// Solvers of cases that a later call will ask for, built on a background thread while the GPU iterates the current point ([`Case3::prefetch_solver`]): the host set-up of a
/// point (mesher, wall distance, the solver's arrays: 0.5-1 s of CPU at the product's grid sizes) is then off the critical path. Newest last, at most [`PREBUILT_MAX`].
static PREBUILT: Mutex<Vec<(String, std::thread::JoinHandle<Result<Solver3, Error>>)>> = Mutex::new(Vec::new());

/// Solvers built ahead and not yet asked for: each is of the order of 500 B per cell on the host.
const PREBUILT_MAX: usize = 2;

impl Case3 {
    /// What the host solver of this case is built from, as text: the grid parameters (the tier's wall spacing included), the solver's configuration (free stream, scheme, limiter),
    /// the viscous equations with their wall treatment and the jet state. The stop-rule settings, the progress and the snapshot sinks are not part of it: a seeded run of a case
    /// (whose `drift_tol` is halved) builds the solver of the plain one.
    pub fn build_key(&self) -> String {
        format!(
            "{:?}|{:?}|{:?}|{:?}|{:?}|{:?}|{:?}",
            self.body,
            self.grid,
            self.config3(),
            self.viscous,
            (self.wall_thermal, self.transition, self.compressibility, self.numerics.pressure_outflow),
            self.jet_state3(),
            self.tier
        )
    }

    /// [`Case3::build_solver`] on the grid of the process-wide cache ([`shared`]); the solver of a [`Case3::prefetch_solver`] of the same case when there is one (waited for when it
    /// is still being built).
    pub fn build_solver_cached(&self) -> Result<Solver3, Error> {
        let key = self.build_key();
        let handle = {
            let mut pre = PREBUILT.lock().unwrap_or_else(|e| e.into_inner());
            pre.iter().position(|(k, _)| *k == key).map(|i| pre.remove(i).1)
        };
        if let Some(h) = handle {
            let t = std::time::Instant::now();
            let built = h.join();
            bowshock_cfd::timing::note("prefetch_wait", t.elapsed().as_secs_f64());
            if let Ok(Ok(solver)) = built {
                return Ok(solver);
            }
        }
        self.build_solver_shared()
    }

    /// [`Case3::build_solver`] on the grid of the process-wide cache, never on a prefetched solver.
    fn build_solver_shared(&self) -> Result<Solver3, Error> {
        shared().lock().unwrap_or_else(|e| e.into_inner()).solver(self)
    }

    /// Starts building the host solver of this case on a background thread: the next [`Case3::build_solver_cached`] of the same case ([`Case3::build_key`]) takes it. A case already
    /// being built or built is not started again; beyond [`PREBUILT_MAX`] solvers the oldest is let go.
    pub fn prefetch_solver(&self) {
        let key = self.build_key();
        let mut pre = PREBUILT.lock().unwrap_or_else(|e| e.into_inner());
        if pre.iter().any(|(k, _)| *k == key) {
            return;
        }
        let mut case = self.clone();
        (case.progress, case.snapshots) = (None, None);
        let spawned = std::thread::Builder::new().name("prefetch-solver".into()).spawn(move || case.build_solver_shared());
        if let Ok(handle) = spawned {
            if pre.len() >= PREBUILT_MAX {
                pre.remove(0);
            }
            pre.push((key, handle));
        }
    }
}

/// Lets go of the solvers built ahead and not taken (the end of a solve): the threads finish, their results are dropped.
pub fn release_prebuilt() {
    PREBUILT.lock().unwrap_or_else(|e| e.into_inner()).clear();
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

    /// The wall distance of an SST run comes from the cache entry of its grid, once for the cases of one grid, and is the one the solver computes itself.
    #[test]
    fn the_cached_wall_distance_is_the_solvers_own() {
        use crate::engine::case3::WallFlow;
        let flow = WallFlow { re_c: 5.0e5, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
        let c = Case3::wedge(2.0, Case3::anf_body(), 0).with_tier(Tier::Coarse(flow));
        assert!(c.viscous.is_some());
        let own = c.build_solver().unwrap();
        let mut cache = GridCache::new(2);
        let first = cache.solver(&c).unwrap();
        let mut c2 = c.clone();
        c2.mach = 0.5;
        let second = cache.solver(&c2).unwrap();
        assert_eq!(cache.builds, 1);
        let want = &own.rans().unwrap().dwall;
        assert!(want.iter().any(|d| d.is_finite()));
        for s in [&first, &second] {
            assert_eq!(&s.rans().unwrap().dwall, want);
        }
        assert!(cache.entries[0].3.is_some(), "kept with the grid");
    }

    #[test]
    fn a_prefetched_solver_is_the_one_a_plain_build_gives_and_is_taken_once() {
        let c = tiny(11);
        let fresh = c.build_solver().unwrap();
        c.prefetch_solver();
        c.prefetch_solver();
        let key = c.build_key();
        assert_eq!(PREBUILT.lock().unwrap().iter().filter(|(k, _)| *k == key).count(), 1, "one build per case");
        let taken = c.build_solver_cached().unwrap();
        assert!(!PREBUILT.lock().unwrap().iter().any(|(k, _)| *k == key), "taken");
        assert_eq!((taken.grid.ni, taken.grid.nj, taken.grid.nk), (fresh.grid.ni, fresh.grid.nj, fresh.grid.nk));
        assert_eq!(taken.grid.vol, fresh.grid.vol);
        assert_eq!(taken.u, fresh.u);
        // another Mach number is another case: not served by it
        let mut other = tiny(11);
        other.mach = 3.0;
        assert!(other.build_key() != key);
        c.prefetch_solver();
        let own = other.build_solver_cached().unwrap();
        assert_eq!(own.u[0], other.build_solver().unwrap().u[0]);
        release_prebuilt();
        assert!(PREBUILT.lock().unwrap().is_empty());
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
