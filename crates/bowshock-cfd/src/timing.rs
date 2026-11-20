//! Named wall-clock notes of the set-up work of a solver (instrumentation only, no numerics): the host grid / solver build, the wall distance, the geometry upload.
//! [`note`] adds seconds to a name on the calling thread; [`take`] drains them. The GPU runner of `bowshock-core` joins them to the point's timing (`BOWSHOCK_TIMING`).

use std::cell::RefCell;
use std::time::Instant;

thread_local! {
    static NOTES: RefCell<Vec<(&'static str, f64)>> = const { RefCell::new(Vec::new()) };
}

/// Adds `secs` to the entry `name` of this thread's notes.
pub fn note(name: &'static str, secs: f64) {
    NOTES.with(|n| {
        let mut n = n.borrow_mut();
        match n.iter_mut().find(|(k, _)| *k == name) {
            Some((_, v)) => *v += secs,
            None => n.push((name, secs)),
        }
    });
}

/// Runs `f` and notes its duration under `name`.
pub fn timed<T>(name: &'static str, f: impl FnOnce() -> T) -> T {
    let t = Instant::now();
    let out = f();
    note(name, t.elapsed().as_secs_f64());
    out
}

/// The notes of this thread, in the order of their first use; the thread's list is emptied.
pub fn take() -> Vec<(&'static str, f64)> {
    NOTES.with(|n| std::mem::take(&mut *n.borrow_mut()))
}
