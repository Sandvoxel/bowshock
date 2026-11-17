//! The work queue that shares a solve's Mach numbers over the machines that are on.
//!
//! A solve is a list of *nodes* (one Mach number: one `run` for the drag table, one `run` at both angles for the 3D solve, so
//! a node keeps its warm-start sequence on one machine). There is one thread per machine. Each connects (in parallel with the
//! others), then takes the next node of the queue whenever it is free, so a faster machine does more of them. A machine that cannot
//! connect is skipped; a machine that fails in the middle of a node is dropped for the rest of the solve and the node goes back to
//! the queue for the others; the solve fails only when no machine is left. Cancel ends the solve at once: threads still in a call
//! (a remote's set-up can take minutes) wind down on their own and their results are dropped.
//!
//! The queue knows nothing about solving: what a machine is and how it runs a node are the two closures.

use super::remote::Failed;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

/// How often a waiting thread looks at the cancel flag.
const POLL: Duration = Duration::from_millis(50);

/// A machine that left the solve.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Lost {
    /// Its index.
    pub machine: usize,
    /// It had connected and was lost mid-solve (else it never connected).
    pub dropped: bool,
    /// Why.
    pub why: String,
}

/// How a solve of the queue ended without all its nodes.
#[derive(Debug, PartialEq, Eq)]
pub(super) enum Stop {
    /// The cancel flag was set.
    Cancelled,
    /// No machine is left; why each left.
    NoMachineLeft(Vec<Lost>),
}

/// All nodes solved.
#[derive(Debug)]
pub(super) struct Finished<R> {
    /// Each node's machine and result, in node order.
    pub results: Vec<(usize, R)>,
    /// The machines that left on the way.
    pub lost: Vec<Lost>,
}

struct State<R> {
    /// Nodes waiting for a machine, ascending.
    pending: VecDeque<usize>,
    results: Vec<Option<(usize, R)>>,
    /// Nodes without a result.
    remaining: usize,
    /// Machines that have not left.
    alive: usize,
    lost: Vec<Lost>,
}

type Shared<R> = Arc<(Mutex<State<R>>, Condvar)>;

fn lock<R>(shared: &Shared<R>) -> MutexGuard<'_, State<R>> {
    shared.0.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Solves `nodes` nodes on `machines` machines. `connect(m)` makes machine `m`'s session (blocking; it may take minutes), `run(m,
/// session, node)` solves a node on it, and `left(lost)` is told of each machine that leaves, as it does.
pub(super) fn schedule<S, R>(
    machines: usize,
    nodes: usize,
    cancel: &Arc<AtomicBool>,
    connect: Arc<dyn Fn(usize) -> Result<S, String> + Send + Sync>,
    run: Arc<dyn Fn(usize, &mut S, usize) -> Result<R, Failed> + Send + Sync>,
    left: Arc<dyn Fn(&Lost) + Send + Sync>,
) -> Result<Finished<R>, Stop>
where
    R: Send + 'static,
    S: 'static,
{
    let state = State { pending: (0..nodes).collect(), results: (0..nodes).map(|_| None).collect(), remaining: nodes, alive: machines, lost: Vec::new() };
    let shared: Shared<R> = Arc::new((Mutex::new(state), Condvar::new()));
    for m in 0..machines {
        let (shared, cancel, connect, run, left) = (shared.clone(), cancel.clone(), connect.clone(), run.clone(), left.clone());
        std::thread::spawn(move || {
            let leave = |dropped: bool, why: String, node: Option<usize>| {
                let lost = Lost { machine: m, dropped, why };
                // told before the solve can end on it, so the screen has every machine's last word
                left(&lost);
                {
                    let mut st = lock(&shared);
                    if let Some(node) = node {
                        st.pending.push_front(node);
                    }
                    st.alive -= 1;
                    st.lost.push(lost.clone());
                }
                shared.1.notify_all();
            };
            let mut session = match connect(m) {
                Ok(session) => session,
                Err(why) => return leave(false, why, None),
            };
            loop {
                let node = {
                    let mut st = lock(&shared);
                    loop {
                        if cancel.load(Ordering::Relaxed) || st.remaining == 0 {
                            return;
                        }
                        if let Some(node) = st.pending.pop_front() {
                            break node;
                        }
                        // the rest is in flight on other machines: one of them may fail and give its node back
                        st = shared.1.wait_timeout(st, POLL).unwrap_or_else(PoisonError::into_inner).0;
                    }
                };
                match run(m, &mut session, node) {
                    Ok(result) => {
                        let mut st = lock(&shared);
                        st.results[node] = Some((m, result));
                        st.remaining -= 1;
                        drop(st);
                        shared.1.notify_all();
                    }
                    Err(Failed::Cancelled) => return,
                    Err(Failed::Message(why)) => return leave(true, why, Some(node)),
                }
            }
        });
    }
    let mut st = lock(&shared);
    loop {
        if cancel.load(Ordering::Relaxed) {
            return Err(Stop::Cancelled);
        }
        if st.remaining == 0 {
            let results = st.results.iter_mut().map(|r| r.take().expect("every node has a result")).collect();
            return Ok(Finished { results, lost: std::mem::take(&mut st.lost) });
        }
        if st.alive == 0 {
            return Err(Stop::NoMachineLeft(std::mem::take(&mut st.lost)));
        }
        st = shared.1.wait_timeout(st, POLL).unwrap_or_else(PoisonError::into_inner).0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;
    use std::time::Instant;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    /// A fake fleet: machine `m` takes `speeds[m]` ms per node (`None`: cannot connect); after `fail_after[m]` nodes it fails.
    fn fleet(
        speeds: Vec<Option<u64>>,
        fail_after: Vec<Option<usize>>,
        nodes: usize,
        cancel: &Arc<AtomicBool>,
    ) -> (Result<Finished<usize>, Stop>, Arc<Mutex<Vec<Lost>>>) {
        let told = Arc::new(Mutex::new(Vec::new()));
        let log = told.clone();
        let result = schedule(
            speeds.len(),
            nodes,
            cancel,
            Arc::new(move |m| speeds[m].map(|s| (s, 0usize)).ok_or_else(|| format!("machine {m} is unreachable"))),
            Arc::new(move |m, session: &mut (u64, usize), node| {
                if fail_after[m].is_some_and(|n| session.1 >= n) {
                    return Err(Failed::Message(format!("machine {m} lost its connection")));
                }
                std::thread::sleep(ms(session.0));
                session.1 += 1;
                Ok(node * 10)
            }),
            Arc::new(move |l| log.lock().unwrap().push(l.clone())),
        );
        (result, told)
    }

    fn by_machine(done: &Finished<usize>, machines: usize) -> Vec<usize> {
        (0..machines).map(|m| done.results.iter().filter(|(r, _)| *r == m).count()).collect()
    }

    #[test]
    fn the_faster_machine_takes_more_nodes_and_the_results_come_back_in_node_order() {
        let (done, told) = fleet(vec![Some(5), Some(30)], vec![None, None], 20, &Arc::default());
        let done = done.unwrap();
        assert_eq!(done.results.iter().map(|(_, r)| *r).collect::<Vec<_>>(), (0..20).map(|n| n * 10).collect::<Vec<_>>());
        let n = by_machine(&done, 2);
        assert_eq!(n.iter().sum::<usize>(), 20);
        assert!(n[0] >= 2 * n[1] && n[1] >= 1, "5 ms against 30 ms a node: {n:?}");
        assert!(done.lost.is_empty() && told.lock().unwrap().is_empty());
    }

    #[test]
    fn a_machine_that_cannot_connect_is_skipped_and_the_others_do_everything() {
        let (done, told) = fleet(vec![None, Some(2), Some(2)], vec![None; 3], 8, &Arc::default());
        let done = done.unwrap();
        assert_eq!(by_machine(&done, 3)[0], 0);
        assert_eq!(done.lost, vec![Lost { machine: 0, dropped: false, why: "machine 0 is unreachable".into() }]);
        assert_eq!(told.lock().unwrap().len(), 1);
    }

    #[test]
    fn a_machine_lost_mid_node_gives_the_node_back_and_is_not_used_again() {
        // machine 1 does two nodes, then fails on its third: that node goes to machine 0, and machine 1 takes no more
        let (done, _) = fleet(vec![Some(10), Some(10)], vec![None, Some(2)], 12, &Arc::default());
        let done = done.unwrap();
        assert_eq!(done.results.iter().map(|(_, r)| *r).collect::<Vec<_>>(), (0..12).map(|n| n * 10).collect::<Vec<_>>());
        assert_eq!(by_machine(&done, 2)[1], 2);
        assert_eq!(done.lost, vec![Lost { machine: 1, dropped: true, why: "machine 1 lost its connection".into() }]);
    }

    #[test]
    fn the_solve_fails_only_when_no_machine_is_left() {
        let (done, _) = fleet(vec![Some(1), Some(1)], vec![Some(1), Some(2)], 10, &Arc::default());
        let Err(Stop::NoMachineLeft(lost)) = done else { panic!("both machines fail: {done:?}") };
        assert_eq!(lost.len(), 2);
        assert!(lost.iter().all(|l| l.dropped));
        let (done, _) = fleet(vec![None, None], vec![None, None], 3, &Arc::default());
        let Err(Stop::NoMachineLeft(lost)) = done else { panic!() };
        assert!(lost.iter().all(|l| !l.dropped));
    }

    #[test]
    fn a_slow_connect_does_not_hold_the_others_up_or_the_end_of_the_solve() {
        let attempts = Arc::new(AtomicUsize::new(0));
        let seen = attempts.clone();
        let t0 = Instant::now();
        let done = schedule(
            2,
            4,
            &Arc::default(),
            Arc::new(move |m| {
                if m == 1 {
                    seen.fetch_add(1, Ordering::Relaxed);
                    std::thread::sleep(ms(1500));
                }
                Ok(())
            }),
            Arc::new(|_, _: &mut (), node| {
                std::thread::sleep(ms(5));
                Ok(node)
            }),
            Arc::new(|_| {}),
        )
        .unwrap();
        assert!(t0.elapsed() < ms(1000), "the solve ended without waiting for the machine still connecting: {:?}", t0.elapsed());
        assert_eq!(done.results.iter().filter(|(m, _)| *m == 0).count(), 4);
        assert_eq!(attempts.load(Ordering::Relaxed), 1, "it was connecting all along");
    }

    #[test]
    fn cancel_ends_the_solve_at_once_whatever_the_machines_are_doing() {
        let cancel = Arc::new(AtomicBool::new(false));
        let flag = cancel.clone();
        std::thread::spawn(move || {
            std::thread::sleep(ms(80));
            flag.store(true, Ordering::Relaxed);
        });
        let t0 = Instant::now();
        let (done, _) = fleet(vec![Some(2000), Some(2000)], vec![None, None], 6, &cancel);
        assert_eq!(done.unwrap_err(), Stop::Cancelled);
        assert!(t0.elapsed() < ms(1000), "{:?}", t0.elapsed());
    }
}
