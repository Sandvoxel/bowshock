//! Background work: one-shot jobs on their own thread and cancellable event streams (a worker thread plus an optional beat).

use std::future::Future;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use iced::advanced::graphics::futures::MaybeSend;
use iced::futures::channel::{mpsc, oneshot};
use iced::Task;

/// Runs `work` on its own thread; resolves to its result, or `None` if the thread died without one.
pub fn offload<T: Send + 'static>(work: impl FnOnce() -> T + Send + 'static) -> impl Future<Output = Option<T>> {
    let (tx, rx) = oneshot::channel();
    std::thread::spawn(move || {
        let _ = tx.send(work());
    });
    async move { rx.await.ok() }
}

/// [`offload`] as a task: `map` gets the result, or `None` if the thread died without one.
pub fn perform<T, M>(work: impl FnOnce() -> T + Send + 'static, map: impl FnOnce(Option<T>) -> M + MaybeSend + 'static) -> Task<M>
where
    T: Send + 'static,
    M: MaybeSend + 'static,
{
    Task::perform(offload(work), map)
}

/// A running stream: its serial (stale messages are told apart by it), its cancel flag and the handle that drops it.
#[derive(Debug)]
pub struct StreamHandle {
    serial: u64,
    cancel: Arc<AtomicBool>,
    handle: iced::task::Handle,
}

impl StreamHandle {
    pub fn serial(&self) -> u64 {
        self.serial
    }

    /// Tells the work to stop through the flag it was given.
    pub fn request_cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    /// Drops the stream: no further messages are delivered.
    pub fn abort(&self) {
        self.handle.abort();
    }
}

/// Runs `work` on its own thread with a cancel flag and a sender for its events; delivers them, then the event `work` returns, as
/// `wrap(serial, event)`. With `beat`, a second thread sends `make_tick()` every `period` until the work is over.
pub fn spawn_stream<E, M>(
    serial: u64,
    beat: Option<(Duration, fn() -> E)>,
    work: impl FnOnce(Arc<AtomicBool>, mpsc::UnboundedSender<E>) -> E + Send + 'static,
    wrap: impl Fn(u64, E) -> M + Send + 'static,
) -> (Task<M>, StreamHandle)
where
    E: Send + 'static,
    M: Send + 'static,
{
    let cancel = Arc::new(AtomicBool::new(false));
    let working = Arc::new(AtomicBool::new(true));
    let (tx, rx) = mpsc::unbounded::<E>();
    if let Some((period, make_tick)) = beat {
        let (beat_tx, flag) = (tx.clone(), working.clone());
        std::thread::spawn(move || {
            while flag.load(Ordering::Relaxed) {
                std::thread::sleep(period);
                if beat_tx.unbounded_send(make_tick()).is_err() {
                    break;
                }
            }
        });
    }
    let flag = cancel.clone();
    std::thread::spawn(move || {
        let end = work(flag, tx.clone());
        working.store(false, Ordering::Relaxed);
        let _ = tx.unbounded_send(end);
    });
    let (task, handle) = Task::run(rx, move |e| wrap(serial, e)).abortable();
    (task, StreamHandle { serial, cancel, handle })
}

/// Plays `events` as `wrap(serial, event)` instead of running work (scripted demos).
pub fn spawn_iter<E, M>(serial: u64, events: Vec<E>, wrap: impl Fn(u64, E) -> M + Send + 'static) -> (Task<M>, StreamHandle)
where
    E: Send + 'static,
    M: Send + 'static,
{
    let (task, handle) = Task::run(iced::futures::stream::iter(events), move |e| wrap(serial, e)).abortable();
    (task, StreamHandle { serial, cancel: Arc::new(AtomicBool::new(false)), handle })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offload_returns_the_value_or_none_when_the_thread_panics() {
        use iced_test::futures::futures::executor::block_on;
        assert_eq!(block_on(offload(|| 7)), Some(7));
        assert_eq!(block_on(offload(|| -> u8 { panic!("worker died") })), None);
    }

    #[test]
    fn a_handle_keeps_its_serial_and_cancels_through_its_flag() {
        let (_task, handle) = spawn_iter(3, vec![1u8], |_, e| e);
        assert_eq!(handle.serial(), 3);
        assert!(!handle.cancel.load(Ordering::Relaxed));
        handle.request_cancel();
        assert!(handle.cancel.load(Ordering::Relaxed));
    }
}
