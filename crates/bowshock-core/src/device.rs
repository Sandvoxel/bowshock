//! Which device a solve runs on.

/// The name of the GPU the solver runs on under [`Backend::Auto`](crate::Backend::Auto), such as `Apple M1 Max` or
/// `NVIDIA GeForce RTX 3090`; `None` when there is none (or only a software device, which the solver does not use) or the crate is
/// built without the `gpu` feature. The first call asks the graphics driver (tens of milliseconds); the answer is kept. Luma opens
/// the same adapter (high-performance preference), so this names the device the iterations execute on.
pub fn gpu_name() -> Option<String> {
    #[cfg(feature = "gpu")]
    {
        use std::sync::LazyLock;
        static NAME: LazyLock<Option<String>> = LazyLock::new(adapter_name);
        NAME.clone()
    }
    #[cfg(not(feature = "gpu"))]
    {
        None
    }
}

#[cfg(feature = "gpu")]
fn adapter_name() -> Option<String> {
    use luma::wgpu;
    let mut desc = wgpu::InstanceDescriptor::new_without_display_handle();
    desc.backends = wgpu::Backends::PRIMARY;
    let instance = wgpu::Instance::new(desc);
    let adapter =
        block_on(instance.request_adapter(&wgpu::RequestAdapterOptions { power_preference: wgpu::PowerPreference::HighPerformance, ..Default::default() }))
            .ok()?;
    let info = adapter.get_info();
    (info.device_type != wgpu::DeviceType::Cpu).then_some(info.name)
}

/// Runs `future` to completion on the calling thread.
#[cfg(feature = "gpu")]
fn block_on<F: std::future::Future>(future: F) -> F::Output {
    use std::sync::Arc;
    use std::task::{Context, Poll, Wake, Waker};
    use std::thread::{self, Thread};

    struct Unpark(Thread);
    impl Wake for Unpark {
        fn wake(self: Arc<Self>) {
            self.0.unpark();
        }
    }
    let mut future = std::pin::pin!(future);
    let waker = Waker::from(Arc::new(Unpark(thread::current())));
    let mut cx = Context::from_waker(&waker);
    loop {
        match future.as_mut().poll(&mut cx) {
            Poll::Ready(out) => return out,
            Poll::Pending => thread::park(),
        }
    }
}
