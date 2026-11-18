//! Which device a solve runs on.

/// The name of the GPU the solver runs on under [`Backend::Auto`](crate::Backend::Auto), such as `Apple M1 Max` or
/// `NVIDIA GeForce RTX 3090`; `None` when there is none (Luma's default context is the CPU backend, which the solver does not use) or
/// the crate is built without the `gpu` feature. The first call opens Luma's default context (the one the solves run on); the answer
/// is kept.
pub fn gpu_name() -> Option<String> {
    #[cfg(feature = "gpu")]
    {
        use std::sync::LazyLock;
        static NAME: LazyLock<Option<String>> = LazyLock::new(|| {
            let device = luma::default_context().ok()?.device();
            (!device.software).then_some(device.name)
        });
        NAME.clone()
    }
    #[cfg(not(feature = "gpu"))]
    {
        None
    }
}
