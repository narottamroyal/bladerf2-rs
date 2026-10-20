//! Synchronous/asynchronous bridge.
//!
//! Protocol logic is written once as `async` code. [`operation`] wraps a
//! future so it implements [`MaybeFuture`]: `.wait()` blocks the current
//! thread (native only, no runtime needed); `.await` runs it asynchronously.
//!
//! While a future is being driven by `.wait()`, a thread-local flag is set.
//! [`usb`], [`sleep`] and [`with_timeout`] consult it so that nested nusb
//! operations, sleeps and timeouts take their blocking path instead of
//! requiring an async runtime.

use std::future::Future;
use std::time::Duration;

use nusb::MaybeFuture;

use crate::{Error, Result};

/// `Send` on native targets, a no-op bound on wasm (where futures hold
/// non-`Send` JS handles).
#[cfg(not(target_arch = "wasm32"))]
pub(crate) use std::marker::Send as NonWasmSend;
#[cfg(target_arch = "wasm32")]
pub(crate) trait NonWasmSend {}
#[cfg(target_arch = "wasm32")]
impl<T> NonWasmSend for T {}

/// Wrap a future so it implements [`MaybeFuture`].
#[must_use = "operations are lazy; call .wait() or .await"]
pub(crate) fn operation<F>(future: F) -> impl MaybeFuture<Output = F::Output>
where
    F: Future + NonWasmSend,
{
    Operation(future)
}

struct Operation<F>(F);

impl<F: Future + NonWasmSend> std::future::IntoFuture for Operation<F> {
    type Output = F::Output;
    type IntoFuture = F;

    fn into_future(self) -> F {
        self.0
    }
}

impl<F: Future + NonWasmSend> MaybeFuture for Operation<F> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> F::Output {
        native::block_on(self.0)
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub(crate) use native::{block_on, is_blocking};

/// Dispatch a nusb `MaybeFuture` (control transfer, `set_alt_setting`, ...)
/// to the blocking or async path.
///
/// nusb's `.await` on operations backed by blocking syscalls **panics** when
/// neither the `tokio` nor the `smol` feature is enabled, so outside a
/// `.wait()` context those features are required to take the async path.
pub(crate) async fn usb<M: MaybeFuture>(op: M) -> M::Output {
    #[cfg(not(target_arch = "wasm32"))]
    if is_blocking() || !cfg!(any(feature = "smol", feature = "tokio")) {
        return op.wait();
    }
    op.await
}

/// Async sleep that also works inside `.wait()`.
pub(crate) async fn sleep(duration: Duration) {
    #[cfg(not(target_arch = "wasm32"))]
    if is_blocking() {
        std::thread::sleep(duration);
        return;
    }
    futures_timer::Delay::new(duration).await;
}

/// Await `future`, failing with [`Error::Timeout`] after `timeout`
/// (`None` waits forever).
pub(crate) async fn with_timeout<T>(
    future: impl Future<Output = Result<T>>,
    timeout: Option<Duration>,
) -> Result<T> {
    let Some(duration) = timeout else {
        return future.await;
    };

    #[cfg(not(target_arch = "wasm32"))]
    if is_blocking() {
        return native::block_on_until(future, Some(duration)).unwrap_or(Err(Error::Timeout));
    }

    let timeout = async {
        sleep(duration).await;
        Err(Error::Timeout)
    };
    futures_lite::future::race(future, timeout).await
}

/// Thread-parking executor used by `.wait()`.
#[cfg(not(target_arch = "wasm32"))]
mod native {
    use std::cell::Cell;
    use std::future::Future;
    use std::sync::Arc;
    use std::task::{Context, Poll, Wake, Waker};
    use std::thread::{self, Thread};
    use std::time::{Duration, Instant};

    thread_local! {
        static BLOCKING: Cell<bool> = const { Cell::new(false) };
    }

    /// Returns `true` while the current thread is inside a `.wait()` call.
    pub(crate) fn is_blocking() -> bool {
        BLOCKING.get()
    }

    struct ThreadWaker(Thread);

    impl Wake for ThreadWaker {
        fn wake(self: Arc<Self>) {
            self.0.unpark();
        }
    }

    /// Restores the previous blocking flag, even if the poll panics.
    struct RestoreFlag(bool);

    impl Drop for RestoreFlag {
        fn drop(&mut self) {
            BLOCKING.set(self.0);
        }
    }

    /// Run a future to completion on the current thread, parking while it is
    /// pending.
    pub(crate) fn block_on<F: Future>(future: F) -> F::Output {
        block_on_until(future, None).expect("block_on without a deadline always completes")
    }

    /// Like [`block_on`], but gives up and returns `None` after `timeout`.
    pub(super) fn block_on_until<F: Future>(
        future: F,
        timeout: Option<Duration>,
    ) -> Option<F::Output> {
        let deadline = timeout.map(|timeout| Instant::now() + timeout);
        let waker = Waker::from(Arc::new(ThreadWaker(thread::current())));
        let mut cx = Context::from_waker(&waker);
        let mut future = std::pin::pin!(future);

        loop {
            let poll = {
                let _restore = RestoreFlag(BLOCKING.replace(true));
                future.as_mut().poll(&mut cx)
            };
            if let Poll::Ready(output) = poll {
                return Some(output);
            }
            match deadline {
                None => thread::park(),
                Some(deadline) => {
                    thread::park_timeout(deadline.checked_duration_since(Instant::now())?);
                }
            }
        }
    }
}
