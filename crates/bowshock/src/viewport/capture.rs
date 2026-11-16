//! A one-shot high-resolution render of the model for "Export image": the pipeline draws the body pass once into an offscreen
//! texture of the requested size and reads it back (see `Pipeline::capture`), and the pixels travel to the app through a
//! one-shot channel.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use iced::futures::channel::oneshot;

use super::Mesh;

/// What a capture delivers: straight (not premultiplied) RGBA8 rows, top to bottom, or why there are none.
pub type Pixels = Result<Vec<u8>, String>;

/// A request for one render of `mesh` at `width` × `height` pixels with the viewport's camera.
#[derive(Debug)]
pub struct Capture {
    /// Image width, pixels.
    pub width: u32,
    /// Image height, pixels.
    pub height: u32,
    /// Leave the background clear (alpha 0) instead of filling it.
    pub transparent: bool,
    /// The fill behind the model when it is not transparent, straight RGBA, 0 to 1.
    pub background: [f32; 4],
    /// The model to draw: the rocket without hover, selection or flash marks.
    pub mesh: Arc<Mesh>,
    done: AtomicBool,
    tx: Mutex<Option<oneshot::Sender<Pixels>>>,
}

impl Capture {
    /// A request and the receiver its pixels arrive on.
    pub fn new(width: u32, height: u32, transparent: bool, background: [f32; 4], mesh: Arc<Mesh>) -> (Arc<Capture>, oneshot::Receiver<Pixels>) {
        let (tx, rx) = oneshot::channel();
        (Arc::new(Capture { width, height, transparent, background, mesh, done: AtomicBool::new(false), tx: Mutex::new(Some(tx)) }), rx)
    }

    /// Whether this call is the first: the primitive's `prepare` runs every frame, the render happens once.
    pub(super) fn claim(&self) -> bool {
        !self.done.swap(true, Ordering::SeqCst)
    }

    /// Whether a render was already taken.
    pub(super) fn is_done(&self) -> bool {
        self.done.load(Ordering::SeqCst)
    }

    /// Delivers the result (once).
    pub(super) fn finish(&self, result: Pixels) {
        let tx = self.tx.lock().ok().and_then(|mut tx| tx.take());
        if let Some(tx) = tx {
            let _ = tx.send(result);
        }
    }
}

/// Turns what the GPU wrote (premultiplied alpha, BGRA when `bgra`) into straight RGBA8: un-premultiplied when `transparent`,
/// else composited over `background` and opaque.
pub fn to_rgba(pixels: &mut [u8], bgra: bool, transparent: bool, background: [f32; 4]) {
    let bg = background.map(|c| c.clamp(0.0, 1.0) * 255.0);
    for px in pixels.as_chunks_mut::<4>().0 {
        if bgra {
            px.swap(0, 2);
        }
        let a = f32::from(px[3]) / 255.0;
        if transparent {
            if px[3] > 0 && px[3] < 255 {
                for c in &mut px[..3] {
                    *c = (f32::from(*c) / a).round().min(255.0) as u8;
                }
            }
        } else {
            for (c, b) in px[..3].iter_mut().zip(&bg) {
                *c = (f32::from(*c) + b * (1.0 - a)).round().min(255.0) as u8;
            }
            px[3] = 255;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transparent_pixels_are_unpremultiplied_and_swizzled() {
        // BGRA premultiplied: blue 100 at half alpha is 200 straight.
        let mut px = [100u8, 0, 0, 128, 0, 0, 0, 0];
        to_rgba(&mut px, true, true, [1.0; 4]);
        assert_eq!(&px[..4], &[0, 0, 199, 128]);
        assert_eq!(&px[4..], &[0, 0, 0, 0]);
    }

    #[test]
    fn opaque_pixels_composite_over_the_background() {
        let mut px = [0u8, 0, 0, 0, 255, 255, 255, 255];
        to_rgba(&mut px, false, false, [1.0, 0.0, 0.0, 1.0]);
        assert_eq!(&px[..4], &[255, 0, 0, 255]);
        assert_eq!(&px[4..], &[255, 255, 255, 255]);
    }

    #[test]
    fn the_capture_delivers_once() {
        let (cap, mut rx) = Capture::new(2, 2, true, [0.0; 4], Arc::new(Mesh::from_rocket(&bowshock_model::Rocket::default(), super::super::Marks::default())));
        assert!(cap.claim());
        assert!(!cap.claim());
        cap.finish(Ok(vec![1, 2, 3, 4]));
        cap.finish(Err("again".into()));
        assert_eq!(rx.try_recv().unwrap().unwrap().unwrap(), vec![1, 2, 3, 4]);
    }
}
