//! The application icon files: `bowshock.ico` (Windows) and `bowshock.icns` (macOS), generated from [`logo::icon_rgba`] so the mark
//! has one source of truth. They are checked in under `assets/icons/` and regenerated with
//!
//! ```text
//! cargo run -p bowshock -- icons crates/bowshock/assets/icons
//! ```
//!
//! * `build.rs` wraps the `.ico` into a Windows resource and links it into the `.exe` (Explorer, task bar, Alt-Tab).
//! * `scripts/bundle-macos.sh` puts the `.icns` into `Bowshock.app` (Finder, Dock, Cmd-Tab).
//! * [`png`] also feeds the Dock icon of an un-bundled run (`platform::set_dock_icon`).

use std::io;
use std::path::Path;

use crate::logo::icon_rgba;

/// Edge lengths in the `.ico`: the Windows shell asks for 16/24/32/48 (task bar, lists, DPI steps), 64/128 and 256 (large views).
pub const ICO_SIZES: [u32; 7] = [16, 24, 32, 48, 64, 128, 256];

/// From this edge length up an `.ico` image is stored as PNG (Windows Vista and later); smaller ones as a classic 32-bit DIB.
const ICO_PNG_FROM: u32 = 64;

/// The `.icns` images: the type code and the pixel size. `ic11`..`ic14` are the `@2x` forms of 16, 32, 128 and 256 pt.
pub const ICNS_TYPES: [(&[u8; 4], u32); 11] = [
    (b"icp4", 16),
    (b"icp5", 32),
    (b"icp6", 64),
    (b"ic07", 128),
    (b"ic08", 256),
    (b"ic09", 512),
    (b"ic10", 1024),
    (b"ic11", 32),
    (b"ic12", 64),
    (b"ic13", 256),
    (b"ic14", 512),
];

/// The icon at `px` pixels square as a PNG file.
pub fn png(px: u32) -> Vec<u8> {
    let mut out = Vec::new();
    let mut enc = png::Encoder::new(&mut out, px, px);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header().and_then(|mut w| w.write_image_data(&icon_rgba(px))).expect("encoding into memory cannot fail");
    out
}

/// A classic ICO image: a `BITMAPINFOHEADER` (height doubled for the AND mask), bottom-up BGRA rows, then an all-zero 1 bpp mask
/// (the alpha channel carries the transparency).
fn dib(px: u32) -> Vec<u8> {
    let rgba = icon_rgba(px);
    let mask_row = px.div_ceil(32) * 4;
    let mut out = Vec::with_capacity(40 + (px * px * 4 + mask_row * px) as usize);
    for v in [40u32, px, px * 2] {
        out.extend_from_slice(&v.to_le_bytes());
    }
    out.extend_from_slice(&1u16.to_le_bytes()); // planes
    out.extend_from_slice(&32u16.to_le_bytes()); // bits per pixel
    out.extend_from_slice(&0u32.to_le_bytes()); // BI_RGB
    out.extend_from_slice(&(px * px * 4 + mask_row * px).to_le_bytes()); // image size
    out.extend_from_slice(&[0; 16]); // resolution, palette
    for row in rgba.chunks_exact(px as usize * 4).rev() {
        for p in row.as_chunks::<4>().0 {
            out.extend_from_slice(&[p[2], p[1], p[0], p[3]]);
        }
    }
    out.resize(out.len() + (mask_row * px) as usize, 0);
    out
}

/// The `.ico` file at [`ICO_SIZES`].
pub fn ico() -> Vec<u8> {
    let images: Vec<(u32, Vec<u8>)> = ICO_SIZES.iter().map(|&px| (px, if px >= ICO_PNG_FROM { png(px) } else { dib(px) })).collect();
    let mut out = Vec::new();
    out.extend_from_slice(&[0, 0, 1, 0]); // reserved, type 1 = icon
    out.extend_from_slice(&(images.len() as u16).to_le_bytes());
    let mut offset = 6 + 16 * images.len() as u32;
    for (px, data) in &images {
        out.push(*px as u8); // 256 is stored as 0
        out.push(*px as u8);
        out.extend_from_slice(&[0, 0]); // palette size, reserved
        out.extend_from_slice(&1u16.to_le_bytes()); // planes
        out.extend_from_slice(&32u16.to_le_bytes()); // bits per pixel
        out.extend_from_slice(&(data.len() as u32).to_le_bytes());
        out.extend_from_slice(&offset.to_le_bytes());
        offset += data.len() as u32;
    }
    for (_, data) in &images {
        out.extend_from_slice(data);
    }
    out
}

/// The `.icns` file: every [`ICNS_TYPES`] image as PNG (16 to 1024 pixels, including the `@2x` forms).
pub fn icns() -> Vec<u8> {
    let mut body = Vec::new();
    for (code, px) in ICNS_TYPES {
        let data = png(px);
        body.extend_from_slice(code);
        body.extend_from_slice(&(data.len() as u32 + 8).to_be_bytes());
        body.extend_from_slice(&data);
    }
    let mut out = Vec::with_capacity(body.len() + 8);
    out.extend_from_slice(b"icns");
    out.extend_from_slice(&(body.len() as u32 + 8).to_be_bytes());
    out.extend_from_slice(&body);
    out
}

/// Writes `bowshock.ico` and `bowshock.icns` into `dir` (created if missing).
pub fn write_all(dir: &Path) -> io::Result<()> {
    std::fs::create_dir_all(dir)?;
    std::fs::write(dir.join("bowshock.ico"), ico())?;
    std::fs::write(dir.join("bowshock.icns"), icns())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode(bytes: &[u8]) -> (u32, Vec<u8>) {
        let mut reader = png::Decoder::new(io::Cursor::new(bytes)).read_info().unwrap();
        let mut buf = vec![0; reader.output_buffer_size().unwrap()];
        let info = reader.next_frame(&mut buf).unwrap();
        assert_eq!((info.color_type, info.bit_depth), (png::ColorType::Rgba, png::BitDepth::Eight));
        assert_eq!(info.width, info.height);
        (info.width, buf)
    }

    /// The (edge, offset, length) of every image in an `.ico`.
    fn ico_entries(bytes: &[u8]) -> Vec<(u32, usize, usize)> {
        let u32_at = |i: usize| u32::from_le_bytes(bytes[i..i + 4].try_into().unwrap()) as usize;
        assert_eq!(&bytes[..4], &[0, 0, 1, 0]);
        let count = u16::from_le_bytes([bytes[4], bytes[5]]) as usize;
        (0..count)
            .map(|i| {
                let e = 6 + 16 * i;
                let edge = if bytes[e] == 0 { 256 } else { bytes[e] as u32 };
                (edge, u32_at(e + 12), u32_at(e + 8))
            })
            .collect()
    }

    /// The images of an `.icns` as (type code, pixel edge).
    fn icns_entries(bytes: &[u8]) -> Vec<([u8; 4], u32)> {
        assert_eq!(&bytes[..4], b"icns");
        assert_eq!(u32::from_be_bytes(bytes[4..8].try_into().unwrap()) as usize, bytes.len());
        let mut at = 8;
        let mut out = Vec::new();
        while at < bytes.len() {
            let len = u32::from_be_bytes(bytes[at + 4..at + 8].try_into().unwrap()) as usize;
            out.push((bytes[at..at + 4].try_into().unwrap(), decode(&bytes[at + 8..at + len]).0));
            at += len;
        }
        assert_eq!(at, bytes.len());
        out
    }

    /// Every channel within `tol` of the master raster (the checked-in files were written on another CPU).
    fn matches_master(px: u32, rgba: &[u8], tol: i16) -> bool {
        icon_rgba(px).iter().zip(rgba).all(|(a, b)| (*a as i16 - *b as i16).abs() <= tol)
    }

    #[test]
    fn the_ico_holds_every_size_as_the_icon_raster() {
        let bytes = ico();
        let entries = ico_entries(&bytes);
        assert_eq!(entries.iter().map(|e| e.0).collect::<Vec<_>>(), ICO_SIZES);
        for (px, offset, len) in entries {
            let data = &bytes[offset..offset + len];
            if px >= ICO_PNG_FROM {
                let (w, rgba) = decode(data);
                assert_eq!(w, px);
                assert!(matches_master(px, &rgba, 0));
            } else {
                // BITMAPINFOHEADER: size, width, doubled height, 1 plane, 32 bpp; bottom-up BGRA.
                assert_eq!(u32::from_le_bytes(data[..4].try_into().unwrap()), 40);
                assert_eq!(u32::from_le_bytes(data[4..8].try_into().unwrap()), px);
                assert_eq!(u32::from_le_bytes(data[8..12].try_into().unwrap()), px * 2);
                assert_eq!(u16::from_le_bytes(data[14..16].try_into().unwrap()), 32);
                let rows = &data[40..40 + (px * px * 4) as usize];
                let rgba: Vec<u8> =
                    rows.chunks_exact(px as usize * 4).rev().flat_map(|r| r.as_chunks::<4>().0.iter().flat_map(|p| [p[2], p[1], p[0], p[3]])).collect();
                assert!(matches_master(px, &rgba, 0));
                assert_eq!(data.len(), 40 + (px * px * 4) as usize + (px.div_ceil(32) * 4 * px) as usize, "pixels, then the AND mask");
            }
        }
    }

    #[test]
    fn the_icns_holds_16_to_1024_with_retina_forms() {
        let entries = icns_entries(&icns());
        assert_eq!(entries.len(), ICNS_TYPES.len());
        for ((code, px), (got_code, got_px)) in ICNS_TYPES.iter().zip(&entries) {
            assert_eq!((*code, *px), (got_code, *got_px));
        }
        assert_eq!(entries.iter().map(|e| e.1).min(), Some(16));
        assert_eq!(entries.iter().map(|e| e.1).max(), Some(1024));
    }

    /// The checked-in files are in step with the logo (regenerate with `cargo run -p bowshock -- icons crates/bowshock/assets/icons`).
    #[test]
    fn the_checked_in_icon_files_match_the_logo() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/icons");
        let ico_bytes = std::fs::read(dir.join("bowshock.ico")).expect("assets/icons/bowshock.ico; regenerate with `bowshock icons`");
        let largest = ico_entries(&ico_bytes).into_iter().find(|e| e.0 == 256).unwrap();
        assert!(matches_master(256, &decode(&ico_bytes[largest.1..largest.1 + largest.2]).1, 2), "bowshock.ico is stale");
        assert_eq!(ico_entries(&ico_bytes).iter().map(|e| e.0).collect::<Vec<_>>(), ICO_SIZES);

        let icns_bytes = std::fs::read(dir.join("bowshock.icns")).expect("assets/icons/bowshock.icns; regenerate with `bowshock icons`");
        let sizes: Vec<_> = icns_entries(&icns_bytes).into_iter().map(|e| e.1).collect();
        assert_eq!(sizes, ICNS_TYPES.map(|t| t.1));
        let (code, len) = (&icns_bytes[8..12], u32::from_be_bytes(icns_bytes[12..16].try_into().unwrap()) as usize);
        assert_eq!(code, b"icp4");
        assert!(matches_master(16, &decode(&icns_bytes[16..8 + len]).1, 2), "bowshock.icns is stale");
    }
}
