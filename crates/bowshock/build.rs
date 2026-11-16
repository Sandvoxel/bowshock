//! Windows: embeds `assets/icons/bowshock.ico` in the executable as its icon (Explorer, task bar, Alt-Tab).
//!
//! The icon becomes a compiled resource (`.res`: one `RT_ICON` per image and one `RT_GROUP_ICON`) that is written here and handed
//! to the linker. Writing the `.res` directly keeps cross builds from a non-Windows host (`cargo xwin`) free of a resource
//! compiler (`rc.exe`, `llvm-rc`, `windres`); `link.exe` and `lld-link` both take `.res` files as inputs. Other targets: nothing.

use std::path::PathBuf;

const RT_ICON: u16 = 3;
const RT_GROUP_ICON: u16 = 14;
/// `MOVEABLE | PURE | DISCARDABLE`, what `rc` gives icons.
const MEMORY_FLAGS: u16 = 0x1030;
/// en-US, what `rc` defaults to.
const LANGUAGE: u16 = 0x0409;
/// The icon group's resource id; the lowest group id is the one Explorer shows.
const GROUP_ID: u16 = 1;

fn main() {
    let ico = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap()).join("assets/icons/bowshock.ico");
    println!("cargo:rerun-if-changed={}", ico.display());
    println!("cargo:rerun-if-changed=build.rs");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("windows") {
        return;
    }
    if std::env::var("CARGO_CFG_TARGET_ENV").as_deref() != Ok("msvc") {
        println!("cargo:warning=the icon resource is only embedded for MSVC targets");
        return;
    }
    let ico = std::fs::read(&ico).unwrap_or_else(|e| panic!("{}: {e}", ico.display()));
    let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap()).join("bowshock-icon.res");
    std::fs::write(&out, res(&ico)).unwrap();
    println!("cargo:rustc-link-arg-bins={}", out.display());
}

/// A 16-bit little-endian value.
fn u16le(out: &mut Vec<u8>, v: u16) {
    out.extend_from_slice(&v.to_le_bytes());
}

/// A 32-bit little-endian value.
fn u32le(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

/// Pads to a 4-byte boundary.
fn align(out: &mut Vec<u8>) {
    while !out.len().is_multiple_of(4) {
        out.push(0);
    }
}

/// One `.res` entry with numeric type and name ids.
fn entry(out: &mut Vec<u8>, kind: u16, id: u16, data: &[u8]) {
    u32le(out, data.len() as u32); // data size
    u32le(out, 32); // header size: 8 + two (0xFFFF, id) pairs + 16
    u16le(out, 0xFFFF);
    u16le(out, kind);
    u16le(out, 0xFFFF);
    u16le(out, id);
    u32le(out, 0); // data version
    u16le(out, MEMORY_FLAGS);
    u16le(out, LANGUAGE);
    u32le(out, 0); // version
    u32le(out, 0); // characteristics
    out.extend_from_slice(data);
    align(out);
}

/// The `.res` for an `.ico` file: the empty 32-byte entry every `.res` starts with, an `RT_ICON` per image, the `RT_GROUP_ICON`.
fn res(ico: &[u8]) -> Vec<u8> {
    let at = |i: usize| u32::from_le_bytes(ico[i..i + 4].try_into().unwrap()) as usize;
    assert!(ico.len() >= 6 && ico[..4] == [0, 0, 1, 0], "bowshock.ico is not an icon file");
    let count = u16::from_le_bytes([ico[4], ico[5]]);

    let mut out = Vec::new();
    entry(&mut out, 0, 0, &[]);
    let mut group = vec![0, 0, 1, 0];
    u16le(&mut group, count);
    for i in 0..count {
        let dir = 6 + 16 * i as usize;
        let (len, offset) = (at(dir + 8), at(dir + 12));
        let id = i + 1;
        entry(&mut out, RT_ICON, id, &ico[offset..offset + len]);
        // GRPICONDIRENTRY: the directory entry with the image id in place of the file offset.
        group.extend_from_slice(&ico[dir..dir + 12]);
        u16le(&mut group, id);
    }
    entry(&mut out, RT_GROUP_ICON, GROUP_ID, &group);
    out
}
