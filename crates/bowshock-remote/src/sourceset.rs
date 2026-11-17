//! The source set of the remote worker: the files `cargo build --profile worker -p bowshock-worker` needs, in a workspace that holds
//! only the crates the worker depends on. Shared by `build.rs` (which embeds the set and its key) and the library (tests).
//!
//! The root manifest is rewritten: `members` lists only [`CRATES`], and the `[patch]` sections (vendored copies of the app's GUI
//! crates, which the worker does not use) are dropped.

use sha2::{Digest, Sha256};
use std::fs;
use std::io;
use std::path::Path;

/// The crates of the workspace that the worker is built from (the worker and its path dependencies).
pub const CRATES: &[&str] = &["bowshock-cfd", "bowshock-core", "bowshock-reference", "bowshock-remote", "bowshock-worker"];

/// Root manifest with `members` restricted to [`CRATES`] and no `[patch]` section.
pub fn rewrite_root_manifest(text: &str) -> String {
    let members: Vec<String> = CRATES.iter().map(|c| format!("\"crates/{c}\"")).collect();
    let mut out = String::new();
    let mut in_patch = false;
    for line in text.lines() {
        let t = line.trim_start();
        if t.starts_with('[') {
            in_patch = t.starts_with("[patch");
        }
        if in_patch {
            continue;
        }
        if t.starts_with("members") && t.contains('=') {
            out.push_str(&format!("members = [{}]\n", members.join(", ")));
        } else {
            out.push_str(line);
            out.push('\n');
        }
    }
    out
}

fn walk(dir: &Path, rel: &str, out: &mut Vec<(String, Vec<u8>)>) -> io::Result<()> {
    let mut entries: Vec<_> = fs::read_dir(dir)?.collect::<Result<_, _>>()?;
    entries.sort_by_key(|e| e.file_name());
    for e in entries {
        let name = e.file_name().to_string_lossy().into_owned();
        let path = e.path();
        let rel = format!("{rel}/{name}");
        if e.file_type()?.is_dir() {
            walk(&path, &rel, out)?;
        } else {
            out.push((rel, fs::read(&path)?));
        }
    }
    Ok(())
}

/// The source set under the workspace `root`, sorted by path.
pub fn collect(root: &Path) -> io::Result<Vec<(String, Vec<u8>)>> {
    let mut files = Vec::new();
    let manifest = fs::read_to_string(root.join("Cargo.toml"))?;
    files.push(("Cargo.toml".to_string(), rewrite_root_manifest(&manifest).into_bytes()));
    for name in ["Cargo.lock", "rust-toolchain", "rust-toolchain.toml"] {
        if let Ok(b) = fs::read(root.join(name)) {
            files.push((name.to_string(), b));
        }
    }
    for c in CRATES {
        let dir = root.join("crates").join(c);
        for name in ["Cargo.toml", "build.rs"] {
            if let Ok(b) = fs::read(dir.join(name)) {
                files.push((format!("crates/{c}/{name}"), b));
            }
        }
        walk(&dir.join("src"), &format!("crates/{c}/src"), &mut files)?;
    }
    files.sort_by(|a, b| a.0.cmp(&b.0));
    Ok(files)
}

/// Hex SHA-256 over the sorted (path, contents) pairs.
pub fn key(files: &[(String, Vec<u8>)]) -> String {
    let mut h = Sha256::new();
    for (path, bytes) in files {
        h.update((path.len() as u64).to_le_bytes());
        h.update(path.as_bytes());
        h.update((bytes.len() as u64).to_le_bytes());
        h.update(bytes);
    }
    h.finalize().iter().map(|b| format!("{b:02x}")).collect()
}

/// The set as a gzip-compressed tar archive (fixed modes and times, so equal sets give equal archives).
pub fn tar_gz(files: &[(String, Vec<u8>)]) -> io::Result<Vec<u8>> {
    let enc = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::new(6));
    let mut tar = tar::Builder::new(enc);
    for (path, bytes) in files {
        let mut header = tar::Header::new_gnu();
        header.set_size(bytes.len() as u64);
        header.set_mode(0o644);
        header.set_mtime(0);
        tar.append_data(&mut header, path, bytes.as_slice())?;
    }
    tar.into_inner()?.finish()
}
