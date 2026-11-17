//! Embeds the source set of the remote worker (a tar.gz) and its key, computed from the workspace this crate sits in. A build of the
//! worker on the remote machine gets its key from `BOWSHOCK_WORKER_KEY` instead (the lock file is pruned there, so the set differs).

#[allow(dead_code)]
#[path = "src/sourceset.rs"]
mod sourceset;

use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    let root = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap()).join("../..");
    println!("cargo:rerun-if-env-changed=BOWSHOCK_WORKER_KEY");
    let (key, tgz) = if let Ok(key) = std::env::var("BOWSHOCK_WORKER_KEY") {
        (key, Vec::new())
    } else if root.join("crates/bowshock-worker/Cargo.toml").exists() {
        println!("cargo:rerun-if-changed={}", root.join("Cargo.toml").display());
        println!("cargo:rerun-if-changed={}", root.join("Cargo.lock").display());
        for c in sourceset::CRATES {
            println!("cargo:rerun-if-changed={}", root.join("crates").join(c).display());
        }
        let files = sourceset::collect(&root).expect("collect the worker source set");
        (sourceset::key(&files), sourceset::tar_gz(&files).expect("pack the worker source set"))
    } else {
        ("unavailable".to_string(), Vec::new())
    };
    std::fs::write(out.join("worker-key.txt"), key).unwrap();
    std::fs::write(out.join("worker-src.tar.gz"), tgz).unwrap();
}
