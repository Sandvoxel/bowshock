//! Installing the worker on the remote machine: the commands (pure, tested) and the steps that run them over ssh.
//!
//! The worker lives in `~/.cache/bowshock/worker/<key>/bowshock-worker`, `<key>` being [`WORKER_KEY`]. A missing worker is installed
//! either from a locally built binary of the same key (same platform) or by building the embedded source set on the remote.

use crate::client::{RemoteError, RemoteHost};
use crate::proto::DeviceInfo;
use crate::{WORKER_KEY, WORKER_SOURCE};
use std::io::{BufRead, BufReader, Read, Write};
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};

/// Where [`Remote::connect`](crate::Remote::connect) is while it sets the remote up.
#[derive(Clone, Debug, PartialEq)]
pub enum SetupStatus {
    /// Opening the ssh connection.
    Connecting,
    /// Looking for the worker on the remote.
    Checking,
    /// Sending the worker (binary or source) to the remote.
    Uploading {
        /// Bytes sent.
        bytes: u64,
    },
    /// Building the worker on the remote: the latest line of cargo's output.
    Building {
        /// The line.
        line: String,
    },
    /// Starting the worker and asking it about its device.
    Probing,
    /// Connected.
    Ready(DeviceInfo),
}

/// Environment variable that disables the shortcut of copying a locally built worker to a remote of the same platform.
pub const NO_PREBUILT_ENV: &str = "BOWSHOCK_REMOTE_NO_PREBUILT";

/// Line the serve script writes to stderr when the worker is not installed.
pub(crate) const MISSING_MARKER: &str = "BSHK-MISSING";

/// `s` single-quoted for a POSIX shell.
pub fn sh_quote(s: &str) -> String {
    format!("'{}'", s.replace('\'', r"'\''"))
}

/// The remote command line that runs `script` with `sh`.
pub fn sh_command(script: &str) -> String {
    format!("sh -c {}", sh_quote(script))
}

/// Remote directory (in shell syntax) that holds the workers, the shared build directory and the unpacked sources.
const ROOT: &str = "$HOME/.cache/bowshock/worker";

/// Script that starts the worker of `key`, or reports it missing.
pub fn serve_script(key: &str) -> String {
    format!("W=\"{ROOT}/{key}/bowshock-worker\"; if [ -x \"$W\" ]; then exec \"$W\" serve; fi; echo {MISSING_MARKER} >&2; exit 3")
}

/// Script that prints the platform of the remote.
pub fn platform_script() -> String {
    "echo \"BSHK-UNAME:$(uname -sm)\"".to_string()
}

/// Script that stores stdin as the worker binary of `key`.
pub fn upload_binary_script(key: &str) -> String {
    let d = format!("\"{ROOT}/{key}\"");
    format!("mkdir -p {d} && cat > {d}/bowshock-worker.tmp && chmod +x {d}/bowshock-worker.tmp && mv {d}/bowshock-worker.tmp {d}/bowshock-worker")
}

/// Script that runs the installed worker's `probe`.
pub fn probe_script(key: &str) -> String {
    format!("\"{ROOT}/{key}/bowshock-worker\" probe")
}

/// Script that removes the installed worker of `key` (a binary that cannot run here).
pub fn remove_script(key: &str) -> String {
    format!("rm -rf \"{ROOT}/{key}\"")
}

/// Script that unpacks the source archive on stdin.
pub fn unpack_script(key: &str) -> String {
    format!("S=\"{ROOT}/src-{key}\"; rm -rf \"$S\" && mkdir -p \"$S\" && tar xzf - -C \"$S\"")
}

/// Script that builds the unpacked source, installs the binary and removes old workers and the sources (the newest two keys stay).
/// Everything cargo prints goes to stdout.
pub fn build_script(key: &str) -> String {
    format!(
        "exec 2>&1; export PATH=\"$HOME/.cargo/bin:$PATH\"; \
if ! command -v cargo >/dev/null 2>&1; then \
echo 'cargo not found: installing rust with rustup'; \
if command -v curl >/dev/null 2>&1; then curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal || exit 1; \
else echo 'BSHK-ERROR: neither cargo nor curl is installed'; exit 1; fi; \
export PATH=\"$HOME/.cargo/bin:$PATH\"; fi; \
R=\"{ROOT}\"; cd \"$R/src-{key}\" || exit 1; \
BOWSHOCK_WORKER_KEY={key} CARGO_TARGET_DIR=\"$R/target\" cargo build --profile worker -p bowshock-worker || exit 1; \
mkdir -p \"$R/{key}\" && cp \"$R/target/worker/bowshock-worker\" \"$R/{key}/bowshock-worker.tmp\" && chmod +x \"$R/{key}/bowshock-worker.tmp\" && mv \"$R/{key}/bowshock-worker.tmp\" \"$R/{key}/bowshock-worker\" || exit 1; \
rm -rf \"$R/src-{key}\"; cd \"$R\" && for d in $(ls -1dt [0-9a-f]*/ 2>/dev/null | tail -n +3); do rm -rf \"$d\"; done; echo BSHK-INSTALLED"
    )
}

/// The platform string of `uname -sm` for this machine, if it is one this crate knows.
pub fn local_uname() -> Option<&'static str> {
    match (std::env::consts::OS, std::env::consts::ARCH) {
        ("linux", "x86_64") => Some("Linux x86_64"),
        ("linux", "aarch64") => Some("Linux aarch64"),
        ("macos", "aarch64") => Some("Darwin arm64"),
        ("macos", "x86_64") => Some("Darwin x86_64"),
        _ => None,
    }
}

/// The ssh command for `host`: key-based only (no prompts), a connect timeout and keep-alives.
pub fn ssh_args(host: &RemoteHost) -> Result<Vec<String>, RemoteError> {
    let t = host.target.trim();
    if t.is_empty() || t.starts_with('-') || t.chars().any(char::is_whitespace) {
        return Err(RemoteError::Ssh(format!("'{}' is not a valid ssh target (user@host or a name from ~/.ssh/config)", host.target)));
    }
    let mut args: Vec<String> = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=15"].iter().map(|s| s.to_string()).collect();
    if let Some(p) = host.port {
        args.push("-p".into());
        args.push(p.to_string());
    }
    args.push(t.to_string());
    Ok(args)
}

/// An ssh process running `script` on `host`, with piped stdin, stdout and stderr.
pub(crate) fn ssh_spawn(host: &RemoteHost, script: &str) -> Result<std::process::Child, RemoteError> {
    let mut cmd = Command::new("ssh");
    cmd.args(ssh_args(host)?).arg(sh_command(script)).stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped());
    cmd.spawn().map_err(|e| RemoteError::Ssh(format!("cannot run ssh: {e}")))
}

/// Keeps the last 8 KiB of what a stream writes, in the background.
pub(crate) fn drain(stream: impl Read + Send + 'static) -> Arc<Mutex<String>> {
    let buf = Arc::new(Mutex::new(String::new()));
    let sink = buf.clone();
    std::thread::spawn(move || {
        for line in BufReader::new(stream).lines().map_while(Result::ok) {
            let mut b = sink.lock().unwrap();
            b.push_str(&line);
            b.push('\n');
            if b.len() > 8192 {
                let cut = b.len() - 8192;
                let cut = (cut..b.len()).find(|&i| b.is_char_boundary(i)).unwrap_or(b.len());
                b.drain(..cut);
            }
        }
    });
    buf
}

/// What an ssh command left behind.
struct Output {
    code: Option<i32>,
    stdout: Vec<String>,
    stderr: String,
}

/// Runs `script` on `host`, feeding `input` to its stdin (`on_sent` sees the bytes sent so far) and passing every stdout line to
/// `on_line`.
fn ssh_run(host: &RemoteHost, script: &str, input: Option<&mut dyn Read>, on_sent: &mut dyn FnMut(u64), on_line: &mut dyn FnMut(&str)) -> Result<Output, RemoteError> {
    let mut child = ssh_spawn(host, script)?;
    let stderr = drain(child.stderr.take().unwrap());
    let mut stdin = child.stdin.take().unwrap();
    if let Some(input) = input {
        let mut sent = 0u64;
        let mut chunk = vec![0u8; 256 << 10];
        let mut next_report = 0u64;
        loop {
            let n = input.read(&mut chunk).map_err(|e| RemoteError::Setup(format!("cannot read the upload: {e}")))?;
            if n == 0 {
                break;
            }
            if stdin.write_all(&chunk[..n]).is_err() {
                break; // the remote command ended early: its exit status tells why
            }
            sent += n as u64;
            if sent >= next_report {
                on_sent(sent);
                next_report = sent + (1 << 20);
            }
        }
        on_sent(sent);
    }
    drop(stdin);
    let mut lines = Vec::new();
    for line in BufReader::new(child.stdout.take().unwrap()).lines().map_while(Result::ok) {
        on_line(&line);
        lines.push(line);
    }
    let status = child.wait().map_err(|e| RemoteError::Ssh(e.to_string()))?;
    std::thread::sleep(std::time::Duration::from_millis(20));
    let stderr = stderr.lock().unwrap().clone();
    Ok(Output { code: status.code(), stdout: lines, stderr })
}

/// Runs a short script and fails with ssh's or the script's message when it does not exit 0.
fn ssh_ok(host: &RemoteHost, script: &str, what: &str) -> Result<Output, RemoteError> {
    let out = ssh_run(host, script, None, &mut |_| {}, &mut |_| {})?;
    check(&out, what)?;
    Ok(out)
}

fn check(out: &Output, what: &str) -> Result<(), RemoteError> {
    match out.code {
        Some(0) => Ok(()),
        Some(255) => Err(RemoteError::Ssh(out.stderr.trim().to_string())),
        _ => Err(RemoteError::Setup(format!("{what} failed: {}{}", out.stderr.trim(), tail(&out.stdout, 12)))),
    }
}

fn tail(lines: &[String], n: usize) -> String {
    let from = lines.len().saturating_sub(n);
    lines[from..].iter().map(|l| format!("\n{l}")).collect()
}

/// The `uname -sm` of the remote from the output of [`platform_script`].
pub fn parse_platform(lines: &[String]) -> Option<&str> {
    lines.iter().find_map(|l| l.strip_prefix("BSHK-UNAME:")).map(str::trim)
}

/// A locally built worker with the key of this build: next to the running program, or in the target directory of the workspace this
/// crate was built in.
fn find_local_worker() -> Option<PathBuf> {
    let name = if cfg!(windows) { "bowshock-worker.exe" } else { "bowshock-worker" };
    let mut candidates = Vec::new();
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            candidates.push(dir.join(name));
            if dir.file_name().is_some_and(|n| n == "deps") {
                if let Some(p) = dir.parent() {
                    candidates.push(p.join(name));
                }
            }
        }
    }
    let target = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target");
    for profile in ["worker", "release", "debug"] {
        candidates.push(target.join(profile).join(name));
    }
    candidates.into_iter().filter(|p| p.is_file()).find(|p| {
        Command::new(p)
            .arg("probe")
            .output()
            .ok()
            .and_then(|o| serde_json::from_slice::<DeviceInfo>(o.stdout.split(|&b| b == b'\n').find(|l| l.first() == Some(&b'{'))?).ok())
            .is_some_and(|d| d.worker_key == WORKER_KEY)
    })
}

/// Installs the worker of this build on `host`, reporting the steps through `status`.
pub(crate) fn install(host: &RemoteHost, status: &dyn Fn(SetupStatus)) -> Result<(), RemoteError> {
    let key = WORKER_KEY;
    let platform = ssh_ok(host, &platform_script(), "reading the platform")?;
    let platform = parse_platform(&platform.stdout).unwrap_or("").to_string();

    if std::env::var_os(NO_PREBUILT_ENV).is_none() && local_uname() == Some(platform.as_str()) {
        if let Some(path) = find_local_worker() {
            if install_binary(host, key, &path, status).is_ok() {
                return Ok(());
            }
        }
    }
    if WORKER_SOURCE.is_empty() {
        return Err(RemoteError::Setup(format!(
            "the worker is not installed on {} and this build of bowshock has no worker source to build it from (platform {platform})",
            host.target
        )));
    }
    let mut src: &[u8] = WORKER_SOURCE;
    let out = ssh_run(host, &unpack_script(key), Some(&mut src), &mut |b| status(SetupStatus::Uploading { bytes: b }), &mut |_| {})?;
    check(&out, "unpacking the worker source")?;
    let mut last = Vec::new();
    let out = ssh_run(host, &build_script(key), None, &mut |_| {}, &mut |l| {
        last.push(l.to_string());
        status(SetupStatus::Building { line: l.trim().to_string() });
    })?;
    if out.code != Some(0) || !out.stdout.iter().any(|l| l == "BSHK-INSTALLED") {
        if out.code == Some(255) {
            return Err(RemoteError::Ssh(out.stderr.trim().to_string()));
        }
        return Err(RemoteError::Setup(format!("building the worker on {} failed:{}", host.target, tail(&out.stdout, 25))));
    }
    Ok(())
}

/// Copies the binary at `path` and checks that it runs on the remote; removes it again when it does not.
fn install_binary(host: &RemoteHost, key: &str, path: &PathBuf, status: &dyn Fn(SetupStatus)) -> Result<(), RemoteError> {
    let mut file = std::fs::File::open(path).map_err(|e| RemoteError::Setup(e.to_string()))?;
    let out = ssh_run(host, &upload_binary_script(key), Some(&mut file), &mut |b| status(SetupStatus::Uploading { bytes: b }), &mut |_| {})?;
    check(&out, "uploading the worker")?;
    let probe = ssh_run(host, &probe_script(key), None, &mut |_| {}, &mut |_| {})?;
    if probe.code == Some(0) && probe.stdout.iter().any(|l| l.starts_with('{')) {
        return Ok(());
    }
    let _ = ssh_run(host, &remove_script(key), None, &mut |_| {}, &mut |_| {});
    Err(RemoteError::Setup("the local worker binary does not run on the remote".into()))
}

/// The message for a remote without a usable GPU.
pub fn no_gpu_message(host: &RemoteHost, device: &DeviceInfo) -> String {
    format!(
        "{} has no usable GPU ({}). Linux: install the Vulkan loader and the vendor driver, e.g. `sudo pacman -S vulkan-icd-loader nvidia-utils` or `sudo apt install libvulkan1 nvidia-driver-XXX`; then connect again",
        host.target,
        device.no_gpu_reason.as_deref().unwrap_or("no hardware Vulkan device")
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sourceset;

    fn host() -> RemoteHost {
        RemoteHost { target: "me@box".into(), port: Some(2222) }
    }

    #[test]
    fn ssh_args_are_batch_mode_with_port_and_target_last() {
        let a = ssh_args(&host()).unwrap();
        assert_eq!(&a[..2], ["-o", "BatchMode=yes"]);
        assert!(a.windows(2).any(|w| w == ["-p", "2222"]));
        assert_eq!(a.last().unwrap(), "me@box");
        assert!(ssh_args(&RemoteHost { target: "-oProxyCommand=x".into(), port: None }).is_err());
        assert!(ssh_args(&RemoteHost { target: "a b".into(), port: None }).is_err());
        assert!(ssh_args(&RemoteHost { target: String::new(), port: None }).is_err());
    }

    #[test]
    fn scripts_are_quoted_for_sh_and_carry_the_key() {
        assert_eq!(sh_quote("it's"), r"'it'\''s'");
        let key = "abc123";
        for s in [serve_script(key), upload_binary_script(key), probe_script(key), unpack_script(key), build_script(key), remove_script(key)] {
            assert!(s.contains(key));
        }
        let cmd = sh_command(&serve_script(key));
        assert!(cmd.starts_with("sh -c '") && cmd.contains("exec \"$W\" serve"));
        let build = build_script(key);
        assert!(build.contains("cargo build --profile worker -p bowshock-worker") && build.contains("BOWSHOCK_WORKER_KEY=abc123"));
        assert!(build.contains("sh.rustup.rs") && build.contains("tail -n +3"));
    }

    #[test]
    fn the_platform_line_is_found_among_noise() {
        let lines = ["Welcome to box".to_string(), "BSHK-UNAME:Linux x86_64".to_string()];
        assert_eq!(parse_platform(&lines), Some("Linux x86_64"));
        assert_eq!(parse_platform(&[]), None);
    }

    #[test]
    fn the_embedded_key_is_the_key_of_the_source_set_and_deterministic() {
        if WORKER_KEY == "unavailable" || WORKER_SOURCE.is_empty() {
            return;
        }
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let a = sourceset::collect(&root).unwrap();
        let b = sourceset::collect(&root).unwrap();
        assert_eq!(sourceset::key(&a), sourceset::key(&b));
        assert_eq!(sourceset::key(&a), WORKER_KEY, "the sources changed since this crate was built");
        assert_eq!(WORKER_KEY.len(), 64);
        // another byte, another key
        let mut c = a.clone();
        c[0].1.push(b'\n');
        assert_ne!(sourceset::key(&c), WORKER_KEY);
        assert_eq!(sourceset::tar_gz(&a).unwrap(), WORKER_SOURCE);
    }

    #[test]
    fn the_source_set_holds_the_worker_crates_and_their_path_dependencies_only() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let set = sourceset::collect(&root).unwrap();
        let manifest = String::from_utf8(set.iter().find(|(p, _)| p == "Cargo.toml").unwrap().1.clone()).unwrap();
        assert!(manifest.contains("members = [\"crates/bowshock-cfd\"") && !manifest.contains("[patch"));
        assert!(manifest.contains("[profile.worker]"));
        // idempotent
        assert_eq!(sourceset::rewrite_root_manifest(&manifest), manifest);
        // every bowshock crate any member depends on (outside dev-dependencies) is in the set
        for c in sourceset::CRATES {
            let text = std::fs::read_to_string(root.join("crates").join(c).join("Cargo.toml")).unwrap();
            let mut dev = false;
            for line in text.lines() {
                if line.starts_with('[') {
                    dev = line.contains("dev-dependencies");
                }
                if !dev {
                    if let Some(name) = line.split(|ch: char| !(ch.is_alphanumeric() || ch == '-')).find(|w| w.starts_with("bowshock-")) {
                        assert!(sourceset::CRATES.contains(&name) || name == c.to_string(), "{c} depends on {name}, which is not in the source set");
                    }
                }
            }
        }
        for (path, _) in &set {
            assert!(!path.starts_with('/') && !path.contains("//"), "{path}");
        }
    }
}
