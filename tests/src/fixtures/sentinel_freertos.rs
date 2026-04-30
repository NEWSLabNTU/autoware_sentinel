//! FreeRTOS sentinel binary build + QEMU launch fixtures
//!
//! The FreeRTOS sentinel is cross-compiled to `thumbv7m-none-eabi` and
//! launched inside `qemu-system-arm -machine mps2-an385`. The QEMU
//! instance reaches the host's zenohd via SLIRP user-mode networking
//! (10.0.2.2:7451 → host 127.0.0.1:7451), so tests must run zenohd on
//! the host loopback at port 7451 to match the locator baked into
//! `src/autoware_sentinel_freertos/config.toml`.
//!
//! Because the locator is compile-time constant, FreeRTOS tests cannot
//! use ephemeral ports — they share `127.0.0.1:7451` and must run
//! serialised (nextest `test-group = "sentinel_freertos"`).

use crate::process::{ManagedProcess, project_root};
use crate::{TestError, TestResult};
use once_cell::sync::OnceCell;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

/// Cached path to the built FreeRTOS sentinel ELF.
static SENTINEL_FREERTOS_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Capacity envs for the FreeRTOS build. Must match `freertos_env` in the
/// root justfile so cached builds are reused. Keep in sync.
fn freertos_capacity_envs() -> &'static [(&'static str, &'static str)] {
    &[
        ("NROS_EXECUTOR_MAX_CBS", "32"),
        ("NROS_MAX_PARAMETERS", "8"),
        ("NROS_PARAM_SERVICE_BUFFER_SIZE", "1024"),
        ("ZPICO_MAX_PUBLISHERS", "40"),
        ("ZPICO_MAX_SUBSCRIBERS", "8"),
        ("ZPICO_MAX_QUERYABLES", "32"),
        ("ZPICO_MAX_LIVELINESS", "80"),
    ]
}

/// nano-ros-sentinel sibling clone path (relative to the autoware_sentinel
/// repo root). Matches the path overrides in
/// `src/autoware_sentinel_freertos/.cargo/config.toml`.
fn nano_ros_root(repo_root: &Path) -> PathBuf {
    repo_root
        .parent()
        .expect("repo has parent")
        .join("nano-ros-sentinel")
}

/// FreeRTOS / lwIP build envs for the board crate. Match `freertos_env`
/// in the root justfile.
fn freertos_build_envs(repo_root: &Path) -> Vec<(String, String)> {
    let nano_ros = nano_ros_root(repo_root);
    let kernel = nano_ros.join("third-party/freertos/kernel");
    let lwip = nano_ros.join("third-party/freertos/lwip");
    let cfg = nano_ros.join("packages/boards/nros-board-mps2-an385-freertos/config");
    vec![
        ("FREERTOS_DIR".into(), kernel.to_string_lossy().into()),
        ("LWIP_DIR".into(), lwip.to_string_lossy().into()),
        ("FREERTOS_PORT".into(), "GCC/ARM_CM3".into()),
        ("FREERTOS_CONFIG_DIR".into(), cfg.to_string_lossy().into()),
    ]
}

/// Build the FreeRTOS sentinel ELF and return its path (cached).
pub fn build_sentinel_freertos() -> TestResult<&'static Path> {
    SENTINEL_FREERTOS_BINARY
        .get_or_try_init(|| {
            let root = project_root();
            let crate_dir = root.join("src/autoware_sentinel_freertos");

            eprintln!("Building FreeRTOS sentinel in {:?}...", crate_dir);

            let mut cmd = Command::new("cargo");
            cmd.args(["build", "--release"]).current_dir(&crate_dir);
            for (k, v) in freertos_build_envs(&root) {
                cmd.env(k, v);
            }
            for &(k, v) in freertos_capacity_envs() {
                cmd.env(k, v);
            }

            let output = cmd
                .output()
                .map_err(|e| TestError::BuildFailed(format!("cargo build failed to start: {e}")))?;

            if !output.status.success() {
                let stderr = String::from_utf8_lossy(&output.stderr);
                return Err(TestError::BuildFailed(format!(
                    "cargo build failed:\n{stderr}"
                )));
            }

            let binary = crate_dir
                .join("target/thumbv7m-none-eabi/release/autoware_sentinel_freertos");
            if !binary.exists() {
                return Err(TestError::BuildFailed(
                    "FreeRTOS sentinel ELF not found after build".into(),
                ));
            }
            eprintln!("FreeRTOS sentinel built: {binary:?}");
            Ok(binary)
        })
        .map(|p| p.as_path())
}

/// rstest fixture returning the cached FreeRTOS sentinel ELF path.
#[rstest::fixture]
pub fn sentinel_freertos_binary() -> PathBuf {
    build_sentinel_freertos()
        .expect("Failed to build FreeRTOS sentinel")
        .to_path_buf()
}

/// Launch `qemu-system-arm` with the FreeRTOS sentinel ELF and wait for
/// the executor-ready string on its semihosting console.
///
/// The QEMU instance is automatically killed when the returned
/// `ManagedProcess` is dropped (process-group cleanup). Returns the
/// boot output collected up to and including the `Executor ready` line
/// so callers can assert on declare-publisher confirmations etc.
///
/// Note: only one FreeRTOS sentinel boot per test process is reliable.
/// `nros-board-mps2-an385-freertos` derives the zenoh session ID from a
/// hash of the configured IP + MAC (`config.toml`), so a second QEMU
/// instance computes the same ZID and zenohd refuses the duplicate
/// session. Tests using this fixture must therefore be designed so a
/// single boot proves the whole path (boot → lwIP → zpico → declares).
pub fn start_sentinel_freertos(elf: &Path) -> TestResult<(ManagedProcess, String)> {
    let mut cmd = Command::new("qemu-system-arm");
    cmd.args([
        "-cpu",
        "cortex-m3",
        "-machine",
        "mps2-an385",
        "-nographic",
        "-semihosting-config",
        "enable=on,target=native",
        "-kernel",
    ])
    .arg(elf);
    let mut proc = ManagedProcess::spawn_command(cmd, "sentinel_freertos")?;

    // FreeRTOS boot: lwIP init (~3 s) + zpico declare burst (~5 s for
    // comp-all). Allow generous timeout for cold-cache CI.
    let output =
        proc.wait_for_output_pattern("Executor ready", Duration::from_secs(30))?;
    eprintln!("FreeRTOS sentinel started:\n{output}");
    if !output.contains("Executor ready") {
        return Err(TestError::Timeout);
    }
    Ok((proc, output))
}

/// FreeRTOS-side zenoh locator (where the QEMU sentinel connects).
///
/// QEMU SLIRP NATs `10.0.2.2:7451` to host `127.0.0.1:7451`, so zenohd
/// must listen on the host loopback at this port for FreeRTOS tests.
pub const FREERTOS_ZENOHD_PORT: u16 = 7451;
