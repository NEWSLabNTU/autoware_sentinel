//! SPE sentinel binary build + spawn fixtures (POSIX dev path).
//!
//! `autoware_sentinel_spe` is excluded from the workspace because its
//! eventual cross-compile target is `armv7r-none-eabihf` (Cortex-R5F).
//! The POSIX dev profile (`posix-mock-ivc`) builds a Linux process that
//! runs the same `wire_executor` body the firmware will link, so this
//! fixture invokes `cargo build --release` from inside the crate.

use crate::process::{ManagedProcess, project_root};
use crate::{TestError, TestResult};
use once_cell::sync::OnceCell;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

static SENTINEL_SPE_BINARY: OnceCell<PathBuf> = OnceCell::new();

fn spe_capacity_envs() -> &'static [(&'static str, &'static str)] {
    &[
        ("ZPICO_MAX_PUBLISHERS", "56"),
        ("ZPICO_MAX_SUBSCRIBERS", "16"),
        ("ZPICO_MAX_QUERYABLES", "32"),
        ("ZPICO_MAX_LIVELINESS", "96"),
        ("NROS_MAX_PARAMETERS", "64"),
        ("NROS_EXECUTOR_MAX_CBS", "64"),
        ("NROS_PARAM_SERVICE_BUFFER_SIZE", "8192"),
    ]
}

pub fn build_sentinel_spe() -> TestResult<&'static Path> {
    SENTINEL_SPE_BINARY
        .get_or_try_init(|| {
            let root = project_root();
            let crate_dir = root.join("src/autoware_sentinel_spe");

            eprintln!("Building SPE sentinel in {:?}...", crate_dir);

            let mut cmd = Command::new("cargo");
            cmd.args(["build", "--release"]).current_dir(&crate_dir);
            for &(k, v) in spe_capacity_envs() {
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

            let binary = crate_dir.join("target/release/autoware_sentinel_spe");
            if !binary.exists() {
                return Err(TestError::BuildFailed(
                    "SPE sentinel binary not found after build".into(),
                ));
            }
            eprintln!("SPE sentinel built: {binary:?}");
            Ok(binary)
        })
        .map(|p| p.as_path())
}

#[rstest::fixture]
pub fn sentinel_spe_binary() -> PathBuf {
    build_sentinel_spe()
        .expect("Failed to build SPE sentinel")
        .to_path_buf()
}

/// Spawn the SPE sentinel against `locator` and wait for `Executor ready`.
///
/// Strips `ZENOH_SESSION_CONFIG_URI` and `ZENOH_ROUTER_CONFIG_URI` from
/// the inherited env (zenoh-pico's liveliness setup misbehaves otherwise
/// — see CLAUDE.md "Phase 12 Important" note).
pub fn start_sentinel_spe(binary: &Path, locator: &str) -> TestResult<ManagedProcess> {
    let mut cmd = Command::new(binary);
    cmd.env("RUST_LOG", "info")
        .env("ZENOH_LOCATOR", locator)
        .env("ZENOH_SESSION_CONFIG_URI", "")
        .env("ZENOH_ROUTER_CONFIG_URI", "");
    let mut proc = ManagedProcess::spawn_command(cmd, "sentinel_spe")?;

    let output = proc.wait_for_output_pattern("Executor ready", Duration::from_secs(15))?;
    eprintln!("SPE sentinel started:\n{output}");
    Ok(proc)
}
