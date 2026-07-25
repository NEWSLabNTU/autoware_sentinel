//! Sentinel binary build fixture
//!
//! Builds the `autoware_sentinel_linux` binary once per test process
//! using `OnceCell`, then provides it as an rstest fixture.

use crate::process::{ManagedProcess, project_root};
use crate::{TestError, TestResult};
use once_cell::sync::OnceCell;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

/// Cached path to the built sentinel binary.
static SENTINEL_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Build the sentinel Linux binary and return its path.
///
/// Uses `OnceCell` so the binary is only built once per test process,
/// even if multiple tests request it.
pub fn build_sentinel() -> TestResult<&'static Path> {
    SENTINEL_BINARY
        .get_or_try_init(|| {
            let root = project_root();
            eprintln!("Building sentinel binary in {:?}...", root);

            let output = Command::new("cargo")
                .args(["build", "-p", "autoware_sentinel_linux"])
                .env("ZPICO_MAX_PUBLISHERS", "56")
                .env("ZPICO_MAX_SUBSCRIBERS", "16")
                .env("ZPICO_MAX_QUERYABLES", "32")
                .env("ZPICO_MAX_LIVELINESS", "96")
                .env("NROS_MAX_PARAMETERS", "64")
                .env("NROS_EXECUTOR_MAX_CBS", "64")
                .env("NROS_PARAM_SERVICE_BUFFER_SIZE", "8192")
                .current_dir(&root)
                .output()
                .map_err(|e| TestError::BuildFailed(format!("cargo build failed to start: {e}")))?;

            if !output.status.success() {
                let stderr = String::from_utf8_lossy(&output.stderr);
                return Err(TestError::BuildFailed(format!(
                    "cargo build failed:\n{}",
                    stderr
                )));
            }

            let binary = root.join("target/debug/autoware_sentinel_linux");
            if !binary.exists() {
                return Err(TestError::BuildFailed(
                    "Binary not found after build".into(),
                ));
            }

            eprintln!("Sentinel binary built: {:?}", binary);
            Ok(binary)
        })
        .map(|p| p.as_path())
}

/// rstest fixture that builds and returns the sentinel binary path.
#[rstest::fixture]
pub fn sentinel_binary() -> PathBuf {
    build_sentinel()
        .expect("Failed to build sentinel binary")
        .to_path_buf()
}

/// Start the sentinel binary as a managed process.
///
/// The process is automatically killed on drop. Returns after the
/// sentinel prints its "Executor ready" message (or timeout).
pub fn start_sentinel(binary: &Path, locator: &str) -> TestResult<ManagedProcess> {
    let mut cmd = Command::new(binary);
    cmd.env("RUST_LOG", "info").env("ZENOH_LOCATOR", locator);
    let mut proc = ManagedProcess::spawn_command(cmd, "sentinel")?;

    // Wait for the sentinel to be ready
    let output = proc.wait_for_output_pattern("Executor ready", Duration::from_secs(10))?;
    eprintln!("Sentinel started:\n{}", output);

    Ok(proc)
}
