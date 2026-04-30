//! NuttX sentinel binary build + QEMU launch fixtures.
//!
//! The NuttX sentinel is cross-compiled to `armv7a-nuttx-eabihf` (Tier-3,
//! requires nightly + `-Z build-std`) and launched inside
//! `qemu-system-arm -M virt -cpu cortex-a7` with virtio-net + SLIRP user
//! networking. The QEMU instance reaches the host's zenohd via the
//! 10.0.2.2 SLIRP gateway alias for host loopback, so tests must run a
//! zenohd on `127.0.0.1:7452` to match the locator baked into
//! `src/autoware_sentinel_nuttx/config.toml`.
//!
//! Like the FreeRTOS fixture, tests serialize on the compile-time port
//! via the `nuttx-qemu` nextest test-group.

use crate::process::{ManagedProcess, project_root};
use crate::{TestError, TestResult};
use once_cell::sync::OnceCell;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

/// Cached path to the built NuttX sentinel ELF.
static SENTINEL_NUTTX_BINARY: OnceCell<PathBuf> = OnceCell::new();

/// Capacity envs for the NuttX build. Mirror `nuttx_env` in the root
/// justfile so cached cargo builds can reuse this set when the test
/// fixture also drives the build.
fn nuttx_capacity_envs() -> &'static [(&'static str, &'static str)] {
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

/// Resolve the NuttX kernel source dir. Tests honour `$NUTTX_DIR` first
/// (matches the justfile recipe), falling back to the sibling
/// nano-ros-sentinel checkout.
fn nuttx_dir() -> PathBuf {
    if let Ok(d) = std::env::var("NUTTX_DIR") {
        return PathBuf::from(d);
    }
    project_root()
        .parent()
        .expect("repo has parent")
        .join("nano-ros-sentinel/third-party/nuttx/nuttx")
}

/// Build the NuttX sentinel ELF and return its path (cached). Assumes the
/// NuttX kernel has already been built at `$NUTTX_DIR` — run
/// `just build-nuttx-kernel` once before invoking the tests.
pub fn build_sentinel_nuttx() -> TestResult<&'static Path> {
    SENTINEL_NUTTX_BINARY
        .get_or_try_init(|| {
            let root = project_root();
            let crate_dir = root.join("src/autoware_sentinel_nuttx");
            let nuttx = nuttx_dir();
            if !nuttx.join("staging/libc.a").exists() {
                return Err(TestError::BuildFailed(format!(
                    "NuttX kernel not built — run `just build-nuttx-kernel` (looked at {nuttx:?})"
                )));
            }

            eprintln!("Building NuttX sentinel in {crate_dir:?}...");
            let mut cmd = Command::new("cargo");
            cmd.args(["build", "--release"])
                .current_dir(&crate_dir)
                // The parent test process runs on stable, which leaks
                // `RUSTUP_TOOLCHAIN=stable...` through `cargo nextest`.
                // Clear it so rustup honours the crate-local
                // `rust-toolchain.toml` (pinned nightly) and the
                // `-Z build-std` directive in `.cargo/config.toml`.
                .env_remove("RUSTUP_TOOLCHAIN")
                .env("NUTTX_DIR", &nuttx);
            for &(k, v) in nuttx_capacity_envs() {
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

            let binary =
                crate_dir.join("target/armv7a-nuttx-eabihf/release/autoware_sentinel_nuttx");
            if !binary.exists() {
                return Err(TestError::BuildFailed(
                    "NuttX sentinel ELF not found after build".into(),
                ));
            }
            eprintln!("NuttX sentinel built: {binary:?}");
            Ok(binary)
        })
        .map(|p| p.as_path())
}

/// rstest fixture returning the cached NuttX sentinel ELF path.
#[rstest::fixture]
pub fn sentinel_nuttx_binary() -> PathBuf {
    build_sentinel_nuttx()
        .expect("Failed to build NuttX sentinel")
        .to_path_buf()
}

/// Launch `qemu-system-arm -M virt -cpu cortex-a7` with the NuttX
/// sentinel ELF and wait for `Executor ready`. The QEMU instance is
/// killed on `ManagedProcess` drop. Returns the boot output collected up
/// to and including `Executor ready`.
pub fn start_sentinel_nuttx(elf: &Path) -> TestResult<(ManagedProcess, String)> {
    let mut cmd = Command::new("qemu-system-arm");
    cmd.args([
        "-M",
        "virt",
        "-cpu",
        "cortex-a7",
        "-nographic",
        "-netdev",
        "user,id=net0",
        "-device",
        "virtio-net-device,netdev=net0",
        "-kernel",
    ])
    .arg(elf);
    let mut proc = ManagedProcess::spawn_command(cmd, "sentinel_nuttx")?;

    // NuttX boot includes virtio-net up + zpico TCP/handshake; the cold
    // path runs ~5–10 s on this host. Allow generous headroom.
    let output =
        proc.wait_for_output_pattern("Executor ready", Duration::from_secs(45))?;
    eprintln!("NuttX sentinel started:\n{output}");
    if !output.contains("Executor ready") {
        return Err(TestError::Timeout);
    }
    Ok((proc, output))
}

/// NuttX-side zenoh locator (where the QEMU sentinel connects).
///
/// QEMU SLIRP NATs `10.0.2.2:7452` to host `127.0.0.1:7452`, so zenohd
/// must listen on the host loopback at this port for NuttX tests.
pub const NUTTX_ZENOHD_PORT: u16 = 7452;
