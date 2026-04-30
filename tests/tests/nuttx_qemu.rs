//! NuttX QEMU integration tests (Phase 13.5.6).
//!
//! Verifies the NuttX sentinel binary (cross-compiled for
//! `armv7a-nuttx-eabihf`, launched inside `qemu-system-arm -M virt -cpu
//! cortex-a7` with virtio-net + SLIRP) boots, opens its zpico session
//! against zenohd, and reaches `executor.spin(...)`.
//!
//! ```text
//! ┌────────────────────────────┐  SLIRP NAT  ┌──────────────────────┐
//! │ NuttX sentinel             │             │ Host                 │
//! │ (QEMU virt + virtio-net)   │────────────▶│ zenohd               │
//! │ src=10.0.2.30:*            │             │ 127.0.0.1:7452       │
//! │ dst=10.0.2.2:7452          │             │                      │
//! └────────────────────────────┘             └──────────────────────┘
//! ```
//!
//! Single-boot per test process: the firmware seeds the zenoh ZID from a
//! deterministic `IP+MAC` hash, so a second QEMU instance against the same
//! zenohd would collide on session ID. Tests that need multiple boots are
//! intentionally avoided.
//!
//! Prerequisites:
//! - `qemu-system-arm` on PATH (virt machine support).
//! - `arm-none-eabi-gcc` toolchain.
//! - NuttX kernel pre-built at `$NUTTX_DIR` (run `just build-nuttx-kernel`).
//! - Locally built zenohd (`just build-zenohd`).
//!
//! Tests skip gracefully when prerequisites are missing.

use rstest::rstest;
use sentinel_tests::count_pattern;
use sentinel_tests::fixtures::{
    sentinel_nuttx_binary, start_sentinel_nuttx, zenohd_nuttx,
};
use sentinel_tests::process::is_zenohd_available;
use std::path::PathBuf;
use std::process::Command;

fn is_qemu_arm_available() -> bool {
    Command::new("qemu-system-arm")
        .arg("--version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn is_arm_gcc_available() -> bool {
    Command::new("arm-none-eabi-gcc")
        .arg("--version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn nuttx_kernel_built() -> bool {
    let project = sentinel_tests::process::project_root();
    let candidate = std::env::var("NUTTX_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            project
                .parent()
                .expect("repo has parent")
                .join("nano-ros-sentinel/third-party/nuttx/nuttx")
        });
    candidate.join("staging/libc.a").exists()
}

fn require_nuttx_prereqs() -> bool {
    if !is_qemu_arm_available() {
        eprintln!("SKIP: qemu-system-arm not available");
        return false;
    }
    if !is_arm_gcc_available() {
        eprintln!("SKIP: arm-none-eabi-gcc not available");
        return false;
    }
    if !is_zenohd_available() {
        eprintln!("SKIP: zenohd not available — run `just build-zenohd`");
        return false;
    }
    if !nuttx_kernel_built() {
        eprintln!("SKIP: NuttX kernel not built — run `just build-nuttx-kernel`");
        return false;
    }
    true
}

#[test]
fn test_nuttx_kernel_present() {
    let present = nuttx_kernel_built();
    eprintln!("NuttX kernel built: {present}");
}

/// NuttX sentinel boots, opens zpico session, and reaches `Executor ready`.
///
/// This single boot covers the full pipeline: virtio-net up → SLIRP TCP
/// to zenohd → zpico session → wire_executor declares all
/// publishers/services → executor armed.
#[rstest]
fn test_nuttx_sentinel_executor_ready(
    zenohd_nuttx: u16,
    sentinel_nuttx_binary: PathBuf,
) {
    if !require_nuttx_prereqs() {
        return;
    }
    let _port = zenohd_nuttx;
    let (_sentinel, output) = start_sentinel_nuttx(&sentinel_nuttx_binary)
        .expect("NuttX sentinel failed to boot");

    let ready = count_pattern(&output, "Executor ready");
    assert!(
        ready >= 1,
        "NuttX sentinel never reached `Executor ready`\noutput:\n{output}"
    );
}
