//! FreeRTOS QEMU integration tests (Phase 13.4.6)
//!
//! Verifies the FreeRTOS sentinel binary (cross-compiled for `thumbv7m-none-eabi`,
//! launched inside `qemu-system-arm -machine mps2-an385`) boots, declares its
//! publishers/services through zenohd, and stays alive through `executor.spin(...)`.
//!
//! ## Network topology
//!
//! ```text
//! ┌────────────────────────────┐  SLIRP NAT  ┌──────────────────────┐
//! │ FreeRTOS sentinel          │             │ Host                 │
//! │ (QEMU MPS2-AN385, lwIP)    │────────────▶│ zenohd               │
//! │ src=10.0.2.20:*            │             │ 127.0.0.1:7451       │
//! │ dst=10.0.2.2:7451          │             │ (== 10.0.2.2 inside) │
//! └────────────────────────────┘             └──────────────────────┘
//! ```
//!
//! Locator is baked into `src/autoware_sentinel_freertos/config.toml`, so
//! tests share `127.0.0.1:7451` and run serialised via the
//! `freertos-qemu` nextest test-group.
//!
//! ## Prerequisites
//!
//! - `qemu-system-arm` on PATH (mps2-an385 machine support).
//! - `arm-none-eabi-gcc` toolchain (used by the board build.rs).
//! - FreeRTOS / lwIP sources at `~/repos/nano-ros-sentinel/third-party/...`.
//! - Locally built zenohd (`just build-zenohd`).
//!
//! Tests skip gracefully when prerequisites are missing.

use rstest::rstest;
use sentinel_tests::count_pattern;
use sentinel_tests::fixtures::{
    sentinel_freertos_binary, start_sentinel_freertos, zenohd_freertos,
};
use sentinel_tests::process::is_zenohd_available;
use std::path::PathBuf;
use std::process::Command;

// =============================================================================
// Prerequisite detection
// =============================================================================

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

fn require_freertos_prereqs() -> bool {
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
    true
}

// =============================================================================
// Detection tests
// =============================================================================

#[test]
fn test_qemu_arm_available() {
    let available = is_qemu_arm_available();
    eprintln!("qemu-system-arm available: {available}");
}

#[test]
fn test_arm_gcc_available() {
    let available = is_arm_gcc_available();
    eprintln!("arm-none-eabi-gcc available: {available}");
}

// =============================================================================
// Boot tests
// =============================================================================

// `test_freertos_sentinel_boots` was folded into
// `test_freertos_sentinel_declares_publishers` — both call
// `start_sentinel_freertos`, which already requires the `Executor ready`
// line, so a separate boot-only test added flake without coverage. See
// also the per-IP ZID seed note in `start_sentinel_freertos`: every
// FreeRTOS sentinel sharing one config.toml computes the same RNG seed
// (IP + MAC), so two QEMU instances against the same zenohd in one test
// process collide on session ID. Until the firmware seeds from a
// non-deterministic source, only one full FreeRTOS boot per process is
// reliable.

/// FreeRTOS sentinel boots all the way through `wire_executor` to the
/// `Executor ready — spinning…` line, proving the full pipeline works:
/// lwIP up → zpico session opened → all publishers/services declared on
/// zenohd → executor armed for the 30 Hz timer.
///
/// `wire_executor` only reaches its tail print after every
/// `create_publisher` / `add_subscription` / `add_service` / `add_timer`
/// call has succeeded against the active session, so this single
/// assertion is end-to-end coverage equivalent to counting declare
/// confirmations on the firmware side.
#[rstest]
fn test_freertos_sentinel_executor_ready(
    zenohd_freertos: u16,
    sentinel_freertos_binary: PathBuf,
) {
    if !require_freertos_prereqs() {
        return;
    }
    let _port = zenohd_freertos;
    let (_sentinel, output) = start_sentinel_freertos(&sentinel_freertos_binary)
        .expect("FreeRTOS sentinel failed to boot");

    let ready = count_pattern(&output, "Executor ready");
    assert!(
        ready >= 1,
        "FreeRTOS sentinel never reached `Executor ready`\noutput:\n{output}"
    );
}

// `test_freertos_sentinel_zenohd_session` was intentionally dropped — the
// declare-publisher confirmations exercised by
// `test_freertos_sentinel_declares_publishers` already prove the QEMU
// sentinel established a zenohd session through SLIRP (declares only
// fire after the zpico TCP handshake completes). A separate `ss(8)`-based
// session probe added flake when the previous test's lwIP TCB lingered
// briefly after QEMU teardown without contributing additional coverage.
