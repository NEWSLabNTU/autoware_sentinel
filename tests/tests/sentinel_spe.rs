//! SPE sentinel smoke tests (Phase 11.3.E).
//!
//! Covers the POSIX dev path of `autoware_sentinel_spe` — the Linux
//! process built from `src/autoware_sentinel_spe/` that exercises the
//! same `wire_executor` body the Cortex-R5F firmware will eventually
//! link. Validates boot, executor wiring, and that the reduced feature
//! set (`comp-mrm` + `comp-engagement`) declares its expected publishers
//! visibly to ROS 2 over zenohd.

use sentinel_tests::fixtures::{
    ZenohRouter, require_ros2_autoware, sentinel_spe_binary, start_sentinel_spe, zenohd_unique,
};
use sentinel_tests::ros2::{ros2_env_setup_with_locator, wait_for_topics};

use rstest::rstest;
use std::path::PathBuf;
use std::time::Duration;

/// SPE sentinel boots and reaches `Executor ready`.
#[rstest]
fn test_sentinel_spe_starts(zenohd_unique: ZenohRouter, sentinel_spe_binary: PathBuf) {
    let locator = zenohd_unique.locator();
    let _spe = start_sentinel_spe(&sentinel_spe_binary, &locator)
        .expect("SPE sentinel failed to start");
    // start_sentinel_spe waited for "Executor ready"; drop kills proc.
}

/// SPE-side comp-mrm + comp-engagement publishers visible via `ros2
/// topic list`. Picks two diagnostic topics every reduced-set boot
/// declares: the gated `/control/command/control_cmd` republish and the
/// MRM operator state.
#[rstest]
fn test_sentinel_spe_topics_visible(
    zenohd_unique: ZenohRouter,
    sentinel_spe_binary: PathBuf,
) {
    if !require_ros2_autoware() {
        return;
    }
    let locator = zenohd_unique.locator();
    let _spe = start_sentinel_spe(&sentinel_spe_binary, &locator)
        .expect("SPE sentinel failed to start");

    let env_setup = ros2_env_setup_with_locator(&locator);
    wait_for_topics(
        &[
            "/control/command/control_cmd",
            "/system/mrm/emergency_stop/status",
            "/system/emergency_holding",
        ],
        &env_setup,
        Duration::from_secs(20),
    )
    .expect("expected SPE topics to appear via rmw_zenoh");
}
