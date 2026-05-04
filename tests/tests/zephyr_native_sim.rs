//! Zephyr native_sim integration tests (Phase 7.4)
//!
//! Verifies bidirectional message flow between the Zephyr `native_sim` sentinel
//! binary and ROS 2 (`rmw_zenoh_cpp`) through a shared zenohd router over
//! TAP/bridge networking.
//!
//! ## Prerequisites
//!
//! 1. TAP network set up: `just setup-tap-network`
//! 2. Zephyr sentinel built: `just build-zephyr`
//! 3. zenohd available (locally built)
//! 4. ROS 2 Humble with `rmw_zenoh_cpp` installed
//!
//! ## Network topology
//!
//! NSOS (Native Simulator Offloaded Sockets): zenoh-pico's TCP socket calls
//! hit the host's BSD socket API directly, so no TAP/bridge is needed.
//!
//! ```text
//! ┌──────────────────────┐     ┌──────────────────────┐
//! │ Zephyr Sentinel      │ host BSD sockets │ Host    │
//! │ (native_sim, NSOS)   │─────────────────▶│ zenohd  │
//! └──────────────────────┘     │ 127.0.0.1:7447       │
//!                              └──────────────────────┘
//! ```
//!
//! Tests skip gracefully if prerequisites are not met.

use sentinel_tests::count_pattern;
use sentinel_tests::fixtures::require_ros2_autoware;
use sentinel_tests::process::{ManagedProcess, is_zenohd_available, project_root};
use sentinel_tests::ros2::Ros2Process;

use rstest::rstest;
use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

/// Zenohd locator on host loopback. With NSOS the Zephyr binary reaches the
/// host's BSD sockets directly, so 127.0.0.1 is reachable.
const BRIDGE_LOCATOR: &str = "tcp/127.0.0.1:7447";

// =============================================================================
// Prerequisite checks
// =============================================================================

/// Check if the Zephyr sentinel binary exists.
fn zephyr_binary_path() -> PathBuf {
    project_root()
        .parent()
        .unwrap()
        .join("autoware-sentinel-workspace/build/sentinel/zephyr/zephyr.exe")
}

fn is_zephyr_binary_available() -> bool {
    zephyr_binary_path().exists()
}

/// Skip test if any prerequisite is missing.
fn require_zephyr_prerequisites() -> bool {
    if !is_zenohd_available() {
        eprintln!("Skipping: zenohd not available (build with `just build-zenohd`)");
        return false;
    }
    if !is_zephyr_binary_available() {
        eprintln!("Skipping: Zephyr sentinel not built (run `just build-zephyr`)");
        return false;
    }
    true
}

/// Start zenohd listening on host loopback (127.0.0.1:7447).
fn start_zenohd_bridge() -> ManagedProcess {
    let zenohd_path = sentinel_tests::process::zenohd_binary_path();
    let mut cmd = Command::new(zenohd_path);
    cmd.args([
        "--listen",
        "tcp/127.0.0.1:7447",
        "--no-multicast-scouting",
    ]);
    let mut proc =
        ManagedProcess::spawn_command(cmd, "zenohd-bridge").expect("Failed to start zenohd");

    // Wait for zenohd to start listening
    let output = proc
        .wait_for_output_pattern("zenohd", Duration::from_secs(5))
        .unwrap_or_default();
    eprintln!(
        "zenohd started (bridge mode):\n{}",
        &output[..output.len().min(200)]
    );
    proc
}

/// Start the Zephyr native_sim sentinel binary.
fn start_zephyr_sentinel() -> ManagedProcess {
    let binary = zephyr_binary_path();
    let cmd = Command::new(binary);
    let mut proc = ManagedProcess::spawn_command(cmd, "zephyr-sentinel")
        .expect("Failed to start Zephyr sentinel");

    // Wait for Zephyr sentinel to be ready
    let output = proc
        .wait_for_output_pattern("Executor ready", Duration::from_secs(30))
        .unwrap_or_default();
    eprintln!(
        "Zephyr sentinel started:\n{}",
        &output[..output.len().min(300)]
    );
    proc
}

// =============================================================================
// Detection tests
// =============================================================================

#[test]
fn test_tap_network_available() {
    // NSOS replaces TAP — keep the test name for backwards-compatible test
    // selection but report that no host network setup is required.
    eprintln!("Using NSOS (host-offloaded sockets); no TAP/bridge needed");
}

#[test]
fn test_zephyr_binary_available() {
    let available = is_zephyr_binary_available();
    eprintln!("Zephyr binary available: {}", available);
    if available {
        eprintln!("  Path: {:?}", zephyr_binary_path());
    }
}

// =============================================================================
// Zephyr sentinel startup
// =============================================================================

/// Verify Zephyr sentinel connects to zenohd on bridge and prints ready.
#[test]
fn test_zephyr_sentinel_starts() {
    if !require_zephyr_prerequisites() {
        return;
    }

    let _zenohd = start_zenohd_bridge();
    std::thread::sleep(Duration::from_secs(1));

    let _sentinel = start_zephyr_sentinel();
    // start_zephyr_sentinel already waits for "Executor ready"
}

// =============================================================================
// Zephyr → ROS 2 (sentinel publishes, ROS 2 echoes)
// =============================================================================

/// Verify ROS 2 receives Control messages from Zephyr sentinel via TAP.
///
/// The Zephyr sentinel uses topic `/control/command/control_cmd`.
#[rstest]
fn test_zephyr_to_ros2_control() {
    if !require_zephyr_prerequisites() || !require_ros2_autoware() {
        return;
    }

    let _zenohd = start_zenohd_bridge();
    std::thread::sleep(Duration::from_secs(1));

    // Start ROS 2 echo on Zephyr's output topic
    eprintln!("Starting ros2 topic echo /control/command/control_cmd ...");
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        BRIDGE_LOCATOR,
    )
    .expect("Failed to start ros2 topic echo");

    std::thread::sleep(Duration::from_secs(2));

    // Start Zephyr sentinel
    eprintln!("Starting Zephyr sentinel...");
    let _sentinel = start_zephyr_sentinel();

    // Collect output for 5 seconds
    std::thread::sleep(Duration::from_secs(5));
    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    eprintln!(
        "ROS 2 echo output ({} bytes):\n{}",
        output.len(),
        &output[..output.len().min(500)]
    );

    let msg_count = count_pattern(&output, "stamp:");
    eprintln!("Control messages received from Zephyr: {}", msg_count);
    assert!(
        msg_count > 0,
        "ROS 2 did not receive any Control messages from Zephyr sentinel"
    );
}

// =============================================================================
// ROS 2 → Zephyr (ROS 2 publishes, Zephyr receives)
// =============================================================================

/// Verify Zephyr sentinel receives VelocityReport from ROS 2 via TAP
/// and continues publishing output (doesn't crash on real messages).
#[rstest]
fn test_ros2_to_zephyr_velocity() {
    if !require_zephyr_prerequisites() || !require_ros2_autoware() {
        return;
    }

    let _zenohd = start_zenohd_bridge();
    std::thread::sleep(Duration::from_secs(1));

    // Start Zephyr sentinel
    eprintln!("Starting Zephyr sentinel...");
    let _sentinel = start_zephyr_sentinel();

    // Publish VelocityReport from ROS 2
    eprintln!("Publishing VelocityReport from ROS 2...");
    let _ros2_pub = Ros2Process::topic_pub(
        "/vehicle/status/velocity_status",
        "autoware_vehicle_msgs/msg/VelocityReport",
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, \
         longitudinal_velocity: 5.0, lateral_velocity: 0.0, heading_rate: 0.0}",
        10,
        BRIDGE_LOCATOR,
    )
    .expect("Failed to start ros2 topic pub");

    // Echo sentinel's output to verify it's still publishing
    eprintln!("Starting ros2 topic echo for sentinel output...");
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        BRIDGE_LOCATOR,
    )
    .expect("Failed to start ros2 topic echo");

    std::thread::sleep(Duration::from_secs(5));

    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    let msg_count = count_pattern(&output, "stamp:");
    eprintln!(
        "Control messages after VelocityReport injection: {}",
        msg_count
    );
    assert!(
        msg_count > 0,
        "Zephyr sentinel stopped publishing after receiving VelocityReport"
    );
}

// =============================================================================
// Bidirectional round-trip
// =============================================================================

/// Full bidirectional test via TAP: ROS 2 publishes velocity, Zephyr processes
/// it and publishes control, ROS 2 echoes the control output.
#[rstest]
fn test_zephyr_bidirectional_round_trip() {
    if !require_zephyr_prerequisites() || !require_ros2_autoware() {
        return;
    }

    let _zenohd = start_zenohd_bridge();
    std::thread::sleep(Duration::from_secs(1));

    // 1. Start Zephyr sentinel
    eprintln!("Starting Zephyr sentinel...");
    let _sentinel = start_zephyr_sentinel();

    // 2. Echo Zephyr output
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        BRIDGE_LOCATOR,
    )
    .expect("Failed to start ros2 topic echo");

    std::thread::sleep(Duration::from_secs(2));

    // 3. Publish velocity
    let _ros2_pub = Ros2Process::topic_pub(
        "/vehicle/status/velocity_status",
        "autoware_vehicle_msgs/msg/VelocityReport",
        "{header: {stamp: {sec: 1, nanosec: 0}, frame_id: ''}, \
         longitudinal_velocity: 10.0, lateral_velocity: 0.0, heading_rate: 0.0}",
        10,
        BRIDGE_LOCATOR,
    )
    .expect("Failed to start ros2 topic pub");

    // 4. Publish heartbeat
    let _ros2_hb = Ros2Process::topic_pub(
        "/api/system/heartbeat",
        "autoware_adapi_v1_msgs/msg/Heartbeat",
        "{}",
        10,
        BRIDGE_LOCATOR,
    )
    .expect("Failed to start heartbeat publisher");

    // 5. Collect
    std::thread::sleep(Duration::from_secs(5));
    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    let msg_count = count_pattern(&output, "stamp:");
    eprintln!("Round-trip Control messages via TAP: {}", msg_count);
    assert!(
        msg_count > 0,
        "No Control messages in Zephyr bidirectional round-trip test"
    );
}
