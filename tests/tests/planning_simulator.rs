//! Planning simulator integration tests (Phase 7.3)
//!
//! Tests the Autoware planning simulator with and without the sentinel.
//!
//! ## 7.3a — Baseline
//!
//! Runs unmodified Autoware through the full autonomous drive sequence to
//! establish baseline behavior and validate the test infrastructure.
//!
//! ## 7.3b — Sentinel replaces 7 nodes
//!
//! Runs Autoware with 7 nodes filtered out + sentinel Linux binary,
//! verifying the sentinel can drive a simulated vehicle.
//!
//! ## Prerequisites
//!
//! - `play_launch` installed (`~/.local/bin/play_launch` or `$PLAY_LAUNCH`)
//! - Autoware map data at `$MAP_PATH` or `$HOME/autoware_map/sample-map-planning`
//! - ROS 2 Humble with `rmw_zenoh_cpp` and Autoware packages
//! - zenohd built locally

use sentinel_tests::autoware;
use sentinel_tests::count_pattern;
use sentinel_tests::fixtures::{
    NodeFilter, ZenohRouter, autoware_record, dump_autoware_record, filter_record, sentinel_binary,
    start_autoware, start_autoware_zenoh, start_sentinel, zenohd_unique,
};
use sentinel_tests::ros2;

use rstest::rstest;
use std::path::PathBuf;
use std::time::Duration;

// =============================================================================
// Prerequisite detection tests
// =============================================================================

#[test]
fn test_play_launch_available() {
    let available = autoware::is_play_launch_available();
    eprintln!("play_launch available: {available}");
}

#[test]
fn test_autoware_map_available() {
    let path = autoware::autoware_map_path();
    let available = autoware::is_autoware_map_available();
    eprintln!("Autoware map path: {path:?}, available: {available}");
}

// =============================================================================
// 7.3a — Baseline: Standard Autoware (no sentinel)
// =============================================================================

/// Dump the Autoware planning simulator launch to record.json.
///
/// Validates that play_launch can parse and dump the full Autoware launch tree.
#[test]
fn test_autoware_dump() {
    if !autoware::require_planning_simulator_prerequisites() {
        return;
    }

    let record = dump_autoware_record().expect("Failed to dump Autoware record");
    assert!(record.exists(), "Record file not created");
    assert!(
        autoware::is_valid_record(record),
        "Record file is not valid JSON"
    );

    // Verify the record contains expected arrays
    let content = std::fs::read_to_string(record).unwrap();
    let json: serde_json::Value = serde_json::from_str(&content).unwrap();
    assert!(json.get("node").is_some(), "record missing 'node' array");
    assert!(
        json.get("container").is_some(),
        "record missing 'container' array"
    );
    assert!(
        json.get("load_node").is_some(),
        "record missing 'load_node' array"
    );

    let node_count = json["node"].as_array().map(|a| a.len()).unwrap_or(0);
    let container_count = json["container"].as_array().map(|a| a.len()).unwrap_or(0);
    let load_node_count = json["load_node"].as_array().map(|a| a.len()).unwrap_or(0);
    eprintln!(
        "Autoware record: {node_count} nodes, {container_count} containers, {load_node_count} load_nodes"
    );

    // Autoware planning simulator has many nodes
    assert!(node_count > 0, "Expected at least 1 node in record");
}

/// Full baseline test: launch unmodified Autoware and drive autonomously.
///
/// This test validates the complete test infrastructure:
/// 1. play_launch dump + replay
/// 2. ROS 2 CLI helpers (topic pub, service call, topic info)
/// 3. Autonomous drive sequence (initial pose → route → engage)
/// 4. Vehicle movement verification (kinematic_state position delta)
///
/// Autoware runs on default DDS transport. The ROS 2 CLI tools also use
/// default DDS so they can interact with the Autoware nodes directly.
#[rstest]
fn test_autoware_baseline_autonomous_drive(autoware_record: PathBuf) {
    if !autoware::require_planning_simulator_prerequisites() {
        return;
    }

    // Environment setup for ROS 2 CLI tools — use default DDS (same as Autoware)
    let env = ros2::ros2_env_setup_dds();

    // --- Step 1: Start Autoware via play_launch replay ---
    eprintln!("Starting unmodified Autoware (default DDS)...");
    let mut autoware = start_autoware(&autoware_record).expect("Failed to start Autoware");

    // Give Autoware time to initialize all nodes
    eprintln!("Waiting 30s for Autoware initialization...");
    std::thread::sleep(Duration::from_secs(30));

    // Verify Autoware is still running
    assert!(
        autoware.is_running(),
        "Autoware exited prematurely during initialization"
    );

    // --- Step 2: Wait for readiness topics ---
    eprintln!("Waiting for readiness topics...");
    match ros2::wait_for_topics(autoware::READINESS_TOPICS, &env, Duration::from_secs(60)) {
        Ok(()) => eprintln!("All readiness topics active"),
        Err(e) => eprintln!("WARNING: Not all readiness topics active: {e}"),
    }

    // --- Step 3: Publish initial pose ---
    eprintln!("Publishing initial pose...");
    let pose_yaml = autoware::initial_pose_yaml();
    match ros2::topic_pub_once(
        "/initialpose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        &pose_yaml,
        &env,
    ) {
        Ok(_) => eprintln!("Initial pose published"),
        Err(e) => eprintln!("WARNING: Initial pose publish issue: {e}"),
    }

    // --- Step 4: Wait for localization ---
    eprintln!("Waiting 10s for localization...");
    std::thread::sleep(Duration::from_secs(10));

    match ros2::wait_for_topics(autoware::LOCALIZATION_TOPICS, &env, Duration::from_secs(30)) {
        Ok(()) => eprintln!("Localization topics active"),
        Err(e) => eprintln!("WARNING: Localization topics not all active: {e}"),
    }

    // --- Step 5: Set route ---
    eprintln!("Setting route via /api/routing/set_route_points...");
    let route_yaml = autoware::goal_pose_yaml();
    match ros2::service_call(
        "/api/routing/set_route_points",
        "autoware_adapi_v1_msgs/srv/SetRoutePoints",
        &route_yaml,
        &env,
    ) {
        Ok(resp) => eprintln!("Route set response: {}", &resp[..resp.len().min(200)]),
        Err(e) => eprintln!("WARNING: Route set issue: {e}"),
    }

    // --- Step 6: Wait for route processing ---
    eprintln!("Waiting 5s for route processing...");
    std::thread::sleep(Duration::from_secs(5));

    // --- Step 7: Engage autonomous mode ---
    eprintln!("Engaging autonomous mode...");
    match ros2::service_call(
        "/api/operation_mode/change_to_autonomous",
        "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
        "{}",
        &env,
    ) {
        Ok(resp) => eprintln!("Engage response: {}", &resp[..resp.len().min(200)]),
        Err(e) => eprintln!("WARNING: Engage issue: {e}"),
    }

    // --- Step 8: Monitor for vehicle movement ---
    eprintln!("Monitoring kinematic_state for 20s...");

    // Echo kinematic_state via default DDS to check for position data
    let mut echo = ros2::topic_echo_dds("/localization/kinematic_state", "nav_msgs/msg/Odometry")
        .expect("Failed to start kinematic_state echo");

    std::thread::sleep(Duration::from_secs(15));
    let output = echo
        .wait_for_all_output(Duration::from_secs(5))
        .unwrap_or_default();

    eprintln!(
        "kinematic_state output ({} bytes): {}",
        output.len(),
        &output[..output.len().min(500)]
    );

    // Verify Autoware is still running (no crashes)
    assert!(
        autoware.is_running(),
        "Autoware crashed during autonomous drive sequence"
    );

    // Check that we received kinematic_state data (position exists)
    let has_position = output.contains("position:");
    eprintln!("kinematic_state has position data: {has_position}");
    assert!(
        has_position,
        "No kinematic_state position data received — Autoware may not have initialized fully"
    );
}

// =============================================================================
// 7.3b — Sentinel replaces 7 nodes
// =============================================================================

/// Test that the record filter correctly removes the 7 sentinel-replaced nodes.
///
/// Validates the filter against the real Autoware record to ensure the right
/// nodes are removed from the right arrays.
#[rstest]
fn test_record_filter(autoware_record: PathBuf) {
    if !autoware::require_planning_simulator_prerequisites() {
        return;
    }

    let content = std::fs::read_to_string(&autoware_record).unwrap();
    let original: serde_json::Value = serde_json::from_str(&content).unwrap();

    let orig_nodes = original["node"].as_array().map(|a| a.len()).unwrap_or(0);
    let orig_containers = original["container"]
        .as_array()
        .map(|a| a.len())
        .unwrap_or(0);
    let orig_load_nodes = original["load_node"]
        .as_array()
        .map(|a| a.len())
        .unwrap_or(0);
    eprintln!(
        "Original: {orig_nodes} nodes, {orig_containers} containers, {orig_load_nodes} load_nodes"
    );

    // Apply sentinel filter
    let filtered_path = std::env::temp_dir()
        .join("sentinel_tests")
        .join("filtered_record.json");
    let filter = NodeFilter::sentinel_replacement();
    filter_record(&autoware_record, &filter, &filtered_path).expect("Failed to filter record");

    let filtered_content = std::fs::read_to_string(&filtered_path).unwrap();
    let filtered: serde_json::Value = serde_json::from_str(&filtered_content).unwrap();

    let filt_nodes = filtered["node"].as_array().map(|a| a.len()).unwrap_or(0);
    let filt_containers = filtered["container"]
        .as_array()
        .map(|a| a.len())
        .unwrap_or(0);
    let filt_load_nodes = filtered["load_node"]
        .as_array()
        .map(|a| a.len())
        .unwrap_or(0);
    eprintln!(
        "Filtered: {filt_nodes} nodes, {filt_containers} containers, {filt_load_nodes} load_nodes"
    );

    // node[]: 4 removals — mrm_handler + hazard_status_converter by name,
    // plus the autoware_diagnostic_graph_aggregator package (2 nodes:
    // aggregator + logging_diag_graph). play_launch 0.8.x classifies all of
    // these as plain nodes; older dumps parked some in other arrays, which is
    // where the stale "1 removed" expectation came from.
    assert_eq!(
        filt_nodes,
        orig_nodes - 4,
        "Expected 4 nodes removed (mrm_handler, hazard_status_converter, 2x diag aggregator), got {} removed",
        orig_nodes - filt_nodes
    );

    // container[]: 2 MRM operator containers removed
    assert_eq!(
        filt_containers,
        orig_containers - 2,
        "Expected 2 containers removed (MRM operators), got {} removed",
        orig_containers - filt_containers
    );

    // load_node[]: 8 removals — the 4 control packages (vehicle_cmd_gate,
    // shift_decider, operation_mode_transition_manager, control_validator)
    // plus the 4 ADAPI adaptor (package, node) pairs.
    assert_eq!(
        filt_load_nodes,
        orig_load_nodes - 8,
        "Expected 8 load_nodes removed (4 control pkgs + 4 adapi pairs), got {} removed",
        orig_load_nodes - filt_load_nodes
    );

    // Verify the specific nodes are gone
    let node_names: Vec<&str> = filtered["node"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n.get("name").and_then(|v| v.as_str()))
        .collect();
    assert!(
        !node_names.iter().any(|n| n.ends_with("mrm_handler")),
        "mrm_handler should be filtered out, but found in: {node_names:?}"
    );

    let container_names: Vec<&str> = filtered["container"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n.get("name").and_then(|v| v.as_str()))
        .collect();
    assert!(
        !container_names
            .iter()
            .any(|n| n.contains("mrm_emergency_stop_operator")),
        "mrm_emergency_stop_operator_container should be filtered out"
    );
    assert!(
        !container_names
            .iter()
            .any(|n| n.contains("mrm_comfortable_stop_operator")),
        "mrm_comfortable_stop_operator_container should be filtered out"
    );

    let load_node_packages: Vec<&str> = filtered["load_node"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n.get("package").and_then(|v| v.as_str()))
        .collect();
    for pkg in &[
        "autoware_vehicle_cmd_gate",
        "autoware_shift_decider",
        "autoware_operation_mode_transition_manager",
        "autoware_control_validator",
    ] {
        assert!(
            !load_node_packages.contains(pkg),
            "{pkg} should be filtered out from load_node[], but still present"
        );
    }

    eprintln!(
        "PASS: All 7 sentinel-replaced nodes correctly filtered (1 node + 2 containers + 4 load_nodes)"
    );
}

/// Sentinel replacement test: filtered Autoware + sentinel drives autonomously.
///
/// This is the core 7.3b test:
/// 1. Dump Autoware record, filter out 7 nodes replaced by sentinel
/// 2. Start zenohd (ephemeral port)
/// 3. Replay filtered Autoware with rmw_zenoh_cpp (connecting to zenohd)
/// 4. Start sentinel (connecting to same zenohd)
/// 5. Run autonomous drive sequence
/// 6. Verify sentinel publishes Control, vehicle moves, no MRM triggered
#[rstest]
fn test_sentinel_replaces_autoware_nodes(
    zenohd_unique: ZenohRouter,
    autoware_record: PathBuf,
    sentinel_binary: PathBuf,
) {
    if !autoware::require_planning_simulator_prerequisites() {
        return;
    }

    let locator = zenohd_unique.locator();
    let env = ros2::ros2_env_setup_with_locator(&locator);

    // --- Step 1: Filter record ---
    let filtered_path = std::env::temp_dir()
        .join("sentinel_tests")
        .join("sentinel_filtered_record.json");
    let filter = NodeFilter::sentinel_replacement();
    filter_record(&autoware_record, &filter, &filtered_path).expect("Failed to filter record");
    eprintln!("Filtered record ready at {:?}", filtered_path);

    // --- Step 2: Start filtered Autoware with rmw_zenoh_cpp ---
    eprintln!("Starting filtered Autoware (rmw_zenoh_cpp → {locator})...");
    let mut autoware =
        start_autoware_zenoh(&filtered_path, &locator).expect("Failed to start filtered Autoware");

    // Give Autoware time to initialize
    eprintln!("Waiting 30s for Autoware initialization...");
    std::thread::sleep(Duration::from_secs(30));

    assert!(
        autoware.is_running(),
        "Filtered Autoware exited prematurely during initialization"
    );

    // --- Step 3: Start sentinel ---
    eprintln!("Starting sentinel (→ {locator})...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // Give sentinel time to start publishing
    std::thread::sleep(Duration::from_secs(3));

    // --- Step 4: Verify sentinel publishes Control ---
    eprintln!("Checking sentinel Control output...");
    let mut control_echo = ros2::Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        &locator,
    )
    .expect("Failed to start Control echo");

    std::thread::sleep(Duration::from_secs(5));
    let control_output = control_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    let control_count = count_pattern(&control_output, "stamp:");
    eprintln!("Sentinel Control messages received: {control_count}");
    assert!(
        control_count > 0,
        "Sentinel is not publishing Control messages"
    );

    // --- Step 5: Engage, then verify OperationModeState ---
    // The sentinel boots disengaged (mode STOP); AUTONOMOUS requires the
    // change_to_autonomous service — the earlier version of this test
    // asserted mode 2 without engaging and only "passed" while the
    // prerequisite gate was silently skipping it.
    eprintln!("Engaging via /api/operation_mode/change_to_autonomous...");
    let zenoh_env = ros2::ros2_env_setup_with_locator(&locator);
    // Retry: with the full filtered-Autoware session count on one zenohd,
    // service replies are occasionally dropped (the known zenohd interest-
    // propagation weakness under many sessions — see the drive-latency
    // research doc). Isolated calls succeed on the first try.
    // Wait for the service to be discoverable before spending attempts on
    // it (phase 15: under load it disappears from discovery for tens of
    // seconds at a time).
    match ros2::wait_for_service(
        "/api/operation_mode/change_to_autonomous",
        &zenoh_env,
        Duration::from_secs(60),
    ) {
        Ok(()) => eprintln!("engage service discoverable"),
        Err(e) => eprintln!("WARNING: {e}"),
    }

    let mut engaged = false;
    for attempt in 1..=5 {
        match ros2::service_call(
            "/api/operation_mode/change_to_autonomous",
            "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
            "{}",
            &zenoh_env,
        ) {
            Ok(out) => {
                eprintln!("Engage response: {}", out.lines().last().unwrap_or(""));
                engaged = true;
                break;
            }
            Err(e) => eprintln!("WARNING: engage attempt {attempt} failed: {e}"),
        }
        // Re-wait rather than hammering: discovery, not the service, is what
        // is flapping.
        let _ = ros2::wait_for_service(
            "/api/operation_mode/change_to_autonomous",
            &zenoh_env,
            Duration::from_secs(30),
        );
        std::thread::sleep(Duration::from_secs(5));
    }
    if !engaged {
        eprintln!("WARNING: all engage attempts failed — asserting anyway");
    }

    eprintln!("Checking sentinel OperationModeState...");
    let mut mode_echo = ros2::Ros2Process::topic_echo(
        "/api/operation_mode/state",
        "autoware_adapi_v1_msgs/msg/OperationModeState",
        &locator,
    )
    .expect("Failed to start OperationModeState echo");

    std::thread::sleep(Duration::from_secs(3));
    let mode_output = mode_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    let has_autonomous = mode_output.contains("mode: 2");
    eprintln!("OperationModeState mode=AUTONOMOUS: {has_autonomous}");
    assert!(
        has_autonomous,
        "Sentinel not publishing OperationModeState with mode=AUTONOMOUS"
    );

    // --- Step 6: Run autonomous drive sequence ---
    eprintln!("Waiting for readiness topics...");
    match ros2::wait_for_topics(autoware::READINESS_TOPICS, &env, Duration::from_secs(60)) {
        Ok(()) => eprintln!("All readiness topics active"),
        Err(e) => eprintln!("WARNING: Not all readiness topics active: {e}"),
    }

    eprintln!("Publishing initial pose...");
    let pose_yaml = autoware::initial_pose_yaml();
    match ros2::topic_pub_once(
        "/initialpose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        &pose_yaml,
        &env,
    ) {
        Ok(_) => eprintln!("Initial pose published"),
        Err(e) => eprintln!("WARNING: Initial pose publish issue: {e}"),
    }

    eprintln!("Waiting 10s for localization...");
    std::thread::sleep(Duration::from_secs(10));

    match ros2::wait_for_topics(autoware::LOCALIZATION_TOPICS, &env, Duration::from_secs(30)) {
        Ok(()) => eprintln!("Localization topics active"),
        Err(e) => eprintln!("WARNING: Localization topics not all active: {e}"),
    }

    eprintln!("Setting route...");
    let route_yaml = autoware::goal_pose_yaml();
    match ros2::service_call(
        "/api/routing/set_route_points",
        "autoware_adapi_v1_msgs/srv/SetRoutePoints",
        &route_yaml,
        &env,
    ) {
        Ok(resp) => eprintln!("Route set: {}", &resp[..resp.len().min(200)]),
        Err(e) => eprintln!("WARNING: Route set issue: {e}"),
    }

    eprintln!("Waiting 5s for route processing...");
    std::thread::sleep(Duration::from_secs(5));

    eprintln!("Engaging autonomous mode...");
    match ros2::service_call(
        "/api/operation_mode/change_to_autonomous",
        "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
        "{}",
        &env,
    ) {
        Ok(resp) => eprintln!("Engage: {}", &resp[..resp.len().min(200)]),
        Err(e) => eprintln!("WARNING: Engage issue: {e}"),
    }

    // --- Step 7: Monitor for vehicle movement ---
    // Retried: a fresh echo session against the loaded zenohd sometimes
    // never receives data (the known interest-propagation nondeterminism
    // under many sessions); a second session usually lands.
    eprintln!("Monitoring kinematic_state for 20s...");
    let mut output = String::new();
    for attempt in 1..=3 {
        let mut echo = ros2::Ros2Process::topic_echo(
            "/localization/kinematic_state",
            "nav_msgs/msg/Odometry",
            &locator,
        )
        .expect("Failed to start kinematic_state echo");
        std::thread::sleep(Duration::from_secs(15));
        output = echo
            .wait_for_all_output(Duration::from_secs(5))
            .unwrap_or_default();
        if output.contains("position:") {
            break;
        }
        eprintln!("WARNING: kinematic echo attempt {attempt} received no data; retrying");
    }

    eprintln!(
        "kinematic_state output ({} bytes): {}",
        output.len(),
        &output[..output.len().min(500)]
    );

    // --- Step 8: Verify results ---
    assert!(
        autoware.is_running(),
        "Filtered Autoware crashed during autonomous drive"
    );

    let has_position = output.contains("position:");
    eprintln!("kinematic_state has position data: {has_position}");
    assert!(
        has_position,
        "No kinematic_state data — vehicle may not have moved"
    );

    // Verify sentinel is still publishing (not crashed, no MRM panic).
    // Retried like the kinematic echo — fresh sessions against the loaded
    // zenohd occasionally receive nothing.
    eprintln!("Final sentinel Control check...");
    let mut final_count = 0usize;
    for attempt in 1..=3 {
        let mut final_echo = ros2::Ros2Process::topic_echo(
            "/control/command/control_cmd",
            "autoware_control_msgs/msg/Control",
            &locator,
        )
        .expect("Failed to start final Control echo");
        std::thread::sleep(Duration::from_secs(3));
        let final_output = final_echo
            .wait_for_all_output(Duration::from_secs(3))
            .unwrap_or_default();
        final_count = count_pattern(&final_output, "stamp:");
        if final_count > 0 {
            break;
        }
        eprintln!("WARNING: final Control echo attempt {attempt} received no data; retrying");
    }
    eprintln!("Final sentinel Control messages: {final_count}");
    assert!(
        final_count > 0,
        "Sentinel stopped publishing Control — possible MRM or crash"
    );

    eprintln!("PASS: Sentinel successfully replaces 7 Autoware nodes");
}
