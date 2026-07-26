//! Autoware-specific helpers for planning simulator integration tests
//!
//! Provides map path resolution, prerequisite checks, pose constants,
//! and the autonomous drive sequence for sample-map-planning.

use std::path::{Path, PathBuf};

// =============================================================================
// Map path resolution
// =============================================================================

/// Default map path (Autoware 1.5.0 test map from installed packages)
const DEFAULT_MAP_PATH: &str = "/opt/autoware/1.5.0/share/autoware_test_utils/test_map";

/// Resolve the Autoware map path.
///
/// Checks `$MAP_PATH` first, then falls back to the Autoware 1.5.0 test map.
pub fn autoware_map_path() -> PathBuf {
    if let Ok(path) = std::env::var("MAP_PATH") {
        return PathBuf::from(path);
    }
    PathBuf::from(DEFAULT_MAP_PATH)
}

/// Check if the Autoware map data is available.
pub fn is_autoware_map_available() -> bool {
    let path = autoware_map_path();
    // The map directory should contain at least lanelet2_map.osm and pointcloud_map.pcd
    path.is_dir() && path.join("lanelet2_map.osm").exists()
}

/// Skip test if map data is not available. Returns false if unavailable.
pub fn require_autoware_map() -> bool {
    if !is_autoware_map_available() {
        eprintln!(
            "Skipping test: Autoware map not found at {:?} (set $MAP_PATH or install autoware_test_utils)",
            autoware_map_path()
        );
        return false;
    }
    true
}

// =============================================================================
// play_launch prerequisite
// =============================================================================

/// Default play_launch binary path
const PLAY_LAUNCH_DEFAULT: &str = "play_launch";

/// Get the play_launch binary path.
///
/// Checks `$PLAY_LAUNCH` env var first, then falls back to PATH lookup.
pub fn play_launch_binary() -> PathBuf {
    if let Ok(path) = std::env::var("PLAY_LAUNCH") {
        return PathBuf::from(path);
    }
    PathBuf::from(PLAY_LAUNCH_DEFAULT)
}

/// Check if play_launch is installed.
pub fn is_play_launch_available() -> bool {
    let binary = play_launch_binary();
    std::process::Command::new(&binary)
        .arg("--version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Skip test if play_launch is not installed. Returns false if unavailable.
pub fn require_play_launch() -> bool {
    if !is_play_launch_available() {
        eprintln!("Skipping test: play_launch not found (install or set $PLAY_LAUNCH)");
        return false;
    }
    true
}

// =============================================================================
// Pose constants (sample-map-planning, pre-validated)
// =============================================================================

/// Initial pose for sample-map-planning
pub mod initial_pose {
    pub const X: f64 = 3752.34;
    pub const Y: f64 = 73736.09;
    pub const Z: f64 = 19.34;
    pub const QX: f64 = -0.0008;
    pub const QY: f64 = -0.0002;
    pub const QZ: f64 = -0.9584;
    pub const QW: f64 = 0.2854;
}

/// Goal pose for sample-map-planning
pub mod goal_pose {
    pub const X: f64 = 3758.96;
    pub const Y: f64 = 73689.29;
    pub const Z: f64 = 19.63;
    pub const QX: f64 = 0.0;
    pub const QY: f64 = 0.0;
    pub const QZ: f64 = 0.9663;
    pub const QW: f64 = -0.2576;
}

/// Format the initial pose as a YAML string for `ros2 topic pub`.
pub fn initial_pose_yaml() -> String {
    format!(
        "{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: map}}, \
         pose: {{pose: {{position: {{x: {x}, y: {y}, z: {z}}}, \
         orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}, \
         covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, \
         0.0, 0.25, 0.0, 0.0, 0.0, 0.0, \
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
         0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}}}",
        x = initial_pose::X,
        y = initial_pose::Y,
        z = initial_pose::Z,
        qx = initial_pose::QX,
        qy = initial_pose::QY,
        qz = initial_pose::QZ,
        qw = initial_pose::QW,
    )
}

/// Format the goal pose as a YAML string for the set_route_points service request.
pub fn goal_pose_yaml() -> String {
    format!(
        "{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: map}}, \
         goal: {{position: {{x: {x}, y: {y}, z: {z}}}, \
         orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}, \
         waypoints: []}}",
        x = goal_pose::X,
        y = goal_pose::Y,
        z = goal_pose::Z,
        qx = goal_pose::QX,
        qy = goal_pose::QY,
        qz = goal_pose::QZ,
        qw = goal_pose::QW,
    )
}

// =============================================================================
// Readiness topics
// =============================================================================

/// Topics that must be active before starting the autonomous drive sequence.
pub const READINESS_TOPICS: &[&str] = &[
    "/map/vector_map",
    "/api/operation_mode/state",
    "/api/routing/state",
];

/// Topics that must be active after setting the initial pose (localization ready).
pub const LOCALIZATION_TOPICS: &[&str] = &["/tf", "/localization/kinematic_state"];

// =============================================================================
// Autonomous drive sequence helpers
// =============================================================================

/// Check all prerequisites for planning simulator tests.
///
/// Returns false (and prints skip message) if any prerequisite is missing.
pub fn require_planning_simulator_prerequisites() -> bool {
    // Phase 15.1 — hard-fails with remediation text unless
    // SENTINEL_ALLOW_SKIP=1. The old body returned false on every missing
    // prerequisite, which read as a pass.
    crate::preflight::planning_simulator()
}

/// Check if a file looks like a valid play_launch record.json.
///
/// Validates that the file exists and contains the expected top-level keys.
pub fn is_valid_record(path: &Path) -> bool {
    if !path.exists() {
        return false;
    }
    match std::fs::read_to_string(path) {
        Ok(content) => {
            // Quick structural check — must have "node" and "container" keys
            content.contains("\"node\"") && content.contains("\"container\"")
        }
        Err(_) => false,
    }
}
