//! Integration test framework for Autoware Sentinel
//!
//! Provides process management, fixtures, and utilities for testing
//! the sentinel Linux binary against ROS 2 via Zenoh transport.
//!
//! Follows the same patterns as nano-ros `nros-tests`:
//! - RAII process cleanup (process group kill + PR_SET_PDEATHSIG)
//! - Ephemeral port allocation for parallel-safe zenohd instances
//! - Non-blocking I/O with poll(2) for output capture

pub mod autoware;
pub mod fixtures;
pub mod preflight;
pub mod process;
pub mod ros2;

use std::time::{Duration, Instant};

/// Error type for test utilities
#[derive(Debug, thiserror::Error)]
pub enum TestError {
    #[error("Process failed to start: {0}")]
    ProcessStart(#[from] std::io::Error),

    #[error("Process failed: {0}")]
    ProcessFailed(String),

    #[error("Timeout waiting for condition")]
    Timeout,

    #[error("Build failed: {0}")]
    BuildFailed(String),
}

pub type TestResult<T> = Result<T, TestError>;

/// Wait for a TCP port to become available
pub fn wait_for_port(port: u16, timeout: Duration) -> bool {
    let start = Instant::now();
    let addr = format!("127.0.0.1:{}", port);

    while start.elapsed() < timeout {
        if std::net::TcpStream::connect(&addr).is_ok() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    false
}

/// Count occurrences of a pattern in output
pub fn count_pattern(output: &str, pattern: &str) -> usize {
    output.matches(pattern).count()
}

/// Assert that output contains all specified patterns
pub fn assert_output_contains(output: &str, patterns: &[&str]) {
    for pattern in patterns {
        assert!(
            output.contains(pattern),
            "Expected output to contain '{}', but it was not found.\nOutput:\n{}",
            pattern,
            output
        );
    }
}
