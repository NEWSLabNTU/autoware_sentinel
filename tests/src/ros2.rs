//! ROS 2 process helpers for integration tests
//!
//! Provides wrappers for running `ros2 topic pub/echo` and other CLI commands
//! with `rmw_zenoh_cpp` transport, matching the nano-ros nros-tests patterns.

use crate::process::{kill_process_group, set_new_process_group};
use crate::{TestError, TestResult};
use std::process::{Child, Command, Stdio};
use std::time::Duration;

/// Default ROS 2 distro
pub const DEFAULT_ROS_DISTRO: &str = "humble";

/// Check if ROS 2 is available
pub fn is_ros2_available() -> bool {
    Command::new("bash")
        // Phase 15.2 — unset RMW_IMPLEMENTATION: with a broken/absent
        // overlay selected, EVERY ros2 invocation fails, which made the
        // preflight report "ROS 2 missing" on a machine that has it.
        .args([
            "-c",
            "unset RMW_IMPLEMENTATION; source /opt/ros/humble/setup.bash && ros2 --help",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if rmw_zenoh_cpp is available
pub fn is_rmw_zenoh_available() -> bool {
    let overlay = crate::process::project_root()
        .join("external/rmw_zenoh_ws/install/local_setup.bash");
    let script = format!(
        "unset RMW_IMPLEMENTATION; source /opt/ros/humble/setup.bash && \
         source {} 2>/dev/null; ros2 pkg list | grep -q rmw_zenoh_cpp",
        overlay.display()
    );
    return Command::new("bash")
        .args(["-c", &script])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    #[allow(unreachable_code)]
    Command::new("bash")
        .args([
            "-c",
            // The overlay is what we are testing for; source it explicitly
            // and keep the ambient RMW selection out of the way.
            "unset RMW_IMPLEMENTATION; source /opt/ros/humble/setup.bash && \
             source \"$(dirname \"$0\")\"/../external/rmw_zenoh_ws/install/local_setup.bash 2>/dev/null; \
             ros2 pkg list | grep -q rmw_zenoh_cpp",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if Autoware message packages are available
pub fn is_autoware_msgs_available() -> bool {
    Command::new("bash")
        .args([
            "-c",
            "unset RMW_IMPLEMENTATION; source /opt/ros/humble/setup.bash && \
             source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null && \
             ros2 pkg list | grep -q autoware_vehicle_msgs",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Require ROS 2 + rmw_zenoh + Autoware msgs for a test
pub fn require_ros2_autoware() -> bool {
    // Phase 15.1 — was: print + return false (a pass). Now the preflight
    // gate hard-fails with remediation text unless SENTINEL_ALLOW_SKIP=1.
    crate::preflight::transport()
}

/// Get ROS 2 + Autoware environment setup command with custom locator
pub fn ros2_env_setup_with_locator(locator: &str) -> String {
    format!(
        "source /opt/ros/{}/setup.bash && \
         source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null && \
         export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
         export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"{}\"]'",
        DEFAULT_ROS_DISTRO, locator
    )
}

/// Phase 14 drive gate — DDS-side test domain. Keeps the baseline replay +
/// its CLI probes discoverable to each other while invisible to any
/// default-domain Autoware session sharing the machine.
pub const TEST_DDS_DOMAIN: u32 = 42;

/// Get ROS 2 + Autoware environment setup command with default DDS transport.
///
/// Used for interacting with Autoware processes that use default CycloneDDS.
pub fn ros2_env_setup_dds() -> String {
    // Pin the RMW explicitly: the harness environment may carry
    // RMW_IMPLEMENTATION=rmw_zenoh_cpp (direnv), which would silently turn
    // every "DDS" probe into a zenoh client pointed at the default router.
    format!(
        "source /opt/ros/{}/setup.bash && \
         source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null && \
         export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID={} && \
         unset ZENOH_CONFIG_OVERRIDE ZENOH_SESSION_CONFIG_URI ZENOH_ROUTER_CONFIG_URI ROS_LOCALHOST_ONLY",
        DEFAULT_ROS_DISTRO, TEST_DDS_DOMAIN,
    )
}

/// Managed ROS 2 process with automatic cleanup.
pub struct Ros2Process {
    handle: Child,
    #[allow(dead_code)]
    name: String,
}

impl Ros2Process {
    /// Spawn a bash command in its own process group.
    pub(crate) fn spawn_bash(cmd: &str, name: impl Into<String>) -> TestResult<Self> {
        let name = name.into();
        let mut command = Command::new("bash");
        command
            .args(["-c", cmd])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut command);
        let handle = command
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to start {name}: {e}")))?;
        Ok(Self { handle, name })
    }

    /// Start `ros2 topic echo` subscriber
    pub fn topic_echo(topic: &str, msg_type: &str, locator: &str) -> TestResult<Self> {
        let env_setup = ros2_env_setup_with_locator(locator);
        let cmd = format!(
            "{env_setup} && timeout 15 ros2 topic echo {topic} {msg_type} \
             --qos-reliability best_effort"
        );
        Self::spawn_bash(&cmd, format!("ros2 topic echo {topic}"))
    }

    /// Start `ros2 topic pub` publisher
    pub fn topic_pub(
        topic: &str,
        msg_type: &str,
        data: &str,
        rate: u32,
        locator: &str,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_with_locator(locator);
        let cmd = format!(
            "{env_setup} && timeout 15 ros2 topic pub -r {rate} {topic} {msg_type} \"{data}\" \
             --qos-reliability best_effort"
        );
        Self::spawn_bash(&cmd, format!("ros2 topic pub {topic}"))
    }

    /// Wait for output with timeout, capturing both stdout and stderr.
    pub fn wait_for_all_output(&mut self, timeout: Duration) -> TestResult<String> {
        use std::io::Read;
        #[cfg(unix)]
        use std::os::unix::io::AsRawFd;

        let start = std::time::Instant::now();
        let mut output = String::new();

        let mut stdout = self.handle.stdout.take();
        let mut stderr = self.handle.stderr.take();

        #[cfg(unix)]
        {
            if let Some(ref out) = stdout {
                let fd = out.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
            if let Some(ref err) = stderr {
                let fd = err.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
        }

        let mut buf = [0u8; 4096];

        loop {
            if start.elapsed() > timeout {
                kill_process_group(&mut self.handle);
                if output.is_empty() {
                    return Err(TestError::Timeout);
                }
                break;
            }

            match self.handle.try_wait() {
                Ok(Some(_)) => {
                    if let Some(ref mut out) = stdout {
                        let _ = out.read_to_string(&mut output);
                    }
                    if let Some(ref mut err) = stderr {
                        let _ = err.read_to_string(&mut output);
                    }
                    break;
                }
                Ok(None) => {
                    if let Some(ref mut out) = stdout {
                        if let Ok(n) = out.read(&mut buf)
                            && n > 0
                        {
                            output.push_str(&String::from_utf8_lossy(&buf[..n]));
                        }
                    }
                    if let Some(ref mut err) = stderr {
                        if let Ok(n) = err.read(&mut buf)
                            && n > 0
                        {
                            output.push_str(&String::from_utf8_lossy(&buf[..n]));
                        }
                    }
                    #[cfg(unix)]
                    {
                        let remaining = timeout.saturating_sub(start.elapsed());
                        let ms = remaining.as_millis().min(500) as i32;
                        let mut fds = Vec::new();
                        if let Some(ref out) = stdout {
                            fds.push(libc::pollfd {
                                fd: out.as_raw_fd(),
                                events: libc::POLLIN,
                                revents: 0,
                            });
                        }
                        if let Some(ref err) = stderr {
                            fds.push(libc::pollfd {
                                fd: err.as_raw_fd(),
                                events: libc::POLLIN,
                                revents: 0,
                            });
                        }
                        if !fds.is_empty() {
                            unsafe {
                                libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, ms);
                            }
                        }
                    }
                    #[cfg(not(unix))]
                    std::thread::sleep(Duration::from_millis(50));
                }
                Err(_) => break,
            }
        }

        Ok(output)
    }

    /// Kill the process group
    pub fn kill(&mut self) {
        kill_process_group(&mut self.handle);
    }
}

impl Drop for Ros2Process {
    fn drop(&mut self) {
        self.kill();
    }
}

// =============================================================================
// One-shot ROS 2 command helpers (for planning simulator tests)
// =============================================================================

/// Publish a single message to a topic and exit.
///
/// Uses `ros2 topic pub --once` which publishes one message then exits.
/// `env_setup` is the shell environment setup string (use `ros2_env_setup_with_locator`
/// for zenoh or `ros2_env_setup_dds` for default DDS).
pub fn topic_pub_once(
    topic: &str,
    msg_type: &str,
    data: &str,
    env_setup: &str,
) -> TestResult<String> {
    let cmd =
        format!("{env_setup} && timeout 30 ros2 topic pub --once {topic} {msg_type} \"{data}\"");
    let output = Command::new("bash")
        .args(["-c", &cmd])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("topic_pub_once failed: {e}")))?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let combined = format!("{stdout}{stderr}");

    if !output.status.success() {
        return Err(TestError::ProcessFailed(format!(
            "topic_pub_once to {topic} failed: {combined}"
        )));
    }
    Ok(combined)
}

/// Call a ROS 2 service and return the response.
///
/// Uses `ros2 service call` with a timeout.
/// `env_setup` is the shell environment setup string.
pub fn service_call(
    service: &str,
    srv_type: &str,
    request: &str,
    env_setup: &str,
) -> TestResult<String> {
    let cmd =
        format!("{env_setup} && timeout 30 ros2 service call {service} {srv_type} \"{request}\"");
    let output = Command::new("bash")
        .args(["-c", &cmd])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("service_call failed: {e}")))?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let combined = format!("{stdout}{stderr}");

    if !output.status.success() {
        return Err(TestError::ProcessFailed(format!(
            "service_call to {service} failed: {combined}"
        )));
    }
    Ok(combined)
}

/// Check if a topic currently has publishers.
///
/// Uses `ros2 topic info` and checks the publisher count.
/// `env_setup` is the shell environment setup string.
pub fn check_topic_active(topic: &str, env_setup: &str) -> TestResult<bool> {
    let cmd = format!("{env_setup} && timeout 10 ros2 topic info {topic}");
    let output = Command::new("bash")
        .args(["-c", &cmd])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .map_err(|e| TestError::ProcessFailed(format!("check_topic_active failed: {e}")))?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    // `ros2 topic info` outputs "Publisher count: N"
    // Topic is active if publisher count > 0
    Ok(stdout.contains("Publisher count: ") && !stdout.contains("Publisher count: 0"))
}

/// Wait until all specified topics have at least one publisher.
///
/// Polls `ros2 topic info` for each topic until all are active or timeout.
/// `env_setup` is the shell environment setup string.
pub fn wait_for_topics(
    topics: &[&str],
    env_setup: &str,
    timeout: std::time::Duration,
) -> TestResult<()> {
    let start = std::time::Instant::now();

    loop {
        if start.elapsed() > timeout {
            // Report which topics are still missing
            let mut missing = Vec::new();
            for topic in topics {
                if !check_topic_active(topic, env_setup).unwrap_or(false) {
                    missing.push(*topic);
                }
            }
            return Err(TestError::ProcessFailed(format!(
                "Timed out waiting for topics: {}",
                missing.join(", ")
            )));
        }

        let all_active = topics
            .iter()
            .all(|topic| check_topic_active(topic, env_setup).unwrap_or(false));

        if all_active {
            return Ok(());
        }

        std::thread::sleep(std::time::Duration::from_secs(2));
    }
}

/// Start `ros2 topic echo` with default DDS transport (no zenoh).
pub fn topic_echo_dds(topic: &str, msg_type: &str) -> TestResult<Ros2Process> {
    let env_setup = ros2_env_setup_dds();
    let cmd = format!("{env_setup} && timeout 30 ros2 topic echo {topic} {msg_type}");
    Ros2Process::spawn_bash(&cmd, format!("ros2 topic echo {topic} (DDS)"))
}
