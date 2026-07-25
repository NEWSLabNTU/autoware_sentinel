//! Managed process utilities for integration tests
//!
//! Provides RAII-based process management with automatic cleanup.
//! All child processes are spawned in their own process group so that
//! `kill_process_group()` can reap the entire tree (bash + children).

use crate::TestError;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

/// Configure a Command to spawn the child in its own process group.
///
/// On Linux, also sets `PR_SET_PDEATHSIG(SIGKILL)` so the child is killed
/// when the parent dies — prevents orphans when nextest SIGKILL's the test.
#[cfg(unix)]
pub fn set_new_process_group(command: &mut Command) -> &mut Command {
    use std::os::unix::process::CommandExt;
    // SAFETY: setpgid and prctl are async-signal-safe and called before exec
    unsafe {
        command.pre_exec(|| {
            libc::setpgid(0, 0);
            #[cfg(target_os = "linux")]
            {
                libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL);
            }
            Ok(())
        })
    }
}

/// Kill an entire process group given a child handle.
///
/// Sends SIGKILL to the process group (negative PID), then waits for
/// the direct child to be reaped.
#[cfg(unix)]
pub fn kill_process_group(handle: &mut Child) {
    let pid = handle.id() as libc::pid_t;
    unsafe {
        libc::kill(-pid, libc::SIGKILL);
    }
    let _ = handle.wait();
}

#[cfg(not(unix))]
pub fn kill_process_group(handle: &mut Child) {
    let _ = handle.kill();
    let _ = handle.wait();
}

/// Managed process with automatic cleanup.
///
/// Wraps a child process and ensures it is killed on drop.
/// Uses process groups so killing also reaps all descendants.
pub struct ManagedProcess {
    handle: Child,
    #[allow(dead_code)]
    name: String,
}

impl ManagedProcess {
    /// Spawn a new managed process from a binary path and args.
    pub fn spawn(
        binary: &std::path::Path,
        args: &[&str],
        name: impl Into<String>,
    ) -> Result<Self, TestError> {
        let name = name.into();
        let mut cmd = Command::new(binary);
        cmd.args(args).stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut cmd);
        let handle = cmd
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to spawn {}: {}", name, e)))?;
        Ok(Self { handle, name })
    }

    /// Spawn a process from a pre-configured Command builder.
    pub fn spawn_command(mut command: Command, name: impl Into<String>) -> Result<Self, TestError> {
        let name = name.into();
        command.stdout(Stdio::piped()).stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut command);
        let handle = command
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to spawn {}: {}", name, e)))?;
        Ok(Self { handle, name })
    }

    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        matches!(self.handle.try_wait(), Ok(None))
    }

    /// Detach background drainer threads on stdout + stderr so the
    /// piped child process keeps writing without filling the kernel
    /// pipe buffer. Call this after `wait_for_output_pattern` returns
    /// when the parent no longer needs further output but the child
    /// will keep producing log lines (e.g. a long-running sentinel
    /// binary that ties up zenoh I/O if its stdout pipe blocks).
    ///
    /// The threads exit naturally when the child closes its end of
    /// the pipe (e.g. on SIGKILL from `Drop`).
    pub fn drain_in_background(&mut self) {
        use std::io::Read;
        if let Some(mut stdout) = self.handle.stdout.take() {
            std::thread::spawn(move || {
                let mut buf = [0u8; 4096];
                loop {
                    match stdout.read(&mut buf) {
                        Ok(0) | Err(_) => break,
                        Ok(_) => continue,
                    }
                }
            });
        }
        if let Some(mut stderr) = self.handle.stderr.take() {
            std::thread::spawn(move || {
                let mut buf = [0u8; 4096];
                loop {
                    match stderr.read(&mut buf) {
                        Ok(0) | Err(_) => break,
                        Ok(_) => continue,
                    }
                }
            });
        }
    }

    /// Wait until a pattern appears in stdout+stderr, then return all output so far.
    pub fn wait_for_output_pattern(
        &mut self,
        pattern: &str,
        timeout: Duration,
    ) -> Result<String, TestError> {
        use std::io::Read;
        #[cfg(unix)]
        use std::os::unix::io::AsRawFd;

        let start = std::time::Instant::now();
        let mut output = String::new();

        let mut stdout = self.handle.stdout.take();
        let mut stderr = self.handle.stderr.take();

        // Set non-blocking mode
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
                self.handle.stdout = stdout;
                self.handle.stderr = stderr;
                if output.is_empty() {
                    return Err(TestError::Timeout);
                }
                return Ok(output);
            }

            if let Ok(Some(_)) = self.handle.try_wait() {
                if let Some(ref mut out) = stdout {
                    let _ = out.read_to_string(&mut output);
                }
                if let Some(ref mut err) = stderr {
                    let _ = err.read_to_string(&mut output);
                }
                break;
            }

            let mut got_data = false;
            if let Some(ref mut out) = stdout
                && let Ok(n) = out.read(&mut buf)
                && n > 0
            {
                output.push_str(&String::from_utf8_lossy(&buf[..n]));
                got_data = true;
            }
            if let Some(ref mut err) = stderr
                && let Ok(n) = err.read(&mut buf)
                && n > 0
            {
                output.push_str(&String::from_utf8_lossy(&buf[..n]));
                got_data = true;
            }

            if output.contains(pattern) {
                self.handle.stdout = stdout;
                self.handle.stderr = stderr;
                return Ok(output);
            }

            if !got_data {
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
        }

        self.handle.stdout = stdout;
        self.handle.stderr = stderr;
        Ok(output)
    }

    /// Wait for output with timeout, capturing both stdout and stderr.
    /// The process is killed when the timeout is reached.
    pub fn wait_for_all_output(&mut self, timeout: Duration) -> Result<String, TestError> {
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
                        match out.read(&mut buf) {
                            Ok(n) if n > 0 => {
                                output.push_str(&String::from_utf8_lossy(&buf[..n]));
                            }
                            _ => {}
                        }
                    }
                    if let Some(ref mut err) = stderr {
                        match err.read(&mut buf) {
                            Ok(n) if n > 0 => {
                                output.push_str(&String::from_utf8_lossy(&buf[..n]));
                            }
                            _ => {}
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

    /// Kill the process group and wait for it to exit
    pub fn kill(&mut self) {
        kill_process_group(&mut self.handle);
    }
}

impl Drop for ManagedProcess {
    fn drop(&mut self) {
        self.kill();
    }
}

/// Get the path to the locally-built zenohd binary.
///
/// Resolution order: `SENTINEL_ZENOHD` env override, the repo-local
/// `external/zenoh` build, then the nano-ros checkout's build (version-
/// matched to the zenoh-pico the sentinel links since the phase-14 pin).
pub fn zenohd_binary_path() -> std::path::PathBuf {
    if let Ok(p) = std::env::var("SENTINEL_ZENOHD") {
        return std::path::PathBuf::from(p);
    }
    let local = project_root().join("external/zenoh/target/fast/zenohd");
    if local.exists() {
        return local;
    }
    if let Some(home) = std::env::home_dir() {
        let nano_ros = home.join("repos/nano-ros/build/zenohd/zenohd");
        if nano_ros.exists() {
            return nano_ros;
        }
    }
    local
}

/// Check if the locally-built zenohd is available.
pub fn is_zenohd_available() -> bool {
    let path = zenohd_binary_path();
    path.exists()
        && Command::new(&path)
            .arg("--version")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .map(|s| s.success())
            .unwrap_or(false)
}

/// Skip test if zenohd is not available.
pub fn require_zenohd() -> bool {
    if !is_zenohd_available() {
        eprintln!("Skipping test: zenohd not found");
        return false;
    }
    true
}

/// Get the project root directory (autoware-nano-ros/)
pub fn project_root() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}
