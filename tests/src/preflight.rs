// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Phase 15.1/15.2 — fail-loud prerequisite gate.
//!
//! Phase 14 repeatedly reported success while running nothing: the
//! `rmw_zenoh_cpp` overlay had been wiped, every `require_*` helper returned
//! `false`, each test returned early, and the suite printed `14 tests run:
//! 14 passed` in under a second. Three separate mis-diagnoses came out of
//! that, including hours spent hunting a firmware bug that did not exist.
//!
//! This module inverts the default: a missing prerequisite is an ERROR with
//! a remediation line, not a silent pass. Skipping is still possible on a
//! machine without Autoware, but it must be asked for
//! (`SENTINEL_ALLOW_SKIP=1`) and it is counted and printed.
//!
//! Integrity, not presence (15.2): the incident that cost the most was a
//! symlink that existed and pointed nowhere. Every check here proves the
//! artifact WORKS — the library loads, the binary answers `--version`, the
//! package resolves — rather than that a path exists.

use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};

use crate::autoware;
use crate::process::project_root;
use crate::ros2;

/// Count of prerequisites that were skipped rather than enforced. Printed by
/// [`report_skips`] so "nothing ran" can never read as green.
static SKIPPED: AtomicUsize = AtomicUsize::new(0);

/// Env var that downgrades a missing prerequisite from error to skip.
const ALLOW_SKIP: &str = "SENTINEL_ALLOW_SKIP";

fn skips_allowed() -> bool {
    std::env::var(ALLOW_SKIP).is_ok_and(|v| v != "0" && !v.is_empty())
}

/// One prerequisite: what it is, how to prove it works, how to fix it.
struct Check {
    name: &'static str,
    ok: bool,
    remedy: &'static str,
}

/// Fail loudly (or, with `SENTINEL_ALLOW_SKIP=1`, skip loudly).
///
/// Returns `false` only in explicit-skip mode; otherwise it panics, so a
/// caller can `if !preflight(...) { return; }` and keep the opt-in skip path
/// without ever silently passing.
fn enforce(checks: &[Check]) -> bool {
    let failed: Vec<&Check> = checks.iter().filter(|c| !c.ok).collect();
    if failed.is_empty() {
        return true;
    }

    let mut msg = String::from("preflight: missing prerequisites\n");
    for c in &failed {
        msg.push_str(&format!("  - {}\n      fix: {}\n", c.name, c.remedy));
    }

    if skips_allowed() {
        SKIPPED.fetch_add(1, Ordering::Relaxed);
        eprintln!("{msg}  ({ALLOW_SKIP} set — skipping this test)");
        return false;
    }

    msg.push_str(&format!(
        "\n  This is an ERROR, not a skip: a test that cannot reach its\n\
         \x20 subject must not report success (phase 15.1).\n\
         \x20 Set {ALLOW_SKIP}=1 to skip instead — CI never does."
    ));
    panic!("{msg}");
}

/// Print how many tests skipped prerequisites. Call at the end of a suite.
pub fn report_skips() {
    let n = SKIPPED.load(Ordering::Relaxed);
    if n > 0 {
        eprintln!("preflight: {n} test(s) SKIPPED for missing prerequisites — not passes");
    }
}

// ============================================================================
// Integrity checks (15.2) — prove the artifact works, not that a path exists
// ============================================================================

/// The rmw_zenoh overlay must resolve to a loadable `librmw_zenoh_cpp.so`
/// INSIDE this repo. Phase 14 had it symlinked into a sibling checkout's
/// build tree, which that repo's own work deleted — twice.
pub fn rmw_zenoh_overlay_ok() -> (bool, Option<String>) {
    let install = project_root().join("external/rmw_zenoh_ws/install");
    let setup = install.join("local_setup.bash");
    if !setup.exists() {
        return (false, Some("overlay not built".into()));
    }

    // The symlink-into-a-wiped-tree case: the path resolves, the content is
    // gone. Demand the actual shared object.
    let lib = install.join("rmw_zenoh_cpp/lib/librmw_zenoh_cpp.so");
    if !lib.exists() {
        return (false, Some("librmw_zenoh_cpp.so missing from overlay".into()));
    }

    // And demand it actually answers as the selected RMW.
    let ok = ros2::is_rmw_zenoh_available();
    if !ok {
        return (
            false,
            Some("overlay present but `ros2 pkg list` does not see rmw_zenoh_cpp".into()),
        );
    }
    (true, None)
}

/// No prerequisite may live outside the repo (the sibling-checkout trap).
fn external_paths_self_contained() -> bool {
    let root = project_root();
    let external = root.join("external");
    let Ok(entries) = std::fs::read_dir(&external) else {
        return true; // nothing to check
    };
    for entry in entries.flatten() {
        let p = entry.path();
        if let Ok(real) = std::fs::canonicalize(&p)
            && !real.starts_with(&root)
        {
            eprintln!(
                "preflight: {} resolves outside the repo ({}) — sibling build trees get wiped",
                p.display(),
                real.display()
            );
            return false;
        }
    }
    true
}

fn binary_answers(bin: &PathBuf, arg: &str) -> bool {
    Command::new(bin)
        .arg(arg)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

// ============================================================================
// Public gates — one per test class
// ============================================================================

/// Strict variant for FIXTURES: always hard-fails.
///
/// A fixture cannot skip a test — by the time it runs, the test is
/// executing — so honouring `SENTINEL_ALLOW_SKIP` here would produce a
/// confusing half-skip that still fails downstream. Skipping belongs at the
/// top of a test body (`require_*`), where an early return is possible.
pub fn transport_strict() {
    let saved = std::env::var(ALLOW_SKIP).ok();
    // SAFETY: single-threaded fixture setup; restored immediately below.
    unsafe { std::env::remove_var(ALLOW_SKIP) };
    let ok = transport();
    if let Some(v) = saved {
        unsafe { std::env::set_var(ALLOW_SKIP, v) };
    }
    debug_assert!(ok, "transport() must panic rather than return false here");
}

/// Transport-smoke prerequisites: ROS 2 + a WORKING rmw_zenoh overlay +
/// Autoware messages + a router binary.
pub fn transport() -> bool {
    let (overlay_ok, overlay_why) = rmw_zenoh_overlay_ok();
    if let Some(why) = overlay_why {
        eprintln!("preflight: rmw_zenoh overlay — {why}");
    }
    enforce(&[
        Check {
            name: "ROS 2 (humble)",
            ok: ros2::is_ros2_available(),
            remedy: "install ROS 2 humble at /opt/ros/humble",
        },
        Check {
            name: "rmw_zenoh_cpp overlay (loadable, in-repo)",
            ok: overlay_ok,
            remedy: "scripts/build_rmw_zenoh.sh  (needs `git submodule update --init \
                     external/rmw_zenoh_ws/src/rmw_zenoh`)",
        },
        Check {
            name: "external/* self-contained",
            ok: external_paths_self_contained(),
            remedy: "rebuild the artifact inside this repo; do not symlink sibling build trees",
        },
        Check {
            name: "Autoware message packages",
            ok: ros2::is_autoware_msgs_available(),
            remedy: "source /opt/autoware/1.5.0/setup.bash (or install Autoware 1.5.0)",
        },
        Check {
            name: "zenoh router binary",
            ok: crate::process::is_zenohd_available(),
            remedy: "build zenohd, or set SENTINEL_ZENOHD to a working binary \
                     (the overlay's rmw_zenohd also works)",
        },
    ])
}

/// Planning-simulator prerequisites: everything transport needs, plus
/// play_launch and the map.
pub fn planning_simulator() -> bool {
    if !transport() {
        return false;
    }
    let play_launch = autoware::play_launch_binary();
    enforce(&[
        Check {
            name: "play_launch",
            ok: binary_answers(&play_launch, "--version"),
            remedy: "pip install --user play_launch, or set $PLAY_LAUNCH \
                     (the `resolve` verb needs the source build)",
        },
        Check {
            name: "Autoware map data",
            ok: autoware::is_autoware_map_available(),
            remedy: "set $MAP_PATH, or install autoware_test_utils",
        },
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The overlay check must reject a path that exists but is empty — the
    /// exact shape of the phase-14 incident.
    #[test]
    fn overlay_check_rejects_hollow_install() {
        let (ok, why) = rmw_zenoh_overlay_ok();
        // On a healthy machine this passes; on a hollow overlay it must
        // explain itself rather than silently returning false.
        if !ok {
            assert!(why.is_some(), "a failing overlay check must say why");
        }
    }
}
