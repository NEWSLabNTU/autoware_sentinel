// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Phase 15.5 — the preflight gate as a first-class CI step.
//!
//! `just ci` runs this before anything else, so a machine missing a
//! prerequisite learns it in two seconds with a remediation line, instead of
//! discovering it as a mystery ten minutes into an integration run (or, as
//! in phase 14, never discovering it because everything "passed").

use sentinel_tests::preflight;

/// Everything the transport suite needs, proven to WORK (not merely exist).
#[test]
fn transport_prerequisites_present() {
    preflight::transport();
}

/// Everything the planning-simulator suite additionally needs.
///
/// Separate test so a box without Autoware/play_launch still gets a precise
/// verdict on the transport half.
#[test]
fn planning_simulator_prerequisites_present() {
    preflight::planning_simulator();
}
