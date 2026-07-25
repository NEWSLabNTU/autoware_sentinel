// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Autoware Sentinel — NuttX QEMU ARM binary.
//!
//! Thin platform shim. All wiring lives in [`autoware_sentinel_core`];
//! this crate provides the NuttX entry point and a `clock_gettime`-backed
//! monotonic clock.
//!
//! `controller-node` is OFF — main compute supplies
//! `/control/.../control_cmd`; sentinel triggers MRM on staleness.

use nros::prelude::*;
use nros_board_nuttx_qemu_arm::{Config, run};

/// Monotonic clock in milliseconds. NuttX exposes a POSIX
/// `clock_gettime(CLOCK_MONOTONIC, …)` via the libc crate.
fn now_ms() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn main() {
    run(Config::from_toml(include_str!("../config.toml")), |config| {
        println!("Autoware Sentinel — Safety Island (NuttX QEMU)");
        println!("Locator: {}", config.zenoh_locator);

        let exec_config = ExecutorConfig::new(config.zenoh_locator)
            .domain_id(config.domain_id)
            .node_name("sentinel");
        // Sized explicitly (phase-271): ~31 callbacks with comp-all; 64 is
        // the executor's ready-set cap.
        let sizing = nros::ExecutorSizing {
            cbs: 64,
            arena: nros::arena_size_for(64),
            ..nros::ExecutorSizing::DEFAULT
        };
        let mut executor = Executor::open_sized(&exec_config, sizing)?;

        // NuttX profile: skip parameter services to keep the binary
        // size manageable inside the NuttX flat-build kernel image.
        // Use compile-time defaults assembled in the same shape as
        // ROS 2 parameter reads would yield.
        let sentinel_params = autoware_sentinel_core::params::default_params();
        autoware_sentinel_core::init_island(sentinel_params);
        autoware_sentinel_core::wire_executor(&mut executor, now_ms)?;

        println!("Executor ready — spinning...");
        executor.spin(core::time::Duration::from_millis(10));

        #[allow(unreachable_code)]
        Ok::<(), NodeError>(())
    })
}
