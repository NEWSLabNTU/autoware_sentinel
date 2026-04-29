// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Autoware Sentinel — Zephyr binary.
//!
//! Thin platform shim. All wiring lives in [`autoware_sentinel_core`];
//! this crate provides the Zephyr entry point, logger, and monotonic clock.
//!
//! The `controller-node` feature is OFF — main compute supplies
//! `/control/.../control_cmd`; sentinel triggers MRM on staleness.

#![no_std]

use core::time::Duration;

use log::info;
use nros::prelude::*;

/// Default zenoh locator. native_sim uses NSOS (host-offloaded sockets) so
/// it reaches the host's loopback directly. Production boards override at
/// build time.
const DEFAULT_LOCATOR: &str = "tcp/127.0.0.1:7447";

/// Zephyr uptime in milliseconds (`k_uptime_get`).
fn now_ms() -> u64 {
    zephyr::sys::uptime_get() as u64
}

#[unsafe(no_mangle)]
extern "C" fn rust_main() {
    unsafe {
        zephyr::set_logger().ok();
    }

    info!("Autoware Sentinel — Safety Island");
    info!("Board: {}", zephyr::kconfig::CONFIG_BOARD);

    if let Err(e) = run() {
        log::error!("Fatal: {:?}", e);
    }
}

fn run() -> Result<(), NodeError> {
    let config = ExecutorConfig::new(DEFAULT_LOCATOR).node_name("sentinel");
    let mut executor = Executor::open(&config)?;

    executor.register_parameter_services()?;
    autoware_sentinel_core::params::declare_parameters(executor.params_mut().unwrap());
    let sentinel_params = autoware_sentinel_core::params::read_params(executor.params().unwrap());
    info!("Declared {} parameters", executor.params().unwrap().len());

    autoware_sentinel_core::init_island(sentinel_params);
    autoware_sentinel_core::wire_executor(&mut executor, now_ms)?;

    info!("Executor ready — spinning...");
    executor.spin(Duration::from_millis(10));
}
