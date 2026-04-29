// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Autoware Sentinel — QEMU FreeRTOS binary.
//!
//! Thin platform shim. All wiring lives in [`autoware_sentinel_core`];
//! this crate provides the FreeRTOS entry point, a board-supplied logger,
//! and `xTaskGetTickCount`-backed clock.
//!
//! `controller-node` is OFF — main compute supplies `/control/.../control_cmd`;
//! sentinel triggers MRM on staleness.

#![no_std]
#![no_main]

use nros::prelude::*;
use nros_board_mps2_an385_freertos::{Config, println, run};
use panic_semihosting as _;

/// Monotonic clock in milliseconds.
///
/// FreeRTOS `xTaskGetTickCount()` returns ticks; the board configures
/// `configTICK_RATE_HZ = 1000`, so ticks are milliseconds 1:1.
fn now_ms() -> u64 {
    unsafe extern "C" {
        fn xTaskGetTickCount() -> u32;
    }
    unsafe { xTaskGetTickCount() as u64 }
}

#[unsafe(no_mangle)]
extern "C" fn _start() -> ! {
    run(Config::from_toml(include_str!("../config.toml")), |config| {
        println!("Autoware Sentinel — Safety Island");
        println!("Locator: {}", config.zenoh_locator);

        let exec_config = ExecutorConfig::new(config.zenoh_locator)
            .domain_id(config.domain_id)
            .node_name("sentinel");
        let mut executor = Executor::open(&exec_config)?;

        // FreeRTOS profile: skip parameter services (heavy code + static
        // tables, blocks 4 MB flash budget). Use compile-time defaults
        // assembled in the same shape as ROS 2 parameter reads would yield.
        let sentinel_params = autoware_sentinel_core::params::default_params();
        println!("Using compile-time default parameters");

        autoware_sentinel_core::init_island(sentinel_params);
        autoware_sentinel_core::wire_executor(&mut executor, now_ms)?;

        println!("Executor ready — spinning...");
        executor.spin(core::time::Duration::from_millis(10));

        #[allow(unreachable_code)]
        Ok::<(), NodeError>(())
    })
}
