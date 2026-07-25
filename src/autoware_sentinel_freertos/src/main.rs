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

// Phase 14.1: the board's C `Reset_Handler` now calls `main` (the pre-N.7
// `_start` shape was retired upstream, commit d99386173).
#[unsafe(no_mangle)]
extern "C" fn main() -> i32 {
    run(
        // Phase 14.1: [scheduling] left the config schema (172.K.6); the
        // app-task stack rides the builder now. 384 KiB default overflows
        // wiring 40+ publishers — the Phase-13 config used 512 KiB.
        Config::from_toml(include_str!("../config.toml")).with_app_stack_bytes(786_432),
        |config| {
        println!("Autoware Sentinel — Safety Island");
        println!("Locator: {}", config.zenoh_locator);

        // Phase 14.1: the zenoh RMW backend is linked via nros-rmw-zenoh and
        // must be registered before the executor opens.
        nros_rmw_zenoh::register().map_err(|_| NodeError::BackendMismatch)?;

        let exec_config = ExecutorConfig::new(config.zenoh_locator)
            .domain_id(config.domain_id)
            .node_name("sentinel");
        // Sized explicitly (phase-271): ~37 callbacks with comp-all; 64 is
        // the executor's ready-set cap.
        let sizing = nros::ExecutorSizing {
            cbs: 64,
            arena: nros::arena_size_for(64),
            ..nros::ExecutorSizing::DEFAULT
        };
        let mut executor = Executor::open_sized(&exec_config, sizing)?;

        let sentinel_params = autoware_sentinel_core::params::default_params();
        autoware_sentinel_core::init_island(sentinel_params);
        autoware_sentinel_core::wire_executor(&mut executor, now_ms)?;

        println!("Executor ready — spinning...");
        executor.spin(core::time::Duration::from_millis(10));

        #[allow(unreachable_code)]
        Ok::<(), NodeError>(())
    })
}
