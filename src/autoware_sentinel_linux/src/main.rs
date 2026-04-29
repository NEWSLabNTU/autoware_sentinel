// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Autoware Sentinel — Linux native binary.
//!
//! Thin platform shim. All wiring lives in [`autoware_sentinel_core`];
//! this crate provides the entry point, monotonic clock, and logger.

use std::sync::OnceLock;
use std::time::Instant;

use log::info;
use nros::prelude::*;

// Stub for zpico debug function (bare-metal semihosting; no-op on Linux).
#[unsafe(no_mangle)]
pub extern "C" fn zpico_debug_i32(_label: *const u8, _value: i32) {}

/// Monotonic clock in milliseconds since process start.
fn now_ms() -> u64 {
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    let epoch = EPOCH.get_or_init(Instant::now);
    epoch.elapsed().as_millis() as u64
}

fn main() {
    env_logger::init();
    info!("Autoware Sentinel — Linux Native Safety Island");

    // Run on a thread with a large stack: debug builds keep the executor
    // arena (~222 KB), the publisher tuple (51 entries), and the timer
    // closure on the stack — the default 8 MB main-thread stack overflows.
    let builder = std::thread::Builder::new().stack_size(64 * 1024 * 1024);
    let handle = builder
        .spawn(|| {
            if let Err(e) = run() {
                log::error!("Fatal: {:?}", e);
                std::process::exit(1);
            }
        })
        .expect("Failed to spawn main thread");
    handle.join().expect("Main thread panicked");
}

fn run() -> Result<(), NodeError> {
    let config = ExecutorConfig::from_env().node_name("sentinel");
    let mut executor = Executor::open(&config)?;

    // Parameter services + declare + read.
    executor.register_parameter_services()?;
    autoware_sentinel_core::params::declare_parameters(executor.params_mut().unwrap());
    let sentinel_params = autoware_sentinel_core::params::read_params(executor.params().unwrap());
    info!("Declared {} parameters", executor.params().unwrap().len());

    autoware_sentinel_core::init_island(sentinel_params);
    autoware_sentinel_core::wire_executor(&mut executor, now_ms)?;

    info!("Executor ready — spinning...");
    executor.spin_blocking(SpinOptions::default())?;
    Ok(())
}
