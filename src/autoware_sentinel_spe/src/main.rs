// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Autoware Sentinel — AGX Orin SPE binary.
//!
//! POSIX dev path (Phase 11.1–11.3 "Stage 1"): runs as a Linux process,
//! uses `nvidia-ivc::unix-mock` for IVC, talks to a host `ivc-bridge`
//! daemon over a Unix socket, and reaches Autoware via the bridge ↔
//! zenohd ↔ rmw_zenoh_cpp chain.
//!
//! Real hardware path (Phase 11.5+): same `wire_executor` body, but
//! cross-compiled for `armv7r-none-eabihf` against NVIDIA FSP via the
//! `nros-board-orin-spe` board crate. Stub for now — the cross-compile
//! glue lands when 11.5 starts.

use nros::prelude::*;

/// Monotonic clock in milliseconds.
fn now_ms() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("Autoware Sentinel — AGX Orin SPE (POSIX dev)");
    if let Err(e) = run() {
        log::error!("Fatal: {e:?}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), NodeError> {

    // Locator: the SPE-side IVC link transport. On real hardware this
    // resolves to channel id 2 (`aon_echo`) via FSP; on POSIX dev we
    // rely on `ivc-bridge` to have called
    // `nvidia_ivc::unix_mock::register_fd(2, …)` against the same
    // Unix socket. Until the `link-ivc` cargo feature is plumbed
    // through `nros` to `nros-rmw-zenoh` for non-orin-spe platform
    // builds, this binary connects via `tcp/127.0.0.1:7447` directly
    // — i.e. it bypasses the bridge in POSIX dev mode and acts as a
    // pure Linux sentinel. The IVC framing pieces are exercised by
    // `nros-tests::orin_spe_mock_ivc`; this binary integrates them
    // once the feature plumbing lands (TODO: track as 11.2.b).
    let locator = std::env::var("ZENOH_LOCATOR")
        .unwrap_or_else(|_| "tcp/127.0.0.1:7447".to_string());

    let exec_config = ExecutorConfig::new(&locator)
        .domain_id(0)
        .node_name("sentinel_spe");
    let mut executor = Executor::open(&exec_config)?;
    log::info!("Connected to zenohd at {locator}");

    // Reduced sentinel parameter set sized for the 256 KB BTCM
    // budget on real Cortex-R5F. Same wire_executor entry the Linux /
    // Zephyr / FreeRTOS / NuttX binaries call.
    let sentinel_params = autoware_sentinel_core::params::default_params();
    autoware_sentinel_core::init_island(sentinel_params);
    autoware_sentinel_core::wire_executor(&mut executor, now_ms)?;

    log::info!("Executor ready — spinning…");
    executor.spin_blocking(SpinOptions::default())?;

    Ok(())
}
