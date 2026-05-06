// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! Autoware Sentinel — SPE firmware (Phase 11.5 + 11.3 status scaffold).
//!
//! Thin staticlib NVIDIA's `rt-aux-cpu-demo-fsp/main_task` calls into
//! via the `nros_app_init()` C shim (Phase 11.5.c).
//!
//! # Build modes
//!
//! Two cargo features control how much SafetyIsland gets wired into
//! the running firmware:
//!
//! | Feature              | Behaviour                                    | BTCM result |
//! |----------------------|----------------------------------------------|-------------|
//! | (default — none)     | `Executor::open` + `spin`, no SafetyIsland   | fits, 10 KB headroom |
//! | `safety-island`      | + `init_island` + `wire_executor`            | overflows by ~37 KB |
//!
//! `just build-spe-image` builds without `safety-island` so it produces a
//! bootable `spe.bin` out-of-the-box. To reproduce the overflow that
//! Phase 11.3.D ends on, pass the feature explicitly:
//!
//! ```sh
//! cd src/sentinel_spe_firmware
//! cargo +nightly build --release --features safety-island
//! ```
//!
//! # Phase 11.3 status — measured
//!
//! - 11.3.A — zero-copy IVC API (nano-ros side): **DONE**.
//! - 11.3.B — FreeRTOS-native condvar wait (nano-ros side): **DONE**.
//! - 11.3.C — feature pruning to fit 256 KB BTCM: **DONE for default
//!   build, partial for `safety-island`** (143 → 37 KB recovered).
//! - 11.3.D — SafetyIsland wiring: **scaffolded behind `safety-island`
//!   feature**; default build skips it. Closing the residual 37 KB
//!   gap is tracked as Phase 11.3.E (DRAM mapping via AST, hardware
//!   required).
//!
//! See `docs/roadmap/11-orin-spe.md` for the full recovery ledger.

#![no_std]
#![allow(internal_features)]

use nros::prelude::*;
use nros_board_orin_spe::{Config, println, run};

// Pull the global-allocator impl in. `nros-platform`'s
// `global-allocator` feature registers a `#[global_allocator]` static
// that forwards to FreeRTOS `pvPortMalloc` / `vPortFree` via the
// `OrinSpe` `PlatformAlloc` impl. The `extern crate` line is what
// actually pulls the symbol — feature-gating alone doesn't.
extern crate nros_platform;
extern crate alloc;

// Panic handler: halt forever. The application closure carries its
// own MRM-on-error path; a panic here only fires on truly unexpected
// state (heap exhaustion, unwrap on None) and the safest response on
// a safety MCU is "stop driving". Future revision: emit panic
// location to TCU before halting.
use panic_halt as _;

/// Monotonic clock in milliseconds. FSP `xTaskGetTickCount` returns
/// ticks; `configTICK_RATE_HZ = 1000` makes ticks ≡ ms.
#[cfg(feature = "safety-island")]
fn now_ms() -> u64 {
    unsafe extern "C" {
        fn xTaskGetTickCount() -> u32;
    }
    unsafe { xTaskGetTickCount() as u64 }
}

/// Called from `app/nros-app.c` (added to the BSP tree by
/// `scripts/spe/apply-patches.sh`).
///
/// Default build: opens the executor over IVC and spins — fits BTCM
/// with ~10 KB headroom.
///
/// `safety-island` feature: also calls `init_island` + `wire_executor`
/// to wire the full SafetyIsland tick path. Currently overflows BTCM
/// by ~37 KB; closing the gap is tracked in Phase 11.3.E.
#[unsafe(no_mangle)]
pub extern "C" fn nros_app_rust_entry() {
    run(Config::default(), |config| -> Result<(), NodeError> {
        // Plain-string banner — `Display` formatting (e.g.
        // `println!("Locator: {}", ...)`) drags `core::fmt::Formatter::pad`,
        // `escape_debug_ext`, `PadAdapter::write_str`, `from_utf8`, …
        // which collectively cost ~2.5 KB BTCM.
        println!("Autoware Sentinel - Safety Island (Orin SPE)");

        let exec_config = ExecutorConfig::new(config.zenoh_locator)
            .domain_id(config.domain_id)
            .node_name("sentinel");
        let mut executor = Executor::open(&exec_config)?;

        #[cfg(feature = "safety-island")]
        {
            let sentinel_params = autoware_sentinel_core::params::default_params();
            autoware_sentinel_core::init_island(sentinel_params);
            autoware_sentinel_core::wire_executor(&mut executor, now_ms)?;
        }

        println!("Executor ready - spinning");
        executor.spin(core::time::Duration::from_millis(10));

        #[allow(unreachable_code)]
        Ok(())
    });
}
