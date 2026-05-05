// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! Autoware Sentinel — SPE firmware (Phase 11.5 + 11.3 status scaffold).
//!
//! Thin staticlib NVIDIA's `rt-aux-cpu-demo-fsp/main_task` calls into
//! via the `nros_app_init()` C shim (Phase 11.5.c).
//!
//! # Phase 11.3 status — measured
//!
//! Three blockers tracked under §11.3 of the phase doc:
//!
//! - 11.3.A — zero-copy IVC API (nano-ros side): **DONE**.
//! - 11.3.B — FreeRTOS-native condvar wait (nano-ros side): **DONE**.
//! - 11.3.C — feature pruning to fit 256 KB BTCM: **MEASURED**, see
//!   below.
//! - 11.3.D — SafetyIsland wiring: **PENDING** the size-budget
//!   architectural decision below.
//!
//! ## 11.3.C measurement results
//!
//! With pin `b5d599f4` (post 11.3.A + 11.3.B), the smallest
//! buildable feature set still overflows BTCM:
//!
//! | Build               | text+data+bss | BTCM overflow |
//! |---------------------|---------------|---------------|
//! | wfi-loop scaffold   | 146 KB        | -             |
//! | + Executor::open    | ~595 KB       | 339 KB        |
//! | + sentinel_core     | ~727 KB       | 471 KB        |
//!
//! Feature pruning at the `nros` / `autoware_sentinel_core` level
//! cannot bridge the gap — the floor is the zenoh-pico session +
//! `nros::Executor` runtime itself (~340 KB). The next architectural
//! decision (deferred from 11.3.C scope) is:
//!
//! 1. **DRAM mapping via AST** — split the linker layout so vector
//!    table + IRQ handlers + critical paths stay in BTCM, bulk
//!    `.text + .rodata` lives in a DRAM carveout reachable through
//!    the SPE's AST.
//! 2. **Custom minimal sentinel** — bypass `nros::Executor` and
//!    speak the IVC wire format directly with a hand-rolled
//!    heartbeat-publish + emergency-stop loop.
//!
//! Both are larger than this phase's scope. Until one lands, the
//! firmware crate stays a `wfi`-loop scaffold so `build-spe-image`
//! produces a bootable (inert) `spe.bin` for CI smoke and hardware
//! bring-up exercise of the FSP boot path.

#![no_std]
#![allow(internal_features)]

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

/// Called from `app/nros-app.c` (added to the BSP tree by
/// `scripts/spe/apply-patches.sh`).
#[unsafe(no_mangle)]
pub extern "C" fn nros_app_rust_entry() {
    run(Config::default(), |config| -> Result<(), &'static str> {
        println!("sentinel-spe-firmware: boot complete on {}", config.zenoh_locator);
        loop {
            unsafe {
                core::arch::asm!("wfi", options(nomem, nostack, preserves_flags));
            }
        }
    });
}
