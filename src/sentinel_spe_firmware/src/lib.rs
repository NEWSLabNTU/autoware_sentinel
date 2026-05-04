// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! Autoware Sentinel — SPE firmware wrap (Phase 11.5.b).
//!
//! Thin staticlib that NVIDIA's `rt-aux-cpu-demo-fsp/main_task` calls
//! into via the `nros_app_init()` C shim (Phase 11.5.c). Sequence:
//!
//! 1. NVIDIA's `main()` runs the FSP HW init (HSP, BPMP IPC, IVC
//!    carveout) and calls `rtosTaskInitializeScheduler` — that creates
//!    `main_task` and starts the FreeRTOS scheduler.
//! 2. Inside `main_task`, the patched call to `nros_app_init()` runs
//!    `nros_app_rust_entry()` (this file's `extern "C"` symbol).
//! 3. `nros_app_rust_entry()` calls `nros_board_orin_spe::run(Config,
//!    closure)`, which `xTaskCreate`s the application task and
//!    returns. Control flows back through the chain to NVIDIA's
//!    `main_task`, which `vTaskDelete(NULL)`s itself.
//! 4. The application task starts running once the scheduler resumes.
//!
//! This crate is deliberately minimal for Phase 11.5 — its job is to
//! prove the boot path + IVC connectivity end-to-end. The reduced
//! sentinel algorithm set (heartbeat watchdog, MRM, cmd-gate) wires
//! into the closure under Phase 11.3.

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
/// `scripts/spe/patches/0001-add-ENABLE_NROS_APP-target-flag.patch`).
///
/// `extern "C"` + `#[no_mangle]` so the C symbol resolves through the
/// link.
#[unsafe(no_mangle)]
pub extern "C" fn nros_app_rust_entry() {
    run(Config::default(), |config| -> Result<(), &'static str> {
        // Phase 11.5 scaffold: announce + idle. Replace with the
        // SafetyIsland wiring under Phase 11.3 once IVC link delivery
        // is verified end-to-end on hardware.
        println!("sentinel-spe-firmware: boot complete on {}", config.zenoh_locator);
        loop {
            // The board crate's run() already xTaskCreate'd this task;
            // when the closure returns, the task `wfi`-loops. We park
            // here explicitly so the bring-up path is grep-friendly.
            unsafe {
                core::arch::asm!("wfi", options(nomem, nostack, preserves_flags));
            }
        }
    });
}
