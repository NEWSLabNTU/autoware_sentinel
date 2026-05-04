// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! Autoware Sentinel — SPE firmware (Phase 11.5 scaffold).
//!
//! Thin staticlib that NVIDIA's `rt-aux-cpu-demo-fsp/main_task` calls
//! into via the `nros_app_init()` C shim. The closure is currently a
//! `wfi`-loop placeholder — Phase 11.3 is BLOCKED on three nano-ros
//! gaps (see `docs/roadmap/11-orin-spe.md` §11.3 blockers); the
//! `nros::Executor` + `autoware_sentinel_core::wire_executor()` body
//! lands once those clear.
//!
//! Until then this crate proves the boot path:
//! - `nros_app_rust_entry()` is callable from C,
//! - `nros-board-orin-spe::run()` xTaskCreates the application task,
//! - the task survives the FreeRTOS scheduler resume,
//! - `println!` reaches the TCU via FSP `printf`.

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
