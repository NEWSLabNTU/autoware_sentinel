// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Sentinel FreeRTOS Entry (phase 14.5) — model-arm macro entry over the
//! 10-node MCU subset. The board crate owns boot, lwIP bring-up, RMW
//! registration, and the spin loop.

#![no_std]
#![no_main]

use panic_semihosting as _;

nros::main!(model = "sentinel_bringup:config/freertos_model.yaml");
