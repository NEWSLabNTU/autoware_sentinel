// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Sentinel Zephyr Entry (phase 14.5) — model-arm macro entry over the
//! 10-node MCU subset, built as a Zephyr west application.

#![no_std]

// Zephyr owns the allocator / panic / boot; pull the crate in so the
// kernel's Rust glue links.
extern crate zephyr;

nros::main!(model = "sentinel_bringup:config/zephyr_model.yaml");
