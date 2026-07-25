// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Phase 14.1 — compile the nano-ros FreeRTOS platform C port into this
// staticlib. Upstream retired the `nros-platform-orin-spe` crate (which
// used to compile its own platform.c); `nros-platform-freertos` is now a
// plain C source package compiled at the consumer's board/build site
// (nros-board-freertos does this for MPS2 via build.rs — the SPE has no
// such board build, so it happens here). Provides `nros_platform_alloc/
// dealloc/wake_signal/wake_wait_ms/...` against the FSP's FreeRTOS
// V10.4.3 headers.

use std::path::PathBuf;

fn env_dir(name: &str, default: &str) -> PathBuf {
    println!("cargo:rerun-if-env-changed={name}");
    std::env::var(name)
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(default))
}

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let repo_root = manifest.ancestors().nth(2).unwrap().to_path_buf();
    let nano_ros = repo_root.parent().unwrap().join("nano-ros");

    let fsp = env_dir(
        "NV_SPE_FSP_DIR",
        repo_root.join("external/spe-fsp/install").to_str().unwrap(),
    );
    let platform_src = nano_ros.join("packages/core/nros-platform-freertos/src");
    let platform_api = nano_ros.join("packages/core/nros-platform-api/include");

    for f in ["platform.c", "timer.c"] {
        println!("cargo:rerun-if-changed={}", platform_src.join(f).display());
    }

    // Same machine flags as the FSP build + the board crate's printf shim
    // (softfp keeps link compat between the FSP `.a` and armv7r-none-eabi
    // rust objects).
    cc::Build::new()
        .file(platform_src.join("platform.c"))
        .file(platform_src.join("timer.c"))
        .include(&platform_api)
        .include(fsp.join("include"))          // FreeRTOSConfig.h
        .include(fsp.join("include/freertos")) // FreeRTOS.h, timers.h
        .include(fsp.join("include/freertos/portable/GCC/ARM_R5")) // portmacro.h
        // FSP's FreeRTOSConfig.h has no configTOTAL_HEAP_SIZE (custom tegra
        // heap). The heap-stat helpers referencing it are unused on the SPE
        // and gc'd at final link; 0 keeps the TU compiling.
        .define("configTOTAL_HEAP_SIZE", "0")
        .flag("-march=armv7-r")
        .flag("-mcpu=cortex-r5")
        .flag("-mfpu=vfpv3-d16")
        .flag("-mfloat-abi=softfp")
        .flag("-Os")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .compile("nros_platform_freertos_spe");
}
