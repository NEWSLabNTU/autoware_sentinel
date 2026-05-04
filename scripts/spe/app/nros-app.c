/*
 * Copyright 2026 Autoware Sentinel contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Phase 11.5.c — C shim for the nano-ros / sentinel SPE app.
 *
 * This file is copied into NVIDIA's `rt-aux-cpu-demo-fsp/app/` tree
 * by `scripts/spe/apply-patches.sh`. Built when `ENABLE_NROS_APP := 1`
 * in the per-SOC `target_specific.mk` (added by patch 0001).
 *
 * Responsibilities:
 *   1. Provide the standard NVIDIA `*_app_init()` entry-point name so
 *      `main_task` can call it via the `ENABLE_*_APP` switch.
 *   2. Forward into Rust via the `extern "C"` symbol exported by
 *      `libsentinel_spe_firmware.a`.
 *
 * Why minimal:
 *   - Hardware init (HSP doorbell, IVC carveout, BPMP IPC) is owned by
 *     the FSP. By the time `main_task` runs we only need to spawn the
 *     application task — `nros_app_rust_entry()` does that via
 *     `xTaskCreate` and returns.
 *   - No #include of <stdio.h>; the FSP's `printf` is wired through
 *     the TCU automatically when the demo's debug_init() runs earlier
 *     in `main_task`.
 */

extern void nros_app_rust_entry(void);

/* Forward decl — BSP CFLAGS include `-Wmissing-prototypes -Werror`. */
void nros_app_init(void);

void nros_app_init(void)
{
    nros_app_rust_entry();
}
