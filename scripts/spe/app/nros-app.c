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

/*
 * Phase 14.1 — newlib `clock_gettime` shim. zenoh-pico's session-seed
 * path (zpico.c `#elif defined(CLOCK_REALTIME)`) calls clock_gettime,
 * and the SPE's newlib has no syscall backend for it. Derive both
 * clocks from the FSP FreeRTOS tick via the nano-ros platform clock.
 */
#include <time.h>
#include <stdint.h>

extern uint64_t nros_platform_time_now_ms(void);

int clock_gettime(clockid_t clock_id, struct timespec *tp);
int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
    (void)clock_id;
    if (tp == 0) {
        return -1;
    }
    uint64_t ms = nros_platform_time_now_ms();
    tp->tv_sec = (time_t)(ms / 1000u);
    tp->tv_nsec = (long)((ms % 1000u) * 1000000u);
    return 0;
}

/* Forward decl — BSP CFLAGS include `-Wmissing-prototypes -Werror`. */
void nros_app_init(void);

void nros_app_init(void)
{
    nros_app_rust_entry();
}
