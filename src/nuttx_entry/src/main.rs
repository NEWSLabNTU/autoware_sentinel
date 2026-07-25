// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Sentinel NuttX Entry (phase 14.5) — model-arm macro entry over the
//! 10-node MCU subset (same model as the FreeRTOS entry; the deploy key
//! differs). The NuttX board owns boot + spin (nsh_main shim).

nros::main!(model = "sentinel_bringup:config/nuttx_model.yaml");
