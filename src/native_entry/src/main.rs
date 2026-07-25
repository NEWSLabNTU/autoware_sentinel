// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Sentinel native Entry (phase 14.4b) — one-line model-arm entry. The
//! macro reads the resolved SystemModel, emits one `register()` per launch
//! `<node>`, opens the executor against the native board, and spins.

nros::main!(model = "sentinel_bringup:config/pilot_model.yaml");
