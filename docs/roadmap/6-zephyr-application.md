# Phase 6: Zephyr Application

**Status:** In progress (6.1–6.7 complete)
**Depends on:** Phase 1–4 (algorithm crates), Phase 5 (verification)
**Goal:** Compose all algorithm crates into a single Zephyr application that runs on a
Cortex-M safety MCU, wiring pub/sub callbacks via the nros `Executor` to form a complete
safety island binary.

## Description

Phases 1–4 built 11 pure algorithm libraries. Phase 5 verified their correctness. This
phase creates the deployment artifact: a single `#![no_std]` Zephyr application that
instantiates all algorithms, wires them to ROS 2 topics via nros, and runs a deterministic
control loop on the safety MCU.

This follows the industry-standard pattern for safety MCUs (see
`docs/research/ecu-deployment-patterns.md`): one monolithic binary with static scheduling,
no dynamic allocation, matching both AUTOSAR Classic and micro-ROS deployment models. The
nano-ros `Executor` serves as the RTE — dispatching callbacks, managing pub/sub, and
connecting to the ROS 2 network via Zenoh over Ethernet.

After this phase, the safety island can:
1. Receive heartbeats, vehicle status, and control commands from the main compute via Zenoh
2. Independently monitor for failures and trigger MRM
3. Gate all vehicle commands through rate limiting and safety validation
4. Publish filtered commands directly to actuator ECUs
5. Run on `native_sim` for development and on Cortex-M7 hardware for production

## Architecture

### Single Binary Composition

All 11 algorithm crates compose into one Zephyr application:

```
+----------------------------------------------------------------+
|                    Zephyr Application Image                     |
|                                                                 |
|  +---------------------------+  +---------------------------+  |
|  | Sensing / Input Layer     |  | MRM Chain                 |  |
|  | - VelocityConverter       |  | - HeartbeatWatchdog       |  |
|  | - StopFilter              |  | - MrmHandler              |  |
|  | - Twist2Accel             |  | - EmergencyStopOperator   |  |
|  +---------------------------+  | - ComfortableStopOperator |  |
|                                 +---------------------------+  |
|  +---------------------------+  +---------------------------+  |
|  | Validation Layer          |  | Command Output            |  |
|  | - ControlValidator        |  | - VehicleCmdGate          |  |
|  | - OpModeTransitionMgr     |  | - ShiftDecider            |  |
|  +---------------------------+  +---------------------------+  |
|                                                                 |
|  +-----------------------------------------------------------+ |
|  | nros Executor (single-threaded event loop)                 | |
|  | - Subscriptions, Timers, Publishers, Service servers       | |
|  | - RMW: Zenoh over Ethernet                                 | |
|  +-----------------------------------------------------------+ |
|                                                                 |
|  +-----------------------------------------------------------+ |
|  | Zephyr RTOS + Drivers (Ethernet, CAN, GPIO)               | |
|  +-----------------------------------------------------------+ |
+----------------------------------------------------------------+
```

### Scheduling Model

The nros `Executor` runs all callbacks in a single-threaded event loop. Effective priority
is determined by callback registration order and timer periods.

| Priority | Period       | Callbacks                                            |
|----------|--------------|------------------------------------------------------|
| Highest  | Event-driven | Heartbeat watchdog (timeout = immediate MRM trigger) |
| High     | 30 Hz        | MRM handler update, emergency stop update            |
| Medium   | 30 Hz        | Vehicle command gate, control validator               |
| Normal   | 30 Hz        | Stop filter, velocity converter, shift decider       |
| Low      | 10 Hz        | Operation mode transition manager, twist2accel       |

### Communication Topology

```
Main Compute (Orin / x86)              Safety MCU (S32K3 / native_sim)
  Autoware stack                          autoware_sentinel
       |                                        |
       |-- /heartbeat ----------------------->| HeartbeatWatchdog
       |-- /vehicle/status/velocity --------->| VelocityConverter → StopFilter
       |-- /control/command/control_cmd ----->| ControlValidator, CmdGate
       |-- /system/autoware_state ----------->| ShiftDecider, OpModeTransitionMgr
       |-- /vehicle/status/gear_status ------>| ShiftDecider
       |-- /vehicle/status/control_mode ----->| OpModeTransitionMgr
       |                                        |
       |<-- /output/vehicle/control_cmd ------| CmdGate (filtered)
       |<-- /output/vehicle/gear_cmd ---------| ShiftDecider
       |<-- /output/vehicle/hazard_lights ----| MrmHandler
       |<-- /output/mrm/state ----------------| MrmHandler
       |<-- /output/diagnostics --------------| ControlValidator
```

Transport: Zenoh over Ethernet (`tcp/<router-ip>:7447`), configured via Kconfig.

## Subphases

### 6.1 — Zephyr workspace setup

- [x] Setup scripts and documentation
- [x] Build verification on `native_sim`

Create the Zephyr west workspace that brings together Zephyr RTOS, the zephyr-lang-rust
module, and nano-ros as a Zephyr module.

**Deliverables:**

- `west.yml` — West manifest declaring module dependencies:
  - `zephyr` (v3.7.0) with filtered module allowlist (HALs for STM32/NXP)
  - `zephyr-lang-rust` (Rust support for Zephyr)
  - `self: path: autoware-sentinel` (this repo is the manifest repository)
- `scripts/zephyr/setup.sh` — Automated workspace setup script:
  - Installs Python tools (`west`, `pyelftools`)
  - Adds Rust embedded targets (`thumbv7em-none-eabihf`)
  - Downloads Zephyr SDK (ARM toolchain for linking)
  - Initializes west workspace in sibling directory (`../autoware-sentinel-workspace/`)
  - Fetches Zephyr + zephyr-lang-rust modules via `west update`
  - Symlinks nano-ros (via `--nros-path`, default `../nano-ros`)
  - Generates `env.sh` with `ZEPHYR_EXTRA_MODULES` pointing to nano-ros
- `docs/guides/zephyr-setup.md` — Developer guide for workspace initialization, building,
  and running on `native_sim`

**Acceptance criteria:**
- [x] `source ../autoware-sentinel-workspace/env.sh` succeeds
- [x] `west build -b native_sim/native/64 ...` configures without errors
- [x] Zephyr SDK, west, and zephyr-lang-rust are all functional

### 6.2 — Application crate skeleton

- [x] CMake + Cargo.toml + prj.conf
- [x] Empty `rust_main` builds and boots on `native_sim`

Create the Zephyr application crate that will host all algorithm instances.

**Target:** `src/autoware_sentinel/`

**New files:**

`Cargo.toml`:
```toml
[package]
name = "rustapp"             # Required by zephyr-lang-rust
version = "0.1.0"
edition = "2024"
license = "Apache-2.0"
publish = false

[lib]
crate-type = ["staticlib"]   # Linked into Zephyr image

[dependencies]
zephyr = "0.1.0"
nros = { version = "*", features = ["rmw-zenoh", "platform-zephyr"] }

# Algorithm crates
autoware_stop_filter = { path = "../autoware_stop_filter" }
autoware_vehicle_velocity_converter = { path = "../autoware_vehicle_velocity_converter" }
autoware_shift_decider = { path = "../autoware_shift_decider" }
autoware_mrm_emergency_stop_operator = { path = "../autoware_mrm_emergency_stop_operator" }
autoware_mrm_comfortable_stop_operator = { path = "../autoware_mrm_comfortable_stop_operator" }
autoware_heartbeat_watchdog = { path = "../autoware_heartbeat_watchdog" }
autoware_mrm_handler = { path = "../autoware_mrm_handler" }
autoware_vehicle_cmd_gate = { path = "../autoware_vehicle_cmd_gate" }
autoware_twist2accel = { path = "../autoware_twist2accel" }
autoware_control_validator = { path = "../autoware_control_validator" }
autoware_operation_mode_transition_manager = { path = "../autoware_operation_mode_transition_manager" }

# Message crates (resolved via [patch.crates-io])
autoware_control_msgs = { version = "*", default-features = false }
autoware_vehicle_msgs = { version = "*", default-features = false }
autoware_system_msgs = { version = "*", default-features = false }
tier4_system_msgs = { version = "*", default-features = false }
autoware_adapi_v1_msgs = { version = "*", default-features = false }
geometry_msgs = { version = "*", default-features = false }
nav_msgs = { version = "*", default-features = false }
```

`CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(autoware_sentinel)
rust_cargo_application()
```

`prj.conf` — Zephyr Kconfig:
```kconfig
# Rust support
CONFIG_RUST=y
CONFIG_RUST_ALLOC=y

# nano-ros
CONFIG_NROS=y
CONFIG_NROS_RUST_API=y
CONFIG_NROS_RMW_ZENOH=y
CONFIG_NROS_ZENOH_LOCATOR="tcp/192.0.2.2:7447"

# Networking (for Zenoh transport)
CONFIG_NETWORKING=y
CONFIG_NET_TCP=y
CONFIG_NET_SOCKETS=y
CONFIG_NET_L2_ETHERNET=y

# POSIX API (required by Zenoh)
CONFIG_POSIX_API=y
CONFIG_MAX_PTHREAD_MUTEX_COUNT=32

# Memory
CONFIG_MAIN_STACK_SIZE=32768
CONFIG_HEAP_MEM_POOL_SIZE=131072

# nano-ros tuning
CONFIG_NROS_MAX_PUBLISHERS=16
CONFIG_NROS_MAX_SUBSCRIBERS=16
CONFIG_NROS_MAX_QUERYABLES=8
```

`src/lib.rs` — Skeleton:
```rust
#![no_std]

#[unsafe(no_mangle)]
extern "C" fn rust_main() {
    // Phase 6.3+ will populate this
}
```

**Acceptance criteria:**
- [x] `west build -b native_sim/native/64 src/autoware_sentinel` succeeds
- [x] `west build -b native_sim/native/64 src/autoware_sentinel -t run` boots and exits cleanly
- [x] Binary is `#![no_std]` — no libstd symbols in the final ELF

### 6.3 — Wire sensing / input layer

- [x] Subscriptions and algorithm instances
- [ ] Integration tests on `native_sim`

Wire the three input-processing algorithms that convert raw vehicle data into filtered
signals consumed by downstream components.

**Algorithms:**

| Algorithm                  | Constructor              | Input Topic(s)                                       | Output                                  | Rate       |
|----------------------------|--------------------------|------------------------------------------------------|-----------------------------------------|------------|
| `VehicleVelocityConverter` | `new(1.0, 0.2, 0.1)`     | `/vehicle/status/velocity_status` (`VelocityReport`) | `TwistWithCovarianceStamped` (internal) | On message |
| `StopFilter`               | `new(0.1, 0.02)`         | Converted twist (internal)                           | `FilterResult` (internal)               | On message |
| `Twist2Accel`              | `new(Params::default())` | Converted twist (internal)                           | `Twist2AccelOutput` (internal)          | On message |

**Wiring sketch:**
```rust
// Velocity conversion → stop filter → twist2accel
executor.add_subscription::<VelocityReport, _>("/vehicle/status/velocity_status", move |msg| {
    let twist_cov = velocity_converter.convert(&msg);
    let twist = &twist_cov.twist.twist;
    let filtered = stop_filter.apply(&twist.linear, &twist.angular);
    // Store filtered velocity for downstream consumers
    // Feed twist2accel for acceleration estimation
})?;
```

**Acceptance criteria:**
- [x] All three algorithms instantiated and receiving data in the executor
- [x] Filtered velocity available to MRM chain and command gate
- [x] Acceleration estimate available to MRM operators

### 6.4 — Wire MRM chain

- [x] Heartbeat watchdog + MRM handler + stop operators
- [ ] Service servers for MRM operate commands
- [ ] Integration tests on `native_sim`

Wire the emergency response pipeline that forms the core safety function.

**Algorithms:**

| Algorithm                 | Constructor                       | Interface                                                                             | Rate                     |
|---------------------------|-----------------------------------|---------------------------------------------------------------------------------------|--------------------------|
| `HeartbeatWatchdog`       | `new(Params { timeout_ms: 500 })` | Sub: `/heartbeat`                                                                     | On message + 30 Hz check |
| `MrmHandler`              | `new(Params::default())`          | Pub: `/output/mrm/state`, `/output/vehicle/hazard_lights`, `/output/vehicle/gear_cmd` | 30 Hz                    |
| `EmergencyStopOperator`   | `new(Params::default())`          | Srv: `/mrm/emergency_stop/operate`                                                    | 30 Hz update             |
| `ComfortableStopOperator` | `new(Params::default())`          | Srv: `/mrm/comfortable_stop/operate`                                                  | 30 Hz update             |

**Data flow:**
```
HeartbeatWatchdog
  on_heartbeat() ← /heartbeat subscription
  check(now_ms)  → updates MrmHandler.update_availability()
                         ↓
                    MrmHandler.update()
                      → MrmOutput.emergency_stop_operate → EmergencyStopOperator.operate()
                      → MrmOutput.comfortable_stop_operate → ComfortableStopOperator.operate()
                      → publish MrmState, HazardLightsCommand, GearCommand
                         ↓
                    EmergencyStopOperator.update(dt) → Control (fed to CmdGate)
                    ComfortableStopOperator.update(dt) → Control (fed to CmdGate)
```

**Acceptance criteria:**
- [x] Heartbeat timeout triggers MRM within one control period (33 ms)
- [x] MRM handler escalates from comfortable stop to emergency stop
- [x] Emergency stop operator converges velocity to zero
- [x] MRM state and hazard lights published on correct topics

### 6.5 — Wire command output layer

- [x] Vehicle command gate + shift decider
- [x] Source arbitration (autonomous / emergency)
- [ ] Integration tests on `native_sim`

Wire the command gating and gear selection that form the final output stage.

**Algorithms:**

| Algorithm        | Constructor                  | Input Topics                                            | Output Topics                 | Rate       |
|------------------|------------------------------|---------------------------------------------------------|-------------------------------|------------|
| `VehicleCmdGate` | `new(GateParams::default())` | `/control/command/control_cmd`, MRM control (internal)  | `/output/vehicle/control_cmd` | 30 Hz      |
| `ShiftDecider`   | `new(true)`                  | `/system/autoware_state`, `/vehicle/status/gear_status` | `/output/vehicle/gear_cmd`    | On message |

**Wiring sketch:**
```rust
// Autonomous commands from main compute
executor.add_subscription::<Control, _>("/control/command/control_cmd", move |msg| {
    gate.set_autonomous_commands(to_source_commands(&msg), now_ms);
})?;

// Emergency commands from MRM operators (internal wiring)
// gate.set_emergency_commands(...) called from MRM timer callback

// 30 Hz output
executor.add_timer(33, move || {
    let output = gate.update(now_ms);
    control_pub.publish(&output.control).ok();
    gear_pub.publish(&output.gear).ok();
    turn_pub.publish(&output.turn_indicators).ok();
    hazard_pub.publish(&output.hazard_lights).ok();
})?;
```

**Acceptance criteria:**
- [x] Autonomous commands pass through gate with rate limiting applied
- [x] Emergency source preempts autonomous source
- [x] Gear commands published correctly based on vehicle state
- [x] All output topics populated at 30 Hz

### 6.6 — Wire validation layer

- [x] Control validator + operation mode transition manager
- [ ] Diagnostic output
- [ ] Integration tests on `native_sim`

Wire the cross-checking components that monitor command quality.

**Algorithms:**

| Algorithm | Constructor | Input | Output | Rate |
|-----------|-------------|-------|--------|------|
| `ControlValidator` | `new(ValidatorParams::default())` | Control commands, vehicle velocity | `ValidationStatus` (internal + diagnostic pub) | 30 Hz |
| `OperationModeTransitionManager` | `new(Params::default())` | Velocity, control commands, mode requests | `TransitionDiagnostics` | 10 Hz |

**Data flow:**
```
ControlValidator.validate(control_cmd, velocity, accel, target_vel, pitch, dt)
  → ValidationStatus
  → If invalid for N consecutive frames → signal MrmHandler to trigger MRM

OpModeTransitionMgr.update_velocity(velocity)
OpModeTransitionMgr.update_control_cmd(cmd)
OpModeTransitionMgr.update(dt)
  → TransitionDiagnostics
  → Controls whether autonomy can engage
```

**Acceptance criteria:**
- [x] Control validator detects unsafe commands (overspeed, excessive jerk)
- [x] Validation failure triggers MRM after threshold consecutive violations
- [x] Operation mode transitions respect vehicle state constraints
- [ ] Diagnostic status published for external monitoring

### 6.7 — Shared state and intra-node data flow

- [x] Zero-copy shared state between callbacks
- [x] Correct data ordering (sense → validate → gate → publish)

The Zephyr application must share mutable state between callbacks without dynamic allocation.
Since the nros `Executor` is single-threaded, no synchronization primitives are needed —
all callbacks run sequentially in the spin loop.

**Shared state pattern:**
```rust
// All algorithm instances and shared buffers in a single struct
struct SafetyIsland {
    // Input processing
    velocity_converter: VehicleVelocityConverter,
    stop_filter: StopFilter,
    twist2accel: Twist2Accel,

    // MRM chain
    watchdog: HeartbeatWatchdog,
    mrm_handler: MrmHandler,
    emergency_stop: EmergencyStopOperator,
    comfortable_stop: ComfortableStopOperator,

    // Output
    cmd_gate: VehicleCmdGate,
    shift_decider: ShiftDecider,

    // Validation
    control_validator: ControlValidator,
    op_mode_mgr: OperationModeTransitionManager,

    // Shared data (written by one callback, read by others)
    current_velocity: f64,
    current_acceleration: f64,
    filtered_twist: Option<FilterResult>,
    is_stopped: bool,
}
```

Callbacks hold `&mut SafetyIsland` references and execute in deterministic order within
each spin iteration, ensuring consistent data flow: sense → validate → gate → publish.

**Acceptance criteria:**
- [x] No dynamic allocation (`#[global_allocator]` not used in application code)
- [x] Data flows in correct order within each control period
- [x] All internal state accessible without `Arc`, `Mutex`, or heap allocation
- [ ] Binary compiles for `thumbv7em-none-eabihf` (Cortex-M7)

### 6.8 — Board support and hardware deployment

- [ ] Cortex-M7 board overlay and devicetree
- [ ] CAN bus output (optional, depends on hardware)
- [ ] Flash and run on target hardware

Add board-specific configuration for a real safety MCU.

**Primary target:** NXP S32K344 (Cortex-M7, dual-core lockstep, ASIL D)
**Development target:** STM32H743 (Cortex-M7, widely available)

**Deliverables:**

- Board-specific overlay files (`boards/nxp_s32k344.overlay`, `boards/stm32h743.overlay`)
- Board-specific Kconfig fragments (`boards/nxp_s32k344.conf`, `boards/stm32h743.conf`)
- Network configuration (Ethernet or CAN-FD for Zenoh transport)
- Memory layout validation (flash usage, RAM usage, stack depth analysis)

**Acceptance criteria:**
- [ ] Binary fits in target flash (< 2 MB for S32K344, < 1.5 MB for STM32H743)
- [ ] RAM usage within target limits (< 384 KB for S32K344)
- [ ] Boot-to-first-heartbeat-check latency < 500 ms
- [ ] Control loop maintains 30 Hz without deadline misses

## Acceptance Criteria (Phase-Level)

- [ ] Single `west build` produces a complete safety island binary
- [ ] All 11 algorithm crates compose into one `#![no_std]` Zephyr application
- [ ] Application runs on `native_sim` with Zenoh transport to a ROS 2 host
- [ ] Binary cross-compiles to `thumbv7em-none-eabihf` (Cortex-M7)
- [ ] No dynamic allocation in application code (heap-free)
- [ ] 30 Hz control loop sustained on `native_sim`
- [ ] Heartbeat timeout → MRM → emergency stop → v=0 end-to-end on `native_sim`
- [ ] `just ci` still passes (algorithm crate tests unaffected)
- [ ] Binary size < 2 MB (flash), RAM < 384 KB (suitable for S32K3 class MCUs)

## References

- `docs/research/ecu-deployment-patterns.md` — Industry deployment patterns
- `~/repos/nano-ros/zephyr/` — nano-ros Zephyr module (CMake, Kconfig)
- `~/repos/nano-ros/examples/zephyr/rust/zenoh/` — Reference Zephyr Rust examples
- `~/repos/nano-ros/scripts/zephyr/setup.sh` — Zephyr workspace setup reference
- `~/repos/nano-ros/docs/guides/zephyr-setup.md` — Zephyr developer guide
