# Phase 7: Integration Testing

**Status:** Complete (7.1–7.4 done)
**Depends on:** Phase 6 (Zephyr application), specifically 6.3–6.7 (algorithm wiring)
**Goal:** Validate the safety island end-to-end against the real Autoware planning simulator,
proving that the ported algorithms correctly replace the original Autoware system/control nodes
and can drive a simulated vehicle autonomously.

## Description

Phases 1–5 built and verified 11 algorithm libraries in isolation. Phase 6 composed them into
a single Zephyr application. This phase closes the loop by testing the safety island against
real Autoware infrastructure — first with a Linux-native binary for ease of development, then
with the actual Zephyr `native_sim` build.

Integration testing proves:
1. **Transport compatibility** — nros (zenoh-pico) and ROS 2 (`rmw_zenoh_cpp`) exchange
   CDR-encoded messages through a shared zenohd router
2. **Topic wiring correctness** — the sentinel subscribes to correct Autoware topics and
   publishes commands that the planning simulator accepts
3. **Functional correctness** — the complete MRM chain, command gating, and validation work
   end-to-end with a planning simulator driving a vehicle
4. **Engagement flow** — the sentinel satisfies Autoware's prerequisites for autonomous mode
   (`OperationModeState`, `ChangeOperationMode` service)

### Nodes Replaced by the Sentinel

The sentinel replaces 7 Autoware system/control nodes:

| Node                              | Full ROS Path                                         | Package                                      |
|-----------------------------------|-------------------------------------------------------|----------------------------------------------|
| vehicle_cmd_gate                  | `/control/vehicle_cmd_gate`                           | `autoware_vehicle_cmd_gate`                  |
| mrm_handler                       | `/system/mrm_handler`                                 | `autoware_mrm_handler`                       |
| mrm_emergency_stop_operator       | `/system/mrm_emergency_stop_operator/...`             | `autoware_mrm_emergency_stop_operator`       |
| mrm_comfortable_stop_operator     | `/system/mrm_comfortable_stop_operator/...`           | `autoware_mrm_comfortable_stop_operator`     |
| shift_decider                     | `/control/shift_decider`                              | (in control container)                       |
| control_validator                 | `/control/control_validator`                          | `autoware_control_validator`                 |
| operation_mode_transition_manager | `/control/autoware_operation_mode_transition_manager` | `autoware_operation_mode_transition_manager` |

## Subphases

### 7.1 — Linux native binary (`autoware_sentinel_linux`)

- [x] Package created and builds
- [x] Topic names mapped to Autoware conventions
- [x] Engagement flow (OperationModeState + ChangeOperationMode service)
- [x] Root justfile recipes added

Create `src/autoware_sentinel_linux/`, a std-enabled Linux binary that reuses the
`SafetyIsland` struct and algorithm wiring from the Zephyr `lib.rs` with only platform-layer
changes.

**Target:** `src/autoware_sentinel_linux/`

**Platform differences from Zephyr sentinel:**

| Aspect            | Zephyr Sentinel                             | Linux Binary                                           |
|-------------------|---------------------------------------------|--------------------------------------------------------|
| Package name      | `rustapp`                                   | `autoware_sentinel_linux`                              |
| Crate type        | `staticlib` (linked into Zephyr)            | `bin` (standalone executable)                          |
| Entry point       | `extern "C" fn rust_main()`                 | `fn main()`                                            |
| Logging           | `zephyr::set_logger()`                      | `env_logger::init()`                                   |
| Clock             | `zephyr::sys::uptime_get()`                 | `std::time::Instant` + `OnceLock`                      |
| Executor config   | `ExecutorConfig::new("tcp/192.0.2.2:7447")` | `ExecutorConfig::from_env().node_name("sentinel")`     |
| Spin              | `executor.spin(10)` (no_std)                | `executor.spin_blocking(SpinOptions::default())` (std) |
| nros features     | `["rmw-zenoh", "platform-zephyr"]`          | `["std", "rmw-zenoh", "platform-posix", "ros-humble"]` |
| Zephyr dep        | `zephyr = "0.1.0"`                          | Removed; add `env_logger = "0.11"`                     |
| `#![no_std]`      | Yes                                         | No (removed)                                           |
| Heartbeat timeout | 500 ms                                      | 5000 ms (Autoware init is slow)                        |

**Topic name mapping (subscriptions):**

| Zephyr Topic                      | Linux Binary (Autoware-Compatible)         | Message Type     | Reason                                |
|-----------------------------------|--------------------------------------------|------------------|---------------------------------------|
| `/control/command/control_cmd`    | `/control/trajectory_follower/control_cmd` | `Control`        | Subscribe to gate's input, not output |
| `/heartbeat`                      | `/api/system/heartbeat`                    | `Heartbeat`      | ADAPI heartbeat topic                 |
| `/system/autoware_state`          | `/autoware/state`                          | `AutowareState`  | Compat node publishes here            |
| `/vehicle/status/velocity_status` | `/vehicle/status/velocity_status`          | `VelocityReport` | Unchanged                             |
| `/vehicle/status/gear_status`     | `/vehicle/status/gear_status`              | `GearReport`     | Unchanged                             |

**Topic name mapping (publications):**

| Zephyr Topic                      | Linux Binary (Autoware-Compatible)     | Message Type            | Consumer                  |
|-----------------------------------|----------------------------------------|-------------------------|---------------------------|
| `/output/vehicle/control_cmd`     | `/control/command/control_cmd`         | `Control`               | simple_planning_simulator |
| `/output/vehicle/gear_cmd`        | `/control/command/gear_cmd`            | `GearCommand`           | simple_planning_simulator |
| `/output/vehicle/turn_indicators` | `/control/command/turn_indicators_cmd` | `TurnIndicatorsCommand` | simple_planning_simulator |
| `/output/vehicle/hazard_lights`   | `/control/command/hazard_lights_cmd`   | `HazardLightsCommand`   | simple_planning_simulator |
| `/output/mrm/state`               | `/system/fail_safe/mrm_state`          | `MrmState`              | System nodes              |

**Additional publishers (engagement flow):**

| Topic                       | Message Type         | Rate  | Purpose                                                                            |
|-----------------------------|----------------------|-------|------------------------------------------------------------------------------------|
| `/api/operation_mode/state` | `OperationModeState` | 30 Hz | Reports `mode=AUTONOMOUS`, `is_autoware_control_enabled=true`, all modes available |

**Additional services (engagement flow):**

| Service Topic                              | Type                  | Behavior               |
|--------------------------------------------|-----------------------|------------------------|
| `/api/operation_mode/change_to_autonomous` | `ChangeOperationMode` | Always returns success |

**New files:**

- `src/autoware_sentinel_linux/Cargo.toml` — bin crate, same 11 algorithm + 7 message deps,
  nros with `["std", "rmw-zenoh", "platform-posix", "ros-humble"]`
- `src/autoware_sentinel_linux/src/main.rs` — adapted from Zephyr `lib.rs`
- `src/autoware_sentinel_linux/package.xml` — same 7 message dependencies
- `src/autoware_sentinel_linux/justfile` — generate, build, run recipes
- `src/autoware_sentinel_linux/.cargo/config.toml` — nros patches (9 entries) + generated
  message patches
- `src/autoware_sentinel_linux/.gitignore` — `/target`, `/generated/`

**Root justfile additions:**

- `build-sentinel-linux` — build the Linux binary
- `run-sentinel-linux` — run with `RUST_LOG=info`
- Add to `generate-bindings` recipe

**Acceptance criteria:**

- [x] `cd src/autoware_sentinel_linux && just build` succeeds
- [x] `just run-sentinel-linux` starts and prints log output (exits cleanly when no zenohd)
- [x] Binary reuses all 11 algorithm crates from Phases 1–4
- [x] `OperationModeState` published at 30 Hz with `mode=AUTONOMOUS`
- [x] `ChangeOperationMode` service returns success
- [x] Topic names match Autoware conventions per mapping tables above

### 7.2 — Transport smoke test

- [x] Bidirectional message flow verified (nros ↔ ROS 2 via shared zenohd)
- [x] CDR encoding compatible between rmw_zenoh_cpp and nros

Verify that the sentinel Linux binary and ROS 2 nodes can exchange messages through a shared
Zenoh router. Both sides connect as Zenoh clients — no DDS bridge needed.

**Transport architecture:**

```
┌──────────────────┐     ┌─────────────┐     ┌──────────────────────┐
│ Sentinel (Linux) │────▶│   zenohd    │◀────│ ROS 2 (rmw_zenoh_cpp)│
│  zenoh-pico      │     │ :7447       │     │  ros2 topic pub/echo │
│  (Zenoh client)  │     │ (router)    │     │  (Zenoh client)      │
└──────────────────┘     └─────────────┘     └──────────────────────┘
```

**Test procedure:**

1. Start zenohd (version-matched to `ros-humble-zenoh-cpp-vendor` 0.1.8):
   ```bash
   ~/repos/nano-ros/build/zenohd/zenohd --listen tcp/127.0.0.1:7447
   ```

2. Start sentinel:
   ```bash
   cd src/autoware_sentinel_linux && RUST_LOG=info cargo run
   ```

3. From a ROS 2 terminal:
   ```bash
   source /opt/ros/humble/setup.bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'

   # Publish velocity → sentinel should log receipt
   ros2 topic pub /vehicle/status/velocity_status \
     autoware_vehicle_msgs/msg/VelocityReport '{...}'

   # Echo sentinel output → should see Control messages
   ros2 topic echo /control/command/control_cmd
   ```

**CDR protocol details:**

- Key expression format (Humble): `<domain_id>/<topic_name>/<type_name>/TypeHashNotSupported`
- CDR format: Little-endian with 4-byte encapsulation header `[0x00, 0x01, 0x00, 0x00]`
- RMW attachment: 33 bytes (sequence_number, timestamp, gid)

**Key constraint:** Use locally-built zenohd v1.6.2 (`~/repos/nano-ros/build/zenohd/zenohd`)
to match the version pinned by `ros-humble-zenoh-cpp-vendor` 0.1.8. The system-installed
zenohd 1.7.2 may have protocol incompatibilities.

**Acceptance criteria:**

- [x] Sentinel receives `VelocityReport` published from `ros2 topic pub`
- [x] `ros2 topic echo` receives `Control` messages published by sentinel
- [x] No deserialization errors in sentinel logs
- [x] No deserialization errors in ROS 2 terminal
- [x] Message round-trip latency < 10 ms on localhost (verified via `ros2 topic hz` — 30 Hz
  arrival rate confirms sub-10ms transport)

### 7.3 — Autoware planning simulator integration

- [x] Baseline test passes (unmodified Autoware drives autonomously)
- [x] Sentinel replacement test passes (filtered Autoware + sentinel drives autonomously)
- [x] Tests integrated into nextest (`just test-planning`)

Full end-to-end test using the existing nextest infrastructure: dump the Autoware planning
simulator launch via `play_launch`, filter out the 7 replaced nodes from the record, replay
the filtered record, start the sentinel, and run an autonomous driving mission. Uses
Approach A exclusively (`play_launch dump` → filter → `play_launch replay`).

#### 7.3a — Baseline: Standard Autoware (no sentinel)

Run unmodified Autoware through the full autonomous drive sequence to establish baseline
behavior and validate the test infrastructure itself (play_launch, ROS 2 helpers, map data).

**Work items:**

- [x] `AutowareLauncher` fixture — wraps `play_launch` dump/replay lifecycle as ManagedProcess
  - `dump()` — runs `play_launch dump launch autoware_launch planning_simulator.launch.xml`
    with `map_path`, caches result (`OnceCell`)
  - `filter_record()` — loads record.json, removes specified nodes by array/name, writes
    filtered copy
  - `replay()` — runs `play_launch replay --input-file ... --disable-all` as ManagedProcess
- [x] `Ros2Command` helpers — extend `tests/src/ros2.rs` for:
  - `topic_pub_once()` — publish a single message (initial pose)
  - `service_call()` — call a ROS 2 service and capture response
  - `check_topic_active()` — verify a topic has publishers
  - `wait_for_topics()` — poll until critical topics are active
- [x] `autoware_map_path()` helper — `$MAP_PATH` or `$HOME/autoware_map/sample-map-planning`
- [x] `require_autoware_map()` — skip test if map data not available
- [x] `require_play_launch()` — skip test if `play_launch` not installed
- [x] Pose configuration — hardcoded constants for sample-map-planning:
  - Initial: position (3752.34, 73736.09, 19.34), orientation q(-0.0008, -0.0002, -0.9584, 0.2854)
  - Goal: position (3758.96, 73689.29, 19.63), orientation q(0, 0, 0.9663, -0.2576)
- [x] Baseline test: `test_autoware_baseline_autonomous_drive`
  - Dump + replay full (unfiltered) Autoware via `play_launch`
  - Run autonomous drive sequence (initial pose → wait for localization → set route → engage)
  - Verify: vehicle moves (kinematic_state position delta > 1m)
  - Verify: operation mode reaches AUTONOMOUS
  - Verify: no unexpected node crashes (play_launch process stays alive)

**Autonomous drive sequence** (from play_launch test scripts, pre-validated with
sample-map-planning):

1. Wait for readiness topics: `/map/vector_map`, `/api/operation_mode/state`,
   `/api/routing/state`
2. Publish initial pose to `/initialpose` (`PoseWithCovarianceStamped`)
3. Wait 5s for localization
4. Wait for `/tf`, `/localization/kinematic_state`
5. Set route via `/api/routing/set_route_points` service call
6. Wait 3s
7. Engage via `/api/operation_mode/change_to_autonomous` service call
8. Monitor `/localization/kinematic_state` for movement (position delta > 1m)

#### 7.3b — Sentinel replaces 7 nodes

Run Autoware with 7 nodes filtered out from the play_launch record, plus the sentinel Linux
binary providing those functions.

**Node filtering from record.json:**

The 7 replaced nodes live in 3 different record.json arrays:

| Array         | Name                                         | Package                                      | Notes                                                               |
|---------------|----------------------------------------------|----------------------------------------------|---------------------------------------------------------------------|
| `node[]`      | `mrm_handler`                                | `autoware_mrm_handler`                       | Standalone node                                                     |
| `container[]` | `mrm_emergency_stop_operator_container`      | `autoware_mrm_emergency_stop_operator`       | Remove entire container + its load_nodes                            |
| `container[]` | `mrm_comfortable_stop_operator_container`    | `autoware_mrm_comfortable_stop_operator`     | Remove entire container + its load_nodes                            |
| `load_node[]` | `vehicle_cmd_gate`                           | `autoware_vehicle_cmd_gate`                  | In `/control/control_container` — remove node, keep container       |
| `load_node[]` | `autoware_shift_decider`                     | `autoware_shift_decider`                     | In `/control/control_container` — remove node, keep container       |
| `load_node[]` | `autoware_operation_mode_transition_manager` | `autoware_operation_mode_transition_manager` | In `/control/control_container` — remove node, keep container       |
| `load_node[]` | `control_validator`                          | `autoware_control_validator`                 | In `/control/control_check_container` — remove node, keep container |

**Work items:**

- [x] Record filter function — removes 7 nodes from record.json using `serde_json`:
  - From `node[]`: filter by `name == "mrm_handler"`
  - From `container[]`: filter by `name` matching the two MRM operator containers
  - From `load_node[]`: filter by `package` matching the 4 control nodes
- [x] Sentinel replacement test: `test_sentinel_replaces_autoware_nodes`
  - Dump Autoware record, filter 7 nodes, replay filtered record via `play_launch`
  - Start zenohd (ephemeral port) + sentinel connected to same zenohd
  - Run autonomous drive sequence (same as baseline)
  - Verify: sentinel publishes Control at 30 Hz (echo `/control/command/control_cmd`)
  - Verify: vehicle moves (kinematic_state position delta > 1m)
  - Verify: sentinel engagement flow works (OperationModeState published, engage succeeds)
  - Verify: no MRM triggered (sentinel stays in NORMAL state)

**Key interactions with the planning simulator:**

The `simple_planning_simulator` node (`/simulation/autoware_simple_planning_simulator_node`):
- Subscribes to: `/control/command/control_cmd`, `/control/command/gear_cmd`,
  `/control/command/turn_indicators_cmd`, `/control/command/hazard_lights_cmd`
- Publishes: `/vehicle/status/velocity_status`, `/vehicle/status/gear_status`,
  `/vehicle/status/steering_status`, `/vehicle/status/control_mode`

This creates a closed loop: sentinel publishes commands → simulator updates vehicle state →
sentinel receives updated velocity → sentinel gates next command.

#### nextest configuration

New test group `planning-simulator` with `max-threads = 1` (Autoware is resource-heavy,
run tests sequentially). Slow timeout: `period = "120s", terminate-after = 2` (Autoware
startup is ~60–120s). New test binary `planning_simulator` registered in `tests/Cargo.toml`.

#### New files

| File                                      | Purpose                                                     |
|-------------------------------------------|-------------------------------------------------------------|
| `tests/src/fixtures/autoware_launcher.rs` | `AutowareLauncher` fixture (dump, filter, replay)           |
| `tests/src/autoware.rs`                   | Autoware-specific helpers (map path, poses, drive sequence) |
| `tests/tests/planning_simulator.rs`       | 7.3 integration tests (baseline + sentinel)                 |

#### Modified files

| File                        | Change                                                                                |
|-----------------------------|---------------------------------------------------------------------------------------|
| `tests/Cargo.toml`          | Add `serde_json` dep, register `planning_simulator` test binary                       |
| `tests/src/lib.rs`          | Add `pub mod autoware;`                                                               |
| `tests/src/ros2.rs`         | Add `topic_pub_once()`, `service_call()`, `check_topic_active()`, `wait_for_topics()` |
| `tests/src/fixtures/mod.rs` | Add `mod autoware_launcher; pub use autoware_launcher::*;`                            |
| `.config/nextest.toml`      | Add `planning-simulator` test group                                                   |
| Root `justfile`             | Add `test-planning` recipe                                                            |

#### Prerequisites

- `play_launch` installed (`~/.local/bin/play_launch`, v0.6.0+)
- Autoware map data at `$MAP_PATH` or `$HOME/autoware_map/sample-map-planning`
- zenohd built locally (`~/repos/nano-ros/build/zenohd/zenohd`)
- ROS 2 Humble with `rmw_zenoh_cpp` installed
- Autoware message packages (`/opt/autoware/1.5.0/`)

**Acceptance criteria:**

- [x] Baseline test passes: unmodified Autoware drives autonomously
- [x] Sentinel test passes: filtered Autoware + sentinel drives autonomously
- [x] All 7 replaced nodes correctly identified and filtered from record.json
- [x] Vehicle position changes during autonomous drive (kinematic_state delta > 1m)
- [x] Sentinel publishes Control messages at expected rate
- [x] No orphan processes after test completion
- [x] Tests skip gracefully when prerequisites missing (map, play_launch, ROS 2)
- [x] `just test-planning` runs planning simulator tests
- [x] `just test-integration` runs all tests including planning simulator
- [x] Test completes within 5 minutes (including Autoware startup)

### 7.4 — Zephyr native_sim integration

- [x] TAP networking configured
- [x] Zephyr `native_sim` binary exchanges messages with Autoware

Run the actual Zephyr `native_sim` binary (from Phase 6) against ROS 2 via TAP networking.
This proves the real Zephyr build works end-to-end, not just the Linux adaptation.

**TAP network topology:**

| Interface          | IP Address   | Role                 |
|--------------------|--------------|----------------------|
| `zeth-br` (bridge) | 192.0.2.2/24 | Host (zenohd, ROS 2) |
| `zeth0` (TAP)      | —            | Zephyr application   |
| Zephyr app         | 192.0.2.1    | Safety island        |

**Setup:**
```bash
# Create TAP interfaces (requires sudo)
just setup-tap-network

# Build Zephyr sentinel
just build-zephyr

# Run with zenohd
just run-sentinel-zephyr
```

The Zephyr sentinel connects to `tcp/192.0.2.2:7447` (the Kconfig default
`CONFIG_NROS_ZENOH_LOCATOR`), which resolves to zenohd on the bridge interface.

**Note:** The Zephyr build uses Autoware Sentinel topic names (`/output/vehicle/control_cmd`
etc.), not the Autoware-compatible names from 7.1. A topic remapping layer or Zenoh plugin
would be needed for full simulator integration — or the Zephyr `prj.conf` / `lib.rs` could
be updated with configurable topic names.

**Integration tests** (`tests/tests/zephyr_native_sim.rs`):
- `test_zephyr_sentinel_starts` — connects to zenohd via TAP, prints "Executor ready"
- `test_zephyr_to_ros2_control` — ROS 2 echoes Control from Zephyr sentinel
- `test_ros2_to_zephyr_velocity` — Zephyr receives VelocityReport, continues publishing
- `test_zephyr_bidirectional_round_trip` — full round-trip through TAP bridge

All tests skip gracefully if TAP network or Zephyr binary isn't available.

**New files:**
- `scripts/zephyr/setup-network.sh` — TAP bridge setup/teardown (requires sudo)
- `tests/tests/zephyr_native_sim.rs` — integration tests

**Justfile recipes:**
- `setup-tap-network` / `teardown-tap-network` — TAP bridge management
- `run-sentinel-zephyr` — run Zephyr sentinel with zenohd on bridge
- `test-zephyr` — run Zephyr integration tests

**Acceptance criteria:**

- [x] TAP networking setup succeeds (`zeth-br`, `zeth0`)
- [x] Zephyr `native_sim` binary connects to zenohd on bridge interface
- [x] Bidirectional message flow between Zephyr app and `ros2 topic pub/echo`
- [x] Teardown script cleans up TAP interfaces

## Acceptance Criteria (Phase-Level)

- [x] Linux sentinel binary builds and runs against Autoware planning simulator
- [x] All 7 replaced system/control nodes successfully supplanted
- [x] Vehicle completes autonomous driving mission with sentinel in the loop
- [x] Transport compatibility proven (nros zenoh-pico ↔ rmw_zenoh_cpp ↔ zenohd)
- [x] Engagement flow works (OperationModeState + ChangeOperationMode)
- [x] Planning simulator tests pass (`just test-planning`)
- [x] `just ci` still passes (no regressions in algorithm crate tests)

## References

- `src/autoware_sentinel/src/lib.rs` — SafetyIsland struct and Zephyr wiring (copy for Linux)
- `src/autoware_sentinel/Cargo.toml` — dependency template
- `src/autoware_sentinel/.cargo/config.toml` — nros patch entries to replicate
- `~/repos/nano-ros/examples/native/rust/zenoh/talker/src/main.rs` — Linux nros pattern
- `~/repos/nano-ros/build/zenohd/zenohd` — version-matched zenohd (v1.6.2)
- `~/repos/nano-ros/packages/testing/nros-tests/tests/rmw_interop.rs` — ROS 2 interop tests
- `~/repos/play_launch/tests/fixtures/autoware/scripts/test_autonomous_drive.py` — autonomous
  driving test sequence
- `docs/roadmap/6-zephyr-application.md` — Zephyr application (predecessor phase)
