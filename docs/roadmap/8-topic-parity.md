# Phase 8: Topic Parity

**Status:** Complete (8.1a–g, 8.2a–d, 8.3a–c)
**Depends on:** Phase 7 (integration testing, 7.1–7.3)
**Goal:** The sentinel must publish every topic that the 7 replaced Autoware nodes publish,
so `ros2 topic list` on modified Autoware + sentinel matches baseline Autoware exactly.

## Description

Phase 7.3 proved the sentinel can drive a simulated vehicle, but a topic diff between
baseline Autoware (555 topics) and modified Autoware + sentinel (531 topics) reveals
24 missing topics. These fall into three categories:

1. **Functional topics** (14) — consumed by other running Autoware nodes. Missing these
   may cause degraded behavior, incomplete diagnostics, or failed mode transitions.
2. **Debug/diagnostic topics** (10) — consumed only by RViz, logging, or monitoring.
   Missing these breaks the observability and debugging experience but doesn't affect
   autonomous driving.
3. **Visibility** — zenoh-pico publishers don't emit ROS 2 liveliness tokens, so sentinel
   topics don't appear in `ros2 topic list` even though data is flowing. This affects all
   sentinel topics that lack an Autoware subscriber using rmw_zenoh_cpp.

All categories must be covered to achieve full behavioral parity with baseline Autoware.

## Gap Analysis

### Sentinel currently publishes (6 topics)

| Topic                                  | Message Type            | Source Node                       |
|----------------------------------------|-------------------------|-----------------------------------|
| `/control/command/control_cmd`         | `Control`               | vehicle_cmd_gate                  |
| `/control/command/gear_cmd`            | `GearCommand`           | shift_decider + vehicle_cmd_gate  |
| `/control/command/hazard_lights_cmd`   | `HazardLightsCommand`   | mrm_handler + vehicle_cmd_gate    |
| `/control/command/turn_indicators_cmd` | `TurnIndicatorsCommand` | mrm_handler + vehicle_cmd_gate    |
| `/system/fail_safe/mrm_state`          | `MrmState`              | mrm_handler                       |
| `/api/operation_mode/state`            | `OperationModeState`    | operation_mode_transition_manager |

### Missing functional topics (14)

These are subscribed by other running Autoware nodes in the planning simulator.

| #  | Topic                                      | Message Type              | Original Node                              | Subscriber(s)                                         |
|----|--------------------------------------------|---------------------------|--------------------------------------------|-------------------------------------------------------|
| 1  | `/api/autoware/get/engage`                 | `Engage`                  | vehicle_cmd_gate                           | simple_planning_simulator, external APIs              |
| 2  | `/autoware/engage`                         | `Engage`                  | operation_mode_transition_manager (compat) | legacy downstream nodes                               |
| 3  | `/control/command/emergency_cmd`           | `VehicleEmergencyStamped` | vehicle_cmd_gate                           | vehicle interface                                     |
| 4  | `/control/gate_mode_cmd`                   | `GateMode`                | vehicle_cmd_gate                           | — (published for external tools)                      |
| 5  | `/control/shift_decider/gear_cmd`          | `GearCommand`             | shift_decider                              | vehicle_cmd_gate (auto/external/emergency gear input) |
| 6  | `/control/vehicle_cmd_gate/is_stopped`     | `IsStopped`               | vehicle_cmd_gate (ModerateStopInterface)   | stop management                                       |
| 7  | `/control/vehicle_cmd_gate/operation_mode` | `OperationModeState`      | vehicle_cmd_gate                           | operation_mode_transition_manager (gate feedback)     |
| 8  | `/system/emergency/gear_cmd`               | `GearCommand`             | mrm_handler                                | vehicle_cmd_gate (emergency gear)                     |
| 9  | `/system/emergency/hazard_lights_cmd`      | `HazardLightsCommand`     | mrm_handler                                | vehicle_cmd_gate (emergency hazard)                   |
| 10 | `/system/emergency/turn_indicators_cmd`    | `TurnIndicatorsCommand`   | mrm_handler                                | vehicle_cmd_gate (emergency turn)                     |
| 11 | `/system/mrm/comfortable_stop/status`      | `MrmBehaviorStatus`       | mrm_comfortable_stop_operator              | mrm_handler                                           |
| 12 | `/system/mrm/emergency_stop/status`        | `MrmBehaviorStatus`       | mrm_emergency_stop_operator                | mrm_handler                                           |
| 13 | `/system/mrm/pull_over_manager/status`     | `MrmBehaviorStatus`       | mrm_pull_over_manager                      | mrm_handler                                           |
| 14 | `/api/autoware/get/emergency`              | `Emergency`               | vehicle_cmd_gate (EmergencyInterface)      | autoware_iv_external_api_adaptor                      |

**Note on self-loops:** Topics 5 and 8–12 are published by removed nodes and consumed by
other removed nodes (e.g. shift_decider → vehicle_cmd_gate, mrm operators → mrm_handler).
Since the sentinel replaces *all* of these, the self-loop topics don't need real data flow —
they only need to exist so `ros2 topic list` matches. The sentinel's internal algorithm wiring
already handles the equivalent data flow.

### Missing debug/diagnostic topics (10)

These are consumed only by RViz, logging, or monitoring tools.

| # | Topic | Message Type | Original Node |
|---|-------|-------------|---------------|
| 15 | `/control/autoware_operation_mode_transition_manager/debug_info` | `DebugInfo` (custom) | operation_mode_transition_manager |
| 16 | `/control/command/control_cmd/debug/published_time` | `PublishedTime` | vehicle_cmd_gate |
| 17 | `/control/control_validator/debug/marker` | `MarkerArray` | control_validator |
| 18 | `/control/control_validator/output/markers` | `MarkerArray` | control_validator |
| 19 | `/control/control_validator/validation_status` | `ControlValidatorStatus` | control_validator |
| 20 | `/control/control_validator/virtual_wall` | `MarkerArray` | control_validator |
| 21 | `/control/vehicle_cmd_gate/is_filter_activated` | `IsFilterActivated` | vehicle_cmd_gate |
| 22 | `/control/vehicle_cmd_gate/is_filter_activated/flag` | `BoolStamped` | vehicle_cmd_gate |
| 23 | `/control/vehicle_cmd_gate/is_filter_activated/marker` | `MarkerArray` | vehicle_cmd_gate |
| 24 | `/control/vehicle_cmd_gate/is_filter_activated/marker_raw` | `MarkerArray` | vehicle_cmd_gate |

## Work Items

### 8.1 — Functional topic publishers

Add publishers for the 14 missing functional topics to the sentinel Linux binary.

**Implementation approach:** Most of these topics carry data that the sentinel already
computes internally. The work is wiring new publishers, not new algorithms.

- [x] 8.1a — MRM behavior status publishers (topics 11–13)
  - Publish `MrmBehaviorStatus` on `/system/mrm/emergency_stop/status`
  - Publish `MrmBehaviorStatus` on `/system/mrm/comfortable_stop/status`
  - Publish `MrmBehaviorStatus` on `/system/mrm/pull_over_manager/status`
  - Status: AVAILABLE when not operating, OPERATING when active
  - **Requires:** `tier4_system_msgs` message generation (already available)

- [x] 8.1b — MRM handler emergency output topics (topics 8–10)
  - Publish `GearCommand` on `/system/emergency/gear_cmd`
  - Publish `HazardLightsCommand` on `/system/emergency/hazard_lights_cmd`
  - Publish `TurnIndicatorsCommand` on `/system/emergency/turn_indicators_cmd`
  - Data: same as MRM output already computed in 30 Hz timer

- [x] 8.1c — Vehicle command gate additional publishers (topics 3–7)
  - Publish `VehicleEmergencyStamped` on `/control/command/emergency_cmd`
  - Publish `GateMode` on `/control/gate_mode_cmd` (always AUTO)
  - Publish `GearCommand` on `/control/shift_decider/gear_cmd`
  - Publish `IsStopped` on `/control/vehicle_cmd_gate/is_stopped`
  - Publish `OperationModeState` on `/control/vehicle_cmd_gate/operation_mode`
  - **Requires:** `autoware_vehicle_msgs` VehicleEmergencyStamped, GateMode generation
  - **Requires:** `IsStopped` message — check which package defines it

- [x] 8.1d — Engagement topics (topics 1–2)
  - Publish `Engage` on `/api/autoware/get/engage` (engagement status)
  - Publish `Engage` on `/autoware/engage` (legacy compatibility)
  - **Requires:** `autoware_vehicle_msgs` Engage generation

- [x] 8.1e — Message generation for new types
  - Generate `VehicleEmergencyStamped` (if not already available)
  - Generate `GateMode` (if not already available)
  - Generate `Engage` (if not already available)
  - Generate `IsStopped` (check package: autoware_adapi_v1_msgs or tier4_system_msgs)
  - Update `package.xml` and re-run `just generate` in sentinel_linux

- [x] 8.1f — Integration test: verify all sentinel topics present
  - `test_sentinel_topic_parity` in `transport_smoke.rs`
  - Starts zenohd + sentinel, checks all 30 topics via `ros2 topic echo --once`
  - Runs in parallel for speed (~8s total)

- [x] 8.1g — Emergency API topic (topic 14)
  - Publish `Emergency` on `/api/autoware/get/emergency`
  - Message type: `tier4_external_api_msgs::msg::Emergency` (stamp + bool emergency)
  - Data: `emergency = true` when mrm_handler state is not NORMAL
  - **Note:** `tier4_external_api_msgs` is already generated as a transitive dependency;
    just needs adding as a direct dependency in `Cargo.toml`
  - Update `test_sentinel_topic_parity` to include the 30th topic

### 8.2 — Debug/diagnostic topic publishers

Add publishers for the 10 missing debug/diagnostic topics.

- [x] 8.2a — Control validator debug topics (topics 17–20)
  - Publish `MarkerArray` on `/control/control_validator/debug/marker`
  - Publish `MarkerArray` on `/control/control_validator/output/markers`
  - Publish `ControlValidatorStatus` on `/control/control_validator/validation_status`
  - Publish `MarkerArray` on `/control/control_validator/virtual_wall`
  - Populate validation_status from sentinel's ControlValidator state
  - MarkerArrays can be empty initially (RViz will show nothing, but topic exists)
  - **Requires:** `visualization_msgs` MarkerArray generation
  - **Requires:** `ControlValidatorStatus` message generation (check package)

- [x] 8.2b — Vehicle command gate debug topics (topics 21–24)
  - Publish `IsFilterActivated` on `/control/vehicle_cmd_gate/is_filter_activated`
  - Publish `BoolStamped` on `/control/vehicle_cmd_gate/is_filter_activated/flag`
  - Publish `MarkerArray` on `/control/vehicle_cmd_gate/is_filter_activated/marker`
  - Publish `MarkerArray` on `/control/vehicle_cmd_gate/is_filter_activated/marker_raw`
  - **Requires:** `IsFilterActivated` message generation (check package)
  - **Requires:** `BoolStamped` message generation (check package)

- [x] 8.2c — Remaining debug topics (topics 15–16)
  - Publish debug info on `/control/autoware_operation_mode_transition_manager/debug_info`
  - Publish `PublishedTime` on `/control/command/control_cmd/debug/published_time`
  - **Requires:** Identify message types for DebugInfo and PublishedTime

- [x] 8.2d — Integration test: full topic parity
  - Run baseline Autoware, capture `ros2 topic list`
  - Run modified Autoware + sentinel, capture `ros2 topic list`
  - Assert: diff is empty (exact topic parity)

### 8.3 — zenoh-pico liveliness tokens for `ros2 topic list` visibility

**Problem:** zenoh-pico publishers don't emit ROS 2 graph discovery liveliness tokens.
Sentinel topics ARE published and receivable (verified via `ros2 topic echo --once`),
but they don't appear in `ros2 topic list` output. The original 6 sentinel topics only
appear because other Autoware nodes subscribe to them via rmw_zenoh_cpp, which creates
liveliness entries on the subscriber side.

With 24 new topics that may not have Autoware subscribers, `ros2 topic list` will show
them as missing even though data is flowing correctly.

- [x] 8.3a — Investigate nano-ros liveliness token support
  - zenoh-pico fully supports liveliness declarations (`zpico_declare_liveliness()`)
  - rmw_zenoh_cpp expects `@ros2_lv/<domain>/<zid>/<version>/<kind>/%/<ns>/<node>/<topic>/<type>/<hash>/<qos>`
  - nano-ros `Ros2Liveliness::publisher_keyexpr()` already generates correct format
  - Problem: `declare_liveliness()` was never called — only logged for debugging

- [x] 8.3b — Implement liveliness token emission in nano-ros
  - Added `node_name` and `namespace` fields to `TopicInfo` (nros-rmw traits)
  - Propagated node name from `Node::create_publisher_with_qos` to `TopicInfo`
  - In `ZenohSession::create_publisher()`: call `declare_liveliness()` with keyexpr
  - Store `LivelinessToken` in `ZenohPublisher` (Drop undeclares — must keep alive)
  - Set `ZPICO_MAX_LIVELINESS=32` (default 16, need 30+ for all publishers)
  - **Upstream nano-ros changes** in 4 files (nros-rmw, nros-node, nros-rmw-zenoh)

- [x] 8.3c — Verify full `ros2 topic list` parity
  - Modified Autoware + sentinel: 555 topics (matches baseline exactly)
  - Diff: 0 missing, 0 extra — full topic parity achieved
  - All 30 sentinel topics visible in `ros2 topic list`

## Implementation Notes

### Priority

8.1 (functional) should be done first — these affect runtime behavior. 8.2 (debug) can
follow later and is lower priority since it only affects observability.

### Self-loop topics

Topics 5 and 8–13 form a closed loop among the 7 replaced nodes. The sentinel doesn't
need to actually route data through these topics — it only needs to advertise them so that
`ros2 topic list` shows them and any external monitoring tools see them. The sentinel's
internal algorithm pipeline already handles the equivalent computation.

### Message generation

Several new message types will need to be generated. Check which are already available in
the sentinel_linux `generated/` directory before generating new ones. Types likely needed:
- `autoware_vehicle_msgs/msg/VehicleEmergencyStamped`
- `autoware_vehicle_msgs/msg/GateMode`
- `autoware_vehicle_msgs/msg/Engage`
- `tier4_system_msgs/msg/MrmBehaviorStatus` (may already exist)
- `visualization_msgs/msg/MarkerArray`
- Various debug message types (IsFilterActivated, BoolStamped, etc.)

### Executor capacity

The sentinel uses `Executor::<_, 48, 16384>` (bumped from 16). Current usage:
30 publishers + 5 subscribers + 1 timer + 1 service = 37 slots, well within the 48 limit.

### ZPICO_MAX_PUBLISHERS

zenoh-pico defaults to 8 max publishers and 16 max liveliness tokens. The sentinel declares
liveliness tokens for all publishers, subscribers, and services (30 + 5 + 1 = 36 tokens).
Build with `ZPICO_MAX_PUBLISHERS=32 ZPICO_MAX_LIVELINESS=48` env vars (set in justfile and
test fixtures).

### no_std compatibility

New publishers must only use message types that compile under `#![no_std]` for the Zephyr
target. `visualization_msgs::MarkerArray` contains `String` fields — check if the generated
`heapless::String<N>` version works. If not, the debug topics (8.2) may need to be
Linux-only behind a `#[cfg(feature = "std")]` gate.

## Acceptance Criteria

- [x] `ros2 topic list` on modified Autoware + sentinel matches baseline exactly (0 diff)
- [x] All 14 functional topics (1–14) carry meaningful data (not just empty messages)
- [x] MrmBehaviorStatus reflects actual operator state (AVAILABLE/OPERATING)
- [x] Engage topics reflect actual engagement state
- [x] VehicleEmergencyStamped reflects actual emergency state
- [x] Debug topics exist (content can be minimal/empty initially)
- [x] No regressions in existing integration tests
- [x] `/api/autoware/get/emergency` publishes Emergency with correct state
- [x] Algorithm crates cross-compile cleanly for `thumbv7em-none-eabihf` (`just cross-check`)
- [x] `just launch-autoware-sentinel` shows 0 topic diff vs baseline

## References

- Phase 7 topic diff: 24 missing topics (baseline 555 vs modified+sentinel 531)
- Autoware source: `~/repos/autoware/1.5.0-ws/src/universe/autoware_universe/`
  - `control/autoware_vehicle_cmd_gate/` — 18 publishers total
  - `control/autoware_control_validator/` — 5 publishers
  - `control/autoware_operation_mode_transition_manager/` — 4 publishers
  - `control/autoware_shift_decider/` — 1 publisher
  - `system/autoware_mrm_handler/` — 4 publishers
  - `system/autoware_mrm_emergency_stop_operator/` — 2 publishers
  - `system/autoware_mrm_comfortable_stop_operator/` — 3 publishers
- Sentinel Linux binary: `src/autoware_sentinel_linux/src/main.rs`
