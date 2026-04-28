# Phase 12: Service & Topic Parity

**Status:** Complete
**Depends on:** Phase 8 (topic parity), Phase 7 (integration testing)
**Goal:** The sentinel must publish every topic and serve every service that the
filtered-out Autoware nodes provide, even if no remaining node currently consumes them.
This ensures full behavioral equivalence so Autoware can be upgraded or reconfigured
without silent regressions from missing interfaces.

## Background

Phase 8 achieved `ros2 topic list` parity (555 topics, 0 diff). However, a detailed
audit of filtered nodes reveals gaps in two categories:

1. **Missing publishers** — topics that filtered nodes publish but the sentinel does not.
   These topics have no remaining consumers today, but external tools or future Autoware
   nodes may depend on them.
2. **Missing services** — service servers that filtered nodes provide. External API clients
   (e.g. Autoware Web UI, fleet management) expect these services to exist.

## Gap Analysis

### Node-by-node audit

Each filtered node was audited against the sentinel's publishers and services.
Nodes with full coverage are listed for completeness.

#### `vehicle_cmd_gate` — 3 missing topics, 4 missing services

**Covered by sentinel (9 topics, 0 services):**
`/control/command/control_cmd`, `/control/command/gear_cmd`,
`/control/command/turn_indicators_cmd`, `/control/command/hazard_lights_cmd`,
`/control/command/emergency_cmd`, `/control/vehicle_cmd_gate/operation_mode`,
`/control/vehicle_cmd_gate/is_stopped`, `/api/autoware/get/engage`,
`/api/autoware/get/emergency`

**Missing topics:**

| # | Topic                                          | Message Type                              | Notes                                               |
|---|------------------------------------------------|-------------------------------------------|-----------------------------------------------------|
| 1 | `/control/vehicle_cmd_gate/is_paused`          | `tier4_control_msgs/msg/IsPaused`         | Consumed by `motion` ADAPI adaptor                  |
| 2 | `/control/vehicle_cmd_gate/is_start_requested` | `tier4_control_msgs/msg/IsStartRequested` | Consumed by `motion` ADAPI adaptor                  |
| 3 | `/control/current_gate_mode`                   | `tier4_control_msgs/msg/GateMode`         | No remaining consumer; published for external tools |

**Missing services:**

| # | Service                                                   | Service Type                               | Notes                           |
|---|-----------------------------------------------------------|--------------------------------------------|---------------------------------|
| 4 | `/api/autoware/set/engage`                                | `tier4_external_api_msgs/srv/Engage`       | Engage/disengage the vehicle    |
| 5 | `/api/autoware/set/emergency`                             | `tier4_external_api_msgs/srv/SetEmergency` | Set/clear external emergency    |
| 6 | `/control/vehicle_cmd_gate/external_emergency_stop`       | `std_srvs/srv/Trigger`                     | Trigger external emergency stop |
| 7 | `/control/vehicle_cmd_gate/clear_external_emergency_stop` | `std_srvs/srv/Trigger`                     | Clear external emergency stop   |

#### `shift_decider` — fully covered

Sentinel publishes `/control/shift_decider/gear_cmd`. No services.

#### `operation_mode_transition_manager` — 1 missing topic, 1 missing service

**Covered:** `/system/operation_mode/state` (published by sentinel)

**Missing:**

| # | Topic/Service                              | Type                                           | Notes                                             |
|---|--------------------------------------------|------------------------------------------------|---------------------------------------------------|
| 8 | `/control/is_autonomous_available` (topic) | `tier4_system_msgs/msg/ModeChangeAvailable`    | No remaining consumer                             |
| 9 | `/control/control_mode_request` (service)  | `autoware_vehicle_msgs/srv/ControlModeCommand` | Client to vehicle interface; sentinel can stub it |

#### `control_validator` — fully covered

Sentinel publishes `/control/control_validator/validation_status` and debug markers.
No services.

#### `operation_mode` ADAPI adaptor — 5 missing services

**Covered:** `/api/operation_mode/state` (topic),
`/api/operation_mode/change_to_autonomous` (service)

**Missing services:**

| #  | Service                                        | Service Type          | Notes                                                    |
|----|------------------------------------------------|-----------------------|----------------------------------------------------------|
| 10 | `/api/operation_mode/change_to_stop`           | `ChangeOperationMode` | All use `autoware_adapi_v1_msgs/srv/ChangeOperationMode` |
| 11 | `/api/operation_mode/change_to_local`          | `ChangeOperationMode` |                                                          |
| 12 | `/api/operation_mode/change_to_remote`         | `ChangeOperationMode` |                                                          |
| 13 | `/api/operation_mode/enable_autoware_control`  | `ChangeOperationMode` |                                                          |
| 14 | `/api/operation_mode/disable_autoware_control` | `ChangeOperationMode` |                                                          |

#### `autoware_state` ADAPI adaptor — fully covered

Sentinel publishes `/autoware/state`. The `shutdown` service is out of scope
(sentinel lifecycle is managed by the MCU, not Autoware).

#### `mrm_handler` — 1 missing topic

**Covered:** `/system/fail_safe/mrm_state`, `/system/emergency/gear_cmd`,
`/system/emergency/turn_indicators_cmd`, `/system/emergency/hazard_lights_cmd`

**Missing:**

| #  | Topic                       | Type                                          | Notes                                                                        |
|----|-----------------------------|-----------------------------------------------|------------------------------------------------------------------------------|
| 15 | `/system/emergency_holding` | `tier4_system_msgs/msg/EmergencyHoldingState` | Consumed by `hazard_status_converter` (also filtered); no remaining consumer |

#### `diagnostic_graph_aggregator` — not required

Both `aggregator_node` and `converter_node` are filtered, along with all their
consumers (`hazard_status_converter`, `diagnostics` ADAPI). The entire diagnostic
graph chain is self-contained and fully replaced by the sentinel's own health logic.

Topics `/diagnostics_graph/struct`, `/diagnostics_graph/status`,
`/system/operation_mode/availability`, `/system/command_mode/availability` have
no remaining consumers after filtering.

#### `hazard_status_converter` — not required

`/system/emergency/hazard_status` is consumed only by nodes that are also filtered.

#### `diagnostics` ADAPI adaptor — not required

`/api/system/diagnostics/struct` and `/api/system/diagnostics/status` are external
API endpoints. The sentinel replaces the diagnostic graph; these endpoints would
return misleading data. Exclude from scope.

#### `interface` ADAPI adaptor — not required

Only serves `/api/interface/version` (returns ADAPI version string). Out of scope.

### Summary

| Category         | Count                | Items                                                        |
|------------------|----------------------|--------------------------------------------------------------|
| Missing topics   | 5                    | #1–3 (gate), #8 (op_mode), #15 (mrm_handler)                 |
| Missing services | 9                    | #4–7 (gate), #9 (op_mode_transition), #10–14 (op_mode ADAPI) |
| Not required     | 8 topics, 2 services | diagnostic chain, hazard_status, ADAPI diagnostics/interface |

## Work Items

### 12.1 — Missing `vehicle_cmd_gate` topics (3 publishers)

- [x] 12.1a — Publish `IsPaused` on `/control/vehicle_cmd_gate/is_paused`
  - Value: `is_paused: false` (sentinel never pauses control output)
  - `tier4_control_msgs` already generated

- [x] 12.1b — Publish `IsStartRequested` on `/control/vehicle_cmd_gate/is_start_requested`
  - Value: `is_start_requested: false`

- [x] 12.1c — Publish `GateMode` on `/control/current_gate_mode`
  - Value: `data: AUTO` (sentinel always uses autonomous control)
  - Uses `tier4_control_msgs/msg/GateMode` (same type as `/control/gate_mode_cmd`)

### 12.2 — Missing `vehicle_cmd_gate` services (4 services)

- [x] 12.2a — Serve `/api/autoware/set/engage`
  - Type: `tier4_external_api_msgs/srv/Engage`
  - Behavior: Set `autonomous_engaged` flag (same effect as `change_to_autonomous`)
  - `tier4_external_api_msgs` already generated

- [x] 12.2b — Serve `/api/autoware/set/emergency`
  - Type: `tier4_external_api_msgs/srv/SetEmergency`
  - Behavior: Set/clear `external_emergency_stop` state; wired to gate + emergency publishers

- [x] 12.2c — Serve `/control/vehicle_cmd_gate/external_emergency_stop`
  - Type: `std_srvs/srv/Trigger`
  - Behavior: Set `external_emergency_stop = true`
  - Added `std_srvs` to `Cargo.toml`

- [x] 12.2d — Serve `/control/vehicle_cmd_gate/clear_external_emergency_stop`
  - Type: `std_srvs/srv/Trigger`
  - Behavior: Set `external_emergency_stop = false`

### 12.3 — Missing `operation_mode_transition_manager` topic and service

- [x] 12.3a — Publish `ModeChangeAvailable` on `/control/is_autonomous_available`
  - Value: `available: true` when `autonomous_engaged`
  - `tier4_system_msgs/msg/ModeChangeAvailable` already generated

- [x] 12.3b — Serve `/control/control_mode_request`
  - Type: `autoware_vehicle_msgs/srv/ControlModeCommand`
  - Behavior: Stub — always returns `success: true`
  - `autoware_vehicle_msgs` service already generated

### 12.4 — Missing `operation_mode` ADAPI services (5 services)

All use `autoware_adapi_v1_msgs/srv/ChangeOperationMode` (already generated).

- [x] 12.4a — Serve `/api/operation_mode/change_to_stop`
  - Behavior: Set `autonomous_engaged = false`, return success

- [x] 12.4b — Serve `/api/operation_mode/change_to_local`
  - Behavior: Return error (sentinel doesn't support local mode)

- [x] 12.4c — Serve `/api/operation_mode/change_to_remote`
  - Behavior: Return error (sentinel doesn't support remote mode)

- [x] 12.4d — Serve `/api/operation_mode/enable_autoware_control`
  - Behavior: No-op, return success (always in autoware control)

- [x] 12.4e — Serve `/api/operation_mode/disable_autoware_control`
  - Behavior: Return error (sentinel always controls)

### 12.5 — Missing `mrm_handler` topic

- [x] 12.5a — Publish `EmergencyHoldingState` on `/system/emergency_holding`
  - Value: `state: NONE` (sentinel doesn't use emergency holding)
  - **Requires:** `tier4_system_msgs/msg/EmergencyHoldingState` (check if available)

### 12.6 — Message generation

- [x] 12.6a — Generate `tier4_control_msgs` (IsPaused, IsStartRequested, GateMode)
  - `<depend>tier4_control_msgs</depend>` already in `package.xml`; crate generated

- [x] 12.6b — Generate `tier4_external_api_msgs` services (Engage, SetEmergency)
  - Available as transitive dependency; added explicit `<depend>tier4_external_api_msgs</depend>`
    to `package.xml` since it is a direct code dependency

- [x] 12.6c — Generate `autoware_vehicle_msgs` services (ControlModeCommand)
  - `<depend>autoware_vehicle_msgs</depend>` already in `package.xml`; `srv/control_mode_command.rs`
    generated

- [x] 12.6d — Verify `std_srvs/srv/Trigger` is available
  - `generated/std_srvs/src/srv/trigger.rs` present; added explicit `<depend>std_srvs</depend>`
    to `package.xml` since it is a direct code dependency

### 12.7 — Capacity updates

- [x] 12.7a — Update executor and zpico capacity limits
  - Actual counts: 37 pub + 9 sub + 1 timer + 11 svc = 58 callbacks; 57 liveliness tokens; 16 queryables (5 param + 11 svc)
  - `.env`: `ZPICO_MAX_PUBLISHERS=40`, `ZPICO_MAX_QUERYABLES=20`, `ZPICO_MAX_LIVELINESS=64`, `NROS_EXECUTOR_MAX_CBS=64`
  - `tests/src/fixtures/sentinel.rs` updated to match
  - Zephyr `prj.conf` unchanged (Zephyr sentinel is a separate application with fewer topics)

### 12.8 — Integration test

- [x] 12.8a — Extend `test_sentinel_topic_parity` with new topics
  - Add the 5 new publisher topics to the parity check

- [x] 12.8b — Service smoke test
  - Call each new service via `ros2 service call` and verify response
  - Verify `/api/autoware/set/engage` actually engages the sentinel
  - Verify `/api/operation_mode/change_to_stop` disengages

### 12.9 — ROS 2 parameter services

- [x] 12.9a — Serve all 6 parameter services (`list/get/get_types/describe/set/set_atomically`)
  - Used `executor.register_parameter_services("sentinel")` — the standard nano-ros API
  - 62 parameters declared via `params::declare_parameters(executor.params_mut().unwrap())`
  - Tests `test_sentinel_param_list` and `test_sentinel_param_get` pass

- [x] 12.9b — Fix `nros_rcl_interfaces` DDS type strings
  - Root cause: codegen `apply_package_renames` replaced `rcl_interfaces::` with
    `nros_rcl_interfaces::` inside string literals, corrupting `TYPE_NAME`/`SERVICE_NAME`
  - Fix 1: corrected all 34 generated files (humble + iron) — `nros_rcl_interfaces::` →
    `rcl_interfaces::` in every `TYPE_NAME`/`SERVICE_NAME` constant
  - Fix 2: `fix_rust_idents_recursive` now uses `replace_outside_strings` which skips
    content inside double-quoted string literals; future regeneration is safe
  - Fix 3: `nros_rcl_interfaces` types are now `pub(crate)` in `parameter_services.rs`;
    external code uses `executor.register_parameter_services()` only

- [x] 12.9c — Buffer sizing
  - `NROS_PARAM_SERVICE_BUFFER_SIZE=8192` in `.env` and `tests/src/fixtures/sentinel.rs`
  - Required because `describe_parameters` with 62 params produces ≈ 4.5 KiB CDR reply
    (exceeds the 4 KiB default)

## Implementation Notes

### Service count impact

Adding 9 services significantly increases the sentinel's resource usage.
Each service needs a queryable slot in zenoh-pico and a callback slot in the executor.
Current capacity (52 CBS, 8 queryables) will need increases.

### `no_std` compatibility

New message types (`IsPaused`, `IsStartRequested`, `EmergencyHoldingState`,
`ModeChangeAvailable`) must compile under `#![no_std]` for the Zephyr target.
Service types (`Engage`, `SetEmergency`, `Trigger`, `ControlModeCommand`) are
only needed in the node binary layer, not the algorithm library.

### Priority

1. **12.1 + 12.5** (5 publishers) — Low effort, completes topic coverage
2. **12.4** (5 operation_mode services) — Same service type, batch implementation
3. **12.2** (4 gate services) — Requires new message types
4. **12.3** (1 topic + 1 service) — Lowest priority, no remaining consumers

## Acceptance Criteria

- [x] All 5 missing topics published by sentinel with correct data
- [x] All 9 missing services respond correctly
- [x] `ros2 service call` succeeds for each new service
- [x] No regressions in existing integration tests
- [x] Capacity limits updated in `.env`, test fixtures, and Zephyr Kconfig
- [x] 6 parameter services respond correctly (`ros2 param list/get /sentinel`)
- [x] `ros2 service list` on sentinel includes all filtered services
  - Verified: all 10 Phase 12 services present in `ros2 service list` output (17 services total,
    including 6 parameter services on `/sentinel`)
  - Note: sentinel must be started without inheriting `ZENOH_SESSION_CONFIG_URI` /
    `ZENOH_ROUTER_CONFIG_URI` — those env vars confuse zenoh-pico's liveliness token setup
    and prevent service discovery. The `just launch-autoware-sentinel` recipe handles this correctly.
- [x] Algorithm crates cross-compile cleanly (`just cross-check`)
  - All algorithm libs build for `thumbv7em-none-eabihf` without errors

## Live Comparison: Baseline vs Sentinel (2026-03-25)

Autoware 1.5.0 planning simulator launched via `play_launch replay` through a shared
zenohd router. Discovery via `ros2 daemon` + `ros2 topic/service list` with rmw_zenoh_cpp.

### Summary

| Metric | Baseline | Sentinel | Delta |
|--------|----------|----------|-------|
| Topics | 571 | 558 | -13 |
| Services | 742 | 653 | -89 |
| Extra (sentinel-only) | — | 1 topic, 6 services | — |

The sentinel adds 1 topic (`/control/is_autonomous_available`) and 6 parameter services
(`/sentinel/{list,get,set,describe,get_types,set_atomically}_parameters`) not present in
the baseline.

### Phase 12 functional services: all present ✓

All 11 sentinel-provided services are present in the sentinel stack:
- `/api/autoware/set/engage`
- `/api/autoware/set/emergency`
- `/control/vehicle_cmd_gate/external_emergency_stop`
- `/control/vehicle_cmd_gate/clear_external_emergency_stop`
- `/control/control_mode_request`
- `/api/operation_mode/change_to_{stop,local,remote,autonomous}`
- `/api/operation_mode/{enable,disable}_autoware_control`

### Missing topics (14)

All from filtered-out system/diagnostic nodes. No consumer in the planning simulator
depends on these — they are monitoring/status topics.

| Topic                                                                                                                    | Source node                        |
|--------------------------------------------------------------------------------------------------------------------------|------------------------------------|
| `/api/system/diagnostics/status`                                                                                         | ADAPI diagnostics adaptor          |
| `/api/system/diagnostics/struct`                                                                                         | ADAPI diagnostics adaptor          |
| `/diagnostics_graph/unknowns`                                                                                            | diagnostic_graph_aggregator        |
| `/system/command_mode/availability`                                                                                      | ADAPI operation_mode adaptor       |
| `/system/component_state_monitor/component/launch/{control,localization,map,perception,planning,sensing,system,vehicle}` | component_state_monitor (8 topics) |
| `/system/emergency/hazard_status`                                                                                        | hazard_status_converter            |
| `/system/operation_mode/availability`                                                                                    | ADAPI operation_mode adaptor       |

### Missing services (95)

#### Parameter services of filtered-out nodes (84)

Each filtered ROS 2 node provides 6 parameter services. The sentinel replaces their
**functional** behavior but not their per-node parameter services. These are ROS 2
infrastructure services rarely called by external clients.

| Node                                         | Services                                                              | Count  |
|----------------------------------------------|-----------------------------------------------------------------------|--------|
| ADAPI `autoware_state` adaptor               | `/adapi/node/autoware_state/{6 param srvs}`                           | 6      |
| ADAPI `diagnostics` adaptor                  | `/adapi/node/diagnostics/{6 param srvs}`                              | 6      |
| ADAPI `interface` adaptor                    | `/adapi/node/interface/{6 param srvs}`                                | 6      |
| ADAPI `operation_mode` adaptor               | `/adapi/node/operation_mode/{6 param srvs}`                           | 6      |
| `autoware_operation_mode_transition_manager` | `/control/autoware_operation_mode_transition_manager/{6 param srvs}`  | 6      |
| `autoware_shift_decider`                     | `/control/autoware_shift_decider/{6 param srvs}`                      | 6      |
| `autoware_control_validator`                 | `/control/control_validator/{6 param srvs}`                           | 6      |
| `autoware_vehicle_cmd_gate`                  | `/control/vehicle_cmd_gate/{6 param srvs + config_logger + set_stop}` | 8      |
| `mrm_comfortable_stop_operator`              | `/system/mrm_comfortable_stop_operator/.../{6 param srvs}`            | 6      |
| `mrm_emergency_stop_operator`                | `/system/mrm_emergency_stop_operator/.../{6 param srvs}`              | 6      |
| `mrm_handler`                                | `/system/mrm_handler/{6 param srvs}`                                  | 6      |
| `aggregator` (diagnostic_graph)              | `/system/aggregator/{6 param srvs}`                                   | 6      |
| `converter` (diagnostic_graph)               | `/system/converter/{6 param srvs}`                                    | 6      |
| `hazard_status_converter`                    | `/system/hazard_status_converter/{6 param srvs}`                      | 6      |
| **Total**                                    |                                                                       | **84** |

#### Functional services of filtered-out nodes (11)

| Service                                          | Source node                     | Purpose                      |
|--------------------------------------------------|---------------------------------|------------------------------|
| `/api/interface/version`                         | ADAPI `interface` adaptor       | Returns ADAPI version string |
| `/api/system/diagnostics/reset`                  | ADAPI `diagnostics` adaptor     | Reset diagnostic state       |
| `/autoware/shutdown`                             | ADAPI `interface` adaptor       | Graceful shutdown            |
| `/control/vehicle_cmd_gate/config_logger`        | `vehicle_cmd_gate`              | Runtime log level control    |
| `/control/vehicle_cmd_gate/set_stop`             | `vehicle_cmd_gate`              | Force stop from external     |
| `/diagnostics_graph/reset`                       | `diagnostic_graph_aggregator`   | Reset diagnostic graph       |
| `/system/aggregator/set_initializing`            | `diagnostic_graph_aggregator`   | Set initializing state       |
| `/system/mrm/comfortable_stop/operate`           | `mrm_comfortable_stop_operator` | Trigger comfortable stop     |
| `/system/mrm/emergency_stop/operate`             | `mrm_emergency_stop_operator`   | Trigger emergency stop       |
| `/system/mrm/pull_over_manager/operate`          | `mrm_handler`                   | Trigger pull-over MRM        |
| `/system/operation_mode/change_autoware_control` | ADAPI `operation_mode` adaptor  | Legacy control mode toggle   |

### Assessment

The sentinel covers all services needed for the autonomous driving pipeline. Missing items
fall into two categories:

1. **Per-node parameter services** (84/95) — ROS 2 infrastructure. The sentinel consolidates
   all parameters under a single `/sentinel/*` parameter namespace. Individual per-node
   parameter services are not needed since the sentinel is a single process.

2. **System monitoring services** (11/95) and **monitoring topics** (14/14) — From diagnostic
   aggregator, hazard_status_converter, and component_state_monitor. These implement Autoware's
   health monitoring stack, which the safety island intentionally replaces with its own
   heartbeat-based approach.

## References

- Phase 8 topic parity: `docs/roadmap/8-topic-parity.md`
- Autoware ADAPI specs: `/opt/autoware/1.5.0/include/autoware/adapi_specs/`
- vehicle_cmd_gate source: `~/repos/autoware/1.5.0-ws/src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/`
- operation_mode_transition_manager: `~/repos/autoware/1.5.0-ws/src/universe/autoware_universe/control/autoware_operation_mode_transition_manager/`
- ADAPI operation_mode adaptor: `autoware_default_adapi_universe` package, `OperationModeNode`
- Sentinel Linux binary: `src/autoware_sentinel_linux/src/main.rs`
