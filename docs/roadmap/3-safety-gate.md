# Phase 3: Safety Gate

**Status:** Complete
**Depends on:** Phase 1 (messages), Phase 2 (MRM chain for emergency source)
**Goal:** Port the vehicle command gate to the safety island so that every command reaching
the vehicle actuators passes through an independent rate limiter and source arbiter.

## Description

The `autoware_vehicle_cmd_gate` is the single most safety-critical software component in
Autoware. It sits between all control sources (autonomous, remote, emergency) and the
vehicle actuators. It enforces rate limits on steering, acceleration, and jerk, and
arbitrates which source gets control.

This is the largest port in the project (3,215 LOC) but the core algorithms are
straightforward arithmetic — clamping, rate limiting, timeout checks. The complexity is
in interface wiring (10+ subscribers, multiple command sources). The port is split into
three incremental milestones.

After this phase, no software failure on the main computer can send unchecked commands to
the vehicle hardware.

## Architecture

Same two-layer pattern as Phase 2:

1. **Algorithm library** — Pure `#![no_std]` rate-limiting, clamping, and source arbitration
   logic. Operates on message types, no ROS dependencies. The gate's core is a struct that
   accepts control commands and current vehicle state, returns filtered commands.
2. **Node wiring** — Uses nros `Executor` with multiple subscription callbacks feeding the
   gate algorithm. The gate has 10+ topics, so the `Executor` needs sufficient callback
   capacity (e.g., `Executor::<_, 16, 16384>`).

```rust
// Example wiring sketch
executor.add_subscription::<Control, _>("input/auto/control_cmd", |msg| {
    gate.set_autonomous_cmd(msg);
})?;
executor.add_subscription::<Control, _>("input/emergency/control_cmd", |msg| {
    gate.set_emergency_cmd(msg);
})?;
executor.add_timer(33, move || {
    let filtered = gate.filter();
    control_pub.publish(&filtered).ok();
})?;
```

## Work Items

### 3.1 — Core rate limiting and command clamping

- [x] Algorithm library
- [x] Unit tests

**Source:** `autoware-repo/src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/`
**Target:** `src/autoware_vehicle_cmd_gate/`

Implement the filter that clamps all control commands to safe ranges, interpolated by
current vehicle speed.

Reference speed points: `[0.1, 0.3, 20.0, 30.0]` m/s

| Limit                     | Values by speed       | Unit  |
|---------------------------|-----------------------|-------|
| Velocity                  | 25.0                  | m/s   |
| Longitudinal acceleration | [5.0, 5.0, 5.0, 4.0]  | m/s²  |
| Longitudinal jerk         | [80.0, 5.0, 5.0, 4.0] | m/s³  |
| Steering angle            | [1.0, 1.0, 1.0, 0.8]  | rad   |
| Steering rate             | [1.0, 1.0, 1.0, 0.8]  | rad/s |
| Lateral acceleration      | [5.0, 5.0, 5.0, 4.0]  | m/s²  |
| Lateral jerk              | [7.0, 7.0, 7.0, 6.0]  | m/s³  |

Special accelerations: `stop_hold = -1.5 m/s²`, `emergency = -2.4 m/s²`.

Core interfaces for this milestone:

| Interface | Direction | Type |
|-----------|-----------|------|
| `input/auto/control_cmd` | Sub | `autoware_control_msgs/Control` |
| `/localization/kinematic_state` | Sub | `nav_msgs/Odometry` |
| `input/steering` | Sub | `autoware_vehicle_msgs/SteeringReport` |
| `output/control_cmd` | Pub | `autoware_control_msgs/Control` |

### 3.2 — Source arbitration

- [x] Algorithm library
- [x] Unit tests

Add support for multiple command sources with priority-based selection:
1. **Emergency** (highest) — from MRM operators on the island
2. **Remote** — from remote operator console
3. **Autonomous** — from main Autoware stack

| Interface | Direction | Type |
|-----------|-----------|------|
| `input/auto/control_cmd` | Sub | `autoware_control_msgs/Control` |
| `input/external/control_cmd` | Sub | `autoware_control_msgs/Control` |
| `input/emergency/control_cmd` | Sub | `autoware_control_msgs/Control` |
| `input/operation_mode` | Sub | `autoware_adapi_v1_msgs/OperationModeState` |
| `input/mrm_state` | Sub | `autoware_adapi_v1_msgs/MrmState` |
| `output/gear_cmd` | Pub | `autoware_vehicle_msgs/GearCommand` |
| `output/turn_indicators_cmd` | Pub | `autoware_vehicle_msgs/TurnIndicatorsCommand` |
| `output/hazard_lights_cmd` | Pub | `autoware_vehicle_msgs/HazardLightsCommand` |

### 3.3 — Heartbeat monitoring and diagnostics

- [x] Algorithm library
- [x] Unit tests

Add heartbeat monitoring for each command source. If the autonomous source stops sending
commands for a configurable timeout, the gate publishes a diagnostic error and the MRM
handler can take over.

| Interface | Direction | Type |
|-----------|-----------|------|
| `input/external_emergency_stop_heartbeat` | Sub | `autoware_adapi_v1_msgs/ManualOperatorHeartbeat` |
| `~/engage` | Srv | `tier4_external_api_msgs/Engage` |

Publish diagnostic status summarizing gate health, active source, and limit violations.

## Acceptance Criteria

- [x] Rate limiter clamps acceleration exceeding limits at each reference speed. Unit tests
      verify clamping at 0.1, 5.0, 20.0, and 30.0 m/s.
- [x] Jerk limiting produces smooth transitions (no discontinuities). Verify with a
      step-input test: command jumps from 0 to max acceleration, output ramps at max jerk.
- [x] Steering rate limiting prevents instantaneous steering jumps.
- [x] Speed-interpolated limits produce correct intermediate values (linear interpolation
      between reference speed points).
- [x] Source arbitration selects emergency commands over autonomous commands when MRM is
      active.
- [x] Source arbitration selects remote commands over autonomous commands when in remote
      mode.
- [x] Heartbeat timeout for autonomous source triggers diagnostic error within configured
      timeout.
- [x] Gate passes through commands unchanged when within all limits.
- [x] All crates compile `no_std` for `thumbv7em-none-eabihf`.
- [x] Integration test: MRM emergency stop command bypasses autonomous commands through
      the gate and reaches the output topic.
