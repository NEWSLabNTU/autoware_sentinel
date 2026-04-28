# Phase 2: Emergency Response

**Status:** Complete (algorithm libraries)
**Depends on:** Phase 1 (message crates)
**Goal:** Build the MRM (Minimum Risk Maneuver) chain on the safety island so it can
autonomously bring the vehicle to a safe stop with zero dependency on the main compute.

## Description

This phase ports the three components that form Autoware's emergency response pipeline:
the emergency stop operator (hard braking), the comfortable stop operator (gentle
deceleration), and the MRM handler that orchestrates between them. A heartbeat watchdog
is added to detect main compute failure.

After this phase, the safety island can:
1. Receive a heartbeat from the main Autoware stack via zenoh.
2. On heartbeat timeout (500 ms), trigger the MRM handler.
3. MRM handler activates comfortable stop first, then escalates to emergency stop.
4. Emergency stop operator publishes jerk-limited brake commands until velocity reaches zero.

This is the core value proposition of the safety island.

## Architecture

Each component is split into two layers:

1. **Algorithm library** (`src/autoware_*/src/lib.rs`) — Pure `#![no_std]` logic operating
   on message types. No ROS dependencies. Testable in isolation.
2. **Node binary** (TBD, e.g. `src/bin/safety_island.rs`) — Uses the nros high-level API
   (`Executor`, `Node`) to wire pub/sub/service callbacks to algorithm instances.

### nros wiring pattern

```rust
use nros::prelude::*;

let config = ExecutorConfig::from_env().node_name("mrm_handler");
let mut executor = Executor::<_, 8, 8192>::open(&config)?;
let mut node = executor.create_node("mrm_handler")?;

// Publishers
let state_pub = node.create_publisher::<MrmState>("/mrm/state")?;

// Service clients
let mut estop_client = node.create_client::<OperateMrm>("/mrm/emergency_stop/operate")?;

// Subscription callbacks registered on executor
executor.add_subscription::<Odometry, _>("/localization/kinematic_state", move |msg| {
    // update handler state, publish via state_pub
})?;

// Timer for periodic checks
executor.add_timer(33, move || { /* 30 Hz control loop */ })?;

executor.spin_blocking(SpinOptions::default())?;
```

## Work Items

### 2.1 — Port `autoware_mrm_emergency_stop_operator`

- [x] Algorithm library
- [x] Unit tests (7 tests)

**Source:** `autoware-repo/src/universe/autoware_universe/system/autoware_mrm_emergency_stop_operator/`
**Target:** `src/autoware_mrm_emergency_stop_operator/`

Service-triggered emergency braking with jerk-limited deceleration ramp.

| Interface                                 | Direction | Type                                  |
|-------------------------------------------|-----------|---------------------------------------|
| `~/input/mrm/emergency_stop/operate`      | Srv       | `tier4_system_msgs/OperateMrm`        |
| `~/input/control/control_cmd`             | Sub       | `autoware_control_msgs/Control`       |
| `~/output/mrm/emergency_stop/status`      | Pub       | `tier4_system_msgs/MrmBehaviorStatus` |
| `~/output/mrm/emergency_stop/control_cmd` | Pub       | `autoware_control_msgs/Control`       |

Algorithm (30 Hz):
```
a(t+1) = max(a(t) + target_jerk * dt, target_acceleration)
v(t+1) = max(v(t) + a(t) * dt, 0.0)
```

Parameters: `target_acceleration = -2.5 m/s²`, `target_jerk = -1.5 m/s³`.

### 2.2 — Port `autoware_mrm_comfortable_stop_operator`

- [x] Algorithm library
- [x] Unit tests (5 tests)

**Source:** `autoware-repo/src/universe/autoware_universe/system/autoware_mrm_comfortable_stop_operator/`
**Target:** `src/autoware_mrm_comfortable_stop_operator/`

Service-triggered gentle deceleration. On the safety island this publishes a control
command with ramped-down velocity rather than a velocity limit (no planner on the island).

| Interface                                   | Direction | Type                                  |
|---------------------------------------------|-----------|---------------------------------------|
| `~/input/mrm/comfortable_stop/operate`      | Srv       | `tier4_system_msgs/OperateMrm`        |
| `~/output/mrm/comfortable_stop/status`      | Pub       | `tier4_system_msgs/MrmBehaviorStatus` |
| `~/output/mrm/comfortable_stop/control_cmd` | Pub       | `autoware_control_msgs/Control`       |

Parameters: `min_acceleration = -1.0 m/s²`, `max_jerk = 0.3 m/s³`, `min_jerk = -0.3 m/s³`.

**Note:** The upstream Autoware version publishes a `VelocityLimit` to the planner. Since
the safety island has no planner, we adapt this to publish direct `Control` commands using
the same deceleration profile as the emergency stop operator but with gentler parameters.

### 2.3 — Port `autoware_mrm_handler`

- [x] Algorithm library (state machine + behavior selection)
- [x] Unit tests (14 tests)

**Source:** `autoware-repo/src/universe/autoware_universe/system/autoware_mrm_handler/`
**Target:** `src/autoware_mrm_handler/`

MRM orchestrator. Receives system health signals, decides which MRM behavior to activate,
and calls the corresponding operator service.

| Interface                               | Direction | Type                                          |
|-----------------------------------------|-----------|-----------------------------------------------|
| `~/input/operation_mode_availability`   | Sub       | `tier4_system_msgs/OperationModeAvailability` |
| `/localization/kinematic_state`         | Sub       | `nav_msgs/Odometry`                           |
| `~/output/mrm/state`                    | Pub       | `autoware_adapi_v1_msgs/MrmState`             |
| `~/output/hazard`                       | Pub       | `autoware_vehicle_msgs/HazardLightsCommand`   |
| `~/output/gear`                         | Pub       | `autoware_vehicle_msgs/GearCommand`           |
| `~/output/mrm/emergency_stop/operate`   | Cli       | `tier4_system_msgs/OperateMrm`                |
| `~/output/mrm/comfortable_stop/operate` | Cli       | `tier4_system_msgs/OperateMrm`                |

State machine:
```
NORMAL ──[emergency detected]──→ MRM_OPERATING
  ↑                                    │
  │                              ┌─────┴─────┐
  │                              ▼           ▼
  └──[recovered]──── MRM_SUCCEEDED    MRM_FAILED
```

Behavior selection (priority):
1. Watchdog timeout → EMERGENCY_STOP
2. Comfortable stop available → COMFORTABLE_STOP
3. Else → EMERGENCY_STOP
4. On any service call failure → escalate to EMERGENCY_STOP

nros provides service client support via `node.create_client::<OperateMrm>(topic)` with
`Promise`-based response handling.

### 2.4 — Implement heartbeat watchdog

- [x] Algorithm library
- [x] Unit tests (6 tests)

**Target:** `src/autoware_heartbeat_watchdog/`

Timer-based watchdog that monitors the main Autoware stack's heartbeat topic. On timeout,
publishes an `OperationModeAvailability` message with all modes set to `false`, which
triggers the MRM handler.

| Interface                              | Direction | Type                                          |
|----------------------------------------|-----------|-----------------------------------------------|
| `/autoware/heartbeat`                  | Sub       | `autoware_adapi_v1_msgs/Heartbeat`            |
| `~/output/operation_mode_availability` | Pub       | `tier4_system_msgs/OperationModeAvailability` |

Parameters: `timeout = 500 ms`.

Use `executor.add_timer(500, ...)` for periodic timeout checks.

### 2.5 — Integration binary

- [ ] Single binary that wires all Phase 2 components on one `Executor`

Create `src/bin/safety_island.rs` (or similar) that instantiates all components on a
single `Executor`, wiring their pub/sub/service interfaces together. This binary depends
on `nros` with `features = ["std", "rmw-zenoh", "platform-posix"]` for native testing.

## Acceptance Criteria

- [x] Emergency stop operator decelerates from 20 m/s to 0 m/s following the jerk-limited
      ramp. Unit test verifies velocity profile over time.
- [x] Comfortable stop operator decelerates with gentler profile (-1.0 m/s² vs -2.5 m/s²).
- [x] MRM handler transitions through NORMAL → MRM_OPERATING → MRM_SUCCEEDED when velocity
      reaches zero. Unit test covers the full state machine.
- [x] MRM handler escalates from COMFORTABLE_STOP to EMERGENCY_STOP when comfortable stop
      becomes unavailable.
- [x] Heartbeat watchdog triggers within 500 ms of last heartbeat. Unit test with simulated
      time verifies timeout detection.
- [ ] End-to-end test: stop heartbeat → watchdog fires → MRM handler activates →
      emergency stop operator publishes brake commands → velocity reaches zero.
- [x] All algorithm crates compile `no_std` for `thumbv7em-none-eabihf`.
- [x] Hazard lights are enabled and gear is set to PARK when MRM succeeds.

## Verification

Formal verification work items for Phase 2. Uses both Kani (bounded model checking) and Verus
(deductive proofs via Z3). Verus proofs operate on integer ghost models to avoid floating-point
arithmetic, which Verus cannot reason about. See [Phase 5](5-verification.md) for the
full verification strategy and ghost type bridge architecture.

### Emergency stop convergence proof (Verus)

Proves that the jerk-limited deceleration ramp converges velocity to zero in bounded time.

**Ghost model:** All values scaled by 1000 (milli-units) for integer arithmetic.

```rust
pub struct DecelState {
    pub velocity_mms: int,       // mm/s (non-negative)
    pub acceleration_mms2: int,  // mm/s^2 (negative during braking)
}
```

**Spec function `decel_step`:** Models one 33 ms iteration. Applies jerk to acceleration
(clamped to `target_accel = -2500`), then applies acceleration to velocity (clamped to 0).

**Two-phase ranking function:**
1. Acceleration ramps from 0 to target (ranking: `acceleration - target_accel`, decreasing by
   `jerk * dt` per step). ~51 steps (~1.7 s).
2. Velocity decreases at target rate (ranking: `velocity`, decreasing by 82 mm/s per step).
   From 20 m/s: ~244 steps (~8.1 s).

**Bounded convergence:** From 20 m/s to 0 in at most 300 steps (~10 seconds).

**Companion Kani harness:** `#[kani::unwind(301)]` bounded check on the actual f32
implementation, verifying `velocity == 0.0` within 300 iterations.

### MRM handler state machine proof (Verus)

Proves three properties of the MRM orchestrator state machine:

| Property                 | Description                                                                            |
|--------------------------|----------------------------------------------------------------------------------------|
| Terminal state stability | Once in `Succeeded` or `Failed`, state never changes                                   |
| Escalation monotonicity  | Behavior only escalates (`None → ComfortableStop → EmergencyStop`), never de-escalates |
| Succeeded requires v=0   | Cannot declare `MRM_SUCCEEDED` while vehicle is moving                                 |

**Ghost model:**

```rust
pub enum MrmState { Normal, Operating, Succeeded, Failed }
pub enum MrmBehavior { None, ComfortableStop, EmergencyStop }
pub struct GhostMrmHandler {
    pub state: MrmState,
    pub current_behavior: MrmBehavior,
    pub velocity_is_zero: bool,
}
```

Escalation monotonicity uses a ranking function: `behavior_ord(None) = 0`,
`behavior_ord(ComfortableStop) = 1`, `behavior_ord(EmergencyStop) = 2`. The proof obligation
is `behavior_ord(post) >= behavior_ord(pre)` for any transition while `state == Operating`.

### Heartbeat watchdog (Kani)

| Harness                   | Property                                      |
|---------------------------|-----------------------------------------------|
| `timeout_fires_correctly` | `elapsed >= timeout` iff watchdog fires       |
| `no_false_positives`      | Watchdog does not fire before timeout elapses |

### Verification crate additions

Phase 2 adds these modules to `src/verification/`:

```
src/verification/src/
  emergency_stop.rs    # DecelState ghost model + convergence proof
  mrm_handler.rs       # GhostMrmHandler + state machine proofs
```
