# Phase 4: Validation Layer

**Status:** Complete
**Depends on:** Phase 1 (messages), Phase 2 (MRM chain)
**Goal:** Add independent validation on the safety island to detect when the main stack's
controller is sending dangerous commands, and provide acceleration estimation for MRM
deceleration profiling.

## Description

This phase adds a cross-checking layer. The control validator detects trajectory deviations
and unsafe commands. The operation mode transition manager prevents engaging autonomy with
degraded sensors. The twist-to-acceleration converter provides the MRM operators with
current deceleration rate feedback.

These components run independently from the main stack's own validators. If the main stack
is compromised or has a software bug that its own validator fails to catch, the safety
island's validator can still trigger MRM.

## Work Items

### 4.1 — Port `autoware_control_validator`

- [x] Algorithm library
- [x] Unit tests (15 tests)

**Source:** `autoware-repo/src/universe/autoware_universe/control/autoware_control_validator/`
**Target:** `src/autoware_control_validator/`

Validates control commands against safety constraints and publishes diagnostic status.

| Check                | Threshold                   | Level      |
|----------------------|-----------------------------|------------|
| Trajectory deviation | 1.0 m max                   | ERROR      |
| Lateral jerk         | 10.0 m/s³                   | ERROR      |
| Acceleration error   | offset 0.8 m/s², scale 20%  | ERROR      |
| Rolling back         | 0.5 m/s opposite direction  | ERROR      |
| Overspeed            | 20% + 2.0 m/s over target   | ERROR      |
| Overrun stop point   | 0.8 m past stop line        | ERROR      |
| Yaw deviation        | 0.5 rad warn, 1.0 rad error | WARN/ERROR |

Key interfaces:

| Interface                                | Direction | Type                                |
|------------------------------------------|-----------|-------------------------------------|
| `/control/command/control_cmd`           | Sub       | `autoware_control_msgs/Control`     |
| `/localization/kinematic_state`          | Sub       | `nav_msgs/Odometry`                 |
| `/planning/scenario_planning/trajectory` | Sub       | `autoware_planning_msgs/Trajectory` |
| `~/output/validation_status`             | Pub       | diagnostic status                   |

**Scope note:** The upstream validator depends on trajectory data from the planner. For the
safety island, implement the subset of checks that can run without a full trajectory:
acceleration error, lateral jerk, rolling back, overspeed. Trajectory-dependent checks
(deviation, overrun) can be added later if a simplified trajectory is forwarded to the
island.

### 4.2 — Port `autoware_operation_mode_transition_manager`

- [x] Algorithm library
- [x] Unit tests (8 tests)

**Source:** `autoware-repo/src/universe/autoware_universe/system/autoware_operation_mode_transition_manager/`
**Target:** `src/autoware_operation_mode_transition_manager/`

Manages transitions between MANUAL, AUTONOMOUS, and REMOTE operation modes. Validates
preconditions before allowing mode changes.

| Interface                       | Direction | Type                                        |
|---------------------------------|-----------|---------------------------------------------|
| `~/input/control_mode`          | Sub       | `autoware_vehicle_msgs/ControlModeReport`   |
| `~/input/odometry`              | Sub       | `nav_msgs/Odometry`                         |
| `~/output/operation_mode/state` | Pub       | `autoware_adapi_v1_msgs/OperationModeState` |
| `~/srv/change_operation_mode`   | Srv       | mode change service                         |

Transition guards:
- Vehicle must be stopped to engage autonomous mode.
- All system health checks must pass.
- No active MRM.

### 4.3 — Port `autoware_twist2accel`

- [x] Algorithm library
- [x] Unit tests (8 tests)

**Source:** `autoware-repo/src/core/autoware_core/localization/autoware_twist2accel/`
**Target:** `src/autoware_twist2accel/`

Numerical differentiation of twist (velocity) to produce acceleration, with 1st-order
lowpass filtering.

| Interface                       | Direction | Type                                       |
|---------------------------------|-----------|--------------------------------------------|
| `/localization/kinematic_state` | Sub       | `nav_msgs/Odometry`                        |
| `~/output/accel`                | Pub       | `geometry_msgs/AccelWithCovarianceStamped` |

Core algorithm:
```
accel = (twist_now - twist_prev) / dt
filtered = prev_filtered + gain * (accel - prev_filtered)
```

Provides acceleration estimation to MRM comfortable stop operator for knowing current
deceleration rate.

## Acceptance Criteria

- [x] Control validator detects acceleration exceeding threshold and publishes ERROR
      diagnostic. Unit test with a command at 1.5x the acceleration limit.
- [x] Control validator detects rolling back (negative velocity when gear is DRIVE) and
      publishes ERROR diagnostic.
- [x] Control validator detects overspeed and publishes ERROR diagnostic.
- [x] Operation mode manager refuses MANUAL → AUTONOMOUS transition when vehicle is moving.
      Unit test verifies rejection.
- [x] Operation mode manager allows transition when all preconditions are met.
- [x] twist2accel produces correct acceleration from a known velocity ramp. Unit test:
      feed linear velocity increasing at 1.0 m/s² for 5 seconds, verify output converges
      to 1.0 m/s² within lowpass settling time.
- [x] twist2accel lowpass filter attenuates noise. Unit test: feed noisy velocity
      (1.0 m/s ± 0.5 m/s random), verify filtered acceleration variance is reduced.
- [x] All crates compile `no_std` for `thumbv7em-none-eabihf`.
