# Phase 9: Behavioral Verification

**Status:** In progress (9.1a complete)
**Depends on:** Phase 8 (topic parity), Phase 7 (integration testing)
**Goal:** Verify that the sentinel's topic outputs match official Autoware behavior under
the same inputs, beyond just topic name parity.

## Description

Phase 8 achieved full `ros2 topic list` parity (555 topics, 0 diff). Phase 7.3 proved the
sentinel can drive a simulated vehicle. But neither phase verifies that the sentinel
produces the *same message content* as the original Autoware nodes under identical inputs.

Behavioral verification closes this gap by comparing outputs side-by-side: run baseline
Autoware, record topic data, replay the same inputs through the sentinel, and diff the
outputs. This proves functional equivalence, not just structural parity.

### What we need to verify

The sentinel replaces 7 Autoware nodes. Each has observable outputs that can be compared:

| Node                          | Key Outputs                            | Verification Challenge                   |
|-------------------------------|----------------------------------------|------------------------------------------|
| vehicle_velocity_converter    | TwistWithCovariance                    | Covariance diagonal values, scale factor |
| stop_filter                   | Odometry (zeroed when stopped)         | Threshold behavior, NaN handling         |
| shift_decider                 | GearCommand                            | State machine transitions, dead-zone     |
| mrm_emergency_stop_operator   | Control, MrmBehaviorStatus             | Jerk-limited deceleration profile        |
| mrm_comfortable_stop_operator | MrmBehaviorStatus                      | Gentle deceleration profile              |
| mrm_handler                   | MrmState, gear/hazard/turn commands    | State machine, escalation logic          |
| vehicle_cmd_gate              | Control, gear, hazard, turn, emergency | Source arbitration, rate limiting        |

Additionally, nodes added in Phase 4 that aren't separate Autoware nodes but produce
observable topics:

| Algorithm                         | Key Outputs                    | Verification Challenge     |
|-----------------------------------|--------------------------------|----------------------------|
| twist2accel                       | AccelWithCovariance (internal) | Lowpass filter convergence |
| control_validator                 | ControlValidatorStatus         | Threshold matching         |
| operation_mode_transition_manager | OperationModeState, debug      | Transition timing          |

### What differs by design

Some differences are intentional and acceptable:

- **Timer frequency:** Sentinel runs all algorithms at 30 Hz in a single loop. Autoware
  runs shift_decider at 10 Hz, mrm_handler at 10 Hz, etc. Message rates will differ.
- **Timestamps:** Different clocks and execution order. Timestamps will never match exactly.
- **QoS settings:** Sentinel uses default QoS. Autoware uses transient_local durability on
  some vehicle_cmd_gate topics. This affects late-joining subscribers but not message content.
- **Parameters:** Sentinel uses hardcoded defaults. Autoware loads from parameter files.
  Values must match but the mechanism differs.
- **Debug topics:** MarkerArray and debug topics publish empty/minimal content in the
  sentinel. These are cosmetic, not behavioral.

## Approach: Record-and-Compare

### Method

1. **Record baseline:** Run full baseline Autoware in planning simulator. Record all outputs
   from the 7 replaced nodes using `ros2 bag record`.
2. **Record sentinel:** Run modified Autoware + sentinel in the same scenario. Record the
   same topics.
3. **Compare:** Diff the two recordings, allowing for timestamp tolerance and known
   acceptable differences.

### Comparison criteria

For each topic, define what "matches" means:

- **Exact match:** Message fields are bit-identical (after timestamp normalization)
- **Approximate match:** Numerical fields within tolerance (e.g., f64 epsilon for float
  operations, small drift from different execution order)
- **Structural match:** Message has correct type and populated fields, but values may differ
  due to timing (e.g., MRM state transitions happen at slightly different times)
- **Existence match:** Topic is published at expected rate (debug topics)

## Subphases

### 9.1 — Baseline recording infrastructure

Build tooling to record and replay Autoware topic data for comparison.

- [x] 9.1a — Record baseline topic data
  - `just record-autoware-baseline [duration]` recipe (default 30s drive)
  - `scripts/record_baseline.sh` — orchestrates Autoware → drive sequence → rosbag
  - Uses CycloneDDS (not rmw_zenoh_cpp) for reliable CLI tool interaction
  - Records 20 functional output topics from 7 replaced nodes during planning simulator drive
  - Uses `ros2 bag record --storage mcap` for efficient MCAP format
  - Autonomous drive sequence: initial pose → localization wait → route → engage
  - Reuses existing poses from `tests/src/autoware.rs` (sample-map-planning)
  - Configurable: `INIT_WAIT` (default 60s), `DRIVE_DURATION` (default 30s)
  - Output: `tmp/bags/baseline/` (gitignored via `tmp/`)
  - Topics recorded:
    - `/control/command/control_cmd`
    - `/control/command/gear_cmd`
    - `/control/command/hazard_lights_cmd`
    - `/control/command/turn_indicators_cmd`
    - `/control/command/emergency_cmd`
    - `/control/shift_decider/gear_cmd`
    - `/control/gate_mode_cmd`
    - `/control/vehicle_cmd_gate/is_stopped`
    - `/control/vehicle_cmd_gate/operation_mode`
    - `/system/fail_safe/mrm_state`
    - `/system/mrm/emergency_stop/status`
    - `/system/mrm/comfortable_stop/status`
    - `/system/emergency/gear_cmd`
    - `/system/emergency/hazard_lights_cmd`
    - `/system/emergency/turn_indicators_cmd`
    - `/api/autoware/get/engage`
    - `/autoware/engage`
    - `/api/autoware/get/emergency`
    - `/api/operation_mode/state`
    - `/control/control_validator/validation_status`

- [ ] 9.1b — Record sentinel topic data
  - Add `just record-autoware-sentinel` recipe
  - Record same topics during modified Autoware + sentinel drive
  - Store as rosbag in `tmp/bags/sentinel/`

- [ ] 9.1c — Comparison script
  - Python script `scripts/compare_bags.py` to diff two rosbags
  - Per-topic comparison with configurable tolerance
  - Output: summary table of matches/mismatches per topic per field
  - Handle timestamp normalization (align by sequence, not wall clock)

### 9.2 — Steady-state verification (normal driving)

Verify outputs match during normal autonomous driving (no faults).

- [ ] 9.2a — Control command comparison
  - Compare `/control/command/control_cmd` between baseline and sentinel
  - During normal driving, both should pass through trajectory follower commands
  - Tolerance: exact match on longitudinal fields (passthrough), timing jitter on stamps
  - Verify gear, hazard, turn indicators match (all DRIVE, off, off during normal driving)

- [ ] 9.2b — State topic comparison
  - Compare `/api/operation_mode/state` — both should report AUTONOMOUS
  - Compare `/api/autoware/get/engage` — both should report engaged=true
  - Compare `/system/fail_safe/mrm_state` — both should report NORMAL
  - Compare `/system/mrm/*/status` — both should report AVAILABLE
  - Compare `/api/autoware/get/emergency` — both should report emergency=false
  - These are structural matches (correct enum values)

- [ ] 9.2c — Shift decider comparison
  - Compare `/control/shift_decider/gear_cmd` — should be DRIVE during forward motion
  - Verify dead-zone behavior matches (gear holds when velocity near zero)
  - Check: sentinel runs shift_decider at 30 Hz vs Autoware's 10 Hz — more frequent
    output is acceptable as long as values match

- [ ] 9.2d — Validation status comparison
  - Compare `/control/control_validator/validation_status`
  - Fields: is_valid_max_distance, lateral_deviation, etc.
  - Tolerance: approximate match (filter state may differ due to timing)

### 9.3 — Fault injection verification (MRM behavior)

Verify outputs match when faults are injected.

- [ ] 9.3a — Heartbeat timeout scenario
  - Stop publishing `/api/system/heartbeat` mid-drive
  - Both baseline and sentinel should trigger MRM
  - Compare MrmState transitions: NORMAL → OPERATING
  - Compare MrmBehaviorStatus: AVAILABLE → OPERATING
  - Compare emergency commands (gear=PARK, hazard=ON when stopped)
  - Tolerance: timing of transition may differ by 1-2 heartbeat periods

- [ ] 9.3b — Emergency stop deceleration profile
  - Trigger emergency stop, record Control commands
  - Compare acceleration ramp: should reach target_acceleration (-2.5 m/s²)
  - Compare jerk profile: should match configured target_jerk
  - Compare velocity reaching zero: should happen within similar timeframe
  - Tolerance: ±1 control cycle (33ms) on timing, float epsilon on values

- [ ] 9.3c — Comfortable stop deceleration profile
  - Trigger comfortable stop via MRM handler (when use_comfortable_stop=true)
  - Compare deceleration to emergency — should be gentler
  - Compare status transitions

- [ ] 9.3d — MRM escalation
  - Trigger comfortable stop, then make it fail → should escalate to emergency
  - Compare MrmState and MrmBehaviorStatus transition sequence
  - Verify gear/hazard commands match during escalation

### 9.4 — Parameter equivalence audit

Verify sentinel hardcoded values match Autoware defaults.

- [ ] 9.4a — Extract Autoware default parameters
  - For each of the 7 nodes, find the default parameter YAML or launch defaults
  - Document each parameter and its default value
  - Sources:
    - `autoware-repo/src/universe/autoware_universe/control/autoware_vehicle_cmd_gate/config/`
    - `autoware-repo/src/universe/autoware_universe/system/autoware_mrm_handler/config/`
    - `autoware-repo/src/universe/autoware_universe/system/autoware_mrm_emergency_stop_operator/config/`
    - Launch files in `autoware-repo/src/launcher/autoware_launch/`

- [ ] 9.4b — Compare against sentinel constants
  - For each parameter, find the corresponding sentinel constant or hardcoded value
  - Flag any mismatches
  - Document which differences are intentional vs bugs

- [ ] 9.4c — Fix parameter mismatches
  - Update sentinel constants to match Autoware defaults where appropriate
  - Add unit tests for critical parameter values

### 9.5 — QoS audit

Verify QoS settings are compatible.

- [ ] 9.5a — Document Autoware QoS per topic
  - vehicle_cmd_gate uses transient_local (durable) on most output topics
  - mrm_handler uses default QoS (reliable, volatile)
  - shift_decider uses transient_local on gear_cmd output
  - control_validator uses default QoS
  - List each topic with its QoS profile

- [ ] 9.5b — Compare sentinel QoS
  - nano-ros/zenoh-pico currently uses default QoS for all topics
  - Identify topics where transient_local is needed for late-joining subscribers
  - Determine if QoS mismatch causes any observable issues

- [ ] 9.5c — Implement QoS matching where needed
  - Add QoS configuration to nano-ros publisher API if not already available
  - Set transient_local on topics that need it
  - Verify late-joining `ros2 topic echo` receives the last message

### 9.6 — Automated regression tests

Turn manual comparisons into automated integration tests.

- [ ] 9.6a — Steady-state integration test
  - New test in `tests/tests/planning_simulator.rs`
  - Run sentinel + planning simulator for 20s
  - Verify: all state topics have expected values (AUTONOMOUS, engaged, NORMAL)
  - Verify: control commands are non-zero (vehicle is moving)
  - Verify: no MRM triggered (MrmState stays NORMAL)

- [ ] 9.6b — Heartbeat timeout integration test
  - New test in `tests/tests/planning_simulator.rs`
  - Run sentinel + planning simulator
  - After engagement, stop heartbeat publisher
  - Verify: MRM triggers within timeout period
  - Verify: vehicle decelerates to stop
  - Verify: gear transitions to PARK, hazard lights ON

- [ ] 9.6c — Topic content spot-checks in transport tests
  - Extend `tests/tests/transport_smoke.rs`
  - For key topics, verify message field values (not just topic existence)
  - Example: `/system/fail_safe/mrm_state` contains state=NORMAL initially
  - Example: `/api/operation_mode/state` contains mode=AUTONOMOUS

## Implementation Notes

### Recording tools

Use `ros2 bag record` with `--storage mcap` for efficient recording. The `mcap` format
supports random access and is easier to process programmatically than SQLite3 bags.

### Comparison challenges

- **Non-determinism:** Autoware nodes communicate asynchronously. Message ordering and
  timing vary between runs. Comparison must be sequence-based, not time-based.
- **Transient state:** First few seconds after startup have initialization transients.
  Skip the first N seconds when comparing.
- **Floating point:** Use `abs(a - b) < epsilon` for float comparisons, not exact equality.
  Epsilon should be per-field (e.g., acceleration vs position have different scales).

### Sentinel parameter locations

| Algorithm                         | Constants location                                                                    |
|-----------------------------------|---------------------------------------------------------------------------------------|
| stop_filter                       | `src/autoware_stop_filter/src/lib.rs` — `VX_THRESHOLD`, `WZ_THRESHOLD`                |
| vehicle_velocity_converter        | `src/autoware_vehicle_velocity_converter/src/lib.rs` — `SPEED_SCALE_FACTOR`, stddevs  |
| shift_decider                     | `src/autoware_shift_decider/src/lib.rs` — `VEL_THRESHOLD`, `park_on_goal`             |
| emergency_stop_operator           | `src/autoware_mrm_emergency_stop_operator/src/lib.rs` — `TARGET_ACCEL`, `TARGET_JERK` |
| comfortable_stop_operator         | `src/autoware_mrm_comfortable_stop_operator/src/lib.rs` — `MIN_ACCEL`, jerk limits    |
| mrm_handler                       | `src/autoware_mrm_handler/src/lib.rs` — timeouts, `use_comfortable_stop`, etc.        |
| vehicle_cmd_gate                  | `src/autoware_vehicle_cmd_gate/src/lib.rs` — `VEL_LIM`, accel/jerk limits             |
| control_validator                 | `src/autoware_control_validator/src/lib.rs` — thresholds                              |
| operation_mode_transition_manager | `src/autoware_operation_mode_transition_manager/src/lib.rs` — thresholds              |

## Acceptance Criteria

- [ ] Baseline and sentinel rosbags recorded for same planning simulator scenario
- [ ] Comparison script produces per-topic match/mismatch report
- [ ] Steady-state (normal driving): all functional topics match within tolerance
- [ ] Fault injection (heartbeat timeout): MRM triggers, state transitions match
- [ ] Parameter audit: all sentinel constants match Autoware defaults (or differences documented)
- [ ] QoS audit: no compatibility issues between sentinel and rmw_zenoh_cpp subscribers
- [ ] At least 2 new integration tests for behavioral verification (steady-state + fault)
- [ ] No regressions in existing tests

## References

- Phase 8: topic parity (30 publishers, 555 topics matched)
- Phase 7: integration tests (transport smoke + planning simulator)
- Autoware source: `autoware-repo/src/universe/autoware_universe/`
- Sentinel binary: `src/autoware_sentinel_linux/src/main.rs`
- Algorithm crates: `src/autoware_*/src/lib.rs`
