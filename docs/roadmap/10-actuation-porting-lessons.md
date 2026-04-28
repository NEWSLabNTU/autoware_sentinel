# Phase 10: Actuation Porting & Infrastructure Improvements

**Status:** In progress (10.1–10.5, 10.6 complete)
**Depends on:** Phase 8 (topic parity), Phase 6 (Zephyr application)
**Goal:** Port Autoware's trajectory follower algorithms (PID longitudinal + MPC lateral
controllers) from ARM's actuation_porting project into Rust `#![no_std]` crates, and improve
the sentinel's Zephyr deployment infrastructure (clock, parameters, board overlays).

## Background

ARM's [actuation_porting](https://github.com/oguzkaganozt/actuation_porting) project ports
Autoware's trajectory follower (MPC lateral + PID longitudinal controllers) to Zephyr RTOS
using CycloneDDS — no ROS 2 runtime. Key differences from our approach:

| Aspect         | actuation_porting (ARM)        | autoware-sentinel (ours)                    |
|----------------|--------------------------------|---------------------------------------------|
| Language       | C++ (Autoware code recompiled) | Rust (`#![no_std]`, rewritten)              |
| Middleware     | CycloneDDS (raw DDS)           | nano-ros + Zenoh                            |
| Ported scope   | Control (MPC+PID)              | Safety monitoring (MRM, gate, validators)   |
| Zephyr version | 3.6.0                          | 3.7.0                                       |
| Target HW      | ARM Cortex-R52 (NXP S32Z), FVP | ARM Cortex-M (thumbv7em)                    |
| Message gen    | IDL → C via `idlc`             | `.msg` → Rust via `cargo nano-ros generate` |

The projects are **complementary**: their actuation module handles the control loop, our
sentinel handles safety monitoring. Together they cover both sides of a safety island.

### What they do well

1. **SNTP clock synchronization** — wall-clock time at startup for ROS-compatible timestamps
2. **Embedded DDS buffer tuning** — 8KB receive, 2KB chunks, 1400B max message (MTU-friendly)
3. **Rosbag-based output comparison** — record DDS outputs, diff against Autoware baselines
4. **Board-specific Zephyr overlays** — clean separation of board configs (FVP, NXP S32Z)
5. **Complete control pipeline** — MPC + PID + interpolation + motion utils in one binary

### Where our approach is stronger

1. **Type/memory safety** — Rust `#![no_std]` with `heapless` containers vs C++ raw pointers
2. **Formal verification** — 13 Kani harnesses + Verus proofs (they have none)
3. **Integration testing** — nextest + ManagedProcess + ephemeral ports vs manual DDS pub/sub
4. **Zenoh transport** — lighter than CycloneDDS for embedded, native rmw_zenoh_cpp bridge
5. **Topic name compatibility** — transparent via rmw_zenoh_cpp (they manually prepend `"rt/"`)
6. **Parameter system** — nano-ros has full `ParameterServer` with typed accessors, descriptors,
   range constraints, read-only enforcement, and ROS 2 parameter services (`ros2 param set/get`)

### Algorithm inventory from actuation_porting

| Component                            | C++ LoC    | Complexity | no_std Feasibility | Dependencies  |
|--------------------------------------|------------|------------|--------------------|---------------|
| autoware_pid_longitudinal_controller | 2,458      | Medium     | **High**           | Pure math     |
| autoware_mpc_lateral_controller      | 4,909      | High       | **Medium**         | Eigen, OSQP   |
| autoware_osqp_interface              | 870        | Medium     | **Medium**         | OSQP C lib    |
| autoware_interpolation               | 1,033      | Low        | **High**           | Eigen (basic) |
| autoware_motion_utils                | 714        | Low        | **High**           | Geometry only |
| autoware_universe_utils              | 2,007      | Low-Med    | **High**           | Geometry/math |
| autoware_vehicle_info_utils          | 237        | Trivial    | **High**           | Parameters    |
| autoware_trajectory_follower_base    | 285        | Trivial    | **High**           | Trait defs    |
| autoware_trajectory_follower_node    | 535        | Low        | **High**           | Node wiring   |
| **Total**                            | **13,048** |            |                    |               |

## Subphases

### 10.1 — Foundation math libraries ✓

Port the supporting math/geometry libraries that both controllers depend on.

**Status:** Complete — 4 crates, 40 tests, all cross-compile to thumbv7em-none-eabihf.

#### 10.1a — Interpolation library (`src/autoware_interpolation/`) ✓

- [x] Linear interpolation — `lerp()`, `lerp_single()`, `lerp_multi()`
- [x] Spline interpolation — `SplineInterpolation` with Thomas algorithm tridiagonal solver
  - Fixed-size arrays `[f64; MAX_POINTS]` where MAX_POINTS=256
  - `eval()`, `eval_diff()`, `eval_quad_diff()`, `eval_multi()`
- [x] Akima spline interpolation — `AkimaInterpolation` (weighted slope, reduces oscillations)
- [x] Spherical linear interpolation (SLERP) — `slerp()`, `slerp_single()` (pure Rust, no Eigen)
- [x] 2D trajectory spline — `SplineInterpolationPoints2d` with arc-length parameterization
  - `interpolated_point()`, `interpolated_yaw()`, `interpolated_curvature()`
- [x] 14 unit tests

#### 10.1b — Motion utilities (`src/autoware_motion_utils/`) ✓

- [x] `TrajectoryAccessor` trait — replaces C++ template `<class T>` pattern
- [x] `find_nearest_index()` — nearest point by 2D squared distance
- [x] `find_nearest_index_with_constraints()` — distance + yaw constraints
- [x] `find_first_nearest_index_with_soft_constraints()` — cascading 3-pass search
- [x] `find_first_nearest_segment_index_with_soft_constraints()` — segment-level search
- [x] `calc_signed_arc_length()`, `calc_signed_arc_length_between_points()`
- [x] `calc_longitudinal_offset_to_segment()` — dot product projection
- [x] `calc_curvature()` — Menger curvature at each point
- [x] `search_zero_velocity_index()` — find stopping points
- [x] `is_driving_forward()` — determine driving direction
- [x] 8 unit tests

#### 10.1c — Universe utilities (`src/autoware_universe_utils/`) ✓

- [x] Constants: `GRAVITY`
- [x] Unit conversion: `deg2rad`, `rad2deg`, `kmph2mps`, `mps2kmph`
- [x] Angle normalization: `normalize_radian`, `normalize_degree`
- [x] Distance: `calc_distance_2d`, `calc_squared_distance_2d`, `calc_distance_3d`
- [x] Quaternion: `create_quaternion`, `get_yaw`, `create_quaternion_from_yaw`,
  `create_quaternion_from_rpy`, `get_rpy`
- [x] Pose deviation: `calc_yaw_deviation`, `calc_lateral_deviation`, `calc_longitudinal_deviation`
- [x] Curvature: `calc_curvature` (Menger formula)
- [x] Interpolation: `lerp_point`, `lerp_vector3`
- [x] Direction: `calc_azimuth_angle`, `calc_elevation_angle`, `is_driving_forward`
- [x] 13 unit tests

#### 10.1d — Vehicle info (`src/autoware_vehicle_info_utils/`) ✓

- [x] `VehicleInfo` struct: 10 base params + 8 derived params
- [x] `new()` constructor with clamping (wheel_base, max_steer_angle ≥ 1e-6)
- [x] Default values from Autoware's sample vehicle config
- [x] 5 unit tests

### 10.2 — PID longitudinal controller (`src/autoware_pid_longitudinal_controller/`) ✓

Port from `external/actuation_porting/actuation_module/src/autoware/autoware_pid_longitudinal_controller/`.
This is the highest-value port — pure math, no external C library dependencies.

**Status:** Complete — 37 tests, cross-compiles to thumbv7em-none-eabihf.

- [x] 10.2a — PID controller core (`src/pid.rs`)
  - Standard PID with anti-windup (integral clamping) and per-component limits
  - `PidGains`, `PidLimits`, `PidContributions` structs
  - 8 unit tests

- [x] 10.2b — Smooth stop algorithm (`src/smooth_stop.rs`)
  - Ring buffer velocity history (64 samples)
  - Linear regression time-to-stop estimation
  - Kinematic deceleration: `a = -v²/(2d)` with clamping
  - Weak/strong braking phases based on distance and velocity
  - 6 unit tests

- [x] 10.2c — Lowpass filter (`src/lowpass_filter.rs`)
  - First-order IIR: `y[n] = gain*y[n-1] + (1-gain)*x[n]`
  - 4 unit tests

- [x] 10.2d — Longitudinal controller state machine (`src/lib.rs`)
  - States: DRIVE → STOPPING → STOPPED → EMERGENCY
  - DRIVE: PID velocity feedback + feedforward scaling + brake keeping
  - STOPPING: SmoothStop algorithm with velocity history
  - STOPPED: Configurable braking force (-3.4 m/s²)
  - EMERGENCY: Hard braking (-5.0 m/s²) with jerk limit
  - Slope compensation (pitch-based gravity compensation, configurable source)
  - Jerk limiting (asymmetric rate limiter on acceleration output)
  - Integration gating (velocity threshold + stuck detection)
  - Steer convergence check (blocks departure until lateral converged)
  - 30+ configurable parameters via `ControllerParams`

- [x] 10.2e — Unit tests (19 controller-level tests)
  - State machine transitions (Drive→Stopping→Stopped, Emergency→Stopped)
  - Emergency on overshoot
  - Steer convergence blocking
  - Slope compensation (uphill, reverse, pitch clamping)
  - Jerk limit enforcement
  - Stop distance calculation
  - Trajectory pitch computation

- [x] 10.2f — Kani verification harnesses (8 proofs: pid output bounded, panic-freedom, anti-windup integral clamping, LPF panic-freedom, LPF finite output, diff-limit panic-freedom, diff-limit bounded, slope compensation bounded)

**Message dependencies:** `autoware_control_msgs`, `autoware_planning_msgs`,
`autoware_vehicle_msgs`, `geometry_msgs`, `nav_msgs`

### 10.3 — Trajectory follower base traits (`src/autoware_trajectory_follower_base/`) ✓

Port the trait definitions that both controllers implement.

**Status:** Complete — 6 tests, cross-compiles to thumbv7em-none-eabihf. Zero dependencies.

- [x] `LateralControllerBase` trait with `fn run(&mut self, input: &InputData) -> LateralOutput`
- [x] `LongitudinalControllerBase` trait with `fn run(&mut self, input: &InputData) -> LongitudinalOutput`
- [x] `InputData` struct: trajectory (fixed-size array, 256 points), ego pose/velocity/steer/accel, operation mode
- [x] `TrajectoryPoint` common type: x, y, z, yaw, velocity, acceleration, heading rate, wheel angle
- [x] `LateralOutput`: steering_tire_angle, steering_tire_rotation_rate, LateralSyncData
- [x] `LongitudinalOutput`: velocity, acceleration, LongitudinalSyncData
- [x] `LateralSyncData` (is_steer_converged) and `LongitudinalSyncData` (reserved)

**Source reference:**
- `autoware_trajectory_follower_base/src/lateral_controller_base.cpp`
- `autoware_trajectory_follower_base/src/longitudinal_controller_base.cpp`
- ~285 LoC C++ → ~190 LoC Rust

### 10.4 — MPC lateral controller (`src/autoware_mpc_lateral_controller/`) ✓

Port from `external/actuation_porting/actuation_module/src/autoware/autoware_mpc_lateral_controller/`.
This is the most complex component due to matrix operations and QP solver dependency.

**Status:** Complete — 25 tests, cross-compiles to thumbv7em-none-eabihf. No external deps (pure `libm`).

- [x] 10.4a — Vehicle models (`src/vehicle_model.rs`)
  - `VehicleModel` struct supporting 3 model types via enum dispatch
  - `Kinematics` (dim_x=3): lateral error, yaw error, steering angle with first-order delay
  - `KinematicsNoDelay` (dim_x=2): lateral error, yaw error (direct steering)
  - `Dynamics` (dim_x=4): lateral error, lateral rate, yaw error, yaw rate (tire slip)
  - Bilinear (Tustin) discretization for all models
  - 6 unit tests

- [x] 10.4b — MPC core solver (`src/mpc.rs`)
  - N-step prediction matrix generation (Aex, Bex, Wex) built incrementally
  - CB = Cex * Bex computed block-by-block (avoids full Cex storage)
  - Cost matrices: diagonal Q (tracking), diagonal R1 (input), banded R2 (rate/acc)
  - Feedforward reference: `u_ref = atan(wheelbase * curvature)`
  - Steer rate, acceleration, and lateral jerk penalties in R2
  - Input delay compensation via stored command buffer
  - Velocity-adaptive weights (`heading_error_squared_vel`, `steering_input_squared_vel`)
  - Low-curvature weight switching
  - `MpcWorkspace` struct (~200KB) with all pre-allocated buffers
  - `MAX_HORIZON = 50` (configurable at compile time for embedded)
  - 7 unit tests

- [x] 10.4c — QP solver (`src/mat.rs`)
  - Unconstrained solver via Cholesky LLT decomposition: `u = -H⁻¹ * f`
  - Small matrix inverse (up to 4×4) via Gaussian elimination with partial pivoting
  - General matrix multiply, transpose-multiply, add, scale operations
  - All operations on flat row-major `[f64]` slices — zero external deps
  - 5 unit tests

- [ ] 10.4d — Steering predictor and offset estimator (future)
  - Not needed for initial safety island use case

- [x] 10.4e — MPC lateral controller orchestration (`src/lib.rs`)
  - `MpcLateralController` with `run()` method
  - Admissibility checks (position error, yaw error thresholds)
  - Stop state handling (holds previous command, steer convergence check)
  - Initial state construction for all 3 vehicle model types
  - Output low-pass filtering
  - 6 unit tests

- [x] 10.4f — Kani verification harnesses (8 proofs: inv_small 2x2 panic-freedom, cholesky_solve 2x2 panic-freedom, inv_small singular returns false, kinematics/kinematics_no_delay/dynamics discretization panic-freedom, clamp bounded, trajectory push capacity)

**Matrix operation strategy (chosen):** Fixed-size flat arrays with manual row-major
operations in `mat.rs`. No `nalgebra` dependency — keeps the crate dependency-free
(only `libm`). Works well for the actual matrix sizes (up to 50×50 for QP, 4×4 for models).
Workspace struct pre-allocates all buffers.

### 10.5 — Controller node wiring (`src/autoware_trajectory_follower_node/`)

Wire the lateral and longitudinal controllers into a nano-ros node binary.

- [x] 10.5a — Controller node algorithm ✓
  - `ControllerNode` struct orchestrating MPC lateral + PID longitudinal
  - `update(input, current_time)` → `Option<ControlOutput>` (None if < 3 traj points)
  - Converts `InputData` → MPC's `LateralInput` (builds MpcTrajectory, computes lateral/yaw error)
  - Converts `InputData` → PID's `ControlData` (nearest point, stop distance, pitch, shift)
  - Sync: lateral `is_steer_converged` → longitudinal (blocks departure with large steer)
  - `traj_utils` module: nearest-point search (3-pass soft constraints), signed arc length,
    stop distance, pitch estimation, Menger curvature — all on `TrajectoryPoint` slices
  - 14 tests (7 orchestration + 7 trajectory utils)
  - Cross-compiles to thumbv7em-none-eabihf
  - `.cargo/config.toml` reuses PID controller's generated message crates (no own generation)
  - Source: `controller_node.cpp` (~535 LoC C++ → ~280 LoC Rust)

- [x] 10.5b — Integration into sentinel binary
  - Added `ControllerNode` + `InputData` to `SafetyIsland` struct in `main.rs`
  - 4 new subscriptions: trajectory, odometry, steering, acceleration
  - `run_controller()` runs before MRM chain in 30 Hz timer, updates `auto_control`
  - Executor slots increased from 48 → 52; `ZPICO_MAX_LIVELINESS=52` in justfile + test fixtures
  - Inlined `quaternion_to_pitch_yaw()` to avoid cross-crate geometry_msgs type mismatch

- [x] 10.5c — Integration tests
  - New test binary `tests/tests/controller_node.rs` (3 tests)
  - `test_controller_produces_output`: publishes straight-line trajectory + odometry at rest,
    verifies PID produces non-zero acceleration toward target velocity
  - `test_controller_graceful_without_all_inputs`: verifies sentinel still publishes control_cmd
    when controller has only partial inputs (no crash, graceful degradation)
  - `test_controller_lateral_correction`: ego offset 1m from trajectory, verifies MPC produces
    non-zero steering angle to correct lateral error
  - Added `controller-node` test group in `.config/nextest.toml` (max-threads=1, 60s timeout)

### 10.6 — Sentinel parameter migration

nano-ros already has a full `ParameterServer` with typed accessors, descriptors, range
constraints, read-only enforcement, and ROS 2 parameter services (`~/get_parameters`,
`~/set_parameters`, etc.) gated by the `param-services` feature. Use this existing API
to make sentinel algorithm parameters configurable.

- [x] 10.6a — Migrate existing sentinel constants to parameters
  - Created `src/autoware_sentinel_linux/src/params.rs` with `declare_parameters()` and `read_params()`
  - 56 read-only parameters declared via `ParameterBuilder::new(server, name).default().description().read_only()`
  - `ro!` macro for concise declaration; `SentinelParams` struct aggregates all algorithm params
  - `NROS_MAX_PARAMETERS=64` env var (default is 32) in justfile, test fixtures

- [x] 10.6b — Add new controller parameters
  - PID gains (kp=1.0, ki=0.1, kd=0.0), acceleration/jerk limits, delay compensation
  - MPC prediction horizon (50), dt (0.1s), steer_tau (0.27s), steering LPF, input delay, rate limit
  - Vehicle info (10 params: wheel_base, max_steer, tread, overhangs, etc.)
  - Controller node: ctrl_period, ego_nearest thresholds

- [x] 10.6c — Parameter equivalence audit
  - Audited all 56 parameters against Autoware 1.5.0 YAML defaults
  - Fixed 5 mismatches: `acc_lpf_gain` 0.8→0.97, `vel_lpf_gain` 0.8→0.9,
    `acc_threshold` 2.0→1.5, `stable_check_duration` 3.0→0.1, `twist2accel.accel_lowpass_gain` 0.8→0.9

- [x] 10.6d — Verify `ros2 param list` / `ros2 param get` works
  - Enabled `param-services` feature in sentinel Cargo.toml
  - Added `Executor::parameter::<T>()` convenience method to nano-ros API
  - Added typed parameter types (`MandatoryParameter`, `OptionalParameter`, `ReadOnlyParameter`, etc.) to nano-ros prelude
  - Integration tests: `test_sentinel_param_list` (56 params via `ros2 param list`)
    and `test_sentinel_param_get` (spot-check 5 values via `ros2 param get`)

### 10.7 — Clock synchronization for Zephyr sentinel

The ARM project initializes wall-clock time via SNTP at startup (`Clock::init_clock_via_sntp()`)
so that published ROS timestamps are meaningful. Our Zephyr sentinel currently uses monotonic
time only.

- [ ] 10.7a — Add SNTP client to Zephyr sentinel startup
  - Use Zephyr's `net_sntp` subsystem (`CONFIG_SNTP=y`)
  - Configure NTP server via Kconfig (default: configurable)
  - Set system clock once at boot, before executor starts
  - Fallback: if SNTP fails, log warning and continue with monotonic time

- [ ] 10.7b — ROS timestamp generation from wall clock
  - `Clock::now()` returns `builtin_interfaces::msg::Time` with sec + nanosec
  - Use Zephyr `k_uptime_get()` + SNTP offset for wall-clock conversion
  - Ensure nanosecond precision (Zephyr kernel tick → ns conversion)

- [ ] 10.7c — Verify timestamp compatibility
  - Publish a stamped message from sentinel, echo with `ros2 topic echo`
  - Confirm timestamps are within 1s of ROS 2 node timestamps
  - Add integration test

### 10.8 — Board overlay infrastructure for real hardware

The ARM project has clean board overlay files for FVP and NXP S32Z. Our Zephyr application
only targets `native_sim`.

- [ ] 10.8a — Survey target boards
  - Primary: STM32H7 (Cortex-M7, Ethernet, 1MB+ RAM) — best match for safety MCU
  - Secondary: NXP S32Z (Cortex-R52) — same target as ARM project, direct comparison
  - Tertiary: nRF5340 (Cortex-M33, BLE/Thread) — constrained IoT use case
  - Document minimum requirements: RAM, flash, networking, timer resolution

- [ ] 10.8b — Create board overlay for primary target
  - `src/autoware_sentinel/boards/<board>.overlay` — device tree
  - `src/autoware_sentinel/boards/<board>.conf` — Kconfig
  - Enable Ethernet, configure network stack sizes
  - Test: `west build -b <board> autoware-sentinel/src/autoware_sentinel`

- [ ] 10.8c — CI cross-compilation check
  - Add `just build-zephyr-<board>` recipe
  - Verify build succeeds in CI (no runtime test needed initially)

### 10.9 — Rosbag-based output comparison (overlaps with Phase 9.1)

The ARM project records DDS outputs and diffs against known-good Autoware baselines.

- [ ] 10.9a — Study ARM project's rosbag test format
  - Their `test/rosbag_test/` has YAML-serialized control commands
  - `test_control_cmd_outputs/` contains expected outputs for FVP, S32Z, and original Autoware
  - Adopt similar YAML-based expected output format for our comparison tooling

- [ ] 10.9b — Implement comparison in Phase 9 scripts
  - Integrate YAML baseline format into `scripts/compare_bags.py` (from Phase 9.1c)
  - Per-field tolerance configuration (float epsilon, timestamp tolerance)
  - Human-readable diff output

## Implementation Notes

### Priority ordering

1. **10.1** (foundation math) — prerequisite for both controllers
2. **10.2** (PID controller) — highest value, lowest risk, no C dependencies
3. **10.3** (base traits) — simple, needed by both controllers
4. **10.6** (parameters) — can be done in parallel with algorithm ports
5. **10.4** (MPC controller) — highest complexity, depends on QP solver decision
6. **10.5** (node wiring) — depends on both controllers
7. **10.7–10.9** (infra) — independent, can be done in parallel

### Porting approach

Following the established two-layer architecture:

1. **Algorithm library** (`src/autoware_*/src/lib.rs`): Pure `#![no_std]` logic. No nros
   dependency. Message types only. Fully testable with `cargo test`.
2. **Node binary** (in sentinel main): Uses nros `Executor`/`Node` to wire pub/sub/timer
   callbacks to algorithm instances.

### Eigen → Rust matrix strategy

| C++ (Eigen) | Rust equivalent | Notes |
|-------------|----------------|-------|
| `VectorXd` | `heapless::Vec<f64, N>` or `nalgebra::SVector<f64, N>` | Fixed-size |
| `MatrixXd` | `nalgebra::SMatrix<f64, R, C>` | Fixed rows/cols |
| `colPivHouseholderQr().solve()` | `nalgebra::SVD::solve()` or custom tridiag solver | For splines |
| `A * B` | `nalgebra` operator overloads | Same syntax |
| `A.transpose()` | `.transpose()` | Same API |

### OSQP decision

Start with unconstrained QP solver (Option A) for the initial port. This avoids the C
dependency entirely and is sufficient for a safety island where hard steering limits are
enforced by the vehicle_cmd_gate downstream. Add OSQP FFI behind a feature flag later
if constrained optimization is needed.

### Relationship to nano-ros changes

Transport tuning documentation is tracked in nano-ros Phase 64 (complete: `docs/guides/embedded-tuning.md`).
No new nano-ros features are needed — the existing `ParameterServer` API is sufficient.

## Acceptance Criteria

- [ ] Interpolation, motion utils, and vehicle info crates compile for `thumbv7em-none-eabihf`
- [ ] PID longitudinal controller passes unit tests and Kani verification
- [ ] MPC lateral controller compiles with unconstrained solver (constrained solver optional)
- [ ] Controller node publishes `Control` commands from trajectory + odometry inputs
- [ ] All existing sentinel constants migrated to `ParameterServer` declarations
- [ ] `ros2 param list /sentinel` shows all parameters (Linux binary with `param-services`)
- [ ] Sentinel publishes wall-clock timestamps (SNTP or fallback monotonic)
- [ ] At least one real hardware board overlay compiles
- [ ] Control command outputs validated against ARM project baselines
- [ ] All new crates are `#![no_std]` and pass `just cross-check`
- [ ] No regressions in existing tests

## References

- ARM actuation_porting: `external/actuation_porting/`
- ARM PID controller: `external/actuation_porting/actuation_module/src/autoware/autoware_pid_longitudinal_controller/`
- ARM MPC controller: `external/actuation_porting/actuation_module/src/autoware/autoware_mpc_lateral_controller/`
- ARM controller node: `external/actuation_porting/actuation_module/src/autoware/autoware_trajectory_follower_node/`
- ARM DDS config: `external/actuation_porting/actuation_module/include/common/dds/config.hpp`
- ARM clock: `external/actuation_porting/actuation_module/include/common/clock/clock.hpp`
- ARM rosbag tests: `external/actuation_porting/actuation_module/test/rosbag_test/`
- nano-ros parameter API: `~/repos/nano-ros/packages/core/nros-params/`
- nano-ros parameter services: `~/repos/nano-ros/packages/core/nros-node/src/parameter_services.rs`
- nano-ros embedded tuning guide: `~/repos/nano-ros/docs/guides/embedded-tuning.md`
- Autoware trajectory follower: `autoware-repo/src/universe/autoware_universe/control/`
