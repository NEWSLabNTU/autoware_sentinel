# Phase 13: Multi-Platform Sentinel

**Status:** Not started
**Depends on:** Phase 6 (Zephyr application), Phase 7 (integration testing), Phase 10.5 (controller node)
**Goal:** Run the sentinel on every nano-ros-supported RTOS (Linux, Zephyr, FreeRTOS, NuttX)
from a single shared core crate, and gate the trajectory follower (controller_node) so
embedded targets ship a minimal safety-island image without MPC/PID overhead.

## Background

Today the sentinel exists as two parallel implementations:

- `src/autoware_sentinel_linux/src/main.rs` — 1436 lines, std + POSIX + Zenoh.
- `src/autoware_sentinel/src/lib.rs` — 439 lines, no_std + Zephyr.

Both wire the same algorithm crates (StopFilter, MrmHandler, VehicleCmdGate,
ControlValidator, OperationModeTransitionManager, etc.) onto an nros `Executor` —
identical topic strings, identical 30 Hz pipeline, identical service handlers. The
divergence is platform-only (entry point, logger, build system).

Adding FreeRTOS and NuttX without refactoring would mean four near-duplicate copies of
the same wiring. Instead, factor the shared logic into a no_std core crate and reduce
each platform binary to a thin entry-point shim (~50 lines) that opens an executor,
calls `core::wire_executor()`, and runs the platform's spin loop.

## Design

### Crate layout

```
src/
├── autoware_sentinel_core/      # NEW — shared wiring (no_std + alloc)
│   ├── Cargo.toml
│   ├── package.xml              # superset of msg deps
│   ├── justfile
│   └── src/lib.rs               # SafetyIsland, with_island, wire_executor()
├── autoware_sentinel_linux/     # exists — slimmed to ~50 lines
├── autoware_sentinel_zephyr/    # renamed from autoware_sentinel — slimmed
├── autoware_sentinel_freertos/  # NEW — QEMU MPS2-AN385 first
└── autoware_sentinel_nuttx/     # NEW — QEMU ARM first
```

`autoware_sentinel_core` lives in the workspace; platform crates remain excluded
(matching the existing `autoware_sentinel` Zephyr crate pattern) because each one
selects a different `nros` platform feature and links a different board crate.

### Core crate API

```rust
// autoware_sentinel_core/src/lib.rs
#![no_std]
extern crate alloc;

pub use safety_island::SafetyIsland;

/// Initialize the static SafetyIsland with default parameters.
pub fn init_island();

/// Register all publishers, subscriptions, services, parameter services, and
/// the 30 Hz control timer on the given executor. Caller is responsible for
/// running the spin loop afterwards.
pub fn wire_executor(executor: &mut nros::Executor) -> Result<(), nros::NodeError>;
```

`SafetyIsland`, `with_island()`, and the `static ISLAND: SyncRefCell<...>` move into
core. Single-threaded executor on every target → `RefCell` borrow checking is safe.

### controller_node feature gate

The trajectory follower (Phase 10.5) is a **convenience** for solo Linux testing.
On a real safety MCU, the main compute runs the production trajectory follower; the
sentinel only needs MRM, cmd_gate, validator, and watchdog. The follower pulls in
heavy crates (MPC + PID + interpolation + universe_utils + vehicle_info_utils) with
non-trivial flash/RAM cost and a non-deterministic QP solver.

`autoware_sentinel_core` exposes a `controller-node` feature:

```toml
[features]
default = []
controller-node = [
    "dep:autoware_trajectory_follower_node",
    "dep:autoware_trajectory_follower_base",
    "dep:autoware_mpc_lateral_controller",
    "dep:autoware_pid_longitudinal_controller",
    "dep:autoware_motion_utils",
    "dep:autoware_interpolation",
    "dep:autoware_universe_utils",
    "dep:autoware_vehicle_info_utils",
]
```

| Platform crate            | controller-node | Rationale                                      |
|---------------------------|-----------------|------------------------------------------------|
| `autoware_sentinel_linux` | on              | Solo dev / planning-simulator e2e              |
| `autoware_sentinel_zephyr`| off             | Safety MCU — main compute provides control     |
| `autoware_sentinel_freertos`| off           | Safety MCU                                     |
| `autoware_sentinel_nuttx` | off             | Safety MCU                                     |

When `controller-node` is off:
- `SafetyIsland` omits `controller_node`, `input_data`, `has_trajectory`, `has_odometry`,
  `has_steering` fields.
- `run_controller(now)` becomes a no-op.
- Subscriptions for `/planning/.../trajectory`, `/localization/kinematic_state`,
  `/vehicle/status/{steering,acceleration}` are not registered.
- Sentinel relies entirely on `/control/command/control_cmd` from the main compute
  (`has_external_control = true`); MRM stale-ness guard already handles dropouts.

### Per-platform glue

| Platform   | Entry point                          | Board crate                                | Logger              | Build system   |
|------------|--------------------------------------|--------------------------------------------|---------------------|----------------|
| Linux      | `fn main()`                          | none (direct `Executor::open`)             | env_logger          | cargo          |
| Zephyr     | `staticlib` via zephyr-lang-rust     | zephyr module                              | zephyr log API      | west           |
| FreeRTOS   | `_start()` extern C                  | `nros-board-mps2-an385-freertos` (QEMU)    | semihosting println | cargo + QEMU   |
| NuttX      | NuttX entry-point macro              | `nros-board-nuttx-qemu-arm`                | NuttX syslog        | NuttX kconfig  |

The board crates' `run(config, |cfg| { ... })` pattern handles task creation, network
bring-up, and zenoh-pico locator — each platform binary's body is essentially:

```rust
run(Config::from_toml(include_str!("config.toml")), |config| {
    let exec_config = ExecutorConfig::new(config.zenoh_locator)
        .domain_id(config.domain_id)
        .node_name("sentinel");
    let mut executor = Executor::open(&exec_config)?;
    autoware_sentinel_core::init_island();
    autoware_sentinel_core::wire_executor(&mut executor)?;
    executor.spin_blocking(SpinOptions::default())
})
```

### Capacity envs

`ZPICO_MAX_PUBLISHERS=40`, `ZPICO_MAX_SUBSCRIBERS=16`, `ZPICO_MAX_LIVELINESS=64`,
`NROS_MAX_PARAMETERS=64`, `NROS_EXECUTOR_MAX_CBS=64`, `NROS_PARAM_SERVICE_BUFFER_SIZE=8192`.
These are read at compile time inside nros. Each platform crate sets them via its own
`.env` (or `build.rs`). Embedded targets without controller-node may lower
`MAX_SUBSCRIBERS` / `MAX_CBS` once the dropped subscriptions are accounted for.

### Parameter services

`executor.register_parameter_services()` lives in `wire_executor()` — uniform across
all platforms. ROS 2 `ros2 param list/get/set` works on every target.

## Work Items

### 13.1 — Extract `autoware_sentinel_core`

- [ ] 13.1.1 Create `src/autoware_sentinel_core/` crate (`#![no_std]`, `alloc` feature).
- [ ] 13.1.2 Move `SafetyIsland` struct + `static ISLAND` + `with_island` from
      `autoware_sentinel_linux/src/main.rs` into `core/src/safety_island.rs`.
- [ ] 13.1.3 Move publisher/subscription/service registration into
      `core::wire_executor(&mut Executor)`.
- [ ] 13.1.4 Add `controller-node` feature; gate the relevant struct fields,
      subscriptions, and `run_controller()` body behind `#[cfg(feature = "controller-node")]`.
- [ ] 13.1.5 Generate the superset `package.xml` and `generated/` for core
      (or have core re-export msg types and let each platform crate generate its own
      subset).
- [ ] 13.1.6 Unit-test compile on `thumbv7em-none-eabihf` with and without
      `controller-node`.

**Acceptance:** `cargo check --target thumbv7em-none-eabihf -p autoware_sentinel_core
--no-default-features` passes (no controller-node). `cargo check ... --features
controller-node` also passes.

### 13.2 — Migrate `autoware_sentinel_linux`

- [ ] 13.2.1 Replace `main.rs` body with `init_island` + `wire_executor` + `spin_blocking`.
- [ ] 13.2.2 Enable `controller-node` feature on core dep.
- [ ] 13.2.3 Verify `just test-transport` — all 14 tests still pass.
- [ ] 13.2.4 Verify `just test-planning` — at least the same passing subset as before
      the refactor.

**Acceptance:** transport_smoke 14/14, no behavioral change vs. pre-refactor.

### 13.3 — Rename + migrate `autoware_sentinel` → `autoware_sentinel_zephyr`

- [ ] 13.3.1 `git mv src/autoware_sentinel src/autoware_sentinel_zephyr`.
- [ ] 13.3.2 Update `west.yml`, `scripts/zephyr/setup.sh`, `docs/guides/zephyr-setup.md`,
      `justfile` (`build-zephyr`, `run-sentinel-zephyr`), and the workspace path comment
      in root `Cargo.toml`.
- [ ] 13.3.3 Replace `lib.rs` body with `wire_executor` call (controller-node off).
- [ ] 13.3.4 `just build-zephyr` succeeds against `native_sim/native/64`.

**Acceptance:** Zephyr native_sim build green; binary boots and `Executor ready`
prints. Transport tests against zenohd + Zephyr binary (if/when available) match the
Linux subset.

### 13.4 — `autoware_sentinel_freertos` (QEMU MPS2-AN385)

- [ ] 13.4.1 Create `src/autoware_sentinel_freertos/` from
      `~/repos/nano-ros/examples/qemu-arm-freertos/rust/zenoh/talker/` template.
- [ ] 13.4.2 Depend on `nros-board-mps2-an385-freertos` + `nros = { features =
      ["rmw-zenoh", "platform-freertos", "link-tcp", "link-udp-unicast", "ros-humble"] }`
      + `autoware_sentinel_core` (no `controller-node`).
- [ ] 13.4.3 Add `config.toml` with QEMU TAP-bridged zenoh locator.
- [ ] 13.4.4 Add `justfile` recipes: `build-sentinel-freertos`, `run-sentinel-freertos`
      (QEMU launch + TAP setup).
- [ ] 13.4.5 Add `tests/src/fixtures/sentinel_freertos.rs` — boot QEMU, wait for
      executor-ready string on semihosting, expose locator.
- [ ] 13.4.6 Run a subset of transport_smoke tests against the FreeRTOS binary.

**Acceptance:** QEMU FreeRTOS binary boots, registers all publishers/services,
`ros2 service list` shows the expected services, at least `test_sentinel_starts` +
`test_sentinel_param_list` pass against the FreeRTOS sentinel.

### 13.5 — `autoware_sentinel_nuttx` (QEMU ARM)

- [ ] 13.5.1 Create `src/autoware_sentinel_nuttx/` from
      `~/repos/nano-ros/examples/qemu-arm-nuttx/rust/zenoh/talker/` template.
- [ ] 13.5.2 Depend on `nros-board-nuttx-qemu-arm` + `nros = { features =
      ["std", "rmw-zenoh", "platform-nuttx", "link-tcp", "ros-humble"] }`
      + `autoware_sentinel_core` (no `controller-node`).
- [ ] 13.5.3 Vendor NuttX + nuttx-apps as submodules under `external/nuttx{,-apps}/`,
      or symlink to the nano-ros checkouts.
- [ ] 13.5.4 Provide NuttX defconfig matching Sentinel's network requirements
      (TCP + ICMP + zenoh-pico-suitable buffers).
- [ ] 13.5.5 `just build-sentinel-nuttx` + `just run-sentinel-nuttx` recipes.
- [ ] 13.5.6 Add fixture + run a subset of transport_smoke tests.

**Acceptance:** same as 13.4 but for NuttX.

### 13.6 — Documentation + cross-check

- [ ] 13.6.1 Update `CLAUDE.md` Project Structure section to list all four platform
      crates + the core crate.
- [ ] 13.6.2 Update root `justfile` `build` recipe to drive every platform target.
- [ ] 13.6.3 Update `just cross-check` to include core (no controller-node) for
      `thumbv7em-none-eabihf`.
- [ ] 13.6.4 Add a short `docs/guides/multi-platform.md` covering how to add a fifth
      platform.

**Acceptance:** `just ci` builds and tests every platform that has CI infrastructure
(at least Linux + Zephyr + one of FreeRTOS/NuttX QEMU).

## Out of Scope

- **Production board ports** (NXP S32K344, STM32H743, NVIDIA Orin SPE). Those land in
  Phase 11 (Orin SPE) and a future "Phase 14: Production Hardware" once 13.4–13.5 prove
  the QEMU paths.
- **Controller-node on embedded**: deferred until production deployment shows main
  compute can fail in a way the safety MCU must compensate for autonomously. Today's
  position is "main compute provides control_cmd; sentinel detects staleness and
  triggers MRM."
- **rmw-xrce / rmw-dds backends**: keeping `rmw-zenoh` only across all four platforms
  for now — switching backends is orthogonal to platform porting.

## References

- Phase 6 Zephyr application: `docs/roadmap/6-zephyr-application.md`
- Phase 7 integration testing: `docs/roadmap/7-integration-testing.md`
- Phase 10.5 controller node: `docs/roadmap/10-actuation-porting-lessons.md`
- nano-ros FreeRTOS example: `~/repos/nano-ros/examples/qemu-arm-freertos/`
- nano-ros NuttX example: `~/repos/nano-ros/examples/qemu-arm-nuttx/`
- nano-ros board crates: `~/repos/nano-ros/packages/boards/`
