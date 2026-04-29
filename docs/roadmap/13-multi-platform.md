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

## Known Issues

### 13.K1 — `z_declare_publisher` fails on Zephyr **and** FreeRTOS QEMU under the bringup-time declare burst

**Status:** Open. Investigated 2026-04-29. Confirmed platform-agnostic
(reproduces on both Zephyr native_sim and QEMU MPS2-AN385 + FreeRTOS).
Tracked as a separate nano-ros / zenoh-pico bug.

The `autoware_sentinel_zephyr` binary boots, connects to host zenohd over NSOS,
declares the node liveliness, registers all 6 ROS 2 parameter services, declares
43 read-only parameters, then enters the publisher loop. After the 27th publisher
succeeds, the 28th call (`/control/vehicle_cmd_gate/is_filter_activated/flag`,
type `BoolStamped`) hangs for ~24 seconds and `z_declare_publisher` returns `-1`.

The `autoware_sentinel_freertos` QEMU binary, with the `monitoring-topics` feature
off (so only the 37 mandatory publishers are declared) and parameter services
disabled, also fails with `Transport(PublisherCreationFailed)` — confirming the
fault is not Zephyr-specific and not tied to the parameter-service queryables.

The failure point on Zephyr is **deterministic**: same publisher, same timestamp
(`00:00:26.677`) across runs.

**Ruled out:**

- `ZPICO_MAX_PUBLISHERS` (compiled to 56, confirmed in `shim_constants.rs`).
- `ZPICO_MAX_LIVELINESS` (96, well above the ~58 declared at fail point).
- `ZPICO_MAX_QUERYABLES` (32, only 6 in use).
- Heap exhaustion (`CONFIG_HEAP_MEM_POOL_SIZE` / `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`
  bumped to 16 MiB each — no change).
- TX batch size (`CONFIG_NROS_BATCH_UNICAST_SIZE=65535` — no change; note that
  zenoh-pico's `Z_BATCH_UNICAST_SIZE` cmake var is **not** plumbed through nano-ros
  and stays at 2048, but 2048 bytes is plenty for a single declare message).
- Real-time slowdown (`CONFIG_NATIVE_SIM_SLOWDOWN_TO_REAL_TIME=n` — no change).

**Likely cause:** zenoh-pico-internal limit triggered by the high-cardinality
declaration burst (≈70 zenoh entities back-to-back, or ≈37 publishers when the
`monitoring-topics` feature is off). Either a `_z_send_declare` TX queue
saturation (with `Z_CONGESTION_CONTROL_BLOCK` and `wait_before_close=5s`) or a
write-filter / session-state allocation issue. The fact that FreeRTOS-with-only-37-publishers
also fails suggests the threshold is closer to ~30 publishers, not 50+.

**Update (2026-04-30) — bug not just `N publishers`.** Wrote a minimal-repro
example crate at `nano-ros-sentinel/examples/qemu-arm-freertos/rust/zenoh/declare-storm/`
that declares 50 publishers in a tight loop (kept alive via a `heapless::Vec`,
sentinel-style long topic names, `Int32` payload). It runs all 50 declarations
to completion on QEMU MPS2-AN385 + FreeRTOS — slot indices increment 0→49,
no failure. The native equivalent (`examples/native/rust/zenoh/declare-storm/`)
also runs N=80 fine on Linux with `ZPICO_MAX_PUBLISHERS=80`. Therefore:

- 13.K1 is **not** a generic "create_publisher up to N" cardinality bug.
- The bug needs more state than just N alive publishers — likely the
  combination of many large generated message types, many add_service
  queryables, or a memory-pressure interaction specific to the sentinel
  binary's bss layout. Next narrowing step: gradually add to the repro
  the things sentinel has but declare-storm does not (5 add_subscription
  + 17 add_service before the publisher loop, dummy huge `static` rodata,
  diverse message types).

**Investigation tooling (2026-04-29):** cloned `~/repos/nano-ros-sentinel`
as a sibling of the upstream `~/repos/nano-ros` checkout (the latter is in
active use by another agent), so the sentinel build can patch nros to a
local path during 13.K1 debugging without disturbing the other agent.
Re-pointing is a two-line edit:

```toml
# Cargo.toml
nros = { path = "../nano-ros-sentinel/packages/core/nros" }
nros-core = { path = "../nano-ros-sentinel/packages/core/nros-core" }
nros-serdes = { path = "../nano-ros-sentinel/packages/core/nros-serdes" }
```

…plus matching `[patch.crates-io.nros*]` entries in
`src/autoware_sentinel_zephyr/.cargo/config.toml` and
`src/autoware_sentinel_freertos/.cargo/config.toml`, and the `freertos_env`
paths in `justfile`. Revert before committing — the sibling clone is local-only.

**Secondary regression on newer nano-ros HEAD (1b7466ce):** the
`Phase 97.4.freertos: lwIP DDS bring-up` commit (`d9722b31`) inflated
`nros-platform-freertos` and `lan9118-lwip` with IGMP/multicast init code,
which combined with sentinel's larger bss footprint stalls the FreeRTOS
network-init phase before reaching `wire_executor`. The talker example still
works on the same HEAD. Sentinel reaches `Network ready.` on the older
`34f1d473` rev. Track separately as a Phase 13 follow-up.

**Workarounds for users today:**

1. ✓ **Landed** — `monitoring-topics` cargo feature in `autoware_sentinel_core`
   (Phase 13.4). Linux enables it for full Phase 12 parity; safety-MCU binaries
   (Zephyr / FreeRTOS / NuttX) leave it off. Removes 14 publishers + the 2.1 MB
   `static DiagGraphStatus` rodata blob (so the FreeRTOS binary actually fits
   the 4 MB Cortex-M3 flash budget). Insufficient on its own to dodge the
   declare-storm bug (FreeRTOS with 37 publishers still fails).
2. Insert a short `k_msleep(5)` / `vTaskDelay(5)` between `create_publisher`
   calls to let the read thread drain acks (would require core API changes;
   may not help if the fault is queue-state rather than timing).
3. Stagger declarations across multiple ticks (declare a few publishers,
   spin once, declare more — would require restructuring `wire_executor`).

**Next steps:**

- Reduce to a minimal nano-ros example that declares 30+ publishers in a tight
  loop on Zephyr native_sim and reproduces upstream.
- File against nano-ros / zenoh-pico for triage.
- Once fixed, re-enable Zephyr E2E tests in CI.

The Linux sentinel is unaffected (full nros + zenoh path; 14/14 transport_smoke
tests pass with all 51 publishers).

## References

- Phase 6 Zephyr application: `docs/roadmap/6-zephyr-application.md`
- Phase 7 integration testing: `docs/roadmap/7-integration-testing.md`
- Phase 10.5 controller node: `docs/roadmap/10-actuation-porting-lessons.md`
- nano-ros FreeRTOS example: `~/repos/nano-ros/examples/qemu-arm-freertos/`
- nano-ros NuttX example: `~/repos/nano-ros/examples/qemu-arm-nuttx/`
- nano-ros board crates: `~/repos/nano-ros/packages/boards/`
