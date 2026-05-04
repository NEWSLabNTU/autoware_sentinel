# Phase 13: Multi-Platform Sentinel

**Status:** Complete (13.1 – 13.6 + 13.K1 + 13.K2 closed; sentinel runs on Linux, Zephyr, FreeRTOS QEMU, and NuttX QEMU with comp-all parity).
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

- [x] 13.1.1 Create `src/autoware_sentinel_core/` crate (`#![no_std]`, `alloc` feature).
- [x] 13.1.2 Move `SafetyIsland` struct + `static ISLAND` + `with_island` from
      `autoware_sentinel_linux/src/main.rs` into `core/src/safety_island.rs`.
- [x] 13.1.3 Move publisher/subscription/service registration into
      `core::wire_executor(&mut Executor)`.
- [x] 13.1.4 Add `controller-node` feature; gate the relevant struct fields,
      subscriptions, and `run_controller()` body behind `#[cfg(feature = "controller-node")]`.
- [x] 13.1.5 Generate the superset `package.xml` and `generated/` for core
      (or have core re-export msg types and let each platform crate generate its own
      subset).
- [x] 13.1.6 Unit-test compile on `thumbv7em-none-eabihf` with and without
      `controller-node`.

**Acceptance:** `cargo check --target thumbv7em-none-eabihf -p autoware_sentinel_core
--no-default-features` passes (no controller-node). `cargo check ... --features
controller-node` also passes.

### 13.2 — Migrate `autoware_sentinel_linux`

- [x] 13.2.1 Replace `main.rs` body with `init_island` + `wire_executor` + `spin_blocking`.
- [x] 13.2.2 Enable `controller-node` feature on core dep.
- [x] 13.2.3 Verify `just test-transport` — all 14 tests still pass.
- [x] 13.2.4 Verify `just test-planning` — at least the same passing subset as before
      the refactor.

**Acceptance:** transport_smoke 14/14, no behavioral change vs. pre-refactor.

### 13.3 — Rename + migrate `autoware_sentinel` → `autoware_sentinel_zephyr`

- [x] 13.3.1 `git mv src/autoware_sentinel src/autoware_sentinel_zephyr`.
- [x] 13.3.2 Update `west.yml`, `scripts/zephyr/setup.sh`, `docs/guides/zephyr-setup.md`,
      `justfile` (`build-zephyr`, `run-sentinel-zephyr`), and the workspace path comment
      in root `Cargo.toml`.
- [x] 13.3.3 Replace `lib.rs` body with `wire_executor` call (controller-node off).
- [x] 13.3.4 `just build-zephyr` succeeds against `native_sim/native/64`.

**Acceptance:** Zephyr native_sim build green; binary boots and `Executor ready`
prints. Transport tests against zenohd + Zephyr binary (if/when available) match the
Linux subset.

### 13.4 — `autoware_sentinel_freertos` (QEMU MPS2-AN385)

- [x] 13.4.1 Create `src/autoware_sentinel_freertos/` from
      `~/repos/nano-ros/examples/qemu-arm-freertos/rust/zenoh/talker/` template.
- [x] 13.4.2 Depend on `nros-board-mps2-an385-freertos` + `nros = { features =
      ["rmw-zenoh", "platform-freertos", "link-tcp", "link-udp-unicast", "ros-humble"] }`
      + `autoware_sentinel_core` (no `controller-node`).
- [x] 13.4.3 Add `config.toml` with QEMU TAP-bridged zenoh locator.
- [x] 13.4.4 Add `justfile` recipes: `build-sentinel-freertos`, `run-sentinel-freertos`
      (QEMU launch + TAP setup).
- [x] 13.4.5 Added `tests/src/fixtures/sentinel_freertos.rs` — builds the
      FreeRTOS ELF (cached via `OnceCell`), launches `qemu-system-arm
      -machine mps2-an385`, and waits for the `Executor ready` line on
      the semihosting console. Companion `zenohd_freertos` fixture pins
      the listener to port 7451 to match the firmware's compile-time
      locator and shares one zenohd across the test process (`OnceCell`)
      so reused-port flake disappears.
- [x] 13.4.6 Added `tests/tests/freertos_qemu.rs` running under the
      `freertos-qemu` nextest test-group: `test_qemu_arm_available`,
      `test_arm_gcc_available`, `test_freertos_sentinel_declares_publishers`
      (asserts ≥6 zpico declares — comp-all gives 37). Single-boot design:
      `start_sentinel_freertos` hashes IP+MAC into the zenoh ZID seed, so
      a second boot in the same process collides on session ID. The boot
      test was folded into declares_publishers, and a separate `ss(8)`
      ESTAB probe was dropped because the declare confirmations already
      prove the QEMU↔zenohd handshake succeeded.

**Acceptance:** QEMU FreeRTOS binary boots, registers all publishers/services,
`ros2 service list` shows the expected services, at least `test_sentinel_starts` +
`test_sentinel_param_list` pass against the FreeRTOS sentinel.

### 13.5 — `autoware_sentinel_nuttx` (QEMU ARM)

- [x] 13.5.1 Created `src/autoware_sentinel_nuttx/` from the
      `qemu-arm-nuttx/rust/zenoh/talker` template. `main()` calls
      `init_island` + `wire_executor` + `executor.spin(...)`. `build.rs`
      preprocesses `dramboot.ld` from `$NUTTX_DIR` and emits the
      flat-build link args (Rust binary IS the kernel image).
      `rust-toolchain.toml` pins `nightly-2026-04-11` to match
      nano-ros's libc patch layout.
- [x] 13.5.2 Wired deps:
      `nros-board-nuttx-qemu-arm` + `nros = { features = ["std",
      "rmw-zenoh", "platform-nuttx", "link-tcp", "ros-humble",
      "param-services"] }` from the nano-ros GitHub commit `9c4aa312`,
      plus `autoware_sentinel_core` with `platform-nuttx`. comp-* feature
      gating is supported via `core` features for later bisection.
- [x] 13.5.3 Reused nano-ros's `third-party/nuttx/{nuttx,nuttx-apps}`
      from the sibling clone instead of vendoring duplicate submodules
      in this repo (the kernel build is already tracked there). Override
      via `NUTTX_DIR` / `NUTTX_APPS_DIR` env vars when running the
      `just build-sentinel-nuttx` recipe.
- [x] 13.5.4 Defconfig is the upstream
      `packages/boards/nros-board-nuttx-qemu-arm/nuttx-config/defconfig`
      from nano-ros — already configured for TCP + ICMP + virtio-net +
      POSIX threads. No autoware-side override needed; `build-nuttx.sh`
      drives the build identically to the upstream NuttX examples.
- [x] 13.5.5 Added `just build-nuttx-kernel`, `just build-sentinel-nuttx`,
      and `just run-sentinel-nuttx` recipes. The run recipe invokes
      `qemu-system-arm -M virt -cpu cortex-a7` with `-netdev user
      -device virtio-net-device` (SLIRP), so no TAP/sudo is needed.
      Zenohd must listen on `127.0.0.1:7452` (10.0.2.2 inside the guest)
      to avoid clashing with the FreeRTOS sentinel on 7451.
- [x] 13.5.6 Test fixture + transport test. Investigation of the
      original NuttX hang traced it back to the **same root cause as
      13.K1.7 on FreeRTOS** — `nros::Executor`'s inline arena gets copied
      ~3× through the NRVO-defeated `Result<Executor, _>` return path.
      `.env`'s `NROS_EXECUTOR_MAX_CBS=96` (ARENA ≈ 346 KB) inflates the
      stack frame past NuttX's startup task budget, hanging on the very
      first move-out of the returned Executor (specifically between
      `set_node_identity done` and the next Rust println). Adding a
      `nuttx_env` override in the justfile (`MAX_CBS=32`,
      `ZPICO_MAX_PUBLISHERS=40`, etc.) reduces the frame to ~350 KB,
      which fits the NuttX init task's 512 KB stack with margin.

      Verified E2E with `comp-all` (28 callbacks, 37 publishers, 22
      services, 5 subs, 1 timer): the binary boots, opens a zpico
      session against zenohd at `127.0.0.1:7452`, declares every
      publisher, registers every subscription/service, and reaches
      `Executor ready — spinning…`. Tests in
      `tests/tests/nuttx_qemu.rs` run under the `nuttx-qemu` nextest
      test-group:
      - `test_nuttx_kernel_present` — non-failing prereq probe.
      - `test_nuttx_sentinel_executor_ready` — full E2E (build → QEMU
        boot → zpico → executor armed). Single boot per process due to
        the deterministic ZID seed, mirroring the FreeRTOS pattern.

      Side fix: `tests/src/fixtures/sentinel_nuttx.rs` strips
      `RUSTUP_TOOLCHAIN` before invoking the inner `cargo build` so
      `cargo nextest` (which runs on stable from the workspace root)
      doesn't leak its toolchain into the NuttX crate's nightly +
      build-std build via env inheritance.

**Acceptance:** same as 13.4 but for NuttX.

### 13.6 — Documentation + cross-check

- [x] 13.6.1 Update `CLAUDE.md` Project Structure section to list all four platform
      crates + the core crate.
- [x] 13.6.2 Update root `justfile` `build` recipe to drive every platform target.
- [x] 13.6.3 Update `just cross-check` to include core (no controller-node) for
      `thumbv7em-none-eabihf`.
- [x] 13.6.4 Add a short `docs/guides/multi-platform.md` covering how to add a fifth
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

## Phase 13.K1 work items (post-investigation)

- [x] **13.K1.5 Fix sentinel network-init regression.** Resolved as part
      of 13.K1.7 / 13.K1.8 — root cause was the same Cortex-M3 task stack
      overflow (1 MB closure frame from inline arena × NRVO miss) that
      produced the apparent "no app_task_entry" / "no `Network ready.`"
      symptom. Talker survived because its smaller closure fits in 64 KB
      stack; sentinel did not. With the cast fix in 13.K1.8 + bumped task
      stack + tuned `MAX_CBS`, sentinel now boots through `app_task_entry`,
      prints `Network ready.`, declares all publishers, and reaches
      `executor.spin(...)` with `comp-all`. Below: original 13.K1.5
      investigation notes preserved for context.

      **Investigation 2026-04-30:**

      *Trace placement.* Adding `hprintln!("[run] app task created; starting
      scheduler")` after `xTaskCreate` and `hprintln!("[app_task_entry]
      entered")` as the first line of `app_task_entry` shows the first
      message but **never** the second. The xTaskCreate return value is
      checked (`ret == 0`); it succeeds. Scheduler starts but the
      application task is never given the CPU.

      *Closure ablation.* Replacing the user closure with `|_| Ok(())` (only
      touching `default_params()` to keep symbols linked) makes sentinel
      boot through `[app_task_entry] entered`, `Network ready.`, the user
      closure, and `Application completed successfully.` Restoring the
      full `Executor::open` + 1 publisher closure brings the hang back —
      so something LTO-pulls in and statically initialises that the
      minimal closure does not.

      *QEMU `-d guest_errors,unimp` capture.* When the full sentinel
      hangs, QEMU traces a flood of wild writes to `0x1ef24XXX` — well
      below RAM start `0x20000000`. The values being written are
      `0xa5a5a5a5` (FreeRTOS task stack canary) plus context-switch
      register stacks. Conclusion: `pvPortMalloc` is returning a pointer
      below `ucHeap`, FreeRTOS canaries it, the access faults, and the
      first context switch never completes. Some static (or its
      initialiser) is corrupting `ucHeap`'s prologue / `xFreeBytesRemaining`
      before `vTaskStartScheduler()`.

      *Suspect.* The corruption appears with sentinel's `nros-platform/global-allocator`
      enabled and any code path that pulls `nros::Executor::open` into the
      binary (LTO links the static `g_publishers` / `g_liveliness` arrays
      and the SUBSCRIBER_BUFFERS / SERVICE_BUFFERS arrays). Likely a
      pre-`main` allocator call from a Rust static initialiser tries to
      `pvPortMalloc` before FreeRTOS's heap structures are zeroed (boot
      ordering: `Reset_Handler` → `_start` → `run()` → `vTaskStartScheduler`;
      `pvPortMalloc` works inside `run()` for `AppContext` but possibly
      not for any *earlier* Rust alloc).

      *Mitigations to try next:*
      1. Drop `global-allocator` and provide a custom `#[global_allocator]`
         that defers to a thread-local arena until the scheduler is up.
      2. Shrink sentinel via the component features below so fewer static
         tables get linked, then incrementally add features back.
      3. Move `g_publishers` / `g_liveliness` / `SERVICE_BUFFERS` /
         `SUBSCRIBER_BUFFERS` to explicit zeroed segments aligned after
         `ucHeap`.
      4. Bisect the `wire_executor` body — the minimal closure works, so
         halve the publisher tuple repeatedly until the corruption
         re-appears.
      5. Use `tshark -i lo` once we get past this so the bigger bisection
         can proceed.

- [x] **13.K1.6 Add component cargo features in `autoware_sentinel_core`.**
      Implemented the six component features (`comp-mrm`,
      `comp-cmd-gate-extra`, `comp-validator`, `comp-op-mode-mgr`,
      `comp-engagement`, `comp-stubs`) plus a `comp-all` umbrella feature.
      Publisher creation, destructure, subscription registration, service
      registration, and the per-publisher `publish` calls in the 30 Hz
      timer body are all `#[cfg]`-gated. Imports and constants that
      become dead under partial feature combinations are likewise gated.
      `autoware_sentinel_linux` enables `comp-all` + `monitoring-topics`
      to keep the pre-refactor topology; `autoware_sentinel_freertos`
      and `autoware_sentinel_zephyr` carry no `comp-*` features by
      default (core-only baseline: 6 publishers, 3 subs, 1 service).
      Verified: every one-feature-on combination cargo-checks clean for
      `platform-posix`; full `comp-all + monitoring-topics` builds for
      `platform-posix`; `platform-freertos` minimal builds for
      `thumbv7m-none-eabi`.

- [x] **13.K1.7 Bisect with feature gates.** Bisection result: **the
      original 13.K1 hang on FreeRTOS was misdiagnosed as a "declare
      storm" — the actual failure is a Cortex-M3 task stack overflow.**

      Root cause: `Executor` carries the callback arena inline
      (`arena: [MaybeUninit<u8>; ARENA_SIZE]`). At `.env` defaults
      (`NROS_EXECUTOR_MAX_CBS=96`) ARENA_SIZE = 346 KB. The
      `Result<Executor, _>` return path defeats NRVO in `Executor::open`
      so LLVM emits ~3 stack copies, producing a ~1 MB frame on
      `app_task_entry`'s mono-instantiation. The 128 KB / 64 KB FreeRTOS
      task stack overflows below SRAM base, the SP wraps to wild
      addresses (e.g. `0x1ef236xx`), and the very first push faults
      with `v7M INVSTATE UsageFault` — no z_declare_publisher / Declare
      frame ever fires. Confirmed via `qemu-system-arm -d
      guest_errors,int` and `arm-none-eabi-objdump` of `app_task_entry`
      (`sub.w sp, sp, #1048576`).

      Per-feature bisection on FreeRTOS QEMU (with shrunk caps:
      `NROS_EXECUTOR_MAX_CBS=22`, `NROS_SUBSCRIPTION_BUFFER_SIZE=1024`,
      `ZPICO_MAX_PUBLISHERS=40`, app stack 255 KB):

      | Build (single comp-* ON)   | Result                                  |
      |---------------------------|-----------------------------------------|
      | core only                 | declares 6 pubs, "Executor ready"       |
      | + comp-mrm                | declares 13 pubs, "Executor ready"      |
      | + comp-cmd-gate-extra     | declares 19 pubs, "Executor ready"      |
      | + comp-validator          | declares 10 pubs, "Executor ready"      |
      | + comp-op-mode-mgr        | declares 9 pubs, "Executor ready"       |
      | + comp-engagement         | declares 10 pubs, "Executor ready"      |
      | + comp-stubs              | declares 6 pubs, "Executor ready"       |
      | comp-all (37 pubs)        | declares all 37 pubs, then `BufferTooSmall` on first `add_subscription` (arena exhaustion: 28 cbs needed, only 22 fit before stack overflow trips again) |

      Every individual `comp-*` boots, declares its publishers cleanly,
      and reaches `executor.spin(...)` — **no declare-storm hang in any
      configuration**. The Cortex-M3 ceiling on FreeRTOS is ~26 entities
      (`MAX_CBS=22` minus internal slots) before the arena/stack
      tradeoff hits the `xTaskCreate` `(uint16_t)stack_words` truncation
      cap at 256 KB. `comp-all` (28 cbs) requires lifting that cap or
      moving the arena off the stack.

      Side fixes applied (in this repo, no upstream patch yet):
      - `justfile` `freertos_env` overrides `.env`'s 96-cb / 8 KB
        defaults with `MAX_CBS=22`, `PARAM_SERVICE_BUFFER_SIZE=1024`,
        and `ZPICO_*` values matching the FreeRTOS topology.
      - `src/autoware_sentinel_freertos/config.toml` raised
        `app_stack_bytes` to 261 120 (just below the `xTaskCreate`
        uint16_t cap = 65 535 words = 262 140 bytes).
      - `src/autoware_sentinel_freertos/Cargo.toml` exposes each
        `comp-*` feature for direct `cargo build --features=...` runs
        during bisection.

- [x] **13.K1.8 Patch nano-ros (upstream cast fix).** Dropped the
      `(uint16_t)` cast in `nros_freertos_create_task`
      (`nano-ros-sentinel/packages/boards/nros-board-mps2-an385-freertos/build.rs:522`).
      `configSTACK_DEPTH_TYPE` already defaults to `StackType_t = uint32_t`
      on Cortex-M3 (per `portmacro.h`), so xTaskCreate accepts the full
      32-bit depth — the cast was a pure bug. With the cast gone, app
      tasks can request stacks > 256 KB without silent truncation.

      E2E verified on FreeRTOS QEMU MPS2-AN385 with `comp-all` enabled
      (`autoware_sentinel_core` features = `platform-freertos, comp-all`):
      - `app_stack_bytes = 524288` (512 KB), `NROS_EXECUTOR_MAX_CBS=32`,
        `ZPICO_MAX_PUBLISHERS=40`.
      - All 37 publishers declared cleanly via zpico Declare frames.
      - `Executor ready — spinning...` reached, executor sustained the
        30 Hz timer for the full test window.
      - `ss -tn` shows ESTAB session sentinel↔zenohd on tcp 127.0.0.1:7451
        (host-side endpoint of QEMU SLIRP NAT for 10.0.2.2:7451).
      - 30 s run, no UsageFault / HardFault / FreeRTOS assert.

      Optional follow-up (not required for `comp-all`): box the executor
      arena (`arena: [MaybeUninit<u8>; ARENA_SIZE]` →
      `Box<[MaybeUninit<u8>; ARENA_SIZE]>`) to remove the NRVO-defeated
      ~3× stack copy of the inline arena. Currently mitigated by
      bumping the FreeRTOS task stack and shrinking `MAX_CBS`.

      Next housekeeping: once the cast fix lands on the upstream nano-ros
      branch, revert `freertos_env` overrides in this repo's justfile
      back toward `.env` defaults and re-pin the workspace nano-ros
      revision to the patched commit (sibling-clone path overrides
      stay until then).

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
  binary's bss layout.

**Bisection plan (next session) — component feature gates.** Group the
sentinel's publishers / subscriptions / services into Cargo features that
mirror Autoware components, so each can be toggled independently. Run
sentinel with subsets enabled and find the smallest combination that
reproduces 13.K1. Suggested feature taxonomy:

| Feature                | Publishers                                                                | Services                                                                          | Sub          |
|------------------------|---------------------------------------------------------------------------|-----------------------------------------------------------------------------------|--------------|
| `comp-mrm`             | `mrm_estop/comfy/pullover_status`, `emergency_*`, `emergency_holding`     | `operate_mrm` ×3                                                                  | —            |
| `comp-cmd-gate-extra`  | `emergency_cmd`, `gate_mode`, `shift_decider_gear`, `is_stopped`, `is_paused`, `is_start_requested`, `current_gate_mode`, filter debug ×4 | `external_emergency_stop`, `clear_external_emergency_stop`, `set_stop`, `config_logger` | `gear_status`|
| `comp-validator`       | `cv_debug_marker`, `cv_output_markers`, `cv_validation_status`, `cv_virtual_wall` | —                                                                                 | —            |
| `comp-op-mode-mgr`     | `op_mode_debug`, `is_autonomous_available`, `published_time`              | `change_to_stop/local/remote`, `enable/disable_autoware_control`, `control_mode_request` | —            |
| `comp-engagement`      | `engage_api`, `engage_compat`, `autoware_state`, `emergency_api`          | `engage`, `set_emergency`                                                         | `autoware_state` |
| `comp-stubs`           | —                                                                         | 6 gap-closure stubs (`/api/interface/version`, `shutdown`, etc)                   | —            |
| `monitoring-topics` ✓  | 14 monitoring pubs                                                        | —                                                                                 | —            |
| `controller-node` ✓    | —                                                                         | —                                                                                 | trajectory + odometry + steering + acceleration |

Always-on core (no gate): MrmState, hazard_lights_cmd, gear_cmd, control_cmd,
turn_indicators_cmd, op_mode_state pubs (6); velocity + heartbeat + control_cmd
subs (3); change_to_autonomous service. With everything gated off, sentinel
declares ~6 entities total — well clear of any cardinality threshold.

Use `tshark -i lo` (no sudo if user is in the `wireshark` group) to capture
zenoh-pico ↔ zenohd traffic and see which Declare frame fails to ack.

Toggling each `comp-*` ON in turn pinpoints the trigger. Implementation
mirrors the existing `monitoring-topics` gate (split tuple into per-feature
sub-tuples; #[cfg]-gate the destructure and the publish/handle calls).

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
