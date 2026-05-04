# Multi-Platform Sentinel Guide

Status: Phase 13 complete (Linux, Zephyr native_sim, FreeRTOS QEMU
MPS2-AN385, NuttX QEMU virt cortex-a7).

This guide explains the platform layering, what each crate owns, and how
to add a fifth platform without touching the algorithm code.

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│ Algorithm crates (workspace, no_std + alloc)                         │
│   autoware_stop_filter, autoware_mrm_handler, autoware_vehicle_cmd_  │
│   gate, autoware_control_validator, autoware_twist2accel, …          │
│                                                                      │
│ Pure data + decision logic. No nros, no zenoh, no I/O.               │
└──────────────────────────────────────────────────────────────────────┘
                              ▲
                              │ used by
                              │
┌──────────────────────────────────────────────────────────────────────┐
│ autoware_sentinel_core (workspace, no_std + alloc)                   │
│                                                                      │
│ Owns SafetyIsland struct, the static ISLAND cell, and                │
│ wire_executor(). Registers every publisher/subscription/service/     │
│ timer against an `nros::Executor` passed in by the platform binary.  │
│                                                                      │
│ Feature gates:                                                       │
│   controller-node     — bundle MPC + PID + interpolation             │
│   monitoring-topics   — Phase 12 diagnostic publishers (~2 MB rodata)│
│   comp-{mrm, cmd-gate-extra, validator, op-mode-mgr,                 │
│         engagement, stubs}                                           │
│   comp-all            — all comp-* on (Phase 12 parity)              │
│   platform-{posix, zephyr, freertos, nuttx, bare-metal}              │
│     — passthrough to nros's matching platform feature                │
└──────────────────────────────────────────────────────────────────────┘
                              ▲
                              │ wire_executor(&mut executor, now_ms)?
                              │
┌─────────────────────┬───────────────────┬──────────────────┬────────┐
│ Linux               │ Zephyr            │ FreeRTOS         │ NuttX  │
│ sentinel_linux      │ sentinel_zephyr   │ sentinel_freertos│ _nuttx │
│                     │                   │                  │        │
│ tokio? no — std     │ #![no_std], main  │ #![no_main],     │ std on │
│ main + env_logger.  │ exposed through   │ _start ABI calls │ NuttX  │
│ Linker: cc.         │ Zephyr `K_THREAD_ │ board::run().    │ POSIX. │
│ Target: x86_64.     │ DEFINE`.          │ Linker: thumbv7m │ Linker:│
│                     │ Built via west.   │ + mps2_an385.ld. │ flat   │
│                     │                   │                  │ build. │
└─────────────────────┴───────────────────┴──────────────────┴────────┘
```

## Per-platform crate contents

Each `src/autoware_sentinel_<platform>/` follows the same shape:

| File | Purpose |
|------|---------|
| `Cargo.toml` | platform-specific deps (`nros-platform`, `nros-board-*`, `nros` features), plus `autoware_sentinel_core` with the right `platform-*` + `comp-*` features |
| `src/main.rs` (or `src/lib.rs` for Zephyr) | thin shim: builds `Config`, opens `Executor`, calls `init_island` + `wire_executor`, calls `executor.spin(...)` |
| `config.toml` | network IP/MAC + zenoh locator (parsed by the board crate's `Config::from_toml`) |
| `build.rs` | NuttX/FreeRTOS only — link script preprocessing + lib search paths |
| `.cargo/config.toml` | target triple, linker, `[patch.crates-io]` overrides for nros + msg crates |
| `rust-toolchain.toml` | NuttX only — pinned nightly for `-Z build-std` |
| `.gitignore` | `/target/` |

**The shim never owns business logic.** Anything that touches a publisher
type or a service handler lives in `autoware_sentinel_core::wire_executor`.

## Build + run recipes (root `justfile`)

| Recipe | What |
|--------|------|
| `just build` | every platform target back-to-back (algorithm crates + Linux + Zephyr + FreeRTOS + NuttX) |
| `just cross-check` | algorithm crates for `thumbv7em-none-eabihf` + core in two flavours (no controller, with controller) |
| `just build-sentinel-linux` | Linux x86_64 release |
| `just build-sentinel-zephyr` | Zephyr `native_sim/native/64` via west |
| `just build-sentinel-freertos` | FreeRTOS QEMU MPS2-AN385 |
| `just build-nuttx-kernel` | NuttX kernel + apps (uses nano-ros's `build-nuttx.sh`) |
| `just build-sentinel-nuttx` | NuttX QEMU virt cortex-a7 — depends on `build-nuttx-kernel` |
| `just run-sentinel-{linux,zephyr,freertos,nuttx}` | boot the target binary against a host zenohd |

Per-platform capacity envs live in justfile variables (`freertos_env`,
`nuttx_env`) so the embedded builds can override `.env`'s 96-callback
arena down to whatever fits the target's task stack — see Phase 13.K1.7
for the back-story (inline arena × NRVO copy = stack overflow).

## Integration tests

`tests/tests/` runs each platform under nextest. Each platform has its
own test-group (`max-threads = 1`) because the QEMU sentinels bake the
zenoh locator port into firmware (7451 FreeRTOS, 7452 NuttX) and seed
the zenoh ZID from `IP+MAC`, making concurrent boots collide on session
ID.

| Binary | Tests | Notes |
|--------|-------|-------|
| `transport_smoke` | 9 | sentinel_linux ↔ ROS 2 over zenohd |
| `planning_simulator` | 6 | full Autoware planning sim |
| `controller_node` | … | Linux dev variant |
| `zephyr_native_sim` | … | Zephyr native_sim binary |
| `freertos_qemu` | 3 | qemu-system-arm MPS2-AN385 |
| `nuttx_qemu` | 2 | qemu-system-arm virt cortex-a7 |
| `auto_drive_comparison` | … | full auto-drive scripted route |

Each platform fixture lives in `tests/src/fixtures/sentinel_<platform>.rs`
and:

1. Builds the cross-target ELF via `cargo build` with the right env vars
   (cached in a `OnceCell` per test process).
2. Spawns QEMU (or the Linux binary directly), wires its stdout/stderr
   into a `ManagedProcess`.
3. Waits for the `Executor ready — spinning…` line on the semihosting/
   serial console.

## Adding a fifth platform

Concrete recipe — assume target `XYZ` with board crate
`nros-board-xyz` already living in nano-ros.

### 1. Pin the platform feature in `autoware_sentinel_core`

Edit `src/autoware_sentinel_core/Cargo.toml`:

```toml
[features]
platform-xyz = ["nros/platform-xyz"]
```

If XYZ provides a libc / std layer add `"std"` to that list and the core
will pick up alloc-backed paths automatically. If XYZ is bare-metal,
keep it std-less and rely on the existing `alloc` crate.

### 2. Create `src/autoware_sentinel_xyz/`

Copy the closest-shaped sibling (FreeRTOS for bare-metal MCUs, NuttX for
POSIX-y RTOSes, Zephyr for west-driven setups) and adjust:

- `Cargo.toml`: deps point at `nros-platform-xyz`, `nros-board-xyz`,
  and `autoware_sentinel_core` with `platform-xyz` (+ `comp-all` if the
  target has enough memory for Phase 12 parity).
- `src/main.rs`: identical wiring; only changes are the platform clock
  source (`now_ms`) and the board-specific `run()` entry.
- `config.toml`: network + zenoh locator. Pick a port that doesn't clash
  with the FreeRTOS (7451) and NuttX (7452) defaults — the next
  unallocated number is 7453.
- `build.rs`: only needed if XYZ requires linker-script preprocessing or
  custom lib search paths. Skip it on flat Linux-style targets.
- `.cargo/config.toml`: target triple, linker, and `[patch.crates-io]`
  block matching the existing per-crate configs (nros + msg patches).
- `rust-toolchain.toml`: only if XYZ is Tier-3 and needs `-Z build-std`.

Add `src/autoware_sentinel_xyz` to the workspace `exclude = […]` list
in the root `Cargo.toml` if it cross-compiles.

### 3. Add justfile recipes

Mirror the FreeRTOS/NuttX pattern in `justfile`:

```just
xyz_env := "XYZ_VAR=value NROS_EXECUTOR_MAX_CBS=N …"

build-sentinel-xyz:
    cd src/autoware_sentinel_xyz
    {{ xyz_env }} cargo build --release

run-sentinel-xyz: build-sentinel-xyz
    # invoke the XYZ runner (qemu-system-arm, west run, …)
```

Add `build-sentinel-xyz` to the root `build` recipe so `just build`
covers it.

### 4. Bump the cross-check

Append a `cargo check -p autoware_sentinel_core --no-default-features
--features platform-xyz --target <xyz-triple>` line to the `cross-check`
recipe so a regression in the platform feature lands in CI.

### 5. Add an integration test

Create `tests/src/fixtures/sentinel_xyz.rs` (see `sentinel_freertos.rs`
or `sentinel_nuttx.rs` as templates). Wire it up via:

- `tests/src/fixtures/mod.rs` — add `mod sentinel_xyz; pub use sentinel_xyz::*;`
- `tests/Cargo.toml` — register the new `[[test]]` entry.
- `.config/nextest.toml` — add a `[test-groups.xyz-qemu]` (or similar)
  with `max-threads = 1` and a `[[profile.default.overrides]]` block.

Add at minimum a `test_xyz_sentinel_executor_ready` test that asserts on
the `Executor ready — spinning…` line. That single assertion covers the
whole pipeline: network up → zpico session opened → all
publishers/services declared → executor armed.

### 6. Document the new platform in this guide

Update the Architecture diagram and the per-platform crate-contents
table at the top of this file.

## Capacity tuning quick reference

If the platform binary boots through `main()` and prints `Locator: …`
but never reaches `Executor ready`, you are probably hitting the same
stack-frame issue Phase 13.K1.7 identified on FreeRTOS:

> `nros::Executor` carries an inline `arena: [MaybeUninit<u8>;
> ARENA_SIZE]`; `ARENA_SIZE = MAX_CBS × (RX_BUF × 3 + 512) + 2048`. The
> `Result<Executor, _>` return defeats NRVO, so LLVM emits ~3 stack
> copies. With `.env`'s `NROS_EXECUTOR_MAX_CBS=96` (ARENA ≈ 346 KB) the
> closure stack frame inflates to ~1 MB.

Fix is per-platform via the justfile env knobs:

| Knob | What it shrinks |
|------|-----------------|
| `NROS_EXECUTOR_MAX_CBS` | callback slots (lower → smaller arena → smaller frame) |
| `NROS_SUBSCRIPTION_BUFFER_SIZE` | per-callback RX buffer |
| `NROS_PARAM_SERVICE_BUFFER_SIZE` | param-service request/reply (boxed, no stack impact) |
| `ZPICO_MAX_PUBLISHERS` / `ZPICO_MAX_SUBSCRIBERS` / `ZPICO_MAX_QUERYABLES` / `ZPICO_MAX_LIVELINESS` | static zpico tables (in `.bss`) |

Pick `MAX_CBS` ≥ (subs + services + timers + headroom). For `comp-all`
the topology is 5 subs + 22 services + 1 timer = 28 callbacks; 32 is the
default sweet spot on FreeRTOS / NuttX.
