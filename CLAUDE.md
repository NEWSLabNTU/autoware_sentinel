# Autoware Sentinel

An independent safety island for autonomous vehicles, porting selected Autoware packages to
[nano-ros](../nano-ros/), a ROS 2 runtime focused on formal verification and real-time safety
for embedded systems. Runs on a dedicated safety MCU to bring the vehicle to a safe stop
when the main compute fails.

**Repository:** https://github.com/jerry73204/autoware_sentinel.git

## Project Structure

All algorithm crates are organized in a Cargo workspace (`Cargo.toml` at root). Message
crates are generated in `src/autoware_sentinel_linux/generated/` and shared via root
`[patch.crates-io]`. nano-ros is referenced as a git dependency (pinned rev).
`src/autoware_sentinel/` (Zephyr), `src/verification/` (Verus), and `tests/` are
excluded from the workspace.

```
autoware-nano-ros/
├── src/
│   ├── autoware_stop_filter/        # Phase 1 — velocity stop filter
│   │   ├── Cargo.toml
│   │   ├── package.xml              # ROS message dependencies
│   │   ├── .gitignore
│   │   └── src/lib.rs
│   ├── autoware_vehicle_velocity_converter/  # Phase 1 — VelocityReport→Twist
│   ├── autoware_shift_decider/      # Phase 1 — gear state machine
│   ├── autoware_mrm_emergency_stop_operator/  # Phase 2 — jerk-limited hard braking
│   ├── autoware_mrm_comfortable_stop_operator/ # Phase 2 — gentle deceleration
│   ├── autoware_heartbeat_watchdog/ # Phase 2 — main compute heartbeat monitor
│   ├── autoware_mrm_handler/        # Phase 2 — MRM orchestrator state machine
│   ├── autoware_vehicle_cmd_gate/   # Phase 3 — rate limiting + source arbitration
│   ├── autoware_twist2accel/        # Phase 4 — velocity→acceleration estimator
│   ├── autoware_control_validator/  # Phase 4 — command safety validation
│   ├── autoware_operation_mode_transition_manager/ # Phase 4 — mode transitions
│   ├── verification/                # Phase 5 — Verus formal proofs
│   ├── autoware_sentinel_core/      # Phase 13 — shared wiring (no_std)
│   ├── autoware_sentinel_zephyr/    # Phase 6 — Zephyr application
│   └── autoware_sentinel_linux/     # Phase 7.1 — Linux native binary
├── tests/                           # Phase 7.2–7.3 — Integration tests (nextest)
│   ├── Cargo.toml                   # sentinel-tests crate
│   ├── src/
│   │   ├── lib.rs                   # TestError, helpers (count_pattern, wait_for_port)
│   │   ├── autoware.rs              # Autoware helpers (map path, poses, prerequisites)
│   │   ├── process.rs               # ManagedProcess, process group cleanup
│   │   ├── ros2.rs                  # Ros2Process, topic_pub_once, service_call, etc.
│   │   └── fixtures/                # rstest fixtures
│   │       ├── mod.rs
│   │       ├── autoware_launcher.rs # AutowareLauncher (dump, filter, replay)
│   │       ├── zenohd_router.rs     # ZenohRouter (ephemeral port allocation)
│   │       └── sentinel.rs          # sentinel_binary(), start_sentinel()
│   └── tests/
│       ├── transport_smoke.rs       # Sentinel ↔ ROS 2 transport tests (9 tests)
│       └── planning_simulator.rs    # Autoware planning simulator tests (6 tests)
├── .config/
│   ├── nextest.toml                 # nextest profiles, test group serialization
│   ├── zenoh_session.json5          # rmw_zenoh_cpp client config (mode=client → localhost:7447)
│   └── zenoh_router.json5           # zenohd router config (listen localhost:7447)
├── docs/
│   ├── roadmap/                     # Phase docs (1–7)
│   ├── guides/                      # Developer guides
│   │   └── zephyr-setup.md         # Zephyr workspace setup guide
│   └── research/                    # Architecture research
├── scripts/
│   ├── activate_autoware.sh         # Source Autoware ROS 2 environment
│   └── zephyr/
│       └── setup.sh                 # Zephyr workspace initialization
├── Cargo.toml                       # Workspace root (members + [patch.crates-io])
├── Cargo.lock                       # Shared lockfile for all workspace members
├── west.yml                         # Zephyr west manifest
├── .env                             # Build-time env vars (loaded by justfile)
├── .envrc                           # direnv: sources ROS 2 + rmw_zenoh_cpp env
├── justfile                         # Root convenience recipes
├── autoware-repo -> ~/repos/autoware/1.5.0-ws  # Autoware source (symlink)
└── CLAUDE.md                        # This file
```

## Build Commands

Workspace builds from root:

```bash
cargo build --workspace        # build all algorithm crates + sentinel
cargo test --workspace         # run all unit tests
cargo check --workspace --exclude autoware_sentinel_linux --target thumbv7em-none-eabihf  # cross-check
```

Root justfile for convenience:

```bash
# Build
just build                # build all packages + Zephyr + Linux sentinel
just build-sentinel-linux # build Linux sentinel binary only
just build-zephyr         # build Zephyr application (native_sim)
just generate-bindings    # regenerate message crates (sentinel_linux is the superset)

# Run (each in a separate terminal)
just run-zenohd           # zenohd router on localhost:7447
just run-autoware         # full baseline Autoware via play_launch
just run-autoware-filtered # filtered Autoware (sentinel replaces 14 nodes)
just run-sentinel         # Linux sentinel binary (unsets CONFIG_URI env vars)
just run-auto-drive       # autonomous drive controller (init pose → route → engage → drive)
just ros2-daemon-restart  # restart ros2 daemon with rmw_zenoh_cpp

# All-in-one (bundles zenohd + play_launch + sentinel + optional auto_drive)
just launch-autoware-sentinel --drive --timeout 300

# Test
just test                 # cargo test --workspace
just test-transport       # transport smoke tests (nextest)
just cross-check          # cross-compile check (excludes sentinel_linux)
just ci                   # format-check + cross-check + test

# Verification
just verify-kani          # Kani harnesses
just verify-verus         # Verus proofs
```

All algorithm crates must be `#![no_std]` and cross-compile to `thumbv7em-none-eabihf`.

## Build-Time Environment Variables (`.env`)

The root `.env` file configures compile-time capacity limits for zpico and nros. The
justfile loads it automatically via `set dotenv-load`. When building manually (e.g.,
`cargo build` in `src/autoware_sentinel_linux/`), either source the `.env` or pass the
vars explicitly.

| Variable | Default | Sentinel Value | Purpose |
|----------|---------|----------------|---------|
| `ZPICO_MAX_PUBLISHERS` | 8 | 40 | Max zenoh-pico publishers |
| `ZPICO_MAX_SUBSCRIBERS` | 8 | 16 | Max zenoh-pico subscribers |
| `ZPICO_MAX_LIVELINESS` | 8 | 64 | Max liveliness tokens |
| `NROS_MAX_PARAMETERS` | 32 | 64 | Max ROS 2 parameters |
| `NROS_EXECUTOR_MAX_CBS` | 4 | 64 | Max executor callback slots |
| `NROS_PARAM_SERVICE_BUFFER_SIZE` | 4096 | 8192 | CDR buffer for each param service (req + reply). 8192 needed because `describe_parameters` with 62 params produces ≈ 4.5 KiB replies. |

The test fixture (`tests/src/fixtures/sentinel.rs`) sets these same values when building
the sentinel binary during integration tests. Keep them in sync.

## Message Generation

**Never hand-write message types.** Each package has a `package.xml` declaring message
dependencies and a `justfile` with a `generate` recipe.

```bash
cd src/autoware_stop_filter
just generate        # runs: cargo nano-ros generate --force
```

The generate command:
1. Reads `package.xml` for `<depend>` entries
2. Resolves transitive dependencies from the ROS 2 ament index
3. Generates Rust crates into `generated/`
4. Creates `.cargo/config.toml` with `[patch.crates-io]` entries

### Prerequisites

The Autoware environment (ROS 2 + Autoware message packages) must be available.
Edit `scripts/activate_autoware.sh` to source the correct setup script for your system.
The root `just generate-bindings` recipe sources this script automatically.

### Adding new message dependencies

1. Add `<depend>pkg_name</depend>` to the package's `package.xml`
2. Source the ROS 2 environment (and ament prefix for Autoware msgs)
3. Regenerate: `just generate` in the package directory

### ROS 2 type names and the `nros_` prefix

Generated crate names are derived from the ROS 2 package name. For a package `foo_msgs`,
the generated Rust crate is `foo_msgs` and the DDS type strings are
`foo_msgs::msg::dds_::Foo_`. This matches what rmw_zenoh_cpp publishes in service/topic
discovery tokens, so ROS 2 tools can find the services.

**Important:** nano-ros has an internal crate `nros-rcl-interfaces`
(`~/repos/nano-ros/packages/interfaces/rcl-interfaces/`) whose Rust crate name is
`nros_rcl_interfaces`. Its `TYPE_NAME`/`SERVICE_NAME` constants now correctly use the
`rcl_interfaces::srv::dds_::*` prefix (matching what rmw_zenoh_cpp expects), so
`executor.register_parameter_services()` works for ROS 2 interop without any
local code generation. The API derives the node FQN from the executor's namespace + node_name
(e.g. node_name `"sentinel"` → services at `/sentinel/list_parameters`).
The types are `pub(crate)` — not exposed in the public nros API.

### Post-generation fix: `[f64; 36]` Default

Generated `geometry_msgs` has types with `[f64; 36]` covariance arrays that fail to compile
because `Default` is not implemented for `[T; N]` where N > 32. After generation, run:

```bash
python3 tmp/fix_covariance_default.py src/autoware_*/generated/geometry_msgs src/autoware_*/generated/geographic_msgs
```

Generated crates are `#![no_std]`, use `heapless::String<N>` / `heapless::Vec<T, N>`,
and implement `Serialize`, `Deserialize`, `RosMessage` traits.

## Porting Conventions

### Two-layer architecture

Each ported package follows two layers:

1. **Algorithm library** (`src/autoware_*/src/lib.rs`):
   - Pure `#![no_std]` logic operating on message types
   - No ROS/nros dependencies — only message crates
   - Fully testable with `cargo test` (no network, no executor)
   - Struct with `new()` constructor and methods like `apply()`, `convert()`, `decide()`

2. **Node binary** (future, when wiring is needed):
   - Uses `nros` high-level API (`Executor`, `Node`, publishers, subscribers)
   - Wires callbacks to algorithm instances
   - Depends on `nros = { ..., features = ["std", "rmw-zenoh", "platform-posix"] }`

### Algorithm crate Cargo.toml pattern

```toml
[package]
name = "autoware_stop_filter"
version = "0.1.0"
edition = "2024"
license = "Apache-2.0"
publish = false

[dependencies]
nav_msgs = { version = "*", default-features = false }
geometry_msgs = { version = "*", default-features = false }

[features]
default = []
std = ["nav_msgs/std", "geometry_msgs/std"]
```

Message deps use `version = "*"` — resolved via `[patch.crates-io]` in the root
workspace `Cargo.toml` pointing at `src/autoware_sentinel_linux/generated/` crates.

### nros node wiring pattern

When creating ROS node binaries (Phase 2+), use the high-level API:

```rust
use nros::prelude::*;
use nav_msgs::msg::Odometry;

let config = ExecutorConfig::from_env().node_name("my_node");
let mut executor = Executor::<_, 8, 8192>::open(&config)?;
let mut node = executor.create_node("my_node")?;

// Create typed publisher
let publisher = node.create_publisher::<Odometry>("/output/odom")?;

// Register subscription callback on executor
executor.add_subscription::<Odometry, _>("/input/odom", move |msg| {
    // process msg, publish result
    publisher.publish(&filtered_msg).ok();
})?;

// Service server (callback-based)
executor.add_service::<OperateMrm, _>("/operate", |request| {
    OperateMrmResponse { /* ... */ }
})?;

// Service client (promise-based)
let mut client = node.create_client::<OperateMrm>("/operate")?;
let mut promise = client.call(&request)?;
let response = promise.wait(&mut executor, 5000)?;

// Timer (periodic)
executor.add_timer(33, move || { /* 30 Hz */ })?;

// Spin
executor.spin_blocking(SpinOptions::default())?;
```

**Do NOT use low-level RMW traits** (`Session`, `Publisher::publish_raw`, etc.) in
application code. Always use the `Executor`/`Node` layer.

## Known Issues

- **`[f64; 36]` Default**: `Default` is not implemented for `[T; N]` where N > 32. Types
  like `PoseWithCovariance` and `GeoPoseWithCovariance` need manual `Default` impls.
  Fix with `tmp/fix_covariance_default.py` (scans geometry_msgs and geographic_msgs).
- **f32→f64 precision**: `VelocityReport` fields are `f32`. In tests, compare against
  `value_f32 as f64`, not `value_f64`.
- **Generated crate edition**: Message crates use edition 2021; algorithm crates use 2024.
- **Constants in generated msgs**: Constants defined in `.msg` files are private in generated
  modules — define your own constants in application crates.
- **`add_service` reply buffer too small for large responses**: `executor.add_service` uses
  `DEFAULT_RX_BUF_SIZE = 1024` bytes for the reply buffer. Services that return large responses
  must use `executor.add_service_sized::<Svc, _, REQ_BUF, REPLY_BUF>()` with explicit sizes.
  Failure mode: callback fires, serialization silently fails, zenohd times out after 30s, client
  sees "future timed out". For parameter services specifically, use
  `executor.register_parameter_services()` which applies `NROS_PARAM_SERVICE_BUFFER_SIZE`
  (default 4096, sentinel sets 8192 via `.env` to handle `describe_parameters` with 62 params
  ≈ 4.5 KiB response).
- **LLVM SIGSEGV on cross-compile**: `autoware_adapi_v1_msgs` triggers an LLVM crash in the
  ARM register scavenger (`scavengeFrameVirtualRegs`) when building at `opt-level=0` for
  `thumbv7em-none-eabihf`. Workaround: add `[profile.dev.package.autoware_adapi_v1_msgs]
  opt-level = 1` in any crate that depends on it. See [LLVM #64277](https://github.com/llvm/llvm-project/issues/64277).

## Zenoh-pico Drive Latency

Full investigation: `docs/research/zenoh-pico-drive-latency.md`

The sentinel drives 1.8× slower than baseline Autoware (46s vs 26s). Two fixes applied:

1. **Removed redundant sleep from `spin_blocking`** (nano-ros) — the executor slept 10ms
   unconditionally after each `spin_once`, causing subscription data loss. Fix: removed
   the sleep (spin_once already provides wait semantics). Impact: 79-134s → 47-50s.

2. **Increased staleness threshold to 2000ms** — at 500ms, brief zenoh-pico subscription
   dropouts triggered the staleness guard, causing the Autoware controller to over-correct
   with hard braking. At 2000ms, brief dropouts pass through unmodified. Impact: eliminated
   20s mid-route stop.

**Remaining 1.8× gap**: zenohd fails to propagate zenoh-pico subscription interests to
rmw_zenoh_cpp publishers under load (222 sessions). With 1 publisher, data arrives instantly;
with full Autoware (222 sessions), external data may never arrive. This is a zenohd
scalability issue, not a zenoh-pico bug. The sentinel's successful runs are non-deterministic,
depending on session connection timing.

**Upstream fix**: zenoh [PR #2096 "Regions"](https://github.com/eclipse-zenoh/zenoh/pull/2096)
merged to `main` on 2026-04-01. It rewrites the interest routing system to fix scalability
with many sessions. Fixes issues [#2224](https://github.com/eclipse-zenoh/zenoh/issues/2224),
[#2233](https://github.com/eclipse-zenoh/zenoh/issues/2233),
[#2278](https://github.com/eclipse-zenoh/zenoh/issues/2278),
[#2283](https://github.com/eclipse-zenoh/zenoh/issues/2283).
**Not in zenoh 1.8.0** — will be in a future release (breaking change).

**Wireshark dissector** installed at `~/.local/lib/wireshark/plugins/4.6/epan/libzenoh_dissector.so`
for zenoh 1.7.2 protocol analysis. Requires `wireshark` group membership for capture.

## Autoware Source Reference

Autoware 1.5.0 workspace is symlinked at `autoware-repo` (points to
`~/repos/autoware/1.5.0-ws`). `autoware_common_msgs` lives in the 1.5.0-ws source,
not alongside `autoware_control_msgs`/`autoware_vehicle_msgs` in `autoware_msgs`.

## Formal Verification (Phase 5)

Kani harnesses live inline in production crates (`#[cfg(kani)] mod verification`).
Verus proofs live in a standalone crate (`src/verification/`).

### Kani harnesses (13 total)

- **stop_filter** (5): NaN→zero, output never NaN, panic-freedom, stopped⇒zeros, moving⇒passthrough
- **vehicle_velocity_converter** (4): covariance diagonal, f32→f64 sign, panic-freedom, output NaN
- **shift_decider** (3): valid gear, dead-zone hold, NaN velocity hold
- **emergency_stop_operator** (1): bounded convergence (`#[kani::unwind(301)]`)

Crates with Kani harnesses need `[lints.rust]` in `Cargo.toml`:

```toml
[lints.rust]
unexpected_cfgs = { level = "warn", check-cfg = ['cfg(kani)'] }
```

### Verus verification crate (`src/verification/`)

Standalone crate (not in workspace) using `vstd`. Integer-scaled ghost models avoid
floating-point reasoning. Modules: `ghost_types`, `shift_decider`, `emergency_stop`,
`mrm_handler`.

### NaN safety pattern

Use negated comparisons for NaN-safe thresholds:

```rust
// WRONG: NaN.abs() < threshold → false (NaN passes through)
linear.x.abs() < self.vx_threshold

// RIGHT: !(NaN.abs() >= threshold) → !(false) → true (NaN treated as stopped = safe)
!(linear.x.abs() >= self.vx_threshold)
```

## Development Phases

| Phase | Focus | Status |
|-------|-------|--------|
| 1 | Foundation (messages + 3 trivial ports) | Complete |
| 2 | Emergency Response (MRM chain) | Complete |
| 3 | Safety Gate (vehicle command gate) | Complete |
| 4 | Validation Layer (control validator, twist2accel) | Complete |
| 5 | Formal Verification (Kani + Verus) | Complete |
| 6 | Zephyr Application (single binary) | In progress |
| 7 | Integration Testing (Autoware planning simulator) | In progress (7.1–7.3 complete) |
| 8 | Topic Parity (match baseline Autoware topics) | Complete |
| 12 | Service & Topic Parity (services + parameter API) | Complete — see below |

See `docs/roadmap/` for detailed phase docs. Roadmap docs use `- [ ]` / `- [x]` checkboxes
on subphase headers and acceptance criteria to track completion progress.

## Temporary Scripts

Place temporary Python/shell scripts in `tmp/` at the repo root. This directory is
gitignored and used for one-off data extraction, analysis, or debugging scripts.

## Zephyr Build (Phase 6)

The safety island deploys as a single Zephyr application. Setup:

```bash
./scripts/zephyr/setup.sh                         # one-time workspace init
source ../autoware-sentinel-workspace/env.sh       # set ZEPHYR_BASE, etc.
cd ../autoware-sentinel-workspace
west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel_zephyr
```

Key files:
- `west.yml` — West manifest (Zephyr v3.7.0, zephyr-lang-rust)
- `scripts/zephyr/setup.sh` — Downloads SDK, creates workspace, symlinks nano-ros
- `src/autoware_sentinel_zephyr/` — Zephyr application crate (Phase 6.2+)

nano-ros is registered as a Zephyr module via `ZEPHYR_EXTRA_MODULES` in `env.sh`.
See `docs/guides/zephyr-setup.md` for full instructions.

## Integration Testing (Phase 7)

Integration tests live in `tests/` as a standalone crate (`sentinel-tests`) run with
[nextest](https://nexte.st/). Tests verify bidirectional message flow between the sentinel
Linux binary and ROS 2 (`rmw_zenoh_cpp`) through a shared zenohd router.

### Running tests

```bash
just test-integration   # all integration tests
just test-transport     # transport smoke tests only
just test-planning      # planning simulator tests only
just dump-autoware            # dump planning simulator to record.json
just launch-autoware-baseline # replay unmodified Autoware
just launch-autoware-sentinel # filtered Autoware + zenohd + sentinel
cd tests && cargo nextest run -E 'test(test_sentinel_starts)'  # single test
```

### Test infrastructure

- **`tests/src/process.rs`** — `ManagedProcess` with RAII process group cleanup
  (`setpgid` + `PR_SET_PDEATHSIG(SIGKILL)`). Never leaves orphan processes.
- **`tests/src/ros2.rs`** — `Ros2Process` wraps `ros2 topic pub/echo` with
  `rmw_zenoh_cpp` transport and Autoware message env.
- **`tests/src/fixtures/`** — rstest fixtures:
  - `zenohd_unique` — starts zenohd on an OS-assigned ephemeral port (parallel-safe)
  - `sentinel_binary` — builds the Linux sentinel once per test run (`OnceCell`)
  - `start_sentinel()` — spawns sentinel as `ManagedProcess`, waits for "Executor ready"

### nextest configuration (`.config/nextest.toml`)

Transport smoke tests run sequentially (`max-threads = 1`) to avoid port/resource contention.
Slow timeout is 60s (these tests involve multi-process coordination with sleep waits).

Planning simulator tests also run sequentially (`max-threads = 1`) with 120s slow timeout
(Autoware startup is ~60–120s). Tests skip gracefully if prerequisites are missing.

### Prerequisites

- zenohd built locally (`~/repos/nano-ros/build/zenohd/zenohd`)
- ROS 2 Humble with `rmw_zenoh_cpp` installed
- Autoware message packages (`/opt/autoware/1.5.0/`)
- Planning simulator tests additionally require:
  - `play_launch` installed (`~/.local/bin/play_launch`, v0.6.0+)
  - Autoware map data at `$MAP_PATH` or `$HOME/autoware_map/sample-map-planning`

### Adding new integration tests

1. Create a new test file in `tests/tests/`
2. Register it as `[[test]]` in `tests/Cargo.toml`
3. Use rstest fixtures from `sentinel_tests::fixtures`
4. If tests need serialization, add to a test group in `.config/nextest.toml`

## nano-ros Reference

The nano-ros repo is at `~/repos/nano-ros/`. Key references:
- `packages/core/nros/` — unified facade crate (use this as dependency)
- `packages/core/nros-node/` — Executor, Node, callback infrastructure
- `packages/core/nros-core/` — foundation types, re-exports `heapless`
- `packages/core/nros-serdes/` — CDR serialization
- `examples/native/rust/zenoh/` — reference examples for node patterns
- `docs/guides/message-generation.md` — codegen guide

## Phase 12: Service & Topic Parity

Full roadmap: `docs/roadmap/12-service-topic-parity.md`

### Completed (12.1 – 12.8 + 12.9 parameter services)

- **12.1** — 3 new publishers: `is_paused`, `is_start_requested`, `current_gate_mode`
- **12.2** — 4 new services: `engage`, `emergency`, `external_emergency_stop`, `clear_external_emergency_stop`
- **12.3** — 1 new publisher (`is_autonomous_available`), 1 new service (`control_mode_request`)
- **12.4** — 5 ADAPI operation mode services: `change_to_stop/local/remote`, `enable/disable_autoware_control`
- **12.5** — 1 new publisher: `emergency_holding`
- **12.6 – 12.8** — message generation, capacity updates, integration tests (all 14 transport smoke tests pass)
- **12.9** — 6 ROS 2 parameter services (`list/get/get_types/describe/set/set_atomically`) via
  `executor.register_parameter_services()`. The API derives the node FQN from the executor's
  namespace + node_name (following the ROS 2 convention: `/{ns}/{node}/list_parameters`).
  Fixed `nros_rcl_interfaces` DDS type strings to use `rcl_interfaces::` prefix (was
  `nros_rcl_interfaces::` — invisible to rmw_zenoh_cpp). `nros_rcl_interfaces` types are now
  `pub(crate)`; users call the high-level API only. `NROS_PARAM_SERVICE_BUFFER_SIZE=8192`
  set in `.env` and test fixture.

### Status: Complete

All acceptance criteria verified:
- `ros2 service list` shows all 11 Phase 12 functional services + 6 parameter services
- `just cross-check` passes — all algorithm crates cross-compile to `thumbv7em-none-eabihf`
- Live comparison (baseline 571 topics / 742 services vs sentinel 572 / 664):
  - 0 missing topics (full parity, +1 extra: `/control/is_autonomous_available`)
  - 84 missing services — all per-node parameter services (by design: sentinel is one node)
  - 0 missing functional services
  - Full analysis in `docs/roadmap/12-service-topic-parity.md`

**Important:** The sentinel must NOT inherit `ZENOH_SESSION_CONFIG_URI` or `ZENOH_ROUTER_CONFIG_URI`
from the shell. Those env vars interfere with zenoh-pico's liveliness token setup and prevent
service/node discovery by rmw_zenoh_cpp. The `just launch-autoware-sentinel` recipe handles this
correctly (it starts the sentinel as a subprocess without those vars). When testing manually, unset
them first: `env ZENOH_SESSION_CONFIG_URI= ZENOH_ROUTER_CONFIG_URI= sentinel`
