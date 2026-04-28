# Phase 11: Orin SPE Deployment

**Status:** Not started
**Depends on:** Phase 7 (integration testing), Phase 10 (actuation porting),
**nano-ros Phase 100** (Orin SPE infrastructure — Cortex-R5 platform layer, `Z_FEATURE_LINK_IVC` in zenoh-pico, `nros-board-orin-spe`).
**Goal:** Run the Autoware Sentinel safety island on the Jetson AGX Orin SPE (Cortex-R5F,
FreeRTOS), communicating with Autoware on the CCPLEX via IVC shared memory through a
zenohd router.

## Split with nano-ros (read first)

The platform/infra subset of this phase is **owned by `nano-ros` Phase 100** (see
`docs/roadmap/phase-100-orin-spe-infra.md` in that repo). The driver/platform/board
stack lands as four independent crates with strict layering:

```
nvidia-ivc                 (packages/drivers/nvidia-ivc)
  ├─ feature `fsp`          → tegra_ivc_channel_* (NVIDIA FSP)
  └─ feature `unix-mock`    → Unix-socket pair (dev/CI)

nros-platform-api::PlatformIvc                                  ← contract
  └─ implemented by ───────► nros-platform-orin-spe (platforms/) ── delegates to nvidia-ivc

zpico-platform-shim::ivc_helpers (cargo feature `ivc`)
  └─ exports _z_open_ivc / _z_read_ivc / … → <P as PlatformIvc>

zenoh-pico Z_FEATURE_LINK_IVC (vendored C, link/unicast/ivc.c)
  └─ calls the shim forwarders

nros-board-orin-spe                                             ← packages/boards/
  └─ Config { zenoh_locator: "ivc/2", … }, run<F>, FSP println
```

Phase 100 sub-items deliver:

- **100.0** — `packages/drivers/nvidia-ivc` driver crate (HAL only, no
  `nros-platform`/`nros-rmw`/zenoh-pico deps). Two backends behind cargo features:
  `fsp` (real NVIDIA FSP, no_std) and `unix-mock` (Unix-domain-socket pair, std,
  Linux-only). Reusable in this repo's `src/ivc-bridge/` too.
- **100.0a** — `PlatformIvc` trait in `nros-platform-api`.
- **100.1** — Cortex-R5 critical-section support in `nros-platform-freertos`.
- **100.2** — `armv7r-none-eabihf` workspace toolchain wiring.
- **100.3** — `zpico-platform-shim` Cortex-R5 build + `ivc` feature for the
  `_z_open_ivc` / `_z_read_ivc` / … forwarders.
- **100.4** — `Z_FEATURE_LINK_IVC` link transport in vendored zenoh-pico.
- **100.5** — `nros-platform-orin-spe` (impl PlatformIvc + Clock + Sleep + Alloc +
  Threading + Random).
- **100.6** — `nros-board-orin-spe` board crate (Config, run, FSP println, links
  nano-ros into the SPE firmware via `ENABLE_NROS_APP := 1`).
- **100.7** — `just orin_spe` recipe set.
- **100.8** — POSIX-mock end-to-end smoke test in nano-ros CI.

This phase (`autoware_sentinel` Phase 11) consumes those pieces and adds:

- The reduced sentinel algorithm set selection that fits the 256 KB BTCM budget
  (11.3 / 11.5).
- The Linux-side **IVC bridge daemon** in `src/ivc-bridge/` (11.2 mock, 11.6 real
  hardware). May also depend on `nano-ros::nvidia-ivc` (with `unix-mock` for test
  builds, plain sysfs `read(2)`/`write(2)` in production) for a single Rust API
  both sides of the wire.
- Hardware verification on the AGX Orin DevKit (11.4, 11.7).
- Production deployment (firmware flashing, capsule update, systemd service).
- Float-ABI mismatch resolution at the application FFI boundary.

Before starting any 11.x sub-item, confirm the matching nano-ros 100.x is at least
in-progress. The dependency direction is strict:

| Sentinel sub-item | nano-ros prerequisites |
|---|---|
| 11.1p (FreeRTOS POSIX setup) | 100.1, 100.2, 100.3 |
| 11.2 (mock IVC transport)    | 100.0 (`unix-mock`), 100.0a, 100.4, 100.5 |
| 11.3 (POSIX sentinel + tests)| 100.7, 100.8 |
| 11.4 (real IVC echo)         | none (hardware-only) |
| 11.5 (board + cross-compile) | 100.0 (`fsp`), 100.5, 100.6 |
| 11.6 (real bridge daemon)    | 100.0 (CCPLEX-side use of the driver) |
| 11.7 (flash + integration)   | all above |

## Description

The sentinel currently runs on Linux (Phase 7) and Zephyr native_sim (Phase 6). This
phase targets the real safety island hardware: the Always-On (AON) Cortex-R5F core on
the Jetson AGX Orin SoC. The SPE runs NVIDIA's FreeRTOS V10.4.3 FSP with 256 KB BTCM.

The primary challenge is transport: the SPE has no Ethernet or dedicated serial port.
The only UART is the TCU (Tegra Combined UART), a shared debug multiplexer for all 8
SoC processors — unsuitable for data transport. **IVC (Inter-VM Communication)** is the
only viable SPE↔Linux transport: shared-memory ring buffers in a DRAM carveout mapped
into SPE address space via AST, with HSP (Hardware Synchronization Primitives) doorbell
signaling.

### Key constraints

- **256 KB BTCM** for code + data + heap + stacks (FreeRTOS ~40 KB, zenoh-pico ~60-80 KB,
  nros ~15 KB, algorithms ~30 KB, messages ~40 KB, stacks ~30 KB ≈ 215–235 KB)
- **IVC frame size**: 16 frames × 64 bytes per channel (configurable in `ivc-config.h`)
- **Float ABI mismatch**: BSP C code uses `-mfloat-abi=softfp`, Rust `eabihf` uses hard
  float — must align at link time
- **No trajectory follower**: MPC controller is too large for 256 KB; sentinel must use
  `has_external_control` mode, receiving `/control/command/control_cmd` from Autoware's
  controller running on CCPLEX
- **IVC on Orin**: L4T 36.4 BSP includes AGX Orin IVC infrastructure (echo channel demo),
  but NVIDIA's earlier forums noted IVC was "verified only on AGX Xavier." L4T 36.4 adds
  Orin support — must verify on actual hardware first

### Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  CCPLEX (Cortex-A78AE, Linux)                                │
│                                                              │
│  Autoware ──rmw_zenoh_cpp──► zenohd ◄──tcp──► IVC bridge    │
│                                 ▲               daemon       │
│                                 │                 │          │
│                              tcp:7447         /dev/aon_echo  │
│                                               (sysfs IVC)    │
└──────────────────────────────────────────────────────────────┘
                    │ DRAM carveout (shared memory) │
                    │     HSP doorbell signaling    │
┌──────────────────────────────────────────────────────────────┐
│  SPE (Cortex-R5F, FreeRTOS)                                  │
│                                                              │
│  nros Executor ──zenoh-pico──► IVC link backend              │
│    sentinel algorithms          (tegra_ivc_channel API)      │
│    (no_std, reduced set)                                     │
└──────────────────────────────────────────────────────────────┘
```

## Development Strategy

Development proceeds in two stages: first validate all sentinel logic on the **FreeRTOS
POSIX simulator** running as a Linux process on the Orin itself, then migrate to real SPE
hardware once the binary is fully tested.

### Stage 1: FreeRTOS POSIX simulator (subphases 11.1p–11.3p)

The FreeRTOS kernel includes an official POSIX port (`portable/ThirdParty/GCC/Posix/`) that
uses pthreads to simulate FreeRTOS tasks and a timer thread for the tick interrupt. This
lets us compile and run the sentinel application natively on the Orin without any emulator
or cross-compilation.

**Advantages:**
- Fastest edit-compile-test cycle (native compilation, no flashing)
- Can connect to the real zenohd on localhost via mocked IVC (Unix domain sockets or
  `shm_open`) and run Autoware planning simulator integration tests
- Full access to GDB, valgrind, AddressSanitizer
- FreeRTOS task scheduling, queues, semaphores, and timers all work correctly

**Limitations:**
- Runs on AArch64 (not ARMv7-R) — cannot catch ABI or ISA-specific issues
- No MPU, no real-time timing guarantees
- Cannot validate 256 KB BTCM memory budget
- Known segfault issue on ARM64 Ubuntu with the POSIX port (signal handling / stack
  alignment) — may need investigation

**What to validate in this stage:**
- All sentinel algorithms (heartbeat watchdog, MRM emergency stop, vehicle command gate)
- FreeRTOS task structure and scheduling
- zenoh-pico IVC link backend (with mock IVC transport)
- IVC bridge daemon (Linux side)
- End-to-end Autoware integration tests through zenohd

### Stage 2: Real SPE hardware (subphases 11.1–11.7)

Once the sentinel is fully tested on the POSIX port, migrate to the actual Cortex-R5F:
- Cross-compile for `armv7r-none-eabihf`
- Replace mock IVC with real `tegra_ivc_channel_*` API
- Validate code fits in 256 KB BTCM
- Flash and test on AGX Orin hardware
- Resolve float ABI mismatch (softfp vs hard float)

### Alternative simulation options considered

| Approach | Verdict |
|----------|---------|
| QEMU Cortex-R5 (`xlnx-zcu102`) | Good for validating actual `armv7r` binaries; use as secondary check before flashing |
| Renode | Multi-core SoC simulation (A53+R5); useful later for IVC integration testing |
| NVIDIA SPE simulator | Does not exist |
| ARM FVP Cortex-R5 | Requires ARM DS license; not worth the cost |
| Xen/FreeRTOS on Orin | Wrong architecture (AArch64 vs ARMv7-R); impractical |

## Subphases

### Stage 1: FreeRTOS POSIX Simulator

#### - [ ] 11.1 — FreeRTOS POSIX Port Setup

Set up the FreeRTOS POSIX port to run FreeRTOS as a native Linux process on the Orin.

**Tasks:**
- [ ] Clone FreeRTOS kernel with POSIX port (`portable/ThirdParty/GCC/Posix/`)
- [ ] Build and run the `FreeRTOS/Demo/Posix_GCC/` demo on the Orin (aarch64)
- [ ] Investigate and fix the known ARM64 POSIX port segfault issue (signal handling /
  stack alignment) if it manifests on L4T Ubuntu
- [ ] Create `src/autoware_sentinel_spe/` application crate with FreeRTOS POSIX as a
  build option (feature flag or conditional compilation)
- [ ] Verify FreeRTOS task creation, queues, semaphores, and timers work correctly
- [ ] Add `just build-spe-sim` recipe to root justfile

**Acceptance criteria:**
- [ ] FreeRTOS POSIX demo runs on Orin aarch64 without crashes
- [ ] Sentinel crate compiles and runs as a FreeRTOS POSIX process

#### - [ ] 11.2 — Mock IVC Transport

Implement a mock IVC transport layer that uses Unix domain sockets (or `shm_open`) to
simulate IVC communication between the sentinel process and a bridge process on localhost.

**Tasks:**
- [ ] Define IVC transport trait/API abstracting `tegra_ivc_channel_read/write` so that
  the real IVC and mock IVC share the same interface
- [ ] Implement mock IVC backend using Unix domain sockets (bidirectional, frame-oriented)
- [ ] Implement mock IVC bridge: a small daemon that reads mock IVC frames from the Unix
  socket and forwards them to zenohd via TCP on `localhost:7447`, and vice versa
- [ ] Handle frame fragmentation: zenoh messages may exceed 64-byte IVC frame size —
  implement simple length-prefixed reassembly (same protocol as real IVC bridge)
- [ ] Add zenoh-pico IVC link backend (`Z_FEATURE_LINK_IVC`) in nano-ros, with the
  transport trait dispatching to mock or real IVC at compile time
- [ ] Add `just run-ivc-bridge-sim` recipe

**Files to modify in nano-ros:**
1. `packages/zpico/zpico-sys/zenoh-pico/src/link/unicast/ivc.c` (new)
2. `packages/zpico/zpico-sys/zenoh-pico/include/zenoh-pico/link/config/ivc.h` (new)
3. `packages/zpico/zpico-sys/zenoh-pico/src/link/link.c` — register IVC in link table
4. `packages/zpico/zpico-sys/zenoh-pico/src/link/endpoint.c` — parse `ivc/` scheme
5. `packages/zpico/zpico-sys/zenoh-pico/include/zenoh-pico/link/link.h` — IVC feature flag
6. `packages/zpico/zpico-sys/zenoh-pico/src/link/unicast/transport.c` — open/listen
7. `packages/zpico/zpico-sys/zenoh-pico/CMakeLists.txt` — conditional compile
8. `packages/zpico/zpico-sys/build.rs` — add IVC source to POSIX and FreeRTOS builds

**Acceptance criteria:**
- [ ] Mock IVC bridge forwards frames between Unix socket and zenohd TCP
- [ ] Fragmented zenoh messages reassembled correctly
- [ ] zenohd sees the simulated sentinel as a connected client
- [ ] IVC link compiles for both native (POSIX mock) and `armv7r-none-eabihf` (real IVC)

#### - [ ] 11.3 — Sentinel POSIX Application and Integration Tests

Wire the sentinel algorithms into the FreeRTOS POSIX application and run end-to-end
integration tests against the Autoware planning simulator.

**Tasks:**
- [ ] Determine minimum viable sentinel feature set:
  - Heartbeat watchdog (subscribe `/autoware/state`, publish MRM state)
  - MRM emergency stop operator (jerk-limited braking)
  - Vehicle command gate (pass-through or emergency override)
  - Drop: trajectory follower, debug topics, parameter services, control validator
- [ ] Wire reduced algorithm set with `Executor::<_, N, ARENA>` sized for SPE constraints
- [ ] Use `has_external_control = true` (receive control commands from CCPLEX)
- [ ] Start mock IVC bridge + zenohd + FreeRTOS POSIX sentinel as a test harness
- [ ] Verify sentinel topics visible via `ros2 topic list`
- [ ] Verify heartbeat watchdog triggers MRM on simulated failure
- [ ] Run Autoware planning simulator integration tests against the POSIX sentinel
- [ ] Add `just test-spe-sim` recipe

**Acceptance criteria:**
- [ ] Sentinel runs as FreeRTOS POSIX process with reduced algorithm set
- [ ] Heartbeat watchdog + emergency stop + gate functional end-to-end
- [ ] Planning simulator integration tests pass with the POSIX sentinel
- [ ] All sentinel logic validated before moving to real hardware

### Stage 2: Real SPE Hardware

#### - [ ] 11.4 — IVC Echo Verification on Hardware

Verify IVC communication works on the AGX Orin 64GB with L4T 36.4.4.

**Tasks:**
- [ ] Download and build SPE BSP with IVC echo enabled (`scripts/spe/download-bsp.sh`)
- [ ] Enable `aon_echo` in device tree (`status = "okay"` in DTB overlay)
- [ ] Flash SPE firmware with `ENABLE_IVC_ECHO := 1`
- [ ] Test bidirectional IVC echo from Linux:
  `echo "hello" > /sys/devices/platform/bus@0/bus@0:aon_echo/data_channel`
  and verify echo response
- [ ] Measure round-trip latency and maximum throughput (16×64B frames)
- [ ] Document device tree changes and any Orin-specific workarounds

**Acceptance criteria:**
- [ ] Bidirectional IVC echo works on AGX Orin with L4T 36.4.4
- [ ] Latency and throughput numbers recorded
- [ ] Device tree overlay documented

#### - [ ] 11.5 — nros-orin-spe Board Crate and Cross-compilation

Create the board support crate for nano-ros on the Orin SPE and set up the full
cross-compilation toolchain. The sentinel logic is already validated from Stage 1 —
this subphase focuses on making it build and link for the real Cortex-R5F target.

**Tasks:**
- [ ] Create `packages/boards/nros-orin-spe/` in nano-ros repo (following
  `nros-mps2-an385-freertos` pattern)
- [ ] Implement `run()` function: allocates FreeRTOS task via `xTaskCreate`, calls user
  closure inside task context, returns immediately (scheduler already running)
- [ ] Implement `Config` with `zenoh_locator: "ivc/2"` default
- [ ] Implement `println!` macro via NVIDIA FSP `printf` (TCU debug output)
- [ ] Wire real IVC init (`tegra_ivc_channel_get()`) — swap mock IVC backend for real
  `tegra_ivc_channel_*` API using the transport trait from 11.2
- [ ] Create `build.rs` to link against NVIDIA FSP static libraries
- [ ] Set target triple to `armv7r-none-eabihf` in `.cargo/config.toml`
- [ ] Resolve float ABI: either build BSP C code with `-mfloat-abi=hard` or use
  `armv7r-none-eabi` (soft float) for Rust and add shims
- [ ] Add `rustup target add armv7r-none-eabihf` to setup instructions
- [ ] Create `scripts/spe/build.sh` — builds Rust board crate, then invokes NVIDIA Makefile
- [ ] Add `ENABLE_NROS_APP := 1` to `target_specific.mk` template
- [ ] Add `nros-app.c` C shim (calls `nros_app_rust_entry()`)
- [ ] Modify NVIDIA Makefile to link `libnros_orin_spe.a` when `ENABLE_NROS_APP=1`
- [ ] Add linker script awareness for BTCM 256 KB limit — fail build if `.text + .data +
  .bss` exceeds budget
- [ ] Create `just build-spe` recipe in root justfile
- [ ] Verify linked binary fits in 256 KB with `arm-none-eabi-size`
- [ ] If 256 KB is exceeded, evaluate:
  - LTO + `opt-level = "z"` for size optimization
  - Drop additional algorithms
  - Move zenoh-pico to DRAM (if accessible from SPE via AST)

**Acceptance criteria:**
- [ ] Board crate compiles for `armv7r-none-eabihf`
- [ ] `just build-spe` produces `spe.bin` with nano-ros + sentinel linked in
- [ ] Binary size < 256 KB (with margin report)
- [ ] Float ABI consistent between Rust and C objects
- [ ] Memory budget documented with per-component breakdown

#### - [ ] 11.6 — Linux IVC Bridge Daemon (Real Hardware)

Adapt the mock IVC bridge from 11.2 to use real IVC sysfs/device interfaces.

**Tasks:**
- [ ] Update `src/ivc-bridge/` daemon to read/write IVC frames via sysfs:
  `/sys/devices/platform/bus@0/bus@0:aon_echo/data_channel` (or `/dev/tegra-ivc-*`)
- [ ] Verify the same frame protocol and fragmentation logic from the mock bridge works
  with real IVC frame sizes (16×64B)
- [ ] Add systemd service file for auto-start
- [ ] Add `just run-ivc-bridge` recipe

**Acceptance criteria:**
- [ ] Bridge daemon forwards real IVC↔TCP bidirectionally
- [ ] Fragmented zenoh messages reassembled correctly
- [ ] zenohd sees SPE sentinel as a connected client

#### - [ ] 11.7 — Integration Test and Flash Deployment

End-to-end test: flash SPE firmware, start bridge daemon, verify sentinel participates
in Autoware planning simulator. The sentinel logic is already proven from Stage 1 —
this subphase validates the real hardware transport and deployment procedure.

**Tasks:**
- [ ] Flash SPE firmware to AGX Orin (see "SPE Firmware Flashing" section below)
- [ ] Start IVC bridge daemon on Linux
- [ ] Start zenohd + Autoware planning simulator
- [ ] Verify SPE sentinel topics visible via `ros2 topic list`
- [ ] Verify heartbeat watchdog triggers MRM on simulated failure
- [ ] Measure end-to-end latency: Autoware pub → IVC → SPE → IVC → Autoware sub
- [ ] Create `just flash-spe` recipe
- [ ] Document full deployment procedure in `docs/guides/orin-spe-setup.md`

**Acceptance criteria:**
- [ ] SPE sentinel runs on real AGX Orin hardware
- [ ] Heartbeat watchdog triggers emergency stop within 3s of CCPLEX failure
- [ ] End-to-end topic latency < 5 ms (IVC shared memory)
- [ ] Deployment guide complete

## Dependencies

| Subphase | Depends on | Repository |
|----------|------------|------------|
| **Stage 1** | | |
| 11.1 | Phase 7 (integration testing) | autoware-nano-ros |
| 11.2 | 11.1 (FreeRTOS POSIX running) | autoware-nano-ros + nano-ros |
| 11.3 | 11.2 (mock IVC transport) | autoware-nano-ros |
| **Stage 2** | | |
| 11.4 | Hardware (AGX Orin 64GB) | autoware-nano-ros |
| 11.5 | 11.3 (sentinel validated) + 11.4 (IVC verified) | autoware-nano-ros + nano-ros |
| 11.6 | 11.4 (IVC verified) + 11.2 (bridge protocol) | autoware-nano-ros |
| 11.7 | 11.5 + 11.6 | autoware-nano-ros |

Note: Stage 2 subphases 11.4 and 11.6 can proceed in parallel. Stage 1 can run entirely
without hardware — Stage 2 begins once the sentinel is validated and hardware is ready.

## Risk Assessment

1. **IVC not working on Orin**: L4T 36.4 adds Orin support but it's unconfirmed by us.
   Mitigation: 11.1 is the first subphase; if IVC fails, fall back to GPIO-only watchdog
   (SPE monitors heartbeat GPIO, asserts e-stop GPIO — no ROS transport needed).

2. **256 KB too small**: zenoh-pico alone may consume 60–80 KB. If the full sentinel
   doesn't fit, options: (a) reduce to watchdog-only, (b) map zenoh-pico to DRAM via AST,
   (c) use a custom minimal protocol instead of zenoh.

3. **Float ABI mismatch**: NVIDIA BSP uses softfp, Rust eabihf uses hard float. May cause
   ABI violations at FFI boundary. Mitigation: test with a minimal FFI example first; if
   incompatible, switch Rust to `armv7r-none-eabi` (softfp).

4. **IVC frame fragmentation**: zenoh messages may exceed the 64-byte IVC frame size.
   Mitigation: implement length-prefixed reassembly in both IVC link backend and bridge
   daemon (11.2 + 11.6).

## SPE Firmware Flashing

The SPE firmware partition (`A_spe-fw` / `B_spe-fw`) lives on **QSPI NOR flash**, not on
the NVMe/eMMC GPT. NVIDIA's hardware firewall blocks direct QSPI access from Linux
userspace — no `/dev/mtd*` devices are exposed and `/dev/disk/by-partlabel/` does not
contain SPE entries. This means `dd` to the partition is not possible.

### Method 1: Host USB recovery flash (recommended for development)

Flash a single partition from an x86 host connected via USB recovery mode:

```bash
# On host, from L4T BSP directory:
sudo ./flash.sh -k A_spe-fw jetson-agx-orin-devkit mmcblk0p1
```

This is the fastest path for iterating on SPE firmware — it targets only the SPE partition
without touching other bootloader components. Requires putting the Orin into USB recovery
mode (hold recovery button + reset).

### Method 2: On-device UEFI capsule update (for deployment/OTA)

The Jetson can update its own bootloader partitions via a UEFI capsule, but this updates
**all** bootloader partitions (MB1, MB2, UEFI, BPMP, SPE, etc.) as a monolithic operation.
There is no way to target only `spe-fw` — NVIDIA's single-partition capsule feature does
not include `spe-fw` in its supported partition list.

**Generating the capsule (on host):**

```bash
# 1. Replace SPE binary in BSP
cp spe.bin Linux_for_Tegra/bootloader/spe_t234.bin

# 2. Generate BUP payload
sudo ./build_l4t_bup.sh jetson-agx-orin-devkit mmcblk0p1

# 3. Generate UEFI capsule
./generate_capsule/l4t_generate_soc_capsule.sh \
    -i <bup_payload> -o TEGRA_BL.Cap t234
```

**Applying the capsule (on Jetson):**

```bash
sudo nv_bootloader_capsule_updater.sh -q /path/to/TEGRA_BL.Cap
sudo reboot  # UEFI applies update to inactive A/B slot
```

All bootloader components in the capsule must be from the same L4T version. Do not power
off during the update.

### Tool summary

| Tool | Can update SPE? | Notes |
|------|----------------|-------|
| `flash.sh -k A_spe-fw` | Yes (single partition) | Host only, USB recovery mode |
| `nv_update_engine` | Yes (all partitions) | On-device, full BUP payload |
| `nv_bootloader_capsule_updater.sh` | Yes (all partitions) | On-device, UEFI capsule on reboot |
| `nvbootctrl` | No | A/B slot metadata only |
| Single-partition capsule | No | `spe-fw` not in supported list |
| Direct `dd` | No | QSPI hardware firewall blocks access |

### Recommendation

Use **Method 1** (host USB flash) during Phase 11 development for fast iteration. Use
**Method 2** (UEFI capsule) for production deployment and field updates where host access
is unavailable.
