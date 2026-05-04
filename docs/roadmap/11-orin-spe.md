# Phase 11: Orin SPE Deployment

**Status:** 11.5 in progress (firmware wrap landed; SafetyIsland wiring + flash pending).
**Depends on:** Phase 7 (integration testing), Phase 10 (actuation porting),
[`NEWSLabNTU/nano-ros` Phase 100](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/roadmap/phase-100-orin-spe-infra.md) (done as of 2026-05-04 — provides everything below the application boundary).
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

All nano-ros 100.x sub-items landed on `NEWSLabNTU/nano-ros` main as of 2026-05-04
(commit `587adc6b`). Sentinel pins this rev via root `Cargo.toml` `[patch.crates-io]`.

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

## Integration model — SPE app is a library, not a binary

The SPE deployment model is **inverted** relative to nano-ros's existing FreeRTOS
examples (e.g. `examples/qemu-arm-freertos/rust/zenoh/talker/`). On QEMU, Cargo owns the
build: it compiles the FreeRTOS kernel via `cc::Build`, links the application + kernel
into one ELF, and ships a `[[bin]]` crate. On the SPE, **NVIDIA's Makefile owns the
final link**, and the sentinel ships as a Rust **`staticlib`** that NVIDIA's firmware
binary calls into.

### Why the inversion

1. **Closed-source FSP under SDK Manager EULA.** `tegra_aon_fsp.a` ships as a binary
   blob; NVIDIA only sanctions building via `rt-aux-cpu-demo-fsp/Makefile`. Their
   Makefile owns the `-D` flags, include paths, and HW-init source files (`spe-vic.c`,
   `spe-pm.c`, `lic-map.c`, BPMP IPC) that we'd otherwise need to reverse-engineer.
2. **Linker script owns BTCM layout.** `soc/t23x/spe.ld.in` is `gcc -E`-preprocessed
   with `address_map_new.h` + `spe-map.h` to substitute `RUN_ADDR=0xc480000`,
   `BTCM_SIZE=256K`, ARM exception-vector slots. Cargo would have to vendor those
   headers and re-implement the preprocess step.
3. **Secure-boot signing.** `flash.sh` re-signs `spe_t234.bin` against the SoC's
   hardware key chain. The bin must be linked exactly the way NVIDIA's tooling expects
   (one flat BTCM segment, fixed entry); a Cargo-driven link layout breaks signature
   validation and the board refuses to boot.
4. **Float ABI is set by the C side.** BSP CFLAGS = `-mfloat-abi=softfp -mfpu=vfpv3-d16`.
   Cargo can't control the C side's ABI, so Rust must conform — switch firmware crate
   to `armv7r-none-eabi` (soft) target. Reverse direction would require rebuilding
   NVIDIA's bundled newlib `libc.a` hardfp, not worth the maintenance cost.
5. **App-init hook pattern.** Every demo app (`gpio-app`, `i2c-app`, `gte-app`, …)
   slots into `main_task()` via `#if defined(ENABLE_*_APP) *_app_init(); #endif`.
   Sentinel adds `ENABLE_NROS_APP` as one more branch — minimum-friction patch on a
   vendor tree.
6. **Scheduler is already running.** NVIDIA's `main()` calls
   `rtosTaskInitializeScheduler(NULL)` *before* any app code runs. App-side
   `xTaskCreate` happens inside an existing task context. Therefore the SPE `run()` is
   **not `-> !`** — it returns after spawning, in contrast to QEMU's `run() -> !` which
   starts the scheduler itself.

### QEMU vs SPE side-by-side

| Aspect | QEMU FreeRTOS example | SPE deployment |
|--------|----------------------|----------------|
| Crate type | `[[bin]]` | `staticlib` |
| Build owner | Cargo | NVIDIA Makefile |
| Cargo output | linked ELF | rlib → `.a` in `target/.../deps/` |
| FreeRTOS sources | Cargo `cc::Build` | NVIDIA Makefile |
| Linker script | Cargo `-Tmps2_an385.ld` | NVIDIA `spe.ld` (preprocessed `spe.ld.in`) |
| Final binary | runnable directly under QEMU | `spe.bin` raw, signed + flashed |
| Network | LAN9118 + lwIP | none (IVC-only) |
| Float ABI | `thumbv7m-none-eabi` | `armv7r-none-eabi` (soft) |
| Boot signing | none | NVIDIA secure boot via `flash.sh` |
| `panic_handler` | example bin owns it | firmware-wrap crate owns it |
| `run()` shape | `pub fn run<F,E>(cfg, f) -> !` (starts scheduler) | `pub fn run<F,E>(cfg, f)` (returns; scheduler already running) |
| Entry point | Rust `_start` | NVIDIA `_stext` → `main` → `main_task` → `nros_app_init` (C) → `nros_app_rust_entry` (Rust) |

### Layout — why the SPE app does NOT live under `examples/orin-spe/`

nano-ros's `examples/<platform>/rust/zenoh/<bin>/` directory layout assumes
`crate-type = ["bin"]` + Cargo-driven build. That layout doesn't fit the SPE
inversion: a flashable `spe.bin` requires NVIDIA's Makefile, not `cargo run`. The
sentinel SPE app therefore lives **here** in this repo, not in nano-ros:

```
autoware-sentinel/                                  ← this repo
├── src/
│   └── sentinel-spe-firmware/                      ← Phase 11.5.b — staticlib wrap
│       ├── Cargo.toml          crate-type = ["staticlib"]
│       │                       lto = "fat", opt-level = "z", panic = "abort"
│       │                       depends on nros-board-orin-spe (git, phase-100-orin-spe)
│       ├── .cargo/config.toml  target = "armv7r-none-eabi"   (soft float)
│       │                       [unstable] build-std = ["core", "alloc"]
│       └── src/lib.rs          #![no_std]
│                               #[no_mangle]
│                               pub extern "C" fn nros_app_rust_entry() {
│                                   run(Config::default(), |cfg| sentinel::run(cfg))
│                               }
│                               use panic_halt as _;
│
├── scripts/spe/                                    ← Phase 11.5.c — BSP patch series
│   ├── downloads/                                  (BSP source + ARM toolchain — gitignored)
│   │   └── spe-freertos-bsp/                       (shared with nano-ros via SPE_BSP_SRC_DIR)
│   ├── patches/
│   │   ├── 0001-add-ENABLE_NROS_APP-target-flag.patch
│   │   └── 0002-main-task-call-nros-app-init.patch
│   ├── app/
│   │   └── nros-app.c          extern void nros_app_rust_entry(void);
│   │                           void nros_app_init(void) { nros_app_rust_entry(); }
│   └── apply-patches.sh        idempotent (`git apply --check` first)
│
├── src/ivc-bridge/                                 ← Phase 11.6 — Linux-side daemon
│   └── ...                     pulls nano-ros's nvidia-ivc crate
│                               unix-mock for tests, sysfs aon_echo for production
│
└── justfile                                        ← Phase 11.5.d / 11.7
    build-spe-sim       ← Stage 1, FreeRTOS POSIX
    build-spe-image     ← Stage 2, → build/spe.bin
    stage-spe-image     ← cp build/spe.bin → $L4T_BSP_DIR/bootloader/spe_t234.bin
    flash-spe           ← flash.sh -k A_spe-fw  (USB recovery)
    run-ivc-bridge      ← production sysfs path, systemd-managed
    run-ivc-bridge-sim  ← unix-mock pair for tests
```

### Build pipeline (one screenful)

```
cargo build -p sentinel-spe-firmware --release --target armv7r-none-eabi
    │   └── pulls nros-board-orin-spe from nano-ros (git, phase-100-orin-spe)
    │       └── pulls nvidia-ivc/fsp + nros-platform-orin-spe + Z_FEATURE_LINK_IVC
    │
    └── target/armv7r-none-eabi/release/libsentinel_spe_firmware.a

just orin_spe bsp-download    (delegated; nano-ros owns the recipe, shares cache)
just orin_spe bsp-build       → libtegra_aon_fsp.a + libnewlib.a + headers
                                under external/spe-fsp/install/

./scripts/spe/apply-patches.sh
    └── git apply 0001-add-ENABLE_NROS_APP-target-flag.patch
        git apply 0002-main-task-call-nros-app-init.patch
        cp scripts/spe/app/nros-app.c $SPE_BSP/rt-aux-cpu-demo-fsp/app/

make -C $SPE_BSP/rt-aux-cpu-demo-fsp -j$(nproc) bin_t23x \
        ENABLE_NROS_APP=1 \
        SENTINEL_FW_OUT=$(pwd)/target/armv7r-none-eabi/release \
        FREERTOS_DIR=$SPE_BSP/FreeRTOSV10.4.3/FreeRTOS/Source \
        FREERTOS_PORT=GCC/ARM_R5 \
        CROSS_COMPILE=$ARM_TC/bin/arm-none-eabi-
    └── compiles FSP + FreeRTOS + nros-app.c
        links libsentinel_spe_firmware.a + libtegra_aon_fsp.a + libnewlib.a
        → out/t23x/spe.elf (entry _stext, RUN 0xc480000)
        → out/t23x/spe.bin (raw, ≤256 KB)

cp out/t23x/spe.bin build/spe.bin

cp build/spe.bin $L4T_BSP_DIR/bootloader/spe_t234.bin
sudo $L4T_BSP_DIR/flash.sh -k A_spe-fw jetson-agx-orin-devkit internal
                                       (board in USB recovery mode)
    └── flash.sh signs spe_t234.bin against SoC HW key chain
        writes A_spe-fw partition on QSPI
```

### Boot sequence

```
power-on
  → BootROM
    → MB1 / MB2 / BPMP firmware
      → SPE firmware (signed spe.bin) loaded into BTCM at 0xc480000
        → entry _stext (ARM_R5 reset vector → portASM.S → _start)
          → main()                                 // demo's main.c
            → spe_vic_init / lic_init / hsp_init / bpmp_ipc_init / spe_late_init
            → rtosTaskInitializeScheduler(NULL)    // creates main_task, starts scheduler
              → main_task()
                → tegra_clk_init / debug_init / ivc_init_channels_ccplex
                → #if defined(ENABLE_NROS_APP) nros_app_init();    // C shim
                  → nros_app_rust_entry()                          // Rust staticlib
                    → nros_board_orin_spe::run(Config { locator: "ivc/2", … }, |cfg| {
                          // sentinel reduced set: heartbeat watchdog, MRM, cmd-gate
                          sentinel::run(cfg)
                      });
                    └── xTaskCreate(app_task_entry, ...)
                        return                      // run() returns
                  return                            // nros_app_rust_entry returns
                vTaskDelete(NULL);                  // main_task self-deletes
              ──── scheduler keeps running ────
                app_task_entry now runs
                  → executor.spin_once forever
                    → publishes / subscribes over Z_FEATURE_LINK_IVC → ivc/2
                      → tegra_ivc_channel_write → HSP doorbell → CCPLEX
```

`/dev/ttyTCU0` on the Linux side surfaces SPE `printf` output (banner + `nros_app_init`
log) — the first sanity signal that the firmware booted.

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
- [x] *(done in nano-ros Phase 100.0/0a)* Define IVC transport trait + driver crate.
  `packages/drivers/nvidia-ivc/` provides both backends (`fsp`, `unix-mock`) behind the
  same `Channel::{open,read,write,notify,frame_size}` API plus C-callable
  `nvidia_ivc_channel_*` wrappers. `nros-platform-api::PlatformIvc` is the trait
  contract.
- [x] *(done in nano-ros Phase 100.0)* Unix-domain-socket mock backend (frame-oriented,
  64-byte frames matching the real IVC default). Loopback test at
  `packages/drivers/nvidia-ivc/tests/loopback.rs`.
- [x] *(done in nano-ros Phase 100.4)* Zenoh-pico `Z_FEATURE_LINK_IVC` link transport.
  Lives on `jerry73204/zenoh-pico` branch `nano-ros-phase-100-link-ivc` (commit
  `3243086b`); wire format spec is the single source of truth for both sides of the
  bridge.
- [ ] **Bridge daemon** (sentinel side, `src/ivc-bridge/`): read/write mock IVC frames
  from `nvidia-ivc/unix-mock` (test path) or sysfs `aon_echo` (production), forward to
  zenohd TCP. Implement the same `u16 total_len + u16 offset + payload` framing the
  nano-ros side uses — `phase-100-04-link-ivc-design.md` §5 pins it.
- [ ] Add `just run-ivc-bridge-sim` recipe (driver = unix-mock pair).

**Wire-format conformance contract:**

The bridge daemon's reassembly state machine must mirror the test fixture in
`packages/testing/nros-tests/tests/orin_spe_mock_ivc.rs` (drops `total=0,offset=0`
keep-alives, rejects `offset != accumulated_len` on a fresh batch). Cite that test by
file path + commit hash in the bridge crate's README so divergence is review-visible.

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

#### - [ ] 11.5 — Sentinel SPE firmware wrap + BSP integration

The board crate (`nros-board-orin-spe`) is provided by nano-ros Phase 100.6 as an
`rlib` for `armv7r-none-eabihf`. This subphase lands the application-side pieces that
turn that rlib into a flashable `spe.bin`: the **firmware wrap crate** (rlib →
staticlib + panic handler + soft-float decision), the **C shim** that NVIDIA's
`main_task` calls, and the **out-of-tree patch set** to the upstream demo Makefile that
glues them together.

##### 11.5.a — Resolve the float-ABI mismatch

NVIDIA's BSP CFLAGS use `-mfloat-abi=softfp -mfpu=vfpv3-d16`. The demo `spe.elf` is
flagged `Version5 EABI, soft-float ABI`. nano-ros's board crate currently builds
against `armv7r-none-eabihf` (hardfp) — link will fail with `error: ... uses VFP
register arguments, ... does not`.

**Resolution path (recommended): switch Rust to soft float.**
- [ ] Switch firmware crate's `.cargo/config.toml` to
  `target = "armv7r-none-eabi"` + `rustflags = ["-C", "target-feature=+vfp3d16,+strict-align"]`
  (or `-Cllvm-args="-mfloat-abi=softfp"` if rustc rejects the flag).
- [ ] Rebuild nano-ros's `nros-board-orin-spe` rlib for the soft-float target. The
  per-package `.cargo/config.toml` in `packages/boards/nros-board-orin-spe/` already
  pins `armv7r-none-eabihf`; sentinel passes `--target armv7r-none-eabi` on the
  cargo command line to override (or ships its own `.cargo/config.toml`).
- [ ] Add `armv7r-none-eabi` to the workspace nightly's pinned targets in
  `tools/rust-toolchain.toml` (nano-ros side; one-line PR upstream).

**Alternative path (if soft float trips bindings):** rebuild BSP with `-mfloat-abi=hard
-mfpu=vfpv3-d16`. Needs a hardfp newlib variant — the bundled toolchain ships only
soft-float libc. Cost: maintain a vendored newlib build. Reject unless soft-float
incurs a measurable perf regression on the sentinel's control-validator path.

**Acceptance:** `arm-none-eabi-readelf -h spe.elf` prints `soft-float ABI` and the link
step has zero `Tag_ABI_VFP_args` warnings.

##### 11.5.b — Sentinel firmware wrap crate

Lives in `src/sentinel-spe-firmware/` (sentinel side). Wraps the nano-ros board rlib
into a `staticlib` + supplies the panic handler, the alloc shim, and the FFI entry
point the C shim calls.

**Files:**
- [ ] `src/sentinel-spe-firmware/Cargo.toml`
  ```toml
  [lib]
  crate-type = ["staticlib"]
  [profile.release]
  panic = "abort"
  lto = "fat"
  opt-level = "z"
  codegen-units = 1
  [dependencies]
  nros-board-orin-spe = { git = "https://github.com/NEWSLabNTU/nano-ros.git", branch = "phase-100-orin-spe", default-features = false, features = ["fsp", "cortex-r"] }
  panic-halt = "1"
  ```
- [ ] `src/sentinel-spe-firmware/.cargo/config.toml`
  ```toml
  [build]
  target = "armv7r-none-eabi"
  [unstable]
  build-std = ["core", "alloc"]
  ```
- [ ] `src/sentinel-spe-firmware/src/lib.rs`
  ```rust
  #![no_std]
  use nros_board_orin_spe::{Config, run};
  use panic_halt as _;

  #[unsafe(no_mangle)]
  pub extern "C" fn nros_app_rust_entry() {
      run(Config::default(), |config| {
          // 11.3 Stage-1 sentinel runtime, reduced algorithm set, picked
          // up here. Heartbeat watchdog + MRM + cmd-gate.
          sentinel::run(config)
      });
  }
  ```

**Build output:** `target/armv7r-none-eabi/release/libsentinel_spe_firmware.a`.

##### 11.5.c — BSP integration patch (out-of-tree)

The upstream `rt-aux-cpu-demo-fsp/` tree is licensed under the NVIDIA SDK Manager EULA
and lives outside both repos (under `scripts/spe/downloads/spe-freertos-bsp/`). Patches
land as a small Git-format-patch series in `scripts/spe/patches/` and are applied at
build time by a wrapper recipe.

- [ ] `scripts/spe/patches/0001-add-ENABLE_NROS_APP-target-flag.patch`
      Adds `app/nros-app.c` to `SRCS` when `ENABLE_NROS_APP := 1`, and
      `LDFLAGS += -L$(SENTINEL_FW_OUT)/lib -lsentinel_spe_firmware`.
- [ ] `scripts/spe/patches/0002-main-task-call-nros-app-init.patch`
      In `main.c::main_task`, adds `#if defined(ENABLE_NROS_APP) nros_app_init();`
      next to the existing `*_app_init()` calls.
- [ ] `scripts/spe/app/nros-app.c` (sentinel-owned, copied into the BSP tree at apply
      time):
      ```c
      #include <stdio.h>
      extern void nros_app_rust_entry(void);
      void nros_app_init(void) {
          printf("nros_app_init: registering nano-ros task\r\n");
          nros_app_rust_entry();
      }
      ```
- [ ] `scripts/spe/apply-patches.sh` — applies the patch series + copies the shim
      into the BSP tree. Idempotent (`git apply --check` first).

##### 11.5.d — `just build-spe-image` recipe

End-to-end build from rlib to flashable `spe.bin`:

- [ ] `just build-spe-image` driver:
      ```
      1. cargo build -p sentinel-spe-firmware --release
         → target/armv7r-none-eabi/release/libsentinel_spe_firmware.a
      2. just orin_spe bsp-download   (delegates to nano-ros's recipe via submodule
         or shared cache; pulls BSP + ARM toolchain if not cached)
      3. ./scripts/spe/apply-patches.sh
      4. make -C $SPE_BSP/rt-aux-cpu-demo-fsp -j$(nproc) bin_t23x \
            ENABLE_NROS_APP=1 \
            SENTINEL_FW_OUT=$(pwd)/target/armv7r-none-eabi/release \
            FREERTOS_DIR=$SPE_BSP/FreeRTOSV10.4.3/FreeRTOS/Source \
            FREERTOS_PORT=GCC/ARM_R5 \
            CROSS_COMPILE=$ARM_TC/bin/arm-none-eabi-
         → out/t23x/spe.bin
      5. cp out/t23x/spe.bin build/spe.bin
      6. arm-none-eabi-size out/t23x/spe.elf  (must show .text+.data+.bss < 256 KB)
      ```

**Acceptance:**
- [ ] `just build-spe-image` produces `build/spe.bin` reproducibly.
- [ ] `arm-none-eabi-size out/t23x/spe.elf` reports `.text + .data + .bss < 256 KB`
      with at least 16 KB headroom for runtime stacks.
- [ ] Float ABI is consistent (no `Tag_ABI_VFP_args` warnings).
- [ ] `arm-none-eabi-nm` confirms `nros_app_rust_entry`, `_z_open_ivc`,
      `tegra_ivc_channel_*`, `xPortStartScheduler` all present in the linked ELF.
- [ ] `scripts/spe/patches/` series applies cleanly to a fresh
      `bsp-download`-extracted tree.

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

#### - [ ] 11.7 — Flash deployment + on-target integration test

End-to-end deploy: stage `spe.bin` from 11.5, flash via L4T `flash.sh -k A_spe-fw`,
boot, verify sentinel participates in the Autoware planning simulator over real IVC.

##### 11.7.a — Stage + flash

- [ ] `just stage-spe-image` recipe:
      `cp build/spe.bin $L4T_BSP_DIR/bootloader/spe_t234.bin` (overwrites the stock
      L4T SPE binary in place — flash.sh picks it up by filename, no command-line
      override).
- [ ] `just flash-spe` recipe (refines nano-ros's `just orin_spe flash`):
      board in USB recovery mode (force-recovery + reset), then
      `sudo $L4T_BSP_DIR/flash.sh -k A_spe-fw jetson-agx-orin-devkit internal`.
      Read `L4T_BSP_DIR` from env; refuse to run without it. (Note: `internal`
      not `mmcblk0p1` for AGX Orin DevKit running off NVMe — verify against
      target-storage env on each board.)
- [ ] First-boot sanity check via TCU console (`sudo tio /dev/ttyTCU0 -b 115200`):
      look for `nros_app_init: registering nano-ros task` printf from the C shim
      and the board crate's banner (`nros-board-orin-spe (Cortex-R5F)`).

##### 11.7.b — Device-tree overlay for `aon_echo`

- [ ] Author DTB overlay enabling `aon_echo { status = "okay"; }` for IVC channel 2,
      apply via `/boot/extlinux/extlinux.conf` `FDT` entry or
      `nv_update_engine`-bundled overlay.
- [ ] Confirm `/sys/devices/platform/bus@0/bus@0:aon_echo/data_channel` appears.
- [ ] Quick echo round-trip from Linux userspace:
      `printf 'ping' > .../data_channel; cat .../data_channel`. Validates the
      hardware-level IVC path independently of zenoh-pico.

##### 11.7.c — Bridge daemon on production sysfs path

- [ ] `src/ivc-bridge/` (the daemon shipped under 11.6) reads/writes the sysfs
      `data_channel` for production deployment, and `nvidia-ivc/unix-mock` for
      tests. systemd unit auto-starts on boot, after `network-online.target` so
      zenohd is reachable.
- [ ] `just run-ivc-bridge` recipe — foreground run with verbose logging for
      bring-up.

##### 11.7.d — Autoware planning-simulator E2E

- [ ] Start the bridge daemon, `zenohd`, and the Autoware planning simulator.
- [ ] `ros2 topic list` shows the SPE-side sentinel topics
      (`/sentinel/heartbeat/state`, `/control/command/control_cmd_emergency`, etc.).
- [ ] Measure end-to-end latency: Autoware pub → IVC frame → SPE handler → IVC
      reply → Autoware sub. Target < 5 ms (one shared-memory round-trip).
- [ ] Heartbeat-watchdog trip test: kill the CCPLEX heartbeat publisher; the SPE
      sentinel must engage MRM emergency-stop within 3 s and assert the e-stop
      GPIO mirror.
- [ ] Soak test: 24 h continuous run under planning-simulator load. Watch for
      memory drift via `/proc/$(pidof zenohd)/status` on Linux side and the SPE's
      `vTaskGetRunTimeStats` over TCU.

##### 11.7.e — Documentation

- [ ] Update `docs/guides/orin-spe-setup.md` with the actual measured numbers
      (text/data/bss footprint, BTCM headroom, IVC RTT, heartbeat trip latency).
- [ ] Add a "What can go wrong" section keyed off failure modes seen during
      bring-up (USB recovery flakes, DTB overlay precedence, capsule-vs-flash.sh
      slot confusion, etc.).
- [ ] Cross-reference nano-ros Phase 100's design doc by URL + commit hash so the
      wire-format provenance trail is permanent.

**Acceptance criteria:**
- [ ] SPE sentinel runs on real AGX Orin hardware after `just flash-spe`.
- [ ] Heartbeat watchdog triggers emergency stop within 3 s of CCPLEX failure.
- [ ] End-to-end topic latency < 5 ms (IVC shared memory).
- [ ] 24-hour soak with no observable memory drift on either side.
- [ ] `docs/guides/orin-spe-setup.md` reflects the as-built procedure with measured
      numbers, not the speculative pre-bring-up estimates.

## Dependencies

| Subphase | Depends on | Repository |
|----------|------------|------------|
| **Stage 1** | | |
| 11.1 | Phase 7 (integration testing) | autoware-sentinel |
| 11.2 | 11.1 (FreeRTOS POSIX running) + nano-ros Phase 100 (done — driver, link, mock) | autoware-sentinel (bridge daemon only) |
| 11.3 | 11.2 (mock IVC transport) | autoware-sentinel |
| **Stage 2** | | |
| 11.4 | Hardware (AGX Orin 64GB) | autoware-sentinel |
| 11.5 | 11.3 (sentinel validated) + 11.4 (IVC verified) + nano-ros Phase 100 (done) | autoware-sentinel (firmware wrap + BSP patch) |
| 11.6 | 11.4 (IVC verified) + 11.2 (bridge protocol) | autoware-sentinel |
| 11.7 | 11.5 + 11.6 | autoware-sentinel |

Note: Stage 2 subphases 11.4 and 11.6 can proceed in parallel. Stage 1 can run entirely
without hardware — Stage 2 begins once the sentinel is validated and hardware is ready.
nano-ros Phase 100 is **done** as of 2026-05-04 on branch `phase-100-orin-spe`; sentinel
sub-items previously labelled "modify nano-ros" are now upstream-provided and marked
`[x]` in the task lists.

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
