# justfile for autoware-nano-ros

set dotenv-load

kani_packages := "autoware_stop_filter autoware_vehicle_velocity_converter autoware_shift_decider autoware_mrm_emergency_stop_operator"

workspace_dir := "../autoware-sentinel-workspace"
env_script := workspace_dir / "env.sh"
zenohd := "external/zenoh/target/fast/zenohd"
session_config := ".config/zenoh_session.json5"
router_config := ".config/zenoh_router.json5"
rmw_zenoh_ws := "external/rmw_zenoh_ws"

# Default recipe - show available commands
default:
    @just --list

# ════════════════════════════════════════════════════════════════════
# Build
# ════════════════════════════════════════════════════════════════════

# Generate message bindings (sentinel_linux is the superset; workspace patches share its generated/)
generate-bindings:
    #!/usr/bin/env bash
    set -eo pipefail
    source scripts/activate_autoware.sh
    echo "=== autoware_sentinel_linux ==="
    (cd "src/autoware_sentinel_linux" && cargo nano-ros generate-rust --force)
    echo "=== autoware_sentinel_zephyr ==="
    (cd "src/autoware_sentinel_zephyr" && cargo nano-ros generate-rust --force)

# Build all packages + every platform target. Cross-target sentinels
# (Zephyr / FreeRTOS / NuttX) are built last so their cross-toolchain
# requirements (`west`, `arm-none-eabi-gcc`, NuttX kernel) only block
# the cross builds, not the workspace test compile.
build: generate-bindings build-rmw-zenoh build-sentinel-linux build-zephyr build-sentinel-freertos build-sentinel-nuttx
    cargo build --workspace --tests

# Build Linux sentinel binary
build-sentinel-linux:
    cargo build -p autoware_sentinel_linux

# FreeRTOS QEMU prereq env (matches nano-ros/just/freertos.just).
#
# Phase 13.K1.7: shrink nros executor capacity for the FreeRTOS build. The
# defaults in `.env` (NROS_EXECUTOR_MAX_CBS=96 → ARENA_SIZE ≈ 346 KB) inline a
# ~340 KB arena directly in `Executor`, which the compiler then duplicates
# (NRVO defeated by the `Result<Executor, _>` return) into a ~1 MB stack
# frame on `app_task_entry`. The Cortex-M3 task stack overflows below SRAM
# base, the SP wraps to wild addresses, and the very first push out of
# `app_task_entry` faults with v7M INVSTATE UsageFault. Override here so
# FreeRTOS gets an arena sized for the minimal sentinel topology (≈26 cbs +
# headroom). Linux keeps `.env` defaults for full Phase 12 capacity.
freertos_env := "FREERTOS_DIR=" + justfile_directory() / "../nano-ros-sentinel/third-party/freertos/kernel" + " LWIP_DIR=" + justfile_directory() / "../nano-ros-sentinel/third-party/freertos/lwip" + " FREERTOS_PORT=GCC/ARM_CM3 FREERTOS_CONFIG_DIR=" + justfile_directory() / "../nano-ros-sentinel/packages/boards/nros-board-mps2-an385-freertos/config" + " NROS_EXECUTOR_MAX_CBS=32 NROS_MAX_PARAMETERS=8 NROS_PARAM_SERVICE_BUFFER_SIZE=1024 ZPICO_MAX_PUBLISHERS=40 ZPICO_MAX_SUBSCRIBERS=8 ZPICO_MAX_QUERYABLES=32 ZPICO_MAX_LIVELINESS=80"

# Build FreeRTOS QEMU MPS2-AN385 sentinel (release; thumbv7m-none-eabi)
build-sentinel-freertos:
    #!/usr/bin/env bash
    set -eo pipefail
    cd src/autoware_sentinel_freertos
    {{ freertos_env }} cargo build --release

# Run FreeRTOS QEMU sentinel (requires zenohd listening on 0.0.0.0:7451)
run-sentinel-freertos: build-sentinel-freertos
    #!/usr/bin/env bash
    set -eo pipefail
    cd src/autoware_sentinel_freertos
    {{ freertos_env }} cargo run --release

# ════════════════════════════════════════════════════════════════════
# NuttX QEMU ARM virt — Phase 13.5
# ════════════════════════════════════════════════════════════════════
#
# Two-phase build:
#   1. NuttX kernel built via the nano-ros board crate's build-nuttx.sh
#      script (depends on arm-none-eabi-gcc + kconfig tools + a NuttX
#      source tree at $NUTTX_DIR). The kernel exports a staging dir +
#      preprocessed linker script that the sentinel binary consumes.
#   2. autoware_sentinel_nuttx cross-compiled to armv7a-nuttx-eabihf
#      with `-Z build-std`. Pinned nightly toolchain in the crate's
#      rust-toolchain.toml. The Rust binary IS the kernel image (NuttX
#      flat-build) so no additional linking step is required.

# Defaults reuse the nano-ros sibling clone, matching freertos_env. Override
# NUTTX_DIR / NUTTX_APPS_DIR before running `just` if you keep a separate
# NuttX checkout.
nuttx_dir := env_var_or_default("NUTTX_DIR", justfile_directory() / "../nano-ros/third-party/nuttx/nuttx")
nuttx_apps_dir := env_var_or_default("NUTTX_APPS_DIR", justfile_directory() / "../nano-ros/third-party/nuttx/nuttx-apps")
nuttx_build_script := justfile_directory() / "../nano-ros/scripts/nuttx/build-nuttx.sh"

# Override `.env`'s NROS_EXECUTOR_MAX_CBS=96 for NuttX. Same root cause as
# the FreeRTOS-side fix: nros::Executor's inline arena ([MaybeUninit<u8>;
# ARENA_SIZE]) gets copied through the NRVO-defeated Result return path,
# inflating the closure stack frame to ~3× ARENA_SIZE. NuttX's main task
# (CONFIG_INIT_STACKSIZE=524288) tolerates more than the FreeRTOS task
# but still overflows when ARENA_SIZE ≈ 346 KB. 32 callbacks gives ARENA
# ≈ 116 KB, frame ≈ 350 KB, fits comfortably and covers comp-all (28 cbs).
nuttx_env := "NROS_EXECUTOR_MAX_CBS=32 NROS_MAX_PARAMETERS=8 NROS_PARAM_SERVICE_BUFFER_SIZE=1024 ZPICO_MAX_PUBLISHERS=40 ZPICO_MAX_SUBSCRIBERS=8 ZPICO_MAX_QUERYABLES=32 ZPICO_MAX_LIVELINESS=80"

# Build the NuttX kernel using the nros board crate's defconfig (idempotent).
build-nuttx-kernel:
    #!/usr/bin/env bash
    set -eo pipefail
    if [ ! -d "{{ nuttx_dir }}/include" ]; then
        echo "ERROR: NuttX not found at {{ nuttx_dir }}"
        echo "Set NUTTX_DIR or check out third-party/nuttx in nano-ros-sentinel."
        exit 1
    fi
    NUTTX_DIR="{{ nuttx_dir }}" NUTTX_APPS_DIR="{{ nuttx_apps_dir }}" \
        bash "{{ nuttx_build_script }}"

# Build NuttX QEMU sentinel (release; armv7a-nuttx-eabihf, build-std).
build-sentinel-nuttx: build-nuttx-kernel
    #!/usr/bin/env bash
    set -eo pipefail
    cd src/autoware_sentinel_nuttx
    NUTTX_DIR="{{ nuttx_dir }}" {{ nuttx_env }} cargo build --release

# Run NuttX QEMU sentinel via QEMU SLIRP (zenohd must listen on
# 127.0.0.1:7452 — see config.toml).
run-sentinel-nuttx: build-sentinel-nuttx
    #!/usr/bin/env bash
    set -eo pipefail
    BIN="src/autoware_sentinel_nuttx/target/armv7a-nuttx-eabihf/release/autoware_sentinel_nuttx"
    if [ ! -f "$BIN" ]; then
        echo "ERROR: sentinel ELF not found at $BIN"
        exit 1
    fi
    qemu-system-arm -M virt -cpu cortex-a7 -nographic \
        -kernel "$BIN" \
        -netdev user,id=net0 \
        -device virtio-net-device,netdev=net0

# Capacity envs for the Zephyr build. Mirror the FreeRTOS / NuttX
# overrides — `.env` defaults (`NROS_EXECUTOR_MAX_CBS=96`) inflate
# the inline Executor arena past native_sim's main-thread stack via
# the same NRVO-defeated copy chain documented in 13.K1.7. Zephyr
# `west build` does not inherit `.env` (just's dotenv only loads in
# the recipe shell), so export them explicitly.
zephyr_env := "NROS_EXECUTOR_MAX_CBS=32 NROS_MAX_PARAMETERS=64 NROS_PARAM_SERVICE_BUFFER_SIZE=8192 ZPICO_MAX_PUBLISHERS=40 ZPICO_MAX_SUBSCRIBERS=8 ZPICO_MAX_QUERYABLES=32 ZPICO_MAX_LIVELINESS=80"

# Build Zephyr application (native_sim)
build-zephyr:
    #!/usr/bin/env bash
    set -eo pipefail
    source {{ env_script }}
    cd {{ workspace_dir }}
    {{ zephyr_env }} west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel_zephyr -d build/sentinel

# Build rmw_zenoh_cpp from source
build-rmw-zenoh:
    scripts/build_rmw_zenoh.sh

# Rebuild zenohd from source
build-zenohd:
    cd external/zenoh && cargo build --profile fast -p zenohd

# Build the IVC ↔ TCP bridge daemon (Phase 11.2 / 11.6).
build-ivc-bridge:
    cargo build -p ivc-bridge --release

# Run the IVC bridge against a host zenohd. Defaults match the
# autoware_sentinel_spe POSIX dev path: a Unix socket at
# /tmp/autoware-sentinel-ivc.sock and zenohd on 127.0.0.1:7447.
run-ivc-bridge: build-ivc-bridge
    target/release/ivc-bridge --backend unix-mock

# Build the AGX Orin SPE sentinel binary (Phase 11.1 — POSIX dev path).
# Same wire_executor body as the other platform binaries; reduced
# feature set sized for the 256 KB BTCM budget on real Cortex-R5F.
# Uses the workspace-default `.env` capacity caps so the comp-mrm +
# comp-engagement entity count fits in zpico's static tables.
build-sentinel-spe-sim:
    #!/usr/bin/env bash
    set -eo pipefail
    cd src/autoware_sentinel_spe
    cargo build --release

# Run the SPE sentinel as a Linux process (Phase 11.3 dev workflow).
# Currently uses TCP locator directly until the link-ivc cargo feature
# is plumbed through nros for the platform-posix profile (TODO 11.2.b);
# pair with `just run-zenohd` in another terminal.
run-sentinel-spe-sim: build-sentinel-spe-sim
    src/autoware_sentinel_spe/target/release/autoware_sentinel_spe

# ════════════════════════════════════════════════════════════════════
# SPE firmware (Phase 11.5 + 11.7 — autoware-sentinel on AGX Orin SPE)
#
# Pipeline: cargo staticlib → BSP patch apply → upstream Makefile →
# spe.bin → cp into L4T bootloader/ → flash.sh -k A_spe-fw.
#
# Env knobs:
#   SPE_BSP_SRC_DIR    pre-extracted BSP root (default:
#                      scripts/spe/downloads/spe-freertos-bsp).
#   ARM_TOOLCHAIN_DIR  pre-extracted arm-none-eabi toolchain root
#                      (default: scripts/spe/downloads/arm-gnu-toolchain-13.2.rel1).
#   L4T_BSP_DIR        L4T BSP root for flashing (typically
#                      ~/nvidia/Linux_for_Tegra). Required for stage-
#                      spe-image / flash-spe.
# ════════════════════════════════════════════════════════════════════

# Download the SPE FreeRTOS BSP + ARM GNU toolchain (idempotent).
orin_spe-bsp-download:
    ./scripts/spe/download-bsp.sh

# Apply the BSP integration patch series (idempotent). Adds
# ENABLE_NROS_APP to the upstream Makefile + main_task hook + copies
# the C shim into the BSP tree.
orin_spe-bsp-patch:
    ./scripts/spe/apply-patches.sh

# Build NVIDIA's FSP into a static archive that the firmware crate's
# Cargo build links against (cargo wants the archive to exist for
# metadata mmap; the final link happens in NVIDIA's Makefile, but
# Cargo errors before the Makefile gets a chance to run if the
# archive is missing).
#
# Repackages every `.o` produced by the upstream demo Makefile (minus
# the demo's `main.o` / `app_init.o` entry points so they don't
# collide with our `nros_app_init`) into a single
# `libtegra_aon_fsp.a` plus stages headers under
# `external/spe-fsp/install/`. Mirrors nano-ros's
# `just orin_spe bsp-build`.
orin_spe-bsp-stage: orin_spe-bsp-download
    #!/usr/bin/env bash
    set -euo pipefail
    BSP="${SPE_BSP_SRC_DIR:-$(pwd)/scripts/spe/downloads/spe-freertos-bsp}"
    TC="${ARM_TOOLCHAIN_DIR:-$(pwd)/scripts/spe/downloads/arm-gnu-toolchain-13.2.rel1}"
    PREFIX="$(pwd)/external/spe-fsp/install"

    [ -d "$BSP/fsp/source" ] || { echo "BSP missing at $BSP"; exit 1; }
    [ -x "$TC/bin/arm-none-eabi-gcc" ] || { echo "Toolchain missing"; exit 1; }

    if [ -f "$PREFIX/lib/libtegra_aon_fsp.a" ]; then
        echo "==> Staged FSP already at $PREFIX (run 'just clean-spe' to refresh)"
        exit 0
    fi

    echo "==> Building rt-aux-cpu-demo-fsp (without ENABLE_NROS_APP) to produce .o files"
    make -C "$BSP/rt-aux-cpu-demo-fsp" -j"$(nproc)" \
        SPE_FREERTOS_BSP="$BSP" \
        FREERTOS_DIR="$BSP/FreeRTOSV10.4.3/FreeRTOS/Source" \
        FREERTOS_PORT="GCC/ARM_R5" \
        FSP_SRC_DIR="$BSP/fsp/source" \
        CROSS_COMPILE="$TC/bin/arm-none-eabi-" \
        bin_t23x

    OUTDIR="$BSP/rt-aux-cpu-demo-fsp/out/t23x"
    [ -d "$OUTDIR" ] || { echo "no $OUTDIR — make failed?"; exit 1; }

    mkdir -p "$PREFIX/lib" "$PREFIX/include"

    echo "==> Packaging FSP/FreeRTOS objects into libtegra_aon_fsp.a"
    OBJS=$(find "$OUTDIR" -name '*.o' \
        -not -name 'main.o' \
        -not -name 'app_init.o' \
        -not -name 'startup.o' \
        | sort)
    [ -n "$OBJS" ] || { echo "no .o under $OUTDIR"; exit 1; }
    "$TC/bin/arm-none-eabi-ar" rcs "$PREFIX/lib/libtegra_aon_fsp.a" $OBJS

    LIBC="$(find "$TC/arm-none-eabi/lib" -name 'libc.a' | head -1 || true)"
    if [ -n "$LIBC" ]; then
        cp "$LIBC" "$PREFIX/lib/libnewlib.a"
    fi

    rm -rf "$PREFIX/include/fsp" "$PREFIX/include/freertos"
    mkdir -p "$PREFIX/include/fsp" "$PREFIX/include/freertos"
    cp -a "$BSP/fsp/source/include/." "$PREFIX/include/fsp/"
    cp -a "$BSP/FreeRTOSV10.4.3/FreeRTOS/Source/include/." "$PREFIX/include/freertos/"
    # Phase 11.3.B — zpico-sys's orin-spe path also needs the ARM_R5
    # port (`portmacro.h`) and the demo's `FreeRTOSConfig.h` to
    # cross-compile zenoh-pico's `system/freertos/system.c`.
    mkdir -p "$PREFIX/include/freertos/portable"
    cp -a "$BSP/FreeRTOSV10.4.3/FreeRTOS/Source/portable/GCC" "$PREFIX/include/freertos/portable/"
    # FreeRTOSConfig.h: drop the `#include <artimer.h>` line — that's
    # an FSP runtime dependency we don't need at C-compile time, and
    # pulling artimer.h would drag in dozens of additional include
    # paths from `rt-aux-cpu-demo-fsp/soc/t23x/include/`. zenoh-pico's
    # `system/freertos/system.c` only needs the `config*` defines.
    sed '/^#include <artimer\.h>/d' "$BSP/rt-aux-cpu-demo-fsp/FreeRTOSConfig.h" \
        > "$PREFIX/include/FreeRTOSConfig.h"

    echo "==> Staged at $PREFIX:"
    ls -lh "$PREFIX/lib"/*.a | awk '{print "    " $0}'

    # Wipe the demo's out/ so the subsequent `build-spe-image` make
    # invocation rebuilds with ENABLE_NROS_APP=1 from scratch (the
    # generated `.o` files are CFLAGS-dependent and we'd otherwise
    # link a mix).
    rm -rf "$OUTDIR"

# Build the Rust firmware staticlib (Phase 11.5.b). Uses its own
# .cargo/config.toml that pins armv7r-none-eabi (soft float).
# Depends on bsp-stage so cargo's link-search resolves
# `libtegra_aon_fsp.a` for metadata.
build-spe-firmware: orin_spe-bsp-stage
    #!/usr/bin/env bash
    set -euo pipefail
    PREFIX="$(pwd)/external/spe-fsp/install"
    TC="${ARM_TOOLCHAIN_DIR:-$(pwd)/scripts/spe/downloads/arm-gnu-toolchain-13.2.rel1}"
    [ -f "$PREFIX/lib/libtegra_aon_fsp.a" ] || \
        { echo "FSP not staged at $PREFIX — run 'just orin_spe-bsp-stage'"; exit 1; }
    # `SENTINEL_SPE_FEATURES` selects between the default (Executor +
    # spin only, fits BTCM) and `safety-island` (Executor + wire_executor,
    # currently overflows by ~37 KB — see docs/roadmap/11-orin-spe.md
    # §11.3.D). Override via `SENTINEL_SPE_FEATURES=safety-island just
    # build-spe-firmware` to reproduce the overflow build.
    FEATURES="${SENTINEL_SPE_FEATURES:-}"
    if [ -n "$FEATURES" ]; then
        FEATURE_FLAG="--features $FEATURES"
        echo "==> building sentinel-spe-firmware with features: $FEATURES"
    else
        FEATURE_FLAG=""
        echo "==> building sentinel-spe-firmware (default features)"
    fi
    cd src/sentinel_spe_firmware
    # Slot counts sized to wire_executor's actual usage (Phase 11.3.D):
    # 6 publishers (core only) / 3 subscribers / 1 service / 1 timer /
    # 1 liveliness token per node. Defaults of 56/16/32/96 (set somewhere
    # else in the workspace) cost ~70 KB BTCM in arena buffers — fatal on
    # 256 KB BTCM. Buffer sizes drop to 256 B since heartbeat /
    # velocity_status / control_cmd msgs all fit well under 256 B.
    NV_SPE_FSP_DIR="$PREFIX" \
        ZPICO_MAX_PUBLISHERS=8 \
        ZPICO_MAX_SUBSCRIBERS=4 \
        ZPICO_MAX_QUERYABLES=2 \
        ZPICO_MAX_LIVELINESS=16 \
        ZPICO_MAX_PENDING_GETS=2 \
        ZPICO_SUBSCRIBER_BUFFER_SIZE=256 \
        ZPICO_SERVICE_BUFFER_SIZE=256 \
        NROS_EXECUTOR_MAX_CBS=8 \
        NROS_SUBSCRIPTION_BUFFER_SIZE=256 \
        cargo +nightly build --release $FEATURE_FLAG
    OUT="$(pwd)/target/armv7r-none-eabi/release/libsentinel_spe_firmware.a"
    [ -f "$OUT" ] || { echo "expected staticlib not produced: $OUT"; exit 1; }

    # Strip FSP/FreeRTOS objects that cargo bundled into the staticlib.
    # The board crate emits `cargo:rustc-link-lib=static=tegra_aon_fsp`
    # in its build.rs, and `staticlib` archives slurp every transitive
    # native lib. NVIDIA's Makefile compiles the same FSP/FreeRTOS
    # sources directly, so leaving them in our staticlib triggers
    # multiple-definition errors at the final link. Drop the
    # duplicates here — at the cost of one `ar d` per .o.
    AR="$TC/bin/arm-none-eabi-ar"
    "$AR" t "$PREFIX/lib/libtegra_aon_fsp.a" | sort -u > /tmp/fsp_objs.list
    "$AR" t "$PREFIX/lib/libnewlib.a"        | sort -u >> /tmp/fsp_objs.list
    cnt=0
    # The staticlib can carry MULTIPLE archive members with the same
    # basename (cargo bundles libtegra_aon_fsp.a AND libnewlib.a, both
    # of which can ship e.g. `event_groups.o`). `ar d` removes only
    # the first match per call, so loop until each name is gone.
    while read -r obj; do
        while "$AR" t "$OUT" 2>/dev/null | grep -qx "$obj"; do
            "$AR" d "$OUT" "$obj" 2>/dev/null || true
            cnt=$((cnt + 1))
        done
    done < /tmp/fsp_objs.list
    rm -f /tmp/fsp_objs.list
    echo "==> built: $OUT ($(du -h "$OUT" | cut -f1)); stripped $cnt FSP/newlib duplicates"

# End-to-end image build: cargo staticlib → patch BSP → upstream Make
# → spe.bin staged into build/. Phase 11.5.d.
build-spe-image: build-spe-firmware orin_spe-bsp-patch
    #!/usr/bin/env bash
    set -euo pipefail
    BSP="${SPE_BSP_SRC_DIR:-$(pwd)/scripts/spe/downloads/spe-freertos-bsp}"
    TC="${ARM_TOOLCHAIN_DIR:-$(pwd)/scripts/spe/downloads/arm-gnu-toolchain-13.2.rel1}"
    FW_OUT="$(pwd)/src/sentinel_spe_firmware/target/armv7r-none-eabi/release"
    [ -f "$FW_OUT/libsentinel_spe_firmware.a" ] || \
        { echo "firmware staticlib missing — build-spe-firmware failed?"; exit 1; }

    echo "==> make bin_t23x ENABLE_NROS_APP=1"
    make -C "$BSP/rt-aux-cpu-demo-fsp" -j"$(nproc)" \
        SPE_FREERTOS_BSP="$BSP" \
        FREERTOS_DIR="$BSP/FreeRTOSV10.4.3/FreeRTOS/Source" \
        FREERTOS_PORT="GCC/ARM_R5" \
        FSP_SRC_DIR="$BSP/fsp/source" \
        CROSS_COMPILE="$TC/bin/arm-none-eabi-" \
        ENABLE_NROS_APP=1 \
        SENTINEL_FW_OUT="$FW_OUT" \
        bin_t23x

    OUTDIR="$BSP/rt-aux-cpu-demo-fsp/out/t23x"
    mkdir -p build
    cp "$OUTDIR/spe.bin" build/spe.bin
    cp "$OUTDIR/spe.elf" build/spe.elf

    echo ""
    echo "==> Image staged:"
    ls -lh build/spe.bin build/spe.elf | awk '{print "    " $0}'
    echo ""
    echo "==> Memory budget (256 KB BTCM):"
    "$TC/bin/arm-none-eabi-size" build/spe.elf
    echo ""
    echo "==> Symbol-presence sanity check:"
    # `_z_open_ivc` / `tegra_ivc_*` are gc-sectioned on the scaffold
    # build (no actual zenoh session opened in the wfi-loop closure).
    # Once 11.3.D wires `wire_executor`, those get pulled in. Until
    # then, only check for the boot-path symbols that prove the C
    # shim resolved + the FreeRTOS port linked.
    NM="$TC/bin/arm-none-eabi-nm"
    # Collect symbol-name column upfront so the per-symbol check is a
    # plain `case` glob match — avoids `pipefail` + `grep -q` SIGPIPE-ing
    # the upstream `awk` and producing false `[MISS]` reports under
    # `set -euo pipefail`.
    SYM_LIST=$("$NM" build/spe.elf | awk '{print $3}')
    for sym in nros_app_rust_entry nros_app_init xPortStartScheduler; do
        case $'\n'"$SYM_LIST"$'\n' in
            *$'\n'"$sym"$'\n'*) echo "    [OK] $sym" ;;
            *) echo "    [MISS] $sym (linker may have garbage-collected it)" ;;
        esac
    done

# Stage the built spe.bin into the L4T BSP's bootloader/ tree so
# flash.sh picks it up by filename. Must run before flash-spe.
stage-spe-image: build-spe-image
    #!/usr/bin/env bash
    set -euo pipefail
    [ -n "${L4T_BSP_DIR:-}" ] || \
        { echo "L4T_BSP_DIR not set (e.g. ~/nvidia/Linux_for_Tegra)"; exit 1; }
    [ -f build/spe.bin ] || { echo "build/spe.bin missing"; exit 1; }
    cp build/spe.bin "$L4T_BSP_DIR/bootloader/spe_t234.bin"
    echo "==> Staged: $L4T_BSP_DIR/bootloader/spe_t234.bin"

# Flash the SPE firmware partition. Board must be in USB recovery
# mode (force-recovery + reset). Phase 11.7.a.
flash-spe: stage-spe-image
    #!/usr/bin/env bash
    set -euo pipefail
    [ -n "${L4T_BSP_DIR:-}" ] || \
        { echo "L4T_BSP_DIR not set"; exit 1; }
    [ -x "$L4T_BSP_DIR/flash.sh" ] || \
        { echo "flash.sh not found at $L4T_BSP_DIR"; exit 1; }
    echo "Put the AGX Orin DevKit in USB recovery mode (force-recovery + reset)."
    echo "Press Enter to continue, Ctrl-C to abort..."
    read -r
    cd "$L4T_BSP_DIR"
    sudo ./flash.sh -k A_spe-fw jetson-agx-orin-devkit internal

# Clean SPE build artifacts. Leaves scripts/spe/downloads/ alone.
clean-spe:
    #!/usr/bin/env bash
    rm -rf src/sentinel_spe_firmware/target build/spe.bin build/spe.elf
    BSP="${SPE_BSP_SRC_DIR:-$(pwd)/scripts/spe/downloads/spe-freertos-bsp}"
    if [ -d "$BSP/rt-aux-cpu-demo-fsp/out" ]; then
        rm -rf "$BSP/rt-aux-cpu-demo-fsp/out"
    fi
    echo "==> SPE build artifacts cleaned"

# ════════════════════════════════════════════════════════════════════
# Run
# ════════════════════════════════════════════════════════════════════

# Run zenohd router on localhost:7447
run-zenohd:
    {{ zenohd }} --config {{ router_config }}

# Run the Linux sentinel binary (unsets SESSION/ROUTER_CONFIG_URI to avoid breaking liveliness)
run-sentinel: build-sentinel-linux
    env ZENOH_SESSION_CONFIG_URI= ZENOH_ROUTER_CONFIG_URI= cargo run -p autoware_sentinel_linux

# Run Zephyr sentinel (native_sim) — requires TAP network + separate zenohd
run-sentinel-zephyr: build-zephyr
    #!/usr/bin/env bash
    set -eo pipefail
    ZEPHYR_BIN="{{ workspace_dir }}/build/sentinel/zephyr/zephyr.exe"
    if [ ! -f "$ZEPHYR_BIN" ]; then
        echo "Error: Zephyr binary not found. Run: just build-zephyr"
        exit 1
    fi
    echo "NOTE: TAP network must be set up first: just setup-tap-network"
    echo "NOTE: zenohd must listen on 0.0.0.0:7447 (bridge IP)"
    "$ZEPHYR_BIN"

# Run full baseline Autoware via play_launch (zenohd must be running separately)
run-autoware: dump-autoware
    play_launch replay --input-file tmp/launch/autoware_record.json --web-addr 0.0.0.0:8080

# Run filtered Autoware via play_launch (zenohd + sentinel must be running separately)
run-autoware-filtered: filter-autoware
    play_launch replay --input-file tmp/launch/autoware_record_filtered.json --web-addr 0.0.0.0:8080

# Run the autonomous drive controller (init pose → route → engage → wait for arrival)
run-auto-drive timeout="120" poses="scripts/poses.yaml":
    python3 scripts/auto_drive.py --timeout {{ timeout }} --poses {{ poses }}

# Restart ros2 daemon with rmw_zenoh_cpp (picks up RMW settings from .envrc)
ros2-daemon-restart:
    #!/usr/bin/env bash
    set -eo pipefail
    ros2 daemon stop 2>/dev/null || true
    sleep 2
    ros2 daemon start
    echo "ros2 daemon started (RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION)"

# ════════════════════════════════════════════════════════════════════
# Test
# ════════════════════════════════════════════════════════════════════

# Test all packages (unit tests)
test:
    cargo test --workspace

# Run integration tests with nextest
test-integration:
    cd tests && cargo nextest run

# Run integration tests (transport smoke only)
test-transport:
    cd tests && cargo nextest run -E 'binary(transport_smoke)'

# Run planning simulator integration tests only
test-planning:
    cd tests && cargo nextest run -E 'binary(planning_simulator)'

# Run auto-drive comparison tests (baseline vs sentinel)
test-auto-drive:
    cd tests && cargo nextest run -E 'binary(auto_drive_comparison)'

# Run SPE sentinel POSIX dev path tests (Phase 11.3.E)
test-spe-sim:
    cd tests && cargo nextest run -E 'binary(sentinel_spe)'

# Run Zephyr native_sim integration tests.
#
# nextest forks a new process per test. The Zephyr round-trip tests
# share `tcp/127.0.0.1:7447` baked into the firmware. Two complications
# stack up across sequential tests:
#
# 1. Native-sim Zephyr's `sys_rand32_get` is the test PRNG (deterministic
#    seed) — every Zephyr boot generates the same zenoh ZID. zenohd
#    rejects the duplicate session until the previous one's keepalive
#    lease (10 s) expires.
# 2. ROS 2's `ros2 daemon` survives `ros2 topic` exits and accumulates
#    DDS/zenoh participants that slow each successive test's discovery
#    handshake.
#
# Workaround: reap orphans before nextest, then run tests with
# `--test-threads=1` and a generous slow timeout so the per-test
# `start_zephyr_sentinel` wait has room for the lease to expire on the
# zenohd side before the next test reuses the same ZID.
test-zephyr:
    #!/usr/bin/env bash
    set -eo pipefail
    # Only target zenohd bound to the Zephyr port so other platform
    # suites running in parallel are unaffected.
    pkill -9 -f 'zenohd.*tcp/127\.0\.0\.1:7447' >/dev/null 2>&1 || true
    pkill -9 -f zephyr.exe >/dev/null 2>&1 || true
    pkill -9 -f _ros2_daemon >/dev/null 2>&1 || true
    sleep 1
    cd tests && cargo nextest run -E 'binary(zephyr_native_sim)'

# ════════════════════════════════════════════════════════════════════
# Autoware planning simulator
# ════════════════════════════════════════════════════════════════════

# Dump Autoware planning simulator launch to record.json
dump-autoware map_path="/opt/autoware/1.5.0/share/autoware_test_utils/test_map":
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    mkdir -p tmp/launch
    echo "Dumping Autoware planning simulator (map: {{ map_path }})..."
    play_launch dump --output tmp/launch/autoware_record.json \
        launch autoware_launch planning_simulator.launch.xml \
        map_path:={{ map_path }}
    echo "Record written to tmp/launch/autoware_record.json"

# Filter play_launch record to remove sentinel-replaced nodes
filter-autoware: dump-autoware
    scripts/filter_autoware_record.sh tmp/launch/autoware_record.json tmp/launch/autoware_record_filtered.json

# Launch baseline Autoware planning simulator (unmodified)
[arg("record", long="record", value="true")]
[arg("drive", long="drive", value="true")]
[arg("timeout", long="timeout")]
[arg("poses", long="poses")]
launch-autoware-baseline $record="false" $drive="false" $timeout="120" $poses="scripts/poses.yaml": dump-autoware
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    export ZENOH_SESSION_CONFIG_URI="$(pwd)/{{ session_config }}"

    JOBS=(
        '{{ zenohd }} --config {{ router_config }} < /dev/null'
        'play_launch replay --input-file tmp/launch/autoware_record.json --web-addr 0.0.0.0:8080'
    )

    if [ "$record" = "true" ]; then
        JOBS+=("scripts/record_bag.sh baseline")
    fi

    if [ "$drive" = "true" ]; then
        JOBS+=("sleep 90 && python3 scripts/auto_drive.py --timeout $timeout --poses $poses")
    fi

    echo "=== Baseline Autoware ==="
    parallel --line-buffer --halt now,done=1 --delay 2 ::: "${JOBS[@]}"

# Launch filtered Autoware + sentinel (14 nodes replaced by sentinel binary)
[arg("record", long="record", value="true")]
[arg("drive", long="drive", value="true")]
[arg("timeout", long="timeout")]
[arg("poses", long="poses")]
launch-autoware-sentinel $record="false" $drive="false" $timeout="120" $poses="scripts/poses.yaml": filter-autoware build-sentinel-linux
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    export ZENOH_SESSION_CONFIG_URI="$(pwd)/{{ session_config }}"

    FILTERED=tmp/launch/autoware_record_filtered.json
    SENTINEL="$(pwd)/target/debug/autoware_sentinel_linux"

    JOBS=(
        '{{ zenohd }} --config {{ router_config }} < /dev/null'
        "play_launch replay --input-file $FILTERED --web-addr 0.0.0.0:8080"
        "$SENTINEL"
    )

    if [ "$record" = "true" ]; then
        JOBS+=("scripts/record_bag.sh sentinel")
    fi

    if [ "$drive" = "true" ]; then
        JOBS+=("sleep 90 && python3 scripts/auto_drive.py --timeout $timeout --poses $poses")
    fi

    echo "=== Autoware + Sentinel ==="
    parallel --line-buffer --halt now,done=1 --delay 2 ::: "${JOBS[@]}"

# ════════════════════════════════════════════════════════════════════
# Quality & verification
# ════════════════════════════════════════════════════════════════════

# Format all packages
format:
    cargo fmt --all

# Check formatting on all packages
format-check:
    cargo fmt --all -- --check

# Cross-compile check.
#
# 1. Algorithm crates + autoware_sentinel_core (no controller-node) for
#    `thumbv7em-none-eabihf` — proves the no_std + alloc surface still
#    compiles for Cortex-M4F class targets.
# 2. autoware_sentinel_core with `controller-node` for the same triple
#    — catches feature-gated code that breaks the embedded path.
#
# `autoware_sentinel_linux` (std-only) is excluded; the sibling
# cross-target binaries (zephyr / freertos / nuttx) are excluded too —
# they each have their own toolchain pin and run via `just
# build-sentinel-{zephyr,freertos,nuttx}`.
cross-check:
    # Algorithm crates only — no nros / no platform features needed.
    # `ivc-bridge` is a Linux-only daemon (clap / env_logger), exclude
    # from the cross target.
    cargo check --workspace --exclude autoware_sentinel_linux --exclude autoware_sentinel_core --exclude ivc-bridge --target thumbv7em-none-eabihf
    # Core no-controller (FreeRTOS profile).
    cargo check -p autoware_sentinel_core --no-default-features --features platform-zephyr --target thumbv7em-none-eabihf
    # Core with controller-node (Linux dev profile, but checked for thumb).
    cargo check -p autoware_sentinel_core --no-default-features --features platform-zephyr,controller-node --target thumbv7em-none-eabihf

# CI: format-check, cross-check, and test
ci: format-check cross-check test

# Run Kani verification on all harness crates
verify-kani:
    parallel --tag --line-buffer --halt now,fail=1 \
      'cd src/{} && cargo kani' ::: {{ kani_packages }}

# Run Verus verification
verify-verus:
    cd src/verification && ~/.verus/verus-main/source/target-verus/release/verus src/lib.rs

# Run all verification
verify: verify-kani verify-verus

# ════════════════════════════════════════════════════════════════════
# Utilities
# ════════════════════════════════════════════════════════════════════

# Clean workspace build artifacts
clean:
    cargo clean

# Setup TAP network for Zephyr native_sim (requires sudo)
setup-tap-network:
    sudo scripts/zephyr/setup-network.sh

# Tear down TAP network (requires sudo)
teardown-tap-network:
    sudo scripts/zephyr/setup-network.sh --down

# Capture initial + goal poses from RViz and save to a file
capture-poses output="scripts/poses.yaml":
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    python3 scripts/capture_poses.py -o {{ output }}

# Run autonomous drive sequence (init pose → route → engage → wait for arrival)
auto-drive timeout="120" poses="scripts/poses.yaml":
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    python3 scripts/auto_drive.py --timeout {{ timeout }} --poses {{ poses }}
