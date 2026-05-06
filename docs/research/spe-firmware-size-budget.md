# SPE firmware size budget — per-component breakdown

Investigation of the 291 KB SafetyIsland-wired `spe.elf` (256 KB BTCM
budget + ~35 KB overflow) into its underlying components: NVIDIA driver
layer, FreeRTOS RTOS, sentinel application (Rust), and the supporting
runtime (newlib, zenoh-pico, nros executor).

Source-based study performed 2026-05-06 against
nano-ros-sentinel `717b75b1` and autoware_sentinel `d1d9524`. Numbers
combine `nm --size-sort` measurements from prior commits with line-of-
code estimates (~1.2 B/LoC for `-Os` Cortex-R5F C output) where a
relink is unavailable.

## Headline

| Layer            | Estimated size (KB)  | Confidence    |
|------------------|----------------------|---------------|
| NVIDIA BSP drivers | ~28–32             | medium-high   |
| FreeRTOS V10.4.3 (text+data) | ~19        | low-medium    |
| FreeRTOS stacks + heap (RAM) | ~30        | medium        |
| newlib post-shim | ~5–8                 | medium        |
| nros executor + rmw-zenoh | ~45–60      | low-medium    |
| zpico (zenoh-pico C) | ~56–68           | low-medium    |
| Sentinel algorithm (Rust) | ~15–25 *     | low           |
| Static data / msg codecs / rodata | ~5–10 | low           |
| **Total**        | **~256 ± 30 KB**     | matches 291 KB measured |

\* Most sentinel algorithm code is already accounted for inside the
nros `app_task_entry` / `timer_try_process` monomorphizations (the
`wire_executor` body inlines into both). Counting it twice would
double-book.

## Per-layer detail

### NVIDIA BSP drivers (~28–32 KB)

From `arm-none-eabi-nm --size-sort` on the prior overflow build, named
in `docs/roadmap/11-orin-spe.md:679`:

| Driver        | Size  | Role |
|---------------|-------|------|
| `tegra-ast`   | 9.3 KB | AST aperture / DRAM mapping |
| `hsp-tegra`   | 4.8 KB | HSP doorbell / mailbox (CCPLEX↔SPE signaling) |
| `uart-tegra`  | 2.7 KB | TCU console + `tcu_print_msg` (panic / boot banner) |
| `lic-tegra`   | 1.7 KB | Local Interrupt Controller |
| **Named**     | **18.5 KB** | |
| `main_task` + boot + IVC channel API + spe-vic / spe-pm / BPMP IPC | ~10–14 KB | residual fixed cost; only IVC + 12 `tegra_ivc_*` symbols actually exercised (`packages/drivers/nvidia-ivc/src/fsp.rs:49-64`) |

**Reduction headroom: zero without DRAM relocation.** These drivers
live in BTCM because the boot path needs them before AST can map DRAM
to a relocatable region. Recovering them needs Phase 11.3.E Option A
(linker split: vector table + IRQ handlers in BTCM, bulk `.text +
.rodata` in DRAM through AST).

### FreeRTOS V10.4.3 kernel (~19 KB code, ~30 KB RAM)

NVIDIA's BSP compiles FreeRTOS V10.4.3 from
`$BSP/FreeRTOSV10.4.3/FreeRTOS/Source` (vendored in the BSP, not from
the nano-ros third-party tree). Source upstream is identical so the
LoC numbers are representative.

| Source file    | LoC  | Estimated `.text+.data` |
|----------------|------|-------------------------|
| `tasks.c`      | 8861 | ~10.5 KB |
| `queue.c`      | 3387 | ~4.0 KB  |
| `timers.c`     | 1343 | ~1.6 KB  |
| `event_groups.c` | 887 | ~1.0 KB |
| `list.c`       | 248  | ~0.3 KB  |
| `port.c` + `portASM.S` (ARM_CR5) | 1028 | ~1.5 KB |
| `heap_4.c`     | 638  | ~0.8 KB code + heap pool |
| **Subtotal**   | **~16 KLoC** | **~19 KB** |
| Per-task stacks (5 tasks × ~6 KB) + heap pool | | **~30 KB RAM** |

`stream_buffer.c` and `croutine.c` are `--gc-sections`'d (no app
references). Stacks + heap come from the FreeRTOS RAM pool, sized
per `docs/roadmap/11-orin-spe.md:97`.

**Reduction headroom: zero.** NVIDIA's FSP ships FreeRTOS pre-built
inside `tegra_aon_fsp.a` — we don't control the compile.

### newlib (~5–8 KB after shim)

The recovery ledger booked **17 KB** of newlib cuts via the
`vsniprintf` shim
([`packages/boards/nros-board-orin-spe/c/printf_shim.c:1-77`](https://github.com/NEWSLabNTU/nano-ros/blob/main/packages/boards/nros-board-orin-spe/c/printf_shim.c)),
which redirects every `printf` / `vsnprintf` / `vprintf` call site to
newlib's integer-only formatter. Pre-shim cost was ~25 KB (the float
formatter chain pulls `_dtoa_r`, `fmaf128`, `__divtf3`, `__addtf3`,
`__multf3`, `lgamma_r`, …). Post-shim residual: integer formatter +
`memcpy` / `memset` / `strlen` / `errno` / minimal `_sbrk` stubs.

**Reduction headroom: ~2–3 KB** with custom `memcpy` / `memset` and
dropping `_sbrk` — not worth the risk of introducing subtle bugs in
hot-path memory primitives.

### nros executor + rmw-zenoh (~45–60 KB)

Two named monomorphizations dominate the residual list:

| Symbol | Size | Source |
|--------|------|--------|
| `app_task_entry::<…closure#0, _>` | 8.5 KB | `packages/boards/nros-board-orin-spe/src/node.rs:57` — Rust trampoline + inlined `Executor::open + wire_executor + spin` body |
| `timer_try_process::<…closure#4>` | 5.5 KB | `packages/core/nros-node/src/executor/arena.rs:660` — 30 Hz tick body inlined per closure type |
| **Named**                          | **14 KB** | |
| Non-generic `spin` / `spin_once` / handle table / type-erased dispatch + `nros-rmw-zenoh` shim | ~30–45 KB | `executor/spin.rs` 2393 LoC + `arena.rs` 1391 + `handles.rs` 2556 + `node.rs` 578 + `rmw-zenoh/src/shim/*` ~7 KLoC |

**Reduction headroom: 9–12 KB** via the type-erasure work documented
in `docs/research/nano-ros-spe-size-opportunities.md`:

1. Type-erase `app_task_entry<F, E>` to `fn(*mut c_void)` (~6 KB
   combined with sentinel-side closure split).
2. `add_timer_boxed` + non-generic dispatch (~1–2 KB).
3. Per-(M, F) sub/service dedup via `dyn-callbacks` feature (~2–4 KB).

### zpico (zenoh-pico C, ~56–68 KB)

Compiled with `Z_FEATURE_LINK_IVC=1` and **everything else off**
(`Z_FEATURE_LINK_TCP/UDP/SERIAL/TLS/RAWETH/SCOUTING_UDP=0`,
`packages/zpico/zpico-sys/build.rs:1660-1830`). Wire size limits:
`Z_BATCH_UNICAST_SIZE=1024`, `Z_FRAG_MAX_SIZE=2048`
(`zpico-sys/c/platform/zenoh_generic_config.h:20-21`).

| Subcomponent                          | Estimated size |
|---------------------------------------|----------------|
| Core (api / collections / link / net / protocol / session / transport / utils + freertos system.c + zpico.c shim) | ~50–60 KB |
| Static slot arenas: 8 pubs / 4 subs / 2 queryables / 16 liveliness / 256 B sub buf / 256 B service buf | ~6–8 KB |

The 69 KB win in the recovery ledger came from right-sizing those
arenas (`justfile:325-333`). Pre-recovery they were 56/16/32/96 with
~70 KB BTCM cost.

**Reduction headroom: ~3–5 KB** by trimming unused protocol state
machine paths (e.g. multi-fragment reassembly when `Z_FRAG_MAX_SIZE`
already fits one batch). Not a feature-flag cut.

### Sentinel algorithm (Rust)

Workspace algorithm crates pulled by `comp-mrm + comp-engagement`:

| Crate                                  | LoC  |
|----------------------------------------|------|
| `autoware_vehicle_cmd_gate`            | 1421 |
| `autoware_mrm_handler`                 | 506  |
| `autoware_stop_filter`                 | 384  |
| `autoware_mrm_emergency_stop_operator` | 275  |
| Others (mrm_comfortable_stop, shift_decider, vehicle_velocity_converter, heartbeat_watchdog) | ~2400 |
| **Subtotal** | **~5 KLoC** |

Most of this code lives inside the
`app_task_entry::<…closure#0>` (8.5 KB) and
`timer_try_process::<…closure#4>` (5.5 KB) monomorphizations —
counted in the nros executor row above to avoid double-booking. The
marginal cost on top of those monos is dominated by message codec
instantiations and per-algorithm state structs (~15–25 KB,
low-confidence estimate).

`compact-trig` (Phase 11.3.C, this branch) cuts ~2.3 KB by replacing
`libm::tanf` / `libm::atanf` in the cmd-gate filter with Padé
approximations.

**Reduction headroom: ~5–10 KB** by switching every `f64`-typed
covariance / state field to `f32` in the SPE feature set, dropping
the `__divdf3` / `__muldf3` instantiations the +vfp3 work didn't
fully cover. Cross-cutting; not scoped to one crate.

### Static data / rodata / message codecs (~5–10 KB)

Per-publisher TypeSupport tables, type-name strings, `Default`
implementations for covariance arrays, `compact-trig` Padé tables.
Hard to bound from sources alone — needs `nm` on the staticlib post-
relink.

## Recovery summary

The recovery ledger so far (143 → 35 KB overflow, ~108 KB recovered):

| Source                | Saving    | Status   |
|-----------------------|-----------|----------|
| zpico arena right-size| 69 KB     | landed   |
| Drop `nros/param-services` | 6 KB | landed   |
| `vsniprintf` printf shim | 17 KB  | landed   |
| Rust `+vfp3,+d32` target features | 10 KB | landed |
| `-Cpanic=immediate-abort` | 4 KB  | landed   |
| `compact-trig` Padé `tan/atan` | ~2.3 KB | landed (this branch) |

Remaining levers (~21 KB recoverable; **~14 KB still needs DRAM
relocation**):

| Source                | Estimated  | Status   |
|-----------------------|------------|----------|
| Type-erase `app_task_entry` + sentinel closure split | ~6 KB | tracked in `nano-ros-spe-size-opportunities.md` |
| `add_timer_boxed` + non-generic dispatch | ~1–2 KB | tracked |
| `dyn-callbacks` for sub/service | ~2–4 KB | tracked |
| Drop residual `f64` math in SPE crates | ~5–10 KB | scoped, not started |
| Trim zpico fragment-reassembly tail | ~3–5 KB | scoped, not started |
| **DRAM relocation (Phase 11.3.E Option A)** | ~18 KB BSP fixed cost | required to fully close |

## What's measurable from sources alone, what isn't

**Measurable:** named-symbol numbers (BSP drivers, two monos),
recovery ledger steps, FreeRTOS LoC × density rule, zpico arena slot
sizes.

**Not measurable without a relink:** non-generic Executor body,
algorithm crate residual on top of monos, message codec rodata,
post-shim newlib floor. The 9.3 / 4.8 / 2.7 / 1.7 KB BSP driver
numbers and the 8.5 / 5.5 KB nros monomorphizations are the
high-confidence anchors; everything else is scaled against them.

A `cargo bloat` or `nm --size-sort` run on a fresh
`spe.elf --features safety-island` build would tighten every
low-confidence row. Not blocked on hardware — only on the BSP
download (~150 MB toolchain + the L4T public sources tarball).
Tracked as part of Phase 11.3 acceptance criteria
(`docs/roadmap/11-orin-spe.md:740`: "add `just orin_spe-bloat-report`
recipe once a build path successfully fits BTCM").
