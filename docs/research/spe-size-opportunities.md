# SPE size optimization opportunities

Exhaustive sweep across all firmware layers for code-size reductions
not already documented elsewhere. Companion to:

- `nano-ros-spe-size-opportunities.md` — three nano-ros pivot points
  (closure type-erasure work, ~9–12 KB).
- `spe-firmware-size-budget.md` — per-component size breakdown.
- `docs/roadmap/11-orin-spe.md §11.3.C` — already-landed cuts ledger.

Read-only audit performed 2026-05-06. Ordered by KB-per-effort.

## Status

| # | Layer | Action | Est. saving | Status |
|---|-------|--------|-------------|--------|
| 1 | zpico build flags | `Z_FEATURE_AUTO_RECONNECT 0` + `Z_FEATURE_ENCODING_VALUES 0` (gated behind `CARGO_FEATURE_ORIN_SPE`) | 3–5 KB | **landed** in nano-ros `93a51cf8` |
| 2 | zpico build flags | `Z_FEATURE_LIVELINESS 0` | 6–8 KB | **REFUTED** — zpico.c declares tokens for every handle (`packages/zpico/zpico-sys/c/zpico/zpico.c:1266`); rmw_zenoh_cpp uses them for ROS 2 entity discovery. Disabling breaks discovery. |
| 3 | Algorithm crates | f32 sweep behind `f32-math` cargo feature | 0 KB | **REFUTED** — see §B-revised below |

**Net new recovery: 3–5 KB landed.** The 17–26 KB earlier estimate
collapsed to 3–5 KB once §A.3 and §B were source-audited.

## Detailed findings

### A — zenoh-pico C protocol state machine

**A.1 — `Z_FEATURE_AUTO_RECONNECT 0`** (currently set to `1` in
`packages/zpico/zpico-sys/build.rs` `use_orin_spe` branch). The IVC
link doesn't disconnect — there's no transport-level reconnect path
for a fixed-frame mailbox. Dead code on SPE. **Save: 1–2 KB.**
Cross-cut: SPE-only branch in `build.rs`, no impact on other targets.

**A.2 — `Z_FEATURE_ENCODING_VALUES 0`** (currently `1`).
`packages/zpico/zpico-sys/zenoh-pico/src/api/encoding.c` is 267 LoC
of named-encoding constants (`text/plain`, `application/json`, …).
ROS-over-Zenoh uses CDR; encoding-name strings are not consulted.
**Save: 2–3 KB rodata + code.** Cross-cut: SPE-only.

**A.3 — `Z_FEATURE_LIVELINESS 0` — REFUTED.** Audit found
`packages/zpico/zpico-sys/c/zpico/zpico.c:1266`:

```c
int lv_ret = z_liveliness_declare_token(z_session_loan(&g_session), ...)
```

Every handle (publisher / subscriber / service) declares a liveliness
token at registration. **rmw_zenoh_cpp uses these tokens for ROS 2
entity discovery** — without them, sentinel's publishers and services
become invisible to the Autoware side. Cannot disable without rewriting
zpico.c to skip token declarations + a parallel discovery scheme.

**A.4 — `Z_FEATURE_QUERY_CLIENT 0` (new sub-feature, upstream patch)**.
Sentinel issues zero `z_get` calls. Both the send path
(`_z_query_send` in `api.c`) and the partial/final reply triggers
(`session/query.c:144-260`) link in. Adding a sub-feature inside
zenoh-pico would need an upstream patch. **Save: ~3–5 KB.**
Confidence: low (needs source patch).

**A.5 — Disable `Z_FEATURE_QUERYABLE` / `Z_FEATURE_QUERY` —
REFUTED.** `add_service` (22 sites in sentinel) is implemented as
`z_declare_queryable` in `nros-rmw-zenoh/src/shim/service.rs`;
replies use `_z_reply_send`. Cannot disable.

### B-revised — f64 → f32 sweep — REFUTED for SPE

The original 6–10 KB estimate was wrong. Source audit:

**B.1 — `autoware_universe_utils` (host of `libm::sin / cos / asin /
atan2 / sqrt / fabs`) is REFUTED.** Pulled only by
`autoware_motion_utils`, `autoware_pid_longitudinal_controller`,
`autoware_trajectory_follower_node` — all gated behind the
`controller-node` cargo feature, **off in the SPE default build**
(`autoware_sentinel_spe/Cargo.toml:58` activates `comp-mrm` +
`comp-engagement` only). LTO + `--gc-sections` already drop the
crate. **Save: 0 KB.**

**B.2 — `autoware_twist2accel/src/lib.rs:8-17` Lowpass — REFUTED.**
The `gain: f64` + `value: Option<f64>` arithmetic looks like a
candidate, but post-`+vfp3,+d32` Cortex-R5F has hardware double-
precision FPU. `__adddf3` / `__muldf3` / `__divdf3` no longer link
in (already eliminated by 11.3.D). Hardware `vadd.f64` / `vmul.f64`
is one instruction — same code size as the f32 equivalent.
**Save: 0 KB.**

**B.3 — `autoware_mrm_handler/src/lib.rs:25,31,77,100` — REFUTED for
the same reason as B.2.** Compare-only operations on f64 fields;
hardware FPU does them in one cycle. **Save: 0 KB.**

**B.4 — `autoware_motion_utils` — REFUTED.** Gated behind
`controller-node`, off in default SPE build.

**B.5 — `autoware_control_validator` — REFUTED.** Gated behind
`comp-validator`, off in default SPE build.

**Why the audit was wrong the first time.** The 8 unique double-libm
symbols × 700–1500 B = 6–10 KB estimate was anchored on
`autoware_universe_utils`'s libm calls — but `universe_utils` is dead
code in the SPE binary. The active SPE crates (`twist2accel`,
`stop_filter`, `vehicle_velocity_converter`, `mrm_*`,
`heartbeat_watchdog`, `vehicle_cmd_gate`) collectively have **zero
`libm::*` (non-`f`-suffixed) call sites**. The only libm calls in
the SPE hot path are `libm::tanf` and `libm::atanf` in
`vehicle_cmd_gate::filter::calc_lateral_accel` —  already replaced
by Padé approximations under the `compact-trig` feature in
Phase 11.3.C.

The lesson: estimating size-cut levers from line counts on dead-coded
crates produces phantom KBs. The correct measurement is `nm
--size-sort` against a real link, gated on the actual feature surface.

### C — Message codec rodata duplication — MOSTLY REFUTED

Each generated message has a `TYPE_NAME` + `TYPE_HASH` const string
(e.g. `geometry_msgs/src/msg/pose.rs:33-36`). The shared
`&'static str` is deduped by the linker — `Pose` instances point at
one rodata copy. **No duplication.**

Single residual: stripping the `"TypeHashNotSupported"` `TYPE_HASH`
strings (~22 B × ~32 crates) saves ~0.5 KB rodata. Not worth the
diff churn.

### D — nros handle-table dispatch dedup — REFUTED

`executor/spin.rs:1665-1692` already vtable-dispatches via
`meta.try_process` function pointer (no per-type match arm). The
`match` on `EntryKind` is statistics counters; collapses to integer
increment under LTO. The closure-axis dedup (covered in
`nano-ros-spe-size-opportunities.md`) is the only remaining lever
here.

### E — Sentinel state struct packing

**E.1 — `current_velocity: f64` (autoware_sentinel_core/src/lib.rs:221)
→ f32.** Simple field flip. **Save: 4 B BSS + propagates the f32
sweep from §B.** Trivial; do as part of the f32-math feature.

**E.2 — `accel_covariance: [f64; 36]` (lib.rs:225)** = 288 B BSS,
mirrored from `geometry_msgs::AccelWithCovariance`. Wire format must
remain `f64`. **Action:** consider not caching the field — repopulate
on demand from the publish path. **Save: 288 B BSS** (not flash).
Low priority unless RAM becomes the bound.

### F — Rust derive-macro generated code

**F.1 — `derive(Debug)` on every generated message — REFUTED.**
`panic=immediate-abort` already drops `core::fmt::Debug` formatter
machinery. With no caller of `{:?}` and `--gc-sections`, the impls
drop. Spot-check confirmed `Pose` has no `fmt::Debug` impl in the
binary post-LTO.

**F.2 — `derive(Clone)` on messages with `heapless::Vec<T, N>`
fields — REFUTED.** Could emit large clone loops, but
`SafetyIsland` (`autoware_sentinel_core/src/lib.rs:213-280`) caches
only fixed-size scalars (`Twist`, `Accel`, `Control`,
`AutowareState`, `GearReport`). No `MarkerArray` / `Trajectory`
clone in the default SPE build.

### G — NVIDIA BSP driver dead code — REFUTED

`nros-board-orin-spe/build.rs` only compiles `c/printf_shim.c` and
links prebuilt `libtegra_aon_fsp.a`. No driver init source compiles
in this repo. The 28–32 KB BSP budget is FSP archive members;
recovery requires either DRAM relocation (Option A) or rebuilding
FSP from NVIDIA source — out of scope.

### H — zpico-sys build flags audit

Currently in the `use_orin_spe` branch
(`packages/zpico/zpico-sys/build.rs`):

| Flag                        | Value | Used? | Action |
|-----------------------------|-------|-------|--------|
| `Z_FEATURE_LINK_IVC`        | 1     | Yes   | keep |
| `Z_FEATURE_LINK_TCP/UDP/SERIAL/TLS/RAWETH` | 0 | — | already off |
| `Z_FEATURE_SCOUTING_UDP`    | 0     | —     | already off |
| `Z_FEATURE_PERIODIC_TASKS`  | 0     | —     | already off |
| `Z_FEATURE_BATCHING`        | 0     | —     | already off |
| `Z_FEATURE_PUBLICATION`     | 1     | Yes   | keep |
| `Z_FEATURE_SUBSCRIPTION`    | 1     | Yes   | keep |
| `Z_FEATURE_QUERYABLE`       | 1     | Yes (services) | keep |
| `Z_FEATURE_QUERY`           | 1     | Yes (reply path) | keep |
| `Z_FEATURE_FRAGMENTATION`   | 1     | Yes   | keep (`Z_FRAG_MAX_SIZE=2048`) |
| `Z_FEATURE_LIVELINESS`      | 1     | **Verify** | candidate cut (§A.3) |
| `Z_FEATURE_ENCODING_VALUES` | 1     | No    | **candidate cut (§A.2)** |
| `Z_FEATURE_AUTO_RECONNECT`  | 1     | No    | **candidate cut (§A.1)** |
| `Z_FEATURE_TCP_NODELAY`     | 1     | No (no TCP) | candidate cut, low priority |

## What landed

**§A.1 + §A.2 — zpico flag flips.** Landed in nano-ros `93a51cf8`.
`Z_FEATURE_AUTO_RECONNECT 0` and `Z_FEATURE_ENCODING_VALUES 0` gated
behind `CARGO_FEATURE_ORIN_SPE` (set by `nros-board-orin-spe`).
Other platforms unchanged. E2E regression: `just orin_spe test`
4/4 PASS, autoware_sentinel `sentinel_spe` 2/2 PASS (validated
against local nano-ros-sentinel HEAD via path patches). **~3–5 KB
BTCM on the SPE build** (source-reviewed; relink against FSP
toolchain needed for exact delta).

**Pin bump pending.** The autoware_sentinel `[patch.crates-io]` block
is currently pinned at `nros@682f1404` and `nros@cbd18a0e` (SPE
crate). Upgrading to `93a51cf8` triggers a transitive
`colcon-nano-ros` submodule fetch that fails on the current registry
state — an unrelated upstream issue. Bump the pin in a follow-up
once that's resolved; the zpico cut takes effect on SPE
firmware builds at that point.

## What was refuted

§A.3 (LIVELINESS), §B (f32-math) — see above. The original
"17–26 KB recoverable" headline was based on those; once audited,
the realistic number drops to 3–5 KB landed.

## Remaining levers

| Source | Estimated | Status |
|--------|-----------|--------|
| nano-ros closure type-erasure (`app_task_entry` + timer + dyn-callbacks) | 9–12 KB | tracked in `nano-ros-spe-size-opportunities.md` |
| `compact-trig` Padé `tan/atan` in cmd_gate | 2.3 KB | landed (Phase 11.3.C) |
| zpico `AUTO_RECONNECT` + `ENCODING_VALUES` off | 3–5 KB | landed (this commit) |
| **DRAM relocation (Phase 11.3.E Option A)** | ~14–18 KB BSP fixed cost | required to close residual |

Total recoverable from feature-flag cuts: ~14–20 KB. Closes about
half the 35 KB overflow; the BSP fixed cost (~14 KB) genuinely needs
DRAM mapping.

## What's blocked / out of scope

- DRAM relocation (Option A) — already in roadmap as Phase 11.3.E.
  Recovers ~14–18 KB BSP fixed cost.
- NVIDIA driver pruning — needs FSP source rebuild. Not in this repo.
- FreeRTOS code reduction — non-negotiable, NVIDIA ships pre-built.
