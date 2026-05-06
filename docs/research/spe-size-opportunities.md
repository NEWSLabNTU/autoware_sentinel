# SPE size optimization opportunities

Exhaustive sweep across all firmware layers for code-size reductions
not already documented elsewhere. Companion to:

- `nano-ros-spe-size-opportunities.md` — three nano-ros pivot points
  (closure type-erasure work, ~9–12 KB).
- `spe-firmware-size-budget.md` — per-component size breakdown.
- `docs/roadmap/11-orin-spe.md §11.3.C` — already-landed cuts ledger.

Read-only audit performed 2026-05-06. Ordered by KB-per-effort.

## Top 3 actionable

| # | Layer | Action | Est. saving | Confidence | Effort |
|---|-------|--------|-------------|------------|--------|
| 1 | zpico build flags | `Z_FEATURE_AUTO_RECONNECT 0` + `Z_FEATURE_ENCODING_VALUES 0` | 3–5 KB | medium | 1 h |
| 2 | zpico build flags | `Z_FEATURE_LIVELINESS 0` (after verifying nros doesn't depend on it for service discovery) | 6–8 KB | medium | 4 h |
| 3 | Algorithm crates | f32 sweep of `autoware_universe_utils` + `autoware_twist2accel` + `autoware_mrm_handler` behind `f32-math` cargo feature | 8–13 KB | high | 1 day |

**Combined optimistic recovery: 17–26 KB.** Stacked with the documented
9–12 KB closure-erasure work, closes most of the 35 KB gap. Residual
fits DRAM relocation (Option A) cleanly.

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

**A.3 — `Z_FEATURE_LIVELINESS 0`** (currently `1`). Drops
`api/liveliness.c` (~150 LoC) + `session/liveliness.c` (404 LoC) +
the liveliness branches in `session/interest.c`. **Save: 6–8 KB.**
**Verify first:** `nros-rmw-zenoh/src/shim/session.rs` — does it call
`z_liveliness_declare_token`? rmw_zenoh discovery uses liveliness
tokens for node detection; if nros mirrors that, this is unsafe.
Audit before flipping.

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

### B — f64 → f32 sweep across SPE-targeted algorithm crates

233 `f64` / `libm::*` (non-`f`-suffixed) call sites across the SPE
feature set. Concentrated in three crates:

**B.1 — `autoware_universe_utils/src/lib.rs:24-196`.** Hosts
`libm::sin / cos / asin / atan2 / sqrt / fabs` (lines 132, 141, 142,
148–150, 165, 169, 172, 178). Used by every algorithm crate. Each
unique double-libm symbol is ~700–1500 B; ~8 unique calls in the
SPE-active surface. **Save: 6–10 KB** if entire crate switches to
`f32` + `libm::sinf` / `cosf` / `asinf` / `atan2f` / `sqrtf` /
`fabsf`. **Cross-cut: BREAKS Linux/Zephyr/NuttX builds** unless
gated. Pivot via `#[cfg(feature = "f32-math")]` or a
`pub type Real = f64;` alias the whole crate uses.

**B.2 — `autoware_twist2accel/src/lib.rs:8-17`.** `Lowpass` struct:
`gain: f64`, `value: Option<f64>`, plus arithmetic at line 17.
Active on SPE tick (called from cmd-gate path). **Save: 2–3 KB**
(eliminates `__adddf3` / `__muldf3` on this hot path; FPU helpers
exist for `f32` via `+vfp3,+d32`). Internal LPF can be `f32` while
the public covariance field stays `[f64; 36]` (ROS message schema
contract).

**B.3 — `autoware_mrm_handler/src/lib.rs:25,31,77,100`.**
`STOPPED_VELOCITY_THRESHOLD`, `current_velocity` field. Compare-only
(no inner-loop arithmetic). **Save: ~0.5 KB.**

**B.4 — `autoware_motion_utils/src/lib.rs:35-308` — REFUTED for
default SPE.** Pulled by `controller-node` only; gated off in
`sentinel_spe_firmware` defaults. LTO + `--gc-sections` already
drops it. Note for when the controller wires in.

**B.5 — `autoware_control_validator/*` — REFUTED.** Gated behind
`comp-validator`, off in default SPE build. Doesn't link.

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

## What to land first

1. **§A.1 + §A.2 — combined zpico flag flips.** Single PR against
   `nano-ros-sentinel/packages/zpico/zpico-sys/build.rs`. Two
   `cflag.define("Z_FEATURE_*", "0")` lines under the `use_orin_spe`
   branch. **3–5 KB, no behavior change, ~1 h work.** Verify with a
   relink + `arm-none-eabi-size build/spe.elf`.

2. **§B (f32-math feature)** — invasive but reversible. Add cargo
   feature on `autoware_universe_utils` (and propagate through
   `autoware_twist2accel`, `autoware_mrm_handler`). Pivot via type
   alias: `pub type Real = f64;` default, `f32` under feature. The
   feature flips on automatically when
   `autoware_sentinel_core/platform-orin-spe` is active (mirror the
   `compact-trig` pattern from Phase 11.3.C). **8–13 KB.**

3. **§A.3 — `Z_FEATURE_LIVELINESS 0`** only after auditing
   `nros-rmw-zenoh/src/shim/session.rs` for `z_liveliness_declare_token`
   uses. If safe, second-largest single cut available. **6–8 KB.**

After all three: **17–26 KB recovered.** Combined with the documented
9–12 KB closure-erasure work, total recoverable becomes 26–38 KB —
fully closes the 35 KB overflow on the optimistic end, leaves
~9–14 KB for DRAM relocation on the conservative end.

## What's blocked / out of scope

- DRAM relocation (Option A) — already in roadmap as Phase 11.3.E.
  Recovers ~14–18 KB BSP fixed cost.
- NVIDIA driver pruning — needs FSP source rebuild. Not in this repo.
- FreeRTOS code reduction — non-negotiable, NVIDIA ships pre-built.
