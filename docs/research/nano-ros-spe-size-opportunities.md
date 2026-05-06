# nano-ros code-size opportunities for SPE BTCM

Background for the residual 35–37 KB BTCM overflow on the SafetyIsland-wired
SPE firmware build (Phase 11.3.C). Captures the read-only audit performed on
nano-ros-sentinel main (`717b75b1`) on 2026-05-06. Source of truth for the
follow-up upstream PRs.

## What's already cut

Cumulative (Phase 11.3.D):

| Step                                                   | overflow |
|--------------------------------------------------------|----------|
| Baseline (`ffi-size-markers` already gated)            | 143 KB   |
| ZPICO_MAX_*/NROS_SUBSCRIPTION_BUFFER_SIZE right-size   | 74 KB    |
| Drop `nros/param-services` from sentinel_core          | 68 KB   |
| `vsniprintf` shim (drop newlib float formatter)        | 51 KB   |
| Rust `+vfp3,+d32` target features                      | 41 KB   |
| `-Cpanic=immediate-abort`                              | 37 KB   |
| `compact-trig` Padé `tan/atan` in cmd_gate (11.3.C)    | ~35 KB *(est.)* |

## Remaining cost (per `arm-none-eabi-nm --size-sort` on spe.elf)

Three groups:

1. **Rust closure-body inlines (~14 KB)** — fixable via API changes on both nano-ros and the sentinel firmware crate.
2. **NVIDIA BSP fixed cost (~9 KB)** — `tegra-ast`, `hsp-tegra`, `uart-tegra`, `lic-tegra`. Out of scope; only DRAM mapping (Phase 11.3.E Option A) recovers it.
3. **Other** (~12 KB) — distributed; covered indirectly by closure-body work.

This note covers group 1.

## Confirmed targets

### 1. `app_task_entry<F, E>` — 8.5 KB monomorphization

**Where.**
[`packages/boards/nros-board-orin-spe/src/node.rs:57-104`](https://github.com/NEWSLabNTU/nano-ros) (entry shim) and `:140-198` (`run`).

```rust
unsafe extern "C" fn app_task_entry<F, E>(arg: *mut c_void)
where
    F: FnOnce(&Config) -> Result<(), E>,
    E: Debug;

pub fn run<F, E>(config: Config, f: F)
where
    F: FnOnce(&Config) -> Result<(), E> + 'static,
    E: Debug + 'static;
```

**Smoking gun.** `xTaskCreate` already takes a `void*`, so the C ABI is
type-erased. The Rust trampoline is generic over both `F` and `E` and calls
`closure(&ctx.config)` directly plus `println!("{e:?}")` directly. Every byte
of `nros_app_rust_entry::{closure#0}` (which inlines `Executor::open +
wire_executor + executor.spin`) ends up in one specialization. Since SPE
links exactly one app, the generic provides no value.

**Proposed pivot.** Type-erase at the boundary:

```rust
pub fn run(config: Config, f: fn(&Config) -> Result<(), AppError>);
```

Or keep the generic alias but funnel everything through a single
non-generic `app_task_entry_dyn(arg: *mut c_void)` that invokes a
`Box<dyn FnOnce(&Config) -> Result<(), DynError>>`.

**Estimated saving.** ~6 KB combined with the sentinel-side companion
change (split `wire_executor` out of the FnOnce closure so the body is
addressable as a plain `fn`, not a capture). **Confidence: medium** —
much of the 8.5 KB is the inlined `wire_executor` body; type erasure
only helps when LTO can't share the body across specializations.

**API break.** `run`'s signature changes. One call site in
`src/sentinel_spe_firmware/src/lib.rs:85`. Trivial to update.

### 2. `timer_try_process<F>` — 5.5 KB inline

**Where.**
[`packages/core/nros-node/src/executor/arena.rs:660-687`](https://github.com/NEWSLabNTU/nano-ros)
and `executor/spin.rs:1002-1039`.

```rust
pub(crate) unsafe fn timer_try_process<F>(ptr: *mut u8, delta_ms: u64) -> Result<bool, TransportError>
where F: FnMut();

pub fn add_timer<F>(&mut self, period: TimerDuration, callback: F) -> Result<HandleId, NodeError>
where F: FnMut() + 'static;
```

**Smoking gun.** `TimerEntry<F>` stores the closure inline in the
arena. `timer_try_process::<F>` is monomorphized per-F and inlines
`(entry.callback)()`. SafetyIsland's 30 Hz tick (the
`wire_executor::{closure#4}` body — heartbeat watchdog poll + cmd_gate
filter + MRM operators + publish chain) is 5.5 KB inside one F.

**Proposed pivot.** Borrow the pattern already proven by
`RawSubscriptionCallback` (`types.rs:560`):

```rust
pub(crate) struct TimerEntry {
    period_ms: u32,
    elapsed_ms: u32,
    cb: fn(*mut c_void),
    ctx: *mut c_void,
}
```

Plus a typed wrapper: `add_timer_boxed<F: FnMut() + 'static>(period,
Box<F>)` that does `Box::into_raw` and stores a tiny `fn(*mut c_void)`
shim that calls `(&mut *(ctx as *mut F))()`. The shim is per-F but only
~10 bytes; the heavy 5.5 KB body still has one instance, but its
codegen unit no longer cross-pollutes the dispatch loop.

**The bigger win is sentinel-side** — break the tick body into a
non-`#[inline]` plain `fn tick(state: &mut Island, now_ms: u64)` so
the closure passed to `add_timer` is just a thin wrapper.

**Estimated saving from nros side alone.** ~1–2 KB.
**Confidence: low–medium.** The 5.5 KB is the body itself, not the
dispatch wrapper.

**API break.** Additive if added as `add_timer_boxed` alongside
`add_timer`.

### 3. Subscription / service per-(M, F) monomorphization

**Where.** `executor/arena.rs:540-620, 626-646`,
`executor/spin.rs:533-640, 927-992`.

```rust
unsafe fn sub_buffered_try_process<M, F>(...)
where M: RosMessage, F: FnMut(&M);

unsafe fn srv_try_process<Svc, F, const REQ_BUF: usize, const REPLY_BUF: usize>(...)
where F: FnMut(&Svc::Request) -> Svc::Reply;
```

**Smoking gun.** Per-message-type monomorphization is unavoidable
(deserialize is per-type), but per-closure-type is an extra axis. With
~10 subs and ~20 services across ~8 distinct message/service types,
the F-axis explosion is real but each body is small (CDR deserialize
→ call → publish).

The C raw path already proves the pattern:
`sub_buffered_raw_c_try_process` (`arena.rs:495`) is non-generic and
uses `RawSubscriptionCallback = unsafe extern "C" fn(*const u8,
usize, *mut c_void)`.

**Proposed.** Mirror this for Rust:

```rust
pub fn add_subscription_dyn<M: RosMessage + 'static>(
    &mut self,
    topic: &str,
    cb: Box<dyn FnMut(&M) + 'static>,
) -> Result<HandleId, NodeError>;
```

**Estimated saving.** Plausibly 2–4 KB across 30 callsites.
**Confidence: low.** Wouldn't push without per-symbol nm dump.

**API break.** Additive.

## Refuted hypotheses

### Const-generic Executor capacity — false

`pub struct Executor` (`spin.rs:229-260`) has **no const generics**.
Capacity is set via the `nros-sizes-build` `links = "nros_node"`
build-time constants. One copy of `spin_once` per build. CLAUDE.md's
`Executor<_, 8, 8192>` reference is stale — current API is
`Executor::open(&config) -> Result<Self, NodeError>`.

### `xTaskCreate` adapter — already type-erased on the C side

The C ABI shim already takes a `void*`. Monomorphization is purely
Rust-side; covered by item 1.

### `register_parameter_services` — already gated off

`spin.rs:1810` is `#[cfg(feature = "param-services")]`. Sentinel
firmware doesn't enable it.

## What doesn't exist yet

No `boxed-callbacks` / `dyn-callbacks` cargo feature on `nros-node`
(grep across `packages/`). Existing features cover transport / platform
/ services only. **Proposal:** add `dyn-callbacks` that flips
`add_timer / add_subscription / add_service` from generic-F to
`Box<dyn FnMut(...)>`. Pivot point is one struct field per entry
type plus one dispatch fn each. Boxes go through `pvPortMalloc` —
already linked, BTCM is the constraint not the heap.

## Punch list — priority order

| # | Where | Action | Saving | Confidence | Sentinel-side change? |
|---|-------|--------|--------|------------|----------------------|
| 1 | `nros-board-orin-spe/src/node.rs:57,140` | Type-erase `run<F,E>` / `app_task_entry<F,E>` → fn-pointer + void* | up to 6 KB *(combined with sentinel-side closure split)* | medium | yes |
| 2 | `nros-node/src/executor/arena.rs:660`, `spin.rs:1002` | `add_timer_boxed` + non-generic dispatch | 1–2 KB | low-medium | yes (split tick into named `fn`) |
| 3 | `nros-node/Cargo.toml:11`, arena/spin | New `dyn-callbacks` feature, additive `add_*_dyn` variants | 2–4 KB | low | no |

**Net.** All three items together plausibly recover ~9–12 KB of the
remaining ~35 KB. Closes maybe one third of the gap. The rest is
dominated by:

- BSP fixed cost (~9 KB) — needs DRAM mapping (Option A).
- Generic body itself (`wire_executor` inline + tick body) — only
  shrinks via algorithm-level cuts, not dispatch refactoring.

## Reading order for the upstream PRs

1. **Land item 1 first** in nano-ros — small, isolated, the
   sentinel-side companion change is a five-line edit in
   `sentinel_spe_firmware::nros_app_rust_entry`.
2. **Sentinel-side: split `wire_executor` tick body** into a non-inline
   named function pulled out of the FnMut closure. Verify the symbol
   appears once in the staticlib's `nm` output.
3. **Item 2** — `add_timer_boxed`. Trivial after step 2.
4. **Item 3** is exploratory; only land if a follow-up `nm --size-sort`
   shows the F-axis is still expensive enough to justify the API
   surface area.

## Files cited

- `~/repos/nano-ros-sentinel/packages/boards/nros-board-orin-spe/src/node.rs`
- `~/repos/nano-ros-sentinel/packages/core/nros-node/src/executor/arena.rs`
- `~/repos/nano-ros-sentinel/packages/core/nros-node/src/executor/spin.rs`
- `~/repos/nano-ros-sentinel/packages/core/nros-node/src/executor/types.rs`
- `~/repos/nano-ros-sentinel/packages/core/nros-node/Cargo.toml`
- `~/repos/autoware_sentinel/src/sentinel_spe_firmware/src/lib.rs`
- `~/repos/autoware_sentinel/src/autoware_sentinel_core/src/lib.rs`
  (lines 504, 728–1029 — `wire_executor` and the 30 Hz closure)
