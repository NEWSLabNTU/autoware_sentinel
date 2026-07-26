# Phase 15: Fail-Loud Quality Gates

**Status:** Proposed
**Depends on:** Phase 14 (multi-node workspace migration)
**Motivation:** Phase 14 lost more time to *silent* failures than to real
defects. Every one of them shared a shape: a missing prerequisite or an
exceeded capacity produced an empty result or a hang instead of an error.
This phase makes those classes loud.

## The evidence (Phase 14, 2026-07-25/26)

Two failure families, nine incidents, every one of them mis-diagnosed at
first as a defect in code that turned out to be correct.

### Family A — absent prerequisite reported as success

| # | Incident | What we saw | What was true | Cost |
|---|----------|-------------|---------------|------|
| A1 | `rmw_zenoh_cpp` overlay missing (`external/rmw_zenoh_ws/install` dangled into a sibling checkout's build tree, wiped twice) | transport suite "14/14 passed" in 0.9 s | Suite never ran a single flow — `require_rmw_zenoh()` returned false, tests returned early | Hours, twice |
| A2 | Same overlay, host graph probes | `ros2 node list` empty against a healthy firmware | No RMW to answer with | ~3 h; sent us hunting a firmware bug that did not exist |
| A3 | Same overlay, second wipe mid-session | FreeRTOS "invisible", NuttX visible | Both were fine; the probe was dead | Chased as a lane asymmetry |
| A4 | `play_launch` uninstalled | planning tests failed at `dump` | Binary absent from PATH | Minutes (loud enough) |
| A5 | `zenohd` binary vanished with the sibling build tree | "router never listened" | Same wipe as A1 | Minutes |
| A6 | `zeth` TAP absent (Zephyr) | `Transport(ConnectionFailed)` | Wrong conclusion drawn ("needs sudo"); NSOS was the supported path | ~1 h + a wrong claim to the user |

### Family B — exceeded capacity reported as hang or opaque error

| # | Incident | What we saw | Actual cap |
|---|----------|-------------|------------|
| B1 | Executor callbacks (`NROS_EXECUTOR_MAX_CBS`, default 4) | `create_timer code=-6 Full`, later stack-overflow/malloc cascades | 4 slots vs ~50 needed |
| B2 | Executor node table (`NROS_EXECUTOR_MAX_NODES`, default 4) | `NodeTableFull` | 4 vs 13 |
| B3 | rmw-cffi subscription pool (hardcoded 4) | `SubscriberCreationFailed`, opaque | 4 vs 5 (upstream 0269) |
| B4 | FreeRTOS app-task stack (384 KiB default) | `*** STACK OVERFLOW ***` mid-registration | Needed 896 KiB (upstream 0274) |
| B5 | Zephyr zenoh shim slots (Kconfig 8/8/8/16) | Register pass hung, no output at all | 17 pubs on one node |
| B6 | Zephyr pthread mutex pool (32) | `z_declare_publisher: -1` on the 28th declare | ~27 usable |

Both families cost far more than the underlying fixes. The fixes were
usually one line; the *diagnosis* was hours of bisection.

## What to build

### 15.1 — Preflight: prerequisites hard-fail by default

The harness has four `require_*` helpers (`require_ros2_autoware`,
`require_zenohd`, `require_autoware_map`, `require_play_launch`) that
`eprintln!` and return `false`, and callers `return` — a pass.

- Add `sentinel_tests::preflight()`: one call, checks every prerequisite,
  **panics with a remediation line** ("rmw_zenoh_cpp not found — run
  `scripts/build_rmw_zenoh.sh`"), and runs from every test's fixture.
- Skipping stays possible but must be *chosen*: `SENTINEL_ALLOW_SKIP=1`
  for a laptop without Autoware. CI and the default local run never skip.
- Report skips in the summary line, so "0 tests ran" can never read as
  green.

### 15.2 — Environment integrity, not just presence

A1–A3 passed a naive "does the path exist" check for months (the symlink
existed; its target did not).

- Preflight resolves each artifact to a *working* one: `librmw_zenoh_cpp.so`
  loads, `rmw_zenohd --help` runs, `zenohd`/`play_launch` answer `--version`.
- No dependency on sibling checkouts' build trees. Phase 14 moved the
  overlay in-repo; codify that as a rule and add a check that fails if any
  `external/*` path resolves outside the repo.

### 15.3 — Capacity assertions at wiring time

Family B is one shape: a compile-time capacity smaller than the declared
topology, discovered at runtime as a hang.

- `autoware_sentinel_core` counts what it is about to declare (nodes,
  callbacks, publishers, subscriptions, services) and asserts against the
  compiled capacities **before** creating anything, with a message naming
  the knob and both numbers.
- Same idea belongs upstream (nano-ros 0257: derive the knobs from the
  model). Until then, the consumer-side assert converts six-hour hunts
  into one line of output.
- Add the MCU lanes' knobs to the assert list: zpico slots, liveliness
  tokens, per-platform stack/heap where readable.

### 15.4 — One probe path for every lane

`scripts/probe_mcu_graph.sh` (Phase 14.5) already boots a lane, waits for
its readiness marker, and queries nodes/topics/data with the RMW pinned.

- Extend to the native and Zephyr lanes; make it the single entry point
  used by humans, by CI, and by upstream bug reports.
- It must distinguish "guest never became ready", "router never listened",
  and "graph empty" — the three states we kept conflating.

### 15.5 — CI wiring

- `just ci` gains `preflight` as its first step.
- A nightly lane runs `probe_mcu_graph.sh` per target and records
  nodes/topics counts, so a silent regression shows up as a diff, not as
  a green build.

## Acceptance

- [ ] Removing `rmw_zenoh_cpp` makes the transport suite **fail**, naming
      the fix — verified by deliberately hiding the overlay.
- [ ] Setting `NROS_EXECUTOR_MAX_CBS=4` makes the sentinel abort at wiring
      with "needs ~50, compiled 4 (NROS_EXECUTOR_MAX_CBS)" instead of
      hanging.
- [ ] `probe_mcu_graph.sh` covers native + freertos + nuttx + zephyr and
      reports the three failure states distinctly.
- [ ] No test in the repo can report success without having exercised its
      subject: every early return is either a hard failure or an explicit,
      counted skip.

## Non-goals

- Rewriting the nano-ros capacity model — that is upstream 0257; this
  phase only makes the consumer side loud.
- Removing skip support entirely; contributors without Autoware installed
  still need a usable subset.

## Notes for whoever picks this up

The cheapest high-value slice is **15.1 + 15.2** (a day at most): they
kill Family A, which cost the most and which recurred three times inside
a single phase. 15.3 needs an entity census in `wire_executor`, which the
Phase 14.3 refactor already makes easy — the counts are all in one
function.
