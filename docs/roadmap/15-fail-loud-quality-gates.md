# Phase 15: Fail-Loud Quality Gates

**Status:** COMPLETE 2026-07-26 (15.1–15.5)
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

### - [x] 15.1 — Preflight: prerequisites hard-fail by default — LANDED

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

### - [x] 15.2 — Environment integrity, not just presence — LANDED

A1–A3 passed a naive "does the path exist" check for months (the symlink
existed; its target did not).

- Preflight resolves each artifact to a *working* one: `librmw_zenoh_cpp.so`
  loads, `rmw_zenohd --help` runs, `zenohd`/`play_launch` answer `--version`.
- No dependency on sibling checkouts' build trees. Phase 14 moved the
  overlay in-repo; codify that as a rule and add a check that fails if any
  `external/*` path resolves outside the repo.

### - [x] 15.3 — Capacity assertions at wiring time — LANDED

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

### - [x] 15.4 — One probe path for every lane — LANDED

`scripts/probe_mcu_graph.sh` (Phase 14.5) already boots a lane, waits for
its readiness marker, and queries nodes/topics/data with the RMW pinned.

- Extend to the native and Zephyr lanes; make it the single entry point
  used by humans, by CI, and by upstream bug reports.
- It must distinguish "guest never became ready", "router never listened",
  and "graph empty" — the three states we kept conflating.

### - [x] 15.5 — CI wiring — LANDED

- `just ci` gains `preflight` as its first step.
- A nightly lane runs `probe_mcu_graph.sh` per target and records
  nodes/topics counts, so a silent regression shows up as a diff, not as
  a green build.

## What landed (15.1 + 15.2)

`tests/src/preflight.rs` — one gate per test class (`transport()`,
`planning_simulator()`), each check carrying a remediation line. The old
`require_*` helpers now delegate to it, so every call site inherited the
behaviour. Fixtures call `transport_strict()`: a fixture cannot skip a
test that is already running, so skipping belongs at the top of a test
body, not here.

Integrity, not presence:
- the overlay check demands `librmw_zenoh_cpp.so` EXISTS and that
  `ros2 pkg list` sees it — the phase-14 failure was a symlink that
  resolved to a wiped tree;
- `external/*` must canonicalize INSIDE the repo (the sibling-checkout
  trap that cost three mis-diagnoses);
- the ROS probes `unset RMW_IMPLEMENTATION` first: with a broken RMW
  selected, every `ros2` call fails, and the gate used to report "ROS 2
  missing" on a machine that has it;
- router liveness is flavour-aware — classic `zenohd` answers
  `--version`, while the overlay's `rmw_zenohd` ignores flags and starts
  routing, so probing it that way hung and then read as "missing".

The harness is now self-contained: `zenohd_binary_path()` prefers the
in-repo overlay's `rmw_zenohd`, and `ZenohRouter` knows how to launch
both flavours (the overlay one needs its env sourced and takes its
endpoint from `ZENOH_CONFIG_OVERRIDE`).

Verified by deliberate breakage — with the overlay hidden, the suite
FAILS naming exactly the two missing artifacts and their fixes; with it
present, 14/14 transport tests pass against the in-repo router.

## What landed (15.3)

`autoware_sentinel_core::capacity` + `wiring_census()`, called at the top
of `wire_executor` before a single entity is created:

- **Enforced** where the binary can read the cap: `MAX_CBS` comes from
  `nros::ExecutorSizing::DEFAULT.cbs`, so an undersized executor is an
  error naming the knob, the requirement, and the compiled value — plus
  the rebuild warning (a resized arena over stale objects SEGVs).
- **Reported** where it cannot: node table, zpico publisher / subscriber /
  queryable / liveliness slots live in C or Kconfig. Those are the ones
  that HANG rather than error, so the census prints the number to set
  (`ZPICO_MAX_LIVELINESS must be >= 101`) at WARN on every boot.
- **Drift-guarded**: `capacity_census_tests` counts the real `create_*`
  and `node_builder(` call sites in the wiring source and fails if the
  census disagrees. It caught two wrong counts in the first run — the
  census is hand-maintained, so this is what keeps it honest.

Today's numbers for the full-feature Linux build: 13 nodes, 38 callbacks,
51 publishers, 101 liveliness tokens.

## What landed (15.4 + 15.5)

`scripts/probe_mcu_graph.sh {native|freertos|nuttx|zephyr}` — one path for
humans, CI and upstream bug reports. It waits for each lane's readiness
marker, pins the RMW env, uses the lane's BAKED locator port, and settles
+ retries once before judging the graph (discovery lags the marker; the
first sweep against a loaded router can miss everything — phase 14 read
both as "empty"). Failure states are now distinct exit codes rather than
one ambiguous "it didn't work": **2** router-down, **3** guest-not-ready,
**4** graph-empty, **0** ok. Missing binaries print the build command for
that lane.

`just preflight` — the gate as a CI step (`tests/tests/preflight_gate.rs`),
now the FIRST thing `just ci` runs: two seconds to learn about a missing
artifact instead of ten minutes of runs that silently skip.

`just probe-lanes` — boots every lane and records its graph size, so a
silent discovery regression appears as a count diff. Today's baseline:

| lane | nodes | topics |
|------|-------|--------|
| native | 13 | 62 |
| freertos | 10 | 44 |
| nuttx | 10 | 44 |

## Acceptance

- [x] Removing `rmw_zenoh_cpp` makes the transport suite **fail**, naming
      the fix — verified by deliberately hiding the overlay.
- [x] Setting `NROS_EXECUTOR_MAX_CBS=4` makes the sentinel abort at wiring
      with the knob and both numbers instead of hanging. Verified:
      `capacity: NROS_EXECUTOR_MAX_CBS needs 38 but this binary compiled 4
      — set NROS_EXECUTOR_MAX_CBS=38 and REBUILD`.
- [x] `probe_mcu_graph.sh` covers native + freertos + nuttx + zephyr and
      reports the three failure states distinctly (exit 2 router-down,
      3 guest-not-ready, 4 graph-empty, 0 ok).
- [x] No test in the repo can report success without having exercised its
      subject: `require_*` delegates to preflight (hard fail unless
      `SENTINEL_ALLOW_SKIP=1`), fixtures use the strict variant, and
      `just ci` runs the gate first.

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
