# Phase 14: Multi-Node Workspace Migration

**Status:** Planned
**Depends on:** Phase 12 (service/topic parity), Phase 13 (multi-platform)
**Research:** `docs/research/multi-node-workspace-migration.md` (2026-07-24)
**Goal:** Migrate the sentinel from a single monolithic node on nano-ros pin
`682f1404` to the current nano-ros multi-node workspace model: per-node
packages authored against the `Node`/`ExecutableNode` traits, a bringup
package carrying `system.toml` + launch XML + system model, and one-line
`nros::main!` Entry packages per deploy target.

## Why

- The pin is 4804 commits behind nano-ros HEAD; the old consumption surface
  (`cargo nano-ros generate`, closure `Executor::add_*`, per-target
  hand-maintained `[patch.crates-io]` blocks) no longer exists upstream.
- Splitting the monolithic `/sentinel` node into per-algorithm nodes closes
  the last parity gap from Phase 12: the 84 missing per-node parameter
  services exist only when the sentinel presents N nodes like baseline
  Autoware.
- Launch-file-driven topology replaces the hand-written `wire_executor`
  (83 entity creations, `src/autoware_sentinel_core/src/lib.rs:504`), making
  per-target node subsets and parameters declarative.
- nano-ros's phase-292 consumer-wall intake is open: walls we surface on the
  zenoh-pico/embedded lane get fixed upstream, as ASI's 9 walls were.

## Non-goals

- Changing any algorithm crate (`src/autoware_*/src/lib.rs`) — pure logic,
  Kani harnesses, and Verus proofs stay as they are.
- Multi-process deployment. Every target keeps all nodes in ONE process /
  one executor; "multi-node" is a ROS-graph-facing and code-organization
  change.
- Launch `<remap>`-driven topic wiring — upstream issue 0255 (remaps parsed
  but not routed) is open; topic names stay resolved in source until it lands.
- Runtime parameter persistence (upstream issue 0080: volatile store only).

## Subphases

### - [ ] 14.1 Pin bump on the compatibility surface (monolith intact)

Bump nano-ros to a current pin and get all five targets green again WITHOUT
restructuring — the renamed closure API is the escape hatch.

Tasks:
- [x] Pick and pin a current nano-ros rev (2026-07-25: local checkout at
      `21a3a4248`, patched by path via `.cargo/config.toml` — stable cargo
      ignores nros-patch.toml's `include =`, so the trio rows are
      hand-maintained). Root + freertos + nuttx + spe-firmware done;
      zephyr deferred (below).
- [x] Mechanical API migration in `autoware_sentinel_core` and the target
      binaries (entities attach via `node_builder("sentinel")` + `node_mut`;
      `create_subscription`/`create_service`/`register_timer`;
      `open_sized(cbs=64, arena_size_for(64))` — the new executor hard-caps
      callback slots at 64 and publishers no longer consume entry slots):
  - `Executor::<_, N, M>::open(cfg)` → `Executor::open_sized(cfg,
    ExecutorSizing { .. })` sized for our 83 callbacks
    (`nros::arena_size_for`).
  - `add_subscription` / `add_service` / `add_service_sized` / `add_timer`
    → `register_subscription` / `register_service` / `register_timer`;
    verify the sized-reply path for the 8 KiB parameter-service replies.
  - `client.call(&req).wait(&mut executor, ms)` → `Promise` polling.
  - `register_parameter_services()` and `spin_blocking(SpinOptions)` are
    unchanged — keep.
- [x] Switch message generation: delete the per-package
      `cargo nano-ros generate` flow, per-package `generated/` trees, and
      `tmp/fix_covariance_default.py`; adopt `nros sync` (colcon-mode needs
      14.2's layout — until then use `nros generate-rust` per package or a
      temporary flat-mode invocation) with the Autoware ament index sourced.
      Confirm the template-emitted manual `Default` for `[f64; 36]` compiles.
- [x] Re-derive capacity env knobs (`.env` + fixture MAX_CBS 96→64; zpico
      knobs baked per-target in `.cargo/config.toml [env]`; new knobs found:
      `ZPICO_MAX_LARGE_SUBSCRIBERS`, `ZPICO_SUBSCRIBER_SIZE_THRESHOLD`,
      `NROS_FREERTOS_HEAP_KB`): `NROS_EXECUTOR_MAX_CBS`,
      `NROS_PARAM_SERVICE_BUFFER_SIZE`, `ZPICO_*` — names/defaults may have
      moved in 4804 commits; re-sync `.env`, the test fixture, and
      `build-spe-firmware`.
- [ ] Wall intake: file every consumer wall as a nano-ros issue against the
      phase-292 W2.a standing intake, with repro. Track the list here (ASI's
      identical bump surfaced 9; ours exercises zenoh-pico paths ASI's
      cyclone lane did not).

  **Wall log:** (append as found)
  - Wall #1 (2026-07-25, freertos/MPS2, OPEN): comp-all wiring (37 pubs /
    21 services / 5 subs on one zenoh-pico session) fails `Executor` setup
    with `Transport(SubscriberCreationFailed)` once the entity total
    crosses a threshold — the 4-combo (mrm + cmd-gate-extra + validator +
    op-mode-mgr) boots, adding comp-engagement (4 pubs / 2 subs / 2 svcs)
    tips it over. Not slot caps: reproduced with ZPICO_MAX_SUBSCRIBERS=64,
    ZPICO_MAX_LIVELINESS=160, 8 large-class blocks, 2.5 MiB heap. Every
    comp-* feature passes alone. Same class as the Phase-13.K1
    "declare-storm" the bisection gates were built for — now a hard error
    instead of a hang. Freertos target ships the core baseline (boots +
    spins, proven under QEMU); comp-all blocked on upstream diagnosis
    (needs zenoh-pico debug logging, hardcoded off in nros-zpico-build).
  - Wall #2 (2026-07-25, orin-spe, worked around): `nros-rmw-zenoh` deps
    `zpico-sys` with DEFAULT features, so cargo feature-unification drags
    the `platform-aliases` TU into the SPE staticlib even though
    zpico-sys's `orin-spe` feature documents it OFF (the SPE system.c
    implements the `_z_*` surface natively) — double-defines
    `z_time_elapsed_ms/_s` + `_z_get_time_since_epoch` at the spe.elf
    link. Worked around in `build-spe-firmware` (ar-strip the alias TU);
    upstream fix: `zpico-sys = { default-features = false, ... }` per
    platform feature in nros-rmw-zenoh.
  - Wall #3 (2026-07-25, orin-spe, OPEN — SIZE): the default SPE build
    (Executor::open + spin, no SafetyIsland) now overflows the 256 KB
    BTCM by 164 KB (.data section named), where the d9af52be pin fit
    with 31 KB headroom — the new pin costs ~+195 KB. Staticlib pre-gc
    totals: text 464 KB / bss 158 KB (compiler_builtins alone 118 KB
    text). Same knobs applied as the 11.3.C campaign (slot rightsizing,
    NROS_EXECUTOR_MAX_CBS=8). Needs an 11.3.C-style size audit round on
    the new pin, or the 11.3.E DRAM/AST mapping. Consumer-side fixes
    that got the link this far: build.rs compiling the
    nros-platform-freertos C port against FSP headers (the old
    nros-platform-orin-spe crate that did this was retired upstream),
    `nros_rmw_zenoh::register()` in the firmware entry, and a newlib
    `clock_gettime` shim in nros-app.c (zpico.c's session-seed path
    calls it; newlib has no syscall backend).
  - Fixed en route (freertos, consumer-side): board C `Reset_Handler` now
    calls `main` not `_start`; config schema flipped to direct-mode
    `[[transport]]`/`[node]` (legacy `[network]`/`[zenoh]`/`[scheduling]`
    dropped, 172.K.6); `[scheduling] app_stack_bytes` replaced by
    `Config::with_app_stack_bytes` (384 KiB default overflows at ~20 pubs;
    set 768 KiB); build now needs `FREERTOS_DIR/FREERTOS_PORT/LWIP_DIR/
    FREERTOS_CONFIG_DIR/NROS_PLATFORM_FREERTOS_SRC/NROS_PLATFORM_CFFI_
    INCLUDE/NROS_LAN9118_LWIP_DIR` baked in `.cargo/config.toml [env]`.

- [x] SPE size checkpoint: **default build no longer fits BTCM** — see
      wall #3 (overflow 164 KB vs 31 KB headroom on the old pin). spe.bin
      link blocked until a size round or 11.3.E lands; staticlib +
      firmware sources fully build on the new pin.: rebuild `spe.bin` on the new pin, record
      text+data+bss against the 224 KB / 31 KB-headroom baseline in the
      11.3.C ledger before any multi-node machinery lands.

Acceptance:
- [x] `just ci` green (format, cross-check `thumbv7em-none-eabihf`, tests).
- [x] All 20 integration tests pass (14 transport smoke + 6 planning
      simulator; `play_launch` 0.8.2 reinstalled, zenohd fallback to the
      nano-ros build).
- [ ] Zephyr native_sim, FreeRTOS MPS2, NuttX QEMU targets build and boot.
      FreeRTOS: boots + spins (core baseline; comp-all = wall #1).
      NuttX: boots + spins with full comp-all.
      Zephyr: DEFERRED to 14.5 — the west workspace was never provisioned
      on this machine and the old zephyr-lang-rust `rustapp` shape is
      replaced by the workspace `zephyr_entry` (west module + Kconfig RMW)
      anyway. The SPE POSIX-sim lane (`autoware_sentinel_spe`) is likewise
      deferred: it rides the old `nros/link-ivc` feature plumbing.
- [ ] `just build-spe-image` fits BTCM; size delta recorded.
- [ ] Zero hand-written message code; `generated/` reproducible from
      `nros sync`/`nros generate-rust` alone.

### - [ ] 14.2 Workspace re-layout (colcon shape)

Move to the `src/<pkg>` colcon layout that `nros sync` colcon-mode detection
and the pkg-index (which backs `nros::main!` and `$(find pkg)`) require.

Tasks:
- [ ] Algorithm crates stay at `src/autoware_*/` — verify each has a valid
      `package.xml` (pkg-index collects every `package.xml` under the root).
- [ ] Root `Cargo.toml` workspace members updated; `generated/` moves to
      workspace root (gitignored), patches auto-written to `.cargo/config.toml`
      by `nros sync` — delete the hand-maintained per-target patch blocks
      where superseded.
- [ ] Scaffold `src/sentinel_bringup/` (RFC-0025 Path A: `package.xml` +
      `system.toml` + `launch/` ONLY — no `Cargo.toml`, no `src/`).
- [ ] Keep `just` recipes working (`generate-bindings` → `nros sync`).

Acceptance:
- [ ] `nros sync` in colcon mode resolves every Autoware message dependency
      from the ament index into root `generated/`.
- [ ] 14.1 acceptance still holds after the move.

### - [ ] 14.3 Node split (Linux-first)

Dissolve `wire_executor` + `SafetyIsland` into per-node packages mirroring
the replaced Autoware nodes.

Node packages (one per filtered Autoware node the sentinel replaces):
`stop_filter_node`, `velocity_converter_node`, `shift_decider_node`,
`emergency_stop_operator_node`, `comfortable_stop_operator_node`,
`heartbeat_watchdog_node`, `mrm_handler_node`, `vehicle_cmd_gate_node`,
`twist2accel_node`, `control_validator_node`, `operation_mode_manager_node`.

Tasks:
- [ ] Each node pkg: `[lib]` crate, `[package.metadata.nros.node]`
      (class/name/default_namespace), `impl Node` (`register(ctx)` declares
      pubs/subs/timers/services), `impl ExecutableNode` (`State` wraps the
      existing algorithm struct; `on_callback` dispatches by callback name;
      service-client calls issued from `tick`, never `on_callback`),
      `nros::node!(T)` last line. Platform/RMW-agnostic deps only
      (`nros` with `alloc` + `rmw-cffi` + `ros-humble`; msg crates
      `version = "*"`).
- [ ] Cross-node state previously shared through `SafetyIsland` becomes
      topics (all in-process on one executor — no transport cost) or stays
      inside the owning node. Document each converted edge.
- [ ] Topic names: keep resolved names in source (issue 0255); mirror them
      as `<remap>`-style comments in the launch file for documentation.
- [ ] Callback-count sizing: count total entities across all nodes, set
      executor sizing explicitly (issue 0257: silent default-4 dies at boot
      with `create_timer code=-6 Full`; clean rebuild after any resize).
- [ ] Sequencing/determinism: the 30 Hz control chain ordering previously
      enforced by one timer callback (watchdog → MRM → gate → validator →
      publish) must be preserved — either one orchestrating timer per chain
      link with topic edges, or callback-effect declarations
      (`callback_for_name(..).reads_entity/publishes_entity`) if the
      executor's ordering honors them. Verify on the planning simulator; no
      regression vs the 46 s drive baseline.
- [ ] Params: split `params.rs` (62 params) per node; each node declares its
      own set.

Acceptance:
- [ ] Old `autoware_sentinel_core` wiring deleted.
- [ ] Unit tests still pass (algorithm crates untouched).
- [ ] Planning-simulator drive completes with behavior parity (MRM
      escalation, gate arbitration, validator flags) vs the monolith.
- [ ] `ros2 node list` shows the N per-algorithm nodes.

### - [ ] 14.4 Bringup, launch, params, native entry

Tasks:
- [ ] `sentinel_bringup/system.toml`: `[system] rmw = "zenoh"`,
      `[[component]]` catalog for all node pkgs, `[param_services]` block
      (auto-registers the 6 ROS 2 parameter services per node — entry must
      enable the `nros/param-services` feature or the block is silently
      inert), `[deploy.*]` per target.
- [ ] `launch/system.launch.xml`: one `<node>` per component with `<param>`
      rows carrying the per-node defaults (replaces most of the old
      `params.rs` values; runtime `ros2 param set` reconfigure is volatile).
- [ ] `config/system_model.yaml`: hand-authored (canonical `model =` macro
      arm; the `launch =` arm is deprecated upstream and warns at build).
- [ ] `src/native_entry/`:
      `nros::main!(model = "sentinel_bringup:config/system_model.yaml")`,
      `[package.metadata.nros.entry] deploy = "native"`, deps on
      `nros-board-native { features = ["rmw-zenoh"] }` + all node pkg rlibs.
- [ ] Integration tests: `tests/src/fixtures/sentinel.rs` builds
      `native_entry` instead of `autoware_sentinel_linux`; capacity env vars
      re-synced.
- [ ] `nros check` / `nros plan` wired into `just ci` as static wiring gates.
- [ ] Retire `src/autoware_sentinel_linux` binary in favor of `native_entry`
      (its `generated/` superset role already moved to root in 14.2).

Acceptance:
- [ ] `just launch-autoware-sentinel --drive` completes a drive with the
      entry binary.
- [ ] `ros2 service list` shows per-node parameter services — Phase 12's
      84-service gap measured again; target: 0 missing services vs baseline.
- [ ] All integration tests pass via the new fixture.
- [ ] `nros check` reports 0 unresolved components.

### - [ ] 14.5 Embedded entries (zephyr, freertos, nuttx, orin-spe)

One Entry pkg per target, all hosting the full launch graph in one process.
Exemplars: `examples/workspaces/ws-realtime-rust/src/{zephyr,nuttx}_entry`,
`examples/workspaces/rust/src/qemu_freertos_entry`. Orin-SPE has NO in-tree
exemplar — we author the first (the `nros-board-orin-spe` crate now lives
upstream and names this repo as its home).

Tasks:
- [ ] `zephyr_entry`: staticlib, board chosen at `west build -b`, RMW via
      Kconfig (`CONFIG_NROS_RMW_ZENOH`); replaces
      `src/autoware_sentinel_zephyr`.
- [ ] `freertos_entry`: `deploy = "freertos"`,
      `nros-board-mps2-an385-freertos { features = ["rmw-zenoh"] }`,
      locator in `[package.metadata.nros.deploy.freertos]`; replaces
      `src/autoware_sentinel_freertos`.
- [ ] `nuttx_entry`: `deploy = "nuttx"`, `nros-board-nuttx-qemu-arm`;
      replaces `src/autoware_sentinel_nuttx`.
- [ ] `spe_entry`: `deploy = "orin-spe"`, `nros-board-orin-spe` (IVC-only
      zenoh-pico, `locator = "ivc/2"`), built OUTSIDE the cargo workspace
      (FSP static-lib dependency), wrapped into the firmware staticlib as
      today. Board capabilities are `heap = false, threads = false`: verify
      the macro's entry arm works without threads, and that the `[tiers]`
      `run_tiers` arm is NOT required (single-tier `run`).
- [ ] Per-target node subsets where BTCM demands it: `[deploy.<t>]
      launch = "<file>"` or a model-file override in the entry macro —
      e.g. a reduced SPE launch matching today's default-feature build,
      full graph behind the `safety-island`-equivalent model.
- [ ] SPE size audit: per-node runtime tables + macro-emitted registration
      code vs the 256 KB ceiling; extend the 11.3.C ledger. If overflow, the
      reduced-launch model is the mitigation (and 11.3.E DRAM mapping stays
      the long-term fix).
- [ ] Optional: `[tiers]` realtime tiers for zephyr/freertos/nuttx entries
      (high tier: control chain timer; low tier: status publishers).

Acceptance:
- [ ] All four embedded entries build; zephyr native_sim + FreeRTOS QEMU +
      NuttX QEMU boot and exchange traffic with a host zenohd.
- [ ] `spe.bin` links within BTCM with the chosen launch subset; size delta
      recorded.
- [ ] Old per-target binary crates (`autoware_sentinel_zephyr`,
      `_freertos`, `_nuttx`) and `sentinel_spe_firmware`'s hand wiring
      retired or reduced to thin shims over the entries.

## Risks

| Risk | Mitigation |
|---|---|
| Pin-bump walls (14.1) on the zenoh-pico lane | File upstream per phase-292 intake; keep the monolith shape until walls close so bisection stays cheap |
| SPE BTCM overflow from node!/main! machinery | Size checkpoint at 14.1 (before split) and 14.5; reduced-launch subset as fallback |
| Control-chain ordering lost in the split (14.3) | Explicit orchestration timer + planning-sim behavior gate before merging |
| Hidden sizing knobs (issue 0257) | Entity census in 14.3; document knob values in `.env` + fixture as today |
| `nros sync` capacity defaults vs large Autoware msgs (RFC-0033) | Audit heapless bounds for our message set at 14.1 |
| Upstream `launch =`→`model =` churn (phase-296 R4) | Author `model =` from the start; never ship the deprecated arm |

## Deliverables

- Working tree: node pkgs + bringup + 5 entries replacing the monolith.
- Updated `CLAUDE.md` (structure, build commands, phase table).
- Wall log upstream-filed; `docs/research/multi-node-workspace-migration.md`
  kept as the rationale record.
