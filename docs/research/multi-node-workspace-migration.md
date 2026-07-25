# Migration Path: Sentinel → nano-ros Multi-Node Workspace with Launch Files

Research date: 2026-07-24. Sources: nano-ros HEAD `21a3a4248` (book, RFC-0025,
phase-292/296 roadmaps, issues 0254–0258, `examples/workspaces/`), sentinel HEAD
`7befb5a`. Filed before any code changes.

## Where we are vs where nano-ros is

Sentinel pins nano-ros `682f1404` — **4804 commits behind HEAD**. Since then
nano-ros replaced its entire consumption model:

| Aspect | Our pin (682f1404) | nano-ros HEAD |
|---|---|---|
| Node authoring | closure `Executor::<_, 8, 8192>::open` + `add_subscription`/`add_service`/`add_timer` | `Node` + `ExecutableNode` traits + `nros::node!(T)`; entities declared in `register(ctx)`, dispatched in `on_callback` by callback name |
| Entry point | hand-written `main()` per target | one-line `nros::main!(model = "sentinel_bringup:config/system_model.yaml")` per Entry pkg |
| Workspace | flat crates + per-target `.cargo/config.toml` patch blocks | colcon-style `src/<pkg>/` + bringup pkg (`system.toml` + `launch/` + model yaml) + Entry pkgs |
| Msg generation | `cargo nano-ros generate` per pkg | **verb is gone** (lib-only crate). `nros sync` scans all `package.xml`, resolves ament index, writes `generated/<pkg>` at workspace root + `.cargo/config.toml` patches automatically |
| RMW selection | `nros` features (`rmw-zenoh`, `platform-posix`) | **board crate** feature (`nros-board-native { features = ["rmw-zenoh"] }`); `nros` itself is RMW/platform-agnostic (`alloc` + `rmw-cffi` + `ros-humble`) |
| Executor sizing | const generics + `NROS_EXECUTOR_MAX_CBS` env | `Executor<'s>` + runtime `ExecutorSizing` / `open_sized` / `arena_size_for(n)`; env default still 4 (issue 0257: not yet derived from model) |
| Parameters | `register_parameter_services()` + custom params module | launch `<param>` baked at compile time; `[param_services]` block in `system.toml` auto-registers the 6 ROS 2 param services (needs `nros/param-services` feature); volatile store |

Old closure API survives as an escape hatch, but renamed: `add_*` →
`register_subscription/register_service/register_timer`, `client.call().wait()` →
`Promise::try_recv()`/`.await`, `spin_blocking` unchanged,
`register_parameter_services()` unchanged.

## Target layout

```
autoware_sentinel/
├── Cargo.toml                      # workspace: algorithm crates + node pkgs + native entry
├── src/
│   ├── autoware_*/                 # 11 algorithm crates — UNCHANGED (pure no_std, Kani inline)
│   ├── <name>_node_pkg/ …          # NEW node pkgs, one per replaced Autoware node
│   │   └── src/lib.rs              # impl Node + ExecutableNode + nros::node!(T)
│   ├── sentinel_bringup/           # NEW — package.xml + system.toml + launch/*.xml
│   │   ├── system.toml             #   [[component]] catalog, [deploy.*], [param_services], [tiers]?
│   │   ├── launch/system.launch.xml
│   │   └── config/system_model.yaml  # hand-authored or `play_launch resolve` output
│   ├── native_entry/               # nros::main!(model = "sentinel_bringup:…")  deploy = "native"
│   ├── zephyr_entry/               # staticlib, west build -b <board>, RMW via Kconfig
│   ├── freertos_entry/             # deploy = "freertos", nros-board-mps2-an385-freertos
│   ├── nuttx_entry/                # deploy = "nuttx", nros-board-nuttx-qemu-arm
│   └── spe_entry/                  # deploy = "orin-spe", nros-board-orin-spe — OUTSIDE workspace (FSP dep)
├── generated/                      # nros sync output (gitignored)
└── tests/                          # integration tests, fixture switched to entry-pkg build
```

Key contracts (verified in nano-ros examples):

- **Node pkgs are platform/RMW-agnostic libs.** Metadata table is
  `[package.metadata.nros.node]` (class/name/default_namespace). RFC-0025's
  `#[nros::component]` + `[package.metadata.nros.component]` is stale — never
  landed; canonical is `nros::node!`.
- **Service clients call from `tick()`, never `on_callback`** (mid-dispatch
  blocking deadlocks): arm a flag in `on_callback`, issue
  `ctx.call_for_name::<Req,Resp,RN,PN>(…)` in `tick`.
- **`launch =` macro arm is deprecated** (phase-296 R3/R4, warns at build);
  `model =` is canonical. Model yaml is committed; `play_launch resolve` can
  generate it but does not read `[tiers]`/`[lifecycle]` — tiered workspaces
  hand-author (~40 lines, see `ws-realtime-rust`).
- **Per-target node subsets**: `[deploy.<t>] launch = "<file>"` in system.toml,
  model-file override in the entry macro, or automatic board-slicing via
  `execution.deploy` placement in the model.
- **`<remap>` is parsed but NOT routed** (issue 0255, open); no `~/` expansion.
  Nodes hardcode resolved topic names in source — which we already do, so this
  costs us nothing today, but launch remaps stay documentation-only.

## Migration phases

### Phase A — pin bump on the escape hatch (monolith survives)

Bump to current nano-ros; keep `wire_executor`'s single-node shape on the
renamed closure API:

- `Executor::<_, N, M>::open` → `Executor::open_sized(config, ExecutorSizing …)`
  (or env knobs; `arena_size_for(83)`-class sizing for our 83 entities).
- `add_subscription/add_service/add_timer` → `register_*`.
- `add_service_sized` — verify the sized-reply story on the new surface
  (param services path unchanged: `register_parameter_services()` survives).
- Promise-based clients replace `call().wait()`.

Expect consumer walls — ASI's identical bump surfaced 9 (phase-292 intake log);
ours exercises the zenoh-pico path they don't. File each wall upstream
(phase-292 W2.a standing intake).

Also switch msg-gen here: `cargo nano-ros generate` no longer exists. `nros
sync` with sourced ROS env (`AMENT_PREFIX_PATH`). Drop
`tmp/fix_covariance_default.py` — the `message_nros.rs.jinja` template now
emits manual `impl Default` for arrays > 32. Per-pkg `generated/` trees and
hand-maintained `[patch.crates-io]` blocks collapse into workspace-root
`generated/` + auto-written `.cargo/config.toml`.

### Phase B — workspace re-layout

Move to `src/<pkg>` colcon layout so `nros sync` colcon-mode detection and the
pkg-index (backs `nros::main!` + `$(find pkg)`) work. Algorithm crates
untouched. Bringup pkg skeleton (no Cargo.toml, no src/ — anti-pattern list in
RFC-0025 still valid).

### Phase C — split monolith into node pkgs

The real work. `wire_executor` (83 entities, 1436 lines) splits into ~11 node
pkgs mirroring the replaced Autoware nodes (cmd_gate, mrm_handler,
emergency/comfortable stop operators, heartbeat_watchdog, stop_filter,
velocity_converter, shift_decider, twist2accel, control_validator,
operation_mode_manager). Each: `register()` declares pubs/subs/timers/services,
`on_callback` + `tick` delegate to the existing algorithm crate. `SafetyIsland`
shared struct dissolves — cross-node data flows over topics (all in-process,
same executor).

Payoffs:
- **Per-node parameter services appear** — closes the 84-missing-services
  parity gap from Phase 12 (sentinel was one node by design; now it's N nodes
  like baseline Autoware).
- Kani/Verus untouched (they live in algorithm crates).

Watch: per-node topic-name resolution is source-hardcoded (issue 0255);
executor callback count must be sized explicitly (issue 0257 — silent
default-4 dies `create_timer code=-6 Full` at boot; ASI's 4-node island needed
32, we need more).

### Phase D — bringup + native entry + launch

`system.toml` (`[[component]]` catalog, `rmw = "zenoh"`, `[param_services]`,
`[deploy.*]`), `launch/system.launch.xml` with per-node `<param>` rows
replacing most of `params.rs` (62 params → launch-baked initials + volatile
runtime store; persistence out of scope upstream, issue 0080). Hand-author
`config/system_model.yaml`. `native_entry` =
`nros::main!(model = "sentinel_bringup:config/system_model.yaml")` +
`nros-board-native { features = ["rmw-zenoh"] }`. Re-point integration-test
fixture at the entry pkg. `nros check` / `nros plan` for static wiring checks.

### Phase E — embedded entries

Per-target Entry pkgs, exemplars in-tree for all but SPE:

| Target | Exemplar | Notes |
|---|---|---|
| zephyr | `ws-realtime-rust/src/zephyr_entry` | staticlib, one entry per RTOS, board at `west build -b`, RMW via Kconfig (`CONFIG_NROS_RMW_ZENOH`) |
| freertos | `rust/src/qemu_freertos_entry` | `deploy = "freertos"`, `nros-board-mps2-an385-freertos`, locator in `[package.metadata.nros.deploy.freertos]` |
| nuttx | `ws-realtime-rust/src/nuttx_entry` | `deploy = "nuttx"`, `nros-board-nuttx-qemu-arm` |
| orin-spe | **none — we author it** | `nros-board-orin-spe` now lives upstream (README names sentinel as its home); IVC-only zenoh-pico, `locator = "ivc/2"`, builds outside the cargo workspace (FSP dep), board caps `heap=false, threads=false` |

Optional: `[tiers]` realtime tiers (`run_tiers`, one task per tier over one
zenoh session) — but SPE's `threads=false` capability likely excludes the tier
arm there; verify before relying on it.

## Risks

1. **Pin-bump blast radius** (Phase A) — 4804 commits; ASI hit 9 walls, ours
   is the zenoh-pico/embedded lane they didn't exercise. Budget for an
   upstream-issue round-trip loop.
2. **SPE BTCM budget** — current default build fits with 31 KB headroom after
   the 11.3.C campaign. New node!/main! machinery, per-node runtime tables,
   and model-baked entry code are unmeasured on the 256 KB ceiling. Re-run the
   size audit early (a Phase-A monolith build on the new pin gives the first
   data point before the multi-node split adds per-node overhead).
3. **Sizing knobs** — issue 0257: `NROS_EXECUTOR_MAX_CBS` and zpico slot env
   vars are still hidden compile-time knobs, not model-derived; a stale-arena
   rebuild SEGVs. Keep `.env`/fixture sync discipline until upstream derives
   them.
4. **Remap gap** (issue 0255) — cosmetic for us (already hardcoded), but the
   launch XML will over-promise until it lands.
5. **`nros sync` + Autoware ament index** — issue 0258 (srv closure breaks
   cyclone idlc) is cyclone-typesupport-only; our zenoh path should not hit
   it, but the heapless capacity resolver (RFC-0033) defaults need checking
   against big Autoware messages.

## Recommended order

A (pin bump + msg-gen, monolith intact, all 5 targets green again) → B
(layout) → C (node split, Linux-first, planning-sim regression as the gate) →
D (bringup/launch/params) → E (embedded entries, SPE last with size audit).
A is independently valuable even if C stalls: it unblocks tracking upstream
and drops our two local workaround scripts.
