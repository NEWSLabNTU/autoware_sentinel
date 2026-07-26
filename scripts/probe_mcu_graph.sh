#!/usr/bin/env bash
# Probe an MCU lane's ROS 2 graph visibility from the host.
#
# Boots one QEMU lane against a private zenohd, waits for the guest's
# readiness marker, then asks the host for the node list, the topic list, and
# a typed echo of the command topic. Used to chase nano-ros issue 0283's
# residual (tokens declare without error, host graph stays empty).
#
# Usage: scripts/probe_mcu_graph.sh {native|freertos|nuttx|zephyr} [port]
#
# Exit codes distinguish the three states phase 14 kept conflating:
#   2  router never listened      (host-side infrastructure)
#   3  guest never became ready   (firmware / boot / connect)
#   4  guest ready but graph empty (discovery / liveliness)
#   0  graph visible
set -uo pipefail

LANE="${1:-freertos}"
# Default port MUST match the locator baked into each entry
# (.cargo/config.toml NROS_LOCATOR): the guest dials it, it is not
# negotiable at run time.
case "${1:-freertos}" in
  nuttx) DEFAULT_PORT=7452 ;;
  *)     DEFAULT_PORT=7447 ;;
esac
PORT="${2:-$DEFAULT_PORT}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# Router: prefer the in-repo rmw_zenoh overlay's rmw_zenohd (self-contained;
# the sibling nano-ros build tree gets wiped by that repo's own work).
ZENOHD="${ZENOHD:-$ROOT/external/rmw_zenoh_ws/install/rmw_zenoh_cpp/lib/rmw_zenoh_cpp/rmw_zenohd}"
LOG_DIR="$(mktemp -d)"
GUEST_LOG="$LOG_DIR/guest.log"
ROUTER_LOG="$LOG_DIR/router.log"

case "$LANE" in
  freertos)
    BIN="$ROOT/src/freertos_entry/target/thumbv7m-none-eabi/release/freertos_entry"
    QEMU=(qemu-system-arm -cpu cortex-m3 -machine mps2-an385 -nographic
          -semihosting-config enable=on,target=native -kernel "$BIN")
    READY="entering spin loop"
    ;;
  nuttx)
    BIN="$ROOT/src/nuttx_entry/target/armv7a-nuttx-eabihf/release/nuttx_entry"
    QEMU=(qemu-system-arm -M virt -cpu cortex-a7 -nographic -kernel "$BIN"
          -netdev user,id=net0 -device virtio-net-device,netdev=net0)
    READY="nros entry ready"
    ;;
  native)
    # Phase 15.4 — the host lane runs the same declarative entry as the MCUs.
    # `spin = "forever"` is compiled in, so no spin env is needed.
    BIN="$ROOT/target/debug/native_entry"
    QEMU=(env "NROS_LOCATOR=tcp/127.0.0.1:$PORT" "ZENOH_SESSION_CONFIG_URI="
          "ZENOH_ROUTER_CONFIG_URI=" "RUST_LOG=info" "$BIN")
    READY="sentinel graph registered"
    ;;
  zephyr)
    # native_sim executable from `west build`; override with ZEPHYR_EXE.
    BIN="${ZEPHYR_EXE:-$ROOT/build/zephyr_entry/zephyr/zephyr.exe}"
    QEMU=("$BIN")
    READY="entry up"
    ;;
  *) echo "unknown lane: $LANE (native|freertos|nuttx|zephyr)" >&2; exit 2 ;;
esac

if [ ! -f "$BIN" ]; then
  echo "missing binary: $BIN" >&2
  case "$LANE" in
    native)   echo "  build: cargo build -p native_entry" >&2 ;;
    freertos) echo "  build: (cd src/freertos_entry && cargo build --release)" >&2 ;;
    nuttx)    echo "  build: just build-sentinel-nuttx" >&2 ;;
    zephyr)   echo "  build: west build -b native_sim/native/64 src/zephyr_entry \
      -- -DCONF_FILE=\"prj.conf;prj-zenoh.conf\"  (set ZEPHYR_EXE to the result)" >&2 ;;
  esac
  exit 1
fi

cleanup() {
  [ -n "${QEMU_PID:-}" ] && kill "$QEMU_PID" 2>/dev/null
  [ -n "${ZENOHD_PID:-}" ] && kill "$ZENOHD_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

echo "== router on :$PORT"
if [[ "$ZENOHD" == *rmw_zenohd ]]; then
  # rmw_zenohd needs the overlay sourced (rcl/rmw shared libs) and takes its
  # listen endpoint from the zenoh config env.
  bash -c "
    source /opt/autoware/1.5.0/setup.bash >/dev/null 2>&1
    source '$ROOT/external/rmw_zenoh_ws/install/local_setup.bash'
    export ZENOH_CONFIG_OVERRIDE='listen/endpoints=[\"tcp/0.0.0.0:$PORT\"]'
    unset ZENOH_SESSION_CONFIG_URI ZENOH_ROUTER_CONFIG_URI
    exec '$ZENOHD'
  " > "$ROUTER_LOG" 2>&1 &
else
  "$ZENOHD" --listen "tcp/0.0.0.0:$PORT" > "$ROUTER_LOG" 2>&1 &
fi
ZENOHD_PID=$!
for _ in $(seq 1 30); do
  ss -ltn 2>/dev/null | grep -q ":$PORT " && break
  sleep 1
done
ss -ltn 2>/dev/null | grep -q ":$PORT " || {
  echo "STATE: router-down — nothing listened on :$PORT" >&2
  tail -5 "$ROUTER_LOG" >&2
  exit 2
}

echo "== booting $LANE"
"${QEMU[@]}" > "$GUEST_LOG" 2>&1 &
QEMU_PID=$!
for _ in $(seq 1 90); do
  grep -q "$READY" "$GUEST_LOG" 2>/dev/null && break
  sleep 1
done
if ! grep -q "$READY" "$GUEST_LOG"; then
  echo "STATE: guest-not-ready — marker '$READY' never appeared" >&2
  tail -8 "$GUEST_LOG" >&2
  exit 3
fi
echo "   guest ready: $(grep -m1 "$READY" "$GUEST_LOG")"
grep -iE "failed|error" "$GUEST_LOG" | head -5

# Host probes. The harness env leaks RMW/zenoh config; pin everything.
probe() {
  bash -c "
    cd '$ROOT'
    source /opt/autoware/1.5.0/setup.bash >/dev/null 2>&1
    source external/rmw_zenoh_ws/install/local_setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"tcp/127.0.0.1:$PORT\"];scouting/multicast/enabled=false'
    unset ZENOH_SESSION_CONFIG_URI ZENOH_ROUTER_CONFIG_URI ROS_LOCALHOST_ONLY
    $1
  " 2>/dev/null
}

# Discovery lags the readiness marker (liveliness tokens propagate after the
# register pass returns), and a fresh session against a loaded router can miss
# the first sweep entirely — phase 14 mistook both for an empty graph. Settle,
# then retry once.
sleep "${PROBE_SETTLE_SECS:-6}"
NODES="$(probe "timeout 25 ros2 node list --no-daemon | sort")"
if ! printf '%s\n' "$NODES" | grep -q '^/'; then
  sleep 8
  NODES="$(probe "timeout 25 ros2 node list --no-daemon | sort")"
fi
TOPICS="$(probe "timeout 25 ros2 topic list --no-daemon | wc -l")"
echo "== nodes:"; echo "$NODES"
echo "== topics:"; echo "$TOPICS"
echo "== typed echo of /control/command/control_cmd (data path, no graph needed):"
probe "timeout 20 ros2 topic echo /control/command/control_cmd autoware_control_msgs/msg/Control --once" | head -5

echo "== router log (liveliness/session lines):"
grep -icE "liveliness|@ros2_lv" "$ROUTER_LOG"
echo "logs: $LOG_DIR"

# Phase 15.4 — the guest booted and the router is up, but ROS 2 sees nothing:
# a discovery/liveliness failure, distinct from the two states above. This is
# the state that masqueraded as a firmware bug for most of phase 14.5.
NODE_COUNT="$(printf '%s\n' "$NODES" | grep -c '^/' || true)"
if [ "$NODE_COUNT" -eq 0 ]; then
  echo "STATE: graph-empty — guest ready, router up, ros2 sees no nodes" >&2
  exit 4
fi
echo "STATE: ok — $NODE_COUNT nodes visible"
