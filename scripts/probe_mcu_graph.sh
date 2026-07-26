#!/usr/bin/env bash
# Probe an MCU lane's ROS 2 graph visibility from the host.
#
# Boots one QEMU lane against a private zenohd, waits for the guest's
# readiness marker, then asks the host for the node list, the topic list, and
# a typed echo of the command topic. Used to chase nano-ros issue 0283's
# residual (tokens declare without error, host graph stays empty).
#
# Usage: scripts/probe_mcu_graph.sh {freertos|nuttx} [port]
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
  *) echo "unknown lane: $LANE (freertos|nuttx)" >&2; exit 2 ;;
esac

[ -f "$BIN" ] || { echo "missing binary: $BIN" >&2; exit 1; }

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
ss -ltn 2>/dev/null | grep -q ":$PORT " || { echo "router never listened" >&2; exit 1; }

echo "== booting $LANE"
"${QEMU[@]}" > "$GUEST_LOG" 2>&1 &
QEMU_PID=$!
for _ in $(seq 1 90); do
  grep -q "$READY" "$GUEST_LOG" 2>/dev/null && break
  sleep 1
done
if ! grep -q "$READY" "$GUEST_LOG"; then
  echo "guest never reached readiness marker; tail:" >&2
  tail -5 "$GUEST_LOG" >&2
  exit 1
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

echo "== nodes:";  probe "timeout 25 ros2 node list --no-daemon | sort"
echo "== topics:"; probe "timeout 25 ros2 topic list --no-daemon | wc -l"
echo "== typed echo of /control/command/control_cmd (data path, no graph needed):"
probe "timeout 20 ros2 topic echo /control/command/control_cmd autoware_control_msgs/msg/Control --once" | head -5

echo "== router log (liveliness/session lines):"
grep -icE "liveliness|@ros2_lv" "$ROUTER_LOG"
echo "logs: $LOG_DIR"
