#!/bin/bash
# Setup bridge network for Zephyr native_sim sentinel
#
# Creates a Linux bridge with a TAP interface so the Zephyr native_sim
# sentinel can communicate with the host (zenohd, ROS 2) via TCP/IP.
#
# Network topology:
#   Zephyr sentinel (192.0.2.1/zeth0) -- Bridge (zeth-br, 192.0.2.2) -- Host
#
# Usage:
#   sudo ./scripts/zephyr/setup-network.sh [USERNAME]
#
# To tear down:
#   sudo ./scripts/zephyr/setup-network.sh --down

set -e

BRIDGE_NAME="zeth-br"
TAP_NAME="zeth0"
HOST_IP="192.0.2.2"
ZEPHYR_IP="192.0.2.1"
NETMASK="24"

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo $0"
    exit 1
fi

teardown() {
    echo "Tearing down network interfaces..."
    ip link set $TAP_NAME nomaster 2>/dev/null || true
    ip link set $TAP_NAME down 2>/dev/null || true
    ip tuntap del dev $TAP_NAME mode tap 2>/dev/null || true
    ip link set $BRIDGE_NAME down 2>/dev/null || true
    ip link delete $BRIDGE_NAME type bridge 2>/dev/null || true
    echo "Done."
}

if [ "$1" = "--down" ]; then
    teardown
    exit 0
fi

# Determine the user who will run Zephyr
if [ -n "$1" ]; then
    TAP_USER="$1"
elif [ -n "$SUDO_USER" ]; then
    TAP_USER="$SUDO_USER"
else
    TAP_USER=$(logname 2>/dev/null || echo "")
    if [ -z "$TAP_USER" ]; then
        echo "Error: Could not determine user. Please specify: sudo $0 USERNAME"
        exit 1
    fi
fi

echo "Setting up bridge network for Zephyr sentinel..."
echo "  TAP owner: $TAP_USER"

# Clean up any existing setup
teardown 2>/dev/null || true

# Create bridge
echo "  Creating bridge $BRIDGE_NAME ($HOST_IP/$NETMASK)..."
ip link add name $BRIDGE_NAME type bridge
ip addr add $HOST_IP/$NETMASK dev $BRIDGE_NAME
ip link set $BRIDGE_NAME up

# Create TAP interface
echo "  Creating TAP $TAP_NAME for sentinel..."
ip tuntap add dev $TAP_NAME mode tap user $TAP_USER
ip link set $TAP_NAME master $BRIDGE_NAME
ip link set $TAP_NAME up

# Enable IP forwarding
echo 1 > /proc/sys/net/ipv4/ip_forward

echo ""
echo "Bridge network ready:"
echo "  Bridge: $BRIDGE_NAME (Host IP: $HOST_IP)"
echo "  TAP:    $TAP_NAME (Zephyr IP: $ZEPHYR_IP)"
echo "  Owner:  $TAP_USER"
echo ""
echo "Next steps:"
echo "  1. Start zenoh router: zenohd --listen tcp/0.0.0.0:7447"
echo "  2. Build and run Zephyr sentinel:"
echo "     source ../autoware-sentinel-workspace/env.sh"
echo "     cd ../autoware-sentinel-workspace"
echo "     west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel_zephyr"
echo "     ./build/sentinel/zephyr/zephyr.exe"
echo ""
echo "To tear down:"
echo "  sudo $0 --down"
