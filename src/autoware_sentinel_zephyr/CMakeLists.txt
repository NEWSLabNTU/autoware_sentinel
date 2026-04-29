# Autoware Sentinel — Zephyr Safety Island Application

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(autoware_sentinel)

# nros module (CONFIG_NROS=y) provides zenoh shim, platform support,
# and Kconfig→Cargo env var bridging automatically.
rust_cargo_application()
