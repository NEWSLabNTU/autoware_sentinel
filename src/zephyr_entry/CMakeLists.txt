# SPDX-License-Identifier: Apache-2.0
# Sentinel Zephyr Entry (phase 14.5).
#
#   west build -b native_sim/native/64 src/zephyr_entry \
#     -- -DCONF_FILE="prj.conf;prj-zenoh.conf"
#   west build -t run

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(autoware_sentinel_zephyr_entry)

if(CONFIG_NROS_RMW_ZENOH)
    set(EXTRA_CARGO_ARGS --no-default-features --features rmw-zenoh)
else()
    message(FATAL_ERROR
        "The sentinel Zephyr Entry is zenoh-only. Build with: "
        "west build -- -DCONF_FILE=\"prj.conf;prj-zenoh.conf\"")
endif()

rust_cargo_application()
