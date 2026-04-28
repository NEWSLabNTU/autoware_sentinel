# nano-ros on Orin SPE: Concrete Implementation

This document shows exactly what the nano-ros app code looks like when integrated
into the NVIDIA SPE FreeRTOS build system, grounded in the real nano-ros API
as seen in `examples/qemu-arm-freertos/`.

---

## Platform: Jetson AGX Orin 64GB + JetPack 6.2.1

### Hardware

The Jetson AGX Orin SoC contains a **Cortex-R5F** core dedicated to the
**SPE (Sensor Processing Engine)**, also called the AON (Always-On) subsystem.
This core runs independently of the main Cortex-A78AE complex (CCPLEX) and has
access to AON-domain peripherals (GPIO, I2C, SPI, GTE hardware timestamping).
It runs FreeRTOS V10.4.3 via NVIDIA's FSP (Firmware Support Package).

**Target triple:** `armv7r-none-eabihf` (Cortex-R5F with hardware float)

> **Note:** The Rust target is `armv7r-none-eabihf` (ARM mode), not `thumbv7r-none-eabihf`
> (Thumb mode). The NVIDIA BSP uses `-mthumb-interwork` but defaults to ARM instruction
> set. Verify float ABI compatibility: BSP uses `-mfloat-abi=softfp` while `eabihf`
> implies hard float — see Phase 11.4 for resolution.

### SPE FreeRTOS BSP (L4T 36.4)

JetPack 6.2.1 ships L4T 36.4.4. The SPE source is distributed separately:

1. Download SPE FreeRTOS BSP from the [Jetson Linux release page](https://developer.nvidia.com/embedded/jetson-linux-r3644)
   (look for `spe_freertos_bsp` in the BSP sources tarball, or check the
   [SPE Developer Guide](https://docs.nvidia.com/jetson/archives/r36.4/spe/rt-compiling.html))
2. Download **ARM GNU Toolchain 13.2.rel1** (`arm-none-eabi-`) from ARM's website
3. Set environment:
   ```bash
   export SPE_FREERTOS_BSP=/path/to/extracted/spe-freertos-bsp
   export CROSS_COMPILE=/path/to/arm-gnu-toolchain-13.2.rel1/bin/arm-none-eabi-
   ```
4. Build:
   ```bash
   cd ${SPE_FREERTOS_BSP}/rt-aux-cpu-demo-fsp
   make -j$(nproc) bin_t23x
   # Output: out/t23x/spe.bin
   ```
5. Flash SPE partition only:
   ```bash
   cp out/t23x/spe.bin ~/Linux_for_Tegra/bootloader/spe_t234.bin
   cd ~/Linux_for_Tegra
   sudo ./flash.sh -k A_spe-fw jetson-agx-orin-devkit internal
   ```

### Transport: IVC Is the Only Viable Option

**IVC (Inter-VM Communication)** is the only viable bidirectional data transport between
the SPE and Linux. It uses shared-memory ring buffers in a DRAM carveout (mapped into
SPE address space via AST) with HSP doorbell signaling for notification.

The L4T 36.4 BSP includes IVC infrastructure for AGX Orin:
- `ivc-config.h`: echo channel with 16 frames × 64 bytes, `CCPLEX_CARVEOUT_BASE 0x80000000`
- `ivc-channels.c`: AST region setup, HSP mailbox IRQ handler (`ccplex_ipc_irq`)
- `ivc-echo-task.c`: working echo demo (read frame, echo back)
- Linux access: `/sys/devices/platform/bus@0/bus@0:aon_echo/data_channel` (needs device
  tree `aon_echo { status = "okay"; }`)

**Important caveat:** NVIDIA forum posts from 2022 (L4T 35.x era) stated IVC was "verified
only on AGX Xavier" and "not yet available for AGX Orin." The L4T 36.4 BSP adds Orin IVC
code, but on-hardware verification is required before relying on it (see Phase 11.1).

#### Why not UART?

The SPE's only UART is the **TCU (Tegra Combined UART)** — a debug multiplexer that
serves all 8 SoC processors through a single physical UART. Analysis of the BSP code:

- `tcu.c`: SPE runs the TCU mux server. Each processor gets a tag (SPE = `0xe0`,
  CCPLEX = `0xe1`). The `spe_rx_task` demuxes incoming bytes by tag prefix.
- `tcu-ids.c` / `tcu-priv.h`: CCPLEX has TX and RX tasks via HSP mailboxes
  (`tegra_hsp_id_top0`), message size 128–256 bytes.
- The TCU output appears on Linux as `/dev/ttyTCU0` — this is the combined debug
  console for all processors, not a dedicated SPE↔Linux serial channel.
- Even if bidirectional TCU messaging were used (via HSP mailboxes), bandwidth is
  limited to ~11 KB/s at 115200 baud, shared with all processor debug output.

**TCU is unsuitable for ROS message transport.** It is useful only for `printf` debug
output from the SPE (which appears on the host's serial console).

#### Fallback: GPIO-only watchdog

If IVC cannot be made to work on Orin, the simplest deployment requires no ROS transport:
SPE monitors a heartbeat GPIO from CCPLEX and asserts an emergency-stop GPIO when it
goes silent. This provides hardware-level safety without any software protocol.

### References

- [SPE Developer Guide (L4T 36.4)](https://docs.nvidia.com/jetson/archives/r36.4/spe/rt-compiling.html) — compile and flash instructions
- [IVC Echo Channel docs (L4T 36.3)](https://docs.nvidia.com/jetson/archives/r36.3/spe/md__home_jenkins_agent_workspace_Utilities_rt_aux_cpu_demo_fsp_docs_work_rt_aux_cpu_demo_fsp_doc_ivc.html) — IVC channel configuration and device tree
- [Forum: Using FreeRTOS on AGX Orin](https://forums.developer.nvidia.com/t/using-free-rtos-in-agx-orin/267845) — source download links, limitations
- [Forum: IVC on AGX Orin SPE](https://forums.developer.nvidia.com/t/inter-vm-communication-of-spe-on-agx-orin/215982) — IVC support status discussion
- [Forum: SPE/FreeRTOS sources for L4T 34.1.1](https://forums.developer.nvidia.com/t/spe-and-freertos-sources-available-for-l4t-34-1-1/215498)

---

## How the Real API Looks (from actual source)

The FreeRTOS talker example is the direct model:

```rust
// examples/qemu-arm-freertos/rust/zenoh/talker/src/main.rs (real code)
#![no_std]
#![no_main]

use nros::prelude::*;
use nros_mps2_an385_freertos::{Config, println, run};

#[unsafe(no_mangle)]
extern "C" fn _start() -> ! {
    run(Config::default(), |config| {
        let exec_config = ExecutorConfig::new(config.zenoh_locator)
            .domain_id(config.domain_id)
            .node_name("talker");
        let mut executor = Executor::<_, 0, 0>::open(&exec_config)?;
        let mut node = executor.create_node("talker")?;
        let publisher = node.create_publisher::<Int32>("/chatter")?;

        for i in 0..10i32 {
            for _ in 0..100 { executor.spin_once(10); }
            publisher.publish(&Int32 { data: i })?;
        }
        Ok::<(), NodeError>(())
    })
}
```

The board crate's `run()` handles everything: it creates a FreeRTOS task, starts
the scheduler, brings up the network stack inside the task, and calls your closure.
Your closure runs inside a FreeRTOS task — you don't manage tasks or the scheduler
at all. This is exactly the integration pattern we replicate for the SPE.

---

## The Adapter: `nros-orin-spe` Board Crate

The gap between the existing FreeRTOS board crate (`nros-mps2-an385-freertos`) and
the Orin SPE is exactly three things:

1. **Network**: replace LAN9118 Ethernet + lwIP with IVC shared memory (DRAM carveout + HSP doorbells)
2. **Output**: replace semihosting (`hprintln!`) with SPE TCU debug output (`printf` via FSP)
3. **Entry point**: replace `_start` (Cortex-M bare-metal reset) with
   `my_app_init()` called from NVIDIA's `main.c`

Everything else — the `run()` pattern, `Executor`, publishers, subscriptions,
services — is identical.

---

## File Layout

```
rt-aux-cpu-demo-fsp/
├── app/
│   └── nros-app.c          ← C shim: my_app_init(), FreeRTOS task, IVC transport
├── soc/t23x/
│   └── target_specific.mk  ← add ENABLE_NROS_APP := 1
├── Makefile                 ← add nros-app.c conditionally
└── nros-orin-spe/          ← Rust board crate (lives next to rt-aux-cpu-demo-fsp)
    ├── Cargo.toml
    ├── build.rs             ← links against FSP static lib
    └── src/
        ├── lib.rs           ← run(), Config, println! macro
        └── ivc.rs           ← IVC transport backend for zenoh-pico
```

---

## 1. The Rust Board Crate

### `nros-orin-spe/src/lib.rs`

```rust
//! Board crate for running nano-ros on the Jetson Orin SPE (Cortex-R5F,
//! FreeRTOS V10.4.3). Provides run(), Config, and println! analogous to
//! nros-mps2-an385-freertos, but backed by IVC transport instead of Ethernet.
//!
//! Entry: my_app_init() is called from NVIDIA's platform main.c.
//! It creates a FreeRTOS task and returns — the scheduler is already running.

#![no_std]

mod config;
mod ivc;

pub use config::Config;

// Re-export the FSP UART printf for println!
extern "C" {
    // NVIDIA FSP UART output — always available on SPE debug UART
    fn printf(fmt: *const u8, ...) -> i32;
}

#[macro_export]
macro_rules! println {
    () => { unsafe { $crate::printf(b"\r\n\0".as_ptr()); } };
    ($fmt:literal) => {
        unsafe { $crate::printf(concat!($fmt, "\r\n\0").as_ptr()); }
    };
    ($fmt:literal, $($arg:expr),*) => {
        unsafe { $crate::printf(concat!($fmt, "\r\n\0").as_ptr(), $($arg),*); }
    };
}

// ── FreeRTOS FFI (NVIDIA BSP provides these) ──────────────────────────────────

extern "C" {
    fn xTaskCreate(
        task_fn:     unsafe extern "C" fn(*mut core::ffi::c_void),
        name:        *const u8,
        stack_depth: u16,
        param:       *mut core::ffi::c_void,
        priority:    u32,
        handle:      *mut *mut core::ffi::c_void,
    ) -> i32;

    fn pvPortMalloc(size: usize) -> *mut core::ffi::c_void;
    fn vTaskDelay(ticks: u32);
}

// Stack size for the nano-ros task (words). nano-ros + zenoh-pico needs
// roughly 8–12 KB. 4096 words = 16 KB provides comfortable headroom.
const NROS_TASK_STACK: u16 = 4096;
const NROS_TASK_PRIORITY: u32 = 2;

struct AppContext<F> {
    config: Config,
    closure: F,
}

unsafe extern "C" fn nros_task_entry<F, E>(arg: *mut core::ffi::c_void)
where
    F: FnOnce(&Config) -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    let ctx = unsafe { &mut *(arg as *mut AppContext<F>) };

    // IVC transport init — establishes the shared-memory channel to Linux
    if ivc::init().is_err() {
        println!("nros_task: IVC init failed");
        return;
    }

    // Brief delay: let Linux-side IVC bridge daemon start up
    unsafe { vTaskDelay(2000) }; // 2 s at 1 kHz tick

    let closure = unsafe { core::ptr::read(&ctx.closure) };
    if let Err(e) = closure(&ctx.config) {
        println!("nros_task: error: {:?}", e);
    }
}

/// Run a nano-ros application on the Orin SPE.
///
/// Call this from `my_app_init()` (the C entry point registered in NVIDIA's
/// `main.c`). It allocates a FreeRTOS task and returns immediately — the
/// scheduler is already running when NVIDIA's main calls the init functions.
///
/// Inside the closure, use `Executor::open()` for full API access.
///
/// # Example
///
/// ```rust,ignore
/// use nros_orin_spe::{Config, run};
/// use nros::prelude::*;
/// use sensor_msgs::msg::Imu;
///
/// pub extern "C" fn nros_app_rust_entry() {
///     run(Config::default(), |config| {
///         let exec_config = ExecutorConfig::new(config.zenoh_locator)
///             .node_name("spe_imu_node");
///         let mut executor = Executor::<_, 0, 0>::open(&exec_config)?;
///         let mut node = executor.create_node("spe_imu_node")?;
///         let pub_ = node.create_publisher::<Imu>("/imu/data_raw")?;
///         loop {
///             let msg = read_imu(); // your I2C HAL call
///             pub_.publish(&msg)?;
///             executor.spin_once(10);
///         }
///     });
/// }
/// ```
pub fn run<F, E>(config: Config, f: F)
where
    F: FnOnce(&Config) -> core::result::Result<(), E> + Send + 'static,
    E: core::fmt::Debug,
{
    let ctx_ptr = unsafe {
        let size = core::mem::size_of::<AppContext<F>>();
        let ptr = pvPortMalloc(size) as *mut AppContext<F>;
        assert!(!ptr.is_null(), "pvPortMalloc failed for AppContext");
        core::ptr::write(ptr, AppContext { config, closure: f });
        ptr
    };

    let ret = unsafe {
        xTaskCreate(
            nros_task_entry::<F, E>,
            b"nros_app\0".as_ptr(),
            NROS_TASK_STACK,
            ctx_ptr as *mut core::ffi::c_void,
            NROS_TASK_PRIORITY,
            core::ptr::null_mut(),
        )
    };

    if ret != 1 /* pdPASS */ {
        println!("nros run(): xTaskCreate failed");
    }
    // Return to NVIDIA's main — scheduler is already running
}
```

### `nros-orin-spe/src/ivc.rs`

```rust
//! IVC transport backend for zenoh-pico on the Orin SPE.
//!
//! zenoh-pico's transport abstraction calls z_link_custom_open(),
//! z_link_custom_send(), z_link_custom_recv(). We implement these
//! by calling the NVIDIA IVC driver (tegra_ivc_channel_*).
//!
//! On the Linux side, a small bridge daemon reads from /dev/tegra-ivc0
//! and forwards frames to zenohd on localhost:7447 via TCP.

extern "C" {
    // NVIDIA FSP IVC driver (fsp/source/include/ivc/tegra-ivc.h)
    fn tegra_ivc_channel_write(ch: *mut IvcChannel, buf: *const u8, len: usize) -> i32;
    fn tegra_ivc_channel_read(ch: *mut IvcChannel, buf: *mut u8, max_len: usize) -> i32;
    fn tegra_ivc_channel_get(id: u32) -> *mut IvcChannel;
    fn tegra_ivc_channel_notified(ch: *mut IvcChannel) -> bool;
}

// Opaque type — layout handled by C
#[repr(C)]
pub struct IvcChannel {
    _opaque: [u8; 0],
}

// IVC channel ID for nano-ros — must match ivc-config.h and Linux DTS
const NROS_IVC_CHAN_ID: u32 = 2; // use a free channel; 0=echo, 1=reserved

static mut IVC_CHAN: *mut IvcChannel = core::ptr::null_mut();

pub fn init() -> Result<(), ()> {
    let ch = unsafe { tegra_ivc_channel_get(NROS_IVC_CHAN_ID) };
    if ch.is_null() {
        return Err(());
    }
    unsafe { IVC_CHAN = ch };
    Ok(())
}

/// Called by zenoh-pico C shim to send a frame to Linux.
#[no_mangle]
pub unsafe extern "C" fn nros_ivc_send(buf: *const u8, len: usize) -> i32 {
    let ch = unsafe { IVC_CHAN };
    if ch.is_null() { return -1; }
    unsafe { tegra_ivc_channel_write(ch, buf, len) }
}

/// Called by zenoh-pico C shim to receive a frame from Linux.
#[no_mangle]
pub unsafe extern "C" fn nros_ivc_recv(buf: *mut u8, max_len: usize) -> i32 {
    let ch = unsafe { IVC_CHAN };
    if ch.is_null() { return -1; }
    // Non-blocking poll
    if unsafe { !tegra_ivc_channel_notified(ch) } { return 0; }
    unsafe { tegra_ivc_channel_read(ch, buf, max_len) }
}
```

### `nros-orin-spe/src/config.rs`

```rust
//! SPE-specific transport configuration.
//! The "zenoh_locator" here is a custom IVC URI that zenoh-pico
//! maps to our nros_ivc_send/recv functions via a custom transport.

#[derive(Clone)]
pub struct Config {
    /// Zenoh locator — for IVC transport, use the custom URI scheme
    pub zenoh_locator: &'static str,
    pub domain_id: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // "ivc/2" → resolved by zpico-sys custom IVC link to nros_ivc_send/recv
            // Channel 0 = echo, channel 2 = nano-ros (must match ivc-config.h)
            zenoh_locator: "ivc/2",
            domain_id: 0,
        }
    }
}
```

---

## 2. The C Shim: `app/nros-app.c`

This is what plugs into NVIDIA's existing build system. It's a thin wrapper
that calls into the compiled Rust static library.

```c
// rt-aux-cpu-demo-fsp/app/nros-app.c
//
// nano-ros application task for Jetson Orin SPE.
// Built when ENABLE_NROS_APP := 1 in target_specific.mk.
//
// Responsibilities:
//   1. Expose my_app_init() — the standard NVIDIA init hook
//   2. Call nros_app_rust_entry() — the Rust run() wrapper
//      (compiled into libnros_orin_spe.a by Cargo)

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

// Defined in nros-orin-spe/src/lib.rs via #[no_mangle]
extern void nros_app_rust_entry(void);

// Called from platform main.c after all NVIDIA inits, before vTaskStartScheduler.
// This matches the ENABLE_* pattern of all other demo apps.
void nros_app_init(void)
{
    printf("nros_app_init: registering nano-ros task\r\n");

    // nros_app_rust_entry() calls run(), which calls xTaskCreate() and returns.
    // The FreeRTOS task will start when the scheduler runs.
    nros_app_rust_entry();
}
```

---

## 3. The Application: What You Actually Write

This is the user-facing code — the part that lives in the closure passed to `run()`.
It looks almost identical to the QEMU FreeRTOS example, with only the board crate
import changed.

### IMU Publisher (I2C → `/imu/data_raw`)

```rust
// nros-orin-spe-apps/src/imu_publisher.rs
#![no_std]

use nros::prelude::*;
use nros_orin_spe::{Config, println, run};
use sensor_msgs::msg::{Imu, Vector3};
use std_msgs::msg::Header;

// FSP I2C driver FFI (fsp/source/include/i2c/tegra-i2c.h)
extern "C" {
    fn i2c_read_bytes(bus: u32, addr: u8, reg: u8, buf: *mut u8, len: usize) -> i32;
    fn gte_get_timestamp_ns() -> u64; // GTE hardware timestamp
}

fn read_bmi160_imu() -> Imu {
    let mut raw = [0u8; 12];
    // BMI160 data registers: 0x0C–0x17 (acc + gyro)
    unsafe { i2c_read_bytes(8, 0x68, 0x0C, raw.as_mut_ptr(), 12) };

    let ax = i16::from_le_bytes([raw[0], raw[1]]) as f64 * 9.81 / 16384.0;
    let ay = i16::from_le_bytes([raw[2], raw[3]]) as f64 * 9.81 / 16384.0;
    let az = i16::from_le_bytes([raw[4], raw[5]]) as f64 * 9.81 / 16384.0;
    let gx = i16::from_le_bytes([raw[6],  raw[7]])  as f64 * 0.00106; // rad/s
    let gy = i16::from_le_bytes([raw[8],  raw[9]])  as f64 * 0.00106;
    let gz = i16::from_le_bytes([raw[10], raw[11]]) as f64 * 0.00106;

    let ts_ns = unsafe { gte_get_timestamp_ns() };

    Imu {
        header: Header {
            stamp: builtin_interfaces::msg::Time {
                sec:     (ts_ns / 1_000_000_000) as i32,
                nanosec: (ts_ns % 1_000_000_000) as u32,
            },
            frame_id: "imu_link",
        },
        linear_acceleration: Vector3 { x: ax, y: ay, z: az },
        angular_velocity:    Vector3 { x: gx, y: gy, z: gz },
        // SPE doesn't compute orientation — Autoware EKF does that
        orientation_covariance: [-1.0; 9],
        ..Imu::default()
    }
}

// This fn is #[no_mangle] so nros-app.c can call it
#[unsafe(no_mangle)]
pub extern "C" fn nros_app_rust_entry() {
    run(Config::default(), |config| {
        let exec_config = ExecutorConfig::new(config.zenoh_locator)
            .domain_id(config.domain_id)
            .node_name("spe_imu_node");

        // Executor::<_, MAX_CBS, CB_ARENA>
        // MAX_CBS=0, CB_ARENA=0 → publisher-only, no callbacks needed
        let mut executor = Executor::<_, 0, 0>::open(&exec_config)?;
        let mut node = executor.create_node("spe_imu_node")?;

        println!("SPE: declaring publisher on /imu/data_raw");
        let publisher = node.create_publisher::<Imu>("/imu/data_raw")?;
        println!("SPE: publisher ready, starting 100 Hz loop");

        loop {
            let msg = read_bmi160_imu();
            publisher.publish(&msg)?;

            // spin_once(10) = pump zenoh-pico for up to 10ms,
            // then return so we can read the next IMU sample
            executor.spin_once(10);
        }
    });
}
```

### GPIO Watchdog + Heartbeat

```rust
// A second example: SPE monitors Autoware heartbeat via IVC,
// asserts emergency-stop GPIO if it goes silent

#[unsafe(no_mangle)]
pub extern "C" fn nros_app_rust_entry() {
    run(Config::default(), |config| {
        let exec_config = ExecutorConfig::new(config.zenoh_locator)
            .node_name("spe_watchdog");

        // MAX_CBS=4: we have one subscription callback
        let mut executor = Executor::<_, 4, 512>::open(&exec_config)?;
        let mut node = executor.create_node("spe_watchdog")?;

        // Subscribe to Autoware's heartbeat
        let mut heartbeat_sub =
            node.create_subscription::<std_msgs::msg::Bool>("/autoware/heartbeat")?;

        let mut last_beat_tick: u32 = freertos_get_tick_count();
        const TIMEOUT_TICKS: u32 = 3000; // 3 s watchdog window

        loop {
            executor.spin_once(10);

            if let Some(_hb) = heartbeat_sub.try_recv()? {
                last_beat_tick = freertos_get_tick_count();
                // Clear e-stop if previously asserted
                unsafe { gpio_set(AON_GPIO_BB1, 0) };
            }

            // Check timeout
            let elapsed = freertos_get_tick_count().wrapping_sub(last_beat_tick);
            if elapsed > TIMEOUT_TICKS {
                println!("SPE watchdog: heartbeat timeout! asserting e-stop");
                unsafe { gpio_set(AON_GPIO_BB1, 1) }; // active-high e-stop
            }
        }
    });
}
```

### Service Server (on-SPE configuration)

```rust
// Expose an ROS 2 service to let Autoware query SPE sensor state

use rcl_interfaces::srv::{GetParameters, GetParametersResponse};

#[unsafe(no_mangle)]
pub extern "C" fn nros_app_rust_entry() {
    run(Config::default(), |config| {
        let exec_config = ExecutorConfig::new(config.zenoh_locator)
            .node_name("spe_param_server");

        // Service requires callback arena: MAX_CBS=4, CB_ARENA=256
        let mut executor = Executor::<_, 4, 256>::open(&exec_config)?;

        executor.add_service::<GetParameters, _>(
            "/spe_node/get_parameters",
            |req| {
                println!("SPE: parameter query for {} names", req.names.len());
                GetParametersResponse {
                    values: req.names.iter().map(|_| {
                        rcl_interfaces::msg::ParameterValue {
                            type_: 3, // PARAMETER_DOUBLE
                            double_value: 100.0, // e.g. IMU rate Hz
                            ..Default::default()
                        }
                    }).collect(),
                }
            },
        )?;

        println!("SPE: service server ready");
        loop { executor.spin_once(10); }
    });
}
```

---

## 4. `target_specific.mk` Change

```makefile
# soc/t23x/target_specific.mk

# Existing NVIDIA apps — leave as-is
ENABLE_GPIO_APP         := 0
ENABLE_GTE_APP          := 0
ENABLE_I2C_APP          := 0
ENABLE_IVC_ECHO         := 1    # keep if you want the echo channel

# nano-ros addition
ENABLE_NROS_APP         := 1

# Path to the compiled Rust static library
NROS_LIB_DIR            := $(SPE_FREERTOS_BSP)/../nros-orin-spe/target/thumbv7r-none-eabihf/release
```

## 5. `Makefile` Addition

```makefile
ifeq ($(ENABLE_NROS_APP), 1)
CFLAGS += -DENABLE_NROS_APP
CSRCS  += $(DEMO_DIR)/app/nros-app.c
# Link the Rust static library
LDFLAGS += -L$(NROS_LIB_DIR) -lnros_orin_spe
# Rust's compiler-builtins (provides __aeabi_* intrinsics)
LDFLAGS += -L$(NROS_LIB_DIR) -lcompiler_builtins
endif
```

## 6. `platform/main.c` Hook

```c
// After all existing NVIDIA inits, before vTaskStartScheduler()

#ifdef ENABLE_IVC_ECHO
    ivc_echo_init();
#endif

// nano-ros — always last
#ifdef ENABLE_NROS_APP
    nros_app_init();
#endif

vTaskStartScheduler();  // never returns
```

---

## 7. Build Script

```bash
#!/bin/bash
# build-spe-nros.sh

set -e

# Step 1: build the Rust board crate
cd nros-orin-spe
CROSS_COMPILE=arm-none-eabi- \
FREERTOS_DIR=$SPE_FREERTOS_BSP/FreeRTOSV10.4.3/FreeRTOS/Source \
FSP_DIR=$SPE_FREERTOS_BSP/fsp/source \
cargo build --release --target armv7r-none-eabihf
cd ..

# Step 2: build the SPE firmware (C + Rust linked together)
cd rt-aux-cpu-demo-fsp
export CROSS_COMPILE=~/toolchains/gcc-arm-none-eabi-7-2018-q2-update/bin/arm-none-eabi-
export SPE_FREERTOS_BSP=$(realpath ..)
make -j$(nproc) bin_t23x
cd ..

# Step 3: flash SPE partition only
cp rt-aux-cpu-demo-fsp/out/t23x/spe.bin ~/Linux_for_Tegra/bootloader/spe_t234.bin
cd ~/Linux_for_Tegra
sudo ./flash.sh -k A_spe-fw jetson-agx-orin-devkit mmcblk0p1
```

---

## 8. Key Differences vs. the QEMU Example

| | QEMU FreeRTOS example | Orin SPE |
|---|---|---|
| Board crate | `nros-mps2-an385-freertos` | `nros-orin-spe` (new) |
| Entry point | `extern "C" fn _start()` | `extern "C" fn nros_app_rust_entry()` |
| `run()` returns | Never (`-> !`) | Returns (scheduler already running) |
| Network | LAN9118 Ethernet + lwIP | IVC shared memory |
| Network init | Inside task, after scheduler | `ivc::init()` inside task |
| Debug output | semihosting `hprintln!` | FSP `printf` → ttyTCU0 |
| Stack size | 16384 words (64 KB) | 4096 words (16 KB) — SPE SRAM limited |
| `Executor` type | `Executor::<_, 0, 0>` for pub-only | Same |
| CPU target | `thumbv7m-none-eabi` (Cortex-M3) | `armv7r-none-eabihf` (Cortex-R5F) |
| App code | Identical | Identical ✓ |

The last row is the whole point: **the application code inside the closure is
completely portable**. The only adaptation is in the board crate layer below it.

---

## 9. Memory Budget (256 KB BTCM)

The SPE has a single `btcm` region at `0x0c480000`, size `0x40000` (256 KB). Everything
must fit: code, static data, heap, and stacks.

| Component | Estimated Size | Notes |
|-----------|---------------|-------|
| FreeRTOS kernel | ~40 KB | heap_3 (malloc-based), 1 kHz tick, 32 priorities |
| NVIDIA FSP drivers | ~20 KB | IVC, HSP, TCU, GPIO (only what's linked) |
| zenoh-pico | 60–80 KB | With IVC link only (no TCP/UDP/serial) |
| nros runtime | ~15 KB | Executor, Node, CDR serialization |
| Sentinel algorithms | ~30 KB | Watchdog + emergency stop + gate (no trajectory follower) |
| Message types | ~40 KB | Reduced set (no debug topics) |
| FreeRTOS stacks | ~16 KB | nros task (4096 words) + idle + timer |
| Heap headroom | ~15 KB | Runtime allocations |
| **Total** | **~236–256 KB** | **Tight fit — LTO + opt-level="z" required** |

If the budget is exceeded, options (in order of preference):
1. Enable LTO and `opt-level = "z"` for all Rust crates
2. Strip unused zenoh-pico features (`Z_FEATURE_QUERYABLE=0`, `Z_FEATURE_QUERY=0`)
3. Reduce sentinel to watchdog-only (drop gate and emergency stop algorithms)
4. Map zenoh-pico to DRAM via AST (trades latency for space)

---

## 10. Roadmap

See [Phase 11: Orin SPE Deployment](../roadmap/11-orin-spe.md) for the phased
implementation plan, covering IVC verification, zenoh-pico link backend, board crate,
cross-compilation, sentinel adaptation, bridge daemon, and deployment.
