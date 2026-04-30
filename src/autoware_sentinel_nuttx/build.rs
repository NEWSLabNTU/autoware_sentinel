// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// NuttX flat-build link script + library wiring. The Rust binary IS the
// NuttX kernel image: `arm-none-eabi-gcc` is invoked as the linker via
// `.cargo/config.toml`, and this script tells it where the staging libs
// and preprocessed `dramboot.ld` live.
//
// Mirrors the talker example in nano-ros so future kernel/linker layout
// changes only need to be tracked in one place. NUTTX_DIR must be set
// (typically by the justfile recipe) and must contain a built kernel
// (run `just build-sentinel-nuttx` first, which calls
// `packages/boards/nros-board-nuttx-qemu-arm/scripts/build-nuttx.sh`).

use std::path::PathBuf;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-env-changed=NUTTX_DIR");

    let nuttx_dir = match std::env::var("NUTTX_DIR") {
        Ok(dir) => PathBuf::from(dir),
        Err(_) => return,
    };

    let staging = nuttx_dir.join("staging");
    if !staging.join("libc.a").exists() {
        return;
    }

    // Preprocess linker script (has #include <nuttx/config.h>).
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let processed_ld = out_dir.join("dramboot.ld");
    let linker_script = nuttx_dir.join("boards/arm/qemu/qemu-armv7a/scripts/dramboot.ld");

    let status = Command::new("arm-none-eabi-gcc")
        .args([
            "-E", "-P", "-x", "c",
            &format!("-isystem{}", nuttx_dir.join("include").display()),
            "-D__NuttX__", "-D__KERNEL__",
            &format!("-I{}", nuttx_dir.join("arch/arm/src/chip").display()),
            &format!("-I{}", nuttx_dir.join("arch/arm/src/common").display()),
            &format!("-I{}", nuttx_dir.join("arch/arm/src/armv7-a").display()),
            &format!("-I{}", nuttx_dir.join("sched").display()),
        ])
        .arg(&linker_script)
        .arg("-o").arg(&processed_ld)
        .status()
        .expect("failed to preprocess NuttX linker script — install arm-none-eabi-gcc");
    assert!(status.success(), "linker script preprocessing failed");

    let board_src = nuttx_dir.join("arch/arm/src/board");
    let vectortab = nuttx_dir.join("arch/arm/src/arm_vectortab.o");

    // Discover libgcc.a path matching our cortex-a7/neon-vfpv4 multilib.
    let gcc_out = Command::new("arm-none-eabi-gcc")
        .args([
            "-mcpu=cortex-a7",
            "-mfloat-abi=hard",
            "-mfpu=neon-vfpv4",
            "-print-libgcc-file-name",
        ])
        .output()
        .expect("failed to find libgcc");
    let libgcc = String::from_utf8(gcc_out.stdout).unwrap().trim().to_string();

    // NuttX flat-build: the Rust binary IS the kernel.
    println!("cargo:rustc-link-arg=-T{}", processed_ld.display());
    println!("cargo:rustc-link-arg=--entry=__start");
    println!("cargo:rustc-link-arg=-nostartfiles");
    println!("cargo:rustc-link-arg=-nodefaultlibs");
    println!("cargo:rustc-link-arg={}", vectortab.display());
    println!("cargo:rustc-link-arg=-L{}", staging.display());
    println!("cargo:rustc-link-arg=-L{}", board_src.display());
    println!("cargo:rustc-link-arg=-Wl,--start-group");
    for lib in [
        "sched", "drivers", "boards", "c", "mm", "arch", "xx", "apps", "net", "crypto",
        "fs", "binfmt", "openamp", "board",
    ] {
        println!("cargo:rustc-link-arg=-l{lib}");
    }
    println!("cargo:rustc-link-arg={libgcc}");
    println!("cargo:rustc-link-arg=-Wl,--end-group");

    println!("cargo:rerun-if-changed={}", linker_script.display());
}
