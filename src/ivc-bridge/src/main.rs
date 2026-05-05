// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Autoware Sentinel — Linux IVC ↔ TCP bridge daemon.
//!
//! Forwards zenoh batches between an IVC mailbox (fixed 64-byte frames
//! with a 4-byte `total_len + offset` header) and a TCP zenohd router
//! on `127.0.0.1:7447`. Pairs with the SPE-side `Z_FEATURE_LINK_IVC`
//! link transport (nano-ros Phase 100.4); together they let the
//! AGX Orin SPE sentinel reach the host's `rmw_zenoh_cpp` Autoware
//! deployment without a TCP capable interface on the SPE side.
//!
//! ## Wire framing (must match `nano-ros-sentinel/docs/roadmap/phase-100-04-link-ivc-design.md` §5.2)
//!
//! ```text
//!  byte 0   1   2   3   4   5   ...                                63
//!     +---+---+---+---+---+---+---+---+---+---+---+---+---+---+...+---+
//!     | total_len (u16, LE) | offset (u16, LE) |  payload (≤ 60 B)   |
//!     +---+---+---+---+---+---+---+---+---+---+---+---+---+---+...+---+
//! ```
//!
//! - `total_len = 0` + `offset = 0` is a keep-alive ping; bridge drops
//!   silently.
//! - `total_len > Z_BATCH_UNICAST_SIZE` (2048) is treated as a wire
//!   violation and resets the reassembly state; nothing is forwarded.
//! - In-order SPSC delivery is assumed (Tegra IVC ring + Unix
//!   `SOCK_DGRAM` both preserve frame boundaries and ordering).
//!
//! ## Backends
//!
//! - `unix-mock` (default): bind a `UnixDatagram` at the socket path
//!   given on the CLI. The SPE-side process connects there and the
//!   pair acts as the IVC ring's two ends. Used by the autoware_sentinel
//!   POSIX dev path and by 11.2 / 11.3 integration tests.
//! - `fsp-sysfs` (Phase 11.6, TODO): read/write the Tegra IVC sysfs
//!   node `/sys/devices/platform/.../data_channel`. Same framing.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::os::unix::net::UnixDatagram;
use std::path::PathBuf;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use clap::Parser;

const FRAME_SIZE: usize = 64;
const HEADER_SIZE: usize = 4;
const MAX_PAYLOAD: usize = FRAME_SIZE - HEADER_SIZE;
const MAX_BATCH: usize = 2048;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Cli {
    /// IVC backend to use.
    #[arg(long, value_enum, default_value_t = Backend::UnixMock)]
    backend: Backend,

    /// Unix socket path to bind on (unix-mock backend).
    #[arg(long, default_value = "/tmp/autoware-sentinel-ivc.sock")]
    ivc_path: PathBuf,

    /// TCP zenohd locator (host side).
    #[arg(long, default_value = "127.0.0.1:7447")]
    tcp_addr: String,
}

#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum Backend {
    UnixMock,
    FspSysfs,
}

fn main() -> std::io::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();

    match cli.backend {
        Backend::UnixMock => run_unix_mock(&cli),
        Backend::FspSysfs => {
            log::error!(
                "fsp-sysfs backend is not yet wired (Phase 11.6 TODO); \
                 falling back to unix-mock for development."
            );
            std::process::exit(1);
        }
    }
}

fn run_unix_mock(cli: &Cli) -> std::io::Result<()> {
    // Remove any stale socket file from a previous run.
    let _ = std::fs::remove_file(&cli.ivc_path);

    let ivc = UnixDatagram::bind(&cli.ivc_path)?;
    log::info!("IVC bridge bound at {:?}", cli.ivc_path);

    // Connect to zenohd. The SPE-side sentinel won't ship batches until
    // the host process accepts them, so it's safe to block briefly.
    let mut tcp = loop {
        match TcpStream::connect(&cli.tcp_addr) {
            Ok(s) => {
                log::info!("Connected to zenohd at {}", cli.tcp_addr);
                break s;
            }
            Err(e) => {
                log::warn!(
                    "zenohd at {} not yet reachable ({e}); retrying in 1 s",
                    cli.tcp_addr
                );
                thread::sleep(Duration::from_secs(1));
            }
        }
    };
    tcp.set_nodelay(true)?;

    // Channels carry already-reassembled zenoh batches between the two
    // forwarder threads.
    let (ivc_to_tcp_tx, ivc_to_tcp_rx) = mpsc::channel::<Vec<u8>>();

    // IVC → TCP: read frames from the Unix socket, reassemble batches,
    // forward to zenohd.
    let ivc_clone = ivc.try_clone()?;
    let ivc_path = cli.ivc_path.clone();
    thread::spawn(move || ivc_to_tcp_loop(ivc_clone, ivc_path, ivc_to_tcp_tx));

    // The TCP-side TX path runs in the main thread so we can use a
    // simple blocking write loop without splitting the TcpStream.
    // Spawn a separate thread for the TCP→IVC direction.
    let mut tcp_clone = tcp.try_clone()?;
    let ivc_clone2 = ivc.try_clone()?;
    thread::spawn(move || tcp_to_ivc_loop(&mut tcp_clone, &ivc_clone2));

    for batch in ivc_to_tcp_rx {
        // Forward a length-prefixed batch over TCP. zenohd's TCP
        // transport already frames its own messages — the link layer's
        // MTU contract is one batch per send. We just write the raw
        // bytes through; zenohd handles framing on its end via the
        // standard link-internal length-prefixed protocol.
        if let Err(e) = tcp.write_all(&batch) {
            log::error!("TCP write failed: {e}; bridge exiting");
            return Err(e);
        }
    }
    Ok(())
}

fn ivc_to_tcp_loop(
    ivc: UnixDatagram,
    _path: PathBuf,
    sink: mpsc::Sender<Vec<u8>>,
) {
    let mut frame = [0u8; FRAME_SIZE];
    let mut rx_buf = vec![0u8; MAX_BATCH];
    let mut expected_total: u16 = 0;
    let mut bytes_received: u16 = 0;

    loop {
        let n = match ivc.recv(&mut frame) {
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(1));
                continue;
            }
            Err(e) => {
                log::error!("IVC recv failed: {e}");
                return;
            }
        };
        if n < HEADER_SIZE {
            log::warn!("Runt frame ({n} B); resetting reassembly state");
            expected_total = 0;
            bytes_received = 0;
            continue;
        }
        let total = u16::from_le_bytes([frame[0], frame[1]]);
        let off = u16::from_le_bytes([frame[2], frame[3]]);
        let payload_len = (n - HEADER_SIZE) as u16;
        if total == 0 && off == 0 {
            // Keep-alive ping — drop silently.
            continue;
        }
        if total as usize > MAX_BATCH {
            log::warn!("Oversized batch (total_len={total}); dropping");
            expected_total = 0;
            bytes_received = 0;
            continue;
        }
        if expected_total == 0 {
            expected_total = total;
            bytes_received = 0;
        } else if total != expected_total {
            log::warn!(
                "Mid-batch total_len changed ({expected_total} → {total}); resetting"
            );
            expected_total = total;
            bytes_received = 0;
        }
        if (off + payload_len) as usize > expected_total as usize {
            log::warn!(
                "Frame overruns batch (offset={off}, payload={payload_len}, total={expected_total}); dropping"
            );
            expected_total = 0;
            bytes_received = 0;
            continue;
        }
        rx_buf[off as usize..(off + payload_len) as usize]
            .copy_from_slice(&frame[HEADER_SIZE..n]);
        bytes_received += payload_len;
        if bytes_received == expected_total {
            let batch = rx_buf[..expected_total as usize].to_vec();
            log::debug!("IVC → TCP: {} B batch", batch.len());
            if sink.send(batch).is_err() {
                log::info!("TCP forwarder dropped; IVC reader exiting");
                return;
            }
            expected_total = 0;
            bytes_received = 0;
        }
    }
}

fn tcp_to_ivc_loop(tcp: &mut TcpStream, ivc: &UnixDatagram) {
    let mut buf = vec![0u8; MAX_BATCH];
    loop {
        // Read up to MAX_BATCH from the TCP stream. zenoh-pico's TCP
        // link is byte-oriented; the upper-layer batches that arrive
        // here may not align to read boundaries, but the link layer's
        // own length-prefix framing inside the zenoh protocol means
        // each TCP read can land on a partial zenoh batch. For the
        // simple bridge case, we forward whatever we get as a
        // single-batch IVC fragment train and let zenohd / zenoh-pico
        // handle protocol framing end-to-end.
        match tcp.read(&mut buf) {
            Ok(0) => {
                log::info!("TCP peer closed; bridge exiting");
                return;
            }
            Ok(n) => {
                if let Err(e) = forward_batch_as_ivc_frames(ivc, &buf[..n]) {
                    log::error!("IVC send failed: {e}");
                    return;
                }
            }
            Err(e) => {
                log::error!("TCP read failed: {e}");
                return;
            }
        }
    }
}

/// Fragment a zenoh batch into 64-byte IVC frames per §5.2 of the design
/// doc and shove them through the Unix datagram socket.
fn forward_batch_as_ivc_frames(ivc: &UnixDatagram, batch: &[u8]) -> std::io::Result<()> {
    if batch.len() > MAX_BATCH {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "batch exceeds Z_BATCH_UNICAST_SIZE",
        ));
    }
    let total = batch.len() as u16;
    let mut off: usize = 0;
    let mut frame = [0u8; FRAME_SIZE];
    while off < batch.len() {
        let chunk = (batch.len() - off).min(MAX_PAYLOAD);
        frame[0..2].copy_from_slice(&total.to_le_bytes());
        frame[2..4].copy_from_slice(&(off as u16).to_le_bytes());
        frame[HEADER_SIZE..HEADER_SIZE + chunk].copy_from_slice(&batch[off..off + chunk]);
        ivc.send(&frame[..HEADER_SIZE + chunk])?;
        off += chunk;
    }
    log::debug!("TCP → IVC: {} B batch in {} frames", batch.len(), off.div_ceil(MAX_PAYLOAD));
    Ok(())
}
