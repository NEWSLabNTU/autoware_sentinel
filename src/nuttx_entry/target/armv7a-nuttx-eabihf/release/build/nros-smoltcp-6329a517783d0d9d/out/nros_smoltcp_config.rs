/// Maximum number of concurrent TCP sockets (NROS_SMOLTCP_MAX_SOCKETS; backend-derived default 1 — brokered).
pub const MAX_SOCKETS: usize = 1;

/// Maximum number of concurrent UDP sockets (NROS_SMOLTCP_MAX_UDP_SOCKETS; default 1 brokered / 4 with the `rtps` feature).
pub const MAX_UDP_SOCKETS: usize = 1;

/// Per-socket staging buffer size in bytes (set via NROS_SMOLTCP_BUFFER_SIZE, default 2048).
pub const SOCKET_BUFFER_SIZE: usize = 2048;

/// Timeout for TCP connect in milliseconds (set via NROS_SMOLTCP_CONNECT_TIMEOUT_MS, default 30000).
pub const CONNECT_TIMEOUT_MS: u64 = 30000;

/// Timeout for TCP read/write operations in milliseconds (set via NROS_SMOLTCP_SOCKET_TIMEOUT_MS, default 10000).
pub const SOCKET_TIMEOUT_MS: u64 = 10000;
