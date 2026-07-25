/// Subscriber buffer size (set via ZPICO_SUBSCRIBER_BUFFER_SIZE, default 1024).
pub const SUBSCRIBER_BUFFER_SIZE: usize = 1024;
/// Service request buffer size (set via ZPICO_SERVICE_BUFFER_SIZE, default 1024).
pub const SERVICE_BUFFER_SIZE: usize = 1024;
/// Default service client RPC timeout in milliseconds
/// (set via NROS_SERVICE_TIMEOUT_MS, default 30000).
pub const SERVICE_DEFAULT_TIMEOUT_MS: u32 = 30000;
/// Maximum key expression string size for topic/service names
/// (set via NROS_KEYEXPR_STRING_SIZE, default 256).
pub const KEYEXPR_STRING_SIZE: usize = 256;
/// Key expression buffer size (KEYEXPR_STRING_SIZE + 1 for null terminator).
pub const KEYEXPR_BUFFER_SIZE: usize = 257;
/// Phase 124.D.3.c — per-subscriber SPSC ring depth
/// (set via ZPICO_SUBSCRIBER_RING_DEPTH, default 4).
pub const SUBSCRIBER_RING_DEPTH: usize = 4;
/// Phase 231 (RFC-0038) — `large` size-class slot size
/// (set via ZPICO_SUBSCRIBER_LARGE_SIZE, default 16384).
pub const SUBSCRIBER_LARGE_SIZE: usize = 16384;
/// Phase 231 — rx_buffer_hint above this routes to the `large` class
/// (set via ZPICO_SUBSCRIBER_SIZE_THRESHOLD, default 2048).
pub const SUBSCRIBER_SIZE_THRESHOLD: usize = 2048;
/// Phase 231 — max concurrent `large`-class subscribers
/// (set via ZPICO_MAX_LARGE_SUBSCRIBERS, default 2).
pub const MAX_LARGE_SUBSCRIBERS: usize = 8;
/// Phase 268 — per-session per-node NN liveliness token cap, tracking
/// `nros-node`'s NROS_EXECUTOR_MAX_NODES (default 4): one session hosts
/// at most that many graph nodes.
pub const MAX_PER_NODE_LIVELINESS: usize = 16;
