/// Subscriber buffer size (set via ZPICO_SUBSCRIBER_BUFFER_SIZE, default 1024).
pub const SUBSCRIBER_BUFFER_SIZE: usize = 1024;
/// Service request buffer size (set via ZPICO_SERVICE_BUFFER_SIZE, default 1024).
pub const SERVICE_BUFFER_SIZE: usize = 1024;
/// Default service client RPC timeout in milliseconds
/// (set via NROS_SERVICE_TIMEOUT_MS, default 10000).
pub const SERVICE_DEFAULT_TIMEOUT_MS: u32 = 10000;
/// Maximum key expression string size for topic/service names
/// (set via NROS_KEYEXPR_STRING_SIZE, default 256).
pub const KEYEXPR_STRING_SIZE: usize = 256;
/// Key expression buffer size (KEYEXPR_STRING_SIZE + 1 for null terminator).
pub const KEYEXPR_BUFFER_SIZE: usize = 257;
