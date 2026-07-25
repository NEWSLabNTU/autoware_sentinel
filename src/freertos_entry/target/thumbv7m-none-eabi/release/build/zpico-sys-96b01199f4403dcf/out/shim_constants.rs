/// Maximum number of concurrent publishers (set via ZPICO_MAX_PUBLISHERS, default 8).
pub const ZPICO_MAX_PUBLISHERS: usize = 56;
/// Maximum number of concurrent subscribers (set via ZPICO_MAX_SUBSCRIBERS, default 8).
pub const ZPICO_MAX_SUBSCRIBERS: usize = 32;
/// Maximum number of concurrent queryables (set via ZPICO_MAX_QUERYABLES, default 8).
pub const ZPICO_MAX_QUERYABLES: usize = 32;
/// Maximum number of concurrent liveliness tokens (set via ZPICO_MAX_LIVELINESS, default 16).
pub const ZPICO_MAX_LIVELINESS: usize = 160;
/// Maximum number of concurrent pending get operations (set via ZPICO_MAX_PENDING_GETS, default 4).
pub const ZPICO_MAX_PENDING_GETS: usize = 4;
