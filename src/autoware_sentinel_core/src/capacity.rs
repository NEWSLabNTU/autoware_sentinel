// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Phase 15.3 — capacity census, asserted BEFORE wiring.
//!
//! Family B of the phase-15 audit: six incidents where a compile-time
//! capacity smaller than the declared topology surfaced at runtime as a hang
//! or an opaque error, minutes-to-hours after boot and far from its cause:
//!
//! | Cap | Symptom we actually saw |
//! |-----|-------------------------|
//! | `NROS_EXECUTOR_MAX_CBS` (default 4) | `create_timer code=-6 Full`, later a stack-overflow / malloc-failed cascade |
//! | `NROS_EXECUTOR_MAX_NODES` (default 4) | `NodeTableFull` |
//! | rmw-cffi subscription pool (was 4) | `SubscriberCreationFailed` |
//! | FreeRTOS app stack | `*** STACK OVERFLOW ***` mid-registration |
//! | Zephyr zpico slots (8/8/8/16) | register pass hung, no output at all |
//! | Zephyr pthread mutex pool (32) | `z_declare_publisher: -1` on the 28th declare |
//!
//! Every one was a one-line fix after hours of bisection. This module counts
//! what the sentinel is about to declare and compares it against the
//! capacities compiled into the binary, so the failure arrives as a sentence
//! naming the knob and both numbers — before a single entity is created.
//!
//! The upstream fix is nano-ros issue 0257 (derive the knobs from the launch
//! model). Until that lands, this is the consumer-side smoke alarm.

use log::{error, warn};

/// What a wiring pass is about to declare.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Census {
    /// Executor nodes (`node_builder(..).build()`).
    pub nodes: usize,
    /// Subscriptions — one executor callback slot each.
    pub subscriptions: usize,
    /// Service servers — one executor callback slot each.
    pub services: usize,
    /// Timers — one executor callback slot each.
    pub timers: usize,
    /// Publishers. No executor callback slot, but they consume RMW-side
    /// publisher slots (`ZPICO_MAX_PUBLISHERS` / `CONFIG_NROS_MAX_PUBLISHERS`)
    /// and one liveliness token apiece.
    pub publishers: usize,
}

impl Census {
    /// Executor callback slots this topology needs.
    pub const fn callbacks(&self) -> usize {
        self.subscriptions + self.services + self.timers
    }

    /// Liveliness tokens: one per entity plus one NN token per node.
    pub const fn liveliness_tokens(&self) -> usize {
        self.nodes + self.publishers + self.subscriptions + self.services
    }
}

/// Verdict for one capacity.
struct Verdict {
    knob: &'static str,
    needed: usize,
    compiled: usize,
    /// Some caps are not readable from here (they live in C or Kconfig); we
    /// report the requirement so a human can compare, without asserting.
    enforceable: bool,
}

/// Check the census against every capacity this binary can see.
///
/// Returns `Err(NodeError::ExecutorFull)` when an enforceable capacity is too
/// small, after logging the knob, the requirement and the compiled value.
/// Non-enforceable caps (RMW/C/Kconfig side) are logged at WARN with the
/// number to set, because those are exactly the ones that hang instead of
/// erroring.
pub fn check(census: &Census) -> Result<(), nros::NodeError> {
    let compiled_cbs = nros::ExecutorSizing::DEFAULT.cbs;

    let verdicts = [
        Verdict {
            knob: "NROS_EXECUTOR_MAX_CBS",
            needed: census.callbacks(),
            compiled: compiled_cbs,
            enforceable: true,
        },
        // The executor's node table capacity is not exported; report the
        // requirement so a `NodeTableFull` at build() has a number to match.
        Verdict {
            knob: "NROS_EXECUTOR_MAX_NODES",
            needed: census.nodes,
            compiled: 0,
            enforceable: false,
        },
        // RMW-side slots live in the C shim (ZPICO_MAX_*) or, on Zephyr, in
        // Kconfig (CONFIG_NROS_MAX_*). Both hang rather than error when
        // exceeded — the Zephyr register pass produced no output at all.
        Verdict {
            knob: "ZPICO_MAX_PUBLISHERS / CONFIG_NROS_MAX_PUBLISHERS",
            needed: census.publishers,
            compiled: 0,
            enforceable: false,
        },
        Verdict {
            knob: "ZPICO_MAX_SUBSCRIBERS / CONFIG_NROS_MAX_SUBSCRIBERS",
            needed: census.subscriptions,
            compiled: 0,
            enforceable: false,
        },
        Verdict {
            knob: "ZPICO_MAX_QUERYABLES / CONFIG_NROS_MAX_QUERYABLES",
            needed: census.services,
            compiled: 0,
            enforceable: false,
        },
        Verdict {
            knob: "ZPICO_MAX_LIVELINESS / CONFIG_NROS_MAX_LIVELINESS",
            needed: census.liveliness_tokens(),
            compiled: 0,
            enforceable: false,
        },
    ];

    let mut fatal = false;
    for v in &verdicts {
        if v.enforceable {
            if v.needed > v.compiled {
                error!(
                    "capacity: {} needs {} but this binary compiled {} — set {}={} and REBUILD \
                     (a stale arena also SEGVs; clean build after resizing)",
                    v.knob, v.needed, v.compiled, v.knob, v.needed
                );
                fatal = true;
            }
        } else {
            warn!(
                "capacity: {} must be >= {} for this topology (not readable here; exceeding it \
                 HANGS instead of erroring)",
                v.knob, v.needed
            );
        }
    }

    if fatal {
        error!(
            "capacity: refusing to wire {} nodes / {} callbacks / {} publishers against a \
             too-small executor — phase 15.3 pre-wiring check",
            census.nodes,
            census.callbacks(),
            census.publishers
        );
        return Err(nros::NodeError::ExecutorFull);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn callbacks_sum_the_slot_consumers() {
        let c = Census {
            nodes: 13,
            subscriptions: 9,
            services: 21,
            timers: 1,
            publishers: 37,
        };
        assert_eq!(c.callbacks(), 31);
        // One NN token per node + one per entity.
        assert_eq!(c.liveliness_tokens(), 13 + 37 + 9 + 21);
    }

    #[test]
    fn publishers_do_not_consume_callback_slots() {
        // The phase-14 mistake was assuming publishers ate executor slots and
        // sizing MAX_CBS accordingly; they do not.
        let c = Census {
            publishers: 100,
            ..Default::default()
        };
        assert_eq!(c.callbacks(), 0);
    }
}
