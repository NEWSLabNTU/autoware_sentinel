// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Sentinel `/sensing/vehicle_velocity_converter` — declarative wrapper
//! (phase 14.4b pilot).
//!
//! Owns the fused velocity pipeline subscription: `VelocityReport` →
//! velocity converter → stop filter → twist2accel, all running inside the
//! shared [`autoware_sentinel_core`] crossbar (`with_island`). This node has
//! no publishers of its own — the safety chain consumes the crossbar state
//! and the command-path nodes publish the results.

#![no_std]

use autoware_sentinel_core::{ensure_island_default, with_island};
use autoware_vehicle_msgs::msg::VelocityReport;
use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};

pub struct VelocityConverterNode;

impl Node for VelocityConverterNode {
    const NAME: &'static str = "vehicle_velocity_converter";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        // Crossbar first-touch: whichever sentinel wrapper registers first
        // initializes the SafetyIsland from compile-time defaults.
        ensure_island_default();

        let mut node = ctx.create_node(NodeOptions::new("vehicle_velocity_converter"))?;
        let sub = node.create_subscription_for_callback_name::<VelocityReport>(
            "on_velocity",
            "/vehicle/status/velocity_status",
        )?;
        node.callback_for_name("on_velocity").reads_entity(&sub)?;
        Ok(())
    }
}

impl ExecutableNode for VelocityConverterNode {
    type State = ();

    fn init() -> Self::State {}

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_velocity"
            && let Ok(msg) = ctx.message::<VelocityReport>()
        {
            with_island(|island| island.on_velocity_report(&msg));
        }
    }
}

nros::node!(VelocityConverterNode);
