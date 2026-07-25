// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Sentinel `/control/autoware_shift_decider`.
//!
//! Phase 14.4b declarative wrapper over the shared safety-chain crossbar.

#![no_std]

use autoware_sentinel_core::{ensure_island_default, with_island};
use autoware_system_msgs::msg::AutowareState;
use autoware_vehicle_msgs::msg::{GearCommand, GearReport};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};

pub struct ShiftDeciderNode;

impl Node for ShiftDeciderNode {
    const NAME: &'static str = "autoware_shift_decider";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node =
            ctx.create_node(NodeOptions::new("autoware_shift_decider").namespace("/control"))?;
        let s1 = node.create_subscription_for_callback_name::<AutowareState>(
            "on_state",
            "/autoware/state",
        )?;
        node.callback_for_name("on_state").reads_entity(&s1)?;
        let s2 = node.create_subscription_for_callback_name::<GearReport>(
            "on_gear",
            "/vehicle/status/gear_status",
        )?;
        node.callback_for_name("on_gear").reads_entity(&s2)?;
        node.create_publisher_for_topic::<GearCommand>("/control/shift_decider/gear_cmd")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(33))?;
        Ok(())
    }
}

impl ExecutableNode for ShiftDeciderNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "on_state" => {
                if let Ok(msg) = ctx.message::<AutowareState>() {
                    with_island(|island| island.on_autoware_state(&msg));
                }
            }
            "on_gear" => {
                if let Ok(msg) = ctx.message::<GearReport>() {
                    with_island(|island| island.on_gear_report(&msg));
                }
            }
            "on_tick" => {
                let out = with_island(|island| island.last_outputs());
                let _ = ctx.publish_to_topic::<GearCommand, 128>(
                    "/control/shift_decider/gear_cmd",
                    &GearCommand {
                        command: out.auto_gear,
                        ..Default::default()
                    },
                );
            }
            _ => {}
        }
    }
}

nros::node!(ShiftDeciderNode);
