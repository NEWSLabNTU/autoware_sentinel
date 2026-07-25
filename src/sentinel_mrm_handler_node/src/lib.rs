// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Sentinel `/system/mrm_handler` — MRM state + emergency command mirrors.
//!
//! Phase 14.4b declarative wrapper over the shared safety-chain crossbar
//! (`autoware_sentinel_core::with_island`). Status publishers run on this
//! node's own 33 ms timer reading the last chain snapshot; the chain itself
//! is driven by the vehicle_cmd_gate node's timer (registered first in the
//! launch order).

#![no_std]

use autoware_adapi_v1_msgs::msg::MrmState;
use autoware_sentinel_core::{ensure_island_default, platform_now_ms, with_island};
use autoware_vehicle_msgs::msg::{GearCommand, HazardLightsCommand, TurnIndicatorsCommand};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use tier4_system_msgs::msg::EmergencyHoldingState;

pub struct MrmHandlerNode;

impl Node for MrmHandlerNode {
    const NAME: &'static str = "mrm_handler";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node = ctx.create_node(NodeOptions::new("mrm_handler").namespace("/system"))?;
        let sub = node
            .create_subscription_for_callback_name::<autoware_adapi_v1_msgs::msg::Heartbeat>(
                "on_heartbeat",
                "/api/system/heartbeat",
            )?;
        node.callback_for_name("on_heartbeat").reads_entity(&sub)?;
        let p1 = node.create_publisher_for_topic::<MrmState>("/system/fail_safe/mrm_state")?;
        let p2 = node.create_publisher_for_topic::<GearCommand>("/system/emergency/gear_cmd")?;
        let p3 = node.create_publisher_for_topic::<HazardLightsCommand>(
            "/system/emergency/hazard_lights_cmd",
        )?;
        let p4 = node.create_publisher_for_topic::<TurnIndicatorsCommand>(
            "/system/emergency/turn_indicators_cmd",
        )?;
        let p5 =
            node.create_publisher_for_topic::<EmergencyHoldingState>("/system/emergency_holding")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(33))?;
        node.callback_for_name("on_tick").publishes_entity(&p1)?;
        node.callback_for_name("on_tick").publishes_entity(&p2)?;
        node.callback_for_name("on_tick").publishes_entity(&p3)?;
        node.callback_for_name("on_tick").publishes_entity(&p4)?;
        node.callback_for_name("on_tick").publishes_entity(&p5)?;
        Ok(())
    }
}

impl ExecutableNode for MrmHandlerNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "on_heartbeat" => {
                let now = platform_now_ms();
                with_island(|island| island.on_heartbeat(now));
            }
            "on_tick" => {
                let out = with_island(|island| island.last_outputs());
                let _ = ctx.publish_to_topic::<MrmState, 512>(
                    "/system/fail_safe/mrm_state",
                    &out.mrm_state,
                );
                let _ = ctx.publish_to_topic::<GearCommand, 128>(
                    "/system/emergency/gear_cmd",
                    &out.mrm_gear,
                );
                let _ = ctx.publish_to_topic::<HazardLightsCommand, 128>(
                    "/system/emergency/hazard_lights_cmd",
                    &out.mrm_hazard,
                );
                let _ = ctx.publish_to_topic::<TurnIndicatorsCommand, 128>(
                    "/system/emergency/turn_indicators_cmd",
                    &TurnIndicatorsCommand::default(),
                );
                let _ = ctx.publish_to_topic::<EmergencyHoldingState, 128>(
                    "/system/emergency_holding",
                    &EmergencyHoldingState {
                        stamp: Default::default(),
                        is_holding: false,
                    },
                );
            }
            _ => {}
        }
    }
}

nros::node!(MrmHandlerNode);
