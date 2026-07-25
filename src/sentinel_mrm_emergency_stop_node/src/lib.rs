// Copyright 2026 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Sentinel `/system/mrm_emergency_stop_operator`.
//!
//! Phase 14.4b declarative wrapper over the shared safety-chain crossbar.

#![no_std]

use autoware_sentinel_core::{ensure_island_default, with_island};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use tier4_system_msgs::msg::MrmBehaviorStatus;
use tier4_system_msgs::srv::{OperateMrm, OperateMrmResponse};

const MRM_BEHAVIOR_AVAILABLE: u8 = 1;
const MRM_BEHAVIOR_OPERATING: u8 = 2;

pub struct EmergencyStopOperatorNode;

impl Node for EmergencyStopOperatorNode {
    const NAME: &'static str = "mrm_emergency_stop_operator";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node =
            ctx.create_node(NodeOptions::new("mrm_emergency_stop_operator").namespace("/system"))?;
        node.create_publisher_for_topic::<MrmBehaviorStatus>("/system/mrm/emergency_stop/status")?;
        node.create_service_server_for_name::<OperateMrm>("/system/mrm/emergency_stop/operate")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(33))?;
        Ok(())
    }
}

impl ExecutableNode for EmergencyStopOperatorNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "/system/mrm/emergency_stop/operate" => {
                // The MRM handler drives the operators through the chain; the
                // service exists for interface parity and acks the request.
                let _ = ctx.reply::<OperateMrmResponse, 256>(&OperateMrmResponse {
                    response: Default::default(),
                });
            }
            "on_tick" => {
                let out = with_island(|island| island.last_outputs());
                let state = if out.estop_operating {
                    MRM_BEHAVIOR_OPERATING
                } else {
                    MRM_BEHAVIOR_AVAILABLE
                };
                let _ = ctx.publish_to_topic::<MrmBehaviorStatus, 128>(
                    "/system/mrm/emergency_stop/status",
                    &MrmBehaviorStatus {
                        stamp: Default::default(),
                        state,
                    },
                );
            }
            _ => {}
        }
    }
}

nros::node!(EmergencyStopOperatorNode);
