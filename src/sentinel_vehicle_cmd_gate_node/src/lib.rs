//! Sentinel `/control/vehicle_cmd_gate` — the safety-chain driver.
//!
//! Phase 14.4b: this node's 33 ms timer runs one
//! [`autoware_sentinel_core::SafetyIsland::chain_tick`] pass and publishes
//! the command topics from the fresh snapshot; the other wrapper nodes read
//! the same snapshot on their own timers (this node registers first in the
//! launch order).

#![no_std]

use autoware_control_msgs::msg::Control;
use autoware_internal_debug_msgs::msg::BoolStamped;
use autoware_internal_msgs::msg::PublishedTime;
use autoware_sentinel_core::{ensure_island_default, platform_now_ms, with_island};
use autoware_vehicle_cmd_gate_msgs::msg::IsFilterActivated;
use autoware_vehicle_msgs::msg::{GearCommand, HazardLightsCommand, TurnIndicatorsCommand};
use logging_demo::srv::{ConfigLogger, ConfigLoggerResponse};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_srvs::srv::{Trigger, TriggerResponse};
use tier4_control_msgs::msg::{GateMode, IsPaused, IsStartRequested, IsStopped};
use tier4_control_msgs::srv::{SetStop, SetStopResponse};
use tier4_vehicle_msgs::msg::VehicleEmergencyStamped;
use visualization_msgs::msg::MarkerArray;

const GATE_MODE_AUTO: u8 = 0;

pub struct VehicleCmdGateNode;

impl Node for VehicleCmdGateNode {
    const NAME: &'static str = "vehicle_cmd_gate";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node =
            ctx.create_node(NodeOptions::new("vehicle_cmd_gate").namespace("/control"))?;
        let sub = node.create_subscription_for_callback_name::<Control>(
            "on_control",
            "/control/trajectory_follower/control_cmd",
        )?;
        node.callback_for_name("on_control").reads_entity(&sub)?;

        node.create_publisher_for_topic::<Control>("/control/command/control_cmd")?;
        node.create_publisher_for_topic::<GearCommand>("/control/command/gear_cmd")?;
        node.create_publisher_for_topic::<TurnIndicatorsCommand>(
            "/control/command/turn_indicators_cmd",
        )?;
        node.create_publisher_for_topic::<HazardLightsCommand>(
            "/control/command/hazard_lights_cmd",
        )?;
        node.create_publisher_for_topic::<VehicleEmergencyStamped>(
            "/control/command/emergency_cmd",
        )?;
        node.create_publisher_for_topic::<GateMode>("/control/gate_mode_cmd")?;
        node.create_publisher_for_topic::<GateMode>("/control/current_gate_mode")?;
        node.create_publisher_for_topic::<IsStopped>("/control/vehicle_cmd_gate/is_stopped")?;
        node.create_publisher_for_topic::<autoware_adapi_v1_msgs_op_mode_alias::OperationModeState>(
            "/control/vehicle_cmd_gate/operation_mode",
        )?;
        node.create_publisher_for_topic::<IsPaused>("/control/vehicle_cmd_gate/is_paused")?;
        node.create_publisher_for_topic::<IsStartRequested>(
            "/control/vehicle_cmd_gate/is_start_requested",
        )?;
        node.create_publisher_for_topic::<IsFilterActivated>(
            "/control/vehicle_cmd_gate/is_filter_activated",
        )?;
        node.create_publisher_for_topic::<BoolStamped>(
            "/control/vehicle_cmd_gate/is_filter_activated/flag",
        )?;
        node.create_publisher_for_topic::<MarkerArray>(
            "/control/vehicle_cmd_gate/is_filter_activated/marker",
        )?;
        node.create_publisher_for_topic::<MarkerArray>(
            "/control/vehicle_cmd_gate/is_filter_activated/marker_raw",
        )?;
        node.create_publisher_for_topic::<PublishedTime>(
            "/control/command/control_cmd/debug/published_time",
        )?;

        node.create_service_server_for_name::<Trigger>(
            "/control/vehicle_cmd_gate/external_emergency_stop",
        )?;
        node.create_service_server_for_name::<Trigger>(
            "/control/vehicle_cmd_gate/clear_external_emergency_stop",
        )?;
        node.create_service_server_for_name::<ConfigLogger>(
            "/control/vehicle_cmd_gate/config_logger",
        )?;
        node.create_service_server_for_name::<SetStop>("/control/vehicle_cmd_gate/set_stop")?;

        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(33))?;
        Ok(())
    }
}

// The gate op-mode publisher reuses adapi's OperationModeState type.
mod autoware_adapi_v1_msgs_op_mode_alias {
    pub use autoware_sentinel_core::OpModeStateMsg as OperationModeState;
}

impl ExecutableNode for VehicleCmdGateNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        let trigger_ok = |ctx: &mut CallbackCtx<'_>| {
            let _ = ctx.reply::<TriggerResponse, 256>(&TriggerResponse {
                success: true,
                message: Default::default(),
            });
        };
        match callback.as_str() {
            "on_control" => {
                if let Ok(msg) = ctx.message::<Control>() {
                    let now = platform_now_ms();
                    with_island(|island| island.on_external_control(&msg, now));
                }
            }
            "/control/vehicle_cmd_gate/external_emergency_stop" => {
                with_island(|island| island.set_external_emergency(true));
                trigger_ok(ctx);
            }
            "/control/vehicle_cmd_gate/clear_external_emergency_stop" => {
                with_island(|island| island.set_external_emergency(false));
                trigger_ok(ctx);
            }
            "/control/vehicle_cmd_gate/config_logger" => {
                let _ =
                    ctx.reply::<ConfigLoggerResponse, 128>(&ConfigLoggerResponse { success: true });
            }
            "/control/vehicle_cmd_gate/set_stop" => {
                let _ = ctx.reply::<SetStopResponse, 256>(&SetStopResponse {
                    status: Default::default(),
                });
            }
            "on_tick" => {
                // THE chain pass — every other node reads this snapshot.
                let now = platform_now_ms();
                let out = with_island(|island| island.chain_tick(now));

                let _ = ctx.publish_to_topic::<Control, 256>(
                    "/control/command/control_cmd",
                    &out.gate_control,
                );
                let _ = ctx.publish_to_topic::<GearCommand, 128>(
                    "/control/command/gear_cmd",
                    &out.gate_gear,
                );
                let _ = ctx.publish_to_topic::<TurnIndicatorsCommand, 128>(
                    "/control/command/turn_indicators_cmd",
                    &out.gate_turn,
                );
                let _ = ctx.publish_to_topic::<HazardLightsCommand, 128>(
                    "/control/command/hazard_lights_cmd",
                    &out.mrm_hazard,
                );
                let _ = ctx.publish_to_topic::<VehicleEmergencyStamped, 128>(
                    "/control/command/emergency_cmd",
                    &VehicleEmergencyStamped {
                        stamp: Default::default(),
                        emergency: out.is_emergency,
                    },
                );
                let gate_mode = GateMode {
                    data: GATE_MODE_AUTO,
                };
                let _ = ctx.publish_to_topic::<GateMode, 64>("/control/gate_mode_cmd", &gate_mode);
                let _ =
                    ctx.publish_to_topic::<GateMode, 64>("/control/current_gate_mode", &gate_mode);
                let _ = ctx.publish_to_topic::<IsStopped, 128>(
                    "/control/vehicle_cmd_gate/is_stopped",
                    &IsStopped {
                        stamp: Default::default(),
                        data: out.is_stopped,
                        requested_sources: Default::default(),
                    },
                );
                let _ = ctx.publish_to_topic::<autoware_adapi_v1_msgs_op_mode_alias::OperationModeState, 128>(
                    "/control/vehicle_cmd_gate/operation_mode",
                    &out.op_mode_state,
                );
                let _ = ctx.publish_to_topic::<IsPaused, 128>(
                    "/control/vehicle_cmd_gate/is_paused",
                    &IsPaused {
                        stamp: Default::default(),
                        data: false,
                    },
                );
                let _ = ctx.publish_to_topic::<IsStartRequested, 128>(
                    "/control/vehicle_cmd_gate/is_start_requested",
                    &IsStartRequested {
                        stamp: Default::default(),
                        data: false,
                    },
                );
                let _ = ctx.publish_to_topic::<IsFilterActivated, 128>(
                    "/control/vehicle_cmd_gate/is_filter_activated",
                    &IsFilterActivated {
                        stamp: Default::default(),
                        is_activated: false,
                        is_activated_on_steering: false,
                        is_activated_on_steering_rate: false,
                        is_activated_on_speed: false,
                        is_activated_on_acceleration: false,
                        is_activated_on_jerk: false,
                    },
                );
                let _ = ctx.publish_to_topic::<BoolStamped, 128>(
                    "/control/vehicle_cmd_gate/is_filter_activated/flag",
                    &BoolStamped {
                        stamp: Default::default(),
                        data: false,
                    },
                );
                let empty = MarkerArray {
                    markers: Default::default(),
                };
                let _ = ctx.publish_to_topic::<MarkerArray, 128>(
                    "/control/vehicle_cmd_gate/is_filter_activated/marker",
                    &empty,
                );
                let _ = ctx.publish_to_topic::<MarkerArray, 128>(
                    "/control/vehicle_cmd_gate/is_filter_activated/marker_raw",
                    &empty,
                );
                let _ = ctx.publish_to_topic::<PublishedTime, 128>(
                    "/control/command/control_cmd/debug/published_time",
                    &PublishedTime {
                        header: Default::default(),
                        published_stamp: Default::default(),
                    },
                );
            }
            _ => {}
        }
    }
}

nros::node!(VehicleCmdGateNode);
