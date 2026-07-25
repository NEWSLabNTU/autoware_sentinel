//! Sentinel `/control/autoware_operation_mode_transition_manager` —
//! operation-mode services + state publishers.
//! Phase 14.4b declarative wrapper over the shared safety-chain crossbar.

#![no_std]

use autoware_adapi_v1_msgs::msg::{OperationModeState, ResponseStatus};
use autoware_adapi_v1_msgs::srv::{ChangeOperationMode, ChangeOperationModeResponse};
use autoware_internal_msgs::msg::PublishedTime;
use autoware_operation_mode_transition_manager_msgs::msg::OperationModeTransitionManagerDebug;
use autoware_sentinel_core::{ensure_island_default, with_island};
use autoware_system_msgs::srv::{ChangeAutowareControl, ChangeAutowareControlResponse};
use autoware_vehicle_msgs::srv::{ControlModeCommand, ControlModeCommandResponse};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use tier4_system_msgs::msg::ModeChangeAvailable;

fn ok_status() -> ResponseStatus {
    ResponseStatus {
        success: true,
        code: 0,
        message: Default::default(),
    }
}
fn fail_status() -> ResponseStatus {
    ResponseStatus {
        success: false,
        code: 1,
        message: Default::default(),
    }
}

pub struct OperationModeNode;

impl Node for OperationModeNode {
    const NAME: &'static str = "autoware_operation_mode_transition_manager";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node = ctx.create_node(
            NodeOptions::new("autoware_operation_mode_transition_manager").namespace("/control"),
        )?;
        node.create_publisher_for_topic::<OperationModeState>("/api/operation_mode/state")?;
        node.create_publisher_for_topic::<OperationModeState>("/system/operation_mode/state")?;
        node.create_publisher_for_topic::<OperationModeTransitionManagerDebug>(
            "/control/autoware_operation_mode_transition_manager/debug_info",
        )?;
        node.create_publisher_for_topic::<ModeChangeAvailable>("/control/is_autonomous_available")?;
        node.create_service_server_for_name::<ChangeOperationMode>(
            "/api/operation_mode/change_to_autonomous",
        )?;
        node.create_service_server_for_name::<ChangeOperationMode>(
            "/api/operation_mode/change_to_stop",
        )?;
        node.create_service_server_for_name::<ChangeOperationMode>(
            "/api/operation_mode/change_to_local",
        )?;
        node.create_service_server_for_name::<ChangeOperationMode>(
            "/api/operation_mode/change_to_remote",
        )?;
        node.create_service_server_for_name::<ChangeOperationMode>(
            "/api/operation_mode/enable_autoware_control",
        )?;
        node.create_service_server_for_name::<ChangeOperationMode>(
            "/api/operation_mode/disable_autoware_control",
        )?;
        node.create_service_server_for_name::<ControlModeCommand>("/control/control_mode_request")?;
        node.create_service_server_for_name::<ChangeAutowareControl>(
            "/system/operation_mode/change_autoware_control",
        )?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(33))?;
        Ok(())
    }
}

impl ExecutableNode for OperationModeNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        let reply_mode = |ctx: &mut CallbackCtx<'_>, status: ResponseStatus| {
            let _ = ctx
                .reply::<ChangeOperationModeResponse, 256>(&ChangeOperationModeResponse { status });
        };
        match callback.as_str() {
            "/api/operation_mode/change_to_autonomous" => {
                with_island(|island| island.set_engaged(true));
                reply_mode(ctx, ok_status());
            }
            "/api/operation_mode/change_to_stop" => {
                with_island(|island| island.set_engaged(false));
                reply_mode(ctx, ok_status());
            }
            "/api/operation_mode/change_to_local"
            | "/api/operation_mode/change_to_remote"
            | "/api/operation_mode/disable_autoware_control" => reply_mode(ctx, fail_status()),
            "/api/operation_mode/enable_autoware_control" => reply_mode(ctx, ok_status()),
            "/control/control_mode_request" => {
                let _ = ctx.reply::<ControlModeCommandResponse, 128>(&ControlModeCommandResponse {
                    success: true,
                });
            }
            "/system/operation_mode/change_autoware_control" => {
                let _ = ctx.reply::<ChangeAutowareControlResponse, 256>(
                    &ChangeAutowareControlResponse {
                        status: Default::default(),
                    },
                );
            }
            "on_tick" => {
                let out = with_island(|island| island.last_outputs());
                let _ = ctx.publish_to_topic::<OperationModeState, 128>(
                    "/api/operation_mode/state",
                    &out.op_mode_state,
                );
                let _ = ctx.publish_to_topic::<OperationModeState, 128>(
                    "/system/operation_mode/state",
                    &out.op_mode_state,
                );
                let _ = ctx.publish_to_topic::<ModeChangeAvailable, 128>(
                    "/control/is_autonomous_available",
                    &ModeChangeAvailable {
                        stamp: Default::default(),
                        available: out.autonomous_engaged,
                    },
                );
                let _ = ctx.publish_to_topic::<OperationModeTransitionManagerDebug, 512>(
                    "/control/autoware_operation_mode_transition_manager/debug_info",
                    &OperationModeTransitionManagerDebug {
                        stamp: Default::default(),
                        status: Default::default(),
                        in_autoware_control: true,
                        in_transition: false,
                        is_all_ok: true,
                        engage_allowed_for_stopped_vehicle: true,
                        trajectory_available_ok: true,
                        lateral_deviation_ok: true,
                        yaw_deviation_ok: true,
                        speed_upper_deviation_ok: true,
                        speed_lower_deviation_ok: true,
                        stop_ok: true,
                        large_acceleration_ok: true,
                        large_lateral_acceleration_ok: true,
                        large_lateral_acceleration_diff_ok: true,
                        current_speed: out.current_velocity,
                        target_control_speed: 0.0,
                        target_planning_speed: 0.0,
                        target_control_acceleration: 0.0,
                        lateral_acceleration: 0.0,
                        lateral_acceleration_deviation: 0.0,
                        lateral_deviation: 0.0,
                        yaw_deviation: 0.0,
                        speed_deviation: 0.0,
                    },
                );
            }
            _ => {}
        }
    }
}

nros::node!(OperationModeNode);
