//! Sentinel `/control/control_validator` — validation status publishers.
//! Phase 14.4b declarative wrapper over the shared safety-chain crossbar.

#![no_std]

use autoware_control_validator_msgs::msg::ControlValidatorStatus;
use autoware_sentinel_core::{ensure_island_default, with_island};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use visualization_msgs::msg::MarkerArray;

pub struct ControlValidatorNode;

impl Node for ControlValidatorNode {
    const NAME: &'static str = "control_validator";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node =
            ctx.create_node(NodeOptions::new("control_validator").namespace("/control"))?;
        node.create_publisher_for_topic::<MarkerArray>("/control/control_validator/debug/marker")?;
        node.create_publisher_for_topic::<MarkerArray>(
            "/control/control_validator/output/markers",
        )?;
        node.create_publisher_for_topic::<ControlValidatorStatus>(
            "/control/control_validator/validation_status",
        )?;
        node.create_publisher_for_topic::<MarkerArray>("/control/control_validator/virtual_wall")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(33))?;
        Ok(())
    }
}

impl ExecutableNode for ControlValidatorNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            let out = with_island(|island| island.last_outputs());
            let empty = MarkerArray {
                markers: Default::default(),
            };
            let _ = ctx.publish_to_topic::<MarkerArray, 128>(
                "/control/control_validator/debug/marker",
                &empty,
            );
            let _ = ctx.publish_to_topic::<MarkerArray, 128>(
                "/control/control_validator/output/markers",
                &empty,
            );
            let _ = ctx.publish_to_topic::<MarkerArray, 128>(
                "/control/control_validator/virtual_wall",
                &empty,
            );
            let cv = out.cv;
            let _ = ctx.publish_to_topic::<ControlValidatorStatus, 512>(
                "/control/control_validator/validation_status",
                &ControlValidatorStatus {
                    stamp: Default::default(),
                    is_valid_max_distance_deviation: true,
                    is_valid_acc: cv.is_valid_acc,
                    is_rolling_back: cv.is_rolling_back,
                    is_over_velocity: cv.is_over_velocity,
                    is_valid_lateral_jerk: cv.is_valid_lateral_jerk,
                    has_overrun_stop_point: false,
                    will_overrun_stop_point: false,
                    is_valid_latency: true,
                    is_valid_yaw: true,
                    is_warn_yaw: false,
                    max_distance_deviation: 0.0,
                    steering_rate: cv.steering_rate,
                    lateral_jerk: cv.lateral_jerk,
                    desired_acc: cv.desired_acc,
                    measured_acc: cv.measured_acc,
                    target_vel: cv.target_vel,
                    vehicle_vel: cv.vehicle_vel,
                    dist_to_stop: 0.0,
                    pred_dist_to_stop: 0.0,
                    nearest_trajectory_vel: 0.0,
                    latency: 0.0,
                    yaw_deviation: 0.0,
                    invalid_count: cv.invalid_count as i64,
                },
            );
        }
    }
}

nros::node!(ControlValidatorNode);
