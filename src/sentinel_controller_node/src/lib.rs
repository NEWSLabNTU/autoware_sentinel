//! Sentinel `/control/trajectory_follower/controller_node_exe` — bundled
//! MPC+PID controller inputs (Linux dev profile).
//! Phase 14.4b declarative wrapper over the shared safety-chain crossbar;
//! the chain's controller section (gated by the core `controller-node`
//! feature) consumes these inputs inside the gate node's tick.

#![no_std]

use autoware_planning_msgs::msg::Trajectory;
use autoware_sentinel_core::{ensure_island_default, with_island};
use autoware_vehicle_msgs::msg::SteeringReport;
use geometry_msgs::msg::AccelWithCovarianceStamped;
use nav_msgs::msg::Odometry;
use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};

pub struct ControllerNode;

impl Node for ControllerNode {
    const NAME: &'static str = "controller_node_exe";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node = ctx.create_node(
            NodeOptions::new("controller_node_exe").namespace("/control/trajectory_follower"),
        )?;
        let s1 = node.create_subscription_for_callback_name::<Trajectory>(
            "on_trajectory",
            "/planning/scenario_planning/trajectory",
        )?;
        node.callback_for_name("on_trajectory").reads_entity(&s1)?;
        let s2 = node.create_subscription_for_callback_name::<Odometry>(
            "on_odometry",
            "/localization/kinematic_state",
        )?;
        node.callback_for_name("on_odometry").reads_entity(&s2)?;
        let s3 = node.create_subscription_for_callback_name::<SteeringReport>(
            "on_steering",
            "/vehicle/status/steering_status",
        )?;
        node.callback_for_name("on_steering").reads_entity(&s3)?;
        let s4 = node.create_subscription_for_callback_name::<AccelWithCovarianceStamped>(
            "on_accel",
            "/localization/acceleration",
        )?;
        node.callback_for_name("on_accel").reads_entity(&s4)?;
        Ok(())
    }
}

impl ExecutableNode for ControllerNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "on_trajectory" => {
                if let Ok(msg) = ctx.message::<Trajectory>() {
                    with_island(|island| island.on_trajectory(&msg));
                }
            }
            "on_odometry" => {
                if let Ok(msg) = ctx.message::<Odometry>() {
                    with_island(|island| island.on_odometry(&msg));
                }
            }
            "on_steering" => {
                if let Ok(msg) = ctx.message::<SteeringReport>() {
                    with_island(|island| island.on_steering(&msg));
                }
            }
            "on_accel" => {
                if let Ok(msg) = ctx.message::<AccelWithCovarianceStamped>() {
                    with_island(|island| island.on_acceleration(&msg));
                }
            }
            _ => {}
        }
    }
}

nros::node!(ControllerNode);
