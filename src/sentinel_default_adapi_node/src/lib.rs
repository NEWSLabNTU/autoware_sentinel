//! Sentinel `/adapi/default_adapi` — external API surface (engage/emergency
//! set+get, interface version, shutdown, diagnostics stubs).
//! Phase 14.4b declarative wrapper over the shared safety-chain crossbar.

#![no_std]

use autoware_adapi_v1_msgs::srv::{ResetDiagGraph, ResetDiagGraphResponse};
use autoware_adapi_version_msgs::srv::{InterfaceVersion, InterfaceVersionResponse};
use autoware_sentinel_core::{ensure_island_default, with_island};
use autoware_system_msgs::msg::AutowareState;
use autoware_vehicle_msgs::msg::Engage;
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_srvs::srv::{SetBool, SetBoolResponse, Trigger, TriggerResponse};
use tier4_external_api_msgs::msg::Emergency;
use tier4_external_api_msgs::srv::{
    Engage as EngageSrv, EngageResponse, SetEmergency, SetEmergencyResponse,
};
use tier4_system_msgs::srv::{
    ResetDiagGraph as ResetDiagGraphTier4, ResetDiagGraphResponse as ResetDiagGraphTier4Response,
};

const TIER4_RESPONSE_SUCCESS: u32 = 1;
const AUTOWARE_STATE_DRIVING: u8 = 5;

fn t4_ok() -> tier4_external_api_msgs::msg::ResponseStatus {
    tier4_external_api_msgs::msg::ResponseStatus {
        code: TIER4_RESPONSE_SUCCESS,
        message: Default::default(),
    }
}

pub struct DefaultAdapiNode;

impl Node for DefaultAdapiNode {
    const NAME: &'static str = "default_adapi";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node = ctx.create_node(NodeOptions::new("default_adapi").namespace("/adapi"))?;
        node.create_publisher_for_topic::<Engage>("/api/autoware/get/engage")?;
        node.create_publisher_for_topic::<Engage>("/autoware/engage")?;
        node.create_publisher_for_topic::<AutowareState>("/autoware/state")?;
        node.create_publisher_for_topic::<Emergency>("/api/autoware/get/emergency")?;
        node.create_service_server_for_name::<EngageSrv>("/api/autoware/set/engage")?;
        node.create_service_server_for_name::<SetEmergency>("/api/autoware/set/emergency")?;
        node.create_service_server_for_name::<InterfaceVersion>("/api/interface/version")?;
        node.create_service_server_for_name::<ResetDiagGraph>("/api/system/diagnostics/reset")?;
        node.create_service_server_for_name::<Trigger>("/autoware/shutdown")?;
        node.create_service_server_for_name::<ResetDiagGraphTier4>("/diagnostics_graph/reset")?;
        node.create_service_server_for_name::<SetBool>("/system/aggregator/set_initializing")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(33))?;
        Ok(())
    }
}

impl ExecutableNode for DefaultAdapiNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "/api/autoware/set/engage" => {
                if let Ok(req) = ctx.message::<tier4_external_api_msgs::srv::EngageRequest>() {
                    with_island(|island| island.set_engaged(req.engage));
                }
                let _ = ctx.reply::<EngageResponse, 256>(&EngageResponse { status: t4_ok() });
            }
            "/api/autoware/set/emergency" => {
                if let Ok(req) = ctx.message::<tier4_external_api_msgs::srv::SetEmergencyRequest>()
                {
                    with_island(|island| island.set_external_emergency(req.emergency));
                }
                let _ = ctx
                    .reply::<SetEmergencyResponse, 256>(&SetEmergencyResponse { status: t4_ok() });
            }
            "/api/interface/version" => {
                let _ = ctx.reply::<InterfaceVersionResponse, 128>(&InterfaceVersionResponse {
                    major: 1,
                    minor: 5,
                    patch: 0,
                });
            }
            "/api/system/diagnostics/reset" => {
                let _ = ctx.reply::<ResetDiagGraphResponse, 256>(&ResetDiagGraphResponse {
                    status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                        success: true,
                        code: 0,
                        message: Default::default(),
                    },
                });
            }
            "/autoware/shutdown" => {
                let _ = ctx.reply::<TriggerResponse, 256>(&TriggerResponse {
                    success: true,
                    message: Default::default(),
                });
            }
            "/diagnostics_graph/reset" => {
                let _ =
                    ctx.reply::<ResetDiagGraphTier4Response, 256>(&ResetDiagGraphTier4Response {
                        status: Default::default(),
                    });
            }
            "/system/aggregator/set_initializing" => {
                let _ = ctx.reply::<SetBoolResponse, 256>(&SetBoolResponse {
                    success: true,
                    message: Default::default(),
                });
            }
            "on_tick" => {
                let out = with_island(|island| island.last_outputs());
                let engaged = Engage {
                    stamp: Default::default(),
                    engage: true,
                };
                let _ = ctx.publish_to_topic::<Engage, 128>("/api/autoware/get/engage", &engaged);
                let _ = ctx.publish_to_topic::<Engage, 128>("/autoware/engage", &engaged);
                let _ = ctx.publish_to_topic::<AutowareState, 128>(
                    "/autoware/state",
                    &AutowareState {
                        stamp: Default::default(),
                        state: AUTOWARE_STATE_DRIVING,
                    },
                );
                let _ = ctx.publish_to_topic::<Emergency, 128>(
                    "/api/autoware/get/emergency",
                    &Emergency {
                        stamp: Default::default(),
                        emergency: out.is_emergency,
                    },
                );
            }
            _ => {}
        }
    }
}

nros::node!(DefaultAdapiNode);
