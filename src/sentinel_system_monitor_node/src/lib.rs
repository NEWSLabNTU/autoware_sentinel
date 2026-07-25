//! Sentinel `/system/system_monitor` — monitoring/diag stub publishers.
//! Phase 14.4b declarative wrapper (Phase 12 parity surface; Linux-only in
//! practice — the launch rosters exclude it on the MCU targets).

#![no_std]

use autoware_adapi_v1_msgs::msg::{DiagGraphStatus, DiagGraphStruct};
use autoware_sentinel_core::ensure_island_default;
use autoware_system_msgs::msg::HazardStatusStamped;
use diagnostic_msgs::msg::DiagnosticArray;
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use tier4_system_msgs::msg::{
    CommandModeAvailability, ModeChangeAvailable, OperationModeAvailability,
};

// Large heapless-Vec message defaults live in statics — inline defaults
// would blow the callback stack frame (same pattern as the core statics).
static DIAG_STATUS_DEFAULT: DiagGraphStatus = DiagGraphStatus {
    stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
    id: nros::heapless::String::new(),
    nodes: nros::heapless::Vec::new(),
    diags: nros::heapless::Vec::new(),
};
static DIAG_ARRAY_DEFAULT: DiagnosticArray = DiagnosticArray {
    header: std_msgs::msg::Header {
        stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
        frame_id: nros::heapless::String::new(),
    },
    status: nros::heapless::Vec::new(),
};
static CMD_MODE_DEFAULT: CommandModeAvailability = CommandModeAvailability {
    stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
    items: nros::heapless::Vec::new(),
};
static HAZARD_DEFAULT: HazardStatusStamped = HazardStatusStamped {
    stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
    status: autoware_system_msgs::msg::HazardStatus {
        level: 0,
        emergency: false,
        emergency_holding: false,
        diag_no_fault: nros::heapless::Vec::new(),
        diag_safe_fault: nros::heapless::Vec::new(),
        diag_latent_fault: nros::heapless::Vec::new(),
        diag_single_point_fault: nros::heapless::Vec::new(),
    },
};
static DIAG_STRUCT_DEFAULT: DiagGraphStruct = DiagGraphStruct {
    stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
    id: nros::heapless::String::new(),
    nodes: nros::heapless::Vec::new(),
    diags: nros::heapless::Vec::new(),
    links: nros::heapless::Vec::new(),
};

const CSM_TOPICS: [&str; 8] = [
    "/system/component_state_monitor/component/launch/control",
    "/system/component_state_monitor/component/launch/localization",
    "/system/component_state_monitor/component/launch/map",
    "/system/component_state_monitor/component/launch/perception",
    "/system/component_state_monitor/component/launch/planning",
    "/system/component_state_monitor/component/launch/sensing",
    "/system/component_state_monitor/component/launch/system",
    "/system/component_state_monitor/component/launch/vehicle",
];

pub struct SystemMonitorNode;

impl Node for SystemMonitorNode {
    const NAME: &'static str = "system_monitor";

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        ensure_island_default();
        let mut node = ctx.create_node(NodeOptions::new("system_monitor").namespace("/system"))?;
        node.create_publisher_for_topic::<DiagGraphStatus>("/api/system/diagnostics/status")?;
        node.create_publisher_for_topic::<DiagGraphStruct>("/api/system/diagnostics/struct")?;
        node.create_publisher_for_topic::<DiagnosticArray>("/diagnostics_graph/unknowns")?;
        node.create_publisher_for_topic::<CommandModeAvailability>(
            "/system/command_mode/availability",
        )?;
        for t in CSM_TOPICS {
            node.create_publisher_for_topic::<ModeChangeAvailable>(t)?;
        }
        node.create_publisher_for_topic::<HazardStatusStamped>("/system/emergency/hazard_status")?;
        node.create_publisher_for_topic::<OperationModeAvailability>(
            "/system/operation_mode/availability",
        )?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(33))?;
        // Last node in the model order — this line is the entry's readiness
        // marker (the test fixture waits for it; the macro entry itself
        // prints nothing until shutdown).
        nros_log::nros_info!(&nros_log::DEFAULT_LOGGER, "sentinel graph registered");
        Ok(())
    }
}

impl ExecutableNode for SystemMonitorNode {
    type State = ();
    fn init() -> Self::State {}

    fn on_callback(_s: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            let _ = ctx.publish_to_topic::<DiagGraphStatus, 512>(
                "/api/system/diagnostics/status",
                &DIAG_STATUS_DEFAULT,
            );
            let _ = ctx.publish_to_topic::<DiagGraphStruct, 512>(
                "/api/system/diagnostics/struct",
                &DIAG_STRUCT_DEFAULT,
            );
            let _ = ctx.publish_to_topic::<DiagnosticArray, 256>(
                "/diagnostics_graph/unknowns",
                &DIAG_ARRAY_DEFAULT,
            );
            let _ = ctx.publish_to_topic::<CommandModeAvailability, 256>(
                "/system/command_mode/availability",
                &CMD_MODE_DEFAULT,
            );
            let avail = ModeChangeAvailable {
                stamp: Default::default(),
                available: true,
            };
            for t in CSM_TOPICS {
                let _ = ctx.publish_to_topic::<ModeChangeAvailable, 128>(t, &avail);
            }
            let _ = ctx.publish_to_topic::<HazardStatusStamped, 512>(
                "/system/emergency/hazard_status",
                &HAZARD_DEFAULT,
            );
            let _ = ctx.publish_to_topic::<OperationModeAvailability, 128>(
                "/system/operation_mode/availability",
                &OperationModeAvailability {
                    stamp: Default::default(),
                    stop: true,
                    autonomous: true,
                    local: true,
                    remote: true,
                    emergency_stop: true,
                    comfortable_stop: true,
                    pull_over: true,
                },
            );
        }
    }
}

nros::node!(SystemMonitorNode);
