// Copyright 2025 Autoware Sentinel contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

//! Autoware Sentinel — Shared Core
//!
//! Platform-agnostic safety-island wiring. Hosts the [`SafetyIsland`] state
//! and the [`wire_executor`] entry point used by every per-platform binary
//! (Linux, Zephyr, FreeRTOS, NuttX).
//!
//! # Public API
//!
//! ```ignore
//! use autoware_sentinel_core as core;
//! use nros::prelude::*;
//!
//! fn now_ms() -> u64 { /* platform clock */ 0 }
//!
//! let mut executor = Executor::open(&config)?;
//! executor.register_parameter_services()?;
//! core::params::declare_parameters(executor.params_mut().unwrap());
//! let p = core::params::read_params(executor.params().unwrap());
//! core::init_island(p);
//! core::wire_executor(&mut executor, now_ms)?;
//! executor.spin_blocking(SpinOptions::default())?;
//! ```
//!
//! # Feature gates
//!
//! - `controller-node` — bundles the trajectory follower (MPC + PID +
//!   interpolation + vehicle-info-utils). Default off. Enabled only by
//!   the Linux dev binary; safety MCUs rely on `/control/command/control_cmd`
//!   from the main compute and trigger MRM on staleness.

#![no_std]

extern crate alloc;

pub mod params;

use core::cell::RefCell;

use log::info;
use nros::prelude::*;

// Algorithm crates
use autoware_control_validator::ControlValidator;
use autoware_heartbeat_watchdog::HeartbeatWatchdog;
use autoware_mrm_comfortable_stop_operator::ComfortableStopOperator;
use autoware_mrm_emergency_stop_operator::EmergencyStopOperator;
use autoware_mrm_handler::MrmHandler;
use autoware_operation_mode_transition_manager::OperationModeTransitionManager;
use autoware_shift_decider::ShiftDecider;
use autoware_stop_filter::StopFilter;
use autoware_twist2accel::Twist2Accel;
use autoware_vehicle_cmd_gate::VehicleCmdGate;
use autoware_vehicle_cmd_gate::gate::SourceCommands;
use autoware_vehicle_velocity_converter::VehicleVelocityConverter;

#[cfg(feature = "controller-node")]
use autoware_trajectory_follower_base::{InputData, TrajectoryPoint};
#[cfg(feature = "controller-node")]
use autoware_trajectory_follower_node::ControllerNode;

// Message types — always-on imports (used by core publishers, the MRM/cmd-gate
// pipeline, or SafetyIsland fields that exist independently of feature gates).
use autoware_adapi_v1_msgs::msg::{Heartbeat, MrmState, OperationModeState};
use autoware_adapi_v1_msgs::srv::{ChangeOperationMode, ChangeOperationModeResponse};
use autoware_control_msgs::msg::Control;
use autoware_system_msgs::msg::AutowareState;
use autoware_vehicle_msgs::msg::{
    GearCommand, GearReport, HazardLightsCommand, TurnIndicatorsCommand, VelocityReport,
};
use geometry_msgs::msg::{Accel, Twist};
use tier4_system_msgs::msg::OperationModeAvailability;

// Feature-gated message types.
#[cfg(feature = "comp-mrm")]
use tier4_system_msgs::msg::{EmergencyHoldingState, MrmBehaviorStatus};
#[cfg(feature = "comp-mrm")]
use tier4_system_msgs::srv::OperateMrm;
#[cfg(feature = "comp-engagement")]
use autoware_vehicle_msgs::msg::Engage;
#[cfg(feature = "comp-engagement")]
use tier4_external_api_msgs::msg::Emergency;
#[cfg(feature = "comp-engagement")]
use tier4_external_api_msgs::srv::{
    Engage as EngageSrv, EngageResponse, SetEmergency, SetEmergencyResponse,
};
#[cfg(feature = "comp-cmd-gate-extra")]
use tier4_control_msgs::msg::{GateMode, IsPaused, IsStartRequested, IsStopped};
#[cfg(feature = "comp-cmd-gate-extra")]
use tier4_control_msgs::srv::SetStop;
#[cfg(feature = "comp-cmd-gate-extra")]
use tier4_vehicle_msgs::msg::VehicleEmergencyStamped;
#[cfg(feature = "comp-cmd-gate-extra")]
use autoware_internal_debug_msgs::msg::BoolStamped;
#[cfg(feature = "comp-cmd-gate-extra")]
use autoware_vehicle_cmd_gate_msgs::msg::IsFilterActivated;
#[cfg(feature = "comp-cmd-gate-extra")]
use logging_demo::srv::ConfigLogger;
#[cfg(any(
    feature = "comp-cmd-gate-extra",
    feature = "comp-validator",
    feature = "monitoring-topics"
))]
use visualization_msgs::msg::MarkerArray;
#[cfg(any(
    feature = "comp-cmd-gate-extra",
    feature = "comp-stubs"
))]
use std_srvs::srv::{Trigger, TriggerResponse};
#[cfg(feature = "comp-validator")]
use autoware_control_validator_msgs::msg::ControlValidatorStatus;
#[cfg(feature = "comp-op-mode-mgr")]
use autoware_internal_msgs::msg::PublishedTime;
#[cfg(feature = "comp-op-mode-mgr")]
use autoware_operation_mode_transition_manager_msgs::msg::OperationModeTransitionManagerDebug;
#[cfg(feature = "comp-op-mode-mgr")]
use autoware_vehicle_msgs::srv::{ControlModeCommand, ControlModeCommandResponse};
#[cfg(any(feature = "comp-op-mode-mgr", feature = "monitoring-topics"))]
use tier4_system_msgs::msg::ModeChangeAvailable;

// Phase 12 gap-closure imports.
#[cfg(feature = "comp-stubs")]
use autoware_adapi_v1_msgs::srv::{ResetDiagGraph, ResetDiagGraphResponse};
#[cfg(feature = "comp-stubs")]
use autoware_adapi_version_msgs::srv::{InterfaceVersion, InterfaceVersionResponse};
#[cfg(feature = "comp-stubs")]
use autoware_system_msgs::srv::ChangeAutowareControl;
#[cfg(feature = "comp-stubs")]
use std_srvs::srv::SetBool;
#[cfg(feature = "comp-stubs")]
use tier4_system_msgs::srv::ResetDiagGraph as ResetDiagGraphTier4;

// Monitoring-topic imports.
#[cfg(feature = "monitoring-topics")]
use autoware_adapi_v1_msgs::msg::{DiagGraphStatus, DiagGraphStruct};
#[cfg(feature = "monitoring-topics")]
use autoware_system_msgs::msg::HazardStatusStamped;
#[cfg(feature = "monitoring-topics")]
use diagnostic_msgs::msg::DiagnosticArray;
#[cfg(feature = "monitoring-topics")]
use tier4_system_msgs::msg::CommandModeAvailability;

// Controller-node-only message types
#[cfg(feature = "controller-node")]
use autoware_planning_msgs::msg::Trajectory;
#[cfg(feature = "controller-node")]
use autoware_vehicle_msgs::msg::SteeringReport;
#[cfg(feature = "controller-node")]
use geometry_msgs::msg::AccelWithCovarianceStamped;
#[cfg(feature = "controller-node")]
use nav_msgs::msg::Odometry;

pub use params::SentinelParams;

// ============================================================================
// Constants
// ============================================================================

/// MRM handler state: OPERATING (emergency response active).
const MRM_STATE_OPERATING: u16 = 2;

/// AutowareState: DRIVING (autonomous engaged).
const AUTOWARE_STATE_DRIVING: u8 = 5;

/// Consecutive invalid validation frames before triggering MRM (~1 s at 30 Hz).
const VALIDATION_FAILURE_THRESHOLD: u32 = 30;

/// Control period (s) for 30 Hz timer.
const DT: f32 = 1.0 / 30.0;

/// External control staleness threshold (ms). If main compute hasn't sent
/// `/control/.../control_cmd` within this window, replace with gentle
/// braking at current speed to avoid runaway during zenoh-pico dropouts.
const EXTERNAL_CONTROL_STALE_MS: u64 = 2000;

/// OperationModeState mode constants.
const OP_MODE_STOP: u8 = 1;
const OP_MODE_AUTONOMOUS: u8 = 2;

/// MrmBehaviorStatus constants.
#[cfg(feature = "comp-mrm")]
const MRM_BEHAVIOR_AVAILABLE: u8 = 1;
#[cfg(feature = "comp-mrm")]
const MRM_BEHAVIOR_OPERATING: u8 = 2;

/// GateMode constant: AUTO.
#[cfg(feature = "comp-cmd-gate-extra")]
const GATE_MODE_AUTO: u8 = 0;

/// tier4_external_api_msgs ResponseStatus: SUCCESS.
#[cfg(feature = "comp-engagement")]
const TIER4_RESPONSE_SUCCESS: u32 = 1;

// ============================================================================
// Static shared state
// ============================================================================

/// `RefCell` wrapper that implements `Sync` for single-threaded contexts.
///
/// # Safety
///
/// The nros executor dispatches all callbacks sequentially in a single thread;
/// no concurrent access is possible.
struct SyncRefCell<T>(RefCell<T>);
unsafe impl<T> Sync for SyncRefCell<T> {}

/// All algorithm instances and shared data for the safety island.
pub struct SafetyIsland {
    // --- Sensing ---
    velocity_converter: VehicleVelocityConverter,
    stop_filter: StopFilter,
    twist2accel: Twist2Accel,
    prev_stamp: Option<(i32, u32)>,
    current_velocity: f64,
    is_stopped: bool,
    twist: Twist,
    accel: Accel,
    accel_covariance: [f64; 36],

    // --- Heartbeat ---
    watchdog: HeartbeatWatchdog,

    // --- MRM chain ---
    mrm_handler: MrmHandler,
    emergency_stop: EmergencyStopOperator,
    comfortable_stop: ComfortableStopOperator,

    // --- Command output ---
    cmd_gate: VehicleCmdGate,
    shift_decider: ShiftDecider,
    auto_control: Control,
    autoware_state: AutowareState,
    gear_report: GearReport,

    // --- Validation ---
    control_validator: ControlValidator,
    op_mode_mgr: OperationModeTransitionManager,

    // --- Trajectory follower (gated) ---
    #[cfg(feature = "controller-node")]
    controller_node: ControllerNode,
    #[cfg(feature = "controller-node")]
    input_data: InputData,
    #[cfg(feature = "controller-node")]
    has_trajectory: bool,
    #[cfg(feature = "controller-node")]
    has_odometry: bool,
    #[cfg(feature = "controller-node")]
    has_steering: bool,

    /// True when external `/control/.../control_cmd` is supplying control.
    has_external_control: bool,
    /// Reception timestamp (ms) of the last external control message.
    last_external_control_ms: u64,
    /// True after `/api/operation_mode/change_to_autonomous` is called.
    autonomous_engaged: bool,
    /// True when external emergency stop is asserted.
    external_emergency_stop: bool,
}

static ISLAND: SyncRefCell<Option<SafetyIsland>> = SyncRefCell(RefCell::new(None));

/// Borrow the safety island for the duration of `f`.
///
/// # Panics
///
/// Panics if [`init_island`] has not been called yet.
#[inline]
fn with_island<R>(f: impl FnOnce(&mut SafetyIsland) -> R) -> R {
    let mut guard = ISLAND.0.borrow_mut();
    f(guard.as_mut().expect("SafetyIsland not initialized"))
}

/// Initialize the static [`SafetyIsland`] from parameters. Must be called
/// before any subscription/timer/service callback can fire.
pub fn init_island(p: SentinelParams) {
    *ISLAND.0.borrow_mut() = Some(SafetyIsland::new(p));
}

impl SafetyIsland {
    fn new(p: SentinelParams) -> Self {
        let mut watchdog = HeartbeatWatchdog::new(autoware_heartbeat_watchdog::Params {
            timeout_ms: p.watchdog_timeout_ms,
        });
        // Pre-seed with boot time so the watchdog doesn't fire before
        // Autoware finishes initialising and starts emitting heartbeats.
        watchdog.on_heartbeat(0);

        Self {
            velocity_converter: VehicleVelocityConverter::new(
                p.velocity_converter_speed_scale,
                p.velocity_converter_stddev_vx,
                p.velocity_converter_stddev_wz,
            ),
            stop_filter: StopFilter::new(p.stop_filter_vx_threshold, p.stop_filter_wz_threshold),
            twist2accel: Twist2Accel::new(autoware_twist2accel::Params {
                accel_lowpass_gain: p.twist2accel_lpf_gain,
            }),
            prev_stamp: None,
            current_velocity: 0.0,
            is_stopped: true,
            twist: Twist::default(),
            accel: Accel::default(),
            accel_covariance: [0.0; 36],

            watchdog,

            mrm_handler: MrmHandler::new(p.mrm_handler),
            emergency_stop: EmergencyStopOperator::new(p.emergency_stop),
            comfortable_stop: ComfortableStopOperator::new(p.comfortable_stop),

            cmd_gate: VehicleCmdGate::new(p.gate),
            shift_decider: ShiftDecider::new(p.shift_decider_park_on_goal),
            auto_control: Control::default(),
            autoware_state: AutowareState {
                state: AUTOWARE_STATE_DRIVING,
                ..Default::default()
            },
            gear_report: GearReport::default(),

            control_validator: ControlValidator::new(p.control_validator),
            op_mode_mgr: OperationModeTransitionManager::new(p.op_mode_mgr),

            #[cfg(feature = "controller-node")]
            controller_node: ControllerNode::new(p.controller_node, p.vehicle_info),
            #[cfg(feature = "controller-node")]
            input_data: InputData::default(),
            #[cfg(feature = "controller-node")]
            has_trajectory: false,
            #[cfg(feature = "controller-node")]
            has_odometry: false,
            #[cfg(feature = "controller-node")]
            has_steering: false,

            has_external_control: false,
            last_external_control_ms: 0,
            autonomous_engaged: false,
            external_emergency_stop: false,
        }
    }

    /// VelocityReport → VehicleVelocityConverter → StopFilter → Twist2Accel.
    fn on_velocity_report(&mut self, msg: &VelocityReport) {
        let twist_cov = self.velocity_converter.convert(msg);
        let twist = &twist_cov.twist.twist;
        let filtered = self.stop_filter.apply(&twist.linear, &twist.angular);

        let stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec);
        let dt = if let Some((ps, pn)) = self.prev_stamp {
            (stamp.0 - ps) as f64 + (stamp.1 as f64 - pn as f64) * 1e-9
        } else {
            0.0
        };
        self.prev_stamp = Some(stamp);

        let filtered_twist = Twist {
            linear: filtered.linear.clone(),
            angular: filtered.angular.clone(),
        };
        let accel_output = self.twist2accel.update(&filtered_twist, dt);

        self.current_velocity = filtered_twist.linear.x;
        self.is_stopped = filtered.was_stopped;
        self.twist = filtered_twist;
        if let Some(output) = accel_output {
            self.accel = output.accel;
            self.accel_covariance = output.covariance;
        }
    }

    #[cfg(feature = "controller-node")]
    fn on_trajectory(&mut self, msg: &Trajectory) {
        let n = msg
            .points
            .len()
            .min(autoware_trajectory_follower_base::MAX_TRAJECTORY_POINTS);
        for i in 0..n {
            let pt = &msg.points[i];
            let q = &pt.pose.orientation;
            let (pitch, yaw) = quaternion_to_pitch_yaw(q.x, q.y, q.z, q.w);
            self.input_data.trajectory[i] = TrajectoryPoint {
                x: pt.pose.position.x,
                y: pt.pose.position.y,
                z: pt.pose.position.z,
                yaw,
                longitudinal_velocity_mps: pt.longitudinal_velocity_mps as f64,
                lateral_velocity_mps: pt.lateral_velocity_mps as f64,
                acceleration_mps2: pt.acceleration_mps2 as f64,
                heading_rate_rps: pt.heading_rate_rps as f64,
                front_wheel_angle_rad: pt.front_wheel_angle_rad as f64,
            };
            if i == 0 {
                self.input_data.current_pose_pitch = pitch;
            }
        }
        self.input_data.trajectory_len = n;
        self.has_trajectory = true;
    }

    #[cfg(feature = "controller-node")]
    fn on_odometry(&mut self, msg: &Odometry) {
        let pos = &msg.pose.pose.position;
        let q = &msg.pose.pose.orientation;
        let (pitch, yaw) = quaternion_to_pitch_yaw(q.x, q.y, q.z, q.w);

        self.input_data.current_pose_x = pos.x;
        self.input_data.current_pose_y = pos.y;
        self.input_data.current_pose_z = pos.z;
        self.input_data.current_pose_yaw = yaw;
        self.input_data.current_pose_pitch = pitch;
        self.input_data.current_velocity = msg.twist.twist.linear.x;
        self.has_odometry = true;
    }

    #[cfg(feature = "controller-node")]
    fn on_steering(&mut self, msg: &SteeringReport) {
        self.input_data.current_steer = msg.steering_tire_angle as f64;
        self.has_steering = true;
    }

    #[cfg(feature = "controller-node")]
    fn on_acceleration(&mut self, msg: &AccelWithCovarianceStamped) {
        self.input_data.current_accel = msg.accel.accel.linear.x;
    }

    #[cfg(feature = "controller-node")]
    fn run_controller(&mut self, current_time_s: f64) {
        if !self.has_trajectory || !self.has_odometry || !self.has_steering {
            return;
        }
        self.input_data.is_autonomous = true;
        self.input_data.is_in_transition = false;

        if let Some(output) = self
            .controller_node
            .update(&self.input_data, current_time_s)
        {
            self.auto_control.lateral.steering_tire_angle =
                output.lateral.steering_tire_angle as f32;
            self.auto_control.lateral.steering_tire_rotation_rate =
                output.lateral.steering_tire_rotation_rate as f32;
            self.auto_control.longitudinal.velocity = output.longitudinal.velocity as f32;
            self.auto_control.longitudinal.acceleration = output.longitudinal.acceleration as f32;
        }
    }
}

/// Extract (pitch, yaw) from quaternion components.
#[cfg(feature = "controller-node")]
fn quaternion_to_pitch_yaw(x: f64, y: f64, z: f64, w: f64) -> (f64, f64) {
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if libm::fabs(sinp) >= 1.0 {
        libm::copysign(core::f64::consts::FRAC_PI_2, sinp)
    } else {
        libm::asin(sinp)
    };
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = libm::atan2(siny_cosp, cosy_cosp);
    (pitch, yaw)
}

// ============================================================================
// Pre-allocated diagnostic message defaults
// ============================================================================

// `DiagGraphStatus` / `DiagGraphStruct` carry large `heapless::Vec`s and would
// blow the closure stack frame if defaulted inline. Lift them to `static`s.
// Each static occupies ≈ 2 MB of `.text`/rodata — only emit them when
// `monitoring-topics` is on (Linux). Cortex-M3 boards default off.
#[cfg(feature = "monitoring-topics")]
static DIAG_STATUS_DEFAULT: DiagGraphStatus = DiagGraphStatus {
    stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
    id: nros::heapless::String::new(),
    nodes: nros::heapless::Vec::new(),
    diags: nros::heapless::Vec::new(),
};
#[cfg(feature = "monitoring-topics")]
static DIAG_STRUCT_DEFAULT: DiagGraphStruct = DiagGraphStruct {
    stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
    id: nros::heapless::String::new(),
    nodes: nros::heapless::Vec::new(),
    diags: nros::heapless::Vec::new(),
    links: nros::heapless::Vec::new(),
};

// ============================================================================
// Public entry point
// ============================================================================

/// Register all publishers, subscriptions, services, parameter services, and
/// the 30 Hz control timer on `executor`. The caller is responsible for
/// driving the spin loop.
///
/// `now_ms` is a monotonic clock in milliseconds; supplied by the platform
/// binary (e.g. `Instant::elapsed` on Linux, `k_uptime_get` on Zephyr).
pub fn wire_executor(executor: &mut Executor, now_ms: fn() -> u64) -> Result<(), NodeError> {
    // --- Publishers ---
    //
    // Phase 13.K1: publishers grouped by Autoware component. Each comp-* feature
    // toggles one group so the declare-storm bug can be bisected. The always-on
    // "core" group covers the mandatory driving topology (mrm_state, hazard,
    // gear, control, turn indicators, op_mode_state). The monitoring-topics
    // feature additionally enables 14 diagnostic publishers (Linux only — too
    // big for the 4 MB Cortex-M3 flash budget).
    let (
        core_pubs,
        _comp_mrm_pubs,
        _comp_cmd_gate_extra_pubs,
        _comp_validator_pubs,
        _comp_op_mode_mgr_pubs,
        _comp_engagement_pubs,
        _monitoring_pubs,
    ) = {
        let mut node = executor.create_node("sentinel")?;

        let core = (
            node.create_publisher::<MrmState>("/system/fail_safe/mrm_state")?,
            node.create_publisher::<HazardLightsCommand>("/control/command/hazard_lights_cmd")?,
            node.create_publisher::<GearCommand>("/control/command/gear_cmd")?,
            node.create_publisher::<Control>("/control/command/control_cmd")?,
            node.create_publisher::<TurnIndicatorsCommand>("/control/command/turn_indicators_cmd")?,
            node.create_publisher::<OperationModeState>("/api/operation_mode/state")?,
        );

        #[cfg(feature = "comp-mrm")]
        let comp_mrm = (
            node.create_publisher::<MrmBehaviorStatus>("/system/mrm/emergency_stop/status")?,
            node.create_publisher::<MrmBehaviorStatus>("/system/mrm/comfortable_stop/status")?,
            node.create_publisher::<MrmBehaviorStatus>("/system/mrm/pull_over_manager/status")?,
            node.create_publisher::<GearCommand>("/system/emergency/gear_cmd")?,
            node.create_publisher::<HazardLightsCommand>("/system/emergency/hazard_lights_cmd")?,
            node.create_publisher::<TurnIndicatorsCommand>(
                "/system/emergency/turn_indicators_cmd",
            )?,
            node.create_publisher::<EmergencyHoldingState>("/system/emergency_holding")?,
        );
        #[cfg(not(feature = "comp-mrm"))]
        let comp_mrm = ();

        #[cfg(feature = "comp-cmd-gate-extra")]
        let comp_cmd_gate_extra = (
            node.create_publisher::<VehicleEmergencyStamped>("/control/command/emergency_cmd")?,
            node.create_publisher::<GateMode>("/control/gate_mode_cmd")?,
            node.create_publisher::<GearCommand>("/control/shift_decider/gear_cmd")?,
            node.create_publisher::<IsStopped>("/control/vehicle_cmd_gate/is_stopped")?,
            node.create_publisher::<OperationModeState>(
                "/control/vehicle_cmd_gate/operation_mode",
            )?,
            node.create_publisher::<OperationModeState>("/system/operation_mode/state")?,
            node.create_publisher::<IsPaused>("/control/vehicle_cmd_gate/is_paused")?,
            node.create_publisher::<IsStartRequested>(
                "/control/vehicle_cmd_gate/is_start_requested",
            )?,
            node.create_publisher::<GateMode>("/control/current_gate_mode")?,
            node.create_publisher::<IsFilterActivated>(
                "/control/vehicle_cmd_gate/is_filter_activated",
            )?,
            node.create_publisher::<BoolStamped>(
                "/control/vehicle_cmd_gate/is_filter_activated/flag",
            )?,
            node.create_publisher::<MarkerArray>(
                "/control/vehicle_cmd_gate/is_filter_activated/marker",
            )?,
            node.create_publisher::<MarkerArray>(
                "/control/vehicle_cmd_gate/is_filter_activated/marker_raw",
            )?,
        );
        #[cfg(not(feature = "comp-cmd-gate-extra"))]
        let comp_cmd_gate_extra = ();

        #[cfg(feature = "comp-validator")]
        let comp_validator = (
            node.create_publisher::<MarkerArray>("/control/control_validator/debug/marker")?,
            node.create_publisher::<MarkerArray>("/control/control_validator/output/markers")?,
            node.create_publisher::<ControlValidatorStatus>(
                "/control/control_validator/validation_status",
            )?,
            node.create_publisher::<MarkerArray>("/control/control_validator/virtual_wall")?,
        );
        #[cfg(not(feature = "comp-validator"))]
        let comp_validator = ();

        #[cfg(feature = "comp-op-mode-mgr")]
        let comp_op_mode_mgr = (
            node.create_publisher::<OperationModeTransitionManagerDebug>(
                "/control/autoware_operation_mode_transition_manager/debug_info",
            )?,
            node.create_publisher::<ModeChangeAvailable>("/control/is_autonomous_available")?,
            node.create_publisher::<PublishedTime>(
                "/control/command/control_cmd/debug/published_time",
            )?,
        );
        #[cfg(not(feature = "comp-op-mode-mgr"))]
        let comp_op_mode_mgr = ();

        #[cfg(feature = "comp-engagement")]
        let comp_engagement = (
            node.create_publisher::<Engage>("/api/autoware/get/engage")?,
            node.create_publisher::<Engage>("/autoware/engage")?,
            node.create_publisher::<AutowareState>("/autoware/state")?,
            node.create_publisher::<Emergency>("/api/autoware/get/emergency")?,
        );
        #[cfg(not(feature = "comp-engagement"))]
        let comp_engagement = ();

        #[cfg(feature = "monitoring-topics")]
        let monitoring = (
            node.create_publisher::<DiagGraphStatus>("/api/system/diagnostics/status")?,
            node.create_publisher::<DiagGraphStruct>("/api/system/diagnostics/struct")?,
            node.create_publisher::<DiagnosticArray>("/diagnostics_graph/unknowns")?,
            node.create_publisher::<CommandModeAvailability>("/system/command_mode/availability")?,
            node.create_publisher::<ModeChangeAvailable>(
                "/system/component_state_monitor/component/launch/control",
            )?,
            node.create_publisher::<ModeChangeAvailable>(
                "/system/component_state_monitor/component/launch/localization",
            )?,
            node.create_publisher::<ModeChangeAvailable>(
                "/system/component_state_monitor/component/launch/map",
            )?,
            node.create_publisher::<ModeChangeAvailable>(
                "/system/component_state_monitor/component/launch/perception",
            )?,
            node.create_publisher::<ModeChangeAvailable>(
                "/system/component_state_monitor/component/launch/planning",
            )?,
            node.create_publisher::<ModeChangeAvailable>(
                "/system/component_state_monitor/component/launch/sensing",
            )?,
            node.create_publisher::<ModeChangeAvailable>(
                "/system/component_state_monitor/component/launch/system",
            )?,
            node.create_publisher::<ModeChangeAvailable>(
                "/system/component_state_monitor/component/launch/vehicle",
            )?,
            node.create_publisher::<HazardStatusStamped>("/system/emergency/hazard_status")?,
            node.create_publisher::<OperationModeAvailability>(
                "/system/operation_mode/availability",
            )?,
        );
        #[cfg(not(feature = "monitoring-topics"))]
        let monitoring = ();

        (
            core,
            comp_mrm,
            comp_cmd_gate_extra,
            comp_validator,
            comp_op_mode_mgr,
            comp_engagement,
            monitoring,
        )
    };

    let (mrm_state_pub, hazard_pub, gear_pub, control_pub, turn_pub, op_mode_pub) = core_pubs;

    #[cfg(feature = "comp-mrm")]
    let (
        mrm_estop_status_pub,
        mrm_comfy_status_pub,
        mrm_pullover_status_pub,
        emergency_gear_pub,
        emergency_hazard_pub,
        emergency_turn_pub,
        emergency_holding_pub,
    ) = _comp_mrm_pubs;

    #[cfg(feature = "comp-cmd-gate-extra")]
    let (
        emergency_cmd_pub,
        gate_mode_pub,
        shift_decider_gear_pub,
        is_stopped_pub,
        gate_op_mode_pub,
        system_op_mode_pub,
        is_paused_pub,
        is_start_requested_pub,
        current_gate_mode_pub,
        filter_activated_pub,
        filter_flag_pub,
        filter_marker_pub,
        filter_marker_raw_pub,
    ) = _comp_cmd_gate_extra_pubs;

    #[cfg(feature = "comp-validator")]
    let (
        cv_debug_marker_pub,
        cv_output_markers_pub,
        cv_validation_status_pub,
        cv_virtual_wall_pub,
    ) = _comp_validator_pubs;

    #[cfg(feature = "comp-op-mode-mgr")]
    let (op_mode_debug_pub, is_autonomous_available_pub, published_time_pub) =
        _comp_op_mode_mgr_pubs;

    #[cfg(feature = "comp-engagement")]
    let (engage_api_pub, engage_compat_pub, autoware_state_pub, emergency_api_pub) =
        _comp_engagement_pubs;

    #[cfg(feature = "monitoring-topics")]
    let (
        diag_status_pub,
        diag_struct_pub,
        diag_unknowns_pub,
        cmd_mode_availability_pub,
        csm_control_pub,
        csm_localization_pub,
        csm_map_pub,
        csm_perception_pub,
        csm_planning_pub,
        csm_sensing_pub,
        csm_system_pub,
        csm_vehicle_pub,
        hazard_status_pub,
        op_mode_availability_pub,
    ) = _monitoring_pubs;

    // --- Sensing / heartbeat / external control subscriptions ---
    executor.add_subscription::<VelocityReport, _>("/vehicle/status/velocity_status", |msg| {
        with_island(|island| island.on_velocity_report(msg))
    })?;
    info!("Subscribed: /vehicle/status/velocity_status");

    let now_ms_hb = now_ms;
    executor.add_subscription::<Heartbeat, _>("/api/system/heartbeat", move |_msg| {
        with_island(|island| island.watchdog.on_heartbeat(now_ms_hb()));
    })?;
    info!("Subscribed: /api/system/heartbeat");

    let now_ms_cc = now_ms;
    executor.add_subscription::<Control, _>(
        "/control/trajectory_follower/control_cmd",
        move |msg| {
            with_island(|island| {
                island.auto_control = msg.clone();
                island.has_external_control = true;
                island.last_external_control_ms = now_ms_cc();
            })
        },
    )?;
    info!("Subscribed: control_cmd");

    #[cfg(feature = "comp-engagement")]
    {
        executor.add_subscription::<AutowareState, _>("/autoware/state", |msg| {
            with_island(|island| island.autoware_state = msg.clone());
        })?;
        info!("Subscribed: /autoware/state");
    }

    #[cfg(feature = "comp-cmd-gate-extra")]
    {
        executor.add_subscription::<GearReport, _>("/vehicle/status/gear_status", |msg| {
            with_island(|island| island.gear_report = msg.clone())
        })?;
        info!("Subscribed: /vehicle/status/gear_status");
    }

    // --- Trajectory follower input subscriptions (gated) ---
    #[cfg(feature = "controller-node")]
    {
        executor
            .add_subscription::<Trajectory, _>("/planning/scenario_planning/trajectory", |msg| {
                with_island(|island| island.on_trajectory(msg))
            })?;
        executor.add_subscription::<Odometry, _>("/localization/kinematic_state", |msg| {
            with_island(|island| island.on_odometry(msg))
        })?;
        executor
            .add_subscription::<SteeringReport, _>("/vehicle/status/steering_status", |msg| {
                with_island(|island| island.on_steering(msg))
            })?;
        executor.add_subscription::<AccelWithCovarianceStamped, _>(
            "/localization/acceleration",
            |msg| with_island(|island| island.on_acceleration(msg)),
        )?;
        info!("Subscribed: trajectory, odometry, steering, acceleration (controller inputs)");
    }

    // --- Services ---
    executor.add_service::<ChangeOperationMode, _>(
        "/api/operation_mode/change_to_autonomous",
        |_request| {
            with_island(|island| {
                island.autonomous_engaged = true;
                info!("ChangeOperationMode: STOP → AUTONOMOUS");
            });
            ChangeOperationModeResponse {
                status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                    success: true,
                    code: 0,
                    message: Default::default(),
                },
            }
        },
    )?;
    info!("Service: /api/operation_mode/change_to_autonomous");

    #[cfg(feature = "comp-engagement")]
    {
        executor.add_service::<EngageSrv, _>("/api/autoware/set/engage", |request| {
            with_island(|island| {
                island.autonomous_engaged = request.engage;
                info!(
                    "set/engage: {}",
                    if request.engage {
                        "ENGAGED"
                    } else {
                        "DISENGAGED"
                    }
                );
            });
            EngageResponse {
                status: tier4_external_api_msgs::msg::ResponseStatus {
                    code: TIER4_RESPONSE_SUCCESS,
                    message: Default::default(),
                },
            }
        })?;
        info!("Service: /api/autoware/set/engage");

        executor.add_service::<SetEmergency, _>("/api/autoware/set/emergency", |request| {
            with_island(|island| {
                island.external_emergency_stop = request.emergency;
                info!(
                    "set/emergency: {}",
                    if request.emergency {
                        "EMERGENCY SET"
                    } else {
                        "EMERGENCY CLEARED"
                    }
                );
            });
            SetEmergencyResponse {
                status: tier4_external_api_msgs::msg::ResponseStatus {
                    code: TIER4_RESPONSE_SUCCESS,
                    message: Default::default(),
                },
            }
        })?;
        info!("Service: /api/autoware/set/emergency");
    }

    #[cfg(feature = "comp-cmd-gate-extra")]
    {
        executor.add_service::<Trigger, _>(
            "/control/vehicle_cmd_gate/external_emergency_stop",
            |_request| {
                with_island(|island| {
                    island.external_emergency_stop = true;
                    info!("external_emergency_stop: TRIGGERED");
                });
                TriggerResponse {
                    success: true,
                    message: Default::default(),
                }
            },
        )?;
        executor.add_service::<Trigger, _>(
            "/control/vehicle_cmd_gate/clear_external_emergency_stop",
            |_request| {
                with_island(|island| {
                    island.external_emergency_stop = false;
                    info!("external_emergency_stop: CLEARED");
                });
                TriggerResponse {
                    success: true,
                    message: Default::default(),
                }
            },
        )?;
        executor.add_service::<ConfigLogger, _>(
            "/control/vehicle_cmd_gate/config_logger",
            |_request| logging_demo::srv::ConfigLoggerResponse { success: true },
        )?;
        executor.add_service::<SetStop, _>("/control/vehicle_cmd_gate/set_stop", |_request| {
            tier4_control_msgs::srv::SetStopResponse {
                status: Default::default(),
            }
        })?;
        info!(
            "Services: vehicle_cmd_gate (external_emergency_stop, clear_external_emergency_stop, config_logger, set_stop)"
        );
    }

    #[cfg(feature = "comp-op-mode-mgr")]
    {
        executor.add_service::<ControlModeCommand, _>(
            "/control/control_mode_request",
            |_request| ControlModeCommandResponse { success: true },
        )?;
        executor.add_service::<ChangeOperationMode, _>(
            "/api/operation_mode/change_to_stop",
            |_request| {
                with_island(|island| {
                    island.autonomous_engaged = false;
                    info!("ChangeOperationMode: → STOP");
                });
                ChangeOperationModeResponse {
                    status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                        success: true,
                        code: 0,
                        message: Default::default(),
                    },
                }
            },
        )?;
        executor.add_service::<ChangeOperationMode, _>(
            "/api/operation_mode/change_to_local",
            |_request| ChangeOperationModeResponse {
                status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                    success: false,
                    code: 1,
                    message: Default::default(),
                },
            },
        )?;
        executor.add_service::<ChangeOperationMode, _>(
            "/api/operation_mode/change_to_remote",
            |_request| ChangeOperationModeResponse {
                status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                    success: false,
                    code: 1,
                    message: Default::default(),
                },
            },
        )?;
        executor.add_service::<ChangeOperationMode, _>(
            "/api/operation_mode/enable_autoware_control",
            |_request| ChangeOperationModeResponse {
                status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                    success: true,
                    code: 0,
                    message: Default::default(),
                },
            },
        )?;
        executor.add_service::<ChangeOperationMode, _>(
            "/api/operation_mode/disable_autoware_control",
            |_request| ChangeOperationModeResponse {
                status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                    success: false,
                    code: 1,
                    message: Default::default(),
                },
            },
        )?;
        info!(
            "Services: /control/control_mode_request, /api/operation_mode/{{stop,local,remote,enable,disable}}"
        );
    }

    #[cfg(feature = "comp-mrm")]
    {
        executor.add_service::<OperateMrm, _>(
            "/system/mrm/comfortable_stop/operate",
            |_request| tier4_system_msgs::srv::OperateMrmResponse {
                response: Default::default(),
            },
        )?;
        executor.add_service::<OperateMrm, _>(
            "/system/mrm/emergency_stop/operate",
            |_request| tier4_system_msgs::srv::OperateMrmResponse {
                response: Default::default(),
            },
        )?;
        executor.add_service::<OperateMrm, _>(
            "/system/mrm/pull_over_manager/operate",
            |_request| tier4_system_msgs::srv::OperateMrmResponse {
                response: Default::default(),
            },
        )?;
        info!("Services: /system/mrm/{{emergency,comfortable,pull_over}}/operate");
    }

    #[cfg(feature = "comp-stubs")]
    {
        executor.add_service::<InterfaceVersion, _>("/api/interface/version", |_request| {
            InterfaceVersionResponse {
                major: 1,
                minor: 5,
                patch: 0,
            }
        })?;
        executor.add_service::<ResetDiagGraph, _>("/api/system/diagnostics/reset", |_request| {
            ResetDiagGraphResponse {
                status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                    success: true,
                    code: 0,
                    message: Default::default(),
                },
            }
        })?;
        executor.add_service::<Trigger, _>("/autoware/shutdown", |_request| TriggerResponse {
            success: true,
            message: Default::default(),
        })?;
        executor.add_service::<ResetDiagGraphTier4, _>("/diagnostics_graph/reset", |_request| {
            tier4_system_msgs::srv::ResetDiagGraphResponse {
                status: Default::default(),
            }
        })?;
        executor.add_service::<SetBool, _>("/system/aggregator/set_initializing", |_request| {
            std_srvs::srv::SetBoolResponse {
                success: true,
                message: Default::default(),
            }
        })?;
        executor.add_service::<ChangeAutowareControl, _>(
            "/system/operation_mode/change_autoware_control",
            |_request| autoware_system_msgs::srv::ChangeAutowareControlResponse {
                status: Default::default(),
            },
        )?;
        info!("Services: Phase 12 gap-closure stubs (6 services)");
    }

    // --- 30 Hz main control timer ---
    let now_ms_timer = now_ms;
    executor.add_timer(TimerDuration::from_millis(33), move || {
        with_island(|island| {
            let now = now_ms_timer();

            // ── Trajectory follower (gated) ─────────────────────────
            #[cfg(feature = "controller-node")]
            if !island.has_external_control {
                island.run_controller(now as f64 / 1000.0);
            }

            // ── Staleness guard ─────────────────────────────────────
            let auto_control_timestamp_ms = if island.has_external_control {
                let age_ms = now.saturating_sub(island.last_external_control_ms);
                if age_ms > EXTERNAL_CONTROL_STALE_MS {
                    island.auto_control.longitudinal.velocity = island.current_velocity as f32;
                    island.auto_control.longitudinal.acceleration = -1.5;
                }
                island.last_external_control_ms
            } else {
                now
            };

            // ── MRM chain ──────────────────────────────────────────
            let watchdog_update = island.watchdog.check(now);

            island.mrm_handler.update_velocity(island.current_velocity);
            if let Some(ref availability) = watchdog_update {
                island.mrm_handler.update_availability(availability);
            }

            let mrm_output = island.mrm_handler.update();

            if let Some(activate) = mrm_output.emergency_stop_operate {
                if activate {
                    island
                        .emergency_stop
                        .set_initial_velocity(island.current_velocity as f32);
                }
                island.emergency_stop.operate(activate);
            }
            if let Some(activate) = mrm_output.comfortable_stop_operate {
                if activate {
                    island
                        .comfortable_stop
                        .set_initial_velocity(island.current_velocity as f32);
                }
                island.comfortable_stop.operate(activate);
            }

            let emergency_control = island.emergency_stop.update(DT);
            let comfortable_control = island.comfortable_stop.update(DT);

            let mrm_control = if island.emergency_stop.is_operating() {
                emergency_control
            } else if island.comfortable_stop.is_operating() {
                comfortable_control
            } else {
                Control::default()
            };

            // ── Command output ─────────────────────────────────────
            island.cmd_gate.set_system_emergency(
                island.mrm_handler.state() == MRM_STATE_OPERATING || island.external_emergency_stop,
            );
            island
                .cmd_gate
                .set_current_speed(island.current_velocity as f32);
            island.cmd_gate.set_engaged(island.autonomous_engaged);

            let auto_gear = island.shift_decider.decide(
                &island.autoware_state,
                &island.auto_control,
                &island.gear_report,
            );
            island.cmd_gate.set_autonomous_commands(
                SourceCommands {
                    control: island.auto_control.clone(),
                    gear: GearCommand {
                        command: auto_gear,
                        ..Default::default()
                    },
                    turn_indicators: TurnIndicatorsCommand::default(),
                    hazard_lights: HazardLightsCommand::default(),
                },
                auto_control_timestamp_ms,
            );

            island.cmd_gate.set_emergency_commands(SourceCommands {
                control: mrm_control,
                gear: mrm_output.gear.clone(),
                turn_indicators: TurnIndicatorsCommand::default(),
                hazard_lights: mrm_output.hazard_lights.clone(),
            });

            let gate_output = island.cmd_gate.update(now);

            // ── Validation ─────────────────────────────────────────
            let target_vel = gate_output.control.longitudinal.velocity as f64;
            island.control_validator.validate(
                &gate_output.control,
                island.current_velocity,
                island.accel.linear.x,
                target_vel,
                0.0,
                DT as f64,
            );

            island.op_mode_mgr.update_velocity(island.current_velocity);
            island.op_mode_mgr.update_control_cmd(&gate_output.control);
            island.op_mode_mgr.update(DT as f64);

            if island.control_validator.status().invalid_count >= VALIDATION_FAILURE_THRESHOLD {
                island
                    .mrm_handler
                    .update_availability(&OperationModeAvailability::default());
            }

            // ── Publish ────────────────────────────────────────────
            mrm_state_pub.publish(&mrm_output.mrm_state).ok();
            hazard_pub.publish(&mrm_output.hazard_lights).ok();
            gear_pub.publish(&gate_output.gear).ok();
            control_pub.publish(&gate_output.control).ok();
            turn_pub.publish(&gate_output.turn_indicators).ok();

            let current_mode = if island.autonomous_engaged {
                OP_MODE_AUTONOMOUS
            } else {
                OP_MODE_STOP
            };
            let op_mode_state = OperationModeState {
                stamp: Default::default(),
                mode: current_mode,
                is_autoware_control_enabled: island.autonomous_engaged,
                is_in_transition: false,
                is_stop_mode_available: true,
                is_autonomous_mode_available: true,
                is_local_mode_available: true,
                is_remote_mode_available: true,
            };
            op_mode_pub.publish(&op_mode_state).ok();

            #[allow(unused_variables)]
            let is_emergency =
                island.mrm_handler.state() == MRM_STATE_OPERATING || island.external_emergency_stop;

            #[cfg(feature = "comp-mrm")]
            {
                mrm_estop_status_pub
                    .publish(&MrmBehaviorStatus {
                        stamp: Default::default(),
                        state: if island.emergency_stop.is_operating() {
                            MRM_BEHAVIOR_OPERATING
                        } else {
                            MRM_BEHAVIOR_AVAILABLE
                        },
                    })
                    .ok();
                mrm_comfy_status_pub
                    .publish(&MrmBehaviorStatus {
                        stamp: Default::default(),
                        state: if island.comfortable_stop.is_operating() {
                            MRM_BEHAVIOR_OPERATING
                        } else {
                            MRM_BEHAVIOR_AVAILABLE
                        },
                    })
                    .ok();
                mrm_pullover_status_pub
                    .publish(&MrmBehaviorStatus {
                        stamp: Default::default(),
                        state: MRM_BEHAVIOR_AVAILABLE,
                    })
                    .ok();
                emergency_gear_pub.publish(&mrm_output.gear).ok();
                emergency_hazard_pub.publish(&mrm_output.hazard_lights).ok();
                emergency_turn_pub
                    .publish(&TurnIndicatorsCommand::default())
                    .ok();
                emergency_holding_pub
                    .publish(&EmergencyHoldingState {
                        stamp: Default::default(),
                        is_holding: false,
                    })
                    .ok();
            }

            #[cfg(feature = "comp-cmd-gate-extra")]
            {
                emergency_cmd_pub
                    .publish(&VehicleEmergencyStamped {
                        stamp: Default::default(),
                        emergency: is_emergency,
                    })
                    .ok();
                gate_mode_pub
                    .publish(&GateMode {
                        data: GATE_MODE_AUTO,
                    })
                    .ok();
                shift_decider_gear_pub
                    .publish(&GearCommand {
                        command: auto_gear,
                        ..Default::default()
                    })
                    .ok();
                is_stopped_pub
                    .publish(&IsStopped {
                        stamp: Default::default(),
                        data: island.is_stopped,
                        requested_sources: Default::default(),
                    })
                    .ok();
                gate_op_mode_pub.publish(&op_mode_state).ok();
                system_op_mode_pub.publish(&op_mode_state).ok();
                is_paused_pub
                    .publish(&IsPaused {
                        stamp: Default::default(),
                        data: false,
                    })
                    .ok();
                is_start_requested_pub
                    .publish(&IsStartRequested {
                        stamp: Default::default(),
                        data: false,
                    })
                    .ok();
                current_gate_mode_pub
                    .publish(&GateMode {
                        data: GATE_MODE_AUTO,
                    })
                    .ok();
                filter_activated_pub
                    .publish(&IsFilterActivated {
                        stamp: Default::default(),
                        is_activated: false,
                        is_activated_on_steering: false,
                        is_activated_on_steering_rate: false,
                        is_activated_on_speed: false,
                        is_activated_on_acceleration: false,
                        is_activated_on_jerk: false,
                    })
                    .ok();
                filter_flag_pub
                    .publish(&BoolStamped {
                        stamp: Default::default(),
                        data: false,
                    })
                    .ok();
                let empty_markers_gate = MarkerArray {
                    markers: Default::default(),
                };
                filter_marker_pub.publish(&empty_markers_gate).ok();
                filter_marker_raw_pub.publish(&empty_markers_gate).ok();
            }

            #[cfg(feature = "comp-engagement")]
            {
                engage_api_pub
                    .publish(&Engage {
                        stamp: Default::default(),
                        engage: true,
                    })
                    .ok();
                engage_compat_pub
                    .publish(&Engage {
                        stamp: Default::default(),
                        engage: true,
                    })
                    .ok();
                autoware_state_pub
                    .publish(&AutowareState {
                        stamp: Default::default(),
                        state: AUTOWARE_STATE_DRIVING,
                    })
                    .ok();
                emergency_api_pub
                    .publish(&Emergency {
                        stamp: Default::default(),
                        emergency: is_emergency,
                    })
                    .ok();
            }

            #[cfg(feature = "comp-validator")]
            {
                let empty_markers = MarkerArray {
                    markers: Default::default(),
                };
                cv_debug_marker_pub.publish(&empty_markers).ok();
                cv_output_markers_pub.publish(&empty_markers).ok();
                cv_virtual_wall_pub.publish(&empty_markers).ok();
                let cv_status = island.control_validator.status();
                cv_validation_status_pub
                    .publish(&ControlValidatorStatus {
                        stamp: Default::default(),
                        is_valid_max_distance_deviation: true,
                        is_valid_acc: cv_status.is_valid_acc,
                        is_rolling_back: cv_status.is_rolling_back,
                        is_over_velocity: cv_status.is_over_velocity,
                        is_valid_lateral_jerk: cv_status.is_valid_lateral_jerk,
                        has_overrun_stop_point: false,
                        will_overrun_stop_point: false,
                        is_valid_latency: true,
                        is_valid_yaw: true,
                        is_warn_yaw: false,
                        max_distance_deviation: 0.0,
                        steering_rate: cv_status.steering_rate,
                        lateral_jerk: cv_status.lateral_jerk,
                        desired_acc: cv_status.desired_acc,
                        measured_acc: cv_status.measured_acc,
                        target_vel: cv_status.target_vel,
                        vehicle_vel: cv_status.vehicle_vel,
                        dist_to_stop: 0.0,
                        pred_dist_to_stop: 0.0,
                        nearest_trajectory_vel: 0.0,
                        latency: 0.0,
                        yaw_deviation: 0.0,
                        invalid_count: cv_status.invalid_count as i64,
                    })
                    .ok();
            }

            #[cfg(feature = "comp-op-mode-mgr")]
            {
                op_mode_debug_pub
                    .publish(&OperationModeTransitionManagerDebug {
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
                        current_speed: island.current_velocity,
                        target_control_speed: 0.0,
                        target_planning_speed: 0.0,
                        target_control_acceleration: 0.0,
                        lateral_acceleration: 0.0,
                        lateral_acceleration_deviation: 0.0,
                        lateral_deviation: 0.0,
                        yaw_deviation: 0.0,
                        speed_deviation: 0.0,
                    })
                    .ok();
                published_time_pub
                    .publish(&PublishedTime {
                        header: Default::default(),
                        published_stamp: Default::default(),
                    })
                    .ok();
                is_autonomous_available_pub
                    .publish(&ModeChangeAvailable {
                        stamp: Default::default(),
                        available: island.autonomous_engaged,
                    })
                    .ok();
            }

            #[cfg(feature = "monitoring-topics")]
            {
                diag_status_pub.publish(&DIAG_STATUS_DEFAULT).ok();
                diag_struct_pub.publish(&DIAG_STRUCT_DEFAULT).ok();
                diag_unknowns_pub.publish(&DiagnosticArray::default()).ok();
                cmd_mode_availability_pub
                    .publish(&CommandModeAvailability::default())
                    .ok();
                let csm_available = ModeChangeAvailable {
                    stamp: Default::default(),
                    available: true,
                };
                csm_control_pub.publish(&csm_available).ok();
                csm_localization_pub.publish(&csm_available).ok();
                csm_map_pub.publish(&csm_available).ok();
                csm_perception_pub.publish(&csm_available).ok();
                csm_planning_pub.publish(&csm_available).ok();
                csm_sensing_pub.publish(&csm_available).ok();
                csm_system_pub.publish(&csm_available).ok();
                csm_vehicle_pub.publish(&csm_available).ok();
                hazard_status_pub
                    .publish(&HazardStatusStamped::default())
                    .ok();
                op_mode_availability_pub
                    .publish(&OperationModeAvailability {
                        stamp: Default::default(),
                        stop: true,
                        autonomous: true,
                        local: true,
                        remote: true,
                        emergency_stop: true,
                        comfortable_stop: true,
                        pull_over: true,
                    })
                    .ok();
            }
        });
    })?;
    info!("30 Hz control loop ready");

    Ok(())
}
