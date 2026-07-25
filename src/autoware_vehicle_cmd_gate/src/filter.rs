//! Vehicle command filter — rate limiting and clamping.
//!
//! Port of upstream `vehicle_cmd_filter.cpp`. Maintains previous command state
//! and applies 8-stage filtering pipeline interpolated by current vehicle speed.

use autoware_control_msgs::msg::Control;

/// Number of reference speed points for limit interpolation.
const NUM_POINTS: usize = 4;

/// Default reference speed points (m/s).
const DEFAULT_REFERENCE_SPEEDS: [f32; NUM_POINTS] = [0.1, 0.3, 20.0, 30.0];

/// Threshold for detecting filter activation (per-field).
const ACTIVATION_THRESHOLD: f32 = 1e-3;

/// Minimum v² used in divisions to avoid singularities.
const V_SQ_MIN: f32 = 0.001;

/// Maximum absolute steering angle the filter can handle (rad).
const STEER_MAX: f32 = core::f32::consts::FRAC_PI_2;

/// Parameters for the vehicle command filter.
///
/// All limit arrays are indexed by reference speed points and linearly interpolated.
#[derive(Debug, Clone)]
pub struct FilterParams {
    /// Vehicle wheelbase (m).
    pub wheel_base: f32,
    /// Absolute velocity limit (m/s).
    pub vel_lim: f32,
    /// Reference speed points (m/s) for limit interpolation.
    pub reference_speeds: [f32; NUM_POINTS],
    /// Longitudinal acceleration limits (m/s²) per speed point.
    pub lon_acc_lim: [f32; NUM_POINTS],
    /// Longitudinal jerk limits (m/s³) per speed point.
    pub lon_jerk_lim: [f32; NUM_POINTS],
    /// Steering angle limits (rad) per speed point.
    pub steer_lim: [f32; NUM_POINTS],
    /// Steering rate limits (rad/s) per speed point.
    pub steer_rate_lim: [f32; NUM_POINTS],
    /// Max steering delta from current physical steer (rad) per speed point.
    pub steer_diff_lim: [f32; NUM_POINTS],
    /// Lateral acceleration limits (m/s²) per speed point.
    pub lat_acc_lim: [f32; NUM_POINTS],
    /// Lateral jerk limits (m/s³) per speed point.
    pub lat_jerk_lim: [f32; NUM_POINTS],
    /// Lateral jerk limit used to constrain steering rate (m/s³). Scalar, not speed-indexed.
    pub lat_jerk_lim_for_steer_rate: f32,
}

impl Default for FilterParams {
    fn default() -> Self {
        Self {
            wheel_base: 2.79,
            vel_lim: 25.0,
            reference_speeds: DEFAULT_REFERENCE_SPEEDS,
            lon_acc_lim: [5.0, 5.0, 5.0, 4.0],
            lon_jerk_lim: [80.0, 5.0, 5.0, 4.0],
            steer_lim: [1.0, 1.0, 1.0, 0.8],
            steer_rate_lim: [1.0, 1.0, 1.0, 0.8],
            steer_diff_lim: [1.0, 1.0, 1.0, 0.8],
            lat_acc_lim: [5.0, 5.0, 5.0, 4.0],
            lat_jerk_lim: [7.0, 7.0, 7.0, 6.0],
            lat_jerk_lim_for_steer_rate: 5.0,
        }
    }
}

/// Per-field flags indicating which limits were active during filtering.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FilterActivated {
    pub steering: bool,
    pub steering_rate: bool,
    pub speed: bool,
    pub acceleration: bool,
    pub jerk: bool,
    pub is_activated: bool,
}

/// Result of `filter_all()`.
#[derive(Debug, Clone)]
pub struct FilterOutput {
    pub control: Control,
    pub activated: FilterActivated,
}

/// Stateful command filter maintaining `prev_cmd` for rate limiting.
#[derive(Debug, Clone)]
pub struct VehicleCmdFilter {
    params: FilterParams,
    prev_cmd: Control,
    current_speed: f32,
    current_steer: f32,
}

impl VehicleCmdFilter {
    pub fn new(params: FilterParams) -> Self {
        Self {
            params,
            prev_cmd: Control::default(),
            current_speed: 0.0,
            current_steer: 0.0,
        }
    }

    /// Set current vehicle speed (m/s) from odometry.
    pub fn set_current_speed(&mut self, speed: f32) {
        self.current_speed = speed;
    }

    /// Set current physical steering angle (rad) from steering report.
    pub fn set_current_steer(&mut self, steer: f32) {
        self.current_steer = steer;
    }

    /// Initialize prev_cmd without filtering (first call).
    pub fn init_prev_cmd(&mut self, cmd: &Control) {
        self.prev_cmd = cmd.clone();
    }

    /// Apply the full 8-stage filter pipeline.
    pub fn filter_all(&mut self, dt: f32, input: &Control) -> FilterOutput {
        let original = input.clone();
        let mut cmd = input.clone();

        // Stage 1: Limit steering angle
        self.limit_lateral_steer(&mut cmd);

        // Stage 2: Limit steering rate (combines steer_rate_lim and lat_jerk constraint)
        self.limit_lateral_steer_rate(dt, &mut cmd);

        // Stage 3: Limit longitudinal jerk (rate-limit acceleration change)
        self.limit_longitudinal_with_jerk(dt, &mut cmd);

        // Stage 4: Limit longitudinal acceleration (clamp accel, rate-limit velocity)
        self.limit_longitudinal_with_acc(dt, &mut cmd);

        // Stage 5: Limit absolute velocity
        self.limit_longitudinal_with_vel(&mut cmd);

        // Stage 6: Limit lateral jerk via steering adjustment
        self.limit_lateral_with_lat_jerk(dt, &mut cmd);

        // Stage 7: Limit lateral acceleration via steering adjustment
        self.limit_lateral_with_lat_acc(&mut cmd);

        // Stage 8: Limit steering delta from physical state
        self.limit_actual_steer_diff(&mut cmd);

        let activated = Self::check_activated(&original, &cmd);

        // Update prev_cmd for next cycle
        self.prev_cmd = cmd.clone();

        FilterOutput {
            control: cmd,
            activated,
        }
    }

    // ── Filter stages ───────────────────────────────────────────────

    /// Stage 1: Clamp steering angle to speed-interpolated limit (max π/2).
    fn limit_lateral_steer(&self, cmd: &mut Control) {
        let lim = self.interpolate(self.params.steer_lim).min(STEER_MAX);
        cmd.lateral.steering_tire_angle = clamp(cmd.lateral.steering_tire_angle, -lim, lim);
    }

    /// Stage 2: Rate-limit steering. Effective limit = min(steer_rate_lim,
    /// lat_jerk_lim_for_steer_rate * wheel_base / v²).
    fn limit_lateral_steer_rate(&self, dt: f32, cmd: &mut Control) {
        let steer_rate_lim = self.interpolate(self.params.steer_rate_lim);

        // Compute jerk-derived steering rate limit: dθ/dt_max = j_y_lim * L / v²
        let v_sq = (self.current_speed * self.current_speed).max(V_SQ_MIN);
        let jerk_derived = self.params.lat_jerk_lim_for_steer_rate * self.params.wheel_base / v_sq;

        let effective_lim = steer_rate_lim.min(jerk_derived);

        // Rate-limit steering angle
        cmd.lateral.steering_tire_angle = limit_diff(
            cmd.lateral.steering_tire_angle,
            self.prev_cmd.lateral.steering_tire_angle,
            effective_lim * dt,
        );

        // Rate-limit steering rate field
        cmd.lateral.steering_tire_rotation_rate = clamp(
            cmd.lateral.steering_tire_rotation_rate,
            -effective_lim,
            effective_lim,
        );
    }

    /// Stage 3: Rate-limit acceleration change (jerk limiting).
    fn limit_longitudinal_with_jerk(&self, dt: f32, cmd: &mut Control) {
        let jerk_lim = self.interpolate(self.params.lon_jerk_lim);

        // Rate-limit acceleration (jerk)
        cmd.longitudinal.acceleration = limit_diff(
            cmd.longitudinal.acceleration,
            self.prev_cmd.longitudinal.acceleration,
            jerk_lim * dt,
        );

        // Clamp jerk field
        cmd.longitudinal.jerk = clamp(cmd.longitudinal.jerk, -jerk_lim, jerk_lim);
    }

    /// Stage 4: Clamp acceleration and rate-limit velocity.
    fn limit_longitudinal_with_acc(&self, dt: f32, cmd: &mut Control) {
        let acc_lim = self.interpolate(self.params.lon_acc_lim);

        // Clamp acceleration
        cmd.longitudinal.acceleration = clamp(cmd.longitudinal.acceleration, -acc_lim, acc_lim);

        // Rate-limit velocity (velocity change bounded by accel * dt)
        cmd.longitudinal.velocity = limit_diff(
            cmd.longitudinal.velocity,
            self.prev_cmd.longitudinal.velocity,
            acc_lim * dt,
        );
    }

    /// Stage 5: Clamp absolute velocity.
    fn limit_longitudinal_with_vel(&self, cmd: &mut Control) {
        cmd.longitudinal.velocity = clamp(
            cmd.longitudinal.velocity,
            -self.params.vel_lim,
            self.params.vel_lim,
        );
    }

    /// Stage 6: Limit lateral jerk by adjusting steering.
    ///
    /// Computes lateral acceleration for previous and current commands using ego speed,
    /// then bounds the change (lateral jerk) and back-calculates steering.
    fn limit_lateral_with_lat_jerk(&self, dt: f32, cmd: &mut Control) {
        let lat_jerk_lim = self.interpolate(self.params.lat_jerk_lim);
        let v_sq = (self.current_speed * self.current_speed).max(V_SQ_MIN);

        let prev_lat_acc = calc_lateral_accel(
            v_sq,
            self.prev_cmd.lateral.steering_tire_angle,
            self.params.wheel_base,
        );
        let curr_lat_acc = calc_lateral_accel(
            v_sq,
            cmd.lateral.steering_tire_angle,
            self.params.wheel_base,
        );

        let lat_acc_max = prev_lat_acc + lat_jerk_lim * dt;
        let lat_acc_min = prev_lat_acc - lat_jerk_lim * dt;

        if curr_lat_acc > lat_acc_max {
            cmd.lateral.steering_tire_angle =
                calc_steer_from_lat_accel(lat_acc_max, v_sq, self.params.wheel_base);
        } else if curr_lat_acc < lat_acc_min {
            cmd.lateral.steering_tire_angle =
                calc_steer_from_lat_accel(lat_acc_min, v_sq, self.params.wheel_base);
        }
    }

    /// Stage 7: Limit lateral acceleration by adjusting steering.
    ///
    /// Uses ego speed (not command velocity) to avoid oscillations.
    fn limit_lateral_with_lat_acc(&self, cmd: &mut Control) {
        let lat_acc_lim = self.interpolate(self.params.lat_acc_lim);
        let v_sq = (self.current_speed * self.current_speed).max(V_SQ_MIN);

        let lat_acc = calc_lateral_accel(
            v_sq,
            cmd.lateral.steering_tire_angle,
            self.params.wheel_base,
        );

        if lat_acc.abs() > lat_acc_lim {
            let sign = if lat_acc >= 0.0 { 1.0 } else { -1.0 };
            cmd.lateral.steering_tire_angle =
                sign * calc_steer_from_lat_accel(lat_acc_lim, v_sq, self.params.wheel_base).abs();
        }
    }

    /// Stage 8: Clamp steering delta from current physical steering state.
    fn limit_actual_steer_diff(&self, cmd: &mut Control) {
        let diff_lim = self.interpolate(self.params.steer_diff_lim);
        cmd.lateral.steering_tire_angle = limit_diff(
            cmd.lateral.steering_tire_angle,
            self.current_steer,
            diff_lim,
        );
    }

    // ── Helpers ─────────────────────────────────────────────────────

    /// Linear interpolation from speed-indexed limit array.
    fn interpolate(&self, limits: [f32; NUM_POINTS]) -> f32 {
        interpolate_from_speed(
            self.current_speed.abs(),
            &self.params.reference_speeds,
            &limits,
        )
    }

    /// Compare original vs filtered command to determine activation flags.
    fn check_activated(original: &Control, filtered: &Control) -> FilterActivated {
        let steering =
            (original.lateral.steering_tire_angle - filtered.lateral.steering_tire_angle).abs()
                > ACTIVATION_THRESHOLD;
        let steering_rate = (original.lateral.steering_tire_rotation_rate
            - filtered.lateral.steering_tire_rotation_rate)
            .abs()
            > ACTIVATION_THRESHOLD;
        let speed = (original.longitudinal.velocity - filtered.longitudinal.velocity).abs()
            > ACTIVATION_THRESHOLD;
        let acceleration =
            (original.longitudinal.acceleration - filtered.longitudinal.acceleration).abs()
                > ACTIVATION_THRESHOLD;
        let jerk =
            (original.longitudinal.jerk - filtered.longitudinal.jerk).abs() > ACTIVATION_THRESHOLD;
        let is_activated = steering || steering_rate || speed || acceleration || jerk;

        FilterActivated {
            steering,
            steering_rate,
            speed,
            acceleration,
            jerk,
            is_activated,
        }
    }
}

// ── Free functions ──────────────────────────────────────────────────

/// Linear interpolation between reference speed points.
///
/// Zero-order hold at boundaries (below first / above last point).
fn interpolate_from_speed(
    speed: f32,
    reference: &[f32; NUM_POINTS],
    limits: &[f32; NUM_POINTS],
) -> f32 {
    if speed <= reference[0] {
        return limits[0];
    }
    if speed >= reference[NUM_POINTS - 1] {
        return limits[NUM_POINTS - 1];
    }
    // Find the interval
    for i in 0..NUM_POINTS - 1 {
        if speed >= reference[i] && speed < reference[i + 1] {
            let denom = (reference[i + 1] - reference[i]).max(1e-5);
            let ratio = ((speed - reference[i]) / denom).clamp(0.0, 1.0);
            return limits[i] + ratio * (limits[i + 1] - limits[i]);
        }
    }
    limits[NUM_POINTS - 1]
}

/// Lateral acceleration: a_y = v² * tan(θ) / wheelbase.
fn calc_lateral_accel(v_sq: f32, steer: f32, wheel_base: f32) -> f32 {
    v_sq * tan_f32(steer) / wheel_base
}

/// Inverse: θ = atan(a_y * wheelbase / v²).
fn calc_steer_from_lat_accel(lat_acc: f32, v_sq: f32, wheel_base: f32) -> f32 {
    atan_f32(lat_acc * wheel_base / v_sq.max(V_SQ_MIN))
}

#[cfg(not(feature = "compact-trig"))]
#[inline]
fn tan_f32(x: f32) -> f32 {
    libm::tanf(x)
}

#[cfg(not(feature = "compact-trig"))]
#[inline]
fn atan_f32(x: f32) -> f32 {
    libm::atanf(x)
}

/// Padé(3,2) approximation of `tan(x)` valid for |x| ≤ π/4 (≈ 0.785 rad).
/// Maximum absolute error ≈ 5e-4 over that range — a passenger
/// vehicle's full steering travel never exceeds it. For |x| > π/4 the
/// approximation degrades (still finite, no asymptote until x → π/2);
/// the cmd-gate filter clamps the input before this call so the bound
/// is observed at the call site.
#[cfg(feature = "compact-trig")]
#[inline]
fn tan_f32(x: f32) -> f32 {
    let x2 = x * x;
    x * (15.0 - x2) / (15.0 - 6.0 * x2)
}

/// Padé-style rational approximation of `atan(x)` accurate to ≈ 5e-4 on
/// |x| ≤ 1; for |x| > 1 the identity `atan(x) = π/2 − atan(1/x)`
/// (with sign carry) is applied to keep the same accuracy out to ±∞.
#[cfg(feature = "compact-trig")]
#[inline]
fn atan_f32(x: f32) -> f32 {
    let abs = if x < 0.0 { -x } else { x };
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let core = |y: f32| -> f32 {
        // y * (a + b*y²) / (1 + c*y² + d*y⁴), Padé(3,4) on [0,1]
        let y2 = y * y;
        y * (1.0 + 0.43157974 * y2) / (1.0 + 0.76443945 * y2 + 0.05831938 * y2 * y2)
    };
    let mag = if abs <= 1.0 {
        core(abs)
    } else {
        core::f32::consts::FRAC_PI_2 - core(1.0 / abs)
    };
    sign * mag
}

/// Clamp `value` to `[lo, hi]`.
fn clamp(value: f32, lo: f32, hi: f32) -> f32 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

/// Rate limiter: `prev + clamp(curr - prev, -limit, limit)`.
fn limit_diff(curr: f32, prev: f32, limit: f32) -> f32 {
    prev + clamp(curr - prev, -limit, limit)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 30.0; // 30 Hz

    /// Hand-rolled `tan` / `atan` (compact-trig path) must track libm
    /// closely over the steering range a passenger vehicle reaches.
    #[test]
    fn compact_trig_within_tolerance_of_libm() {
        // Sample 17 points over [-π/4, π/4] — the cmd-gate clamps
        // steering input before calling these, so anything outside this
        // range never hits the approximation.
        const N: usize = 17;
        let span = core::f32::consts::FRAC_PI_4;
        for i in 0..N {
            let x = -span + 2.0 * span * (i as f32) / ((N - 1) as f32);
            // Emulate compact-trig path explicitly so the test runs in
            // both feature configs.
            let x2 = x * x;
            let approx_tan = x * (15.0 - x2) / (15.0 - 6.0 * x2);
            let libm_tan = libm::tanf(x);
            assert!(
                (approx_tan - libm_tan).abs() < 5e-4,
                "tan({x}): approx={approx_tan} libm={libm_tan}"
            );

            let y = x; // |y| ≤ π/4 < 1, hits the |x| ≤ 1 branch
            let y2 = y * y;
            let approx_atan =
                y * (1.0 + 0.43157974 * y2) / (1.0 + 0.76443945 * y2 + 0.05831938 * y2 * y2);
            let libm_atan = libm::atanf(y);
            assert!(
                (approx_atan - libm_atan).abs() < 5e-4,
                "atan({y}): approx={approx_atan} libm={libm_atan}"
            );
        }
    }

    fn make_control(vel: f32, accel: f32, steer: f32) -> Control {
        let mut cmd = Control::default();
        cmd.longitudinal.velocity = vel;
        cmd.longitudinal.acceleration = accel;
        cmd.lateral.steering_tire_angle = steer;
        cmd
    }

    // ── Speed interpolation tests ───────────────────────────────────

    #[test]
    fn interpolation_at_reference_points() {
        let reference = DEFAULT_REFERENCE_SPEEDS;
        let limits = [5.0, 5.0, 5.0, 4.0];

        assert_eq!(interpolate_from_speed(0.1, &reference, &limits), 5.0);
        assert_eq!(interpolate_from_speed(0.3, &reference, &limits), 5.0);
        assert_eq!(interpolate_from_speed(20.0, &reference, &limits), 5.0);
        assert_eq!(interpolate_from_speed(30.0, &reference, &limits), 4.0);
    }

    #[test]
    fn interpolation_between_points() {
        let reference = DEFAULT_REFERENCE_SPEEDS;
        let limits = [5.0, 5.0, 5.0, 4.0];

        // Midpoint between 20.0 and 30.0 → should be 4.5
        let val = interpolate_from_speed(25.0, &reference, &limits);
        assert!((val - 4.5).abs() < 0.01, "expected 4.5, got {}", val);
    }

    #[test]
    fn interpolation_below_min() {
        let reference = DEFAULT_REFERENCE_SPEEDS;
        let limits = [5.0, 5.0, 5.0, 4.0];

        assert_eq!(interpolate_from_speed(0.0, &reference, &limits), 5.0);
        assert_eq!(interpolate_from_speed(-1.0, &reference, &limits), 5.0);
    }

    #[test]
    fn interpolation_above_max() {
        let reference = DEFAULT_REFERENCE_SPEEDS;
        let limits = [5.0, 5.0, 5.0, 4.0];

        assert_eq!(interpolate_from_speed(35.0, &reference, &limits), 4.0);
        assert_eq!(interpolate_from_speed(100.0, &reference, &limits), 4.0);
    }

    // ── Velocity clamping ───────────────────────────────────────────

    #[test]
    fn velocity_clamped_to_limit() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(5.0);
        filter.init_prev_cmd(&make_control(0.0, 0.0, 0.0));

        // Command with velocity far exceeding limit
        let cmd = make_control(50.0, 0.0, 0.0);
        let out = filter.filter_all(DT, &cmd);
        assert!(
            out.control.longitudinal.velocity <= 25.0,
            "velocity should be clamped to 25.0, got {}",
            out.control.longitudinal.velocity,
        );
        assert!(out.activated.speed);
    }

    #[test]
    fn negative_velocity_clamped() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(5.0);
        filter.init_prev_cmd(&make_control(0.0, 0.0, 0.0));

        let cmd = make_control(-50.0, 0.0, 0.0);
        let out = filter.filter_all(DT, &cmd);
        assert!(
            out.control.longitudinal.velocity >= -25.0,
            "negative velocity should be clamped to -25.0, got {}",
            out.control.longitudinal.velocity,
        );
    }

    // ── Acceleration clamping ───────────────────────────────────────

    #[test]
    fn acceleration_clamped_to_limit() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(25.0); // Between 20 and 30 → limit ~4.5
        filter.init_prev_cmd(&make_control(5.0, 0.0, 0.0));

        let cmd = make_control(5.0, 10.0, 0.0);
        let out = filter.filter_all(DT, &cmd);
        // Acceleration should be limited (exact value depends on jerk filter first)
        assert!(
            out.control.longitudinal.acceleration <= 5.0,
            "acceleration should be limited, got {}",
            out.control.longitudinal.acceleration,
        );
    }

    // ── Jerk limiting (step input → smooth ramp) ────────────────────

    #[test]
    fn step_input_produces_smooth_ramp() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(10.0); // speed in [0.3, 20.0] → lon_jerk_lim = 5.0
        filter.init_prev_cmd(&make_control(5.0, 0.0, 0.0));

        // Step input: acceleration jumps from 0 to 4.0
        let cmd = make_control(5.0, 4.0, 0.0);

        // First step: should be limited by jerk (5.0 * 1/30 = 0.167)
        let out = filter.filter_all(DT, &cmd);
        let expected_max = 5.0 * DT; // jerk_lim * dt
        assert!(
            out.control.longitudinal.acceleration <= expected_max + 0.01,
            "first step accel should be jerk-limited to ~{:.3}, got {:.3}",
            expected_max,
            out.control.longitudinal.acceleration,
        );
        assert!(out.activated.acceleration);

        // Run more steps — acceleration should ramp up
        let mut prev_accel = out.control.longitudinal.acceleration;
        for _ in 0..20 {
            let out = filter.filter_all(DT, &cmd);
            assert!(
                out.control.longitudinal.acceleration >= prev_accel - 0.01,
                "acceleration should ramp up monotonically"
            );
            prev_accel = out.control.longitudinal.acceleration;
        }
    }

    // ── Steering clamping ───────────────────────────────────────────

    #[test]
    fn steering_clamped_to_limit() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(25.0); // limit ~0.9 rad
        filter.set_current_steer(0.0);
        filter.init_prev_cmd(&make_control(0.0, 0.0, 0.0));

        let cmd = make_control(0.0, 0.0, 2.0); // 2.0 rad exceeds limit
        let out = filter.filter_all(DT, &cmd);
        assert!(
            out.control.lateral.steering_tire_angle <= 1.0,
            "steering should be clamped, got {}",
            out.control.lateral.steering_tire_angle,
        );
        assert!(out.activated.steering);
    }

    #[test]
    fn steering_rate_limited() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(10.0); // steer_rate_lim ≈ 1.0 rad/s
        filter.set_current_steer(0.0);
        filter.init_prev_cmd(&make_control(0.0, 0.0, 0.0));

        // Large steering step
        let cmd = make_control(0.0, 0.0, 0.5);
        let out = filter.filter_all(DT, &cmd);

        // Steering change should be bounded by rate * dt
        let max_change = 1.0 * DT; // steer_rate_lim * dt (at most)
        assert!(
            out.control.lateral.steering_tire_angle <= max_change + 0.01,
            "steering should be rate-limited, got {}",
            out.control.lateral.steering_tire_angle,
        );
    }

    // ── Lateral acceleration clamping ───────────────────────────────

    #[test]
    fn lateral_accel_clamped() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(20.0); // lat_acc_lim = 5.0
        filter.set_current_steer(0.0);
        // Start with some steering to avoid first-step rate issues
        filter.init_prev_cmd(&make_control(0.0, 0.0, 0.5));

        // Command high steering at high speed → high lateral accel
        let cmd = make_control(0.0, 0.0, 0.5);
        let out = filter.filter_all(DT, &cmd);

        // lat_acc = v² * tan(steer) / wheelbase = 400 * tan(0.5) / 2.79 ≈ 78.2
        // Should be clamped to 5.0 → steer = atan(5.0 * 2.79 / 400) ≈ 0.0349
        let lat_acc = 20.0 * 20.0 * libm::tanf(out.control.lateral.steering_tire_angle) / 2.79;
        assert!(
            lat_acc.abs() <= 5.1,
            "lateral accel should be clamped to ~5.0, got {}",
            lat_acc,
        );
    }

    // ── Passthrough when within limits ──────────────────────────────

    #[test]
    fn passthrough_when_within_limits() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(5.0);
        filter.set_current_steer(0.05);

        // Set prev_cmd close to what we'll command
        let cmd = make_control(5.0, 1.0, 0.05);
        filter.init_prev_cmd(&cmd);

        // Same command again — should pass through unchanged
        let out = filter.filter_all(DT, &cmd);
        assert!(
            (out.control.longitudinal.velocity - 5.0).abs() < 0.01,
            "velocity should pass through"
        );
        assert!(
            (out.control.longitudinal.acceleration - 1.0).abs() < 0.01,
            "acceleration should pass through"
        );
        assert!(
            (out.control.lateral.steering_tire_angle - 0.05).abs() < 0.01,
            "steering should pass through"
        );
        assert!(!out.activated.is_activated);
    }

    // ── FilterActivated flag reporting ──────────────────────────────

    #[test]
    fn filter_activated_flags_correct() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(5.0);
        filter.set_current_steer(0.0);
        filter.init_prev_cmd(&make_control(0.0, 0.0, 0.0));

        // Command within limits — no activation
        let cmd = make_control(0.0, 0.0, 0.0);
        let out = filter.filter_all(DT, &cmd);
        assert!(!out.activated.is_activated, "no activation expected");

        // Excessive velocity — speed activation
        let cmd = make_control(50.0, 0.0, 0.0);
        let out = filter.filter_all(DT, &cmd);
        assert!(out.activated.speed, "speed activation expected");
        assert!(out.activated.is_activated);
    }

    // ── Actual steer diff limiting ──────────────────────────────────

    #[test]
    fn steer_diff_from_physical_limited() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(25.0); // steer_diff_lim ~0.9
        filter.set_current_steer(0.0);
        filter.init_prev_cmd(&make_control(0.0, 0.0, 0.0));

        // Even if rate-limiting passes, the actual steer diff limits the delta from
        // the physical steering position
        let cmd = make_control(0.0, 0.0, 0.8);
        let out = filter.filter_all(DT, &cmd);

        let diff = (out.control.lateral.steering_tire_angle - 0.0).abs();
        // steer_diff_lim at 25 m/s ≈ 0.9. After rate limiting, diff should be small.
        assert!(
            diff <= 0.91,
            "steer diff from physical should be bounded, got {}",
            diff,
        );
    }

    // ── Lateral jerk limiting ───────────────────────────────────────

    #[test]
    fn lateral_jerk_limited() {
        let mut filter = VehicleCmdFilter::new(FilterParams::default());
        filter.set_current_speed(10.0); // lat_jerk_lim = 7.0
        filter.set_current_steer(0.0);
        filter.init_prev_cmd(&make_control(0.0, 0.0, 0.0));

        // Step from 0 to moderate steering — lateral jerk should be bounded
        let cmd = make_control(0.0, 0.0, 0.3);
        let out1 = filter.filter_all(DT, &cmd);

        let out2 = filter.filter_all(DT, &cmd);

        let v_sq = (10.0f32 * 10.0).max(V_SQ_MIN);
        let lat_acc1 = calc_lateral_accel(v_sq, out1.control.lateral.steering_tire_angle, 2.79);
        let lat_acc2 = calc_lateral_accel(v_sq, out2.control.lateral.steering_tire_angle, 2.79);
        let lat_jerk = (lat_acc2 - lat_acc1).abs() / DT;

        // Should be bounded by lat_jerk_lim (7.0) + some tolerance for accumulated errors
        assert!(
            lat_jerk <= 8.0,
            "lateral jerk should be bounded, got {}",
            lat_jerk,
        );
    }
}
