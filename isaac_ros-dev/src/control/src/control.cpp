#include <rclcpp/rclcpp.hpp>
#include "serialib.hpp"
#include "xbox.hpp"
#include "motor_controller.hpp"
#include "autonomous.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonav_interfaces/msg/encoders.hpp"
#include "autonav_interfaces/srv/configure_control.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <chrono>

#define WHEEL_BASE 0.6858

#define DEBUG_ESTOP

class ControlNode : public rclcpp::Node {

    public:


    ControlNode() 
      : Node("control_node")
       {

        // topic names
        this->declare_parameter("controller_topic", "joy");
        this->declare_parameter("encoder_topic", "encoders");
        this->declare_parameter("path_planning_topic", "cmd_vel");
        // Autonomous turn slowdown. Preserve the controller's commanded
        // yaw rate, but reduce forward progress as |angular.z| rises so
        // the robot does not overrun newly visible hazards during turns.
        // Live-tunable via `ros2 param set /control_node ...`.
        this->declare_parameter("turn_slowdown_enabled", true);
        this->declare_parameter("turn_slowdown_start_angular_rad_s", 0.20);
        this->declare_parameter("turn_slowdown_full_angular_rad_s", 0.65);
        this->declare_parameter("turn_slowdown_min_linear_scale", 0.40);
        // Autonomous motor deadband compensation. Nav2's small nonzero
        // /cmd_vel commands are still valid motion requests, but the
        // RoboteQ + drivetrain can sit below static-friction threshold.
        // Compensation is applied per wheel after cmd_vel->wheel-speed
        // conversion and grade compensation. Exact zero stays exact
        // zero so stop/watchdog behavior is unchanged.
        this->declare_parameter("auto_deadband_comp_enabled", true);
        this->declare_parameter("auto_deadband_min_motor_arg", 6.5);
        this->declare_parameter("auto_deadband_apply_below_motor_arg", 6.5);

        // serial ports test
        this->declare_parameter(
            "motor_port",
            // "/dev/serial/by-id/usb-RoboteQ_RoboteQ_FBLG2360T_HABJAA5QR0E5NDEg_207A34554147-if00"
            "/dev/serial/by-id/usb-RoboteQ_RoboteQ_FBLG2360T_IwBCAAlQMTlZMDIg_2074307C3931-if00");

        this->declare_parameter(
            "arduino_port",
            "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_8583030363935190F141-if00");

        this->declare_parameter("estop_port", "/dev/ttyTHS1");

        // PHASE D — grade compensation (gravity-vector approach). Reads
        // the IMU's body-frame X accelerometer reading (which IS the
        // gravity projection along the robot's longitudinal axis at
        // quasi-static velocities), low-pass filters it, and multiplies
        // the motor command by a clamped multiplier so the 20 lb
        // payload doesn't stall on a 15 % grade going up, and the
        // robot doesn't run away going down. Disabled by default — set
        // `grade_comp_enabled: true` (or `ros2 param set ...`) to turn
        // it on. First test should be on a tilt block with this flag
        // OFF, watch a_fwd in the log to confirm sign and magnitude,
        // then enable.
        this->declare_parameter("imu_topic", "/sick_scansegment_xd/imu_inflated");
        // +1.0 is the correct convention for this stack. imu_cov_inflator
        // rotates the raw SICK IMU into base_link before publishing on
        // /sick_scansegment_xd/imu_inflated, so the body-X accel reading
        // already comes out correctly oriented (nose-up = positive,
        // nose-down = negative). Bench-tested 2026-05-18 on tilt block.
        this->declare_parameter("imu_a_fwd_sign", 1.0);
        this->declare_parameter("grade_comp_enabled", false);
        // Bounded linear map from pitch (deg) to multiplier delta.
        // Mathematically incapable of runaway: input clamped to
        // [-max_deg, +max_deg], output clamped to
        // [1 - max_downhill_pct, 1 + max_uphill_pct].
        //
        //   pitch ≤ -max_deg   →   multiplier = 1 - max_downhill_pct
        //   pitch =  0         →   multiplier = 1.0
        //   pitch ≥ +max_deg   →   multiplier = 1 + max_uphill_pct
        //   linear in between
        //
        // Deadband is small (±0.5°) — just filters IMU noise at level.
        // Pitch is reconstructed from a_fwd via asin(a_fwd/g), with
        // the input clamped so a transient |a_fwd| > g (from real
        // forward acceleration) cannot produce NaN.
        this->declare_parameter("grade_comp_max_deg", 10.0);
        // Default tuned for a 90 lb robot on a 15 % grade: gravity
        // component is ~13.4 lbf, so to maintain cruise the motors
        // need ~2.5x-4x the level throttle (depending on surface).
        // 2.0 (cap = 3.0x multiplier) hits the asphalt-to-grass band.
        // Live-tunable; see node_params.yaml for the full rationale.
        this->declare_parameter("grade_comp_max_uphill_pct", 2.0);
        // Downhill side gets gravity assist, so a moderate damping
        // cap is enough — 0.30 (floor = 0.70x). Tune up if the
        // robot still accelerates past the smoother cap on descent.
        this->declare_parameter("grade_comp_max_downhill_pct", 0.30);
        this->declare_parameter("grade_comp_deadband_deg", 0.5);
        // Ramp-speed safety cap. When the robot detects it's on an
        // incline (|pitch| > deadband), the base linear velocity is
        // clamped to this value BEFORE the multiplier boost. Without
        // this, a 0.75 m/s autonomous base × 3.0x boost would imply
        // 2.25 m/s of motor throttle going uphill — catastrophic if
        // the gravity load doesn't fully balance it. Set to a speed
        // the robot can handle on a ramp regardless of multiplier
        // (the boost still fires, just on the reduced base).
        this->declare_parameter("grade_comp_ramp_max_velocity_mps", 0.30);
        // If no IMU message for this many seconds, multiplier reverts
        // to 1.0 — prevents stale-gravity compensation if the IMU
        // pipeline silently dies mid-mission.
        this->declare_parameter("grade_comp_imu_timeout_sec", 0.5);
        // EWMA alpha on a_fwd. With robot-accel subtraction the
        // remaining signal is mostly gravity, so we can afford a
        // longer time constant for defense in depth: 0.05 ≈ 1 s decay
        // at 50 Hz IMU. Ramp-entry response slows by ~700 ms vs the
        // old 0.2 — well inside the multi-second ramp transit.
        this->declare_parameter("grade_comp_alpha", 0.05);

        // PHASE D — subtract the robot's own forward acceleration from
        // the IMU a_fwd reading before reconstructing pitch. The IMU
        // CANNOT distinguish gravity-projection from chassis
        // acceleration; without subtraction, the multiplier boosts
        // throttle, the boost accelerates the robot, the IMU sees more
        // "tilt", and the loop runs away (oscillation observed
        // 2026-05-18 on the bench). Robot a_fwd is computed from
        // encoder counts at the 30 ms control tick — same wheel
        // constants as wheel_odom_pub.
        this->declare_parameter("grade_comp_robot_accel_subtract", true);
        // EWMA alpha on the encoder-derived robot acceleration. 0.2 is
        // moderate smoothing: the derivative of velocity is noisy, but
        // we want enough rate to track the acceleration spike that's
        // causing the oscillation. Increase (toward 1.0) for more
        // responsive subtraction; decrease for smoother but laggier.
        this->declare_parameter("grade_comp_robot_accel_alpha", 0.2);

        configure_server = this->create_service<autonav_interfaces::srv::ConfigureControl>
             ("configure_control", std::bind(&ControlNode::configure, this, std::placeholders::_1, std::placeholders::_2));

    }

    serialib arduinoSerial;
    Xbox controller;
    MotorController motors;
    Autonomous currPose;

    private:
    bool autonomousMode = false;
    // Cached AUTO wheel-speed state. Zero-init matters: without it the
    // first AUTO tick after node startup would write uninitialized
    // memory into motors.move(); with it, AUTO-on with no /cmd_vel
    // yet arrived holds the wheels at rest.
    float linear_move = 0.0f;
    float angular_move = 0.0f;
    float left_wheel_speed = 0.0f;
    float right_wheel_speed = 0.0f;

    // AUTO /cmd_vel safety constants. The manual branch already has a
    // 0.5 s /joy watchdog (see publish_encoder_data); these are the
    // AUTO-side analogue plus a transition grace window.
    //
    //   kCmdVelStaleSec — wall-clock budget. If no /cmd_vel arrives
    //     within this many seconds the AUTO branch zeroes motors
    //     instead of re-applying the latched wheel-speed values.
    //     Slightly under velocity_smoother's 1.0 s velocity_timeout
    //     so we react before the smoother's own ramp-down would.
    //
    //   kAutoOnGraceSec — held-at-zero window after the AUTO rising
    //     edge. velocity_smoother runs OPEN_LOOP at 20 Hz; on AUTO-on
    //     it can deliver a stale-content / fresh-timestamp tick within
    //     ~50 ms, which used to translate straight into a lurch. The
    //     grace window eats that tail; a freshly-dispatched goal
    //     simply starts ~200 ms later — below operator perception.
    static constexpr double kCmdVelStaleSec = 0.35;
    static constexpr double kAutoOnGraceSec = 0.20;

    // subscription for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controllerSub;
    //configuration server
    rclcpp::Service<autonav_interfaces::srv::ConfigureControl>::SharedPtr configure_server;

    // publisher for encoder values
     rclcpp::Publisher<autonav_interfaces::msg::Encoders>::SharedPtr encodersPub;
     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autonomous_mode_pub_;
     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr estop_sub_;
     rclcpp::TimerBase::SharedPtr encoder_timer_;

    // rclcpp::TimerBase::SharedPtr joy_timer_;
    // subscription for Nav2 pose
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pathPlanningSub;

    // Speed publisher for DAQ logging
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_pub_;

    // Direct manual-speed setpoint from the HUD's Manual Drive mode
    // (1-9 keys → 5/10/15/.../45). Only honored when !autonomousMode
    // so a stray publish can't muck with AUTO driving.
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr manualSpeedSub;

    // Debounce for bumper speed changes (500ms between changes)
    std::chrono::steady_clock::time_point last_speed_change_time_{};
    // Motor-write rate gate. The /joy callback fires at 100 Hz with
    // joy_node autorepeat, and unthrottled motor writes were sharing
    // the same RoboteQ serial line with the 30 Hz encoder reads —
    // the high write rate corrupted encoder responses (we saw
    // /encoders publishing concatenated/truncated garbage that the
    // wheel_odom MAX_ENCODER_DELTA filter then rejected, leaving
    // /odom stuck). Cap motor writes at 33 Hz so the encoder reader
    // gets clean intervals on the bus.
    std::chrono::steady_clock::time_point last_motor_send_time_{};

    // /joy watchdog: timestamp of last received /joy. Initialized to
    // epoch so the watchdog also fires before the first /joy arrives.
    rclcpp::Time last_joy_msg_time_{0, 0, RCL_ROS_TIME};

    // /cmd_vel watchdog (AUTO branch only). Stamped on every
    // path_planning_callback. Reset to epoch on the AUTO rising edge
    // so a stale-but-fresh-timestamp message from velocity_smoother
    // can't slip past kAutoOnGraceSec and immediately drive motors.
    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};

    // AUTO rising-edge grace deadline. Set to (now + kAutoOnGraceSec)
    // inside joystick_callback when autonomousMode flips true. While
    // now() < auto_on_grace_until_, publish_encoder_data() forces
    // motors.move(0, 0) regardless of what /cmd_vel says.
    rclcpp::Time auto_on_grace_until_{0, 0, RCL_ROS_TIME};

    bool currX = false;
    bool prevX = false;

    // Y button rising-edge -> clear accumulated marks in the global
    // costmap's LocalMirrorLayer within the robot's current local-
    // costmap footprint. Fire-and-forget publish on /local_mirror_layer
    // /clear; the layer subscribes there, zeroes its accumulator inside
    // the local footprint on its next update cycle, and the same cycle
    // re-stamps live obstacles on top — so smears behind the robot
    // disappear without losing persistent map further away.
    //
    // Index 4 is empirically what the physical Y button fires on this
    // controller/Bluetooth-mount combination (17-element SDL2-style
    // /joy array). It does NOT match the xpad layout where Y=2 and
    // does NOT match the swap-to-3 fallback. If the mount layout
    // changes again, sniff /joy --field buttons while holding Y and
    // update kY_button_index_.
    static constexpr int kY_button_index_ = 4;
    bool currY_clear = false;
    bool prevY_clear = false;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_local_mirror_pub_;

    // PHASE D — gravity-vector grade compensation state.
    double latest_a_fwd_ = 0.0;             // EWMA-filtered body-X accel.
    bool have_imu_ = false;
    rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;

    // PHASE D — robot self-acceleration estimate, derived from encoder
    // counts at the 30 ms control tick. Subtracted from IMU a_fwd in
    // imu_callback so the EWMA tracks gravity projection only. Wheel
    // constants mirror wheel_odom_pub.cpp:25-37 — keep in sync.
    int prev_left_enc_count_ = 0;
    int prev_right_enc_count_ = 0;
    rclcpp::Time prev_enc_time_{0, 0, RCL_ROS_TIME};
    double latest_v_fwd_robot_ = 0.0;       // raw most-recent v_fwd (m/s).
    double latest_a_robot_fwd_ = 0.0;       // EWMA-filtered da/dt (m/s²).
    bool have_robot_kinematics_ = false;

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        last_joy_msg_time_ = this->now();
        currX = joy_msg->buttons[3];

        // Detect rising edge
        if (!prevX && currX) {
            autonomousMode = !autonomousMode;

            if (autonomousMode) {
                char mode[12] = "AUTONOMOUS\n";
                arduinoSerial.writeString(mode);
            } else {
                char mode[8] = "MANUAL\n";
                arduinoSerial.writeString(mode);
            }

            // Clear cached wheel-speed state on EVERY transition so the
            // AUTO branch can't re-apply a value latched from a
            // previous AUTO session. On the AUTO-ON transition arm the
            // grace window and reset last_cmd_vel_time_ to epoch — the
            // staleness watchdog will hold motors at zero until a
            // genuinely fresh /cmd_vel arrives AFTER the grace window.
            linear_move = 0.0f;
            angular_move = 0.0f;
            left_wheel_speed = 0.0f;
            right_wheel_speed = 0.0f;
            if (autonomousMode) {
                auto_on_grace_until_ = this->now() +
                    rclcpp::Duration::from_seconds(kAutoOnGraceSec);
                last_cmd_vel_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            }

            // Publish mode so NAV2 / GPS waypoint handler know the state
            if (autonomous_mode_pub_) {
                std_msgs::msg::Bool mode_msg;
                mode_msg.data = autonomousMode;
                autonomous_mode_pub_->publish(mode_msg);
            }
        }

        prevX = currX;

        // Y rising-edge -> publish on /local_mirror_layer/clear. Fires
        // in both AUTO and MANUAL modes since the costmap state is
        // shared. Pub-and-forget — the layer's subscription handles it
        // on its own update cycle without blocking the joy callback.
        int curr_y_btn = (static_cast<int>(joy_msg->buttons.size()) > kY_button_index_)
            ? joy_msg->buttons[kY_button_index_] : 0;
        currY_clear = curr_y_btn != 0;
        if (!prevY_clear && currY_clear && clear_local_mirror_pub_) {
            clear_local_mirror_pub_->publish(std_msgs::msg::Empty());
            RCLCPP_INFO(this->get_logger(),
                "Y pressed -> /local_mirror_layer/clear");
        }
        prevY_clear = currY_clear;

        if(!autonomousMode){
            controller.set_b(joy_msg->buttons[1]);
            controller.set_x(joy_msg->buttons[3]);
            controller.set_y(joy_msg->buttons[2]);

            // Bumpers are not in a stable position across the controllers
            // / xpad-driver combinations we've seen on this Jetson: live
            // /joy diagnostics captured presses at indices {6, 7} in the
            // normal mount and {9, 10} in the wrong-container-mount
            // layout. The mapping pairs right=6/9 and left=7/10. Rather
            // than pin to one layout and silently break on the next
            // swap, OR both index pairs. Trade-off: on a layout where
            // 9/10 are stick clicks instead of bumpers, clicking a
            // stick will also nudge the speed — non-critical UX cost
            // vs. the bumpers becoming inert.
            auto safe_btn = [&joy_msg](size_t i) -> int {
                return i < joy_msg->buttons.size() ? joy_msg->buttons[i] : 0;
            };
            controller.set_right_bumper(safe_btn(6) || safe_btn(9));
            controller.set_left_bumper(safe_btn(7) || safe_btn(10));

            controller.set_left_stick_x(joy_msg->axes[0]);
            controller.set_left_stick_y(joy_msg->axes[1]);
            controller.set_right_stick_x(joy_msg->axes[2]);
            controller.set_right_stick_y(joy_msg->axes[3]);

            // Drive the motors directly off the /joy callback. Previously
            // manual commands fired only from the 30 ms encoder timer
            // which serialized motor writes behind two blocking RoboteQ
            // encoder reads (up to 20 ms each tick), giving the stick
            // 30-50 ms of perceived lag. Autonomy path is untouched —
            // cmd_vel still feeds publish_encoder_data() via
            // path_planning_callback below.
            apply_manual_command();
        }
    }

    // Manual-mode motor dispatch. Single-threaded executor guarantees
    // this never races with publish_encoder_data() or path_planning_callback,
    // so no locking on motor serial is needed.
    void apply_manual_command() {
        if (autonomousMode) return;

        // Bumper hold ramps the speed setpoint at 5 Hz (75 steps × 200 ms
        // = 15 s from 0 to max). With autorepeat=100 Hz this cadence is
        // audibly musical on the bench, but operator feedback (2026-05-18)
        // is that the responsive ramp is worth the noise — the 30 s ramp
        // at 400 ms cooldown felt sluggish in actual driving.
        constexpr long kBumperCooldownMs = 200;
        // Manual-mode speed is unitless gear (motors.move multiplies by
        // stepSize=10 internally). 0 = stopped at any stick, 75 ≈ 7.5x
        // the level-cruise baseline (which is 10). Hard upper bound
        // prevents the bumper from being held into runaway throttle;
        // hard lower bound prevents accidentally going negative (which
        // motors.move() would interpret as reverse direction, decoupled
        // from stick sign).
        constexpr int kSpeedMin = 0;
        constexpr int kSpeedMax = 75;

        Xbox::CommandData command = controller.calculateCommand();

        // STOP preempts. motors.shutdown() closes the motor serial,
        // so no wheel update follows.
        if (command.cmd == Xbox::STOP) {
            motors.shutdown();
            return;
        }

        // Bumpers ramp the speed setpoint as a side effect. Stick
        // values in `command` are still driven into the wheels below,
        // so a bumper-held stick-deflected case both bumps speed AND
        // updates wheel velocity in real time on the very next
        // motors.move() call (next /joy tick, ~10 ms away).
        if (command.cmd == Xbox::SPEED_UP || command.cmd == Xbox::SPEED_DOWN) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_speed_change_time_).count();
            if (elapsed >= kBumperCooldownMs) {
                const int delta = (command.cmd == Xbox::SPEED_UP) ? +1 : -1;
                const int new_speed = std::clamp(
                    motors.getSpeed() + delta, kSpeedMin, kSpeedMax);
                if (new_speed != motors.getSpeed()) {
                    motors.setSpeed(new_speed);
                    last_speed_change_time_ = now;
                    RCLCPP_INFO(this->get_logger(),
                        "speed %s new speed: %d",
                        delta > 0 ? "up!" : "down.", new_speed);
                }
                // At the bound: no-op, don't reset cooldown, don't log
                // — keeps the held-at-max bumper silent.
            }
        }

        // Always drive the wheels from the stick portion of `command`.
        //   Xbox::NONE  -> left/right_motor_speed are 0  -> motors.move(0,0)
        //                  zeros wheels and resets the RoboteQ
        //                  command-watchdog so we don't get the ~1 s
        //                  coast-then-zero behavior on stick release.
        //   Xbox::MOVE  -> stick values scaled by current speed gear.
        //   SPEED_UP/DOWN -> calculateCommand still populates motor_speeds
        //                    from sticks when bumpers are held, so wheels
        //                    track the just-updated speed gear in real time.
        // PHASE D grade compensation applies in both manual and auto.
        //
        // Hard 33 Hz cap on motor writes regardless of /joy rate. The
        // 100 Hz autorepeat path was saturating the RoboteQ serial
        // line and corrupting concurrent encoder reads; rate gate
        // restores clean read windows. Stick → motor latency goes
        // back to up to 30 ms (matching the pre-fix/lines behaviour)
        // but the encoder pipeline is what actually works.
        constexpr long kMotorSendIntervalMs = 30;
        const auto now_steady = std::chrono::steady_clock::now();
        const auto since_last_send_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now_steady - last_motor_send_time_).count();
        if (since_last_send_ms < kMotorSendIntervalMs) {
            return;
        }
        last_motor_send_time_ = now_steady;
        const double forward_cmd_manual = 0.5 * (
            command.left_motor_speed + command.right_motor_speed);
        const double mult_manual = grade_speed_multiplier(forward_cmd_manual);
        motors.move(
            command.right_motor_speed * motors.getSpeed() * mult_manual,
            command.left_motor_speed * motors.getSpeed() * mult_manual);
    }

    // Direct manual-speed setpoint from /manual_speed_setpoint. The
    // HUD's Manual Drive mode (1-9 keys) publishes a target gear here
    // so the operator can snap to a known speed instead of waiting on
    // the bumper ramp (200 ms per +1, ~9 s 0→45). Ignored under AUTO
    // so a stray publish can't perturb nav2 driving.
    void manual_speed_callback(const std_msgs::msg::Int32::SharedPtr msg){
        if (autonomousMode) return;
        constexpr int kSpeedMin = 0;
        constexpr int kSpeedMax = 75;
        const int target = std::clamp(static_cast<int>(msg->data),
                                      kSpeedMin, kSpeedMax);
        if (target != motors.getSpeed()) {
            motors.setSpeed(target);
            RCLCPP_INFO(this->get_logger(),
                "manual speed setpoint: %d", target);
        }
    }

    void estop_callback(const std_msgs::msg::String::SharedPtr msg){


	   
            
	    std::string incoming = msg->data;

            
            if (incoming.empty()) {
	       #ifdef DEBUG_ESTOP
	       RCLCPP_INFO(this->get_logger(), "incoming string empty");
	       #endif

                return;

            }

            #ifdef DEBUG_ESTOP
            RCLCPP_INFO(this->get_logger(), "incoming string: %s", incoming.c_str());
            #endif

            if (incoming == "STOP") {
                RCLCPP_WARN(this->get_logger(), "ESTOP PRESSED: MOTORS SHUTTING DOWN");
                motors.shutdown();
            }

    }

    // PHASE D — encoder-derived robot self-acceleration. Runs every
    // 30 ms from publish_encoder_data() after the two RoboteQ encoder
    // reads complete. Computes instantaneous forward velocity from the
    // wheel deltas using the same constants as wheel_odom_pub
    // (wheel_radius, ticks_per_rev, left_encoder_scale — keep in sync
    // or our subtraction won't match the IMU's actual sensed accel).
    // Then differentiates v_fwd into a_fwd and EWMA-smooths.
    //
    // The sign convention matches wheel_odom_pub: left raw counts are
    // negated before being used (the left motor is mounted reversed
    // such that forward motion gives negative raw counts).
    void update_robot_kinematics(int left_count, int right_count) {
        // OSCILLATION-SENSITIVE — these constants must match
        // wheel_odom_pub.cpp:25-37 or the subtracted robot-accel
        // estimate will be biased and Phase D will still oscillate
        // (just at a different gain). DO NOT change without also
        // updating wheel_odom_pub.
        constexpr double kWheelRadiusM = 0.12946;
        constexpr double kTicksPerRev = 81923.0;
        constexpr double kLeftEncoderScale = 1.0 / 1.016335;
        constexpr int kMaxEncoderDelta = 20000;

        const rclcpp::Time now = this->now();
        if (!have_robot_kinematics_) {
            prev_left_enc_count_ = left_count;
            prev_right_enc_count_ = right_count;
            prev_enc_time_ = now;
            have_robot_kinematics_ = true;
            return;
        }

        const double dt = (now - prev_enc_time_).seconds();
        if (dt <= 1e-3 || dt > 0.5) {
            // Timer hiccup or first-tick after long pause — reset
            // baseline rather than emit a wild derivative.
            prev_left_enc_count_ = left_count;
            prev_right_enc_count_ = right_count;
            prev_enc_time_ = now;
            return;
        }

        // wheel_odom_pub negates left counts before differencing; we
        // mirror that so the "forward = both wheels positive
        // displacement" convention holds.
        const int left_delta_for_motion = -(left_count - prev_left_enc_count_);
        const int right_delta_for_motion = right_count - prev_right_enc_count_;

        if (std::abs(left_delta_for_motion) > kMaxEncoderDelta ||
            std::abs(right_delta_for_motion) > kMaxEncoderDelta) {
            // Single-tick glitch — same guard as wheel_odom_pub
            // (cpp:68-73). Reset baseline; emit nothing.
            prev_left_enc_count_ = left_count;
            prev_right_enc_count_ = right_count;
            prev_enc_time_ = now;
            return;
        }

        const double left_disp_m =
            (2.0 * M_PI * kWheelRadiusM) *
            (left_delta_for_motion / kTicksPerRev) *
            kLeftEncoderScale;
        const double right_disp_m =
            (2.0 * M_PI * kWheelRadiusM) *
            (right_delta_for_motion / kTicksPerRev);
        const double v_inst = 0.5 * (left_disp_m + right_disp_m) / dt;

        const double a_inst = (v_inst - latest_v_fwd_robot_) / dt;
        const double alpha =
            this->get_parameter("grade_comp_robot_accel_alpha").as_double();
        if (std::isfinite(a_inst)) {
            latest_a_robot_fwd_ = alpha * a_inst + (1.0 - alpha) * latest_a_robot_fwd_;
        }
        latest_v_fwd_robot_ = v_inst;

        prev_left_enc_count_ = left_count;
        prev_right_enc_count_ = right_count;
        prev_enc_time_ = now;
    }

    // PHASE D — gravity-vector callback. Body-frame X accel reading
    // contains both linear acceleration and the projection of gravity
    // along base_link's forward axis. At quasi-static velocities (or
    // when filtered) the gravity projection dominates: a_fwd = g*sin(pitch)
    // for a nose-up slope. With robot-accel subtraction enabled, the
    // chassis a_fwd component is removed before EWMA so the filter
    // tracks gravity only — breaks the boost→accel→more-boost positive
    // feedback loop that produced oscillation 2026-05-18.
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_stamp_ = this->now();
        const double sign = this->get_parameter("imu_a_fwd_sign").as_double();
        const double alpha = this->get_parameter("grade_comp_alpha").as_double();
        double raw_ax = sign * msg->linear_acceleration.x;

        // Subtract the encoder-derived robot self-acceleration. Both
        // signals are in m/s² with forward-positive convention (the
        // IMU after imu_a_fwd_sign normalization, the encoders by
        // construction in update_robot_kinematics).
        if (have_robot_kinematics_ &&
            this->get_parameter("grade_comp_robot_accel_subtract").as_bool() &&
            std::isfinite(latest_a_robot_fwd_)) {
            raw_ax -= latest_a_robot_fwd_;
        }

        if (!have_imu_) {
            latest_a_fwd_ = raw_ax;
            have_imu_ = true;
        } else {
            latest_a_fwd_ = alpha * raw_ax + (1.0 - alpha) * latest_a_fwd_;
        }
    }

    // Reconstruct body-pitch (deg) from the EWMA-filtered a_fwd. The
    // a_fwd/g ratio is clamped to [-1, 1] BEFORE asin so a transient
    // |a_fwd| > g (forward-acceleration spike) returns ±90 deg instead
    // of NaN. Returns 0 if no IMU data or input is non-finite.
    double current_pitch_deg() {
        if (!have_imu_) return 0.0;
        if (!std::isfinite(latest_a_fwd_)) return 0.0;
        constexpr double g = 9.81;
        const double sin_pitch = std::clamp(latest_a_fwd_ / g, -1.0, 1.0);
        return std::asin(sin_pitch) * 180.0 / M_PI;
    }

    // PHASE D — bounded, direction-aware multiplier. Returns 1.0 when:
    //   - feature disabled via param
    //   - IMU silent (timeout)
    //   - |forward_command| < eps (pure rotation, no translation)
    //   - a_fwd is not finite (NaN/inf guard)
    //   - |pitch| inside deadband (level ground)
    //
    // Otherwise: compensation depends on whether motion is AGAINST
    // gravity or WITH it. Sign of (forward_command × pitch_deg) tells us:
    //   pitch>0 (nose-up) and fwd>0: forward uphill   → AGAINST → boost
    //   pitch>0 (nose-up) and fwd<0: reverse uphill   → WITH    → damp
    //   pitch<0 (nose-down) and fwd>0: forward downhill → WITH    → damp
    //   pitch<0 (nose-down) and fwd<0: reverse downhill → AGAINST → boost
    //
    // This prevents the failure mode where backing down a hill (BackUp
    // recovery, for example) doesn't get gravity-damped and the robot
    // accelerates downhill in reverse. Multiplier magnitude is set by
    // |pitch| × the appropriate boost or damp pct, bounded:
    //   multiplier ∈ [1 - max_downhill_pct, 1 + max_uphill_pct]
    // With defaults (2.0, 0.30): multiplier ∈ [0.70, 3.00] for ANY
    // IMU + motion combination.
    double grade_speed_multiplier(double forward_command) {
        if (!this->get_parameter("grade_comp_enabled").as_bool()) return 1.0;
        if (!have_imu_) return 1.0;
        const double timeout = this->get_parameter("grade_comp_imu_timeout_sec").as_double();
        if ((this->now() - last_imu_stamp_).seconds() > timeout) return 1.0;
        if (std::abs(forward_command) < 1e-6) return 1.0;
        if (!std::isfinite(latest_a_fwd_)) return 1.0;

        const double pitch_deg = current_pitch_deg();
        const double deadband_deg =
            this->get_parameter("grade_comp_deadband_deg").as_double();
        if (std::abs(pitch_deg) < deadband_deg) return 1.0;

        const double max_deg = this->get_parameter("grade_comp_max_deg").as_double();
        const double max_up_pct =
            this->get_parameter("grade_comp_max_uphill_pct").as_double();
        const double max_dn_pct =
            this->get_parameter("grade_comp_max_downhill_pct").as_double();

        // |pitch| / max_deg, clamped to [0, 1]. Magnitude-only.
        const double t = std::min(1.0, std::abs(pitch_deg) / max_deg);

        // Sign of (pitch × forward) tells us motion direction relative
        // to gravity. Positive product means BOTH pitch and motion have
        // the same sign — which is "motion against gravity" (forward
        // uphill or reverse downhill). Negative product means motion
        // with gravity (forward downhill or reverse uphill).
        const bool against_gravity = (pitch_deg * forward_command) > 0.0;

        if (against_gravity) {
            return 1.0 + max_up_pct * t;
        } else {
            return 1.0 - max_dn_pct * t;
        }
    }

    double compensate_auto_deadband(double motor_arg) {
        if (!std::isfinite(motor_arg)) return 0.0;
        if (!this->get_parameter("auto_deadband_comp_enabled").as_bool()) {
            return motor_arg;
        }

        const double abs_arg = std::abs(motor_arg);
        if (abs_arg < 1e-6) return 0.0;

        const double min_arg = std::max(
            0.0,
            this->get_parameter("auto_deadband_min_motor_arg").as_double());
        const double apply_below = std::max(
            0.0,
            this->get_parameter("auto_deadband_apply_below_motor_arg").as_double());
        if (min_arg <= 0.0 || apply_below <= 0.0 || abs_arg >= apply_below) {
            return motor_arg;
        }

        return std::copysign(std::max(abs_arg, min_arg), motor_arg);
    }


    void publish_encoder_data() {
        autonav_interfaces::msg::Encoders encoder_msg;
        encoder_msg.left_motor_rpm = 0;
        encoder_msg.right_motor_rpm = 0;
        // Both counts negated 2026-10-12: the Roboteq encoders were set to
        // negative PPR (EPPR -20481) so encoder speed matches command polarity
        // for closed-loop speed mode, which also inverts the raw ?C counters.
        // Negating here restores the count convention that wheel_odom_pub and
        // update_robot_kinematics were validated against (2026-10-05, vs IMU).
        const int left_count_raw = -motors.getLeftEncoderCount();
        const int right_count_raw = -motors.getRightEncoderCount();
        encoder_msg.left_motor_count = left_count_raw;
        encoder_msg.right_motor_count = right_count_raw;
        //RCLCPP_INFO(this->get_logger(), "LEC: %s", motors.getLeftEncoderCount());

        // PHASE D — update robot self-acceleration estimate so the
        // imu_callback can subtract it before EWMA. Runs every 30 ms
        // here, IMU fires at 50 Hz; latest_a_robot_fwd_ is always
        // within 30 ms of fresh, which is well under the IMU EWMA
        // time constant.
        update_robot_kinematics(left_count_raw, right_count_raw);


        std::string arduinoEncoderCounts = "L:";
        arduinoEncoderCounts += encoder_msg.left_motor_count;
        arduinoEncoderCounts += " R:";
        arduinoEncoderCounts += encoder_msg.right_motor_count;



        if(!autonomousMode){
            // Joy watchdog ONLY. Manual motor commands now flow from
            // joystick_callback at /joy arrival rate (~100 Hz with the
            // tuned joy_node), so this timer's job in manual mode is
            // just to zero the motors if /joy goes silent — the
            // joystick_callback won't fire to do it itself.
            if ((this->now() - last_joy_msg_time_).seconds() > 0.5) {
                motors.move(0, 0);
            }
        }
        else {
           // AUTO safety gates BEFORE applying the latched wheel-speed
           // values. Two independent ways to land at motors.move(0, 0):
           //
           //   1. Grace window. For kAutoOnGraceSec after the AUTO
           //      rising edge, hold the wheels at rest even if a fresh
           //      /cmd_vel arrives. velocity_smoother (OPEN_LOOP,
           //      smoothing_frequency=20 Hz, velocity_timeout=1.0 s)
           //      can deliver a fresh-timestamp / stale-content tick
           //      within ~50 ms of AUTO-on; the grace window eats it.
           //
           //   2. Staleness watchdog. If /cmd_vel hasn't arrived
           //      within kCmdVelStaleSec, stop. Symmetric to the
           //      manual-mode 0.5 s /joy watchdog above. Without it,
           //      the AUTO branch would re-apply the last
           //      left/right_wheel_speed every 30 ms forever — so a
           //      nav2 crash mid-mission leaves the robot driving
           //      blind at its last commanded velocity.
           //
           // Both gates also handle the AUTO-on case where /cmd_vel
           // has never been received this session: last_cmd_vel_time_
           // is at epoch, so (now - epoch) >> kCmdVelStaleSec.
           const rclcpp::Time now_t = this->now();
           const bool in_grace = now_t < auto_on_grace_until_;
           const bool cmd_stale =
               (now_t - last_cmd_vel_time_).seconds() > kCmdVelStaleSec;
           if (in_grace || cmd_stale) {
               motors.move(0, 0);
           } else {
               // PHASE D — apply gravity-vector grade compensation. Forward
               // command = average of left/right wheel speeds (positive means
               // forward intent). The multiplier is 1.0 unless climbing or
               // descending past the deadband; clamped to safe motor-current
               // bounds by grade_comp_max_uphill_multiplier /
               // grade_comp_min_downhill_multiplier.
               const double forward_cmd = 0.5 * (left_wheel_speed + right_wheel_speed);
               const double mult = grade_speed_multiplier(forward_cmd);
               const double right_motor_arg =
                   compensate_auto_deadband(right_wheel_speed * 40 * mult);
               const double left_motor_arg =
                   compensate_auto_deadband(left_wheel_speed * 40 * mult);
               motors.move(right_motor_arg, left_motor_arg);
           }
        }

        arduinoEncoderCounts += "\n";
        // arduinoSerial.writeString(arduinoEncoderCounts.c_str());

        if (encodersPub) {
            encodersPub->publish(encoder_msg);
        }

        // Publish current speed for DAQ logging
        if (speed_pub_) {
            auto speed_msg = std_msgs::msg::String();
            speed_msg.data = std::to_string(motors.getSpeed());
            speed_pub_->publish(speed_msg);
        }
    }

    void path_planning_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        // Stamp the AUTO-side watchdog on every arrival, regardless of
        // mode — guarantees that the first /cmd_vel after AUTO-on
        // refreshes the staleness clock even if the message races the
        // mode flip by a few microseconds.
        last_cmd_vel_time_ = this->now();

        if (autonomousMode) {
            /*Move to Pose Way*/
            // currPose.linearX = msg->linear.x;
            // currPose.linearY = msg->linear.y;
            //currPose.linearZ = msg->linear.z;

            // currPose.angularZ = msg->angular.z;

            // currPose.goToPose(linearX, linearY, motors);

            /*Direct Speed to Motor Way*/

            linear_move = msg->linear.x;
            angular_move = msg->angular.z;

            // PHASE D safety — when grade comp is enabled AND the robot
            // is detected on an incline (|pitch| past deadband), cap the
            // base linear velocity at grade_comp_ramp_max_velocity_mps
            // BEFORE the grade-comp multiplier amplifies it. std::clamp
            // only caps the upper bound — slower commands pass through
            // unchanged, so the planner can still throttle down on the
            // ramp. The multiplier still fires on the reduced base so
            // the boost compensates for the gravity load.
            if (this->get_parameter("grade_comp_enabled").as_bool() && have_imu_) {
                const double timeout =
                    this->get_parameter("grade_comp_imu_timeout_sec").as_double();
                if ((this->now() - last_imu_stamp_).seconds() <= timeout) {
                    const double pitch_deg = current_pitch_deg();
                    const double deadband_deg =
                        this->get_parameter("grade_comp_deadband_deg").as_double();
                    if (std::abs(pitch_deg) > deadband_deg) {
                        const double ramp_max =
                            this->get_parameter("grade_comp_ramp_max_velocity_mps").as_double();
                        linear_move = std::clamp<float>(
                            linear_move,
                            static_cast<float>(-ramp_max),
                            static_cast<float>(ramp_max));
                    }
                }
            }

            // Reduce forward progress during harder turns while keeping
            // the commanded yaw rate untouched. Only applies to
            // forward-driving autonomous commands; reverse and rotate-
            // in-place behavior are intentionally left alone.
            if (this->get_parameter("turn_slowdown_enabled").as_bool() &&
                linear_move > 0.0f) {
                const double start_angular = std::max(
                    0.0,
                    this->get_parameter("turn_slowdown_start_angular_rad_s").as_double());
                const double full_angular = std::max(
                    start_angular,
                    this->get_parameter("turn_slowdown_full_angular_rad_s").as_double());
                const double min_linear_scale = std::clamp(
                    this->get_parameter("turn_slowdown_min_linear_scale").as_double(),
                    0.0,
                    1.0);
                const double abs_angular = std::abs(static_cast<double>(angular_move));

                double linear_scale = 1.0;
                if (abs_angular > start_angular) {
                    if (full_angular <= start_angular) {
                        linear_scale = min_linear_scale;
                    } else {
                        const double blend = std::clamp(
                            (abs_angular - start_angular) / (full_angular - start_angular),
                            0.0,
                            1.0);
                        linear_scale = 1.0 + blend * (min_linear_scale - 1.0);
                    }
                }

                linear_move = static_cast<float>(
                    static_cast<double>(linear_move) * linear_scale);
            }

            left_wheel_speed = linear_move - ( angular_move * (WHEEL_BASE/2));
            right_wheel_speed = linear_move +( angular_move * (WHEEL_BASE/2));

        }
    }


    void init_serial_arduino(const char * arduino_port) {

        char ret;
        ret = arduinoSerial.openDevice(arduino_port, 9600);

        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Arduino serial error: %s", arduinoSerial.error_map.at(ret).c_str());

        }
	else {
	    RCLCPP_INFO(this->get_logger(), "arduino connection success!");
	}


        char mode[8] = "MANUAL\n";
        arduinoSerial.writeString(mode);
    }



    void configure(const std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Request> request, 
                         std::shared_ptr<autonav_interfaces::srv::ConfigureControl::Response> response) {


        // configure serial
        std::string motor_port  = this->get_parameter("motor_port").as_string();
        std::string arduino_port = this->get_parameter("arduino_port").as_string();
        std::string estop_port = this->get_parameter("estop_port").as_string();


        if (request->arduino) {
            init_serial_arduino(arduino_port.c_str());
        }
        if (request->motors) {
            char ret = motors.configure(motor_port.c_str());
            if (ret != 1) {
                // RCLCPP_ERROR(this->get_logger(), "Motor serial error: %s", .error_map.at(ret).c_str());
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Motor serial connection success!");
            }

        }

	    
        estop_sub_ = this->create_subscription<std_msgs::msg::String>("/estop", 10, std::bind(&ControlNode::estop_callback, this, std::placeholders::_1));

        manualSpeedSub = this->create_subscription<std_msgs::msg::Int32>(
            "/manual_speed_setpoint", 10,
            std::bind(&ControlNode::manual_speed_callback, this, std::placeholders::_1));


        std::string leftMotorCommand = "!C 1 0\r";
        std::string rightMotorCommand = "!C 2 0 \r";
        motors.motorSerial.writeString(leftMotorCommand.c_str());
        motors.motorSerial.writeString(rightMotorCommand.c_str());

        // configure topics
        std::string controller_topic = this->get_parameter("controller_topic").as_string();
        std::string encoder_topic = this->get_parameter("encoder_topic").as_string();
        std::string path_planning_topic = "cmd_vel";

        //XBOX SUB
        controllerSub = this->create_subscription<sensor_msgs::msg::Joy>(
            controller_topic, 10, std::bind(&ControlNode::joystick_callback, this, std::placeholders::_1));

        // Publisher backing the Y-button costmap clear. Subscribed by
        // LocalMirrorLayer on the global_costmap node, which zeroes
        // accumulated cells inside the current local-costmap footprint
        // on its next update cycle. QoS depth 1 reliable matches the
        // layer's subscription.
        clear_local_mirror_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/local_mirror_layer/clear", rclcpp::QoS(1).reliable());

        // PHASE D — IMU subscription for gravity-vector grade compensation.
        // imu_topic param defaults to /sick_scansegment_xd/imu_inflated
        // (already on the bus for ekf_local). MUST be BEST_EFFORT QoS —
        // imu_cov_inflator publishes BEST_EFFORT (standard for high-rate
        // sensor data), and a RELIABLE subscription gets the "incompatible
        // QoS, no messages will be received" warning and silently drops
        // everything. rclcpp::SensorDataQoS() is the standard preset
        // (depth 5, best_effort) for exactly this.
        const std::string imu_topic =
            this->get_parameter("imu_topic").as_string();
        imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&ControlNode::imu_callback, this, std::placeholders::_1));

        // ESTOP CALLBACK
	
        
        // Publish autonomous mode state so NAV2 / GPS waypoint handler can react
        autonomous_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/autonomous_mode", 10);
        {
            std_msgs::msg::Bool mode_msg;
            mode_msg.data = autonomousMode;
            autonomous_mode_pub_->publish(mode_msg);
        }

        // ----- HARD GUARD FOR ENCODER PUBLISHER -----
        auto existing_pubs = this->get_publishers_info_by_topic(encoder_topic, false);

        if (!existing_pubs.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Another node already publishes %s. Skipping encoder publisher, but motor control timer still active.",
                encoder_topic.c_str());
        } else {
            // NAVIGATION ENCODER PUB
            encodersPub = this->create_publisher<autonav_interfaces::msg::Encoders>(encoder_topic, 10);
        }

        // Always create the control timer — motor commands and speed changes depend on it
        encoder_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ControlNode::publish_encoder_data, this));
        // ----- END HARD GUARD -----

        // Speed publisher for DAQ logging
        speed_pub_ = this->create_publisher<std_msgs::msg::String>("/motor_speed", 10);

       /* joy_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControlNode::joy_timer_callback, this)
        );*/
        //GPS PUB
        //PATH PLANNING SUB
        pathPlanningSub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ControlNode::path_planning_callback, this, std::placeholders::_1));


        response->ret = 0;
    }
};


int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;

}
