

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <stdint.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include <algorithm>

#include "px4_ros_com/action/track_color.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using TrackColor = px4_ros_com::action::TrackColor;
using GoalHandleTrackColor = rclcpp_action::ClientGoalHandle<TrackColor>;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl(std::string px4_namespace)
    : Node("offboard_control_srv"),
      state_{State::init},
      service_result_{0},
      service_done_{false},
      vehicle_altitude_{0.0f},
      vehicle_xdistance_{0.0f},
      vehicle_ydistance_{0.0f},
      vehicle_vertical_speed_{0.0f},
      current_yaw_{0.0f},
      init_yaw_{0.0f},
      init_yaw_sign_{false},
      init_altitude_{5},              // 设定飞行高度 m
      source_{"none"},
      num_of_steps_{0},
      buffer_threshold_{50},          // 5 s 缓冲
      target_x_{0.0f},
      target_y_{0.0f},
      target_initialized_{false},
      latest_offset_x_{0.0f},
      latest_offset_y_{0.0f},
      stable_count_{0},
      user_input_received_{false},
      tracking_goal_sent_{false},
      tracking_result_received_{false},
      tracking_success_{false},
      tracking_feedback_received_{false},
      guidance_timeout_sec_{1.5},     // 比你当前 2.0 稍紧一点，利于及时退出异常引导
      smoothing_alpha_{0.25f},        // 新目标点占 25%
      position_deadband_{0.12f},      // 小于 12 cm 不再更新目标
      max_step_per_feedback_{0.15f},  // 单次最多修正 15 cm
      max_abs_offset_world_{2.0f},    // 单次视觉世界偏移最大可信值，防止异常跳变
      offboard_control_mode_publisher_{
          this->create_publisher<OffboardControlMode>(px4_namespace + "in/offboard_control_mode", 10)},
      trajectory_setpoint_publisher_{
          this->create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", 10)},
      vehicle_command_client_{
          this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command")},
      track_color_action_client_{
          rclcpp_action::create_client<TrackColor>(this, "track_color")}
    {
        RCLCPP_INFO(this->get_logger(), "Starting optimized Offboard Control with TrackColor action");

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            px4_namespace + "out/vehicle_odometry", qos,
            std::bind(&OffboardControl::odometry_callback, this, std::placeholders::_1));

        while (!vehicle_command_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for vehicle_command service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "vehicle_command service not available, waiting again...");
        }

        while (!track_color_action_client_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for track_color action server. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "track_color action server not available, waiting again...");
        }

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));

        user_input_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                if (state_ == State::wait_for_color_input && !user_input_received_) {
                    std::string color;
                    std::cout << "\nEnter color to track (red/green/blue/black): ";
                    std::cin >> color;

                    std::lock_guard<std::mutex> lock(user_input_mutex_);
                    requested_color_ = color;
                    user_input_received_ = true;
                }
                std::this_thread::sleep_for(100ms);
            }
        });
    }

    ~OffboardControl() {
        if (user_input_thread_.joinable()) {
            user_input_thread_.join();
        }
    }

    void switch_to_offboard_mode();
    void arm();
    void disarm();
    void auto_land();

private:
    enum class State {
        init,
        offboard_requested,
        wait_for_stable_offboard_mode,
        arm_requested,
        armed,
        wait_for_color_input,
        send_track_goal,
        visual_guidance,
        land_requested,
        wait_for_stable_land,
        landing,
        complete
    } state_;

    uint8_t service_result_;
    bool service_done_;

    // 飞机状态
    float vehicle_altitude_;
    float vehicle_xdistance_;
    float vehicle_ydistance_;
    float vehicle_vertical_speed_;
    float current_yaw_;
    float init_yaw_;
    bool init_yaw_sign_;
    uint8_t init_altitude_;
    const char *source_;

    // 状态切换缓冲
    uint8_t num_of_steps_;
    uint8_t buffer_threshold_;

    // 当前目标点（世界系绝对参考点）
    float target_x_;
    float target_y_;
    bool target_initialized_;

    // 最近一次视觉偏移（机体系）
    float latest_offset_x_;
    float latest_offset_y_;
    int stable_count_;

    // 用户输入
    bool user_input_received_;
    std::string requested_color_;
    std::mutex user_input_mutex_;
    std::thread user_input_thread_;

    // Action 跟踪状态
    bool tracking_goal_sent_;
    bool tracking_result_received_;
    bool tracking_success_;
    bool tracking_feedback_received_;
    std::string tracking_result_message_;
    rclcpp::Time last_feedback_time_;

    // === 新增的稳定性参数 ===
    double guidance_timeout_sec_;
    float smoothing_alpha_;
    float position_deadband_;
    float max_step_per_feedback_;
    float max_abs_offset_world_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
    rclcpp_action::Client<TrackColor>::SharedPtr track_color_action_client_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

    void publish_offboard_control_mode();
    void publish_hover_trajectory_setpoint();
    void set_position(float xpoint, float ypoint, float zpoint, float setyaw);
    void switch_buffer(State next_state, const std::string &log_msg);
    void request_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f);
    void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void timer_callback();
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    void send_track_color_goal();
    void track_goal_response_callback(const GoalHandleTrackColor::SharedPtr &goal_handle);
    void track_feedback_callback(
        GoalHandleTrackColor::SharedPtr,
        const std::shared_ptr<const TrackColor::Feedback> feedback);
    void track_result_callback(const GoalHandleTrackColor::WrappedResult &result);

    static float clamp_float(float v, float lo, float hi) {
        return std::max(lo, std::min(v, hi));
    }
};

void OffboardControl::switch_to_offboard_mode() {
    RCLCPP_INFO(this->get_logger(), "requesting switch to offboard mode");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

void OffboardControl::arm() {
    RCLCPP_INFO(this->get_logger(), "requesting arm");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void OffboardControl::disarm() {
    RCLCPP_INFO(this->get_logger(), "requesting disarm");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void OffboardControl::auto_land() {
    RCLCPP_INFO(this->get_logger(), "requesting auto_land");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 6);
}

void OffboardControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_hover_trajectory_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {0.0f, 0.0f, -(float)init_altitude_};
    msg.yaw = init_yaw_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::set_position(float xpoint, float ypoint, float zpoint, float setyaw) {
    TrajectorySetpoint msg{};
    msg.position = {xpoint, ypoint, zpoint};
    msg.yaw = setyaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2, float param3) {
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    request->request = msg;

    service_done_ = false;
    vehicle_command_client_->async_send_request(
        request,
        std::bind(&OffboardControl::response_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Command send");
}

void OffboardControl::switch_buffer(State next_state, const std::string &log_msg) {
    if (++num_of_steps_ > buffer_threshold_) {
        num_of_steps_ = 0;
        RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
        state_ = next_state;
    }
}

void OffboardControl::send_track_color_goal() {
    if (tracking_goal_sent_) {
        return;
    }

    auto goal_msg = TrackColor::Goal();
    goal_msg.color = requested_color_;
    goal_msg.centered_tolerance = 0.12f;
    goal_msg.stable_count_required = 5;   // 原来 3，适当提高，减少过早判定
    goal_msg.timeout_sec = 20.0f;
    goal_msg.feedback_interval_sec = 0.5f; // 原来 0.3，降低反馈频率，减小追逐抖动

    RCLCPP_INFO(this->get_logger(),
                "Sending TrackColor goal: color=%s tolerance=%.2f stable=%d feedback_interval=%.2f",
                requested_color_.c_str(),
                goal_msg.centered_tolerance,
                goal_msg.stable_count_required,
                goal_msg.feedback_interval_sec);

    rclcpp_action::Client<TrackColor>::SendGoalOptions options;
    options.goal_response_callback =
        std::bind(&OffboardControl::track_goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback =
        std::bind(&OffboardControl::track_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
        std::bind(&OffboardControl::track_result_callback, this, std::placeholders::_1);

    track_color_action_client_->async_send_goal(goal_msg, options);
    tracking_goal_sent_ = true;
}

void OffboardControl::track_goal_response_callback(const GoalHandleTrackColor::SharedPtr &goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "TrackColor goal was rejected by server");
        tracking_goal_sent_ = false;
        state_ = State::wait_for_color_input;
        user_input_received_ = false;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "TrackColor goal accepted");
    tracking_feedback_received_ = false;
    last_feedback_time_ = this->now();
    state_ = State::visual_guidance;
}

void OffboardControl::track_feedback_callback(
    GoalHandleTrackColor::SharedPtr,
    const std::shared_ptr<const TrackColor::Feedback> feedback)
{
    tracking_feedback_received_ = true;
    last_feedback_time_ = this->now();

    if (!feedback->detected) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "TrackColor feedback: target not detected");
        stable_count_ = 0;
        return;
    }

    latest_offset_x_ = feedback->offset_x_body;
    latest_offset_y_ = feedback->offset_y_body;
    stable_count_ = feedback->stable_count;

    // 死区：已经很接近目标时，不再继续修正，避免围绕目标来回抖动
    if (feedback->planar_error < position_deadband_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Inside deadband, hold target. planar_error=%.3f stable_count=%d",
            feedback->planar_error, stable_count_);
        return;
    }

    // 对异常大的视觉世界偏移做限幅，避免单帧跳变导致猛冲
    float safe_offset_x_world =
        clamp_float(feedback->offset_x_world, -max_abs_offset_world_, max_abs_offset_world_);
    float safe_offset_y_world =
        clamp_float(feedback->offset_y_world, -max_abs_offset_world_, max_abs_offset_world_);

    // 本次视觉测得的绝对目标点
    float measured_target_x = vehicle_xdistance_ + safe_offset_x_world;
    float measured_target_y = vehicle_ydistance_ + safe_offset_y_world;

    if (!target_initialized_) {
        // 第一次视觉接管：直接初始化目标点
        target_x_ = measured_target_x;
        target_y_ = measured_target_y;
        target_initialized_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Initialize guidance target at (%.2f, %.2f)",
                    target_x_, target_y_);
        return;
    }

    // 单次修正量限幅，防止每次反馈跳太大
    float dx_cmd = measured_target_x - target_x_;
    float dy_cmd = measured_target_y - target_y_;

    dx_cmd = clamp_float(dx_cmd, -max_step_per_feedback_, max_step_per_feedback_);
    dy_cmd = clamp_float(dy_cmd, -max_step_per_feedback_, max_step_per_feedback_);

    float limited_target_x = target_x_ + dx_cmd;
    float limited_target_y = target_y_ + dy_cmd;

    // 一阶低通平滑
    target_x_ = (1.0f - smoothing_alpha_) * target_x_ + smoothing_alpha_ * limited_target_x;
    target_y_ = (1.0f - smoothing_alpha_) * target_y_ + smoothing_alpha_ * limited_target_y;

    RCLCPP_INFO(this->get_logger(),
                "Track feedback | offset_body=(%.2f, %.2f) rel_world=(%.2f, %.2f) target_abs=(%.2f, %.2f) stable_count=%d error=%.3f",
                latest_offset_x_,
                latest_offset_y_,
                safe_offset_x_world,
                safe_offset_y_world,
                target_x_,
                target_y_,
                stable_count_,
                feedback->planar_error);
}

void OffboardControl::track_result_callback(const GoalHandleTrackColor::WrappedResult &result) {
    tracking_result_received_ = true;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            tracking_success_ = result.result->success;
            tracking_result_message_ = result.result->message;
            RCLCPP_INFO(this->get_logger(), "TrackColor succeeded: %s", tracking_result_message_.c_str());
            if (tracking_success_) {
                state_ = State::land_requested;
            } else {
                state_ = State::wait_for_color_input;
                user_input_received_ = false;
                tracking_goal_sent_ = false;
                target_initialized_ = false;
            }
            break;

        case rclcpp_action::ResultCode::ABORTED:
            tracking_success_ = false;
            tracking_result_message_ = "aborted";
            RCLCPP_ERROR(this->get_logger(), "TrackColor aborted");
            state_ = State::wait_for_color_input;
            user_input_received_ = false;
            tracking_goal_sent_ = false;
            target_initialized_ = false;
            break;

        case rclcpp_action::ResultCode::CANCELED:
            tracking_success_ = false;
            tracking_result_message_ = "canceled";
            RCLCPP_WARN(this->get_logger(), "TrackColor canceled");
            state_ = State::wait_for_color_input;
            user_input_received_ = false;
            tracking_goal_sent_ = false;
            target_initialized_ = false;
            break;

        default:
            tracking_success_ = false;
            tracking_result_message_ = "unknown result";
            RCLCPP_WARN(this->get_logger(), "TrackColor unknown result");
            state_ = State::wait_for_color_input;
            user_input_received_ = false;
            tracking_goal_sent_ = false;
            target_initialized_ = false;
            break;
    }
}

void OffboardControl::timer_callback() {
    publish_offboard_control_mode();

    if (state_ != State::land_requested &&
        state_ != State::wait_for_stable_land &&
        state_ != State::landing &&
        state_ != State::complete) {

        if (state_ == State::visual_guidance && target_initialized_) {
            set_position(target_x_, target_y_, -(float)init_altitude_, init_yaw_);
        } else {
            publish_hover_trajectory_setpoint();
        }
    }

    switch (state_) {
        case State::init:
            switch_to_offboard_mode();
            state_ = State::offboard_requested;
            break;

        case State::offboard_requested:
            if (service_done_) {
                if (service_result_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
                    state_ = State::wait_for_stable_offboard_mode;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
                    rclcpp::shutdown();
                }
            }
            break;

        case State::wait_for_stable_offboard_mode:
            if (++num_of_steps_ > 10) {
                num_of_steps_ = 0;
                arm();
                state_ = State::arm_requested;
            }
            break;

        case State::arm_requested:
            if (service_done_) {
                if (service_result_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "Vehicle armed and taking off");
                    state_ = State::armed;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
                    rclcpp::shutdown();
                }
            }
            break;

        case State::armed:
            if (!init_yaw_sign_) {
                init_yaw_ = current_yaw_;
                init_yaw_sign_ = true;
            }

            RCLCPP_INFO(this->get_logger(),
                        "ARMED - altitude=%.3fm source=%s yaw=%.3f num_of_steps=%u",
                        vehicle_altitude_, source_, init_yaw_, num_of_steps_);

            if (vehicle_altitude_ > init_altitude_ * 0.95f) {
                switch_buffer(State::wait_for_color_input,
                              "Reached target altitude, waiting for color input");
                user_input_received_ = false;
            }
            break;

        case State::wait_for_color_input: {
            std::lock_guard<std::mutex> lock(user_input_mutex_);
            if (user_input_received_) {
                RCLCPP_INFO(this->get_logger(), "Color selected: %s", requested_color_.c_str());

                tracking_goal_sent_ = false;
                tracking_result_received_ = false;
                tracking_success_ = false;
                tracking_feedback_received_ = false;
                target_initialized_ = false;
                stable_count_ = 0;

                state_ = State::send_track_goal;
            }
            break;
        }

        case State::send_track_goal:
            send_track_color_goal();
            break;

        case State::visual_guidance: {
            if (tracking_feedback_received_) {
                double dt = (this->now() - last_feedback_time_).seconds();
                if (dt > guidance_timeout_sec_) {
                    RCLCPP_WARN(this->get_logger(),
                                "No TrackColor feedback for %.2f s, fallback to hover / wait_for_color_input",
                                dt);
                    target_initialized_ = false;
                    tracking_goal_sent_ = false;
                    tracking_feedback_received_ = false;
                    user_input_received_ = false;
                    state_ = State::wait_for_color_input;
                    break;
                }
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "VISUAL_GUIDANCE | pos=(%.2f, %.2f) target=(%.2f, %.2f) stable_count=%d",
                vehicle_xdistance_, vehicle_ydistance_, target_x_, target_y_, stable_count_);
            break;
        }

        case State::land_requested:
            auto_land();
            state_ = State::wait_for_stable_land;
            break;

        case State::wait_for_stable_land:
            if (service_done_) {
                if (service_result_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "Land command accepted");
                    state_ = State::landing;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Land command failed");
                    rclcpp::shutdown();
                }
            }
            break;

        case State::landing:
            if (vehicle_altitude_ < 2.0f && std::abs(vehicle_vertical_speed_) < 0.15f) {
                RCLCPP_INFO(this->get_logger(),
                            "Landing complete. Altitude: %.2fm Speed: %.2fm/s",
                            vehicle_altitude_, vehicle_vertical_speed_);
                switch_buffer(State::complete, "Entered complete mode");
            }
            break;

        case State::complete:
            RCLCPP_INFO(this->get_logger(), "Mission complete");
            rclcpp::shutdown();
            break;

        default:
            break;
    }
}

void OffboardControl::response_callback(
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
{
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        auto reply = future.get()->reply;
        service_result_ = reply.result;

        switch (service_result_) {
            case reply.VEHICLE_CMD_RESULT_ACCEPTED:
                RCLCPP_INFO(this->get_logger(), "command accepted");
                break;
            case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
                break;
            case reply.VEHICLE_CMD_RESULT_DENIED:
                RCLCPP_WARN(this->get_logger(), "command denied");
                break;
            case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
                RCLCPP_WARN(this->get_logger(), "command unsupported");
                break;
            case reply.VEHICLE_CMD_RESULT_FAILED:
                RCLCPP_WARN(this->get_logger(), "command failed");
                break;
            case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
                RCLCPP_WARN(this->get_logger(), "command in progress");
                break;
            case reply.VEHICLE_CMD_RESULT_CANCELLED:
                RCLCPP_WARN(this->get_logger(), "command cancelled");
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "command reply unknown");
                break;
        }
        service_done_ = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void OffboardControl::odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    if (!std::isnan(msg->position[2])) {
        vehicle_altitude_ = -msg->position[2];
    }

    if (!std::isnan(msg->position[0]) && !std::isnan(msg->position[1])) {
        vehicle_xdistance_ = msg->position[0];
        vehicle_ydistance_ = msg->position[1];
    }

    if (!std::isnan(msg->velocity[2])) {
        vehicle_vertical_speed_ = -msg->velocity[2];
    }

    if (!std::isnan(msg->q[0])) {
        float q0 = msg->q[0];
        float q1 = msg->q[1];
        float q2 = msg->q[2];
        float q3 = msg->q[3];
        current_yaw_ = std::atan2(
            2.0f * (q0 * q3 + q1 * q2),
            1.0f - 2.0f * (q2 * q2 + q3 * q3));
    }

    source_ = "ODOMETRY";
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>("/fmu/"));
    rclcpp::shutdown();
    return 0;
}