/****************************************************************************
 *
 * Copyright 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source_ and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source_ code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief 基于原始 offboard_control_srv.cpp 改造的“连续重识别版”控制节点
 * @file offboard_control_srv_optimized_annotated.cpp
 *
 * 设计目标：
 * 1. 保留原有 PX4 Offboard 控制、起飞、等待颜色输入、自动降落的主框架。
 * 2. 将“只识别一次再飞过去”的流程，改成“首次粗定位 + 飞行过程中持续重识别”。
 * 3. 将 detector 返回的“相对偏移”显式转换为“绝对目标点”，避免语义混淆。
 * 4. 增加识别请求节流、超时保护和目标点平滑，提升稳定性。
 */
#include <algorithm>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include "px4_msgs/srv/detect_color.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl(std::string px4_namespace) :
        Node("offboard_control_srv"),

        // ==================== 基本状态初始化 ====================
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
        init_altitude_{5},          // 初始目标高度，单位 m
        source_{"none"},
        num_of_steps_{0},
        buffer_threshold_{50},      // 100ms 定时器下，约等于 5 秒缓冲
        target_x_{0.0f},
        target_y_{0.0f},
        latest_offset_x_{0.0f},
        latest_offset_y_{0.0f},
        user_input_received_{false},

        // ====================【修改点2】连续重识别所需状态 ====================
        target_initialized_{false},
        detect_request_in_progress_{false},
        detect_request_seq_{0},
        active_detect_request_seq_{0},
        centered_count_{0},

        // 识别频率和超时控制参数
        detection_interval_{350ms},
        detection_timeout_{1500ms},

        // “已位于目标正上方”的判据
        centered_tolerance_m_{0.12f},
        centered_required_count_{3},

        // 目标点平滑参数
        smoothing_alpha_{0.40f},

        // ==================== ROS2 通信对象 ====================
        offboard_control_mode_publisher_{this->create_publisher<OffboardControlMode>(px4_namespace + "in/offboard_control_mode", 10)},
        trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", 10)},
        vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command")},
        detect_color_client_{this->create_client<px4_msgs::srv::DetectColor>("detect_color")}
    {
        RCLCPP_INFO(this->get_logger(), "Starting optimized offboard controller with continuous color re-detection");
        RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
        RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for detect_color service");

        // 里程计订阅沿用原逻辑，使用 sensor data QoS
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            px4_namespace + "out/vehicle_odometry", qos,
            std::bind(&OffboardControl::odometry_callback, this, std::placeholders::_1));

        // 等待 vehicle_command 服务可用
        while (!vehicle_command_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for vehicle_command service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "vehicle_command service not available, waiting again...");
        }

        // 等待 detect_color 服务可用
        while (!detect_color_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for detect_color service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "detect_color service not available, waiting again...");
        }

        // 主状态机定时器：100ms 触发一次
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));

        // 保留原始的终端输入颜色方式，方便测试
        user_input_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                if (state_ == State::wait_for_color_input && !user_input_received_) {
                    std::string color;
                    std::cout << "\nEnter color to detect (red/green/blue/black): ";
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
    // ====================【修改点1】状态机重构 ==================
    // 这样做的目的是把“第一次粗识别”和“飞行中的持续重识别”分开。
    enum class State {
        init,
        offboard_requested,
        wait_for_stable_offboard_mode,
        arm_requested,
        armed,
        wait_for_color_input,
        detect_color_once,   // 【修改点1】第一次粗定位
        visual_guidance,     // 【修改点1】持续视觉引导
        land_requested,
        wait_for_stable_land,
        landing,
        complete
    } state_;

    // ==================== 原始控制与飞行状态变量 ====================
    uint8_t service_result_;
    bool service_done_;

    float vehicle_altitude_;          // 当前高度（上为正）
    float vehicle_xdistance_;         // 世界系/NED 下的 x 位置
    float vehicle_ydistance_;         // 世界系/NED 下的 y 位置
    float vehicle_vertical_speed_;    // 竖直速度（上为正）
    float current_yaw_;               // 当前 yaw
    float init_yaw_;                  // 起飞后的初始 yaw
    bool init_yaw_sign_;              // 是否已记录初始 yaw
    uint8_t init_altitude_;           // 设定飞行高度
    const char *source_;              // 高度来源标记
    uint8_t num_of_steps_;            // 状态缓冲计数器
    uint8_t buffer_threshold_;        // 状态缓冲阈值

    // ====================【修改点3】====================
    // 原代码中 target_x_ / target_y_ 直接接 detector 返回值，这会把“偏移量”当成“绝对点”使用。
    float target_x_;                  // 发给 PX4 的绝对目标点 x
    float target_y_;                  // 发给 PX4 的绝对目标点 y
    float latest_offset_x_;           // 最新一次识别返回的相对偏移 x
    float latest_offset_y_;           // 最新一次识别返回的相对偏移 y

    // ==================== 用户输入相关 ====================
    bool user_input_received_;
    std::string requested_color_;
    std::mutex user_input_mutex_;
    std::thread user_input_thread_;

    // ====================【修改点2】持续重识别相关状态 ====================
    bool target_initialized_;         // 是否已经拿到至少一个有效目标点
    bool detect_request_in_progress_; // 当前是否有识别请求尚未返回
    uint64_t detect_request_seq_;     // 请求自增编号
    uint64_t active_detect_request_seq_; // 当前有效请求编号，用于丢弃过期回调
    int centered_count_;              // 连续“目标已在正下方”的次数

    // 使用 steady_clock 记录请求时间，更适合做纯时间间隔判断
    std::chrono::steady_clock::time_point last_detect_request_time_{};

    // ====================【修改点4】参数配置 ====================
    const std::chrono::milliseconds detection_interval_; // 连续重识别最小时间间隔
    const std::chrono::milliseconds detection_timeout_;  // 单次识别超时时间
    const float centered_tolerance_m_;                   // 判定正下方的平面误差阈值
    const int centered_required_count_;                  // 连续满足多少次才允许降落
    const float smoothing_alpha_;                        // 目标点平滑系数

    // ==================== ROS2 通信对象 ====================
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
    rclcpp::Client<px4_msgs::srv::DetectColor>::SharedPtr detect_color_client_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

    // ==================== 成员函数声明 ====================
    void publish_offboard_control_mode();
    void publish_hover_setpoint();
    void set_position(float xpoint, float ypoint, float zpoint, float setyaw);
    void switch_buffer(State next_state, const std::string &log_msg);
    void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
    void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void timer_callback();
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    // ====================【修改点5】新增的视觉引导辅助函数 ====================
    void request_color_detection(bool first_detection);
    bool should_request_redetection() const;
    bool detect_request_timed_out() const;
    void handle_successful_detection(double dx_world, double dy_world, bool first_detection);
};

/**
 * @brief 请求切入 Offboard 模式
 */
void OffboardControl::switch_to_offboard_mode() {
    RCLCPP_INFO(this->get_logger(), "requesting switch to offboard mode");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

/**
 * @brief 请求解锁
 */
void OffboardControl::arm() {
    RCLCPP_INFO(this->get_logger(), "requesting arm");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

/**
 * @brief 请求上锁
 */
void OffboardControl::disarm() {
    RCLCPP_INFO(this->get_logger(), "requesting disarm");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

/**
 * @brief 请求自动降落模式
 */
void OffboardControl::auto_land() {
    RCLCPP_INFO(this->get_logger(), "requesting auto_land");
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 6);
}

/**
 * @brief 发布 Offboard 控制模式
 * @note 与轨迹点配套使用。这里仅开启位置控制。
 */
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

/**
 * @brief 发布悬停参考点
 * @note 原来的 publish_trajectory_setpoint() 本质上是“原点上空悬停”。
 *       这里更名为 publish_hover_setpoint()，语义更清楚。
 */
void OffboardControl::publish_hover_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {0.0f, 0.0f, -(float)init_altitude_};
    msg.yaw = init_yaw_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief 发布任意位置参考点
 * @param xpoint 世界系/NED 下的 x 目标点
 * @param ypoint 世界系/NED 下的 y 目标点
 * @param zpoint 世界系/NED 下的 z 目标点（NED 下向下为正，因此高度通常取负）
 * @param setyaw yaw 角
 */
void OffboardControl::set_position(float xpoint, float ypoint, float zpoint, float setyaw) {
    TrajectorySetpoint msg{};
    msg.position = {xpoint, ypoint, zpoint};
    msg.yaw = setyaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief 发送 PX4 VehicleCommand 请求
 */
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

    RCLCPP_INFO(this->get_logger(), "Vehicle command sent");
}

/**
 * @brief 带缓冲的状态切换函数
 * @note 连续多次满足条件后再切换，用来抑制瞬时抖动。
 */
void OffboardControl::switch_buffer(State next_state, const std::string &log_msg) {
    if (++num_of_steps_ > buffer_threshold_) {
        num_of_steps_ = 0;
        RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
        state_ = next_state;
    }
}

/**
 * @brief 判断是否应该发起下一次重识别
 * @return true 表示可发，false 表示当前不该发
 *
 * 判定逻辑：
 * 1. 当前不能有请求在飞。
 * 2. 距离上一次发请求已经超过 detection_interval_。
 */
bool OffboardControl::should_request_redetection() const {
    if (detect_request_in_progress_) {
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    return (now - last_detect_request_time_) >= detection_interval_;
}

/**
 * @brief 判断当前识别请求是否超时
 */
bool OffboardControl::detect_request_timed_out() const {
    if (!detect_request_in_progress_) {
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    return (now - last_detect_request_time_) >= detection_timeout_;
}

/**
 * @brief 处理一次成功的颜色识别结果
 * @param dx_world detector 返回的世界系 x 偏移
 * @param dy_world detector 返回的世界系 y 偏移
 * @param first_detection 是否为首次识别
 *改：
 *   绝对目标点 = 当前无人机位置 + 本次视觉偏移
 * 然后再把绝对目标点发给 PX4。
 */
void OffboardControl::handle_successful_detection(double dx_world, double dy_world, bool first_detection) {
    // ====================【修改点3】先保存“相对偏移” ====================
    latest_offset_x_ = static_cast<float>(dx_world);
    latest_offset_y_ = static_cast<float>(dy_world);

    // ====================【修改点3】把“相对偏移”转成“绝对目标点” ====================
    // detector 返回的是“目标相对当前无人机位置的世界系偏移”
    // 所以要显式加到当前 odometry 位置上，才能得到真正要发给 PX4 的绝对参考点。
    const float measured_target_x = vehicle_xdistance_ + latest_offset_x_;
    const float measured_target_y = vehicle_ydistance_ + latest_offset_y_;

    // 首次识别：直接初始化目标点
    // 后续识别：采用低通平滑，减小视觉抖动引起的目标点跳变
    if (!target_initialized_ || first_detection) {
        target_x_ = measured_target_x;
        target_y_ = measured_target_y;
        target_initialized_ = true;
    } else {
        // ====================【修改点4】目标点平滑 ====================
        target_x_ = smoothing_alpha_ * measured_target_x + (1.0f - smoothing_alpha_) * target_x_;
        target_y_ = smoothing_alpha_ * measured_target_y + (1.0f - smoothing_alpha_) * target_y_;
    }

    // ====================【修改点6】用“视觉偏移”而不是“固定点误差”判断是否对准 ====================
    // 当 latest_offset 足够小，说明当前无人机已经很接近目标正上方。
    const float planar_error = std::hypot(latest_offset_x_, latest_offset_y_);
    if (planar_error < centered_tolerance_m_) {
        centered_count_++;
    } else {
        centered_count_ = 0;
    }

    RCLCPP_INFO(this->get_logger(),
                "Detection update | rel_offset=(%.2f, %.2f)m abs_target=(%.2f, %.2f)m centered_count=%d",
                latest_offset_x_, latest_offset_y_, target_x_, target_y_, centered_count_);
}

/**
 * @brief 发起一次颜色识别请求
 * @param first_detection true 表示第一次粗定位，false 表示飞行中的重识别
 *
 * 优化点：
 * 1. 增加 detect_request_in_progress_，防止重复并发请求。
 * 2. 增加请求编号 request_id，防止超时后的旧回调污染当前状态。
 */
void OffboardControl::request_color_detection(bool first_detection) {
    // 若当前已有请求尚未返回，则不再重复发送
    if (detect_request_in_progress_) {
        return;
    }

    auto request = std::make_shared<px4_msgs::srv::DetectColor::Request>();
    {
        std::lock_guard<std::mutex> lock(user_input_mutex_);
        request->color = requested_color_;
    }

    detect_request_in_progress_ = true;
    last_detect_request_time_ = std::chrono::steady_clock::now();

    // ====================【修改点5】为每次请求分配唯一编号 ====================
    const uint64_t request_id = ++detect_request_seq_;
    active_detect_request_seq_ = request_id;

    detect_color_client_->async_send_request(
        request,
        [this, first_detection, request_id](rclcpp::Client<px4_msgs::srv::DetectColor>::SharedFuture future) {
            // 若当前回调对应的不是最新有效请求，则直接丢弃
            if (request_id != active_detect_request_seq_) {
                return;
            }

            detect_request_in_progress_ = false;
            auto response = future.get();

            // 识别失败时的处理：
            // - 第一次识别失败：回到等待颜色输入状态
            // - 飞行中重识别失败：保持当前目标点，继续飞向上一次有效目标点
            if (!response->success) {
                RCLCPP_WARN(this->get_logger(), "Color detection failed");
                centered_count_ = 0;

                if (first_detection) {
                    state_ = State::wait_for_color_input;
                    user_input_received_ = false;
                    target_initialized_ = false;
                }
                return;
            }

            // 成功识别后的统一处理
            handle_successful_detection(response->x, response->y, first_detection);

            // 若这是第一次识别成功，则正式进入“持续视觉引导”状态
            if (first_detection && state_ == State::detect_color_once) {
                RCLCPP_INFO(this->get_logger(), "First detection succeeded, entering visual guidance");
                state_ = State::visual_guidance;
            }
        });
}

/**
 * @brief 主状态机定时回调
 *   wait_for_color_input -> detect_color_once -> visual_guidance -> land_requested
 *   先粗定位，再持续重识别和修正目标点。
 */
void OffboardControl::timer_callback() {
    // Offboard 模式下，每次都需要持续发布控制模式
    publish_offboard_control_mode();

    // ====================【修改点7】不同状态发布不同参考点 ====================
    // 非降落状态下持续发 setpoint：
    // 1. 在 visual_guidance 状态且已有目标点时，飞向最新更新后的目标点。
    // 2. 其余状态下，保持原点上空悬停。
    if (state_ != State::land_requested &&
        state_ != State::wait_for_stable_land &&
        state_ != State::landing &&
        state_ != State::complete) {
        if (state_ == State::visual_guidance && target_initialized_) {
            set_position(target_x_, target_y_, -(float)init_altitude_, init_yaw_);
        } else {
            publish_hover_setpoint();
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
        // ====================【修改点8】起飞后记录初始 yaw ====================
        // 这里在起飞阶段记录一次当前 yaw，后续悬停和视觉引导都沿用这个 yaw。
        if (!init_yaw_sign_) {
            init_yaw_ = current_yaw_;
            init_yaw_sign_ = true;
            RCLCPP_INFO(this->get_logger(), "Captured initial yaw: %.3f rad", init_yaw_);
        }

        RCLCPP_INFO(this->get_logger(),
                    "ARMED - altitude=%.2fm source=%s yaw=%.3f",
                    vehicle_altitude_, source_, init_yaw_);

        // 接近目标高度后进入等待颜色输入状态
        if (vehicle_altitude_ > init_altitude_ * 0.95f) {
            user_input_received_ = false;
            switch_buffer(State::wait_for_color_input, "Reached target altitude, waiting for color input");
        }
        break;

    case State::wait_for_color_input:
        {
            std::lock_guard<std::mutex> lock(user_input_mutex_);
            if (user_input_received_) {
                // ====================【修改点9】新任务开始前清理旧状态 ====================
                centered_count_ = 0;
                target_initialized_ = false;
                detect_request_in_progress_ = false;
                active_detect_request_seq_ = 0;

                RCLCPP_INFO(this->get_logger(), "Color selected: %s", requested_color_.c_str());
                state_ = State::detect_color_once;
            }
        }
        break;

    case State::detect_color_once:
        // ====================【修改点10】第一次识别只负责“粗定位” ====================
        // 若超时，则回到等待输入状态。
        if (detect_request_timed_out()) {
            RCLCPP_ERROR(this->get_logger(), "Initial color detection timed out");
            detect_request_in_progress_ = false;
            active_detect_request_seq_ = 0;
            user_input_received_ = false;
            state_ = State::wait_for_color_input;
            break;
        }

        if (!detect_request_in_progress_) {
            request_color_detection(true);
        }
        break;

    case State::visual_guidance:
        // ====================【修改点11】持续重识别状态 ====================
        // 1. 周期性重识别
        // 2. 更新目标点
        // 3. 连续多次确认已对准后再降落

        // 如果某次重识别超时，则不退出任务，只是继续飞向上一次有效目标点
        if (detect_request_timed_out()) {
            RCLCPP_WARN(this->get_logger(), "Re-detection timed out, keep flying to last valid target");
            detect_request_in_progress_ = false;
            active_detect_request_seq_ = 0;
        }

        // 达到最小时间间隔后，再发起下一次识别
        if (should_request_redetection()) {
            request_color_detection(false);
        }

        RCLCPP_INFO(this->get_logger(),
                    "GUIDANCE | pos=(%.2f, %.2f) target=(%.2f, %.2f) latest_offset=(%.2f, %.2f) centered_count=%d",
                    vehicle_xdistance_, vehicle_ydistance_,
                    target_x_, target_y_,
                    latest_offset_x_, latest_offset_y_,
                    centered_count_);

        // 若连续多次都满足“目标已在正下方附近”，则进入降落申请
        if (centered_count_ >= centered_required_count_) {
            switch_buffer(State::land_requested, "Target centered stably, requesting land");
        }
        break;

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

/**
 * @brief 处理 PX4 VehicleCommand 的服务响应
 */
void OffboardControl::response_callback(
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
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

/**
 * @brief odometry 回调
 *
 * 功能：
 * 1. 更新当前高度、平面位置、竖直速度。
 * 2. 从四元数解算当前 yaw。
 * 3. 为后续“目标偏移 -> 绝对目标点”转换提供当前位置。
 */
void OffboardControl::odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(),
                 "odometry: x=%.3f y=%.3f z=%.3f vx=%.3f vy=%.3f vz=%.3f frame=%u",
                 msg->position[0], msg->position[1], msg->position[2],
                 msg->velocity[0], msg->velocity[1], msg->velocity[2],
                 msg->pose_frame);

    // NED 坐标系下 z 向下为正，这里取负号，转成更符合直觉的“高度为正”表达
    if (!std::isnan(msg->position[2])) {
        vehicle_altitude_ = -msg->position[2];
    }

    // 记录当前世界系/NED 平面位置
    if (!std::isnan(msg->position[0]) && !std::isnan(msg->position[1])) {
        vehicle_xdistance_ = msg->position[0];
        vehicle_ydistance_ = msg->position[1];
    }

    // NED 系下速度 z 向下为正，这里取反，使上升为正
    if (!std::isnan(msg->velocity[2])) {
        vehicle_vertical_speed_ = -msg->velocity[2];
    }

    // 从四元数解算 yaw
    if (!std::isnan(msg->q[0])) {
        float q0 = msg->q[0];
        float q1 = msg->q[1];
        float q2 = msg->q[2];
        float q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
    }

    source_ = "ODOMETRY";
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>("/fmu/"));
    rclcpp::shutdown();
    return 0;
}
