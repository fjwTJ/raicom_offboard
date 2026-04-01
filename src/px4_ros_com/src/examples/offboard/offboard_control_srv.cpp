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
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples * 
 * @author Beniamino Pozzan <beniamino.pozzan@gmail.com>
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

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
#include "px4_msgs/srv/detect_color.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string px4_namespace) :
		Node("offboard_control_srv"),
		state_{State::init},
		service_result_{0},
		service_done_{false},
		current_yaw_{0},
		init_yaw_{0},
		init_yaw_sign_{false},
		init_altitude_{5},	//设定飞行高度m
		source_{"none"},
		num_of_steps_{0},
    	buffer_threshold_{50},	// 默认阈值5s
		target_x_{0},
        target_y_{0},
        user_input_received_{false},
		offboard_control_mode_publisher_{this->create_publisher<OffboardControlMode>(px4_namespace+"in/offboard_control_mode", 10)},
		trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>(px4_namespace+"in/trajectory_setpoint", 10)},
		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace+"vehicle_command")},
		detect_color_client_{this->create_client<px4_msgs::srv::DetectColor>("detect_color")}
	{
		RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "detect_color service");

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

		while (!detect_color_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for detect_color service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "detect_color service not available, waiting again...");
        }

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));

        // 启动用户输入线程
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
	enum class State{
		init,
		offboard_requested,
		wait_for_stable_offboard_mode,
		arm_requested,
		armed,
		wait_for_color_input,
        detect_color,
		mission,
		returned,
		land_requested,
		wait_for_stable_land,
		landing,
		complete
	} state_;
    
	uint8_t service_result_;
	bool service_done_;
    float vehicle_altitude_;            // 当前高度
    float vehicle_xdistance_;           // x方向移动距离
    float vehicle_ydistance_;           // y方向移动距离
	float vehicle_vertical_speed_;
    float current_yaw_;                 // 当前yaw
    float init_yaw_;                    // 初始yaw
    bool init_yaw_sign_;                // 设置初始yaw标志位
    uint8_t init_altitude_;             // 初始设置高度
    const char* source_;                // 高度数据来源
    uint8_t num_of_steps_;              // 计数器
    uint8_t buffer_threshold_;          // 计数阈值
	float target_x_;                    // 目标点x坐标
    float target_y_;                    // 目标点y坐标
    bool user_input_received_;          // 用户输入已接收
    std::string requested_color_;       // 用户请求的颜色
    std::mutex user_input_mutex_;       // 用户输入互斥锁
    std::thread user_input_thread_;     // 用户输入线程

    rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
	rclcpp::Client<px4_msgs::srv::DetectColor>::SharedPtr detect_color_client_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void set_position(float xpoint, float ypoint, float zpoint, float setyaw);
	void switch_buffer(State next_state, const std::string& log_msg);
	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	void timer_callback(void);
	void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
};

/**
 * @brief Send a command to switch to offboard mode
 */
void OffboardControl::switch_to_offboard_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	RCLCPP_INFO(this->get_logger(), "requesting arm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	RCLCPP_INFO(this->get_logger(), "requesting disarm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

/**
 * @brief Send a command to Auto_Land the vehicle
 */
void OffboardControl::auto_land()
{
	RCLCPP_INFO(this->get_logger(), "requesting auto_land");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 6);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
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
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -(float)init_altitude_};
	msg.yaw = init_yaw_; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publishing a parameter trajectory setpoint
 * @param xpoint    x-direction moving distance(forward)
 * @param ypoint    y-direction moving distance(right)
 * @param zpoint    z-direction moving distance(down)
 * @param setyaw    vehicle yaw angle [-PI:PI]
 */
void OffboardControl::set_position(float xpoint, float ypoint, float zpoint, float setyaw)
{
	TrajectorySetpoint msg{};
	msg.position = {xpoint, ypoint, zpoint};
	msg.yaw = setyaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @param param3    Command parameter 3
 */
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
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
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
                           std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Command send");
}

void OffboardControl::switch_buffer(State next_state, const std::string& log_msg){
	if (++num_of_steps_ > buffer_threshold_) {
        num_of_steps_ = 0;
        RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
        state_ = next_state;
    }
}

void OffboardControl::timer_callback(void){

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();

    // 在特定状态发布轨迹设定点
    if (state_ != State::land_requested && 
        state_ != State::wait_for_stable_land && 
        state_ != State::landing && 
        state_ != State::complete) {
        publish_trajectory_setpoint();
    }

	switch (state_){
    case State::init:
		switch_to_offboard_mode();
		state_ = State::offboard_requested;
		break;

    case State::offboard_requested:
		if(service_done_){
			if (service_result_ == 0){
			RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
			state_ = State::wait_for_stable_offboard_mode;				
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
				rclcpp::shutdown();
			}
		}
		break;

    case State::wait_for_stable_offboard_mode:
		if (++num_of_steps_ > 10){
			num_of_steps_ = 0;
			arm();
			state_ = State::arm_requested;
		}
		break;

    case State::arm_requested:
		if(service_done_){
			if (service_result_ == 0){
                RCLCPP_INFO(this->get_logger(), "Vehicle armed and taking off");
				state_ = State::armed;
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
				rclcpp::shutdown();
			}
		}
		break;

	case State::armed:
		// 打印当前高度与计数，便于调试
		RCLCPP_INFO(this->get_logger(), "ARMED - altitude=%.3fm source=%s yaw=%.3f num_of_steps=%u",
					vehicle_altitude_, source_, init_yaw_, num_of_steps_);
		if(vehicle_altitude_ > 0.5f)
			RCLCPP_INFO(this->get_logger(), "Altitude: %.2fm (Source: %s) Yaw: %.2f", 
						vehicle_altitude_, source_, init_yaw_);

		if(vehicle_altitude_ > init_altitude_ * 0.95){
			switch_buffer(State::wait_for_color_input, "Reached target altitude, waiting for color input");
			user_input_received_ = false;
		}
		break;

    case State::wait_for_color_input:
		{
			std::lock_guard<std::mutex> lock(user_input_mutex_);
			if (user_input_received_) {
				RCLCPP_INFO(this->get_logger(), "Color selected: %s", requested_color_.c_str());
				state_ = State::detect_color;
			}
		}
		break;
		
    case State::detect_color:
        {
            auto request = std::make_shared<px4_msgs::srv::DetectColor::Request>();
            request->color = requested_color_;
            
            // 使用异步调用服务，避免重复spin节点
            auto response_received_callback = [this](rclcpp::Client<px4_msgs::srv::DetectColor>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    target_x_ = response->x;
                    target_y_ = response->y;
                    RCLCPP_INFO(this->get_logger(), "Target detected at (%.2f, %.2f) meters", target_x_, target_y_);
                    state_ = State::mission;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Color detection failed, retrying...");
                    state_ = State::wait_for_color_input;
                    user_input_received_ = false;
                }
            };
            
            // 直接使用future对象
            auto future_result = detect_color_client_->async_send_request(request, response_received_callback);
            
            // 使用move转移所有权，避免复制问题
            auto timer = this->create_wall_timer(5s, [this, future_result = std::move(future_result)]() {
                if (future_result.wait_for(0s) != std::future_status::ready) {
                    RCLCPP_ERROR(this->get_logger(), "Color detection service call timed out");
                    state_ = State::wait_for_color_input;
                    user_input_received_ = false;
                }
            });
        }
        break;

    case State::mission:
		RCLCPP_INFO(this->get_logger(), "x移动: %.2fm y移动: %.2fm yaw: %.2f",
					vehicle_xdistance_, vehicle_ydistance_, init_yaw_);
		set_position(target_x_, target_y_, -(float)init_altitude_, init_yaw_);

		if ((std::abs(vehicle_xdistance_ - target_x_) < 0.05) && 
			(std::abs(vehicle_ydistance_ - target_y_) < 0.05)){
			switch_buffer(State::land_requested, "Reached target, requesting land");
		}
		break;

	/* case State::returned:
		RCLCPP_INFO(this->get_logger(), "x移动: %.2fm y移动: %.2fm yaw: %.2f",
					vehicle_xdistance_, vehicle_ydistance_, init_yaw_);
		set_position(0.0, 0.0, -(float)init_altitude_, init_yaw_);

		if ((std::abs(vehicle_xdistance_) < 0.05) && 
            (std::abs(vehicle_ydistance_) < 0.05)){
			switch_buffer(State::land_requested, "Reached home, requesting land");
		}
		break; */
		
	case State::land_requested:
		auto_land();
		state_ = State::wait_for_stable_land;
		break;
            
    case State::wait_for_stable_land:
		if(service_done_){
			if (service_result_ == 0){
				RCLCPP_INFO(this->get_logger(), "Land command accepted");
			state_ = State::landing;
			} 
			else {
				RCLCPP_ERROR(this->get_logger(), "Land command failed");
			rclcpp::shutdown();
			}
		}
		break;

	case State::landing:
		// 检查高度和垂直速度是否满足着陆条件
		if (vehicle_altitude_ < 2 && std::abs(vehicle_vertical_speed_) < 0.15) {
			RCLCPP_INFO(this->get_logger(), "Landing complete. Altitude: %.2fm Speed: %.2fm/s", 
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
      rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
	  auto reply = future.get()->reply;
	  service_result_ = reply.result;
      switch (service_result_)
		{
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
    // 打印 debug 信息
    RCLCPP_DEBUG(this->get_logger(),
                 "odometry: x=%.3f y=%.3f z=%.3f vx=%.3f vy=%.3f vz=%.3f frame=%u",
                 msg->position[0], msg->position[1], msg->position[2],
                 msg->velocity[0], msg->velocity[1], msg->velocity[2],
                 msg->pose_frame);

    // 高度：NED 系，z 下为正 -> 转换成“高度为正”的常用表达
    if (!std::isnan(msg->position[2])) {
        vehicle_altitude_ = -msg->position[2];  // 起飞后变正值
    }

    // 平面位置
    if (!std::isnan(msg->position[0]) && !std::isnan(msg->position[1])) {
        vehicle_xdistance_ = msg->position[0];
        vehicle_ydistance_ = msg->position[1];
    }

    // 垂直速度（NED：下为正，这里取反让上升为正）
    if (!std::isnan(msg->velocity[2])) {
        vehicle_vertical_speed_ = -msg->velocity[2];
    }

    // yaw 从四元数解算（NED frame）
    if (!std::isnan(msg->q[0])) {
        float q0 = msg->q[0];
        float q1 = msg->q[1];
        float q2 = msg->q[2];
        float q3 = msg->q[3];
        // 参考公式：yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
        current_yaw_ = std::atan2(2.0f * (q0*q3 + q1*q2),
                                  1.0f - 2.0f * (q2*q2 + q3*q3));
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