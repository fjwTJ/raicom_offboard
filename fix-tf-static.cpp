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
 * @brief tf
 * @file fix-tf-static.cpp
 *
 * 设计目标：
 *  ：发布一个静态 TF，表示相机坐标系相对于机体坐标系的固定变换。
#pragma once
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticCameraTFHelper
{
public:
    explicit StaticCameraTFHelper(rclcpp::Node * node)
    : node_(node)
    {//
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    }
     // 发布相机到机体的静态 TF
    void publishCameraToBodyTF(
        const std::string & body_frame,
        const std::string & camera_frame,
        double x, double y, double z,
        double roll, double pitch, double yaw
        )
    { // 构造 TransformStamped 消息
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = node_->get_clock()->now();
        t.header.frame_id = body_frame;
        t.child_frame_id = camera_frame;
// 设置平移
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
       // 将 roll, pitch, yaw 转换为四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        broadcaster_->sendTransform(t);

        RCLCPP_INFO(node_->get_logger(),
                    "Published static TF: %s -> %s",
                    body_frame.c_str(), camera_frame.c_str());
    }

private:
    rclcpp::Node * node_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};