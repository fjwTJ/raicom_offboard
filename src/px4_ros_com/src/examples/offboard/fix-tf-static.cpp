#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticCameraTFNode : public rclcpp::Node
{
public:
    StaticCameraTFNode()
    : Node("static_camera_tf_node")
    {
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        publish_camera_to_body_tf(
            "uav_base_link",
            "camera_link",
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        );
    }

private:
    void publish_camera_to_body_tf(
        const std::string & body_frame,
        const std::string & camera_frame,
        double x, double y, double z,
        double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = body_frame;
        t.child_frame_id = camera_frame;

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        broadcaster_->sendTransform(t);

        RCLCPP_INFO(this->get_logger(),
                    "Published static TF: %s -> %s",
                    body_frame.c_str(), camera_frame.c_str());
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticCameraTFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}