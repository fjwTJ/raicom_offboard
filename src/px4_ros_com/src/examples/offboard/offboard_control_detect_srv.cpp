#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "px4_ros_com/action/track_color.hpp"

#include <mutex>
#include <map>
#include <atomic>
#include <vector>
#include <algorithm>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;
using TrackColor = px4_ros_com::action::TrackColor;
using GoalHandleTrackColor = rclcpp_action::ServerGoalHandle<TrackColor>;

class ColorDetectionActionServer : public rclcpp::Node {
public:
    ColorDetectionActionServer() : Node("color_detection_service")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&ColorDetectionActionServer::image_callback, this, std::placeholders::_1));

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&ColorDetectionActionServer::odom_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        action_server_ = rclcpp_action::create_server<TrackColor>(
            this,
            "track_color",
            std::bind(&ColorDetectionActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ColorDetectionActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ColorDetectionActionServer::handle_accepted, this, std::placeholders::_1));

        color_thresholds_["red"]   = std::make_pair(cv::Scalar(0, 200, 150),   cv::Scalar(10, 255, 230));
        color_thresholds_["green"] = std::make_pair(cv::Scalar(50, 200, 150),  cv::Scalar(70, 255, 230));
        color_thresholds_["blue"]  = std::make_pair(cv::Scalar(110, 200, 150), cv::Scalar(130, 255, 230));
        color_thresholds_["black"] = std::make_pair(cv::Scalar(0, 0, 50),      cv::Scalar(180, 50, 100));

        cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "TrackColor action server ready.");
    }

    ~ColorDetectionActionServer() {
        cv::destroyAllWindows();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp_action::Server<TrackColor>::SharedPtr action_server_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Mat current_frame_;
    std::mutex frame_mutex_;

    double current_yaw_{0.0};
    std::mutex odom_mutex_;

    std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_thresholds_;

    std::atomic<bool> detection_active_{false};
    std::string active_color_;
    std::mutex detection_mutex_;

    double latest_cam_x_{0.0};
    double latest_cam_y_{0.0};

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                current_frame_ = cv_ptr->image.clone();
            }

            cv::Mat display_frame = current_frame_.clone();
            if (detection_active_) {
                std::lock_guard<std::mutex> det_lock(detection_mutex_);
                process_frame(display_frame, active_color_, false);
            }

            cv::imshow("Camera Feed", display_frame);
            cv::waitKey(1);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        double q0 = msg->q[0];
        double q1 = msg->q[1];
        double q2 = msg->q[2];
        double q3 = msg->q[3];
        double yaw = std::atan2(2.0 * (q0 * q3 + q1 * q2),
                                1.0 - 2.0 * (q2 * q2 + q3 * q3));
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_yaw_ = yaw;
    }

    bool process_frame(cv::Mat &image, const std::string &color, bool annotate = true) {
        if (color_thresholds_.find(color) == color_thresholds_.end()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Unsupported color: %s", color.c_str());
            return false;
        }

        auto &thresholds = color_thresholds_[color];
        cv::Mat hsv_img, mask_color;
        cv::cvtColor(image, hsv_img, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_img, thresholds.first, thresholds.second, mask_color);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask_color, mask_color, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask_color, mask_color, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_color, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        if (contours.empty()) {
            if (annotate) {
                cv::putText(image, "No " + color + " object", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }
            return false;
        }

        auto max_contour = *std::max_element(
            contours.begin(), contours.end(),
            [](const auto &a, const auto &b) { return cv::contourArea(a) < cv::contourArea(b); });

        cv::Rect rect = cv::boundingRect(max_contour);
        int x = rect.x, y = rect.y, w = rect.width, h = rect.height;

        double img_cx = image.cols / 2.0;
        double img_cy = image.rows / 2.0;
        double obj_cx = x + w / 2.0;
        double obj_cy = y + h / 2.0;

        double rel_x = -(obj_cy - img_cy); // 上为正
        double rel_y =  (obj_cx - img_cx); // 右为正

        double pixel_to_meter_x = 1.0 / std::max(w, 1);
        double pixel_to_meter_y = 1.0 / std::max(h, 1);

        latest_cam_x_ = rel_x * pixel_to_meter_x;
        latest_cam_y_ = rel_y * pixel_to_meter_y;

        if (annotate) {
            cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);
            cv::circle(image, cv::Point(obj_cx, obj_cy), 5, cv::Scalar(0, 255, 0), -1);
        }
        return true;
    }

    bool detect_once_and_transform(
        const std::string &color,
        double &dx_body, double &dy_body,
        double &dx_world, double &dy_world)
    {
        cv::Mat frame_copy;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (current_frame_.empty()) {
                RCLCPP_WARN(this->get_logger(), "No frame available");
                return false;
            }
            frame_copy = current_frame_.clone();
        }

        {
            std::lock_guard<std::mutex> det_lock(detection_mutex_);
            detection_active_ = true;
            active_color_ = color;
        }

        bool ok = process_frame(frame_copy, color, true);
        if (!ok) {
            return false;
        }

        geometry_msgs::msg::PointStamped point_cam;
        geometry_msgs::msg::PointStamped point_body;

        point_cam.header.stamp = this->get_clock()->now();
        point_cam.header.frame_id = "camera_link";
        point_cam.point.x = latest_cam_x_;
        point_cam.point.y = latest_cam_y_;
        point_cam.point.z = 0.0;

        try {
            auto transform = tf_buffer_->lookupTransform(
                "uav_base_link",
                "camera_link",
                tf2::TimePointZero);

            tf2::doTransform(point_cam, point_body, transform);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", ex.what());
            return false;
        }

        dx_body = point_body.point.x;
        dy_body = point_body.point.y;

        double yaw = 0.0;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            yaw = current_yaw_;
        }

        double cy = std::cos(yaw);
        double sy = std::sin(yaw);
        dx_world = cy * dx_body - sy * dy_body;
        dy_world = sy * dx_body + cy * dy_body;

        return true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const TrackColor::Goal> goal)
    {
        if (color_thresholds_.find(goal->color) == color_thresholds_.end()) {
            RCLCPP_WARN(this->get_logger(), "Reject goal: unsupported color %s", goal->color.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Accept TrackColor goal: color=%s tolerance=%.3f stable=%d timeout=%.2f",
                    goal->color.c_str(),
                    goal->centered_tolerance,
                    goal->stable_count_required,
                    goal->timeout_sec);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTrackColor>)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel TrackColor goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTrackColor> goal_handle) {
        std::thread{std::bind(&ColorDetectionActionServer::execute_track_color, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_track_color(const std::shared_ptr<GoalHandleTrackColor> goal_handle) {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<TrackColor::Feedback>();
        auto result = std::make_shared<TrackColor::Result>();

        rclcpp::Rate rate(1.0 / std::max(0.05f, goal->feedback_interval_sec));
        auto start_time = this->now();
        int stable_count = 0;

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                detection_active_ = false;
                result->success = false;
                result->message = "cancelled";
                result->final_stable_count = stable_count;
                goal_handle->canceled(result);
                return;
            }

            double elapsed = (this->now() - start_time).seconds();
            if (elapsed > goal->timeout_sec) {
                detection_active_ = false;
                result->success = false;
                result->message = "timeout";
                result->final_stable_count = stable_count;
                goal_handle->abort(result);
                return;
            }

            double dx_body = 0.0, dy_body = 0.0, dx_world = 0.0, dy_world = 0.0;
            bool detected = detect_once_and_transform(goal->color, dx_body, dy_body, dx_world, dy_world);

            feedback->detected = detected;
            feedback->offset_x_body = static_cast<float>(dx_body);
            feedback->offset_y_body = static_cast<float>(dy_body);

            double planar_error = std::hypot(dx_body, dy_body);
            feedback->planar_error = static_cast<float>(planar_error);

            if (detected) {
                stable_count = (planar_error < goal->centered_tolerance) ? (stable_count + 1) : 0;
                feedback->stable_count = stable_count;
                feedback->offset_x_world = static_cast<float>(dx_world);
                feedback->offset_y_world = static_cast<float>(dy_world);

                if (stable_count >= goal->stable_count_required) {
                    detection_active_ = false;
                    result->success = true;
                    result->message = "target centered";
                    result->final_offset_x_world = static_cast<float>(dx_world);
                    result->final_offset_y_world = static_cast<float>(dy_world);
                    result->final_stable_count = stable_count;
                    goal_handle->succeed(result);
                    return;
                }
            } else {
                stable_count = 0;
                feedback->stable_count = 0;
                feedback->offset_x_world = 0.0f;
                feedback->offset_y_world = 0.0f;
            }

            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        detection_active_ = false;
        result->success = false;
        result->message = "node shutdown";
        result->final_stable_count = stable_count;
        goal_handle->abort(result);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorDetectionActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}