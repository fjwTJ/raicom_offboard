#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "px4_msgs/srv/detect_color.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <mutex>
#include <map>
#include <atomic>
#include <vector>
#include <algorithm>
#include <cmath>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "fix-tf-static.hpp"

using namespace std::chrono_literals;
using DetectColor = px4_msgs::srv::DetectColor;

class ColorDetectionService : public rclcpp::Node {
public:
    ColorDetectionService() : Node("color_detection_service") {
        // 创建颜色检测服务
        service_ = this->create_service<DetectColor>(
            "detect_color",
            std::bind(&ColorDetectionService::handle_detect_color, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        // 订阅相机画面
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&ColorDetectionService::image_callback, this, std::placeholders::_1));

        // 订阅PX4姿态信息
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", 10,
            std::bind(&ColorDetectionService::odom_callback, this, std::placeholders::_1));

        // TF 相关初始化
         static_tf_helper_ = std::make_shared<StaticCameraTFHelper>(this);

         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 发布静态相机 -> 机体变换
        // 后续需要改参数
         static_tf_helper_->publishCameraToBodyTF(
        "uav_base_link",   // 父坐标系：机体
         "camera_link",     // 子坐标系：相机
        0.0, 0.0, 0.0,     // x y z
         0.0, 0.0, 0.0      // roll pitch yaw);

        // 初始化颜色阈值映射表
        color_thresholds_["red"]   = std::make_pair(cv::Scalar(0, 200, 150),  cv::Scalar(10, 255, 230));
        color_thresholds_["green"] = std::make_pair(cv::Scalar(50, 200, 150), cv::Scalar(70, 255, 230));
        color_thresholds_["blue"]  = std::make_pair(cv::Scalar(110, 200, 150),cv::Scalar(130, 255, 230));
        color_thresholds_["black"] = std::make_pair(cv::Scalar(0, 0, 50),     cv::Scalar(180, 50, 100));
        
        cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "Color detection service ready, waiting for requests...");
    }

    ~ColorDetectionService() {
        cv::destroyAllWindows();
    }

private:
    // --- 新增 TF 成员 --- 
    std::shared_ptr<StaticCameraTFHelper> static_tf_helper_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // --- ROS 组件 ---
    rclcpp::Service<DetectColor>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    // --- 图像数据 ---
    cv::Mat current_frame_;
    std::mutex frame_mutex_;

    // --- 姿态缓存 ---
    double current_yaw_ = 0.0;
    std::mutex odom_mutex_;

    // --- 检测状态 ---
    std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_thresholds_;
    std::atomic<bool> detection_active_{false};
    std::string active_color_;
    std::mutex detection_mutex_;
    double service_x_;
    double service_y_;

    // -------------------- 图像回调 --------------------
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
                process_frame(display_frame, active_color_);
            }

            cv::imshow("Camera Feed", display_frame);
            cv::waitKey(1);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // -------------------- 姿态回调 --------------------
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        double q0 = msg->q[0];
        double q1 = msg->q[1];
        double q2 = msg->q[2];
        double q3 = msg->q[3];
        double yaw = std::atan2(2.0 * (q0*q3 + q1*q2),
                                1.0 - 2.0 * (q2*q2 + q3*q3));
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_yaw_ = yaw;
    }

    // -------------------- 颜色检测 --------------------
    bool process_frame(cv::Mat &image, const std::string &color) {
        if (color_thresholds_.find(color) == color_thresholds_.end()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Unsupported color: %s", color.c_str());
            return false;
        }

        auto &thresholds = color_thresholds_[color];
        cv::Mat hsv_img, mask_color;
        cv::cvtColor(image, hsv_img, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_img, thresholds.first, thresholds.second, mask_color);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::morphologyEx(mask_color, mask_color, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask_color, mask_color, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_color, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        if (contours.empty()) {
            cv::putText(image, "No " + color + " object", cv::Point(20,40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,255), 2);
            return false;
        }

        auto max_contour = *std::max_element(contours.begin(), contours.end(),
            [](const auto &a, const auto &b){ return cv::contourArea(a) < cv::contourArea(b); });
        cv::Rect rect = cv::boundingRect(max_contour);
        int x = rect.x, y = rect.y, w = rect.width, h = rect.height;
        double img_cx = image.cols / 2.0, img_cy = image.rows / 2.0;
        double obj_cx = x + w / 2.0, obj_cy = y + h / 2.0;

        double rel_x = -(obj_cy - img_cy); // 上为正
        double rel_y =  (obj_cx - img_cx); // 右为正
   
        // 计算像素到米的比例（物体实际尺寸1m x 1m）
        double pixel_to_meter_x = 1.0 / w;
        double pixel_to_meter_y = 1.0 / h;

        // 相机系平面下的位移
        service_x_ = rel_x * pixel_to_meter_x;
        service_y_ = rel_y * pixel_to_meter_y;

        cv::rectangle(image, rect, cv::Scalar(0,255,0), 2);
        cv::circle(image, cv::Point(obj_cx, obj_cy), 5, cv::Scalar(0,255,0), -1);
        return true;
    }

    // -------------------- 服务回调 --------------------
    void handle_detect_color(const std::shared_ptr<DetectColor::Request> request,
                             std::shared_ptr<DetectColor::Response> response) {
        std::string color = request->color;
        RCLCPP_INFO(this->get_logger(), "Service request for color: %s", color.c_str());

        if (color_thresholds_.find(color) == color_thresholds_.end()) {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Unsupported color: %s", color.c_str());
            return;
        }

        cv::Mat frame_copy;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (current_frame_.empty()) {
                response->success = false;
                RCLCPP_WARN(this->get_logger(), "No frame available for service request");
                return;
            }
            frame_copy = current_frame_.clone();
        }

        {
            std::lock_guard<std::mutex> det_lock(detection_mutex_);
            detection_active_ = true;
            active_color_ = color;
        }

        bool success = process_frame(frame_copy, color);

       //相机->机体->世界坐标变换
        if (success) {
           geometry_msgs::msg::PointStamped point_cam;
           geometry_msgs::msg::PointStamped point_body;

             // 1. 把检测结果封装成 camera_link 下的点
    point_cam.header.stamp = this->get_clock()->now();
    point_cam.header.frame_id = "camera_link";
    point_cam.point.x = service_x_;
    point_cam.point.y = service_y_;
    point_cam.point.z = 0.0;   // 当前先按二维平面处理

    // 2. 查询 camera_link -> uav_base_link 的变换
    try {
        auto transform = tf_buffer_->lookupTransform(
            "uav_base_link",   // 目标坐标系
            "camera_link",     // 源坐标系
            tf2::TimePointZero);

        // 3. 把点从相机系变到机体系
        tf2::doTransform(point_cam, point_body, transform);
    } catch (const tf2::TransformException & ex) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", ex.what());
        return;
    }

    // 4. 现在才真正得到机体系偏移
    double dx_body = point_body.point.x;
    double dy_body = point_body.point.y;

    // --- 获取当前yaw ---
    double yaw = 0.0;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        yaw = current_yaw_;
    }

    // --- 机体 -> 世界NED ---
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double dx_world = cy * dx_body - sy * dy_body;
    double dy_world = sy * dx_body + cy * dy_body;

    response->x = dx_world;
    response->y = dy_world;
    response->success = true;

    RCLCPP_INFO(this->get_logger(),
        "Detected %s | Camera(%.2f, %.2f) -> Body(%.2f, %.2f) -> World(%.2f, %.2f), yaw=%.1f°",
        color.c_str(),
        service_x_, service_y_,
        dx_body, dy_body,
        dx_world, dy_world,
        yaw * 180.0 / M_PI);
} else {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "No %s object detected", color.c_str());
}
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorDetectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}