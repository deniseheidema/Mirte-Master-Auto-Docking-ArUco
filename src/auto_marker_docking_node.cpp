// === AutoMarkerDockingNode (precision adaptive version) ===
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class AutoMarkerDockingNode : public rclcpp::Node {
public:
  AutoMarkerDockingNode()
  : Node("auto_marker_docking_node")
  {
    // === Parameters ===
    declare_parameter<std::string>("camera_topic", "/camera/color/image_raw");
    declare_parameter<std::string>("cmd_vel_topic", "/mirte_base_controller/cmd_vel");
    declare_parameter<int>("aruco_dict_id", 10);
    declare_parameter<double>("marker_size", 0.1);
    declare_parameter<std::vector<double>>("camera_matrix", {543.8, 0.0, 312.5, 0.0, 543.8, 235.1, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>("dist_coeffs", {0, 0, 0, 0, 0});
    declare_parameter<double>("drive_speed", 0.3);
    declare_parameter<double>("stop_distance", 0.50);
    declare_parameter<double>("min_step", 0.10);
    declare_parameter<double>("turn_speed", 0.4);
    declare_parameter<double>("yaw_gain", 1);             // stronger correction
    declare_parameter<double>("yaw_turn_threshold", 0.09);  // ~5 degrees
    declare_parameter<double>("marker_timeout", 1.0);
    declare_parameter<int>("lost_threshold", 5);

    // === Load parameters ===
    get_parameter("camera_topic", camera_topic_);
    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("aruco_dict_id", dict_id_);
    get_parameter("marker_size", marker_size_);
    get_parameter("drive_speed", drive_speed_);
    get_parameter("stop_distance", stop_distance_);
    get_parameter("min_step", min_step_);
    get_parameter("turn_speed", turn_speed_);
    get_parameter("yaw_gain", yaw_gain_);
    get_parameter("yaw_turn_threshold", yaw_turn_threshold_);
    get_parameter("marker_timeout", marker_timeout_);
    get_parameter("lost_threshold", lost_threshold_);

    std::vector<double> K, D;
    get_parameter("camera_matrix", K);
    get_parameter("dist_coeffs", D);
    camera_matrix_ = cv::Mat(3, 3, CV_64F, K.data()).clone();
    dist_coeffs_ = cv::Mat(1, 5, CV_64F, D.data()).clone();

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id_));

    // === ROS interfaces ===
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AutoMarkerDockingNode::image_cb, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    // Run control loop at 30 Hz for smoother correction
    control_timer_ = create_wall_timer(33ms, std::bind(&AutoMarkerDockingNode::control_loop, this));

    // Initialize last marker time (prevent clock mismatch)
    last_marker_time_ = this->get_clock()->now();

    RCLCPP_INFO(get_logger(), "AutoMarkerDockingNode started with precision control");
  }

private:
  // === Marker detection callback ===
  void image_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);

    if (ids.empty()) {
      marker_lost_counter_++;
      return;
    }

    marker_lost_counter_ = 0;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    double yaw = std::atan2(tvecs[0][0], tvecs[0][2]);  // x/z = yaw offset
    double yaw_deg = yaw * 180.0 / M_PI;

    RCLCPP_INFO(get_logger(), "Marker detected: %.2f deg off-center, distance: %.2fm", yaw_deg, tvecs[0][2]);

    last_marker_distance_ = tvecs[0][2];
    last_marker_yaw_ = -yaw;
    last_marker_time_ = this->get_clock()->now();
  }

  // === Control loop ===
  void control_loop() {
    geometry_msgs::msg::Twist cmd;
    auto now = this->get_clock()->now();

    // Safe time subtraction
    double dt_since_last_seen = 0.0;
    if (last_marker_time_.get_clock_type() == now.get_clock_type()) {
      dt_since_last_seen = (now - last_marker_time_).seconds();
    }

    bool marker_valid = (marker_lost_counter_ < lost_threshold_) || (dt_since_last_seen < marker_timeout_);

    if (!marker_valid) {
      // No marker → search rotation
      cmd.angular.z = turn_speed_;
      cmd.linear.x = 0.0;
    } else {
      double distance_to_target = last_marker_distance_ - stop_distance_;

      // === Adaptive linear speed based on proximity ===
      double linear_speed = 0.0;
      if (distance_to_target > 0.5)
        linear_speed = drive_speed_;               // normal speed
      else if (distance_to_target > 0.25)
        linear_speed = drive_speed_ * 0.5;         // slower mid-range
      else
        linear_speed = drive_speed_ * 0.25;        // precision crawl

      // === Control logic ===
      if (distance_to_target > 0.05) {
        if (std::abs(last_marker_yaw_) > yaw_turn_threshold_) {
          cmd.angular.z = std::copysign(turn_speed_, last_marker_yaw_);
          cmd.linear.x = 0.0;
        } else {
          cmd.linear.x = linear_speed;
          cmd.angular.z = yaw_gain_ * last_marker_yaw_;
        }
      } else {
        // Close enough: fine yaw alignment before stopping
        if (std::abs(last_marker_yaw_) > 0.09) {  // ~5 degrees
          cmd.linear.x = 0.0;
          cmd.angular.z = yaw_gain_ * last_marker_yaw_;
        } else {
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.0;
        }
      }
    }

    cmd_pub_->publish(cmd);
  }

  // === Member variables ===
  std::string camera_topic_, cmd_vel_topic_;
  int dict_id_, lost_threshold_;
  double marker_size_, drive_speed_, stop_distance_, min_step_;
  double turn_speed_, yaw_gain_, yaw_turn_threshold_, marker_timeout_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  cv::Mat camera_matrix_, dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  double last_marker_distance_ = 999.0;
  double last_marker_yaw_ = 0.0;
  int marker_lost_counter_ = 999;
  rclcpp::Time last_marker_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoMarkerDockingNode>());
  rclcpp::shutdown();
  return 0;
}
