#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoSimpleDetector : public rclcpp::Node {
public:
  ArucoSimpleDetector() : Node("aruco_simple_detector")
  {
    // hardcode to same topic as your camera
    camera_topic_ = "/camera/color/image_raw";
    dictionary_id_ = 10;  // 6x6_250 dictionary (same as original project)

    RCLCPP_INFO(this->get_logger(), "Subscribing to camera topic: %s", camera_topic_.c_str());

    // Load predefined ArUco dictionary
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id_);

    // Create image subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 10,
      std::bind(&ArucoSimpleDetector::image_callback, this, std::placeholders::_1)
    );
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      // Convert to OpenCV format
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    // Detect markers
    cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids);

    if (!marker_ids.empty()) {
      RCLCPP_INFO(this->get_logger(), "Detected %zu markers", marker_ids.size());
      for (size_t i = 0; i < marker_ids.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Marker ID: %d", marker_ids[i]);
      }

      // Draw detections for debug
      cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
      cv::imshow("Aruco Detection", image);
      cv::waitKey(1);
    }
  }

  std::string camera_topic_;
  int dictionary_id_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoSimpleDetector>());
  rclcpp::shutdown();
  return 0;
}
