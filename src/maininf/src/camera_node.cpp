#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera")
  {
    // Open the video file
    cap_.open("/home/abdurrahman/AirDefense25/sample.mp4");
    
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video file!");
      rclcpp::shutdown();
    }

    // Create a publisher for the topic "camera/image_raw"
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    
    // Create a timer to publish frames at ~30 FPS (33ms interval)
    timer_ = this->create_wall_timer(
      33ms, std::bind(&CameraNode::timer_callback, this));
  }

  // Destructor to release video capture
  ~CameraNode() {
    if(cap_.isOpened()) {
      cap_.release();
      RCLCPP_INFO(this->get_logger(), "Video capture released");
    }
  }

private:
  void timer_callback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "End of video stream");
      rclcpp::shutdown();
      return;
    }

    // Convert the OpenCV frame to a ROS 2 image message
    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_frame";

    // Publish the image message
    publisher_->publish(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Published video frame to /camera/image_raw");
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}