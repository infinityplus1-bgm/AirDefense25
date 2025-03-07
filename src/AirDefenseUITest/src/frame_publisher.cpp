#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node
{
public:
  VideoPublisher() : Node("video_publisher")
  {
    cap_.open("/home/teknofest/Documents/UITest/assets/test.mp4");
    
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video file!");
      rclcpp::shutdown();
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_frames", 10);
    
    timer_ = this->create_wall_timer(
      33ms, std::bind(&VideoPublisher::timer_callback, this));
  }

  // Add destructor to properly release video capture
  ~VideoPublisher() {
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

    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "video_frame";

    publisher_->publish(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Published video frame");
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoPublisher>());
  rclcpp::shutdown();
  return 0;
}