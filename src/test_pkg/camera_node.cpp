#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
    : Node("camera_node")
    {
        this->declare_parameter<std::string>("video_path", "0");
        this->declare_parameter<double>("timer_period", 0.08);
        this->declare_parameter<int>("zoom_factor", 1);
        this->declare_parameter<int>("laser_center_x", 635);
        this->declare_parameter<int>("laser_center_y", 370);

        std::string video_path = this->get_parameter("video_path").as_string();
        double timer_period = this->get_parameter("timer_period").as_double();
        zoom_factor_ = this->get_parameter("zoom_factor").as_int();
        laser_center_x_ = this->get_parameter("laser_center_x").as_int();
        laser_center_y_ = this->get_parameter("laser_center_y").as_int();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        status_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/system/status", 10, std::bind(&CameraNode::status_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period), std::bind(&CameraNode::timer_callback, this));

        health_publisher_ = this->create_publisher<std_msgs::msg::String>("/health/camera_node", 10);
        health_timer_ = this->create_wall_timer(1s, std::bind(&CameraNode::publish_health_status, this));

        if (video_path.find("/dev/video") != std::string::npos) {
            cap_.open(video_path, cv::CAP_V4L2);
        } else {
            try {
                cap_.open(std::stoi(video_path));
            } catch (const std::invalid_argument&) {
                cap_.open(video_path);
            }
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open video source: %s", video_path.c_str());
            rclcpp::shutdown();
        }
    }

private:
    void status_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        system_enabled_ = (msg->data != 0);
    }

    void timer_callback()
    {
        if (!system_enabled_) {
            return;
        }

        cv::Mat frame;
        cap_ >> frame;

        if (!frame.empty()) {
            cv::Mat resized_frame;
            cv::resize(frame, resized_frame, cv::Size(frame.cols * zoom_factor_, frame.rows * zoom_factor_), 0, 0, cv::INTER_LINEAR);

            int start_x = (resized_frame.cols / 2) - (frame.cols / 2);
            int start_y = (resized_frame.rows / 2) - (frame.rows / 2);
            cv::Mat cropped_frame = resized_frame(cv::Rect(start_x, start_y, frame.cols, frame.rows));

            cv::line(cropped_frame, cv::Point(laser_center_x_ - 20, laser_center_y_), cv::Point(laser_center_x_ + 20, laser_center_y_), cv::Scalar(0, 255, 0), 1);
            cv::line(cropped_frame, cv::Point(laser_center_x_, laser_center_y_ - 20), cv::Point(laser_center_x_, laser_center_y_ + 20), cv::Scalar(0, 255, 0), 1);

            publisher_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cropped_frame).toImageMsg());
        }
    }

    void publish_health_status()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "healthy";
        health_publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_publisher_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    cv::VideoCapture cap_;
    bool system_enabled_ = false;
    int zoom_factor_;
    int laser_center_x_;
    int laser_center_y_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
