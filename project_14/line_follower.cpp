#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class LineFollowerNode : public rclcpp::Node {
public:
    LineFollowerNode() : Node("line_follower") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&LineFollowerNode::processImage, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Convert to grayscale and threshold to get the binary image
        cv::Mat gray, binary;
        cv::cvtColor(cv_image, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);

        // Find the contours of the black line
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Assuming the largest contour is the line
            auto largest_contour = std::max_element(
                contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            // Get the centroid of the largest contour
            cv::Moments M = cv::moments(*largest_contour);
            double cx = M.m10 / M.m00;
            double cy = M.m01 / M.m00;

            // Generate control commands based on the position of the centroid
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.2;  // Move forward at a constant speed

            // Proportional control for steering
            double error = cx - (cv_image.cols / 2);
            cmd_vel_msg.angular.z = -error / 100;  // Adjust this constant for smoother turning

            publisher_->publish(cmd_vel_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "No line detected");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowerNode>());
    rclcpp::shutdown();
    return 0;
}