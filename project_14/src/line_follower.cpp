#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LineFollowerNode : public rclcpp::Node
{
public:
    LineFollowerNode() : Node("line_follower")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_compressed", 10, std::bind(&LineFollowerNode::image_callback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image message to OpenCV format
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Process the image to follow the line
        // Implement the actual line following logic here

        // Example of a simple line following logic:
        int cx = frame.cols / 2;
        int cy = frame.rows / 2;

        // For simplicity, let's assume the line is detected at the center of the image
        if (true /* replace with actual line detection */)
        {
            // Move forward
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
        }
        else
        {
            // Stop the robot if the line is not detected
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
        }

        // Check if the TurtleBot3 has reached its destination
        if (true /* replace with actual destination detection */)
        {
            RCLCPP_INFO(this->get_logger(), "Destination reached. Printing 'O'");
            std::cout << "O" << std::endl;

            // Stop the robot
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
