#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "turtlesim/msg/pose.hpp"

class LineFollowingRobot : public rclcpp::Node
{
public:
    LineFollowingRobot() : Node("line_follower")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

        // 카메라 1과 2를 위한 이미지 구독 설정
        cam1_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "tb3_0/image_raw/compressed", qos_profile,
            std::bind(&LineFollowingRobot::sub_img1, this, std::placeholders::_1));

        cam2_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "tb3_1/image_raw/compressed", qos_profile,
            std::bind(&LineFollowingRobot::sub_img2, this, std::placeholders::_1));

        // 두 로봇에 대한 속도 출판 설정
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/tb3_0/cmd_vel", 10);
        cmd_pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        _sub = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", qos_profile, std::bind(&LineFollowingRobot::sub_turtlesim_pose, this, std::placeholders::_1));
        // V4L2 카메라 실행 명령 설정
        // system("v4l2-ctl --set-fmt-video=width=640,height=480,pixelformat=1 --stream-mmap --stream-count=60 --stream-to=/dev/null");

        // 타이머 설정 (60프레임/초)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(16),
            std::bind(&LineFollowingRobot::update, this));
    }

private:
    int _i;
    geometry_msgs::msg::Twist cmd_msg_0;
    geometry_msgs::msg::Twist cmd_msg_1;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr cam1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr cam2_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub1_;
    rclcpp::TimerBase::SharedPtr timer_;

    turtlesim::msg::Pose _pose_msg;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr _sub;

    void sub_turtlesim_pose(const turtlesim::msg::Pose::SharedPtr msg);

    cv::Mat img1_, img2_;

    // 카메라 1 이미지 처리
    void sub_img1(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img1_ = cv_ptr->image;
    }

    // 카메라 2 이미지 처리
    void sub_img2(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img2_ = cv_ptr->image;
    }

    // 센서와 액추에이터 데이터를 통합하여 처리
    void update()
    {
        // if (!img1_.empty() && !img2_.empty())
        // {
        //     // 이미지 1 처리 (예시로만 보여주는 코드)
        //     process_image(img1_, cmd_msg_0);

        //     // 이미지 2 처리 (예시로만 보여주는 코드)
        //     process_image(img2_, cmd_msg_1);
        // }

        // turtlebot3 move
        cmd_msg_0.linear.x = 0.1;
        cmd_msg_0.angular.z = 1.0;

        // cmd_msg_1.linear.x = 0.05;
        // cmd_msg_1.angular.z = 1.0;
        //  사각형으로 움직이기.
        switch (_i)
        {
        case 0:
            if (_pose_msg.x < 6.5)
            {
                // 직진
                cmd_msg_1.linear.x = 0.1;
                cmd_msg_1.angular.z = 0.0;
            }
            else if (_pose_msg.theta < 1.57)
            {
                // 회전
                cmd_msg_1.linear.x = 0.0;
                cmd_msg_1.angular.z = 1.8;
            }
            else
            {
                _i++;
            }
            break;
        case 1:
            if (_pose_msg.y < 6.5)
            {
                // 직진
                cmd_msg_1.linear.x = 0.1;
                cmd_msg_1.angular.z = 0.0;
            }
            else if (_pose_msg.theta < 3.14 && _pose_msg.theta > 0)
            {
                // 회전
                cmd_msg_1.linear.x = 0.0;
                cmd_msg_1.angular.z = 1.8;
            }
            else
            {
                _i++;
            }
            break;
        case 2:
            if (_pose_msg.x > 5.5)
            {
                // 직진
                cmd_msg_1.linear.x = 0.1;
                cmd_msg_1.angular.z = 0.0;
            }
            else if (_pose_msg.theta < -1.57)
            {
                // 회전
                cmd_msg_1.linear.x = 0.0;
                cmd_msg_1.angular.z = 1.8;
            }
            else
            {
                _i++;
            }
            break;
        case 3:
            if (_pose_msg.y > 5.5)
            {
                // 직진
                cmd_msg_1.linear.x = 0.1;
                cmd_msg_1.angular.z = 0.0;
            }
            else if (_pose_msg.theta < 0)
            {
                // 회전
                cmd_msg_1.linear.x = 0.0;
                cmd_msg_1.angular.z = 1.8;
            }
            else
            {
                _i = 0;
            }
            break;
        }

        cmd_pub_->publish(cmd_msg_0);
        cmd_pub1_->publish(cmd_msg_1);
    }

    // void process_image(cv::Mat& img, geometry_msgs::msg::Twist &cmd_msg)
    // {
    //     // 그레이스케일로 변환
    //     cv::Mat gray;
    //     cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    //     // 양자화 적용
    //     int num_levels = 4;
    //     cv::Mat quantized_gray = gray / (256 / num_levels);  // 이미지 값 범위를 [0, num_levels)로 조정
    //     quantized_gray = quantized_gray * (256 / num_levels);  // 원래 범위로 복원

    //     // 이진화
    //     cv::Mat binary;
    //     cv::threshold(quantized_gray, binary, 127, 255, cv::THRESH_BINARY_INV);

    //     // 관심 영역(ROI) 설정
    //     int height = binary.rows;
    //     int width = binary.cols;
    //     cv::Mat roi = binary(cv::Rect(0, height / 2, width, height / 2));

    //     // 윤곽선 검출
    //     std::vector<std::vector<cv::Point>> contours;
    //     cv::findContours(roi, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    //     if (!contours.empty())
    //     {
    //         // 가장 큰 윤곽선 찾기
    //         auto max_contour = std::max_element(contours.begin(), contours.end(),
    //         [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
    //         {return cv::contourArea(a) < cv::contourArea(b);});

    //         // 무게중심 계산
    //         cv::Moments M = cv::moments(*max_contour);
    //         int cx = static_cast<int>(M.m10 / M.m00);

    //         // 중심선과 프레임의 중심 비교
    //         int error = cx - width / 2;

    //         // 기본 주행 명령
    //         cmd_msg.linear.x = 0.1;
    //         cmd_msg.angular.z = 0.0;

    //         // 방향 조정
    //         if (error > 10)  // 허용 오차를 두고 오른쪽으로 돌도록 설정
    //         {
    //             cmd_msg.linear.x = -0.1; // 오른쪽으로 회전
    //         }
    //         else if (error < -10)  // 허용 오차를 두고 왼쪽으로 돌도록 설정
    //         {
    //             cmd_msg.angular.z = 0.5;  // 왼쪽으로 회전
    //         }
    //     }
    // }

    void publish_motor_speed(float linear_speed, float angular_speed)
    {

        // 첫 번째 로봇에 대한 명령 설정
        cmd_msg_0.linear.x = linear_speed;
        cmd_msg_0.angular.z = angular_speed;
    }

    void sub_turtlesim_pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        _pose_msg = *msg;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<LineFollowingRobot>(); // 로봇 노드 객체 생성
    rclcpp::spin(robot);                                 // 노드 실행
    rclcpp::shutdown();                                  // 종료
    return 0;
}
