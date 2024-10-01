#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"  // 위치와 방향을 나타내는 메시지 타입
#include "geometry_msgs/msg/twist.hpp" // 선속도와 각속도를 나타내는 메시지 타입
#include <iostream>
#include <chrono>
using namespace std;

class TargetPoseVelocityInputNode : public rclcpp::Node
{
public:
    TargetPoseVelocityInputNode()
    : Node("target_pose_velocity_input_node")
    {
        // 'target_pose' 토픽으로 Pose 메시지를 퍼블리시할 퍼블리셔 생성
        target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);

        // 'target_velocity' 토픽으로 Twist 메시지를 퍼블리시할 퍼블리셔 생성
        target_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("target_velocity", 10);

        // 사용자 입력 받기
        get_user_input();

        // 받은 입력 값을 퍼블리시
        publish_target_pose_velocity();
    }

private:
    void get_user_input()
    {
        // 현재 시간 계산
        auto current_time = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed_time = current_time - last_time_;

        // 시간 간격 계산 (초 단위)
        double dt = elapsed_time.count();

        // 사용자 입력을 통해 위치를 설정
        cout << "Enter x (position): ";
        cin >> target_pose_.position.x;

        cout << "Enter y (position): ";
        cin >> target_pose_.position.y;

        cout << "Enter z (position): ";
        cin >> target_pose_.position.z;

        double roll, pitch, yaw;
        cout << "Enter roll (orientation in rad): ";
        cin >> roll;

        cout << "Enter pitch (orientation in rad): ";
        cin >> pitch;

        cout << "Enter yaw (orientation in rad): ";
        cin >> yaw;

        // 이전 각도와의 차이를 계산하여 변화율 계산
        droll_ = (roll - prev_roll_) / dt;
        dpitch_ = (pitch - prev_pitch_) / dt;
        dyaw_ = (yaw - prev_yaw_) / dt;

        // 이전 값을 업데이트
        prev_roll_ = roll;
        prev_pitch_ = pitch;
        prev_yaw_ = yaw;
        last_time_ = current_time;

        // RPY의 시간 변화율을 각속도(wx, wy, wz)로 변환
        convert_rpy_to_angular_velocity(droll_, dpitch_, dyaw_);
    }

    void convert_rpy_to_angular_velocity(double droll, double dpitch, double dyaw)
    {
        // RPY를 각속도로 변환하는 자코비안 행렬
        double wx, wy, wz;

        // 자코비안 행렬의 요소들 계산
        wx = droll + dpitch * sin(prev_roll_) * tan(prev_pitch_) + dyaw * cos(prev_roll_) * tan(prev_pitch_);
        wy = dpitch * cos(prev_roll_) - dyaw * sin(prev_roll_);
        wz = dpitch * sin(prev_roll_) / cos(prev_pitch_) + dyaw * cos(prev_roll_) / cos(prev_pitch_);

        // 변환된 각속도를 Twist 메시지의 angular 필드에 저장
        target_velocity_.angular.x = wx;
        target_velocity_.angular.y = wy;
        target_velocity_.angular.z = wz;

        RCLCPP_INFO(this->get_logger(), "Converted RPY to angular velocity: wx=%.2f, wy=%.2f, wz=%.2f", wx, wy, wz);
    }

    void publish_target_pose_velocity()
    {
        // Pose 메시지 퍼블리시
        target_pose_publisher_->publish(target_pose_);
        RCLCPP_INFO(this->get_logger(), "Published target pose");

        // Twist 메시지 퍼블리시
        target_velocity_publisher_->publish(target_velocity_);
        RCLCPP_INFO(this->get_logger(), "Published target velocity");
    }

    // 퍼블리셔 변수
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr target_velocity_publisher_;

    // 위치와 방향을 저장하는 변수
    geometry_msgs::msg::Pose target_pose_;

    // 각속도를 저장하는 변수
    geometry_msgs::msg::Twist target_velocity_;

    // 시간 및 각도 변화율 계산을 위한 변수
    chrono::high_resolution_clock::time_point last_time_ = chrono::high_resolution_clock::now();
    double prev_roll_ = 0.0;
    double prev_pitch_ = 0.0;
    double prev_yaw_ = 0.0;
    double droll_ = 0.0;
    double dpitch_ = 0.0;
    double dyaw_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<TargetPoseVelocityInputNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
