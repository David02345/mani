// MotionPlanning.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class MotionPlanningNode : public rclcpp::Node
{
public:
    MotionPlanningNode()
    : Node("motion_planning_node")
    {
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&MotionPlanningNode::jointStateCallback, this, std::placeholders::_1));

        trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "planned_trajectory", 10);

        // 현재 관절 상태를 초기화 (예: 0으로 초기화)
        current_joint_positions_ = std::vector<double>(6, 0.0);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 받은 관절 상태를 목표 상태로 설정
        std::vector<double> target_joint_positions = msg->position;

        // 경로 생성 (예: 선형 보간)
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}; // 실제 관절 이름으로 대체

        int num_points = 50;
        for (int i = 1; i <= num_points; ++i)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            for (size_t j = 0; j < current_joint_positions_.size(); ++j)
            {
                double position = current_joint_positions_[j] + (target_joint_positions[j] - current_joint_positions_[j]) * i / num_points;
                point.positions.push_back(position);
                // 속도 및 가속도는 필요에 따라 계산
            }
            point.time_from_start = rclcpp::Duration::from_seconds(i * 0.1); // 각 포인트 사이의 시간 간격 설정
            trajectory.points.push_back(point);
        }

        // 경로 퍼블리시
        trajectory_publisher_->publish(trajectory);
        RCLCPP_INFO(this->get_logger(), "Published planned trajectory.");

        // 현재 관절 상태 업데이트
        current_joint_positions_ = target_joint_positions;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

    std::vector<double> current_joint_positions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
