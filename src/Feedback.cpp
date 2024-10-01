// Feedback.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class FeedbackNode : public rclcpp::Node
{
public:
    FeedbackNode()
    : Node("feedback_node")
    {
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/robot/joint_states", 10, std::bind(&FeedbackNode::jointStateCallback, this, std::placeholders::_1));

        // 목표 관절 상태를 수신하기 위한 구독자 추가
        target_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&FeedbackNode::targetJointStateCallback, this, std::placeholders::_1));

        // 필요시 제어 퍼블리셔 추가
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 현재 로봇의 관절 상태 수신
        current_joint_positions_ = msg->position;

        // 오차 계산
        if (target_joint_positions_.size() == current_joint_positions_.size())
        {
            std::vector<double> errors;
            for (size_t i = 0; i < current_joint_positions_.size(); ++i)
            {
                double error = target_joint_positions_[i] - current_joint_positions_[i];
                errors.push_back(error);
            }

            // 오차 출력 또는 제어 로직 수행
            RCLCPP_INFO(this->get_logger(), "Joint errors: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                        errors[0], errors[1], errors[2], errors[3], errors[4], errors[5]);

            // 필요시 제어 명령 퍼블리시
            // ...
        }
    }

    void targetJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 목표 관절 상태 수신
        target_joint_positions_ = msg->position;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_subscriber_;
    // rclcpp::Publisher<...>::SharedPtr control_publisher_; // 필요시 추가

    std::vector<double> current_joint_positions_;
    std::vector<double> target_joint_positions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeedbackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
