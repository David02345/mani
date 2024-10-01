// Controller.cpp

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
    : Node("controller_node")
    {
        trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "planned_trajectory", 10, std::bind(&ControllerNode::trajectoryCallback, this, std::placeholders::_1));

        command_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/robot_controller/command", 10);
    }

private:
    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // 받은 경로를 로봇에게 명령으로 보냄
        command_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Published trajectory to robot controller.");
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
