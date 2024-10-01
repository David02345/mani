// IKSolver.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace std;
using namespace Eigen;

class IKSolverNode : public rclcpp::Node
{
public:
    IKSolverNode()
    : Node("ik_solver_node")
    {
        target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_pose", 10, std::bind(&IKSolverNode::targetPoseCallback, this, std::placeholders::_1));

        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // 초기 관절 각도 설정 (예: 0으로 초기화)
        current_joint_positions_ = VectorXd::Zero(6);
    }

private:
    void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        geometry_msgs::msg::Pose target_pose = *msg;

        // 역기구학 계산
        VectorXd joint_positions = computeIK(target_pose);

        if (joint_positions.size() == 6)
        {
            // 관절 각도 퍼블리시
            sensor_msgs::msg::JointState joint_state;
            joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}; // 실제 관절 이름으로 대체
            joint_state.position = {joint_positions(0), joint_positions(1), joint_positions(2),
                                    joint_positions(3), joint_positions(4), joint_positions(5)};
            joint_state_publisher_->publish(joint_state);
            RCLCPP_INFO(this->get_logger(), "IK solution found and published joint states.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to find IK solution.");
        }
    }

    VectorXd computeIK(const geometry_msgs::msg::Pose& target_pose)
    {
        // DH 파라미터 설정
        double pi = M_PI;
        double d[6] = {0, 0, 0, 0.48, 0, 0.14};
        double a[6] = {0, 0.5, 0, 0, 0, 0};
        double alpha[6] = {-pi / 2, 0, -pi / 2, pi / 2, -pi / 2, 0};

        VectorXd theta = current_joint_positions_;

        // 목표 위치 설정
        Vector3d target_position(target_pose.position.x, target_pose.position.y, target_pose.position.z);

        // 반복 역기구학 (예: Newton-Raphson 방법)
        int max_iterations = 100;
        double tolerance = 1e-4;
        double lambda = 0.3;

        for (int i = 0; i < max_iterations; ++i)
    {
        // 현재 포워드 키네매틱스 계산
        Matrix4d T = forwardKinematics(theta, d, a, alpha);
        Vector3d current_position = T.block<3,1>(0,3);

        // 위치 오차 계산
        Vector3d position_error = target_position - current_position;

        if (position_error.norm() < tolerance)
        {
            RCLCPP_INFO(this->get_logger(), "IK converged in %d iterations.", i);
            current_joint_positions_ = theta;
            return theta;
        }

        // 자코비안 계산
        Matrix<double, 6, 6> J = computeJacobian(theta, d, a, alpha);

        // 위치에 대한 자코비안 추출 (상위 3행)
        Matrix<double, 3, 6> J_pos = J.topRows(3);

        // 자코비안 유사역행렬 계산
        Matrix<double, 6, 3> J_DPI = J_pos.transpose() * (J_pos * J_pos.transpose() + lambda * lambda * Matrix3d::Identity()).inverse();

        // 관절 각도 업데이트
        VectorXd delta_theta = J_DPI * position_error;
        theta += delta_theta;
    }

    RCLCPP_WARN(this->get_logger(), "IK did not converge.");
    return VectorXd(); // 실패 시 빈 벡터 반환
}

    Matrix4d forwardKinematics(const VectorXd& theta, const double d[6], const double a[6], const double alpha[6])
    {
        // 각 링크에 대한 변환 행렬 계산
        Matrix4d T = Matrix4d::Identity();
        for (int i = 0; i < 6; ++i)
        {
            Matrix4d Ti;
            Ti << cos(theta(i)), -sin(theta(i))*cos(alpha[i]), sin(theta(i))*sin(alpha[i]), a[i]*cos(theta(i)),
                  sin(theta(i)), cos(theta(i))*cos(alpha[i]), -cos(theta(i))*sin(alpha[i]), a[i]*sin(theta(i)),
                  0, sin(alpha[i]), cos(alpha[i]), d[i],
                  0, 0, 0, 1;
            T *= Ti;
        }
        return T;
    }

    Matrix<double, 6, 6> computeJacobian(const VectorXd& theta, const double d[6], const double a[6], const double alpha[6])
    {
        // 제공해주신 자코비안 계산 함수 구현
        // ... (사용자의 J 함수 내용 구현)
        // 이 부분은 사용자의 코드를 그대로 옮겨오거나 필요한 부분만 수정하여 사용하면 됩니다.
        // 여기서는 간단히 placeholder로 두겠습니다.

        Matrix<double, 6, 6> Jacobian = Matrix<double, 6, 6>::Zero();

        // 자코비안 계산 코드 작성
        // ...

        return Jacobian;
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    VectorXd current_joint_positions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
