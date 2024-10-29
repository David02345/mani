#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "Eigen/Dense"
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "angles/angles.h"

using namespace std;
using namespace Eigen;

double pi = M_PI;
double theta[6];
double d[6] = {0.7, 0.2, 0.2, 0.48, 0, 0.11};
double a[6] = {0, 0.5, 0, 0, 0, 0};
double alpha[6] = {pi / 2, -pi, -pi / 2, pi / 2, -pi / 2, 0};
const double position_tolerance = 0.01;   // 1 cm tolerance
const double orientation_tolerance = 0.0005; // ~0.57 degrees

Matrix<double, 4, 4> compute_transform_matrix(double theta, double d, double a, double alpha) {
    Matrix<double, 4, 4> T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
}
template <typename Derived>
void epsilon(MatrixBase<Derived>& M, double epsilon = 1e-10) {
    M = M.unaryExpr([epsilon](double val) { return std::abs(val) < epsilon ? 0.0 : val; });
}

class ManipulatorController : public rclcpp::Node
{
public:
    ManipulatorController()
        : Node("manipulator_controller"), loop_rate_(std::chrono::milliseconds(100)) // 10Hz 주기 설정
    {
        target_pose_.position.x = 0.943553;
        target_pose_.position.y = 0.000000;
        target_pose_.position.z = 0.346447;
        target_roll_ = 3.141593;
        target_pitch_ = -1.570796;
        target_yaw_ = 0.000000;

        target_orientation_.setRPY(target_roll_, target_pitch_, target_yaw_);
        target_orientation_.normalize();

        current_pose_.position.x = 0.0;
        current_pose_.position.y = 0.0;
        current_pose_.position.z = 0.0;
        current_pose_.orientation.x = 0.0;
        current_pose_.orientation.y = 0.0;
        current_pose_.orientation.z = 0.0;
        current_pose_.orientation.w = 1.0;

        std::fill(current_joint_positions_, current_joint_positions_ + 6, std::numeric_limits<double>::quiet_NaN());

        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ManipulatorController::joint_state_callback, this, std::placeholders::_1));

        joint_command_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/mani_arm_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(loop_rate_, std::bind(&ManipulatorController::control_loop, this));
    }

private:
    geometry_msgs::msg::Pose current_pose_;
    double current_joint_positions_[6];
    tf2::Quaternion current_orientation_;
    tf2::Quaternion target_orientation_;
    double target_roll_, target_pitch_, target_yaw_;
    double current_roll, current_pitch, current_yaw;
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", msg->name[i].c_str(), msg->position[i]);
            if (msg->name[i] == "Revolute_1")
                current_joint_positions_[0] = msg->position[i];
            else if (msg->name[i] == "Revolute_2")
                current_joint_positions_[1] = msg->position[i];
            else if (msg->name[i] == "Revolute_3")
                current_joint_positions_[2] = msg->position[i];
            else if (msg->name[i] == "Revolute_4")
                current_joint_positions_[3] = msg->position[i];
            else if (msg->name[i] == "Revolute_5")
                current_joint_positions_[4] = msg->position[i];
            else if (msg->name[i] == "Revolute_6")
                current_joint_positions_[5] = msg->position[i];
        }
        RCLCPP_INFO(this->get_logger(), "\n  Target Pose: x=%f, y=%f, z=%f", target_pose_.position.x, target_pose_.position.y, target_pose_.position.z);
        RCLCPP_INFO(this->get_logger(), "Current Pose: x=%f, y=%f, z=%f", current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
        RCLCPP_INFO(this->get_logger(), "\n  Target RPY: r=%f, p=%f, y=%f", target_roll_, target_pitch_, target_yaw_);
        RCLCPP_INFO(this->get_logger(),"Current RPY: r=%f, p=%f, y=%f", current_roll, current_pitch, current_yaw);

     }

    void control_loop()
    {
        bool joint_initialized = true;
        for (int i = 0; i < 6; ++i)
        {
            if (std::isnan(current_joint_positions_[i]))
            {
                joint_initialized = false;
                break;
            }
        }

        if (!joint_initialized)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for joint state initialization...");
            return;
        }

        for (int i = 0; i < 6; ++i)
        {
            theta[i] = current_joint_positions_[i];
        }

        compute_forward_kinematics();

        Vector3d position_error_vec(target_pose_.position.x - current_pose_.position.x,
                                    target_pose_.position.y - current_pose_.position.y,
                                    target_pose_.position.z - current_pose_.position.z);
        double position_error = position_error_vec.norm();

        tf2::Quaternion q_current(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        q_current.normalize(); // 쿼터니언 정규화

        // 쿼터니언을 RPY로 변환
        
        tf2::Matrix3x3(q_current).getRPY(current_roll, current_pitch, current_yaw);

        
         tf2::Quaternion q_error = target_orientation_ * q_current.inverse();
        q_error.normalize();

        // 에러 쿼터니언을 회전 벡터로 변환
        double angle = q_error.getAngle();
        tf2::Vector3 axis = q_error.getAxis();
        Vector3d orientation_error_vec = angle * Vector3d(axis.x(), axis.y(), axis.z());
        double orientation_error = orientation_error_vec.norm();

        if (position_error < position_tolerance && orientation_error < orientation_tolerance)
        {
            RCLCPP_INFO(this->get_logger(), "\n\nGoal reached!\n\n");
            timer_->cancel();
            joint_state_subscriber_.reset();
            return;
        }

        Matrix<double, 6, 6> J_ = compute_jacobian(theta, d, a, alpha);
        Matrix<double, 6, 6> J_DPI = compute_inverse_jacobian(J_);

        Matrix<double, 6, 1> error;
        error.head<3>() = position_error_vec;
        error.tail<3>() = orientation_error_vec;

        Matrix<double, 6, 6> K_p = MatrixXd::Zero(6,6);
        K_p.block<3,3>(0,0) = 5.0 * Matrix3d::Identity(); // Position gains
        K_p.block<3,3>(3,3) = 5.0 * Matrix3d::Identity(); // Orientation gains

        Matrix<double, 6, 1> dq = J_DPI * (K_p * error);

        Matrix<double, 6, 1> q_current_;
        for (int i = 1; i < 6; ++i) {
            q_current_(i) = current_joint_positions_[i];
        }
        q_current_(0) = current_joint_positions_[0] + pi/2;

        Matrix<double, 6, 1> q_desired;
        q_desired = q_current_ + (dq * dt_);

        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        joint_trajectory.joint_names = {"Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5", "Revolute_6"};

        double joint_limits_upper[6] = {3.141593, 3.141593, 0.0, 3.141593, 3.141593, 3.141593};
        double joint_limits_lower[6] = {-3.141593, -3.141593, -6.283185, -3.141593, -3.141593, -1.570796};
        
        double new_position = q_desired(0) - pi/2;

            if (new_position > joint_limits_upper[0])
                new_position = joint_limits_upper[0];
            if (new_position < joint_limits_lower[0])
                new_position = joint_limits_lower[0];

            point.positions.push_back(new_position);
        for (int i = 1; i < 6; ++i)
        {
            double new_position = q_desired(i);

            if (new_position > joint_limits_upper[i])
                new_position = joint_limits_upper[i];
            if (new_position < joint_limits_lower[i])
                new_position = joint_limits_lower[i];

            point.positions.push_back(new_position);
        }
        point.time_from_start = rclcpp::Duration::from_seconds(0.05);
        joint_trajectory.points.push_back(point);

        joint_command_publisher_->publish(joint_trajectory);

        last_time_ = rclcpp::Clock().now();
    }

    void compute_forward_kinematics()
    {
        for(int i=0; i<6; i++){
            theta[i] = current_joint_positions_[i];
        }
        theta[0] += pi/2;

        Matrix<double, 4, 4> T01 = compute_transform_matrix(theta[0], d[0], a[0], alpha[0]);
        Matrix<double, 4, 4> T12 = compute_transform_matrix(theta[1], d[1], a[1], alpha[1]);
        Matrix<double, 4, 4> T23 = compute_transform_matrix(theta[2], d[2], a[2], alpha[2]);
        Matrix<double, 4, 4> T34 = compute_transform_matrix(theta[3], d[3], a[3], alpha[3]);
        Matrix<double, 4, 4> T45 = compute_transform_matrix(theta[4], d[4], a[4], alpha[4]);
        Matrix<double, 4, 4> T56 = compute_transform_matrix(theta[5], d[5], a[5], alpha[5]);

        epsilon(T01);
        epsilon(T12);
        epsilon(T23);
        epsilon(T34);
        epsilon(T45);
        epsilon(T56);

        Matrix<double, 4, 4> D6 = T01 * T12 * T23 * T34 * T45 * T56;
        epsilon(D6);

        double x = D6(0, 3);
        double y = D6(1, 3);
        double z = D6(2, 3);

        Matrix3d rotation_matrix = D6.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation_matrix);
    
        current_pose_.position.x = x;
        current_pose_.position.y = y;
        current_pose_.position.z = z;

        current_pose_.orientation.x = quaternion.x();
        current_pose_.orientation.y = quaternion.y();
        current_pose_.orientation.z = quaternion.z();
        current_pose_.orientation.w = quaternion.w();
        
        // current_orientation_ 업데이트
        current_orientation_.setX(quaternion.x());
        current_orientation_.setY(quaternion.y());
        current_orientation_.setZ(quaternion.z());
        current_orientation_.setW(quaternion.w());
    }

    Matrix<double, 6, 6> compute_jacobian(double theta[6], double d[6], double a[6], double alpha[6])
    {
        // 각 변환 행렬 계산
    Matrix<double, 4, 4> T[6];
    for (int i = 0; i < 6; i++) {
        T[i] = compute_transform_matrix(theta[i], d[i], a[i], alpha[i]);
    }

    // 누적 변환 행렬과 회전 축, 위치 벡터 추출
    Matrix<double, 4, 4> D[7]; // D[0]은 단위 행렬
    D[0] = Matrix4d::Identity();
    Vector3d z[6];
    Vector3d p[6];
    for (int i = 0; i < 6; i++) {
        D[i + 1] = D[i] * T[i];
        z[i] = D[i].block<3,1>(0,2); // z_i: 회전 축
        p[i] = D[i].block<3,1>(0,3); // p_i: 위치 벡터
    }
    Vector3d p_n = D[6].block<3,1>(0,3); // 엔드 이펙터 위치

    // 제이콥비안 계산
    Matrix<double, 6, 6> Jacobian;
    for (int i = 0; i < 6; i++) {
        Vector3d J_v = z[i].cross(p_n - p[i]);
        Vector3d J_w = z[i];
        Jacobian.block<3,1>(0,i) = J_v;
        Jacobian.block<3,1>(3,i) = J_w;
    }

    return Jacobian;
    }

    Matrix<double, 6, 6> compute_inverse_jacobian(const Matrix<double, 6, 6>& J)
    {
        double lambda = 0.3; // Adjust damping factor as needed
        Matrix<double, 6, 6> JJt = J * J.transpose() + lambda * MatrixXd::Identity(6,6);
        return J.transpose() * JJt.inverse();
    }
    
    rclcpp::Time last_time_ = rclcpp::Clock().now();
    double dt_ = 0.1;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose target_pose_;

    chrono::milliseconds loop_rate_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<ManipulatorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}