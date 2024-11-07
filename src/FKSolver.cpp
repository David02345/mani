#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "Eigen/Dense"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <cmath>
#include <angles/angles.h>

using namespace std;
using namespace Eigen;

double pi = M_PI;
double q[6] = {0, };
double theta[6] = {pi, -pi/7, -pi, 0, 0, 0 }; // 각 관절의 각도값 입력
double d[6] = {0.7, 0.2, 0.2, 0.48, 0, 0.11};
double a[6] = {0, 0.5, 0, 0, 0, 0};
double alpha[6] = {pi / 2, -pi, -pi / 2, pi / 2, -pi / 2, 0};

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

class FKSolver : public rclcpp::Node
{
public:
    FKSolver() : Node("fksolver") {}

    void compute_forward_kinematics() {
        for (int i = 0; i < 6; i++) {
            q[i] = theta[i];
        }
        current_pose_.position.x = 0.0;
        current_pose_.position.y = 0.0;
        current_pose_.position.z = 0.0;
        current_pose_.orientation.x = 0.0;
        current_pose_.orientation.y = 0.0;
        current_pose_.orientation.z = 0.0;
        current_pose_.orientation.w = 1.0;

        Matrix<double, 4, 4> T01 = compute_transform_matrix(q[0], d[0], a[0], alpha[0]);
        Matrix<double, 4, 4> T12 = compute_transform_matrix(q[1], d[1], a[1], alpha[1]);
        Matrix<double, 4, 4> T23 = compute_transform_matrix(q[2], d[2], a[2], alpha[2]);
        Matrix<double, 4, 4> T34 = compute_transform_matrix(q[3], d[3], a[3], alpha[3]);
        Matrix<double, 4, 4> T45 = compute_transform_matrix(q[4], d[4], a[4], alpha[4]);
        Matrix<double, 4, 4> T56 = compute_transform_matrix(q[5], d[5], a[5], alpha[5]);
        /*
        epsilon(T01);
        epsilon(T12);
        epsilon(T23);
        epsilon(T34);
        epsilon(T45);
        epsilon(T56);
        */
        Matrix<double, 4, 4> D6 = T01 * T12 * T23 * T34 * T45 * T56;
        //epsilon(D6);

        double x = D6(0, 3);
        double y = D6(1, 3);
        double z = D6(2, 3);

        Matrix3d rotation_matrix = D6.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation_matrix);

        current_pose_.orientation.x = quaternion.x();
        current_pose_.orientation.y = quaternion.y();
        current_pose_.orientation.z = quaternion.z();
        current_pose_.orientation.w = quaternion.w();

        double roll, pitch, yaw;
        tf2::Quaternion tf_quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "\nx, y, z, r, p, y => %f %f %f %f %f %f", x, y, z, roll, pitch, yaw);
    }

private:
    geometry_msgs::msg::Pose current_pose_;
    tf2::Quaternion current_orientation_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<FKSolver>();

    node->compute_forward_kinematics();

    rclcpp::shutdown();
    return 0;
}
