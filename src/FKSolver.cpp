#include "rclcpp/rclcpp.hpp"
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
double theta[6] = {-pi/2, 0, pi/4, 0, 0, 0 }; // 각 관절의 각도값 입력
double d[6] = {0.7, 0.2, 0.2, 0.48, 0, 0.11};
double a[6] = {0, 0.5, 0, 0, 0, 0};
double alpha[6] = {pi / 2, -pi, -pi / 2, pi / 2, -pi / 2, 0};
Matrix<double, 4, 4> compute_transform_matrix(double theta, double d, double a, double alpha) {
    Matrix<double, 4, 4> T;
    T << cos(theta), -cos(alpha) * sin(theta), sin(alpha) * sin(theta), a * cos(theta),
         sin(theta), cos(alpha) * cos(theta), -sin(alpha) * cos(theta), a * sin(theta),
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
        for (int i = 1; i < 6; i++) {
            q[i] = theta[i];
        }
        q[0] = theta[0]+pi/2;
        

        Matrix<double, 4, 4> T01 = compute_transform_matrix(q[0], d[0], a[0], alpha[0]);
        Matrix<double, 4, 4> T12 = compute_transform_matrix(q[1], d[1], a[1], alpha[1]);
        Matrix<double, 4, 4> T23 = compute_transform_matrix(q[2], d[2], a[2], alpha[2]);
        Matrix<double, 4, 4> T34 = compute_transform_matrix(q[3], d[3], a[3], alpha[3]);
        Matrix<double, 4, 4> T45 = compute_transform_matrix(q[4], d[4], a[4], alpha[4]);
        Matrix<double, 4, 4> T56 = compute_transform_matrix(q[5], d[5], a[5], alpha[5]);

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

        // 오리엔테이션 추출
        Matrix3d rotation_matrix = D6.block<3, 3>(0, 0);
        Quaterniond quaternion(rotation_matrix);

        // 쿼터니언을 RPY로 변환
        tf2::Quaternion q_current(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);

        // 각도 정규화 (필요 시)
        roll = angles::normalize_angle(roll);
        pitch = angles::normalize_angle(pitch);
        yaw = angles::normalize_angle(yaw);

        // 결과 출력
        RCLCPP_INFO(this->get_logger(), "\nx = %f, y = %f, z = %f\nroll = %f, pitch = %f, yaw = %f", x, y, z, roll, pitch, yaw);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<FKSolver>();

    // 각도를 설정한 후, forward kinematics 계산
    node->compute_forward_kinematics();

    rclcpp::shutdown();
    return 0;
}