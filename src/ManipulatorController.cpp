#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "Eigen/Dense"
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>



using namespace std;
using namespace Eigen;
double pi = M_PI;
double theta[6];
double d[6] = { 700, 0, 0, 480, 0, 110 };
double a[6] = { 0, 500, 0, 0, 0, 0 };
double alpha[6] = { -pi / 2, 0, -pi / 2, pi / 2, -pi / 2, 0 };

void epsilon_6(Matrix<double, 6, 6>& M) {
	double epsilon = 1e-10;
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			if (abs(M(i, j)) < epsilon) {
				M(i, j) = 0.0;
			}
		}
	}
}

void epsilon_4(Matrix<double, 4, 4>& M) {
	double epsilon = 1e-10;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (abs(M(i, j)) < epsilon) {
				M(i, j) = 0.0;
			}
		}
	}
}

class ManipulatorController : public rclcpp::Node
{
public:
    ManipulatorController()
        : Node("manipulator_controller"), loop_rate_(std::chrono::milliseconds(100)) // 10Hz 주기 설정
    {
        // 목표 위치와 자세 설정
        target_pose_.position.x = 0.5; // 초기 목표(임의로 설정)
        target_pose_.position.y = 0.0;
        target_pose_.position.z = 0.5;
        target_roll_ = 0.0;
        target_pitch_ = 0.0;
        target_yaw_ = 0.0;

        // 관절 위치를 NaN으로 초기화하여 언제 설정되는지 감지
        fill(begin(current_joint_positions_), end(current_joint_positions_), numeric_limits<double>::quiet_NaN());
        
        // 현재 위치 및 자세 구독
        current_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/current_pose", 10, std::bind(&ManipulatorController::current_pose_callback, this, std::placeholders::_1));

        // 관절 상태 구독
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ManipulatorController::joint_state_callback, this, std::placeholders::_1));

        // 관절 명령 퍼블리셔
        joint_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_position_controller/commands", 10);

        // 타이머 설정 (주기적으로 제어 루프 실행)
        timer_ = this->create_wall_timer(loop_rate_, std::bind(&ManipulatorController::control_loop, this));
    }

private:
    // 현재 위치 및 자세를 업데이트하는 콜백 함수
    void current_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (std::isnan(msg->position.x) || std::isnan(msg->position.y) || std::isnan(msg->position.z))
        {
            RCLCPP_WARN(this->get_logger(), "Received invalid pose data");
            return;
        }
        current_pose_ = *msg;
    }
    
    geometry_msgs::msg::Pose current_pose_;
    // 현재 관절 상태를 업데이트하는 콜백 함수
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", msg->name[i].c_str(), msg->position[i]);
            if (msg->name[i] == "Revolute 1")
                current_joint_positions_[0] = msg->position[i];
            if (msg->name[i] == "Revolute 2")
                current_joint_positions_[1] = msg->position[i];
            if (msg->name[i] == "Revolute 3")
                current_joint_positions_[2] = msg->position[i];
            if (msg->name[i] == "Revolute 4")
                current_joint_positions_[3] = msg->position[i];
            if (msg->name[i] == "Revolute 5")
                current_joint_positions_[4] = msg->position[i];
            if (msg->name[i] == "Revolute 6")
                current_joint_positions_[5] = msg->position[i];
        }
    }

    // 제어 루프 함수 (주기적으로 실행)
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
            return; // 관절이 초기화되지 않은 경우 제어 루프 건너뜀
        }

        // 현재 관절 각도를 theta에 업데이트
        for (int i = 0; i < 6; ++i)
        {
            theta[i] = current_joint_positions_[i];
        }

        // 목표 위치와 현재 위치의 차이 계산
        double dx = target_pose_.position.x - current_pose_.position.x;
        double dy = target_pose_.position.y - current_pose_.position.y;
        double dz = target_pose_.position.z - current_pose_.position.z;

        // 목표 자세와 현재 자세의 각도 차이 계산 (오일러 각도 차이)
        double roll_error, pitch_error, yaw_error;
        get_euler_error(roll_error, pitch_error, yaw_error);


        // 오차가 충분히 작으면 제어 종료
        if (sqrt(dx * dx + dy * dy + dz * dz) < 0.01 && 
            fabs(roll_error) < 0.01 && fabs(pitch_error) < 0.01 && fabs(yaw_error) < 0.01)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            return;
        }

        // 각도 변화율 계산 (dt 간격에서의 오차)
        double droll = roll_error / dt_;
        double dpitch = pitch_error / dt_;
        double dyaw = yaw_error / dt_;

        // 선속도 및 각속도를 계산
        Matrix<double, 6, 1> du;
        du << dx, dy, dz, droll, dpitch, dyaw;

        // 현재 자코비안 계산        
        Matrix<double, 6, 6> J_ = compute_jacobian(theta, d, a, alpha);

        // 역자코비안 계산
        Matrix<double, 6, 6> J_DPI = compute_inverse_jacobian(J_);
		Matrix<double, 6, 1> dq;
				
        // dq 계산 (각 관절 변화량)
	    dq = J_DPI * du;

        // 관절 변화량이 너무 크지 않도록 제한
        for (int i = 0; i < 6; ++i)
        {
            if (fabs(dq(i, 0)) > 0.1)
                dq(i, 0) = 0.1 * (dq(i, 0) / fabs(dq(i, 0)));
        }

        // 각 관절에 명령 전송 (퍼블리시)
        std_msgs::msg::Float64MultiArray joint_command;
        joint_command.data.resize(6);
        for (int i = 0; i < 6; ++i)
        {
            joint_command.data[i] = current_joint_positions_[i] + dq(i, 0);
        }
        joint_command_publisher_->publish(joint_command);

        // dt 업데이트
        last_time_ = rclcpp::Clock().now();
    }

    // 자코비안 행렬 계산 함수
    Matrix<double, 6, 6> compute_jacobian(double theta[6], double d[6], double a[6], double alpha[6])
    {
        Matrix<double, 4, 4> T01;
        T01 << cos(theta[0]), -cos(alpha[0]) * sin(theta[0]), sin(alpha[0])* sin(theta[0]), a[0]* cos(theta[0]),
	        sin(theta[0]), cos(alpha[0])* cos(theta[0]), -sin(alpha[0]) * cos(theta[0]), a[0]* sin(theta[0]),
	        0, sin(alpha[0]), cos(alpha[0]), d[0],
	        0, 0, 0, 1;
        Matrix<double, 4, 4> T12;
        T12 << cos(theta[1]), -cos(alpha[1]) * sin(theta[1]), sin(alpha[1])* sin(theta[1]), a[1]* cos(theta[1]),
	        sin(theta[1]), cos(alpha[1])* cos(theta[1]), -sin(alpha[1]) * cos(theta[1]), a[1]* sin(theta[1]),
	        0, sin(alpha[1]), cos(alpha[1]), d[1],
	        0, 0, 0, 1;
        Matrix<double, 4, 4> T23;
        T23 << cos(theta[2]), -cos(alpha[2]) * sin(theta[2]), sin(alpha[2])* sin(theta[2]), a[2]* cos(theta[2]),
	        sin(theta[2]), cos(alpha[2])* cos(theta[2]), -sin(alpha[2]) * cos(theta[2]), a[2]* sin(theta[2]),
	        0, sin(alpha[2]), cos(alpha[2]), d[2],
        	0, 0, 0, 1;
        Matrix<double, 4, 4> T34;
        T34 << cos(theta[3]), -cos(alpha[3]) * sin(theta[3]), sin(alpha[3])* sin(theta[3]), a[3]* cos(theta[3]),
	        sin(theta[3]), cos(alpha[3])* cos(theta[3]), -sin(alpha[3]) * cos(theta[3]), a[3]* sin(theta[3]),
	        0, sin(alpha[3]), cos(alpha[3]), d[3],
	        0, 0, 0, 1;
        Matrix<double, 4, 4> T45;
        T45 << cos(theta[4]), -cos(alpha[4]) * sin(theta[4]), sin(alpha[4])* sin(theta[4]), a[4]* cos(theta[4]),
	        sin(theta[4]), cos(alpha[4])* cos(theta[4]), -sin(alpha[4]) * cos(theta[4]), a[4]* sin(theta[4]),
	        0, sin(alpha[4]), cos(alpha[4]), d[4],
	        0, 0, 0, 1;
        Matrix<double, 4, 4> T56;
        T56 << cos(theta[5]), -cos(alpha[5]) * sin(theta[5]), sin(alpha[5])* sin(theta[5]), a[5]* cos(theta[5]),
	        sin(theta[5]), cos(alpha[5])* cos(theta[5]), -sin(alpha[5]) * cos(theta[5]), a[5]* sin(theta[5]),
	        0, sin(alpha[5]), cos(alpha[5]), d[5],
	        0, 0, 0, 1;

        epsilon_4(T01);
        epsilon_4(T12);
        epsilon_4(T23);
        epsilon_4(T34);
        epsilon_4(T45);
        epsilon_4(T56);

        Matrix<double, 4, 4> D2 = T01 * T12;
        Matrix<double, 4, 4> D3 = T01 * T12 * T23;
        Matrix<double, 4, 4> D4 = T01 * T12 * T23 * T34;
        Matrix<double, 4, 4> D5 = T01 * T12 * T23 * T34 * T45;
        Matrix<double, 4, 4> D6 = T01 * T12 * T23 * T34 * T45 * T56;

        epsilon_4(D2);
        epsilon_4(D3);
        epsilon_4(D4);
        epsilon_4(D5);
        epsilon_4(D6);

        Matrix<double, 3, 1> X1;
        X1 << 0 * D6(2, 3) - 1 * D6(1, 3), 0 * D6(2, 3) - 1 * D6(0, 3), 0 * D6(1, 3) - 0 * D6(0, 3);
        Matrix<double, 3, 1> X2;
        X2 << T01(1, 2) * (D6(2, 3) - T01(2, 3)) - T01(2, 2) * (D6(1, 3) - T01(1, 3)),
	        T01(0, 2)* (T01(2, 3) - D6(2, 3)) - T01(2, 2) * (T01(0, 3) - D6(0, 3)),
	        T01(0, 2)* (D6(1, 3) - T01(1, 3)) - T01(1, 2) * (D6(0, 3) - T01(0, 3));
        Matrix<double, 3, 1> X3;
        X3 << D2(1, 2) * (D6(2, 3) - D2(2, 3)) - D2(2, 2) * (D6(1, 3) - D2(1, 3)),
	        D2(0, 2)* (D2(2, 3) - D6(2, 3)) - D2(2, 2) * (D2(0, 3) - D6(0, 3)),
	        D2(0, 2)* (D6(1, 3) - D2(1, 3)) - D2(1, 2) * (D6(0, 3) - D2(0, 3));
        Matrix<double, 3, 1> X4;
        X4 << D3(1, 2) * (D6(2, 3) - D3(2, 3)) - D3(2, 2) * (D6(1, 3) - D3(1, 3)),
	        D3(0, 2)* (D3(2, 3) - D6(2, 3)) - D3(2, 2) * (D3(0, 3) - D6(0, 3)),
	        D3(0, 2)* (D6(1, 3) - D3(1, 3)) - D3(1, 2) * (D6(0, 3) - D3(0, 3));
        Matrix<double, 3, 1> X5;
        X5 << D4(1, 2) * (D6(2, 3) - D4(2, 3)) - D4(2, 2) * (D6(1, 3) - D4(1, 3)),
	        D4(0, 2)* (D4(2, 3) - D6(2, 3)) - D4(2, 2) * (D4(0, 3) - D6(0, 3)),
	        D4(0, 2)* (D6(1, 3) - D4(1, 3)) - D4(1, 2) * (D6(0, 3) - D4(0, 3));
        Matrix<double, 3, 1> X6;
        X6 << D5(1, 2) * (D6(2, 3) - D5(2, 3)) - D5(2, 2) * (D6(1, 3) - D5(1, 3)),
	        D5(0, 2)* (D5(2, 3) - D6(2, 3)) - D5(2, 2) * (D5(0, 3) - D6(0, 3)),
	        D5(0, 2)* (D6(1, 3) - D5(1, 3)) - D5(1, 2) * (D6(0, 3) - D5(0, 3));

        Matrix<double, 6, 6> Jacobian;
        Jacobian << X1(0, 0), X2(0, 0), X3(0, 0), X4(0, 0), X5(0, 0), X6(0, 0),
    	    X1(1, 0), X2(1, 0), X3(1, 0), X4(1, 0), X5(1, 0), X6(1, 0),
	        X1(2, 0), X2(2, 0), X3(2, 0), X4(2, 0), X5(2, 0), X6(2, 0),
        	0, T01(0, 2), D2(0, 2), D3(0, 2), D4(0, 2), D5(0, 2),
	        0, T01(1, 2), D2(1, 2), D3(1, 2), D4(1, 2), D5(1, 2),
	        1, T01(2, 2), D2(2, 2), D3(2, 2), D4(2, 2), D5(2, 2);

        epsilon_6(Jacobian);

        return Jacobian;
    }

    // 자코비안 행렬의 역행렬 계산 함수 (모어-펜로즈 역행렬)
    Matrix<double, 6, 6> compute_inverse_jacobian(const Matrix<double, 6, 6>& J)
    {
        Matrix<double, 6, 6> I = MatrixXd::Identity(6, 6);
        double lambda = 0.3; // 댐핑 팩터
        Matrix<double, 6, 6> dpi_value = J * J.transpose() + lambda * lambda * I;
        return J.transpose() * dpi_value.inverse();
    }

    // 목표와 현재 자세 간의 오일러 각도 차이 계산
    void get_euler_error(double& roll_error, double& pitch_error, double& yaw_error)
    {
        // 현재 오일러 각도 계산
        double current_roll, current_pitch, current_yaw;
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(current_roll, current_pitch, current_yaw);

        // 오일러 각도 차이 계산
        roll_error = target_roll_ - current_roll;
        pitch_error = target_pitch_ - current_pitch;
        yaw_error = target_yaw_ - current_yaw;
    }

    // 현재 시간
    rclcpp::Time last_time_ = rclcpp::Clock().now();
    double dt_ = 0.05; // 제어 주기 (초 단위)

    // 퍼블리셔 및 서브스크라이버
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 목표 위치 및 자세
    geometry_msgs::msg::Pose target_pose_;
    double target_roll_, target_pitch_, target_yaw_;

    // 현재 관절 각도
    double current_joint_positions_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 제어 주기
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
