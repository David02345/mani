from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 이름과 경로 설정
    pkg_name = 'mani'
    pkg_share = get_package_share_directory(pkg_name)

    # Gazebo 환경 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # 로봇 모델 로드
    robot_description_path = os.path.join(pkg_share, 'urdf', '6dof_mani.urdf')
    with open(robot_description_path, 'r') as file:
        robot_description = file.read()

    # 로봇 상태 퍼블리셔 실행
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Gazebo에 로봇 스폰
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description'],
        output='screen'
    )

    # 컨트롤러 매니저 실행
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_share, 'config', 'ros_controllers.yaml')
        ],
        output='screen'
    )

    # 컨트롤러 로드 및 시작 (딜레이 적용)
    load_joint_state_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
    output='screen' 
    )

    load_trajectory_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'position_trajectory_controller'],
    output='screen'
    )

    # 사용자 노드 실행
    target_pose_input_node = Node(
        package=pkg_name,
        executable='target_pose_input',
        output='screen'
    )

    ik_solver_node = Node(
        package=pkg_name,
        executable='ik_solver',
        output='screen'
    )

    motion_planning_node = Node(
        package=pkg_name,
        executable='motion_planning_node',
        output='screen'
    )

    controller_node = Node(
        package=pkg_name,
        executable='controller',
        output='screen'
    )

    feedback_node = Node(
        package=pkg_name,
        executable='feedback',
        output='screen'
    )

    # 컨트롤러 로드 딜레이 설정
    delay_load_joint_state_controller = TimerAction(
        period=5.0,
        actions=[load_joint_state_controller]
    )

    delay_load_trajectory_controller = TimerAction(
        period=5.5,
        actions=[load_trajectory_controller]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        load_joint_state_controller,
        load_trajectory_controller,
        target_pose_input_node,
        ik_solver_node,
        motion_planning_node,
        controller_node,
        feedback_node,
    ])
