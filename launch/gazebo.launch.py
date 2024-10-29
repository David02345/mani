from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 설정
    share_dir = get_package_share_directory('mani')
    xacro_file = os.path.join(share_dir, 'urdf', '6dof_mani.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    robot_controllers = "/home/ydg/mani_ws/src/mani/config/ros2_controllers.yaml"

    iiwa_simulation_world = PathJoinSubstitution(
        [share_dir, 'worlds', 'empty.world']
    )

    # 로봇 상태 퍼블리셔
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'verbose': 'false', 'world': iiwa_simulation_world}.items(),
    )

    # Gazebo에 URDF 스폰
    urdf_spawn_node = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', '6dof_mani',
        '-topic', 'robot_description',
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.0' 
    ],
    output='screen'
)

    # 컨트롤러 매니저 실행
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # joint_state_broadcaster 스폰
    joint_state_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # joint_group_position_controller 스포너
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mani_arm_controller', '--controller-manager', '/controller_manager'],
    )

    # 컨트롤러 스폰 순서 설정
    delay_joint_state_controller_spawner_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawn_node,
            on_exit=[joint_state_controller_spawner],
        )
    )

    delay_position_controller_spawner_after_joint_state_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_controller_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        urdf_spawn_node,
        # controller_manager,
        delay_joint_state_controller_spawner_after_controller_manager,
        delay_position_controller_spawner_after_joint_state_controller_spawner
    ])
