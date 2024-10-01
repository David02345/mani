cmake_minimum_required(VERSION 3.5)
project(mani)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(orocos_kdl)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(xacro REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)


# Build executables

# 1. TargetPoseInputNode
add_executable(target_pose_input src/TargetPoseInput.cpp)
ament_target_dependencies(target_pose_input rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# 2. IKSolverNode
add_executable(ik_solver src/IKSolver.cpp)
ament_target_dependencies(ik_solver rclcpp geometry_msgs sensor_msgs Eigen3)
# KDL 사용 여부에 따라 라이브러리 링크 설정
if(orocos_kdl_FOUND)
  target_link_libraries(ik_solver ${orocos_kdl_LIBRARIES})
endif()

# 3. MotionPlanningNode
add_executable(motion_planning_node src/MotionPlanning.cpp)
ament_target_dependencies(motion_planning_node rclcpp sensor_msgs trajectory_msgs Eigen3)

# 4. ControllerNode
add_executable(controller src/Controller.cpp)
ament_target_dependencies(controller rclcpp trajectory_msgs)

# 5. FeedbackNode
add_executable(feedback src/Feedback.cpp)
ament_target_dependencies(feedback rclcpp std_msgs sensor_msgs)

# 6. ManipulatorControllerNode
add_executable(manipulator_controller src/ManipulatorController.cpp)
ament_target_dependencies(manipulator_controller
  rclcpp 
  rclcpp_components 
  std_msgs geometry_msgs 
  sensor_msgs 
  trajectory_msgs 
  Eigen3 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  )
target_link_libraries(manipulator_controller
  rclcpp::rclcpp
)



# Install executables
install(TARGETS
  target_pose_input
  ik_solver
  motion_planning_node
  controller
  feedback
  manipulator_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY meshes/
        DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)        

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/urdf/6dof_mani.urdf
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/6dof_mani.xacro -o ${CMAKE_CURRENT_SOURCE_DIR}/urdf/6dof_mani.urdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/6dof_mani.xacro
)
add_custom_target(generate_urdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/6dof_mani.urdf)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()