cmake_minimum_required(VERSION 3.8)
project(kuka_mini)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(xacro REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY
  urdf
  config
  include/
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)



add_executable(mini_effort_traj
  src/cpp/mini_effort_traj.cpp
)

add_executable(desired_trajectory
  src/cpp/desired_trajectory.cpp
)

add_executable(mini_effort_tm_real
  src/cpp/mini_effort_tm_real.cpp
)

add_executable(mini_effort_tm_real_final
  src/cpp/mini_effort_tm_real_final.cpp
)

add_executable(mini_effort_tm_sim
  src/cpp/mini_effort_tm_sim.cpp
)

add_executable(mini_kuka_transform_trial
  src/cpp/mini_kuka_transform_trial.cpp
)

add_executable(mini_kuka_transform_for_traj
  src/cpp/mini_kuka_transform_for_traj.cpp
)

add_executable(mini_impedance_trajectory_world
  src/cpp/mini_impedance_trajectory_world.cpp
)

add_executable(iiwa_mini_active_real
  src/cpp/iiwa_mini_active_real.cpp
)




ament_target_dependencies(mini_effort_traj
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

ament_target_dependencies(desired_trajectory
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

ament_target_dependencies(mini_effort_tm_real
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

ament_target_dependencies(mini_effort_tm_real_final
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

ament_target_dependencies(mini_effort_tm_sim
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

ament_target_dependencies(mini_kuka_transform_trial
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

ament_target_dependencies(mini_kuka_transform_for_traj
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)
ament_target_dependencies(mini_impedance_trajectory_world
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)
ament_target_dependencies(iiwa_mini_active_real
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

install(TARGETS
  mini_effort_tm_real
  mini_effort_tm_real_final
  mini_effort_tm_sim
  mini_effort_traj
  mini_kuka_transform_trial
  mini_kuka_transform_for_traj
  desired_trajectory
  mini_impedance_trajectory_world
  iiwa_mini_active_real
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/iiwa_FK_tool_real.py
  scripts/iiwa_FK_tool_sim.py
  scripts/iiwa_vel_telop_real.py
  scripts/iiwa_vel_telop_real_pedal_final.py
  scripts/iiwa_vel_real_final.py
  scripts/iiwa_vel_telop_sim.py
  scripts/interactive_marker_mini.py
  scripts/interactive_marker_mini_real.py
  scripts/interactive_marker_mini_decoupled.py
  scripts/mini_ee_visual.py
  scripts/iiwa_mini2r_redundancy_sim.py
  scripts/iiwa_joint_speed_sim.py
  scripts/mini2r_joint_impedance_sim.py
  scripts/iiwa_mini_vel_telop.py
  scripts/kuka_mini_marker.py
  scripts/kuka_mini_joints.py
 

  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
