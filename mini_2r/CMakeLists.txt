cmake_minimum_required(VERSION 3.8)
project(mini_2r)

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


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(mini_impedance_sim
  src/mini_impedance_sim.cpp
)

ament_target_dependencies(mini_impedance_sim
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

add_executable(mini_impedance_trajectory_sim
  src/mini_impedance_trajectory_sim.cpp
)

ament_target_dependencies(mini_impedance_trajectory_sim
  rclcpp
  sensor_msgs
  pinocchio

  std_msgs
)

add_executable(mini_impedance_trajectory
  src/mini_impedance_trajectory.cpp
)

ament_target_dependencies(mini_impedance_trajectory
  rclcpp
  sensor_msgs
  pinocchio

  std_msgs
)

install(DIRECTORY
  description
  launch
  config
  include/
  rviz
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
  mini_impedance_sim
  mini_impedance_trajectory_sim
  mini_impedance_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generator_trajectory.py
  scripts/generator_trajectory_sim.py
  scripts/interactive_marker_mini_sim.py
  scripts/interactive_marker_mini.py
  scripts/mini_state_fusion.py
  scripts/mini_2r_position_sim.py
  scripts/mini_2r_position.py
  scripts/RR_joint_states.py
  scripts/trajectory_plot.py
  scripts/mini_speed_and_impedance.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
