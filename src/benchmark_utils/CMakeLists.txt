cmake_minimum_required(VERSION 3.0.2)
project(benchmark_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  quadrotor_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    message_runtime
    quadrotor_msgs
    mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/benchmark_node.py
  scripts/real_benchmark_node.py
  scripts/repeat_real_benchmark_runner.py
  scripts/trajectory_generator.py
  scripts/benchmark_logger.py
  scripts/task_manager.py
  scripts/benchmark_analyzer.py
  scripts/bag_trajectory_player.py
  scripts/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
