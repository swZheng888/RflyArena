cmake_minimum_required(VERSION 3.0.2)
project(trajectory_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(FILES TrajectoryPoint.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

# 更新脚本名称
catkin_install_python(PROGRAMS
  scripts/traj_pub_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)