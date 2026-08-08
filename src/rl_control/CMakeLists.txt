cmake_minimum_required(VERSION 3.0.2)
project(rl_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    mavros_msgs
)

catkin_install_python(PROGRAMS
  scripts/control_rl_node.py
  scripts/control_rl_rknn_node.py
  scripts/convert_onnx_to_rknn.py
  scripts/vrpn_to_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
