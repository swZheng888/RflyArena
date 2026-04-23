cmake_minimum_required(VERSION 3.0.2)
project(nmpc_control)

## 查找 catkin 和依赖包
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  tf2_ros
  tf2_geometry_msgs
)

## Python 设置
catkin_python_setup()

###################################
## catkin 特定配置 ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    mavros_msgs 
    tf2_ros 
    tf2_geometry_msgs
)

###################################
## 安装 ##
###################################

## 安装 Python 可执行脚本
catkin_install_python(PROGRAMS
  scripts/control_nmpc_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安装 rviz 配置文件
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

## 安装配置文件（如果有）
# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )