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

set(BENCHMARK_PROOF_TOOL_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/private/benchmark_proof_tool.cpp
)
if(EXISTS ${BENCHMARK_PROOF_TOOL_SRC})
  find_package(OpenSSL REQUIRED)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  add_executable(benchmark_proof_tool
    ${BENCHMARK_PROOF_TOOL_SRC}
  )
  target_link_libraries(benchmark_proof_tool
    ${catkin_LIBRARIES}
    OpenSSL::Crypto
  )
  install(TARGETS benchmark_proof_tool
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(STATUS "Private benchmark_proof_tool.cpp not found; proof signing tool will not be built.")
endif()

catkin_install_python(PROGRAMS
  scripts/benchmark_node.py
  scripts/real_benchmark_node.py
  scripts/repeat_real_benchmark_runner.py
  scripts/trajectory_generator.py
  scripts/benchmark_logger.py
  scripts/task_manager.py
  scripts/benchmark_analyzer.py
  scripts/generate_submission_package.py
  scripts/bag_trajectory_player.py
  scripts/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
