cmake_minimum_required(VERSION 3.5)
project(ros2_introspection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rmw_cyclonedds_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)

# Library for internal use
add_library(${PROJECT_NAME}
  src/ros2_introspection.cpp
  src/stringtree.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcpputils
  sensor_msgs
  geometry_msgs
  rmw_cyclonedds_cpp
  rosbag2_cpp
)

target_link_libraries(${PROJECT_NAME}
  Boost::container
)

# Executable node
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${PROJECT_NAME})
ament_target_dependencies(test_node rclcpp sensor_msgs)

# Installation
install(TARGETS
  ${PROJECT_NAME}
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
