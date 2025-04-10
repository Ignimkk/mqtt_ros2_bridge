cmake_minimum_required(VERSION 3.8)
project(ros2_introspection)

# Default to C++17
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
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rmw_cyclonedds_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)
find_package(nlohmann_json REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(yaml-cpp REQUIRED)

# 플러그인 라이브러리 직접 포함
add_library(${PROJECT_NAME}
  src/ros2_introspection.cpp
  src/stringtree.cpp
  src/plugin_loader.cpp
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
  tf2_msgs
  tf2
  tf2_ros
  pluginlib
  class_loader
)

target_link_libraries(${PROJECT_NAME}
  Boost::container
)

# 별도의 플러그인 라이브러리 생성
add_library(${PROJECT_NAME}_plugins SHARED
  plugins/tf_message_handler.cpp
)

target_include_directories(${PROJECT_NAME}_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_plugins
  rclcpp
  tf2_msgs
  tf2
  tf2_ros
  pluginlib
  class_loader
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${PROJECT_NAME}
)

# Message formatter implementation
add_library(message_formatter
  src/message_formatter.cpp
)

target_include_directories(message_formatter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(message_formatter
  rclcpp
)

target_link_libraries(message_formatter
  ${PROJECT_NAME}
  nlohmann_json::nlohmann_json
)

# Dynamic introspection node
add_executable(dynamic_introspection_node
  src/dynamic_introspection_node.cpp
)

target_link_libraries(dynamic_introspection_node
  ${PROJECT_NAME}
  message_formatter
)

ament_target_dependencies(dynamic_introspection_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_msgs
  tf2
  tf2_ros
  pluginlib
  class_loader
)

# Test node
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${PROJECT_NAME})
ament_target_dependencies(test_node rclcpp sensor_msgs)

# Installation
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_plugins
  message_formatter
  test_node
  dynamic_introspection_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 플러그인 XML 설치
install(
  FILES plugins/plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 플러그인 내보내기
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins/plugins.xml)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} message_formatter)
ament_export_dependencies(
  rclcpp
  rcpputils
  sensor_msgs
  geometry_msgs
  tf2_msgs
  tf2
  tf2_ros
  pluginlib
  class_loader
)

# Add library for the message parser
add_library(message_parser SHARED
  src/topic_introspection.cpp
)
ament_target_dependencies(message_parser
  rclcpp
)
target_include_directories(message_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add library for REST API server
add_library(rest_api_server SHARED
  src/rest_api_server.cpp
)
ament_target_dependencies(rest_api_server
  rclcpp
  nlohmann_json
)
target_include_directories(rest_api_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add library for message formatter
add_library(message_formatter SHARED
  src/message_formatter.cpp
)
ament_target_dependencies(message_formatter
  rclcpp
  std_msgs
  nlohmann_json
)
target_include_directories(message_formatter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(message_formatter
  nlohmann_json::nlohmann_json
)

# Add plugins library
add_library(plugins SHARED
  plugins/tf_message_handler.cpp
)
target_include_directories(plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Link plugins library with pluginlib
ament_target_dependencies(plugins
  rclcpp
  tf2_msgs
  tf2
  tf2_ros
  pluginlib
)
target_link_libraries(plugins
  message_parser
  message_formatter
  pluginlib::pluginlib
)

# Add node executable
add_executable(dynamic_introspection_node
  src/dynamic_introspection_node.cpp
)
ament_target_dependencies(dynamic_introspection_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  tf2_msgs
  tf2
  tf2_ros
  yaml-cpp
  nlohmann_json
)
target_link_libraries(dynamic_introspection_node
  message_parser
  message_formatter
  plugins
  rest_api_server
  yaml-cpp
)

# Install
install(TARGETS
  message_parser
  message_formatter
  plugins
  rest_api_server
  dynamic_introspection_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 플러그인 XML 설치
install(
  FILES plugins/plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 플러그인 내보내기
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins/plugins.xml)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} message_formatter)
ament_export_dependencies(
  rclcpp
  rcpputils
  sensor_msgs
  geometry_msgs
  tf2_msgs
  tf2
  tf2_ros
  pluginlib
  class_loader
)

# Add library for the message parser
add_library(message_parser SHARED
  src/topic_introspection.cpp
)
ament_target_dependencies(message_parser
  rclcpp
)
target_include_directories(message_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_package()
