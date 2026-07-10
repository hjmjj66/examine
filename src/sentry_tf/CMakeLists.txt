cmake_minimum_required(VERSION 3.8)
project(sentry_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gimbal_driver REQUIRED)
#find_package(gimbal_driver REQUIRED)

add_executable(tf_node src/tf_node.cpp)
target_include_directories(tf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tf_node PUBLIC cxx_std_17)

ament_target_dependencies(tf_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  std_msgs
  gimbal_driver
  #gimbal_driver
)

install(TARGETS tf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
