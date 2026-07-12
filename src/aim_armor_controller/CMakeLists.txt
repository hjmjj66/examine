cmake_minimum_required(VERSION 3.8)
project(aim_armor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gimbal_driver REQUIRED)
#find_package(gimbal_driver REQUIRED)
find_package(aim_target_msgs REQUIRED)
find_package(sentry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(armor_controller_node
  src/armor_controller_node.cpp
)

target_include_directories(armor_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(armor_controller_node PUBLIC cxx_std_17)

ament_target_dependencies(armor_controller_node
  aim_msgs
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  gimbal_driver
#  gimbal_driver
  aim_target_msgs
  sentry_msgs
  std_msgs
)

install(TARGETS armor_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_time_alignment
    test/test_control_time_alignment.cpp)
  target_include_directories(test_control_time_alignment PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_control_time_alignment
    builtin_interfaces
    rclcpp)
endif()

ament_package()
