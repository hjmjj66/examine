cmake_minimum_required(VERSION 3.8)
project(aim_armor_decider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aim_target_msgs REQUIRED)

add_executable(aim_armor_decider_node
  src/armor_decider_node.cpp
)

target_compile_features(aim_armor_decider_node PUBLIC cxx_std_17)

ament_target_dependencies(aim_armor_decider_node
  aim_msgs
  geometry_msgs
  rclcpp
  aim_target_msgs
)

install(TARGETS aim_armor_decider_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
