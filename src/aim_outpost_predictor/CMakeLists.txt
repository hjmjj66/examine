cmake_minimum_required(VERSION 3.8)
project(aim_outpost_predictor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(aim_outpost_predictor_node
  src/aim_outpost_predictor_main.cpp
  src/aim_outpost_predictor_node.cpp
)

target_compile_features(aim_outpost_predictor_node PUBLIC cxx_std_17)
target_include_directories(aim_outpost_predictor_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(aim_outpost_predictor_node
  aim_msgs
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

install(TARGETS
  aim_outpost_predictor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_front_camera_arbitrator test/test_front_camera_arbitrator.cpp)
  if(TARGET test_front_camera_arbitrator)
    target_include_directories(test_front_camera_arbitrator PRIVATE include)
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
