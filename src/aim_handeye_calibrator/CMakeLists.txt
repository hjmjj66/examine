cmake_minimum_required(VERSION 3.8)
project(aim_handeye_calibrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sentry_gimbal REQUIRED)

add_executable(aim_handeye_calibrator_node
  src/aim_handeye_calibrator_node.cpp
)

add_executable(aim_camera_intrinsic_calibrator_node
  src/aim_camera_intrinsic_calibrator_node.cpp
)

target_compile_features(aim_handeye_calibrator_node PUBLIC cxx_std_17)
target_include_directories(aim_handeye_calibrator_node PRIVATE
  include
)

target_compile_features(aim_camera_intrinsic_calibrator_node PUBLIC cxx_std_17)
target_include_directories(aim_camera_intrinsic_calibrator_node PRIVATE
  include
)

ament_target_dependencies(aim_handeye_calibrator_node
  aim_msgs
  cv_bridge
  rclcpp
  sensor_msgs
  std_srvs
  sentry_gimbal
)

ament_target_dependencies(aim_camera_intrinsic_calibrator_node
  cv_bridge
  rclcpp
  sensor_msgs
  std_srvs
)

target_link_libraries(aim_handeye_calibrator_node
  ${OpenCV_LIBS}
)

target_link_libraries(aim_camera_intrinsic_calibrator_node
  ${OpenCV_LIBS}
)

install(TARGETS
  aim_handeye_calibrator_node
  aim_camera_intrinsic_calibrator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
