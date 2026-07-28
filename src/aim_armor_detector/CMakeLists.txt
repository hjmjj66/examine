cmake_minimum_required(VERSION 3.8)
project(aim_armor_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenVINO REQUIRED)

add_library(aim_armor_detector_component SHARED
  src/lightbar_pca_corrector.cpp
  src/openvino_infer.cpp
  src/aim_armor_detector_node.cpp
)

target_compile_features(aim_armor_detector_component PUBLIC cxx_std_17)
target_include_directories(aim_armor_detector_component PRIVATE
  include
)

ament_target_dependencies(aim_armor_detector_component
  aim_msgs
  cv_bridge
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

target_link_libraries(aim_armor_detector_component
  openvino::runtime
  ${OpenCV_LIBS}
)

add_executable(aim_armor_detector_node
  src/aim_armor_detector_main.cpp
)

target_compile_features(aim_armor_detector_node PUBLIC cxx_std_17)
target_include_directories(aim_armor_detector_node PRIVATE
  include
)

target_link_libraries(aim_armor_detector_node
  aim_armor_detector_component
)

rclcpp_components_register_nodes(aim_armor_detector_component
  "aim_armor_detector::AimArmorDetectorNode"
)

install(TARGETS
  aim_armor_detector_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS
  aim_armor_detector_node
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

  ament_add_gtest(test_team_color
    test/test_team_color.cpp
  )
  if(TARGET test_team_color)
    target_include_directories(test_team_color PRIVATE include)
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
