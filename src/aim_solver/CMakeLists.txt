cmake_minimum_required(VERSION 3.8)
project(aim_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES  Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(aim_solver_node
  src/aim_solver_node.cpp
  src/time_alignment_estimator.cpp
)

target_compile_features(aim_solver_node PUBLIC cxx_std_17)
target_include_directories(aim_solver_node PRIVATE
  include
  /usr/include/opencv4
)

ament_target_dependencies(aim_solver_node
  aim_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

target_link_libraries(aim_solver_node
  ${OpenCV_LIBS}
)

install(TARGETS
  aim_solver_node
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
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_coordinate_utils
    test/test_coordinate_utils.cpp
  )
  if(TARGET test_coordinate_utils)
    target_include_directories(test_coordinate_utils PRIVATE
      include
      /usr/include/opencv4
    )
    target_link_libraries(test_coordinate_utils
      ${OpenCV_LIBS}
    )
  endif()

  ament_add_gtest(test_time_alignment_estimator
    test/test_time_alignment_estimator.cpp
  )
  if(TARGET test_time_alignment_estimator)
    target_include_directories(test_time_alignment_estimator PRIVATE include)
    target_sources(test_time_alignment_estimator PRIVATE
      src/time_alignment_estimator.cpp
    )
  endif()

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
