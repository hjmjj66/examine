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
)

target_compile_features(aim_solver_node PUBLIC cxx_std_17)
target_include_directories(aim_solver_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
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
      ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(test_coordinate_utils
      ${OpenCV_LIBS}
    )
  endif()

  ament_add_gtest(test_fixed_timestamp_offsets
    test/test_fixed_timestamp_offsets.cpp
  )
  if(TARGET test_fixed_timestamp_offsets)
    target_include_directories(test_fixed_timestamp_offsets PRIVATE include)
  endif()

  ament_add_gtest(test_pose_observation_forwarding
    test/test_pose_observation_forwarding.cpp
  )
  if(TARGET test_pose_observation_forwarding)
    target_include_directories(test_pose_observation_forwarding PRIVATE include)
    ament_target_dependencies(test_pose_observation_forwarding
      aim_msgs
      geometry_msgs
    )
  endif()

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
