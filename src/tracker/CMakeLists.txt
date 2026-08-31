cmake_minimum_required(VERSION 3.8)
project(tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(tracker_factor_primitives
  src/factors.cpp
  src/target_tracker.cpp
)

target_compile_features(tracker_factor_primitives PUBLIC cxx_std_17)
target_include_directories(tracker_factor_primitives PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(tracker_factor_primitives PUBLIC gtsam ${OpenCV_LIBS})
ament_target_dependencies(tracker_factor_primitives
  aim_msgs
  builtin_interfaces
  geometry_msgs
  rclcpp
  std_msgs
  visualization_msgs
)

install(TARGETS tracker_factor_primitives
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

add_executable(tracker_node
  src/tracker_main.cpp
  src/tracker_node.cpp
)

target_compile_features(tracker_node PUBLIC cxx_std_17)
target_include_directories(tracker_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(tracker_node
  tracker_factor_primitives
  gtsam
  ${OpenCV_LIBS}
)
ament_target_dependencies(tracker_node
  aim_msgs
  builtin_interfaces
  geometry_msgs
  rclcpp
  std_msgs
  visualization_msgs
)

install(TARGETS tracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tracker_factors
    test/test_factors.cpp
  )
  if(TARGET tracker_factors)
    target_compile_features(tracker_factors PUBLIC cxx_std_17)
    target_include_directories(tracker_factors PRIVATE
      include
      ${EIGEN3_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(tracker_factors
      tracker_factor_primitives
      gtsam
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(tracker_factors
      aim_msgs
      builtin_interfaces
      geometry_msgs
      std_msgs
    )
  endif()

  ament_add_gtest(tracker_target_tracker
    test/test_target_tracker.cpp
  )
  if(TARGET tracker_target_tracker)
    target_compile_features(tracker_target_tracker PUBLIC cxx_std_17)
    target_include_directories(tracker_target_tracker PRIVATE
      include
      ${EIGEN3_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(tracker_target_tracker
      tracker_factor_primitives
      gtsam
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(tracker_target_tracker
      aim_msgs
      builtin_interfaces
      geometry_msgs
      std_msgs
    )
  endif()

  ament_add_gtest(tracker_window
    test/test_window.cpp
  )
  if(TARGET tracker_window)
    target_compile_features(tracker_window PUBLIC cxx_std_17)
    target_include_directories(tracker_window PRIVATE
      include
      ${EIGEN3_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(tracker_window
      tracker_factor_primitives
      gtsam
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(tracker_window
      aim_msgs
      builtin_interfaces
      geometry_msgs
      std_msgs
    )
  endif()

  ament_add_gtest(tracker_node_test
    test/test_tracker_node.cpp
  )
  if(TARGET tracker_node_test)
    target_compile_features(tracker_node_test PUBLIC cxx_std_17)
    target_include_directories(tracker_node_test PRIVATE
      include
      ${EIGEN3_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(tracker_node_test
      tracker_factor_primitives
      gtsam
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(tracker_node_test
      aim_msgs
      builtin_interfaces
      geometry_msgs
      rclcpp
      std_msgs
      visualization_msgs
    )
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  aim_msgs builtin_interfaces Eigen3 GTSAM geometry_msgs OpenCV rclcpp std_msgs visualization_msgs)
ament_export_include_directories(include)
ament_export_libraries(tracker_factor_primitives)

ament_package()
