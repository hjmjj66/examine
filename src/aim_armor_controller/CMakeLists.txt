cmake_minimum_required(VERSION 3.8)
project(aim_armor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gimbal_driver REQUIRED)
#find_package(gimbal_driver REQUIRED)
find_package(sentry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(aim_armor_tinympc STATIC
  third_party/tinympc/admm.cpp
  third_party/tinympc/tiny_api.cpp
  third_party/tinympc/codegen.cpp
  third_party/tinympc/rho_benchmark.cpp
)
target_compile_features(aim_armor_tinympc PUBLIC cxx_std_17)
target_include_directories(aim_armor_tinympc PUBLIC third_party)
target_link_libraries(aim_armor_tinympc PUBLIC Eigen3::Eigen)
set_property(TARGET aim_armor_tinympc PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(armor_controller_node
  src/armor_controller_node.cpp
  src/mpc_aim_geometry.cpp
  src/mpc_planner.cpp
  src/mpc_target_model.cpp
  src/armor_selection_policy.cpp
  src/mpc_trajectory_solver.cpp
)

target_include_directories(armor_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:include>)
target_compile_features(armor_controller_node PUBLIC cxx_std_17)
target_link_libraries(armor_controller_node
  aim_armor_tinympc
  Eigen3::Eigen
)

ament_target_dependencies(armor_controller_node
  aim_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
  gimbal_driver
#  gimbal_driver
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

  ament_add_gtest(test_outpost_selection
    test/test_outpost_selection.cpp)

  ament_add_gtest(test_armor_selection_policy
    test/test_armor_selection_policy.cpp)
  target_include_directories(test_armor_selection_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_sources(test_armor_selection_policy PRIVATE src/armor_selection_policy.cpp)
  ament_add_gtest(test_controller_trajectory_message
    test/test_controller_trajectory_message.cpp)
  target_include_directories(test_controller_trajectory_message PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_controller_trajectory_message
    gimbal_driver)

  target_include_directories(test_outpost_selection PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../aim_outpost_predictor/include>)
  ament_target_dependencies(test_outpost_selection
    aim_msgs
    geometry_msgs
    rclcpp)
  target_sources(test_outpost_selection PRIVATE src/mpc_target_model.cpp src/armor_selection_policy.cpp)

  ament_add_gtest(test_fire_code_state
    test/test_fire_code_state.cpp)
  target_include_directories(test_fire_code_state PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  ament_add_gtest(test_aim_result_kinematics
    test/test_aim_result_kinematics.cpp)
  target_include_directories(test_aim_result_kinematics PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_aim_result_kinematics
    sentry_msgs)

  add_executable(test_mpc_math test/test_mpc_math.cpp)
  target_include_directories(test_mpc_math PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  add_test(NAME test_mpc_math COMMAND test_mpc_math)

  add_executable(test_mpc_trajectory_solver
    test/test_mpc_trajectory_solver.cpp
    src/mpc_trajectory_solver.cpp)
  target_include_directories(test_mpc_trajectory_solver PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  add_test(NAME test_mpc_trajectory_solver COMMAND test_mpc_trajectory_solver)

  add_executable(test_mpc_aim_geometry
    test/test_mpc_aim_geometry.cpp
    src/mpc_aim_geometry.cpp
    src/mpc_target_model.cpp
    src/armor_selection_policy.cpp
    src/mpc_trajectory_solver.cpp)
  target_include_directories(test_mpc_aim_geometry PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_mpc_aim_geometry
    aim_msgs
    geometry_msgs
    rclcpp)
  target_link_libraries(test_mpc_aim_geometry Eigen3::Eigen)
  add_test(NAME test_mpc_aim_geometry COMMAND test_mpc_aim_geometry)

  add_executable(test_mpc_planner test/test_mpc_planner.cpp)
  target_include_directories(test_mpc_planner PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>)
  target_link_libraries(test_mpc_planner
    aim_armor_tinympc
    Eigen3::Eigen)
  target_sources(test_mpc_planner PRIVATE src/mpc_planner.cpp)
  add_test(NAME test_mpc_planner COMMAND test_mpc_planner)
endif()

ament_package()
