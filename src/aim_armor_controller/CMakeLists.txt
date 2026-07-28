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

ament_package()
