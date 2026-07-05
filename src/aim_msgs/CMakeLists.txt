cmake_minimum_required(VERSION 3.8)
project(aim_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/ArmorArray.msg"
  "msg/ArmorClass.msg"
  "msg/ArmorPoseSet.msg"
  "msg/ArmorPoseSetArray.msg"
  "msg/ArmorSet.msg"
  "msg/ArmorSetArray.msg"
  "msg/ArmorSetWithState.msg"
  "msg/ArmorSetArrayWithState.msg"
  "msg/ControlAngles.msg"
  "msg/GimbalState.msg"
  "msg/OutpostState.msg"
  "msg/OutpostPredictorState.msg"
  "msg/SelectedTargetId.msg"
  "msg/TargetState.msg"
  "msg/DelayStatistics.msg"
  "msg/TargetStateArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
