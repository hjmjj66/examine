cmake_minimum_required(VERSION 3.8)
project(aim_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(COMMON_DEPENDENCIES
  cv_bridge
  aim_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

add_library(aim_usb_camera_component SHARED
  src/usb_camera_component.cpp
)

target_compile_features(aim_usb_camera_component PUBLIC cxx_std_17)
target_include_directories(aim_usb_camera_component PRIVATE
  include
)

ament_target_dependencies(aim_usb_camera_component
  ${COMMON_DEPENDENCIES}
)

target_link_libraries(aim_usb_camera_component
  ${OpenCV_LIBS}
)

rclcpp_components_register_nodes(aim_usb_camera_component
  "aim_camera_driver::UsbCameraComponent"
)

set(GXIAPI_SEARCH_HINTS
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/galaxy"
  "$ENV{GALAXY_SDK_ROOT}"
  "$ENV{GALAXY_SDK_ROOT}/include"
  "$ENV{GALAXY_SDK_ROOT}/inc"
  /usr/include
  /usr/local/include
  /opt/DahengImaging/GalaxySDK/include
  /opt/DahengImaging/GalaxySDK/inc
)

find_path(GXIAPI_INCLUDE_DIR
  NAMES GxAPI.h
  PATHS ${GXIAPI_SEARCH_HINTS}
)

find_library(GXIAPI_LIB
  NAMES gxiapi
  PATHS
    "$ENV{GALAXY_SDK_ROOT}/lib"
    "$ENV{GALAXY_SDK_ROOT}/lib/x86_64"
    /usr/lib
    /usr/local/lib
)

if(GXIAPI_INCLUDE_DIR AND GXIAPI_LIB)
  add_library(aim_gx_camera_component SHARED
    src/gx_camera_component.cpp
  )

  target_compile_features(aim_gx_camera_component PUBLIC cxx_std_17)
  target_include_directories(aim_gx_camera_component PRIVATE
    include
    ${GXIAPI_INCLUDE_DIR}
  )

  ament_target_dependencies(aim_gx_camera_component
    ${COMMON_DEPENDENCIES}
  )

  target_link_libraries(aim_gx_camera_component
    ${GXIAPI_LIB}
    ${OpenCV_LIBS}
  )

  rclcpp_components_register_nodes(aim_gx_camera_component
    "aim_camera_driver::GxCameraComponent"
  )

  add_executable(gx_camera_rate_probe
    tools/gx_camera_rate_probe.cpp
  )

  target_compile_features(gx_camera_rate_probe PUBLIC cxx_std_17)
  target_include_directories(gx_camera_rate_probe PRIVATE
    ${GXIAPI_INCLUDE_DIR}
  )

  ament_target_dependencies(gx_camera_rate_probe
    rclcpp
    sensor_msgs
  )

  target_link_libraries(gx_camera_rate_probe
    ${GXIAPI_LIB}
  )

  install(TARGETS
    aim_gx_camera_component
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  install(TARGETS
    gx_camera_rate_probe
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "GXIAPI SDK headers/library not found; skipping GxCameraComponent. "
    "Install the Galaxy SDK or set GALAXY_SDK_ROOT to enable GX camera support."
  )
endif()

install(TARGETS
  aim_usb_camera_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
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
  ament_add_gtest(test_gx_timestamp_mapper
    test/test_gx_timestamp_mapper.cpp
  )
  if(TARGET test_gx_timestamp_mapper)
    target_include_directories(test_gx_timestamp_mapper PRIVATE include)
    ament_target_dependencies(test_gx_timestamp_mapper rclcpp)
  endif()
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
