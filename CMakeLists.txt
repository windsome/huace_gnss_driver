cmake_minimum_required(VERSION 3.8)
project(huace_gnss_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
  
## Declare  messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gpchc.msg"
  "msg/Gpchcx.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

set(LIBRARY_NAME huace_gnss_node)

ament_auto_add_library(${LIBRARY_NAME} SHARED
  src/huace_gnss_driver/message_handler.cpp 
  src/huace_gnss_driver/huace_node.cpp
  src/huace_gnss_driver/parsers/nmea_parsers/gpgga.cpp 
  src/huace_gnss_driver/parsers/nmea_parsers/gprmc.cpp 
  src/huace_gnss_driver/parsers/nmea_parsers/gpgsa.cpp 
  src/huace_gnss_driver/parsers/nmea_parsers/gpgsv.cpp
  src/huace_gnss_driver/parsers/nmea_parsers/gpchc.cpp
  src/huace_gnss_driver/parsers/nmea_parsers/gpchcx.cpp
  src/huace_gnss_driver/parsers/parsing_utilities.cpp 
  src/huace_gnss_driver/parsers/string_utilities.cpp 
)

# 设置包含目录
target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(${LIBRARY_NAME}
  PLUGIN "huace_node::HuaceNode"
  EXECUTABLE ${LIBRARY_NAME}_exe
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(${LIBRARY_NAME}
  "${cpp_typesupport_target}"
)

ament_auto_add_executable(io_test
  src/test/test_udp_server.cpp
)

ament_auto_add_executable(${LIBRARY_NAME}_main
  src/huace_gnss_driver/main.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  msg
  launch
  config
  rviz_cfg
)

