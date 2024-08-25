cmake_minimum_required(VERSION 3.5)
project(huace_gnss_driver)

## Compile as C++17
add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
message(STATUS "Setting build type to Release as none was specified.")
set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)
# Set the possible values of build type for cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
  "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(WARNFLAGS "-Wall -Wextra -Wpedantic -Wcast-align -Wnon-virtual-dtor -Woverloaded-virtual -Wcast-qual -Wctor-dtor-privacy")

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED)
#LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
#find_package(GeographicLib REQUIRED)

## For PCAP file handling
find_library(libpcap_LIBRARIES pcap)
if ("${libpcap_LIBRARIES}" STREQUAL "pcap-NOTFOUND")
    set(libpcap_FOUND FALSE)
else ()
    set(libpcap_FOUND TRUE)
endif ()

find_package(ament_cmake QUIET)

  add_compile_definitions(ROS2)
  set(ENV{ROS_VERSION} "2")

  #configure_file(${CMAKE_SOURCE_DIR}/msg_cache/BlockHeaderRos2.msg ${CMAKE_SOURCE_DIR}/msg/BlockHeader.msg COPYONLY)

  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_SYMLINK_INSTALL)

  # Dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(nmea_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(gps_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  find_package(boost_io_context REQUIRED)
  find_package(boost_tcp_driver REQUIRED)
  find_package(boost_udp_driver REQUIRED)
  
  ## Declare  messages
  rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gpchc.msg"
  "msg/Gpchcx.msg"
  DEPENDENCIES std_msgs
  )

  set(library_name huace_gnss_driver_core)
  set(executable_name huace_gnss_driver_node)

  set(dependencies
  rclcpp
  rclcpp_components
  diagnostic_msgs
  nmea_msgs
  sensor_msgs
  geometry_msgs
  gps_msgs
  nav_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  boost_io_context
  boost_tcp_driver
  boost_udp_driver
  )

  ## shared library
  add_library(${library_name} SHARED
  src/huace_gnss_driver/communication_core.cpp
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
  target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"  
  "$<INSTALL_INTERFACE:include>"
  )
  target_link_libraries(${library_name}
  boost_regex # ordering is important for regex https://stackoverflow.com/questions/17588440/cannot-link-boost-regex
  ${Boost_LIBRARIES}
  pcap
  ${GeographicLib_LIBRARIES}
  )
  ament_target_dependencies(${library_name}
  ${dependencies}
  )

  if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(${library_name}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  else()
    rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${library_name} "${cpp_typesupport_target}")
    add_compile_definitions(ROS2_VER_N250)
  endif()

  ## executable
  add_executable(${executable_name}
  src/huace_gnss_driver/main.cpp
  )
  target_link_libraries(${executable_name}
  ${library_name}
  )
  ament_target_dependencies(${executable_name}
  ${dependencies}
  )

  rclcpp_components_register_nodes(${library_name} "huace_node::HuaceNode")

  ## Testing
  #find_package(ament_cmake_gtest REQUIRED)
  #add_subdirectory(test)

  # Export Targets
  ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
  ament_export_dependencies(${dependencies} rosidl_default_runtime)

  # Installation
  install(TARGETS
  ${library_name} EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

  install(TARGETS ${executable_name} EXPORT ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
  DESTINATION include/
  )

  install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
