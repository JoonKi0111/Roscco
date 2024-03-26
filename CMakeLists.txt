cmake_minimum_required(VERSION 3.8)

project(roscco)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roscco_msgs REQUIRED)

include(oscc/api/OsccConfig.cmake)
include_directories(include)

################################################oscc_api#####################################################

add_library(
  oscc_api 
  oscc/api/src/oscc.c
)

set_target_properties(oscc_api PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  oscc_api PUBLIC
  include
  oscc/api/include
)

#############################################ros_to_oscc########################################################

add_library(
  ros_to_oscc_node SHARED
  src/ros_to_oscc.cpp
)

ament_target_dependencies(
  ros_to_oscc_node 
  rclcpp
  rclcpp_components
  std_msgs 
  roscco_msgs
)

rclcpp_components_register_nodes(
  ros_to_oscc_node 
  "roscco_component::RosToOscc"
)

target_link_libraries(
  ros_to_oscc_node 
  oscc_api
)

#############################################executor########################################################

add_executable(
  roscco_executor
  src/roscco_executor.cpp
)

ament_target_dependencies(
  roscco_executor
  rclcpp
)

target_link_libraries(
  roscco_executor
  ros_to_oscc_node
)

install(TARGETS 
  oscc_api
  ros_to_oscc_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  roscco_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
