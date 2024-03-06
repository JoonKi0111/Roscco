cmake_minimum_required(VERSION 3.5)

project(roscco)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(oscc/api/OsccConfig.cmake)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_can_msgs)
find_package(roscco_msgs)
find_package(autoware_auto_vehicle_msgs)


################################################oscc_api#####################################################


add_library(
  oscc_api
  oscc/api/src/oscc.c
)

target_include_directories(
  oscc_api PUBLIC
  include
  oscc/api/include
)


################################################oscc_api#####################################################




################################################ros_to_oscc#####################################################


add_library(
  ros_to_oscc
  src/ros_to_oscc.cpp
)

ament_target_dependencies(ros_to_oscc rclcpp std_msgs roscco_msgs)

target_include_directories(
  ros_to_oscc PUBLIC
  include
  oscc/api/include
)
target_link_libraries(
  ros_to_oscc
  oscc_api
)


################################################ros_to_oscc#####################################################




#############################################oscc_to_ros########################################################


add_library(
  oscc_to_ros
  src/oscc_to_ros.cpp
)

target_include_directories(
  oscc_to_ros PUBLIC
  include
  oscc/api/include
)

target_link_libraries(
  oscc_to_ros
  oscc_api
)

ament_target_dependencies(oscc_to_ros rclcpp std_msgs roscco_msgs)


#############################################oscc_to_ros########################################################



#############################################roscco_node########################################################


add_executable(oscc_to_ros_node src/oscc_to_ros_node.cpp)


target_include_directories(
  oscc_to_ros_node PUBLIC
  include
  oscc/api/include
)

ament_target_dependencies(oscc_to_ros_node rclcpp std_msgs roscco_msgs)

target_link_libraries(
  oscc_to_ros_node
  oscc_api
  oscc_to_ros
  ${rclcpp_LIBRARIES}
)


#############################################roscco_node########################################################



#############################################ros_to_oscc_node########################################################


add_executable(ros_to_oscc_node src/ros_to_oscc_node.cpp)

target_include_directories(
  ros_to_oscc_node PUBLIC
  include
  oscc/api/include
)

ament_target_dependencies(ros_to_oscc_node rclcpp std_msgs roscco_msgs)

target_link_libraries(
  ros_to_oscc_node
  oscc_api
  ros_to_oscc
  ${rclcpp_LIBRARIES}
)


#############################################ros_to_oscc_node########################################################





#############################################roscco2aw########################################################


add_executable(
  roscco2aw
  src/roscco2aw.cpp
)

ament_target_dependencies(
  roscco2aw
  rclcpp
  roscco_msgs
  autoware_auto_vehicle_msgs
)


#############################################roscco2aw########################################################


install(TARGETS 
  ros_to_oscc_node
  ros_to_oscc
  oscc_to_ros_node
  oscc_to_ros
  roscco2aw
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
