cmake_minimum_required(VERSION 2.4.6)

project(map_odom_icp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  dynamic_reconfigure
)

find_package(PCL REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_service_files(
  FILES
  IcpSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/MapOdomIcp.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
  INCLUDE_DIRS include
  DEPENDS PCL octomap
)

add_executable(map_odom_icp src/map_odom_icp.cpp src/ros_pcl_icp.cpp)

target_link_libraries(map_odom_icp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(map_odom_icp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_gencfg)

install(TARGETS
  map_odom_icp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

