cmake_minimum_required(VERSION 2.8.3)
project(organized_point_normals)

set(PACKAGE_DEPENDENCIES 
  roscpp
  pcl_ros
  pcl_conversions
  std_msgs
  dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
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
add_definitions(
  ${PCL_DEFINITIONS}
)

generate_dynamic_reconfigure_options(cfg/OrganizedPointNormals.cfg)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    DEPENDS PCL
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME} src/organized_point_normals.cpp)

add_dependencies(${PROJECT_NAME} ${PROJEKT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
