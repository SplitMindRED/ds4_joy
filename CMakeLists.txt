cmake_minimum_required(VERSION 3.0.2)
project(ds4_joy)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
)
message("YOYOYOYO")
message(${CMAKE_PREFIX_PATH})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ds4_joy
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
message("include DIRRRS " ${catkin_INCLUDE_DIRS})


add_executable(ds4_control src/ds4_control.cpp)

add_dependencies(ds4_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ds4_control ${catkin_LIBRARIES})

