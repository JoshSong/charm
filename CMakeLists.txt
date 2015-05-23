cmake_minimum_required(VERSION 2.8.3)
project(charm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include src/dynamixel/include
  LIBRARIES charm
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} include src/dynamixel/include
)

add_library(dynamixel
  src/dynamixel/src/dxl_hal.c
  src/dynamixel/src/dynamixel.c
)

add_library(charm
  src/vision/ImageProcessor.cpp
  src/arm/ArmProxy.cpp
  src/shared/Point.cpp
  src/shared/PointNode.cpp
  src/shared/Item.cpp
  src/shared/Globals.cpp
  src/planner/AStar.cpp
  src/planner/MainPlanner.cpp
)
target_link_libraries(charm ${catkin_LIBRARIES} ${OpenCV_LIBS} dynamixel)

add_executable(charm_node src/ros/CharmNode.cpp)
target_link_libraries(charm_node ${catkin_LIBRARIES} charm)
