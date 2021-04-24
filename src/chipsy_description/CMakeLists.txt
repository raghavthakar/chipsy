cmake_minimum_required(VERSION 2.8.3)
project(chipsy_description)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For model_push
add_library(ROSConveyorBeltPlugin src/ConveyorBeltPlugin.cc)
target_link_libraries(ROSConveyorBeltPlugin ${CATKIN_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)
