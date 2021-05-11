cmake_minimum_required(VERSION 2.8.3)
project(chipsy_plugins)

# Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# For model_push
add_library(ConveyorBeltPlugin src/ConveyorBeltPlugin.cc)
target_link_libraries(ConveyorBeltPlugin ${CATKIN_LIBRARIES} ${GAZEBO_LIBRARIES})

# For contact
add_library(ContactPlugin src/ContactPlugin.cc)
target_link_libraries(ContactPlugin ${CATKIN_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)
