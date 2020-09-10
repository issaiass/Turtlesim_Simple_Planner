# Minimal Cmake required version
cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)

# Project name
project(turtlesim_simple_planner)

# Neccesary ROS packages for the application
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  turtlesim
)

# Use this catkin packages
catkin_package( )

# The directory for the header files (location)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ADDING LIBRARIES
# turtlesim library
add_library(turtle 
  STATIC
  include/turtle/turtle.hpp
  src/turtle/turtle.cpp
)

# tf2 Library
# for the static broadcaster
add_library(tf2_static
  STATIC
  include/tf2_broadcaster/static/tf2_static.hpp 
  src/tf2_broadcaster/static/tf2_static.cpp 
)

# for the frame broadcaster
add_library(tf2_frame 
  STATIC
  include/tf2_broadcaster/frame/tf2_frame.hpp 
  src/tf2_broadcaster/frame/tf2_frame.cpp 
)

# pid library
add_library(pid 
  STATIC
  include/pid/pid.hpp 
  src/pid/pid.cpp 
)

# Node for the planner algorithm
add_executable(simple_planner src/simple_planner.cpp
)

# Node that exposes the pose to /tf topic
add_executable(pose_broadcaster src/pose_broadcaster.cpp
)

# Node that exposes the pose to the /tf_static topic
add_executable(goal_broadcaster src/goal_broadcaster.cpp
)

# The planner algorithm uses this libraries (dynamically linked)
target_link_libraries(simple_planner 
  turtle 
  tf2_frame
  pid
  ${catkin_LIBRARIES}
)

# The pose broadcaster uses this libraries (dynamically linked)
target_link_libraries(pose_broadcaster
  ${catkin_LIBRARIES}
)

# The goal broadcaster uses this libraries (dynamically linked)
target_link_libraries(goal_broadcaster
  tf2_static
  ${catkin_LIBRARIES}
)