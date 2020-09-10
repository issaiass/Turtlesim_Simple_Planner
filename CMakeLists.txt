cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_simple_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  turtlesim
)

catkin_package( )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(turtle 
  STATIC
  include/turtle/turtle.hpp
  src/turtle/turtle.cpp
)

add_library(tf2_frame 
  STATIC
  include/tf2_broadcaster/frame/tf2_frame.hpp 
  src/tf2_broadcaster/frame/tf2_frame.cpp 
)

add_library(pid 
  STATIC
  include/pid/pid.hpp 
  src/pid/pid.cpp 
)

add_library(tf2_static
  STATIC
  include/tf2_broadcaster/static/tf2_static.hpp 
  src/tf2_broadcaster/static/tf2_static.cpp 
)

add_executable(simple_planner src/simple_planner.cpp
)

add_executable(pose_broadcaster src/pose_broadcaster.cpp
)

add_executable(goal_broadcaster src/goal_broadcaster.cpp
)

target_link_libraries(simple_planner 
  turtle 
  tf2_frame
  pid
  ${catkin_LIBRARIES}
)

target_link_libraries(pose_broadcaster
  ${catkin_LIBRARIES}
)

target_link_libraries(goal_broadcaster
  tf2_static
  ${catkin_LIBRARIES}
)