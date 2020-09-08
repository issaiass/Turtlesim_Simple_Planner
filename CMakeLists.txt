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

add_executable(simple_planner src/simple_planner.cpp
                              src/turtle/turtle.cpp
                              src/tf2_broadcaster/frame/tf2_frame.cpp 
                              src/pid/pid.cpp                             
)

add_executable(pose_broadcaster src/pose_broadcaster.cpp
)

add_executable(goal_broadcaster src/goal_broadcaster.cpp
                                src/tf2_broadcaster/static/tf2_static.cpp
)

target_link_libraries(simple_planner
  ${catkin_LIBRARIES}
)

target_link_libraries(pose_broadcaster
  ${catkin_LIBRARIES}
)

target_link_libraries(goal_broadcaster
  ${catkin_LIBRARIES}
)