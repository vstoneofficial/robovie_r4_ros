cmake_minimum_required(VERSION 3.0.2)
project(r4_motion)

find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r4_motion
#  CATKIN_DEPENDS moveit_msgs moveit_ros_planning_interface moveit_visual_tools roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(r4_move_group_joint_value
  src/move_group_joint_value.cpp
)
add_dependencies(r4_move_group_joint_value ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(r4_move_group_joint_value
  ${catkin_LIBRARIES}
)