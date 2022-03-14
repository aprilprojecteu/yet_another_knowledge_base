cmake_minimum_required(VERSION 3.0.2)
project(yet_another_knowledge_base)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES symbolic_fact_generation
#  CATKIN_DEPENDS gazebo_msgs roscpp rospy sensor_msgs
#  DEPENDS system_lib
)