cmake_minimum_required(VERSION 3.0.2)
project(yet_another_knowledge_base)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Fact.msg
)

add_service_files(
  FILES
  YakobUpdateGraph.srv
  YakobUpdateFacts.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES symbolic_fact_generation
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)