cmake_minimum_required(VERSION 3.0.2)
project(veides_agent_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_message_files(
   DIRECTORY
   msg
   FILES
   Action.msg
   ActionEntity.msg
   Fact.msg
   Method.msg
   Trail.msg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  ActionCompleted.srv
  Event.srv
  Facts.srv
  MethodCompleted.srv
  Trails.srv
  RegisterMethod.srv
)

generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  nodes/veides_agent_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
