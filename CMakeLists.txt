cmake_minimum_required(VERSION 2.8.3)
project(swarm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
	nav_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
        FILES
        Position.msg
        Movement.msg
        Area.msg

        SwarmHeader.msg
        StateStatus.msg
        SwarmCommand.msg
        SwarmOdometry.msg
        SwarmStatus.msg

        BoatOdometry.msg
        BoatStatus.msg



)

include_directories( ${catkin_INCLUDE_DIRS} )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
	nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autopilot_ROSpkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)



