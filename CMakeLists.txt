cmake_minimum_required(VERSION 2.8.3)
project(vlc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs rospy)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   PlayerState.msg
)

## Generate services in the 'srv' folder
add_service_files(
   DIRECTORY srv
   FILES
   Play.srv
   Pause.srv
   Back10.srv
   Forward10.srv
   MuteToggle.srv   
)

catkin_python_setup()
generate_messages(
  DEPENDENCIES std_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

