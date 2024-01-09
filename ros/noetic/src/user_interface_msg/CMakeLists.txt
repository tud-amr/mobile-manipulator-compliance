cmake_minimum_required(VERSION 3.0.2)
project(user_interface_msg)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
)

add_message_files(
  FILES
  Ufdbk.msg
  Ucmd.msg
  Ustate.msg
  Utarget.msg
  Record.msg
  Data.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
 CATKIN_DEPENDS message_runtime
)