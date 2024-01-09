cmake_minimum_required(VERSION 3.0.2)
project(user_interface)

find_package(catkin REQUIRED user_interface_msg)

catkin_package()

catkin_install_python(PROGRAMS user_interface/user_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
