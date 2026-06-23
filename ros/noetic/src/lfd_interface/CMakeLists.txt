cmake_minimum_required(VERSION 3.0.2)
project(lfd_interface)

find_package(catkin REQUIRED user_interface_msg)

catkin_package()

catkin_install_python(PROGRAMS lfd_interface/play_record_kinova.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
