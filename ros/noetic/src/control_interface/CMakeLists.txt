cmake_minimum_required(VERSION 3.0.2)
project(control_interface)

find_package(catkin REQUIRED user_interface_msg)


catkin_package()

catkin_install_python(PROGRAMS 
  control_interface/control_interface_node.py
  control_interface/fk_node.py
  control_interface/make_compliant_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
