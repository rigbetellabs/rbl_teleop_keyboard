cmake_minimum_required(VERSION 2.8.3)
project(rbl_teleop_keyboard)

find_package(catkin REQUIRED)

catkin_package()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   rbl_teleop_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

