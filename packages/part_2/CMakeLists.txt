cmake_minimum_required(VERSION 3.0.2)  # Use a newer version for better compatibility
project(part_2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation  # Required for defining custom services/messages
  led_controller
)

## Declare the package dependencies
catkin_package(
  CATKIN_DEPENDS rospy std_msgs led_controller 
)

## Install scripts (Python nodes)
catkin_install_python(PROGRAMS
  src/composite_node.py  # Ensure this script exists in `src/`
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)