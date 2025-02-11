cmake_minimum_required(VERSION 3.0.2)
project(led_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# Declare the custom service file
add_service_files(
  FILES
  SetLEDColor.srv
)

# Generate the messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare the package dependencies
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

# Install scripts (Python nodes)
catkin_install_python(PROGRAMS
  src/led_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
