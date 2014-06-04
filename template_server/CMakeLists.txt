cmake_minimum_required(VERSION 2.8.3)
project(template_server)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  rospy
  template_markers
  template_server_protobuf
  visualization_msgs
  nasa_robot_teleop
)

catkin_python_setup()

#############
## Install ##
#############

catkin_package()
install(PROGRAMS
            scripts/server.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


