cmake_minimum_required(VERSION 2.8.3)
project(template_server_protobuf)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

SET(template_server_protobuf_DIR ${PROJECT_SOURCE_DIR})
## Find catkin macros and libraries
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(ProtocolBuffers REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

# protobuf
include_directories(${PROTOBUF_INCLUDE_DIR})
run_genproto()

#############
## Install ##
#############

catkin_package(INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/proto_gen/cpp/)
