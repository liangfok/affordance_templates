cmake_minimum_required(VERSION 2.8.3)
project(rviz_template_panel)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# find ZMQ
find_package(ZeroMQ REQUIRED)
include_directories(${ZeroMQ_INCLUDE_DIR})

# add protobuf include
find_package(template_server_protobuf)
include_directories(${template_server_protobuf_DIR}/proto_gen/cpp)
#include_directories(/home/khambuch/hydro_workspace/src/affordance_templates/template_server_protobuf/proto_gen/cpp)

# find Qt
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
qt4_wrap_cpp(MOC_FILES
    src/TemplatePanel.hpp
    src/Affordance.hpp
)
set(SRC_FILES
    src/TemplatePanel.cpp
    src/Affordance.cpp
    ${MOC_FILES}
)

# everything else
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  rviz
  template_server
  template_server_protobuf
  visualization_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rviz_template_panel
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp roslib rviz template_server template_server_protobuf visualization_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SRC_FILES})


# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARY} AffordanceTemplateServerCmd)

