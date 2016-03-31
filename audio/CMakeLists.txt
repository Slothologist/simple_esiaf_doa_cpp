cmake_minimum_required(VERSION 2.8.3)
project(simple_robot_gaze_tools_audio)

SET(ENV{ROS_LANG_DISABLE} "genjava")
INCLUDE(FindPkgConfig)

FIND_PACKAGE(catkin COMPONENTS roscpp std_msgs sensor_msgs)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system program_options)
FIND_PACKAGE(ALSA REQUIRED)
FIND_PACKAGE(RSB REQUIRED)
FIND_PACKAGE(RST REQUIRED)
FIND_PACKAGE(RSC REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp>=0.5)

SET(ROS_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Release)

#######################################
## Declare ROS messages and services ##
#######################################

#Generate actions:
#add_action_files(
# 	DIRECTORY action
# 	FILES register_access.action
# )


## Generate messages in the 'msg' folder
#add_message_files(
#	FILES
#	joint_state.msg
#	device_info.msg
#	device_state.msg
#	bus_state.msg
#	health_state.msg
#	smoothmotion_joint_target.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   register_access.srv
#   server_state.srv
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#	DEPENDENCIES
#	std_msgs
#
#)

#
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
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(BEFORE include ${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIRS} ${ALSA_INCLUDE_DIRS} ${RST_INCLUDE_DIRS} ${RSC_INCLUDE_DIRS} ${RSB_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS} ${YAMLCPP_LIBRARY_DIRS} ${ALSA_LIBRARY_DIRS} ${RST_LIBRARY_DIRS} ${RSC_LIBRARY_DIRS} ${RSB_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# Unfortuantely this is necessary for my ide (qt-creator) to resolve dependencies...
file(GLOB DUMMY_HEADER_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h src/*.h)

add_executable(simple_robot_gaze_tools_ssloc
        src/ssloc.cpp	
        ${DUMMY_HEADER_LIST}
)

target_link_libraries(simple_robot_gaze_tools_ssloc
        ${RSB_LIBRARIES}
        ${RST_LIBRARIES}
        ${RSC_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${ALSA_LIBRARIES}
)

set_property(TARGET simple_robot_gaze_tools_ssloc PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(simple_robot_gaze_tools_ssloc PROPERTIES COMPILE_FLAGS -DCMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS simple_robot_gaze_tools_ssloc
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION bin
)


## Mark cpp header files for installation
#install(DIRECTORY include/simple_robot_gaze_tools_ssloc
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..
#   FILES_MATCHING
#      PATTERN "*_t.h"
#      PATTERN datatypes.h
#      PATTERN exceptions.h
#      PATTERN protocol_defines.h
#      PATTERN ".git" EXCLUDE
#)

# Config files
# install(DIRECTORY etc/
#   DESTINATION ${CATKIN_PACKAGE_ETC_DESTINATION}
#   FILES_MATCHING
#      PATTERN "config*"
#)


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_xsc3.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
