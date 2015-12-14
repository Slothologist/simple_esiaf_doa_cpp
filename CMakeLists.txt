cmake_minimum_required(VERSION 2.8.3)
project(simple_robot_gaze_tools)

SET(ENV{ROS_LANG_DISABLE} "genjava")

INCLUDE(FindPkgConfig)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS roscpp std_msgs sensor_msgs)
find_package(dlib REQUIRED)
find_package(nmpt 0.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# OpenCV
SET(OpenCV_MIN_VERSION "2.3.1")
find_path(OpenCV_DIR NAMES opencv-config.cmake OpenCVConfig.cmake PATH_SUFFIXES lib/cmake lib)
FIND_PACKAGE( OpenCV REQUIRED COMPONENTS core contrib features2d imgproc legacy objdetect video highgui)
IF(OpenCV_FOUND)
        IF(${OpenCV_VERSION} VERSION_LESS ${OpenCV_MIN_VERSION} AND (SUPPORT_VIDEO OR SUPPORT_IMAGE) )
                MESSAGE(FATAL_ERROR "OpenCV version (${OpenCV_VERSION} is present, while ${OpenCV_MIN_VERSION}+ is required")
        ENDIF()
        INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
        MESSAGE("Found OpenCV ${OpenCV_VERSION} libraries ${OpenCV_LIBRARIES} and headers ${OpenCV_INCLUDE_DIRS}")
        IF(${OpenCV_VERSION_MAJOR} MATCHES 2 AND ${OpenCV_VERSION_MINOR} MATCHES 2)
                ADD_DEFINITIONS(-DUSE_OPENCV22)
        ELSE()
                ADD_DEFINITIONS(-DUSE_OPENCV)
        ENDIF()
ELSE()
        MESSAGE(FATAL_ERROR "OpenCV not found.")
ENDIF()

PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp>=0.5)

SET(ROS_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE DEBUG)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(BEFORE ${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIRS} ${NMPT_INCLUDE_DIRS} ${DLIB_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

link_directories (${Boost_LIBRARY_DIRS} ${YAMLCPP_LIBRARY_DIRS} ${NMPT_LIBRARY_DIRS} ${DLIB_LIBRARY_DIRS} ${OpenCV_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") #-std=c++0x

# Unfortuantely this is necessary for my ide (qt-creator) to resolve dependencies...
file(GLOB DUMMY_HEADER_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.hpp src/*.hpp)

add_executable(simple_robot_gaze_tools
			src/main.cpp
                        src/main.hpp
			${DUMMY_HEADER_LIST}
)

## Add cmake target dependencies of the executable/library
## As an example, message headers may need to be generated before nodes
## add_dependencies(simple_robot_gaze_tools ${catkin_EXPORTED_TARGETS})

target_link_libraries(simple_robot_gaze_tools
                        ${Boost_LIBRARIES}
                        ${catkin_LIBRARIES}
                        ${YAMLCPP_LIBRARIES}
                        ${NMPT_LIBRARIES}
                        ${DLIB_LIBRARY_DIRS}
                        ${OpenCV_LIBRARIES}
)

set_property(TARGET simple_robot_gaze_tools PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(simple_robot_gaze_tools PROPERTIES COMPILE_FLAGS -DCMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")


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

## Mark executables and/or libraries for installation
#install(TARGETS simple_robot_gaze_tools
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION bin
#)

install(TARGETS simple_robot_gaze_tools
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION bin
)


## Mark cpp header files for installation
#install(DIRECTORY include/simple_robot_gaze_tools
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
