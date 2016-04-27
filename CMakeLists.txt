cmake_minimum_required(VERSION 2.8.3)
project(simple_robot_gaze_tools)

SET(ENV{ROS_LANG_DISABLE} "genjava")

FIND_PACKAGE(catkin COMPONENTS roscpp std_msgs sensor_msgs image_transport cv_bridge)
FIND_PACKAGE(dlib REQUIRED)
FIND_PACKAGE(nmpt 0.7 REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system program_options)
SET(RSX_MIN_VERSION "0.11")
FIND_PACKAGE(Boost ${RST_MIN_VERSION} REQUIRED COMPONENTS system program_options)
FIND_PACKAGE(RSC ${RST_MIN_VERSION} REQUIRED)
FIND_PACKAGE(RSB ${RST_MIN_VERSION} REQUIRED)
FIND_PACKAGE(RST ${RST_MIN_VERSION} REQUIRED sandbox)
FIND_PACKAGE(rst-converters ${RST_MIN_VERSION} REQUIRED)

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

SET(OpenCV_MIN_VERSION "2.4.10")
FIND_PATH(OpenCV_DIR NAMES opencv-config.cmake OpenCVConfig.cmake PATH_SUFFIXES lib/cmake lib)
FIND_PACKAGE( OpenCV ${OpenCV_MIN_VERSION} REQUIRED COMPONENTS core contrib features2d imgproc legacy objdetect video highgui)
IF(OpenCV_FOUND)
        IF(${OpenCV_VERSION} VERSION_LESS ${OpenCV_MIN_VERSION} AND (SUPPORT_VIDEO OR SUPPORT_IMAGE) )
                MESSAGE(FATAL_ERROR "OpenCV version (${OpenCV_VERSION} is present, while ${OpenCV_MIN_VERSION}+ is required")
        ENDIF()
        include_directories(${OpenCV_INCLUDE_DIRS})
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

# PKG_CHECK_MODULES(XIMEA REQUIRED xiapi>=4.3.6)
# IF (XIMEA_FOUND)
#   MESSAGE("-- using ximea xiAPI version ${XIMEA_VERSION} from ${XIMEA_PREFIX}")
# ELSE (XIMEA_FOUND)
#   message(FATAL_ERROR "ERROR: Cannot find ximea api")
# ENDIF (XIMEA_FOUND)

SET(ROS_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Release)

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

include_directories(BEFORE include ${Boost_INCLUDE_DIRS} ${RSB_INCLUDE_DIRS} ${RSC_INCLUDE_DIRS} ${RST_INCLUDE_DIRS} ${RST_CONVERTERS_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIRS} ${NMPT_INCLUDE_DIRS} ${dlib_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS} ${YAMLCPP_LIBRARY_DIRS} ${NMPT_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# Unfortuantely this is necessary for my ide (qt-creator) to resolve dependencies...
file(GLOB DUMMY_HEADER_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h src/*.h)

add_executable(simple_robot_gaze_tools
        src/main.cpp
        src/Faces.cpp
        src/Saliency.cpp
        src/Grabber.cpp
        src/Grabber_RSB.cpp
        # src/Grabber_XIMEA.cpp
        src/Grabber_ROS.cpp
        ${DUMMY_HEADER_LIST}
)

target_link_libraries(simple_robot_gaze_tools
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${NMPT_LIBRARIES}
        ${dlib_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${RSB_LIBRARIES}
        ${RST_CONVERTERS_LIBRARIES}
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
