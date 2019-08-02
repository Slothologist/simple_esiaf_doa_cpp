cmake_minimum_required(VERSION 2.8.3)
project(simple_esiaf_doa_cpp)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        esiaf_ros
        )

FIND_PACKAGE(Boost REQUIRED COMPONENTS system program_options)

SET(ROS_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Release)

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
)

###########
## Build ##
###########

include_directories(BEFORE include ${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} )
link_directories(${Boost_LIBRARY_DIRS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# Unfortuantely this is necessary for my ide (qt-creator) to resolve dependencies...
file(GLOB DUMMY_HEADER_LIST RELATIVE audio include/*.h src/*.h)

add_executable(ssloc
        src/ssloc.cpp	
        ${DUMMY_HEADER_LIST}
)

target_link_libraries(ssloc
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)

set_property(TARGET ssloc PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(ssloc PROPERTIES COMPILE_FLAGS -DCMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")


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

install(TARGETS ssloc
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION bin
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
