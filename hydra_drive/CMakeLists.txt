cmake_minimum_required(VERSION 2.8.3)
project(hydra_drive)

set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost 1.54.0 COMPONENTS
  system
  coroutine
  context
  )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_definitions(
  "-std=c++11"
)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TreadSpeeds.msg
  PowerLevels.msg
)

add_custom_command(
  OUTPUT
  ${hydra_drive_BINARY_DIR}/arduino/ros_lib
  arduino/ros_lib/hydra_drive/TreadSpeeds.h
  arduino/ros_lib/hydra_drive/PowerLevels.h
  COMMAND rosrun rosserial_arduino make_libraries.py ${hydra_drive_BINARY_DIR}/arduino/ros_lib

  DEPENDENCIES
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include/hydra_drive/TreadSpeeds.h
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include/hydra_drive/PowerLevels.h
  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
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
#  LIBRARIES hydra_drive
 CATKIN_DEPENDS message_runtime
#  DEPENDS std_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
# TODO: Check names of system library include directories (std_msgs, geometry_msgs)
include_directories(
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

add_library(hydra_drive_twist_speed_mapper src/TwistSpeedMapper.cc)
add_dependencies(hydra_drive_twist_speed_mapper
  hydra_drive_generate_messages_cpp)

# On multiarch (i.e. recent Debian) systems, architecture-specific library
# files are stored in /usr/lib/`dpkg-architecture` -- e.g.,
# /usr/lib/i386-linux-gnu or /usr/lib/x86_64-linux-gnu.  ROS simply assumes
# that libraries exist in the same places they were found on the system it was
# built on, which is not always the case.
#
# We fix this by taking each library and stripping down the path to get the
# actual library name, then using `find_library` to determine its path on the
# local system.
set(hydra_drive_twist_speed_mapper_LIBS ${catkin_LIBRARIES} ${geometry_msgs_LIBRARIES})
string(REPLACE "/usr/lib/lib" "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/lib" hydra_drive_twist_speed_mapper_LIBS "${hydra_drive_twist_speed_mapper_LIBS}")
foreach(lib ${hydra_drive_twist_speed_mapper_LIBS})
  get_filename_component(libfile "${lib}" NAME_WE)
  string(REGEX REPLACE "^lib" "" libfile "${libfile}")
  
  # if(NOT "${lib}" MATCHES "boost")
  find_library(liblib "${libfile}")
  list(APPEND hydra_drive_twist_speed_mapper_LIBS_NEW "${liblib}")
  # endif(NOT "${lib}" MATCHES "boost")
endforeach(lib)
set(hydra_drive_twist_speed_mapper_LIBS "${hydra_drive_twist_speed_mapper_LIBS_NEW}")


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/hydra_drive_logic.py
   scripts/pygame_drive_control.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS hydra_drive hydra_drive_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hydra_drive.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
