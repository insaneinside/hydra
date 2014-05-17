cmake_minimum_required(VERSION 2.8.3)
project(hydra_imu)

set(CATKIN_ENABLE_TESTING OFF CACHE BOOL "Don't test me, foo")
set(Boost_USE_MULTITHREADED OFF)
find_package(Boost REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hydra_common
  dynamic_reconfigure
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  nodelet
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
  vector3_int16.msg
  RawIMUData.msg
  IMUSensorConfiguration.msg
  IMUConfiguration.msg
  IMUData.msg
  IMUDataLimits.msg
  )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetIMUConfiguration.srv
  SetIMUConfiguration.srv
)

generate_dynamic_reconfigure_options(
  cfg/HoltWintersIMUFilter.py
  cfg/AHRS.py
  )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
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
 LIBRARIES hydra_imu
 CATKIN_DEPENDS geometry_msgs message_runtime roscpp sensor_msgs std_msgs nodelet dynamic_reconfigure
 DEPENDS system_lib
)

###########
## Build ##
###########

add_definitions(
  "-std=c++11"
)
link_directories("${catkin_LIBRARY_DIRS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(hydra_imu SHARED
  src/hydra_imu/IMUDataScaler.cc
  src/hydra_imu/HoltWintersIMUFilter.cc
  src/hydra_imu/MadgwickAHRS.cc
  src/hydra_imu/AHRS.cc)

add_dependencies(hydra_imu
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
  )
target_link_libraries(hydra_imu ${dynamic_reconfigure_LIBRARIES} ${tf_LIBRARIES})


# On multiarch (i.e. recent Debian) systems, architecture-specific library
# files are stored in /usr/lib/`dpkg-architecture` -- e.g.,
# /usr/lib/i386-linux-gnu or /usr/lib/x86_64-linux-gnu.  ROS simply assumes
# that libraries exist in the same places they were found on the system it was
# built on, which is not always the case.
#
# We fix this by taking each library and stripping down the path to get the
# actual library name, then using `find_library` to determine its path on the
# local system.
foreach(tgt hydra_imu)
  get_target_property("${tgt}_LIBS" "${tgt}" LINK_LIBRARIES)
  string(REPLACE "/usr/lib/lib" "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/lib" ${tgt}_LIBS "${${tgt}_LIBS}")
  foreach(lib ${${tgt}_LIBS})
    if("${lib}" MATCHES "boost" AND IS_ABSOLUTE "${lib}" AND NOT EXISTS "${lib}")
      string(REGEX REPLACE "-mt" "" lib "${lib}")
    endif("${lib}" MATCHES "boost" AND IS_ABSOLUTE "${lib}" AND NOT EXISTS "${lib}")

    if(IS_ABSOLUTE "${lib}" AND EXISTS "${lib}")
      list(APPEND "${tgt}_LIBS_NEW" "${lib}")
    else(IS_ABSOLUTE "${lib}" AND EXISTS "${lib}")
      get_filename_component(libfile "${lib}" NAME_WE)
      string(REGEX REPLACE "^lib" "" libfile "${libfile}")
      
      find_library(liblib NAMES "${libfile}"
        NO_DEFAULT_PATH
        NO_CMAKE_PATH
        NO_CMAKE_SYSTEM_PATH
        HINTS "${CATKIN_PACKAGE_LIB_DESTINATION}"
        PATHS
        "/opt/ros/hydro/lib/" "/usr/lib/"
        "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
        )
      if(liblib_FOUND)
        list(APPEND "${tgt}_LIBS_NEW" "${liblib}")
      else(liblib_FOUND)
        list(APPEND  "${tgt}_LIBS_NEW" "${libfile}")
        message(WARNING "Could not find ${libfile} (${lib})")
      endif(liblib_FOUND)
      # endif(NOT "${lib}" MATCHES "boost")
    endif(IS_ABSOLUTE "${lib}" AND EXISTS "${lib}")
  endforeach(lib)
  message(STATUS "${tgt}_LIBS: " "${${tgt}_LIBS_NEW}")
  set_property(TARGET "${tgt}" PROPERTY LINK_LIBRARIES
    "${${tgt}_LIBS_NEW}")
endforeach(tgt)


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
install(TARGETS hydra_imu
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h[h]"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hydra_imu.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
