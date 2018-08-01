# rxKinFu: Moving Volume KinectFusion for 3D Perception and Robotics
#
# Author: Dimitrios Kanoulas (dkanoulas@gmail.com)

cmake_minimum_required(VERSION 2.8.3)
project(rxkinfu_ros)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse -msse2 -msse3 -O3 -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        roslib
                                        std_msgs
                                        rxkinfu
                                        tf
                                        image_transport
                                        cv_bridge)

catkin_package(
      CATKIN_DEPENDS roscpp rospy roslib std_msgs
      INCLUDE_DIRS include
      LIBRARIES rxkinfu)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
#find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common
                                         console
                                         cv_bridge
                                         image_transport
                                         io
                                         filter
                                         features
                                         segmentation
                                         search
                                         surface
                                         visualization)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(tf_conversions REQUIRED)
FIND_PACKAGE(eigen_conversions REQUIRED)
find_package(rxkinfu REQUIRED)
include_directories(${rxkinfu_INCLUDE_DIRS})
message("---------------------------- ${rxkinfu_INCLUDE_DIRS}")
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rxkinfu_ros
  CATKIN_DEPENDS roscpp
                 rospy
                 roslib
                 rxkinfu
                 tf
                 image_transport
                 cv_bridge
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include include/${PROJECT_NAME}/
                    ${Eigen_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${boost_INCLUDE_DIRS}
                            ${Eigen_INCLUDE_DIRS}
                            ${PCL_INCLUDE_DIRS})
link_directories(${ROS_LIB_FOLDER})
link_directories(${PCL_LIBRARY_DIRS})

## Specify additional locations of header files
set(HEADERS
  #include/${PROJECT_NAME}/pc_to_di.h
  include/${PROJECT_NAME}/kinfu_app_ros.h
)

set(SOURCES
  #src/pc_to_di.cpp
  src/kinfu_app_ros.cpp
  src/rxkinfu_ros.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Declare a cpp executable
add_executable(rxkinfu_ros_test src/rxkinfu_ros.cpp)
#add_executable(pc_to_di src/pc_to_di.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} rxkinfu)
add_dependencies(rxkinfu_ros_test ${catkin_EXPORTED_TARGETS} rxkinfu)
#add_dependencies(pc_to_di ${catkin_EXPORTED_TARGETS} rxkinfu)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${PCL_LIBRARIES}
                                      rxkinfu)

target_link_libraries(rxkinfu_ros_test ${catkin_LIBRARIES}
                                       ${PCL_LIBRARIES}
                                       ${PROJECT_NAME})

#target_link_libraries(pc_to_di ${catkin_LIBRARIES}
#                               ${PCL_LIBRARIES}
#                               ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rxkinfu_ros_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

message("************** ${catkin_INCLUDE_DIRS}")
