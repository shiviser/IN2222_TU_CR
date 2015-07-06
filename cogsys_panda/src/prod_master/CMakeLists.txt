cmake_minimum_required(VERSION 2.8.3)
project(prod_master)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  camera_srvs
  comm_lib
  tf
  perception_msgs
  perception_srvs
  robot_control_srvs
  gripper_control_srvs
  shape_detect_srvs
  prod_master_srvs
  robotino_controller
)

INCLUDE(${CMAKE_ROOT}/Modules/FindQt4.cmake)
FIND_PACKAGE(OpenCV REQUIRED )
INCLUDE(${PROJECT_SOURCE_DIR}/Modules/FindEigen3.cmake)


catkin_package(
 INCLUDE_DIRS include include/shapes_detector include/bot_controller include/robotino_controller include/prod_master
 LIBRARIES prod_master
 CATKIN_DEPENDS roscpp roslib comm_lib shape_detect_srvs prod_master_srvs robotino_controller
#  DEPENDS system_lib
)


INCLUDE_DIRECTORIES(
        ${QT_INCLUDES}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/src
        ${OPENCV_INCLUDES}
        ${EIGEN3_INCLUDE_DIR}
        include
        include/shapes_detector
        include/bot_controller
        include/robotino_controller
        include/prod_master
        ${catkin_INCLUDE_DIRS}
        /usr/include
        /usr/local/include
)

LINK_DIRECTORIES(
        /usr/local/lib
        /usr/lib
)


SET(HDRS
        include/shapes_detector/shapes_detector.h
        include/bot_controller/bot_controller.h
        include/robotino_controller/robotino_controller.h
        include/prod_master/prod_master.h
        include/main.h
)

SET(SRCS
        src/shapes_detector/shapes_detector.cpp
        src/bot_controller/bot_controller.cpp
        src/robotino_controller/robotino_controller.cpp
        src/prod_master/prod_master.cpp
        src/main.cpp
)

## Declare a cpp library
# TODO - add the object detector too but there's some opencv problem
# maybe this will solve the problem:
add_library(prod_master
        include/bot_controller/bot_controller.h
#        include/shapes_detector/shapes_detector.h
        src/bot_controller/bot_controller.cpp
#        src/shapes_detector/shapes_detector.cpp
)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(prod_master_node ${HDRS} ${SRCS})
target_link_libraries(prod_master_node
    ${catkin_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${OpenCV_LIBRARIES}
)
add_dependencies(prod_master_node shape_detect_srvs_generate_messages_cpp robotino_controller_srvs_generate_messages_cpp prod_master_srvs_generate_messages_cpp)


#cmake_minimum_required(VERSION 2.8.3)
#project(prod_master)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)

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
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prod_master
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(prod_master
#   src/${PROJECT_NAME}/prod_master.cpp
# )

## Declare a cpp executable
# add_executable(prod_master_node src/prod_master_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(prod_master_node prod_master_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(prod_master_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS prod_master prod_master_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_prod_master.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
