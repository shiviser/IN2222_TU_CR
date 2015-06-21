cmake_minimum_required(VERSION 2.8.3)
project(trans_cam_to_gripper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  trans_cam_to_gripper_srvs

)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES mt_gripper_helper
 CATKIN_DEPENDS roscpp trans_cam_to_gripper_srvs
#  DEPENDS system_lib
)


INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/src
        include
        ${catkin_INCLUDE_DIRS}
        /usr/include
        /usr/local/include
)


SET(HDRS
    include/main.h
)


SET(SRCS
    src/main.cpp
)


LINK_DIRECTORIES(
        /usr/local/lib
        /usr/lib
)


add_executable(trans_cam_to_gripper_node ${HDRS} ${SRCS})
target_link_libraries(trans_cam_to_gripper_node
    ${catkin_LIBRARIES}
)
add_dependencies(trans_cam_to_gripper_node trans_cam_to_gripper_srvs_generate_messages_cpp)


##common commands for building c++ executables and libraries
##rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
##target_link_libraries(${PROJECT_NAME} another_library)
##rosbuild_add_boost_directories()
##rosbuild_link_boost(${PROJECT_NAME} thread)
#add_executable(trans_cam_to_gripper_node ${HDRS} ${SRCS})
#target_link_libraries(trans_cam_to_gripper_node
#    ${catkin_LIBRARIES}
#)