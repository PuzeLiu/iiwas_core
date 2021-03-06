cmake_minimum_required(VERSION 3.0.2)
project(iiwas_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    joint_trajectory_controller
    hardware_interface
    controller_manager
    controller_interface
    )

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy joint_trajectory_controller controller_interface controller_manager
    DEPENDS EIGEN3
)

###########
## Build ##
###########
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ff_joint_trajectory_controller
    src/ff_joint_trajectory_controller.cpp)

target_compile_definitions(ff_joint_trajectory_controller PRIVATE ${PINOCCHIO_CFLAGS_OTHER})

target_link_libraries(ff_joint_trajectory_controller
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
    )


#############
## Install ##
#############

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
