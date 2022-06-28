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
    dynamic_reconfigure
    )

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
        FILES
        BsplineSegmentMsg.msg
        BsplineTrajectoryMsg.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/Parameters.cfg
)


catkin_package(
    INCLUDE_DIRS include
    #   LIBRARIES iiwas_kinematics
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
add_executable(iiwas_dynamics_example
    src/iiwas_dynamics_example.cpp
    )

add_library(ff_joint_trajectory_controller
    src/ff_joint_trajectory_controller.cpp)

add_library(adrc_joint_trajectory_controller
    src/adrc/adrc_joint_trajectory_controller.cpp
    src/adrc/adrc_single.cpp)

add_library(bspline_joint_trajectory_controller
    src/bspline_joint_trajectory_controller.cpp)

add_library(bspline_ff_joint_trajectory_controller
        src/bspline_ff_joint_trajectory_controller.cpp)

# make sure configure headers are built before any node using them
add_dependencies(adrc_joint_trajectory_controller ${PROJECT_NAME}_gencfg)

target_compile_definitions(iiwas_dynamics_example PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_compile_definitions(ff_joint_trajectory_controller PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_compile_definitions(adrc_joint_trajectory_controller PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_compile_definitions(bspline_joint_trajectory_controller PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
target_compile_definitions(bspline_ff_joint_trajectory_controller PRIVATE ${PINOCCHIO_CFLAGS_OTHER})

target_link_libraries(iiwas_dynamics_example
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
    )

target_link_libraries(ff_joint_trajectory_controller
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
    )

target_link_libraries(adrc_joint_trajectory_controller
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
    )

target_link_libraries(bspline_joint_trajectory_controller
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
    tinysplinecxx
    )

target_link_libraries(bspline_ff_joint_trajectory_controller
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
