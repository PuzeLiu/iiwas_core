cmake_minimum_required(VERSION 3.0.2)
project(iiwas_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   )

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   #   LIBRARIES iiwas_kinematics
   CATKIN_DEPENDS roscpp rospy
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
   src/iiwas_dynamics.cpp)

target_compile_definitions(iiwas_dynamics_example PRIVATE ${PINOCCHIO_CFLAGS_OTHER})

target_link_libraries(iiwas_dynamics_example
   ${catkin_LIBRARIES}
   iiwas_kinematics
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

