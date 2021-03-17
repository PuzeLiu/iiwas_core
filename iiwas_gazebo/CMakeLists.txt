cmake_minimum_required(VERSION 3.0.2)
project(iiwas_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    gazebo_ros_control)

find_package(gazebo REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp gazebo_ros gazebo_ros_control
    DEPENDS EIGEN3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(iiwas_gazebo_gravity_compensation_hw_sim
    src/gravity_compensation_hw_sim.cpp)
target_link_libraries(iiwas_gazebo_gravity_compensation_hw_sim
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    pinocchio::pinocchio)


install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
