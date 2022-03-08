cmake_minimum_required(VERSION 3.0.2)
project(iiwas_pybullet)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
    pybullet_ros
    )

## System dependencies are found with CMake's conventions
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_service_files(
    FILES
    SetGains.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES iiwas_pybullet_gravity_compensation_hw_sim
    CATKIN_DEPENDS message_runtime roscpp pybullet_ros
    DEPENDS pinocchio
)

include_directories(
     include
     ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(iiwas_pybullet_gravity_compensation_hw_sim
    src/gravity_compensation_hw_sim.cpp)
target_link_libraries(iiwas_pybullet_gravity_compensation_hw_sim
    ${catkin_LIBRARIES}
    pinocchio::pinocchio)
add_dependencies(iiwas_pybullet_gravity_compensation_hw_sim
    ${iiwas_pybullet_gravity_compensation_hw_sim_EXPORTED_TARGETS}
    ${pybullet_ros_EXPORTED_TARGETS})
