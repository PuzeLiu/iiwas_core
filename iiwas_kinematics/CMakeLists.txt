cmake_minimum_required(VERSION 3.0.2)
project(iiwas_kinematics)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy)

find_package(Eigen3 REQUIRED)
catkin_python_setup()

catkin_package(
          INCLUDE_DIRS include
          LIBRARIES iiwas_kinematics
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
)

## Declare a C++ library
add_library(${PROJECT_NAME}
            src/iiwas_kinematics.cpp)
            
add_executable(${PROJECT_NAME}_example src/example.cpp)

target_link_libraries(${PROJECT_NAME}_example
        ${catkin_LIBRARIES}
        iiwas_kinematics
        )

add_subdirectory(pybind11)
pybind11_add_module(iiwas_kinematics_py src/python_bindings.cpp src/iiwas_kinematics.cpp)

add_custom_command(TARGET iiwas_kinematics_py POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:iiwas_kinematics_py>
    							     ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
