cmake_minimum_required(VERSION 3.0.2)
project(iiwas_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             controller_interface
             controller_manager
             hardware_interface
             urdf
             iiwas_srv
             sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp iiwas_srv
)

###########
## Build ##
###########
set(HEADERS include/iiwas_driver/configuration_manager.h
						include/iiwas_driver/iiwa_hw_interface.h
						include/iiwas_driver/iiwa_ctrl_loop.h
)
set(SOURCES src/configuration_client.cpp
            src/iiwas_driver_node.cpp
            src/iiwa_hw_interface.cpp
	        	src/iiwa_ctrl_loop.cpp
	        	src/configuration_manager.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_executable(iiwas_fake_joint_publisher src/iiwas_fake_joint_publisher.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(iiwas_fake_joint_publisher ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} rt)
target_link_libraries(iiwas_fake_joint_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

