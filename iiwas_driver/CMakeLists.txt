cmake_minimum_required(VERSION 2.8.3)
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
             iiwa_fri_client
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp iiwa_fri_client iiwas_srv
)

###########
## Build ##
###########
set(HEADERS include/configuration_manager.h
			include/iiwa_hw_interface.h
		    include/iiwa_ctrl_loop.h
)
set(SOURCES src/configuration_client.cpp
            src/fri_client.cpp
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
add_executable(fake_iiwas_mask src/iiwas_mask.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(fake_iiwas_mask ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} rt)
target_link_libraries(fake_iiwas_mask ${catkin_LIBRARIES})

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

