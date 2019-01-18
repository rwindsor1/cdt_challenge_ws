cmake_minimum_required(VERSION 2.8.3)

project(position_controller_cdt)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  joy_manager_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS eigen_conversions roscpp joy_manager_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)





######################################################
## Declare a C++ library
add_library(position_controller_cdt
  src/position_controller_cdt/position_controller_cdt.cpp
)

add_dependencies(position_controller_cdt ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_cdt ${catkin_LIBRARIES})


## Application
add_executable(position_controller_cdt_node
  src/position_controller_cdt/position_controller_cdt_node.cpp )
target_link_libraries(position_controller_cdt_node ${catkin_LIBRARIES} position_controller_cdt)




#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


############ Python
#catkin_python_setup()
#catkin_install_python(PROGRAMS scripts/high_level_actions/high_level_actions.py
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
