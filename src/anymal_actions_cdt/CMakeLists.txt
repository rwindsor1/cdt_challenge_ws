# Project configuration
cmake_minimum_required (VERSION 2.8)
project(anymal_actions_cdt)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  #INCLUDE_DIRS
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)

install(DIRECTORY actions
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES actions.yaml collections.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
