cmake_minimum_required (VERSION 2.8.3)
project(pose_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  )

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME}
  src/pose_publisher.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
