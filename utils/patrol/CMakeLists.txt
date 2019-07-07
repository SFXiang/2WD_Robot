cmake_minimum_required(VERSION 2.8.3)
project(patrol)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mr_patrol
#  CATKIN_DEPENDS actionlib roscpp
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ executable
add_executable(patrol src/patrol_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(patrol ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(patrol
  ${catkin_LIBRARIES}
)



