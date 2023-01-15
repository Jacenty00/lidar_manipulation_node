cmake_minimum_required(VERSION 2.8.3)
project(laser_manipulation)

## Find catkin macros and libraries
find_package(catkin REQUIRED tf roscpp sensor_msgs ford_msgs pedsim_msgs geometry_msgs yocs_math_toolkit)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tf roscpp sensor_msgs geometry_msgs ford_msgs pedsim_msgs yocs_math_toolkit
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/laser_manipulation_node.cpp)

## Add cmake target dependencies of the executable/library
# add_dependencies(${PROJECT_NAME}_node yocs_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
                 scripts/agent_pos_gen.py
                 scripts/agents_visualizer.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# TODO
