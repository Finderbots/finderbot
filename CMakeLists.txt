cmake_minimum_required(VERSION 2.8.3)
project(finderbot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  gazebo_ros
  angles
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

find_package(gazebo REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PF_Input.msg
   Pose.msg
 )

## Generate services in the 'srv' folder
add_service_files(
    FILES
    GetMap.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


catkin_package(
  INCLUDE_DIRS 
  include

#  LIBRARIES finderbot

  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs
  message_runtime
  gazebo_ros
  angles
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS} 
  ${SDFormat_INCLUDE_DIRS}
)

add_executable(mini_mapping_node src/mapping_node.cpp src/mapping.cpp src/ray_caster.cpp)


add_executable(SLAM_node src/SLAM_node.cpp)
add_executable(simple_motion_node src/simple_motion_node.cpp)

add_executable(global_map_node src/global_map_node.cpp src/global_map_builder.cpp src/ray_caster.cpp)

add_executable(distance_grid_test_node src/distance_grid_test.cpp src/DistanceGrid.cpp)


#add_executable(uwb_node src/uwb.cpp)
#set_source_files_properties(src/uwb.cpp PROPERTIES COMPILE_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0 -std=gnu++11")

add_executable(a_star_test src/a_star_test.cpp src/planner.cpp src/DistanceGrid.cpp)

add_executable(exploration_test src/exploration_test.cpp src/exploration.cpp src/planner.cpp src/DistanceGrid.cpp)

add_executable(path_execution_node src/path_execution_node.cpp src/path_execution.cpp src/exploration.cpp src/planner.cpp src/DistanceGrid.cpp)

##(test_path_exec_node src/test_path_exec_node.cpp src/path_execution_node.cpp src/exploration.cpp src/planner.cpp src/DistanceGrid.cpp)

## add_executable(dummy_map_node src/dummy_map_node.cpp src/mapping.cpp src/ray_caster.cpp)

add_dependencies(global_map_node finderbot_generate_messages_cpp finderbot_gencpp)
add_dependencies(SLAM_node finderbot_generate_messages_cpp)
add_dependencies(path_execution_node finderbot_generate_messages_cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



### Specify libraries to link a library or executable target against

#target_link_libraries(
#    uwb_node
#    ${catkin_LIBRARIES}
#    ~/catkin_ws/src/finderbot/lib/libModuleConnector.so
#)


target_link_libraries(
  exploration_test
  ${catkin_LIBRARIES}
)

target_link_libraries(
  path_execution_node
  ${catkin_LIBRARIES}
)
target_link_libraries(
   SLAM_node
   ${catkin_LIBRARIES}
)

target_link_libraries(
   mini_mapping_node
   ${catkin_LIBRARIES}
)
target_link_libraries(
  simple_motion_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  global_map_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  distance_grid_test_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  a_star_test
  ${catkin_LIBRARIES}
)

## target_link_libraries(
##   test_path_exec_node
##   ${catkin_LIBRARIES}
## )

## target_link_libraries(
##   dummy_map_node
##   ${catkin_LIBRARIES}
## )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_finderbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
