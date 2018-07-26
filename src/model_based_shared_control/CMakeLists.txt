cmake_minimum_required(VERSION 2.8.3)
project(model_based_shared_control)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  visualization_msgs
  tf
  cmake_modules
)

add_message_files(
  FILES
  State.msg
  Control.msg
)

## Generate services in the 'srv' folder
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime
  geometry_msgs
  visualization_msgs
  cmake_modules
  tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Collect Data Node
add_executable(collect_data src/collect_data.cpp)
add_dependencies(collect_data ${catkin_EXPORTED_TARGETS} model_based_shared_control_generate_messages_cpp)
target_link_libraries(collect_data ${catkin_LIBRARIES} armadillo)

# Shared Control Node
add_executable(model_based_shared_control src/model_based_shared_control.cpp)
add_dependencies(model_based_shared_control ${catkin_EXPORTED_TARGETS} model_based_shared_control_generate_messages_cpp)
target_link_libraries(model_based_shared_control ${catkin_LIBRARIES} armadillo)

# Online Shared Control Node
add_executable(online_model_based_shared_control src/online_model_based_shared_control.cpp)
add_dependencies(online_model_based_shared_control ${catkin_EXPORTED_TARGETS} model_based_shared_control_generate_messages_cpp)
target_link_libraries(online_model_based_shared_control ${catkin_LIBRARIES} armadillo)
