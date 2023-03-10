cmake_minimum_required(VERSION 2.8.3)
project(scout_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    gazebo_msgs
    tf2 
    tf2_ros
    ugv_sdk
    scout_msgs
    mavros_msgs
)

find_package (Eigen3 REQUIRED)
    
# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scout_messenger
  CATKIN_DEPENDS ugv_sdk scout_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(scout_messenger STATIC src/scout_messenger.cpp)
target_link_libraries(scout_messenger ${catkin_LIBRARIES})
add_dependencies(scout_messenger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scout_base_node src/scout_base_node.cpp)
target_link_libraries(scout_base_node scout_messenger ${catkin_LIBRARIES})

add_executable(speed src/speed.cpp)
target_link_libraries(speed ${catkin_LIBRARIES})

add_executable(scout_vicon src/scout_vicon.cpp)
target_link_libraries(scout_vicon 
${catkin_LIBRARIES}
${Sophus_LIBRARIES}
)

add_executable(scout_vicon_lhj src/scout_vicon_lhj.cpp)
target_link_libraries(scout_vicon_lhj 
${catkin_LIBRARIES}
${Sophus_LIBRARIES}
)

add_executable(scout_vicon_sim src/scout_vicon_sim.cpp)
target_link_libraries(scout_vicon_sim 
${catkin_LIBRARIES}
${Sophus_LIBRARIES}
)

add_executable(scout_vicon_cam_lhj src/scout_vicon_cam_lhj.cpp)
target_link_libraries(scout_vicon_cam_lhj 
${catkin_LIBRARIES}
${Sophus_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS scout_messenger scout_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
