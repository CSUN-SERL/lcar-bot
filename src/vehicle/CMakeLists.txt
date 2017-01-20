cmake_minimum_required(VERSION 2.8.3)
project(vehicle)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
            tf
            mavros  
            roscpp
            std_msgs
            lcar_msgs
            mavros_msgs
            sensor_msgs
            geometry_msgs
            image_transport
            eigen_conversions )

#add lcar-bot/src/vehicle and lcar-bot/build to search dirs
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(vehicle_INCLUDE_DIRS
     #add these directories to cmake search paths
     include
     nodes
     src
      )

catkin_package( 
  INCLUDE_DIRS ${vehicle_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs)

## Specify additional locations of header files
include_directories (${catkin_INCLUDE_DIRS} 
		     ${vehicle_INCLUDE_DIRS}
                     ${CMAKE_CURRENT_BINARY_DIR})


#compiler flags and options
include(EnableCXX11)
include(MavrosMavlink)

file(GLOB vehicle_SRCS src/*.cpp )
list(REMOVE_ITEM vehicle_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/backup_control.cpp )



###########
## Build ##
###########

add_library(${PROJECT_NAME} ${vehicle_SRCS})

add_executable(backup_control_node 
    nodes/backup_control_node.cpp 
    src/backup_control.cpp)

add_executable(uav_control_test_node nodes/uav_control_test_node.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
add_dependencies(uav_control_test_node ${catkin_EXPORTED_TARGETS} )

add_dependencies(backup_control_node ${catkin_EXPORTED_TARGETS} )

target_link_libraries(backup_control_node  ${catkin_LIBRARIES} )
target_link_libraries(uav_control_test_node 
                        ${PROJECT_NAME} 
                        ${catkin_LIBRARIES} )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})