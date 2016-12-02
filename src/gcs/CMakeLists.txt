cmake_minimum_required(VERSION 2.8.3)
project(gcs) # ${PROJECT_NAME} == gcs

find_package(catkin REQUIRED COMPONENTS
            tf
            mavros     
            roscpp
            rospy
            roslib
            std_msgs
            cv_bridge
            lcar_msgs
            mavros_msgs
            sensor_msgs
            geometry_msgs
            image_transport )

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Svg REQUIRED)
find_package(Qt5WebKitWidgets REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

#add lcar-bot/src/gcs and lcar-bot/build to search dirs
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(gcs_INCLUDE_DIRECTORIES
     #add these directories to cmake search paths
     include
     src
     resources )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${gcs_INCLUDE_DIRECTORIES}     
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5Svg_INCLUDE_DIRS}
                    ${Qt5WebKitWidgets_INCLUDE_DIRS}  
                    ${OpenCV_INCLUDE_DIRS} 
#                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS} )


#currently not necessary but it doesn't hurt
#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ${gcs_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME} vehicle
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs )

file(GLOB gcs_SRCS src/lib/*.cpp )

file(GLOB vehicle_SRCS src/vehicle/*.cpp )
list(REMOVE_ITEM vehicle_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/vehicle/init_node.cpp
                              ${CMAKE_CURRENT_SOURCE_DIR}/src/vehicle/backup_control.cpp )

file(GLOB gcs_HDRS include/qt/*.h )
list(REMOVE_ITEM gcs_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/include/qt/my_q_widget.h )

file(GLOB gcs_UIS resources/*.ui )

file(GLOB gcs_RCCS resources/*.qrc )

#run qt rcc, moc & uic
qt5_add_resources(gcs_VFS ${gcs_RCCS}) #VFIR = Virtual File System Resources
qt5_wrap_cpp(gcs_MOCS ${gcs_HDRS})
qt5_wrap_ui(gcs_UIS_H ${gcs_UIS})


#compiler flags and options
include(EnableCXX11)
include(MavrosMavlink)
add_definitions(${Qt5_DEFINITIONS})

# Declare executables with source files
add_executable(backup_control src/vehicle/backup_control.cpp)
add_executable(init_node src/vehicle/init_node.cpp)
#add_executable(gps_publisher src/vehicle/gps_publisher.cpp)
#add_executable(gps_demo src/vehicle/gps_demo.cpp)
#add_executable(safe_mocap src/vehicle/safe_mocap.cpp)

add_library(${PROJECT_NAME}
            ${gcs_SRCS}
            ${gcs_MOCS}
            ${gcs_UIS_H}
            ${gcs_VFS} )

add_library(vehicle ${vehicle_SRCS} )

add_executable(gcs_node src/main.cpp)

#add_dependencies(uav_control ${catkin_EXPORTED_TARGETS})
#library dependencies
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(vehicle ${catkin_EXPORTED_TARGETS})

#executable dependencies
add_dependencies(init_node ${catkin_EXPORTED_TARGETS})
#add_dependencies(gcs_node ${catkin_EXPORTED_TARGETS})
#add_dependencies(gps_demo ${catkin_EXPORTED_TARGETS})

#Specify libraries against which to link
target_link_libraries(backup_control ${catkin_LIBRARIES})
target_link_libraries(init_node ${catkin_LIBRARIES})
#target_link_libraries(gps_publisher ${catkin_LIBRARIES})
#target_link_libraries(gps_demo ${catkin_LIBRARIES})
#target_link_libraries(safe_mocap ${catkin_LIBRARIES})
target_link_libraries(  gcs_node
                        ${PROJECT_NAME}
                        vehicle
                        ${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        Qt5::Widgets
                        Qt5::Svg
                        Qt5::WebKitWidgets )
