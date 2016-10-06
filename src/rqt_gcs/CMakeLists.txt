cmake_minimum_required(VERSION 2.8.3)
project(rqt_gcs)

find_package(catkin REQUIRED COMPONENTS
            tf
            mavros     
            roscpp
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

#add lcar-bot/src/rqt_gcs and lcar-bot/build to search dirs
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(rqt_gcs_INCLUDE_DIRECTORIES
     #add these directories to cmake search paths
     include
     src
     resources )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${rqt_gcs_INCLUDE_DIRECTORIES}     
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5Svg_INCLUDE_DIRS}
                    ${Qt5WebKitWidgets_INCLUDE_DIRS}  
                    ${OpenCV_INCLUDE_DIRS} 
#                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS} )


#currently not necessary but it doesn't hurt
#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ${rqt_gcs_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs )

file(GLOB rqt_gcs_SRCS src/rqt_gcs/*.cpp src/vehicle/*.cpp )
list(REMOVE_ITEM rqt_gcs_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/rqt_gcs/main.cpp )

file(GLOB rqt_gcs_HDRS include/rqt_gcs/*.h )
list(REMOVE_ITEM rqt_gcs_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/include/rqt_gcs/access_point.h 
                              ${CMAKE_CURRENT_SOURCE_DIR}/include/rqt_gcs/my_q_widget.h )

file(GLOB rqt_gcs_UIS resources/*.ui )

file(GLOB rqt_gcs_RCCS resources/*.qrc )

#run qt rcc, moc & uic
qt5_add_resources(rqt_gcs_VFIR ${rqt_gcs_RCCS}) #VFIR = Virtual File System Resources
qt5_wrap_cpp(rqt_gcs_MOCS ${rqt_gcs_HDRS})
qt5_wrap_ui(rqt_gcs_UIS_H ${rqt_gcs_UIS})


#compiler flags and options
include(EnableCXX11)
include(MavrosMavlink)
add_definitions(${Qt5_DEFINITIONS})

# Declare executables with source files
add_executable(backup_control src/vehicle/backup_control.cpp)
#add_executable(uav_control src/vehicle/uav_control.cpp)
#add_executable(gps_publisher src/vehicle/gps_publisher.cpp)
#add_executable(gps_demo src/vehicle/gps_demo.cpp)
#add_executable(safe_mocap src/vehicle/safe_mocap.cpp)

add_library(_islurp
            ${rqt_gcs_SRCS}
            ${rqt_gcs_MOCS}
            ${rqt_gcs_UIS_H}
            ${rqt_gcs_VFIR} )

add_executable(${PROJECT_NAME} src/rqt_gcs/main.cpp)

#add_dependencies(uav_control ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#add_dependencies(gps_demo ${catkin_EXPORTED_TARGETS})

#Specify libraries against which to link
target_link_libraries(backup_control ${catkin_LIBRARIES})
#target_link_libraries(uav_control ${catkin_LIBRARIES})
#target_link_libraries(gps_publisher ${catkin_LIBRARIES})
#target_link_libraries(gps_demo ${catkin_LIBRARIES})
#target_link_libraries(safe_mocap ${catkin_LIBRARIES})
target_link_libraries(  ${PROJECT_NAME}
                        _islurp
                        ${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        Qt5::Widgets
                        Qt5::Svg
                        Qt5::WebKitWidgets )
