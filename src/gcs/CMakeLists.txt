cmake_minimum_required(VERSION 2.8.3)
project(gcs) # ${PROJECT_NAME} == gcs

option(USEOSGEARTH "use OsgEarth" On)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            roslib
            cv_bridge
            image_transport
            vehicle
            std_msgs
            sensor_msgs
            lcar_msgs )

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Svg REQUIRED)

if(USEOSGEARTH)
    find_package(Qt5OpenGL REQUIRED)
    find_package(OpenSceneGraph REQUIRED osgDB)
    find_package(OsgEarth REQUIRED osgEarthQt)
else()
    find_package(Qt5WebKitWidgets REQUIRED)
endif()
    

set(gcs_INCLUDE_DIRS
        include )

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${gcs_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}    
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Svg_INCLUDE_DIRS} ) 

if(USEOSGEARTH)
    include_directories(
        ${Qt5OpenGL_INCLUDE_DIRS}
        ${OPENSCENEGRAPH_INCLUDE_DIRS}
        ${OSGEARTH_INCLUDE_DIRS} )
else()
    include_directories(
        ${Qt5WebKitWidget_INCLUDE_DIRS} )
endif()

#currently not necessary but it doesn't hurt
#catkin_python_setup()
    
set(DEPENDENCIES Qt5Widgets Qt5Svg)
if(USEOSGEARTH)
    list(APPEND DEPENDENCIES
        Qt5OpenGL
        OpenSceneGraph
        OsgEarth )
else()
    list(APPEND DEPENDENCIES
        Qt5WebKitWidgets )
endif()

catkin_package(
    INCLUDE_DIRS ${gcs_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
    DEPENDS ${DEPENDENCIES}
    CATKIN_DEPENDS
        roscpp
        cv_bridge
        vehicle
        std_msgs
        sensor_msgs
        lcar_msgs 
        image_transport )

file(GLOB gcs_SRCS src/lib/*.cpp )
file(GLOB gcs_HDRS include/qt/*.h )
list(REMOVE_ITEM gcs_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/include/qt/my_q_widget.h )

file(GLOB gcs_UIS resources/forms/*.ui )
if(USEOSGEARTH)
    list(REMOVE_ITEM gcs_UIS ${CMAKE_CURRENT_SOURCE_DIR}/include/qt/GCSMainWindow.ui)
else()
    list(REMOVE_ITEM gcs_UIS ${CMAKE_CURRENT_SOURCE_DIR}/include/qt/GCSMainWindow_OSG.ui)
endif()

file(GLOB gcs_RCCS resources/*.qrc )

#run qt rcc, moc & uic
qt5_add_resources(gcs_VFS ${gcs_RCCS}) #VFS = Virtual File System
qt5_wrap_cpp(gcs_MOCS ${gcs_HDRS})
qt5_wrap_ui(gcs_UIS_H ${gcs_UIS})


#compiler flags and options
add_definitions(${Qt5_DEFINITIONS} -std=c++11)
if(USEOSGEARTH)
    add_definitions(-DUSEOSGEARTH ${Qt5OpenGL_DEFINITIONS})
endif()

# Declare executables with source files
#add_executable(gps_publisher src/vehicle/gps_publisher.cpp)
#add_executable(gps_demo src/vehicle/gps_demo.cpp)
#add_executable(safe_mocap src/vehicle/safe_mocap.cpp)

add_library(${PROJECT_NAME}
            ${gcs_SRCS}
            ${gcs_MOCS}
            ${gcs_UIS_H}
            ${gcs_VFS} )

#library dependencies
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(gcs_node src/nodes/gcs_main_node.cpp)

#executable dependencies
#add_dependencies(gcs_node ${catkin_EXPORTED_TARGETS})
#add_dependencies(gps_demo ${catkin_EXPORTED_TARGETS})

#Specify libraries against which to link
#target_link_libraries(gps_publisher ${catkin_LIBRARIES})
#target_link_libraries(gps_demo ${catkin_LIBRARIES})
#target_link_libraries(safe_mocap ${catkin_LIBRARIES})

target_link_libraries(gcs_node
                        ${PROJECT_NAME}
                        ${catkin_LIBRARIES}
                        Qt5::Widgets
                        Qt5::Svg )
if(USEOSGEARTH)
    target_link_libraries(gcs_node
                            ${OPENSCENEGRAPH_LIBRARIES}
                            ${OSGEARTH_LIBRARIES}
                            ${OSGEARTHQT_LIBRARY} )
else()
    target_link_libraries(gcs_node
                            Qt5::WebKitWidgets )
endif()
