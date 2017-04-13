cmake_minimum_required(VERSION 2.8.3)
project(lcar_msgs)

find_package(Qt5Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
                                rospy
                                roscpp
                                message_generation
                                sensor_msgs
                                geometry_msgs
                                mavros_msgs )

include_directories(
    ${Qt5Widgets_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    include
    src )

add_message_files(
    DIRECTORY msg
    FILES
    Query.msg
    AccessPoint.msg
    AccessPointStamped.msg
    TargetLocal.msg
    TargetGlobal.msg
    InitResponse.msg )

add_service_files(
    DIRECTORY srv
    FILES
    ODParams.srv
    InitRequest.srv
    WorldMap.srv )

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs mavros_msgs )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES UiAdapter
    DEPENDS Qt5Widgets
    CATKIN_DEPENDS
        message_runtime
        sensor_msgs
        geometry_msgs
        mavros_msgs )

add_definitions(${Qt5_DEFINITIONS} -std=c++14 )

add_library(UiAdapter
            include/lcar_msgs/ui_adapter.h
            src/ui_adapter.cpp )

add_dependencies(UiAdapter
                    ${catkin_EXPORTED_TARGETS} )

target_link_libraries(UiAdapter
                        ${catkin_LIBRARIES}
                        Qt5::Widgets )

## Mark executables and/or libraries for installation
install(TARGETS UiAdapter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)