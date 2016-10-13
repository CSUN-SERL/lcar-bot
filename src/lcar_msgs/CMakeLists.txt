cmake_minimum_required(VERSION 2.8.3)
project(lcar_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation
                                        sensor_msgs
                                        geometry_msgs
                                        mavros_msgs)

include_directories(include)

add_message_files(
    DIRECTORY msg
    FILES
    Query.msg
    AccessPoint.msg
    AccessPointStamped.msg
    TargetLocal.msg
    TargetGlobal.msg)

add_service_files(
    DIRECTORY srv
    FILES
    Init.srv
)

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs mavros_msgs)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs mavros_msgs)
