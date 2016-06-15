cmake_minimum_required(VERSION 2.8.3)
project(lcar_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation
                                        sensor_msgs
                                        geometry_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  Door.msg
  Target.msg)

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs)
