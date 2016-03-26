cmake_minimum_required(VERSION 2.8.3)
project(query_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation 
                                        sensor_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  Door.msg)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
