# Required CMake Version
cmake_minimum_required(VERSION 2.8.3)

# Package Name
project(machine_vision)

# Find catkin build system and specify needed packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(libuvc REQUIRED)

# Declare the package
catkin_package()

# Specify locations of header files
include_directories(include
  ${catkin_INCLdUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${libuvc_INCLUDE_DIRS}
)

# Declare executables with source files
add_executable(camera_publisher src/ros_camera_publisher.cpp)
add_executable(camera_subscriber src/ros_camera_subscriber.cpp)
add_executable(stereo_publisher src/stereo_cam_pub.cpp)

#Specify libraries against which to link
target_link_libraries(camera_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(camera_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(stereo_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${libuvc_LIBRARIES}
)
