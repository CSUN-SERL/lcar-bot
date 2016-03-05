# Required CMake Version
cmake_minimum_required(VERSION 2.8.3)

# Package Name
project(machine_vision)

# Find catkin build system and specify needed packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge # requires opencv 3 version
  image_transport
  sensor_msgs
  roscpp
  roslib
  std_msgs
<<<<<<< HEAD
  image_transport
=======
  camera_info_manager
>>>>>>> f7a6bf2539b82f7b446b8dce84b6005bd96f88c3
)

find_package(OpenCV 3 REQUIRED)
find_package(libuvc REQUIRED)

#Enable C++11 Support
add_definitions(-std=c++11)

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
<<<<<<< HEAD
add_executable(object_detection src/object_detect.cpp)

=======
#<<<<<<< HEAD
# add_executable(stereo_driver src/ros_camera_driver.cpp src/camera_driver_main.cpp)
#=======
add_executable(stereo_driver src/stereo_driver.cpp src/stereo_driver_main.cpp)
#>>>>>>> 4bac26099c102b8ebb1d5427c8ad0a88dce04c73
>>>>>>> f7a6bf2539b82f7b446b8dce84b6005bd96f88c3

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

<<<<<<< HEAD
target_link_libraries(object_detection
=======
target_link_libraries(stereo_driver
>>>>>>> f7a6bf2539b82f7b446b8dce84b6005bd96f88c3
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${libuvc_LIBRARIES}
)
