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
  usb_cam
)

find_package(OpenCV REQUIRED)


# Declare the package
catkin_package()

# Specify locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare executables with source files
#add_executable(cam_parallel src/camera_parallel.cpp)
add_executable(cam_serial src/camera_serial.cpp)

#Specify libraries against which to link
#target_link_libraries(cam_parallel
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)

target_link_libraries(cam_serial
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
