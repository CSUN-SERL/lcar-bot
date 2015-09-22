# Required CMake Version
cmake_minimum_required(VERSION 2.8.3)

# Package Name
project(machine_vision)

# Find catkin build system and specify needed packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
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
# Example:
#add_executable(main_node src/main_node.cpp)

#Specify libraries against which to link
# Example:
#target_link_libraries(main_node
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)
