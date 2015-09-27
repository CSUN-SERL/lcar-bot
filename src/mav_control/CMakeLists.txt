# Required CMake Version
cmake_minimum_required(VERSION 2.8.3)

# Package Name
project(mav_control)

# Find catkin build system and specify needed packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
)

include(EnableCXX11)
include(MavrosMavlink)

# Declare the package
catkin_package()

# Specify locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare executables with source files
add_executable(simple_control src/simple_control.cpp)

#Specify libraries against which to link
target_link_libraries(simple_control
  ${catkin_LIBRARIES}
)
