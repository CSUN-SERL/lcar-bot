# Required CMake Version
cmake_minimum_required(VERSION 2.8.3)

# Package Name
project(mav_control)

# Find catkin build system and specify needed packages
find_package(catkin REQUIRED COMPONENTS roscpp mavros)

# Declare the package
catkin_package()

# Specify locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare executables with source files
# Example: add_executable(control_node src/control_node.cpp)

#Specify libraries against which to link
# Example: target_link_libraries(control_node ${catkin_LIBRARIES})
