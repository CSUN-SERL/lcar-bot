cmake_minimum_required(VERSION 2.8.3)
project(rqt_gcs)

find_package(catkin REQUIRED COMPONENTS
            query_msgs
            geometry_msgs
            mavros
            mavros_msgs
            roscpp
            rqt_gui
            rqt_gui_cpp
            roslib
            sensor_msgs
            std_msgs
            tf
            cv_bridge
            image_transport
)
#stupid qt5
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Svg REQUIRED)
#find_package(Qt5Widget REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

#stupid qt5
include_directories(${Qt5Core_INCLUDE_DIRS} 
                    ${Qt5Gui_INCLUDE_DIRS} 
                    ${Qt5Svg_INCLUDE_DIRS}
#                    $Qt5Widget_INCLUDE_DIRS
)
include(EnableCXX11)
include(MavrosMavlink)

set(rqt_gcs_SRCS
  src/rqt_gcs/simple_gcs.cpp
  src/rqt_gcs/simple_control.cpp
  src/FIS/qfi_PFD.cpp
  src/FIW/LayoutSquare.cpp
  src/FIW/WidgetPFD.cpp
  src/rqt_image_view/ratio_layouted_frame.cpp
)

set(rqt_gcs_HDRS
  include/rqt_gcs/simple_gcs.h
  include/FIS/qfi_PFD.h
  include/FIW/LayoutSquare.h
  include/FIW/WidgetPFD.h
  include/rqt_image_view/ratio_layouted_frame.h
)

set(rqt_gcs_UIS
  resources/gcs.ui
  resources/AccessPointsMenu.ui
  resources/AccessPointStats.ui
  resources/MissionSelect.ui
  resources/MissionCancel.ui
  resources/MissionProgress.ui
  resources/UavStat.ui
  resources/UavQuestion.ui
  resources/QuadStats.ui
  resources/WidgetPFD.ui
  resources/PFDWidget_custom.ui
  resources/WidgetMain.ui
  resources/UAVCondition.ui
  resources/ImageView.ui
  resources/MissionConfirm.ui
  resources/PictureMsg.ui
)

SET(rqt_gcs_RCCS resources/qfi.qrc)

set(rqt_gcs_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ${rqt_gcs_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs sensor_msgs
)

include_directories(include
	${Qt5_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_definitions(${Qt5_DEFINITIONS})


qt5_add_resources(rqt_gcs_RCC_SRCS ${rqt_gcs_RCCS})
qt5_wrap_cpp(rqt_gcs_MOCS ${rqt_gcs_HDRS})
qt5_wrap_ui(rqt_gcs_UIS_H ${rqt_gcs_UIS})

include_directories(${rqt_gcs_INCLUDE_DIRECTORIES}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

# Declare executables with source files
add_executable(simple_control src/rqt_gcs/simple_control.cpp)
add_executable(gps_publisher src/rqt_gcs/gps_publisher.cpp)
add_executable(gps_demo src/rqt_gcs/gps_demo.cpp)
add_executable(connection_manager src/rqt_gcs/connection_manager.cpp)
add_executable(safe_mocap src/rqt_gcs/safe_mocap.cpp)

add_dependencies(simple_control ${catkin_EXPORTED_TARGETS})
add_dependencies(gps_demo ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}
            ${rqt_gcs_SRCS}
            ${rqt_gcs_MOCS}
            ${rqt_gcs_UIS_H}
            ${rqt_gcs_RCC_SRCS})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#Specify libraries against which to link
target_link_libraries(simple_control ${catkin_LIBRARIES})
target_link_libraries(gps_publisher ${catkin_LIBRARIES})
target_link_libraries(gps_demo ${catkin_LIBRARIES})
target_link_libraries(connection_manager ${catkin_LIBRARIES})
target_link_libraries(safe_mocap ${catkin_LIBRARIES})
target_link_libraries(  ${PROJECT_NAME}
                        ${catkin_LIBRARIES}
                        ${Qt5Core_LIBRARIES}
                        ${Qt5Gui_LIBRARIES}
                        ${Qt5Svg_LIBRARIES}
#                        ${Qt}
                        ${OpenCV_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_gcs
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_gcs
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
