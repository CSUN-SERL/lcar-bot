cmake_minimum_required(VERSION 2.8.3)
project(rqt_gcs)

find_package(catkin REQUIRED COMPONENTS
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
)

find_package(Qt4 COMPONENTS QtCore QtGui QtSvg REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include(${QT_USE_FILE})
include(EnableCXX11)
include(MavrosMavlink)

set(rqt_gcs_SRCS
  src/rqt_gcs/gcs.cpp
  src/rqt_gcs/simple_control.cpp 

  src/FIS/qfi_ADI.cpp
  src/FIS/qfi_ALT.cpp
  src/FIS/qfi_ASI.cpp
  src/FIS/qfi_HSI.cpp
  src/FIS/qfi_NAV.cpp
  src/FIS/qfi_PFD.cpp
  src/FIS/qfi_TC.cpp
  src/FIS/qfi_VSI.cpp

  src/FIW/LayoutSquare.cpp
  src/FIW/MainWindow.cpp
  src/FIW/WidgetADI.cpp
  src/FIW/WidgetALT.cpp
  src/FIW/WidgetASI.cpp
  src/FIW/WidgetHSI.cpp
  src/FIW/WidgetNAV.cpp
  src/FIW/WidgetPFD.cpp
  src/FIW/WidgetSix.cpp
  src/FIW/WidgetTC.cpp
  src/FIW/WidgetVSI.cpp
)

set(rqt_gcs_HDRS
  include/rqt_gcs/gcs.h
  include/rqt_gcs/simple_control.h

  include/FIS/qfi_ADI.h
  include/FIS/qfi_ALT.h
  include/FIS/qfi_ASI.h
  include/FIS/qfi_HSI.h 
  include/FIS/qfi_NAV.h
  include/FIS/qfi_PFD.h
  include/FIS/qfi_TC.h
  include/FIS/qfi_VSI.h

  include/FIW/LayoutSquare.h
  include/FIW/MainWindow.h
  include/FIW/WidgetADI.h
  include/FIW/WidgetALT.h
  include/FIW/WidgetASI.h
  include/FIW/WidgetHSI.h
  include/FIW/WidgetNAV.h
  include/FIW/WidgetPFD.h
  include/FIW/WidgetSix.h
  include/FIW/WidgetTC.h
  include/FIW/WidgetVSI.h
)

set(rqt_gcs_UIS
  resources/gcs.ui
  resources/MissionSelect.ui
  resources/MissionCancel.ui
  resources/MissionProgress.ui
  resources/UavStat.ui
  resources/UavQuestion.ui
  resources/QuadStats.ui
  resources/MainWindow.ui
  resources/WidgetNAV.ui
  resources/WidgetPFD.ui
  resources/WidgetADI.ui
  resources/WidgetALT.ui
  resources/WidgetASI.ui
  resources/WidgetHSI.ui
  resources/WidgetTC.ui
  resources/WidgetVSI.ui
  resources/WidgetSix.ui
)

SET(rqt_gcs_RCCS resources/qfi.qrc)

set(rqt_gcs_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${rqt_gcs_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp
)

include_directories(${Qt4_INCLUDE_DIRS})

add_definitions(${Qt4_DEFINITIONS})


catkin_python_setup()

qt4_add_resources(rqt_gcs_RCC_SRCS ${rqt_gcs_RCCS})
qt4_wrap_cpp(rqt_gcs_MOCS ${rqt_gcs_HDRS})
qt4_wrap_ui(rqt_gcs_UIS_H ${rqt_gcs_UIS})

include_directories(${rqt_gcs_INCLUDE_DIRECTORIES}
										${Boost_INCLUDE_DIRS}
										${catkin_INCLUDE_DIRS})

# Declare executables with source files
add_executable(simple_control src/rqt_gcs/simple_control.cpp)
add_executable(gps_publisher src/rqt_gcs/gps_publisher.cpp)


add_library(${PROJECT_NAME}
						${rqt_gcs_SRCS} 
						${rqt_gcs_MOCS}
						${rqt_gcs_UIS_H}
                                                ${rqt_gcs_RCC_SRCS})

#Specify libraries against which to link
target_link_libraries(simple_control ${catkin_LIBRARIES})
target_link_libraries(gps_publisher ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}
											${catkin_LIBRARIES}
											${QT_QTCORE_LIBRARY}
											${QT_QTGUI_LIBRARY}

${QT_QTSVG_LIBRARY})

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
