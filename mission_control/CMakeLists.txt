cmake_minimum_required(VERSION 2.8.3)
project(mission_control)
#set(CMAKE_BUILD_TYPE debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
  mission_ctrl_msgs
  XMLOperations
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS
             system
             filesystem
)

set(QT_QMAKE_EXECUTABLE /usr/bin/qmake-qt4) #Force the system to use QT4
find_package(Qt4 REQUIRED)

# enable/disable some Qt features
#set(QT_USE_QTOPENGL TRUE)
#SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTGUI true)
SET(QT_USE_QTCORE true)
#SET(QT_USE_QDBUS true)

include(${QT_USE_FILE})

set(qt_srcs
src/gui_node.cpp
src/GUI/mainwindow.cpp
src/GUI/loadMission.cpp
src/GUI/ros_interface.cpp
src/GUI/addTask.cpp
)

set(qt_hdrs
src/GUI/mainwindow.h
src/GUI/loadMission.hpp
src/GUI/ros_interface.hpp
src/GUI/addTask.hpp
)

qt4_automoc(${qt_srcs})

QT4_WRAP_UI(uis_h
            src/GUI/mainwindow.ui
            src/GUI/loadMission.ui
            src/GUI/addTask.ui
            )

QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   DeviceState.msg
   HardwareStates.msg
   uiMessage.msg
   Function.msg
   Progress.msg
   NavGoal.msg
   StudPattern.msg
   TaskData.msg
   MissionData.msg
   ParamDouble.msg
   ParamString.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Trigger.srv
   getTaskList.srv
   getMissionList.srv
   getMissionData.srv
   getTaskData.srv
   setTaskData.srv
   setMissionData.srv
   getTaskParams.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  roscpp
  std_msgs
  roslib
  message_runtime
  mission_ctrl_msgs
  XMLOperations
  DEPENDS Boost
  )

  
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

## Declare a cpp library
# add_library(mission_control
#   src/${PROJECT_NAME}/mission_control.cpp
# )

## Declare a cpp executable
# add_executable(mission_control_node src/mission_control_node.cpp)

add_executable(mission_controller_node
               src/mission_controller_node.cpp
               src/mission_handler.cpp
               src/UI_API.cpp
               src/hw_state_machine.cpp
               src/system_engine.cpp
               src/execution_engine.cpp
               src/instruction_engine.cpp
               src/action_interface.cpp
               )

add_executable(mission_controller_gui
               ${qt_srcs}
               ${qt_moc_srcs}
               ${uis_h}
               )
               
target_link_libraries(mission_controller_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )

target_link_libraries(mission_controller_gui
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${QT_LIBRARIES}
                      )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(mission_controller_node mission_control_generate_messages_cpp)
 add_dependencies(mission_controller_gui mission_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(mission_control_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mission_control mission_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mission_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
