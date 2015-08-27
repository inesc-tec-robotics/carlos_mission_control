cmake_minimum_required(VERSION 2.8.3)
project(mission_control)
#set(CMAKE_BUILD_TYPE debug)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  std_srvs
  actionlib
  actionlib_msgs
  message_generation
  mission_ctrl_msgs
  XMLOperations
  MathOp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
             system
             filesystem
)

set(QT_QMAKE_EXECUTABLE /usr/bin/qmake-qt4) #Force the system to use QT4
find_package(Qt4 REQUIRED)

# enable/disable some Qt features
SET(QT_USE_QTGUI true)
SET(QT_USE_QTCORE true)


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

################################################
## Declare ROS messages, services and actions ##
################################################
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
   ParamInt.msg
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
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  roscpp
  std_msgs
  std_srvs
  roslib
  message_runtime
  mission_ctrl_msgs
  XMLOperations
  MathOp
  DEPENDS Boost
  )

  
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)


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

add_dependencies(mission_controller_node mission_control_generate_messages_cpp)
add_dependencies(mission_controller_gui mission_control_generate_messages_cpp)


