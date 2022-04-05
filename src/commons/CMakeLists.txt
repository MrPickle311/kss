cmake_minimum_required(VERSION 3.16)
project(commons)

#set(QT_VERSION 6.2.3)

#set(CMAKE_PROJECT_INCLUDE_BEFORE /home/$ENV{USER}/Qt/Tools/QtCreator/share/qtcreator/package-manager/auto-setup.cmake)
#set(QT_QMAKE_EXECUTABLE /home/$ENV{USER}/Qt/${QT_VERSION}/gcc_64/bin/qmake)

#list(APPEND CMAKE_PREFIX_PATH /home/$ENV{USER}/Qt/${QT_VERSION}/gcc_64)

#add_compile_definitions(PROJECT_FOLDER_NAME="${CMAKE_CURRENT_SOURCE_DIR}")
#add_compile_options(-Wall -Wextra)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)

find_package(catkin REQUIRED
        actionlib
        roscpp
        rospy
        )

#find_package(QT NAMES Qt6 COMPONENTS Core Network SerialBus SerialPort)
#find_package(Qt6 COMPONENTS Core Network SerialBus SerialPort REQUIRED)

#include_directories(${CMAKE_CURRENT_SOURCE_DIR})

catkin_python_setup()

add_message_files(
        FILES
        TrackerConnectionSignal.msg
        Signal.msg
        HttpExposedResourcesContent.msg
        HttpServerSimpleEvent.msg
        MissionsUploadedEvent.msg
        ReceivedImagesFromFtpServerEvent.msg
        SendImagesToServerEvent.msg
        StationState.msg
        StationError.msg
        RotorAbsolutePosition.msg
        RotorReset.msg
        RotorRelativePosition.msg
        DroneState.msg
        DroneEvent.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        TryDroneStart.srv
        SwitchModulePower.srv
        AutomationStates.srv
        TryUploadMission.srv
        TryLand.srv
        DroneSimpleTask.srv
        DroneReturn.srv
)

## Generate actions in the 'action' folder
add_action_files(
        DIRECTORY action
        FILES
        AutomationProcess.action
        TrackerProcess.action
        MeteoProcess.action
        PowerManagementProcess.action
        DroneCommunicationProcess.action
        HttpServerProcess.action
        HttpClientProcess.action
        FTPServerProcess.action
        RotorProcess.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        actionlib_msgs
)

catkin_package(
        #        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp actionlib rospy message_generation std_msgs message_runtime
        #        DEPENDS Qt6Core Qt6SerialBus
)

include_directories(
        #        include
        ${catkin_INCLUDE_DIRS}
        #        ${Qt6Core_INCLUDE_DIRS}
)

#set(SrcDir src/${PROJECT_NAME})
#set(IncDir include/${PROJECT_NAME})

#set(ModbusSrcDir ${SrcDir}/modbus)
#set(ModbusIncDir ${IncDir}/modbus)

#set(NodeUtilitiesIncDir ${IncDir}/node_utilities)
#set(NodeUtilitiesSrcDir ${SrcDir}/node_utilities)

#set(ActionServerIncDir ${IncDir}/action_server)

#set(Sources
#        ${NodeUtilitiesSrcDir}/sigwatch.cpp
#        ${NodeUtilitiesSrcDir}/LinuxSignalHandler.cpp
#        ${NodeUtilitiesSrcDir}/ROSQtNode.cpp
#        ${ModbusSrcDir}/ModbusReader.cpp
#        ${ModbusSrcDir}/ModbusScanner.cpp
#        ${ModbusSrcDir}/ModbusWriter.cpp
#        ${ModbusSrcDir}/ModbusBase.cpp
#        ${ModbusSrcDir}/ModbusDevice.cpp)

#set(Includes
#        ${ActionServerIncDir}/ActionServer.hpp
#        ${NodeUtilitiesIncDir}/sigwatch.h
#        ${NodeUtilitiesIncDir}/LinuxSignalHandler.hpp
#        ${NodeUtilitiesIncDir}/ROSQtNode.hpp
#        ${ModbusIncDir}/ModbusReader.hpp
#        ${ModbusIncDir}/ModbusScanner.hpp
#        ${ModbusIncDir}/ModbusWriter.hpp
#        ${ModbusIncDir}/ModbusBase.hpp
#        ${ModbusIncDir}/ModbusDevice.hpp)

#add_library(${PROJECT_NAME} STATIC
#        ${Sources}
#        ${Includes}
#        )

#set(QT_LIBRARIES
#        Qt6::Core
#        Qt6::Network
#        Qt6::SerialBus
#        Qt6::SerialPort
#        )

#target_link_libraries(${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${QT_LIBRARIES}
#        modbuspp
#        )

#catkin_install_python(PROGRAMS
#        scripts/automation_interface.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

#install(
#        TARGETS ${PROJECT_NAME}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

#install(
#        DIRECTORY include/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)