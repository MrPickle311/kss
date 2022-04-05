cmake_minimum_required(VERSION 3.0.2)
project(http_bridge)

find_package(catkin REQUIRED COMPONENTS
        rospy
        roslint
        )

catkin_package(
        CATKIN_DEPENDS
)

catkin_python_setup()

roslint_python()

catkin_install_python(PROGRAMS
        scripts/http_client.py
        scripts/http_server.py
        scripts/com_client.py
        DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
        )