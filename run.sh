#!/bin/bash
source ~/03_kss/devel/setup.bash

gnome-terminal --window-with-profile=kss -e 'roscore'
gnome-terminal --window-with-profile=kss -e 'rosrun http_bridge http_server.py'
gnome-terminal --window-with-profile=kss -e 'rosrun http_bridge http_client.py'
gnome-terminal --window-with-profile=kss -e 'rosrun http_bridge com_client.py'
gnome-terminal --window-with-profile=kss -e 'rosrun automation_interface automation_interface.py'
gnome-terminal --window-with-profile=kss -e 'rosrun power_management power_management.py'
gnome-terminal --window-with-profile=kss -e 'rosrun tracker tracker.py'
gnome-terminal --window-with-profile=kss -e 'rosrun ftp_server ftp_server.py'
gnome-terminal --window-with-profile=kss -e 'rosrun meteo meteo.py'

sleep 5s
gnome-terminal --window-with-profile=kss -e 'rosrun core core.py'

# DEBUG STUFF
#rosservice call /send_mission  "/home/oze/mission_region0_route0_variant0.waypoint"
