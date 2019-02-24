#!/bin/sh
# SLAM Test Shell Script
# Sebastian Castro, 2019

# Find folder containing script
DIR=$(rospack find add_markers)/..

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$DIR/World/u_world.world " &
sleep 5
xterm -e " rosrun gmapping slam_gmapping " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch "