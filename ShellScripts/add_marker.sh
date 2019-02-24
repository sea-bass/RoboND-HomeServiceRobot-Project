#!/bin/sh
# Marker Spawning Shell Script
# Sebastian Castro, 2019

# Find folder containing script
DIR=$(rospack find add_markers)/..

# Launch Gazebo world and TurtleBot
xterm  -e  " export ROBOT_INITIAL_POSE=\"-x -3 -y -1.5\"; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$DIR/World/orange_world.world " &
sleep 5
# Launch AMCL node (with presaved map)
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$DIR/World/orange_world_map.yaml" &
sleep 5
# Launch RViz for viewing navigation progress
xterm  -e  " rosrun rviz rviz -d $DIR/RvizConfigs/home_service_config.rviz " &
sleep 5
# Start the marker spawning node
xterm -e " rosrun add_markers add_markers_node "