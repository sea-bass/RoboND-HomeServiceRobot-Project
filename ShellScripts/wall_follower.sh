#!/bin/sh
# Wall Follower Shell Script
# Sebastian Castro, 2019

# Find folder containing script
DIR=$(rospack find add_markers)/..

# Launch Gazebo world and TurtleBot
#xterm  -e  " export ROBOT_INITIAL_POSE=\"-x -3 -y 0\"; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$DIR/World/u_world.world " &
xterm  -e  " export ROBOT_INITIAL_POSE=\"-x -3 -y -1.5\"; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$DIR/World/orange_world.world " &
sleep 5
# Launch gmapping node (with tuned parameters)
xterm -e " rosrun gmapping slam_gmapping _linearUpdate:=0.1 _angularUpdate:=0.1 _particles:=100 _map_update_interval:=3 _xmin:=-10 _xmax:=10 _ymin:=-10 _ymax:=10" &
sleep 5
# Launch RViz for viewing navigation progress
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
# Launch the wall follower executable node
xterm  -e  " rosrun wall_follower wall_follower_node "