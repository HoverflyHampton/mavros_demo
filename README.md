# MavRos Demo

Just a simple waypoint following demo. 

# Building

Clone this repo to your catkin workspace's src directory, then build with catkin build and resource your devel/setup.bash

# Usage

With either a craft ready to connect or a simulator running, call `roslaunch mavros_demo apm.launch`. Then, from the scripts directory (at least until I get around to fixing the file pathing) call `rosrun mavros_demo WaypintDemo.py`. 

# Modifications

To change the flight path, edit the waypoints_test.data file. 

# Further steps

This is just a quick demo I set up. It might be worth modifying the WaypointDemo code to take user input, fly in different modes, or create a new node that publishes on rc_input to fake manual flight.