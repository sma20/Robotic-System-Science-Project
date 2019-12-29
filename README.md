# Robotic-System-Science-Project


RSS package:

Navigation

Launch:
All the navigation is started from one launch file. 
Except the gmapping node (as the scan name "hokuy_laser" or "scan" depended if we were on gazebo or on the turtlebot)

SRC:
master_node.py: is the node managing all the services related to navigation

map.py: is the wavefront service searching a trajectory to an unknown part of the map.

move.py: start the move service, send data and return state of the service
move_to_goal.py: the move service, move from the robot pose to the coordinates sent to it

Check_final_map.py: start the final service, send data and return state of the service
work_final_map.py: decide a trajectory to travel throught the whole map

SRV:
find_frontier.srv: related to map.py (wait for signal to start, send back a trajectory)
my_goal.srv: related to move.py (wait for coordinates, send back state of the service (success, fail))
find_goals.srv: related to check_final_map.py (wait for signal to start, send back a trajectory)

Package for the project: "rss_project"
To use it:
  1. Copy the rss_project folder in your catkin workspace
  2. In a terminal: roscd; cd ..;
  3. catkin_make --only-pkg-with-deps rss_project
  4. add permission to all the py files, using "chmod +x [file.py]" command
 
 Roslaunch rss_project 
 
 image processing :
 
 roslaunch marker_map launch.launch
    load the rightful data set on find 2d object
    click on "update object"
    add the Visual MArker in Rviz
