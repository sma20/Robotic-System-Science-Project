
Navigation :

  1. Copy the rss_project folder in your catkin workspace
  2. In a terminal: roscd; cd ..;
  3. catkin_make --only-pkg-with-deps rss_project
  4. add permission to all the py files, using "chmod +x [file.py]" command
 
 Roslaunch rss_project srv_server_moverobot.launch
 Rosrun gmapping scan:= hokuyo_laser or scan
 
Find Marker :
 
    1. roslaunch marker_map launch.launch
    2. load the rightful data set from the folder "image" on find 2d object
    3. click on "update object"
    4. add the Visual Marker on Rviz
