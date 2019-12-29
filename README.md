
Package for the project: "rss_project"
To use it:
  1. Copy the rss_project folder in your catkin workspace
  2. In a terminal: roscd; cd ..;
  3. catkin_make --only-pkg-with-deps rss_project
  4. add permission to all the py files, using "chmod +x [file.py]" command
 
 Roslaunch rss_project srv_server_moverobot.launch
 Rosrun gmapping scan:= hokuyo_laser or scan
 
 image processing :
 
 roslaunch marker_map launch.launch
    load the rightful data set on find 2d object
    click on "update object"
    add the Visual MArker in Rviz
