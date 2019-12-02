#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
#from project_g.msg import the_map
import matplotlib.pyplot as plt
import numpy as np
from rss_project.srv import *


rospy.init_node('master_node') # Initialise THE ROS node

rospy.wait_for_service('/findfrontier')# Wait for the service to be running (with launch file)
find_front = rospy.ServiceProxy('/findfrontier', find_frontier) # Create connection to service



rospy.wait_for_service('/move_robot_to_goal')# Wait for the service to be running (with launch file)
move_to_goal = rospy.ServiceProxy('/move_robot_to_goal', my_goal) # Create connection to service
move_request_object = my_goalRequest() # Create an object of type EmptyRequest
#move_request_object.x_goal=2
#move_request_object.y_goal=2

try:
    response_front=find_front()

    if response.complete= False:

        try:
            response = move_to_goal(move_request_object)
            if response.success==True:
                    print("success")
            else: 
                print("damn it")
        except rospy.ServiceException, e:
            print ("Service call failed: %s"%e)

    else:
        #Rest of the code when map completed
 except rospy.ServiceException, e:
     print ("Service call failed: %s"%e)