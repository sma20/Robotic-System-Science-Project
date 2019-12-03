#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
#from project_g.msg import the_map
import matplotlib.pyplot as plt
import numpy as np
from rss_project.srv import *


rospy.init_node('master_node') # Initialise THE ROS node
print("matser_node started")
try:
    rospy.wait_for_service('/findfrontier')# Wait for the service to be running (with launch file)
    find_front = rospy.ServiceProxy('/findfrontier', find_frontier) # Create connection to service
except rospy.ServiceException, e:
    print ("Service call findfrontier failed: %s"%e)

try:
    rospy.wait_for_service('/move_robot_to_goal')# Wait for the service to be running (with launch file)
    move_to_goal = rospy.ServiceProxy('/move_robot_to_goal', my_goal) # Create connection to service
    move_request_object = my_goalRequest() # Create an object of type EmptyRequest
except rospy.ServiceException, e:
    print ("Service call move_robot_to_goal failed: %s"%e)

complete=False

while complete==False:
    response_front=find_front() #we start find_front until map complete
    complete= response_front.complete

    if response_front.complete== False: #if map isn't complete
        len_trajectory=len(response_front.positionX) #get how many goals there is in the trajectory

        for traj in range(len_trajectory): #for each point until the goal
            move_request_object.x_goal=response_front.positionX[traj]
            move_request_object.y_goal=response_front.positionY[traj]

            response_move = move_to_goal(move_request_object)
            if response_move.success==True and traj==(len_trajectory-1): #once the whole path is done, check point
                print("success")

      
print("everything done, map completed")   
#Rest of the code when map completed missing the 360 turn now
 