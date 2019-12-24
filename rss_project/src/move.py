#! /usr/bin/env python

import rospy
from move_to_goal import movetogoal
from rss_project.srv import *

#The call service

def my_callback(request):

    rospy.loginfo("move_robot Service called")
    response=my_goalResponse() 

    print("my x and y coordinates")
    print(request.x_goal,request.y_goal) #to check everything is alright

    move = movetogoal(request.x_goal,request.y_goal) #the coordinate are sent to the service
    success=move.moverobot() #the move starts

    response.success=success 
    rospy.loginfo("Finished move_robot service")
    print(response.success)
    return response.success #return if the service resulted in a success or fail to reach its coordinates
    

rospy.init_node('move_to_goal')
my_service = rospy.Service('/move_robot_to_goal', my_goal , my_callback)
rospy.loginfo("MOVE_ROBOT Service Ready")


rospy.spin()
