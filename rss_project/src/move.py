#! /usr/bin/env python

import rospy
from move_to_goal import movetogoal
from rss_project.srv import *

def my_callback(request):

    rospy.loginfo("move_robot Service called")
    response=my_goalResponse()
    print("my x and y coordinates")
    print(request.x_goal,request.y_goal)
    move = movetogoal(request.x_goal,request.y_goal)
    move.moverobot()
    rospy.loginfo("Finished move_robot service")
    response.success=True
    return response.success

rospy.init_node('move_to_goal')
my_service = rospy.Service('/move_robot_to_goal', my_goal , my_callback)
rospy.loginfo("MOVE_ROBOT Service Ready")


rospy.spin()
