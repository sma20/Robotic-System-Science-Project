#! /usr/bin/env python

import rospy
from move_to_goal import movetogoal
from ros_ass_world.srv import *

def my_callback(request):

    rospy.loginfo("Service called")
    response=my_goalResponse()
    move = movetogoal(request.x_goal,request.y_goal)
    move.moverobot()
    rospy.loginfo("Finished service")
    response.success=True
    return response.success

rospy.init_node('move_to_goal')
my_service = rospy.Service('/move_robot_to_goal', my_goal , my_callback)
rospy.loginfo("Service Ready")


rospy.spin()
