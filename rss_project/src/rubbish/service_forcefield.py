#! /usr/bin/env python

import rospy
from force_field_with_service import moveforcefield
from rss_project.srv import *

def my_callback(request):

    rospy.loginfo("Service forcefield called")
    response=my_goalResponse()
    move =  moveforcefield(request.x_goal,request.y_goal)
    move.moverobot()
    success=move.moverobot()
    response.success=success
    rospy.loginfo("Finished forcefield service")
    print(response.success)
    return response.success

rospy.init_node('move_to_goal_forcefield')
my_service = rospy.Service('/move_robot_forcefield', my_goal , my_callback)
rospy.loginfo("Service forcefield Ready")


rospy.spin()
