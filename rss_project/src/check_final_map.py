#! /usr/bin/env python

import rospy
from work_final_map import get_final_map
from rss_project.srv import *

def service_callback(request): 


	rospy.loginfo("check_map Service called")
	response=find_goalsResponse()
	start_check=request.start_check

	check= get_final_map(start_check)
	positionsX, positionsY=check.my_return()
	response.positionsX, response.positionsY= positionsX,positionsY
	rospy.loginfo("Finished ceck map service")
	print(response.positionsX, response.positionsY)
	print(len(response.positionsX))
	request.start_check=False
	return response.positionsX, response.positionsY


rospy.init_node('check_map')
my_service = rospy.Service('/check_map', find_goals , service_callback)
rospy.loginfo("check map Service Ready")


rospy.spin()
