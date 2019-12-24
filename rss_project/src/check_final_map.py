#! /usr/bin/env python

import rospy
from work_final_map import get_final_map
from rss_project.srv import *

#here is called the service to recheck the whole map once map deemed complete 

def service_callback(request): 
	rospy.loginfo("check_map Service called")
	response=find_goalsResponse()
	start_check=request.start_check #check if the request is True or false to allow service to start

	check= get_final_map(start_check) #send this order to the function
	
	positionsX, positionsY=check.my_return() #get the full trajectory coordinates
	response.positionsX, response.positionsY= positionsX,positionsY
	rospy.loginfo("Finished ceck map service")
	print(response.positionsX, response.positionsY) #check
	print(len(response.positionsX)) #check
	request.start_check=False
	return response.positionsX, response.positionsY


rospy.init_node('check_map')
my_service = rospy.Service('/check_map', find_goals , service_callback)
rospy.loginfo("check map Service Ready")


rospy.spin()
