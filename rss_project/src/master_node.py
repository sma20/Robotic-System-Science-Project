#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist #to send the command
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
#from project_g.msg import the_map
from tf.transformations import euler_from_quaternion#
import matplotlib.pyplot as plt
import numpy as np
from rss_project.srv import *
from math import atan2

poseX=0
poseY=0
posew=0
theta=0
ctrl_c=False

def publish_once_in_cmd(speed):
    """
    This is because publishing in topics sometimes fails the first time you publish.
    In continuous publishing systems, this is no big deal, but in systems that publish only
    once, it IS very important.
    """
    pub= rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size=4)
    while not ctrl_c:
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(speed)

            #rospy.loginfo("Cmd Published")
            break
        else:
            rospy.sleep(5)
def call_pose():
    """to call the odometry"""
    sub = rospy.Subscriber('/odom', Odometry, getPose)

def getPose(msg):
    """to get all the pose we need as global, could be done with a return, nut well"""
    global poseX
    global poseY
    global posew
    global theta
    poseX,poseY,posew=msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.w
    
    rot_q= msg.pose.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z,rot_q.w])#

def first_move():
    """we go forward 30cm than do a 360"""
    global poseX
    global poseY
    global posew
    #make the robot turn 360 then move straight ahead 
    call_pose()
    while poseX <0.3:
        speed.linear.x=0.15
        speed.angular.z=0.0
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        call_pose()
        print poseX
    #duration=0

    while abs(posew)>0.5 :
        #rospy.loginfo(duration)
        speed.linear.x=0.0 
        speed.angular.z=0.3
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        call_pose()
        #print posew
        #duration+=1

    while abs(posew)<0.98 :
        #rospy.loginfo(duration)
        speed.linear.x=0.0 
        speed.angular.z=0.3
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        call_pose()
        #print posew
        #duration+=1
    rospy.sleep(1)

	 
def turn():
    """we do a 360
    """
    global posew
    #make the robot turn 360 then move straight ahead 
    call_pose()
    while abs(posew)>0.5 :
        #rospy.loginfo(duration)
        speed.linear.x=0.0 
        speed.angular.z=0.3
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        call_pose()
        print posew
        #duration+=1

    while abs(posew)<0.98 :
        #rospy.loginfo(duration)
        speed.linear.x=0.0 
        speed.angular.z=0.3
        #pub.publish(speed)
        publish_once_in_cmd(speed)
        rospy.sleep(0.5)
        call_pose()
        #print posew
        #duration+=1

    rospy.sleep(1)


#Risk of infinite loop with the findfrontier sending the same trajectory again
def move_backward(posX,posY):
    """
    if a wall is too close from it it goes to its previous pose
    Actually we reach the coordinate frontally and not backward (so no need for a 360)
    """
    global poseX
    global poseY
    global theta
    global ctrl_c
    #make the robot turn 360 then move straight ahead 
    call_pose()
    angle_goal = atan2(posY-poseY,posX-poseX)#
    if ctrl_c==False:
        while abs(poseX-posX) >0.09 or abs(poseY-posY) >0.09:
            if abs(angle_goal-theta) >0.1 :#if the angle difference is too big
                if (angle_goal-theta)>0.1: #check the shortest way to turn toward our goal
                    #print("in big angle")
                    speed.linear.x=0.0 #correct the angle
                    speed.angular.z=0.3#
                    publish_once_in_cmd(speed)
                else:
                    speed.linear.x=0.0 #correct the angle
                    speed.angular.z= -0.3#
                    publish_once_in_cmd(speed)
            else:
                speed.linear.x=0.15 #not too fast not to make the error margin of the odometry larger
                speed.angular.z=0.0#
                publish_once_in_cmd(speed)
            rospy.sleep(0.5)
            call_pose()
            print poseX,poseY
    find_request_object.start_frontier=True #Now that the previous pos is reached we can restart the frontier service
    start=True #double security 

def deviate(posX,posY):
    """
    if a wall is too close from it it deviates

    """

    global poseX
    global poseY
    global theta
    global ctrl_c
    #make the robot turn 360 then move straight ahead 
    call_pose()
    angle_goal = atan2(posY-poseY,posX-poseX)
    while abs(poseX-posX) >0.09 or abs(poseY-posY) >0.09:
        speed.linear.x=-0.2 #correct the angle
        speed.angular.z=0.0#
        publish_once_in_cmd(speed)
        speed.linear.x=-0.2 #correct the angle
        speed.angular.z=0.0#
        publish_once_in_cmd(speed)
            


    find_request_object.start_frontier=True #Now that the previous pos is reached we can restart the frontier service
    start=True #double security 



def shutdownhook():
# works better than the rospy.is_shutdown(), to force ctrl_c to respond
    ctrl_c = True

    
if __name__ == '__main__':

    rospy.init_node('master_node') # Initialise THE ROS node
    print("matser_node started")
    speed=Twist()
    #rate=rospy.rate(5)
    rospy.on_shutdown(shutdownhook)

    #Start both services move_robot and findfrontier
    try:
        rospy.wait_for_service('/findfrontier')# Wait for the service to be running (with launch file)
        find_front = rospy.ServiceProxy('/findfrontier', find_frontier) # Create connection to service
        find_request_object= find_frontierRequest()
        rospy.loginfo("find_frontier Service Ready")
    except rospy.ServiceException, e:
        print ("Service call findfrontier failed: %s"%e)

    try:
        rospy.wait_for_service('/move_robot_to_goal')# Wait for the service to be running (with launch file)
        move_to_goal = rospy.ServiceProxy('/move_robot_to_goal', my_goal) # Create connection to service
        move_request_object = my_goalRequest() # Create an object of type EmptyRequest
    except rospy.ServiceException, e:
        print ("Service call move_robot_to_goal failed: %s"%e)

    #we go forward 30cm than do a 360
    first_move()
    previousX,previousY=0.3,0.1 #we set the starting point 


    """
    we set the variables to control the services 
    """
    
    complete=False
    find_request_object.start_frontier=True #2 services to start the findfrontier service when WE desire
    start=True

    while complete==False: #while the map isn't complete

        if start==True: #If we allows the find frontier to do it's work
            response_front=find_front(find_request_object) #we start find_front until map complete4
            complete= response_front.complete #we actualize the statut of the map
            find_request_object.start_frontier=False #We set the variables to avoid find_frontier to start whenever it wants
            start=False

        if response_front.complete== False: #if map isn't complete

            TrajectoryX=response_front.positionX #we collect the positions to reach
            TrajectoryY=response_front.positionY
            len_trajectory=len(TrajectoryX) #get how many goals there is in the trajectory

            for traj in range(len_trajectory): #for each point until the goal
                move_request_object.x_goal=TrajectoryX[traj]
                move_request_object.y_goal=TrajectoryY[traj]
                
                response_move = move_to_goal(move_request_object)
                
                if response_move.success==True and traj==(len_trajectory-1): #once the whole path is done, check point
                    print("success move")
                    previousX,previousY = TrajectoryX[traj],TrajectoryY[traj] #we save the position reached
                    print("previous pose")
                    print(previousX,previousY)
                    turn() #do a 360
                    find_request_object.start_frontier=True #we can now redo a findfrontier
                    start=True

                elif response_move.success==False: #if there is been a wall in front of the robot 
                    """
                    if traj-1>=0:

                        previousX,previousY=TrajectoryX[traj-1],TrajectoryY[traj-1]
                        print("previous pose")
                        print(previousX,previousY)
                        move_backward(previousX,previousY)
                    else:
                        move_backward(previousX,previousY)
                    """    
                    """
                    try:
                        rospy.wait_for_service('/move_robot_forcefield')# Wait for the service to be running (with launch file)
                        move_forcefield = rospy.ServiceProxy('/move_robot_forcefield', my_goal) # Create connection to service
                    except rospy.ServiceException, e:
                        print ("Service call forcefieldfailed: %s"%e)
                    
                    move_request_object.x_goal=TrajectoryX[len_trajectory-1]
                    move_request_object.y_goal=TrajectoryY[len_trajectory-1]
                    response_move = move_forcefield(move_request_object)
                    print("forcefield response:")
                    print(response_move)
                    """
                    turn()
                    find_request_object.start_frontier=True #we can now redo a findfrontier
                    start=True
                    break
        
    print("everything done, map completed")   
    #Rest of the code when map completed 
    
    try:
        rospy.wait_for_service('/check_map')# Wait for the service to be running (with launch file)
        find_goals = rospy.ServiceProxy('/check_map',find_goals) # Create connection to service
        find_goals_request_object= find_goalsRequest()
        rospy.loginfo("check map Service Ready")
        start=True
    except rospy.ServiceException, e:
        print ("Service call check map failed: %s"%e)

    find_goals_request_object.start_check=start
    response_goals= find_goals(find_goals_request_object)
    start=False
    find_goals_request_object.start_check=start

    TrajX=response_goals.positionsX
    TrajY=response_goals.positionsY
    len_trajectory=len(TrajX)
    if len_trajectory>0:

        for traj in range(len_trajectory): #for each point until the goal
            move_request_object.x_goal=TrajX[traj]
            move_request_object.y_goal=TrajY[traj]

            response_move = move_to_goal(move_request_object)
            if traj%30 ==0:
                turn()
            if response_move.success==True and traj==(len_trajectory-1): #once the whole path is done, check point
                print("success move")
            elif response_move.success==False: #once the whole path is done, check point
                continue