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
    pub= rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size=4)
    """
    This is because publishing in topics sometimes fails the first time you publish.
    In continuous publishing systems, this is no big deal, but in systems that publish only
    once, it IS very important.
    """
    while not ctrl_c:
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(speed)

            #rospy.loginfo("Cmd Published")
            break
        else:
            rospy.sleep(5)
def call_pose():
    sub = rospy.Subscriber('/odom', Odometry, getPose)

def getPose(msg):
    global poseX
    global poseY
    global posew
    global theta
    poseX,poseY,posew=msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.w
    
    rot_q= msg.pose.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z,rot_q.w])#

def first_move():

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
        print posew
        #duration+=1
    rospy.sleep(1)

	 
def turn():

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
        print posew
        #duration+=1

    rospy.sleep(1)



def move_backward(posX,posY):

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
def shutdownhook():
# works better than the rospy.is_shutdown()
    ctrl_c = True
if __name__ == '__main__':

    rospy.init_node('master_node') # Initialise THE ROS node
    print("matser_node started")
    speed=Twist()
    #rate=rospy.rate(5)
    rospy.on_shutdown(shutdownhook)

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

    first_move()

    complete=False
    find_request_object.start_frontier=True
    start=True
    while complete==False:

        if start==True:
            response_front=find_front(find_request_object) #we start find_front until map complete4
            complete= response_front.complete
            find_request_object.start_frontier=False
            start=False

        if response_front.complete== False: #if map isn't complete
            TrajectoryX=response_front.positionX
            TrajectoryY=response_front.positionY
            len_trajectory=len(TrajectoryX) #get how many goals there is in the trajectory

            for traj in range(len_trajectory): #for each point until the goal
                move_request_object.x_goal=TrajectoryX[traj]
                move_request_object.y_goal=TrajectoryY[traj]

                response_move = move_to_goal(move_request_object)
                if response_move.success==True and traj==(len_trajectory-1): #once the whole path is done, check point
                    print("success move")
                    turn() #do a 360
                    find_request_object.start_frontier=True
                    start=True
                if response_move.success==False:
                    move_backward(TrajectoryX[traj-1],TrajectoryY[traj-1])
                    find_request_object.start_frontier=True
                    start=True
                    break
        
    print("everything done, map completed")   
    #Rest of the code when map completed missing the 360 turn now
 