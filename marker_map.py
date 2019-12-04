#!/usr/bin/env python


import rospy
import roslib
import sys 
from std_msgs.msg import String, Int64, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
roslib.load_manifest('image_proc')
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, Pose,Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler



x = 0.0
y = 0.0
theta = 0.0

sym_x = 0.0
sym_y = 0.0
depth = 0.0
angle = 0.0

pix = 0
angle = 0
value = 0



#############################################################################################################################################################################

def find_symbol(arg):

  global depth

  depth = arg.ranges[value]
  #print(depth)
  

#############################################################################################################################################################################


def find_position(arg):

    global x
    global y
    global theta

    x = round(arg.pose.pose.position.x,2)
    y = round(arg.pose.pose.position.y,2)
    #print("x:",x)
    #print("y:",y)


    rot_q = arg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


#############################################################################################################################################################################


def find_pixel(var):

  global pix
  global angle
  global value

  pix = var.data
  #print(pix)
  angle = (pix*0.0890625) + 61.5 - 90
  #print(angle)
  value = 720/2
  #print(value)


#############################################################################################################################################################################
  

def init_marker(index,x,y,symbole):

    marker_object = Marker()
    marker_object.header.frame_id = "/map"
    marker_object.header.stamp    = rospy.get_rostime()
    marker_object.ns = "haro"
    marker_object.id = index
    marker_object.type = Marker.SPHERE
    marker_object.action = Marker.ADD
    
    my_point = Point()
    my_point.x = x
    my_point.y = y
    my_point.z = 0
    marker_object.pose.position = my_point
    
    marker_object.pose.orientation.x = 0
    marker_object.pose.orientation.y = 0
    marker_object.pose.orientation.z = 0.0
    marker_object.pose.orientation.w = 1.0
    marker_object.scale.x = 0.6
    marker_object.scale.y = 0.6
    marker_object.scale.z = 0.6

    if symbole == 'alive_worker':

        marker_object.color.r = 0.0
        marker_object.color.g = 1.0
        marker_object.color.b = 0.0
        # This has to be, otherwise it will be transparent
        marker_object.color.a = 1.0
    
    if symbole == 'dead_worker':
        marker_object.color.r = 1.0
        marker_object.color.g = 0.0
        marker_object.color.b = 0.0
        # This has to be, otherwise it will be transparent
        marker_object.color.a = 1.0
        
    # If we want it for ever, 0, otherwise seconds before desapearing
    marker_object.lifetime = rospy.Duration(0)
    
    return marker_object




#############################################################################################################################################################################


# craeting the node
rospy.init_node("mapping_corpses")
# creating subscriber to odometry and laserscan
pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=1)


all_marker = []
x_marker = []
y_marker = []
symbol_marker = []

while not rospy.is_shutdown():

  rospy.sleep(1)    
  
  sub_pixel = rospy.Subscriber("/objects", Float32MultiArray, find_pixel)
  
  if pix != 0 :

    sub_odom = rospy.Subscriber("/odom", Odometry, find_position)
    sub_depth = rospy.Subscriber("/scan", LaserScan, find_symbol)


    sym_x = x + depth*np.cos(theta + angle)
    sym_y = y + depth*np.sin(theta + angle)
    
 
    
    """
    print" the object is at :", depth, "m"
    print" "
    print" x object : ", sym_x
    print" "
    print" y object : ", sym_y
    print" "
    """
    # Initialization of the first symbole founded
    if len(x_marker) < 1 and sym_x != 0 and depth != 0 :

      x_marker.append(round(sym_x,3))
      y_marker.append(round(sym_y,3))
      symbol_marker.append("alive_worker")
    
    else:
      pass
    
    # checking if the new symbol published was found before
    # if not then add it to the array
    for i in range(len(x_marker)):
      if x_marker[i] < sym_x - 0.5 and x_marker > sym_x + 0.5 and y_marker[i] < sym_y - 0.5 and y_marker > sym_y + 0.5:
          x_marker.append(sym_x)
          y_marker.append(sym_y)
          symbol_marker.append("alive_worker")
        
      else:
        pass
    
    # concatenated every x , y position and the symbole meaning
    print("    x       y    symbole")
    symbol_array = np.concatenate((x_marker,y_marker,symbol_marker))
    print(symbol_array)
    print" "

    # publish the list of the marker on Rviz
    for i in range(len(x_marker)):

      mark = init_marker(i,x_marker[i],y_marker[i],symbol_marker[i])
      pub_marker.publish(mark)

    




  
