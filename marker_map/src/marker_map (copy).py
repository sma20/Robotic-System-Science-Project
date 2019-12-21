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
angle_value = 0.0

pix = 0
angle = 0
value = 0
symbole = ""



#############################################################################################################################################################################

def find_symbol(arg):

    global depth

    depth = arg.ranges[value]





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
    #print"this is Theta :",theta


#############################################################################################################################################################################


def find_pixel(var):

    global symbole

    global pix
    global angle
    global value


    #makes sure that there is data
    if len(var.data) == 0 :
        pass
    else:
        pix = var.data[9]

        angle = -1*((65*(pix/600)) - 28.5)
        #angle = 28.5 - (65*(pix/600))


        value = int(2.86406132*(angle - 28.5 + 163.5))

        #angle_scan = value * 0.351562498 -
        #print"angle scan", angle_scan
        #print"angle vision", angle + 28.5 + 98.94 + 7.558594523

        if var.data[0] == 49.0:
            symbole = "Biohazard"

        elif var.data[0] == 50.0:
            symbole = "Danger"

        elif var.data[0] == 51.0:
            symbole = "Fire"

        elif var.data[0] == 52.0:
            symbole = "Radioactive"

        elif var.data[0] == 53.0:
            symbole = "no smoking"

        elif var.data[0] == 54.0:
            symbole = "toxic"

        elif var.data[0] == 55.0 or var.data[0] == 56.0:
            pass

        #print"\n the symbole found is :", symbole

        rviz_marker(value,angle,symbole)

#############################################################################################################################################################################


def init_marker(index,x,y,symbole):

    marker_object = Marker()
    marker_object.header.frame_id = "/map"
    marker_object.header.stamp    = rospy.get_rostime()
    marker_object.ns = "haro"
    marker_object.id = index
    marker_object.type = Marker.SPHERE
    marker_object.action = Marker.ADD

    #position
    my_point = Point()
    my_point.x = x
    my_point.y = y
    my_point.z = 0

    #orientation
    marker_object.pose.position = my_point
    marker_object.pose.orientation.x = 0
    marker_object.pose.orientation.y = 0
    marker_object.pose.orientation.z = 0.0
    marker_object.pose.orientation.w = 1.0

    #size
    marker_object.scale.x = 0.2
    marker_object.scale.y = 0.2
    marker_object.scale.z = 0.2

    if symbole == 'Biohazard':
        marker_object.color.r = 1.0
        marker_object.color.g = 1.0
        marker_object.color.b = 0.0
        # This has to be, otherwise it will be transparent
        marker_object.color.a = 1.0

    if symbole == 'Danger':
        marker_object.color.r = 1.0
        marker_object.color.g = 1.0
        marker_object.color.b = 1.0
        # This has to be, otherwise it will be transparent
        marker_object.color.a = 1.0

    if symbole == 'Fire':
        marker_object.color.r = 1.0
        marker_object.color.g = 0.5
        marker_object.color.b = 0.0
        # This has to be, otherwise it will be transparent
        marker_object.color.a = 1.0

    if symbole == 'Radioactive':
        marker_object.color.r = 204/255
        marker_object.color.g = 255/255
        marker_object.color.b = 153/255
        # This has to be, otherwise it will be transparent
        marker_object.color.a = 1.0

    if symbole == 'no smoking':
        marker_object.color.r = 153/255
        marker_object.color.g = 76/255
        marker_object.color.b = 0/255
        # This has to be, otherwise it will be transparent
        marker_object.color.a = 1.0

    if symbole == 'Toxic':
        marker_object.color.r = 153/255
        marker_object.color.g = 51/255
        marker_object.color.b = 255/255
        # This has to be, otherwise it will be transparent
        marker_object.color.a = 1.0

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


def rviz_marker(value,angle,symbole):

    global x_marker
    global y_marker
    global symbol_marker

    sub_odom = rospy.Subscriber("/odom", Odometry, find_position)
    sub_depth = rospy.Subscriber("/scan", LaserScan, find_symbol)

    sym_x = x + np.cos(theta + angle*np.pi/180)
    sym_y = y + np.sin(theta + angle*np.pi/180)




    print"\n pix", pix
    print"\n angle", angle
    print"\n scan", value
    print"\n depth", depth
    print" "


    # Initialization of the first symbole founded
    if len(x_marker) > 0 :

        for i in range(len(x_marker) - 1):

            if symbol_marker[i] != symbol_marker[i+1]:

                symbol_marker = []
                x_marker = []
                y_marker = []

    if len(x_marker) == 0 and sym_x != 0 and depth != 0 :

      x_marker.append(round(sym_x,3))
      y_marker.append(round(sym_y,3))
      symbol_marker.append(symbole)

    # checking if the new symbol published was found before
    # if not then add it to the array
    if len(x_marker) > 0 and len(x_marker) < 10 :

      x_marker.append(round(sym_x,3))
      y_marker.append(round(sym_y,3))
      symbol_marker.append(symbole)

        # concatenated every x , y position and the symbole meaning



    print("    temp   ")
    symbol_array = np.concatenate((x_marker,y_marker))
    print(symbol_array)




    if len(x_marker) == 10 and len(x_marker_final) == 0:

        mean_x = np.sum(x_marker)/len(x_marker)
        mean_y = np.sum(y_marker)/len(y_marker)
        x_marker_final.append(mean_x)
        y_marker_final.append(mean_y)
        symbole_final.append(symbole)
        x_marker = []
        y_marker = []


    if len(x_marker) == 10 and len(x_marker_final) > 0:

        mean_x = np.sum(x_marker)/len(x_marker)
        mean_y = np.sum(y_marker)/len(y_marker)
        x_marker_final.append(mean_x)
        y_marker_final.append(mean_y)
        symbole_final.append(symbole)
        x_marker = []
        y_marker = []

    """
    if len(x_marker) == 10 and len(x_marker_final) > 0:

        mean_x = np.sum(x_marker)/len(x_marker)
        mean_y = np.sum(y_marker)/len(y_marker)
        #print"mean x",mean_x
        #print"mean y",mean_y
        print"symbole : ", symbole
        x_marker = []
        y_marker = []
        count = 0
        for i in range(len(x_marker_final)):
            error = 0.8
            #print(i)

            if (mean_x > x_marker_final[i] - error or mean_x < x_marker_final[i] + error) and (mean_y > y_marker_final[i] - error or mean_y < y_marker_final[i] + error) and symbole_final[i] == symbole:
                count = count + 1
            else:
                pass


        if count > 0 :
            x_marker_final.append(mean_x)
            y_marker_final.append(mean_y)
            symbole_final.append(symbole)

    """
    if len(x_marker_final) > 0:

        for i in range(len(x_marker_final)):

            mark = init_marker(i,x_marker_final[i],y_marker_final[i],symbole_final[i])

            pub_marker.publish(mark)
        #print"done !"


    print" "
    print("   final     ")
    symbol_array = np.concatenate((x_marker_final,y_marker_final,symbole_final))
    print(symbol_array)
    print" "








#############################################################################################################################################################################






if __name__ == "__main__":
    # craeting the node
    rospy.init_node("mapping_corpses")
    # creating subscriber to odometry and laserscan
    pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=1)


    all_marker = []
    x_marker = []
    y_marker = []
    symbol_marker = []

    x_marker_final = []
    y_marker_final = []
    symbole_final  = []

    while not rospy.is_shutdown():

        rospy.sleep(10)
        sub_pixel = rospy.Subscriber("/objects", Float32MultiArray, find_pixel)
