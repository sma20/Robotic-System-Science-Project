#! /usr/bin/env python
#with services


import rospy
from geometry_msgs.msg import Point, Twist #to send the command
from nav_msgs.msg import Odometry #to get his position
from math import atan2, exp, pi, cos, sin, radians
from tf.transformations import euler_from_quaternion#
from sensor_msgs.msg import LaserScan
from rss_project.srv import my_goal, my_goalResponse
import time
#initialization, just to be able to use them wherever they are
#
x= 0.0# not sure they are read
y= 0.0#
theta=0.0#
print("check")

class moveforcefield():

    def __init__(self,x_goal,y_goal):
        self.time_begin = rospy.Time.now()
        self.pub= rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size=1)#
        self.sub= rospy.Subscriber('/odom', Odometry, self.resetodometry)#call this fct
        self.sub1 = rospy.Subscriber('/scan', LaserScan, self.forcefield)
        self.ctrl_c = False #to exit
        self.speed=Twist() #to move the robot
        self.rate=rospy.Rate(4) #to give time to the robot to proess instructions
        self.goal_x= x_goal #my goal position
        self.goal_y=y_goal
        self.np= Point()#move forcefield
        rospy.on_shutdown(self.shutdownhook)
        self.wall=False # to check if there is a wall or not, to adapt comportement
        self.check_goal=Point() #to check if position goal = x or y robot
        self.inc_goal=Point() #diff between pos goal and actual pos
        self.success=False


    def checkgoaly(self):
        if (y==0 and self.goal_y==0 and self.goal_x < 0) or ( y==0 and self.goal_x-x<0): #prob with arctan
            self.check_goal.y=1
            self.goal_y+=1
        else :
            self.check_goal.y=0


    def publish_once_in_cmd(self):
        while not self.ctrl_c:
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(self.speed)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
    # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def stoprobot(self):
        #rospy.loginfo("Stop robot")
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.publish_once_in_cmd()

    def resetodometry(self, Odometry):
        #rospy.loginfo("starting here")
        global x #not sure global is usefull here
        global y
        global theta
        x= Odometry.pose.pose.position.x
        y= Odometry.pose.pose.position.y
        rot_q= Odometry.pose.pose.orientation
        #this fct gives us 3 angles, we only use the thera but we have to put them all for this fct
        (roll,pitch,theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z,rot_q.w])#

    def get_diff_of_position(self):

        if self.check_goal.y==1 and abs(y)>0.2: #check if we passed this annoying configuration that atan doesn't like
            self.goal_y-=1 #we set back the goal to normal
            self.check_goal.y=0 #we don't want to enter this if anymore

        self.inc_goal.x= self.goal_x -x #calculate the error
        self.inc_goal.y= self.goal_y -y#

    

    def forcefield(self,LaserScan):
        self.get_diff_of_position()

        counter=1
        Frep_x=0
        Frep_y=0
        Ftot_Rep_x=0 #reset Frep before each new check
        Ftot_Rep_y=0
        cstrepulse=10 #to adapt later on
        cstattraction=2 #to adapt later on
        pas=0.5 #to move a distance from actual point
        po=2 #if the scan value is above this, we don't take it into account

        euclidian_dist_goal=((x-self.goal_x)**2 + (y-self.goal_y)**2)**0.5 #not used

        Fatt_x= abs(cstattraction*(self.inc_goal.x)/euclidian_dist_goal)#NB: inc_goalx = xgoal-xposition that's why the - disapear of this formula
        Fatt_y= abs(cstattraction*(self.inc_goal.y)/euclidian_dist_goal)
        """rospy.loginfo("Fatt_x and Fatt_y")
        rospy.loginfo(Fatt_x)
        rospy.loginfo(Fatt_y)"""
        ranges=LaserScan.ranges[:]
        lenght=len(LaserScan.ranges[:])
        #ranges= LaserScan.ranges[80:lenght-80] #real robot
        #print("ranges")
        #print (ranges)
        for i in range(len(ranges)):
            dist_o=LaserScan.ranges[i]

            Beta= (360-i)*pi/len(ranges) #720  #(360-i)*pi/720
            
            if dist_o < po and not dist_o == "inf" :
            #else we are repulsed from the wall even if it doesnt bother
                x_obstacle= x + dist_o*cos(Beta)
                y_obstacle= y + dist_o*sin(Beta)
                #euclidian_dist_goal=((x-x_obstacle)**2 + (y-y_obstacle)**2)**0.5 # = dist_o

                Frep_x= -abs(cstrepulse*((x-x_obstacle)/dist_o)*(1/dist_o- 1/po)*(1/dist_o**2))
                Frep_y= -abs(cstrepulse*((y-y_obstacle)/dist_o)*(1/dist_o- 1/po)*(1/dist_o**2))
                counter+=1
            else:
                Frep_x=0
                Frep_y=0

            Ftot_Rep_x=Frep_x+Ftot_Rep_x
            Ftot_Rep_y=Frep_y+Ftot_Rep_y
        
        rospy.loginfo("Ftot_rep_x and Ftot_Rep_y")
        rospy.loginfo(Ftot_Rep_x)
        rospy.loginfo(Ftot_Rep_y)
        rospy.loginfo("Fatttttt x y")
        rospy.loginfo(Fatt_x)
        rospy.loginfo(Fatt_y)
        
        #WARNING: This IF only works with this cst of repulsion/attraction, if i modify it i have to modify this fct
        #rospy.loginfo(self.inc_goal)
        #rospy.loginfo(LaserScan.ranges[360])
        """
        if LaserScan.ranges[360]<1.5 and LaserScan.ranges[360]>0.5 : #if the goal is right in front of the obstacle yet no TOO close
            if abs(self.inc_goal.y) <0.4 and abs(self.inc_goal.x) < 0.8 :
                Ftot_Rep_y=0
                Ftot_Rep_x=0
            if abs(self.inc_goal.y) <0.8 and abs(self.inc_goal.x) < 0.4 :
                Ftot_Rep_y=0
                Ftot_Rep_x=0
        """
        self.np.x= x+pas*(Ftot_Rep_x/counter+Fatt_x) #position now+ something--> position where to go
        self.np.y= y+pas*(Ftot_Rep_y/counter+Fatt_y)

        if Ftot_Rep_x==0 and Ftot_Rep_y==0 :
            self.wall= False
            #print("no wall")
        else:
            self.wall= True

    def moverobot(self):

        count=0 #check if goal really reached
        #doesn't work want called in this service thing
        if count==0 : #we want this fct ran once at the beginning. the count will get +1 at the end of teh while (at the start)
            self.checkgoaly()
            #print("i'm there")
        print("goal")
        print(self.goal_x, self.goal_y)

        while not self.ctrl_c :
            """
            time_end = rospy.Time.now()
            if (time_end-self.time_begin).to_sec()> 500:
                rospy.loginfo("forcefield too slow")
                break
            """
            angle_d = atan2(self.np.y,self.np.x)
            angle_goal = atan2(self.inc_goal.y,self.inc_goal.x)
            """
            print ("np x et y")
            print self.np.x,self.np.y
            print("angle_d")
            print(angle_d)
            print("angle_goal")
            print(angle_goal)
            """

            """
            print("x ")
            print(x)
            print("y")
            print(y)
            """
            print("theta")
            print(theta)
            
            print("inc x  y ")
            print(self.inc_goal.x, self.inc_goal.y)
            
            if abs(self.inc_goal.x)>0.15 or abs(self.inc_goal.y)>0.15 : #if we are far from goal
                if self.wall==False: #if it doesn't see a wall
                    if abs(angle_goal-theta)> 0.1 :#if the angle difference is too big
                        if angle_goal-theta> 0.1:
                            self.speed.linear.x= 0#correct the angle
                            self.speed.angular.z= 0.7#
                            self.publish_once_in_cmd()

                        if angle_goal-theta< -0.1 :
                            self.speed.linear.x= 0 #correct the angle
                            self.speed.angular.z= -0.7#
                            self.publish_once_in_cmd()

                    else : #keep going straight
                        self.speed.linear.x=0.8 #not too fast not to make the error margin of the odometry larger
                        self.speed.angular.z=0.0#
                        self.publish_once_in_cmd()
                    #break #No wall, no need for the forcefield

                if self.wall==True: #if it sees a wall
                    if abs(angle_goal-angle_d)> 0.1 :#if the angle difference is too big
                            if angle_goal-angle_d> 0.1:
                                self.speed.linear.x= 0.1#correct the angle
                                self.speed.angular.z= 0.7
                                self.publish_once_in_cmd()

                            if angle_goal-angle_d< -0.1 :
                                self.speed.linear.x= 0.1 #correct the angle
                                self.speed.angular.z= -0.7
                                self.publish_once_in_cmd()

                    else : #keep going straight
                        self.speed.linear.x=0.8#not too fast not to make the error margin of the odometry larger
                        self.speed.angular.z=0.0#
                        self.publish_once_in_cmd()

            else : #if we are close enough from the goal

                count+=1
                if count >1:
                    self.stoprobot() #goal so stop
                    self.success=True
                    break

            #rospy.loginfo("going round here")#
            self.rate.sleep() #give the robot tome to act
        self.stoprobot() #to stop
        return self.success


if __name__ == '__main__':

    rospy.init_node("avoid_obstacles", anonymous=True)
    move_r =  moveforcefield()
    #move_r.checkgoaly()

    try:
        move_r.moverobot()
    except rospy.ROSInterruptException:
        pass

