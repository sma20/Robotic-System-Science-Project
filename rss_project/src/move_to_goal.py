#! /usr/bin/env python
import rospy#
from geometry_msgs.msg import Point, Twist #to send the command
from nav_msgs.msg import Odometry #to get his position
from math import atan2 #takes into account wether there is a - and where it is
from tf.transformations import euler_from_quaternion#
from rss_project.srv import my_goal, my_goalResponse


#initialization, just to be able to use them wherever they are
#
x= 0.0# not sure they are read
y= 0.0#
theta=0.0#
print("check")#

class movetogoal():

    def __init__(self,x_goal,y_goal):
        self.pub= rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size=1)#
        self.goal_x= x_goal #my goal position
        self.goal_y=y_goal

        self.ctrl_c = False #to exit
        self.speed=Twist()#
        self.rate=rospy.Rate(4) #to give time to the robot to proess instructions
        self.goal= Point()#to give the arrival point
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
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
	rospy.loginfo("Goal reached")

        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.publish_once_in_cmd()
	self.ctrl_c=True

    def resetodometry(self, Odometry):

        #print("starting here")
        global x #not sure global is usefull here
        global y
        global theta
        x= Odometry.pose.pose.position.x
        y= Odometry.pose.pose.position.y
        rot_q= Odometry.pose.pose.orientation
        #this fct gives us 3 angles, we only use the thera but we have to put them all for this fct
        (roll,pitch,theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z,rot_q.w])#

    def get_diff_of_position(self):

        global inc_x
        global inc_y
        global angle_goal
	


        inc_x= self.goal_x -x #calculate the error
        inc_y= self.goal_y -y#
        angle_goal = atan2(inc_y,inc_x)#
	#print("inc_x")
	#print(inc_x)

    def moverobot(self):
        sub= rospy.Subscriber('/odom', Odometry, self.resetodometry)#call this fct
        while not self.ctrl_c :
            """
	    print("x   ")
            print(x)
            print("y")
            print(y)
            print("theta")
            print(theta)
	    """

            self.get_diff_of_position()

            if abs(inc_x)>0.1 or abs(inc_y)>0.1 : #if we are far from goal

                if abs(angle_goal-theta) >0.1 :#if the angle difference is too big
                    if (angle_goal-theta)>0.1: #check the shortest way to turn toward our goal
                        #print("in big angle")
                        self.speed.linear.x=0.0 #correct the angle
                        self.speed.angular.z=0.3#
                        self.publish_once_in_cmd()
                    else:
                        self.speed.linear.x=0.0 #correct the angle
                        self.speed.angular.z= -0.3#
                        self.publish_once_in_cmd()

                else: #keep going straight
		    #print("in straight")
                    self.speed.linear.x=0.15 #not too fast not to make the error margin of the odometry larger
                    self.speed.angular.z=0.0#
                    self.publish_once_in_cmd()
            else:
                self.stoprobot() #goal so stop


            #print("going round here")#
            self.rate.sleep() #give the robot tome to act


if __name__ == '__main__':

    rospy.init_node("decide_goal", anonymous=True)
    move_r = movetogoal()

    try:
        move_r.moverobot()

	#rospy.spin()
    except rospy.ROSInterruptException:
        pass
