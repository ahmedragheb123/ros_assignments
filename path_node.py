#!/usr/bin/env python3
import rospy
import numpy as num
import matplotlib.pyplot as plot
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
wheel_radius = 0.033 # in meters
wheel_base = 0.16 # in meters
dt=0.1 # time step in seconds
total_time = 10.0 # total simulation time in seconds

x = 0.0
y = 0.0
theta = 0.0

def callback(msg):
    	
    global x
    global y
    global theta
	
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
	
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    	
class DifferentialDriveRobot:
    def __init__ (self, wheel_radius, wheel_base):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.x =[] #@initial x position
        self.y=[] #e initial y position
        self.theta=0.0 #@initial orientation
        
    def move(self, x, y, theta):#Compute wheel speeds

            #Update robot pose
            self.x.append(x)
            self.y.append(y)
            self.theta = theta
            
    def plot_robot(self):
        #Plot robot
        plot.plot(self.x,self.y,'ro')
        #plot.quiver(self.x,self.y, num.cos(self.theta), num.sin(self.theta))
        plot.axis('equal')
        plot.grid(True)
        
def listener():
    global speed, thetadot, wheel_radius, wheel_base, dt, total_time, Flag_recieved
    rospy.init_node('plotting', anonymous=True)
    rospy.Subscriber("/odom", Odometry, callback)
    robot = DifferentialDriveRobot(wheel_radius, wheel_base)
    while not rospy.is_shutdown():
        robot.move(x, y, theta)
        robot.plot_robot()
        plot.draw()
        plot.pause(0.1)
        
if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
        	pass
