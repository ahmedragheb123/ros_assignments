#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pow
import numpy as num
from tf.transformations import euler_from_quaternion

class GoToGoal:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  #10Hz
        self.goal = Point()
        self.current_pose = Point()
        
        
        self.orientation = 0.0
        self.angle_z = 0.0
        
        self.letter = [1, 2, 2, 0.0, 1.5, 1, 0.5, 1]   #letter A      
        self.pointer = 0
        
        self.kp_linear = 0.05  # Proportional gain for linear velocity              
        self.kp_angular = 0.09  # Proportional gain for angular velocity
        
        self.ki_linear = 0.01  # Integral gain for linear velocity
        self.ki_angular = 0.01  # Integral gain for angular velocity
        
        self.kd_linear = 0.01  # Derivative gain for linear velocity
        self.kd_angular = 0.05  # Derivative gain for angular velocity
        
        self.prev_error_linear = 0.0  # Previous error for linear velocity
        self.prev_error_angular = 0.0  # Previous error for angular velocity
        self.integral_linear = 0.0  # Integral term for linear velocity
        self.integral_angular = 0.0  # Integral term for angular velocity

    def update_pose(self, data):
        self.current_pose.x = data.pose.pose.position.x
        self.current_pose.y = data.pose.pose.position.y
        self.orientation = data.pose.pose.orientation
        (roll, pitch, self.angle_z) = euler_from_quaternion ([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        if self.angle_z < 0:
        	self.angle_z = (num.pi * 2) - (self.angle_z * -1)

    def get_distance(self):
    	return sqrt(pow((self.goal.x - self.current_pose.x), 2) + pow(((self.goal.y) - self.current_pose.y), 2))

    def get_orientation(self):
        temp_angle =  atan2(abs(self.goal.y - self.current_pose.y), abs(self.goal.x - self.current_pose.x)) 
        if((self.goal.y - self.current_pose.y) > 0) and ((self.goal.x - self.current_pose.x) > 0):
        	return temp_angle
        elif ((self.goal.y - self.current_pose.y) > 0) and ((self.goal.x - self.current_pose.x) < 0):
        	return num.pi - temp_angle
        elif ((self.goal.y - self.current_pose.y) < 0) and ((self.goal.x - self.current_pose.x) < 0):
        	return num.pi + temp_angle
        elif ((self.goal.y - self.current_pose.y) < 0) and ((self.goal.x - self.current_pose.x) > 0):
        	return (num.pi * 2) - temp_angle

    def move_to_goal(self):
    	while not rospy.is_shutdown():
    		self.goal.x = self.letter[self.pointer]
    		self.goal.y = self.letter[self.pointer + 1]
    		
    		distance_to_goal = self.get_distance()
    		
    		desired_angle = self.get_orientation()
    		
    		error_linear = distance_to_goal
    		error_angular = abs(desired_angle - self.angle_z )
    		
    		self.integral_linear += error_linear * (0.1)
    		self.integral_angular += error_angular * (0.1)
    		
    		derivative_linear = abs(error_linear - self.prev_error_linear) / 0.1
    		derivative_angular = abs(error_angular - self.prev_error_angular) / 0.1
    		
    		linear_velocity = self.kp_linear * error_linear + self.ki_linear * self.integral_linear + self.kd_linear * derivative_linear
    		angular_velocity = self.kp_angular * error_angular + self.ki_angular * self.integral_angular + self.kd_angular * derivative_angular
    		
    		vel_msg = Twist()
    	
    		
    		if (linear_velocity > 0.1):
    			linear_velocity = 0.1
    
    		if angular_velocity < 0:
    			angular_velocity = angular_velocity * -1
    			
    		if (angular_velocity > 0.4):
    			angular_velocity = 0.4
    			
    			
    		
    		if error_angular > 0.15:
    			vel_msg.angular.z = angular_velocity
    			linear_velocity = 0.0
    		else:
    			vel_msg.linear.x = linear_velocity
    			vel_msg.angular.z = 0
    			
    		self.velocity_publisher.publish(vel_msg)
    		
    		self.prev_error_linear = error_linear
    		self.prev_error_angular = error_angular
    		
    		
    		
    		if (abs(self.current_pose.x - self.goal.x) < 0.05) and (abs(self.current_pose.y - self.goal.y) < 0.05) and ((len(self.letter) - 2) != self.pointer):
    			self.pointer = self.pointer + 2
    			#time.sleep(5)
    			
    		if (abs(self.current_pose.x - self.goal.x) < 0.05) and (abs(self.current_pose.y - self.goal.y) < 0.05) and ((len(self.letter) - 2) == self.pointer):
    			vel_msg = Twist()
    			vel_msg.linear.x = 0.0
    			vel_msg.angular.z = 0.0
    			self.velocity_publisher.publish(vel_msg)
    			rospy.loginfo("Goal reached!")
   
    		self.rate.sleep()


if __name__ == '__main__':
    try:
        bot = GoToGoal()
        bot.move_to_goal()
    except rospy.ROSInterruptException:
        pass
