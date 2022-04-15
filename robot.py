#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from nav_msgs.msg import Odometry #import library for position and orientation data
from geometry_msgs.msg import Twist
import os
import time
import math


class Circling(): #main class
   
    def __init__(self): #main function
        global circle
        circle = Twist() #create object of twist type  
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #publish message
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback) #subscribe message 
        #self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) #subscribe message
    
    def odometry(self, msg): #function for odometry
        self.odometry.var = msg.pose.pose
    	
        print (msg.pose.pose) #print position and orientation of turtlebot    
        

    def callback(self, msg): #function for obstacle avoidance

      
      	#Obstacle Avoidance
        a = self.odometry.var
        self.distance = 0.5
        dx = 3
        dy = 7
        v = 0.2*math.sqrt(((dx-a.position.x)**2)+((dy-a.position.y)**2))
        w = 0.5*(math.atan((dy-a.position.y)/(dx-a.position.x))-(a.orientation.z))
        if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance: 
        #when no any obstacle near detected
            circle.linear.x = v # go (linear velocity)
            circle.angular.z = w # rotate (angular velocity)
            if 5*v < 1:
              circle.linear.x = 0.0
              circle.angular.z = 0.0
            if 10*w < 0.2:
              circle.linear.x = 0.0
              circle.angular.z = 0.0
            self.pub.publish(circle)
            rospy.loginfo("Circling") #state situation constantly
        else: #when an obstacle near detected
            rospy.loginfo("An Obstacle Near Detected") #state case of detection
            circle.linear.x = 0.0 # stop
            circle.angular.z = 0.5 # rotate counter-clockwise
            if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
                #when no any obstacle near detected after rotation
                circle.linear.x = 0.5 #go
                circle.angular.z = 0.1 #rotate
        self.pub.publish(circle) # publish the move object



if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node') #initilize node
    Circling() #run class
    rospy.spin() #loop it