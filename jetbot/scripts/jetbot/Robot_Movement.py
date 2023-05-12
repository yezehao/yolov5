#!/usr/bin/env python3

#Import the required libraries
import rospy
from std_msgs.msg import String
import os 
import sys
import select
import time
from robot import Robot
import numpy as np
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE
robot = Robot()
#Declare two variables stop1 and stop2
stop1 = False  
stop2 = False

def Jetbot_Subscriber():
    rospy.init_node('Jetbot_Subscriber', anonymous=True)  #Define a node named 'Jetbot_Subscriber'
    rospy.Subscriber("LF_Command", String, callback_Camera)  #Subscribe to the topic 'LF_Command'
    rospy.spin()

#Callback function for the 'OR_Status' topic
def callback_Recognition(data):
    global stop1  #Define global variable stop1
    if data.data == "Object Recognized":  #If the object is recognized'
        #rospy.loginfo(data.data)
        stop1 = True
    else:
        stop1 = False
    return stop1

#Callback function for the 'TOF' topic
def callback_TOF(data):
    global stop2  #Define global variable stop2
    if data.data == "Obstacle detected, stop!":  #If obstacle is detected
        #rospy.loginfo(data.data)
        stop2 = True
    else:
        stop2 = False
    return stop2

#Callback function for the 'LF_Command' topic
def callback_Camera(data):
    global stop1, stop2
    #Subscribe to the topic 'OR_Status'
    rospy.Subscriber("OR_Status", String, callback_Recognition)
    #Subscribe to the topic 'TOF'
    rospy.Subscriber("TOF", String, callback_TOF)
    #Whenever the object is recognized or obstacle is detected
    if stop1 == True or stop2 == True:
        robot.stop()  #The robot stops
    else:  #Otherwise do line following
        rospy.loginfo(data.data)
        #Give different speed to the motors
        if data.data == "Turn Left":
            robot.left_motor.value = -0.35
            robot.right_motor.value = 0.4
        elif data.data == "Turn Right":
            robot.left_motor.value = 0.35
            robot.right_motor.value = -0.4
        elif data.data == "On Track!":
            robot.left_motor.value = 0.35
            robot.right_motor.value = 0.4

if __name__ == '__main__':
    Jetbot_Subscriber()
