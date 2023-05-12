#!/usr/bin/env python

#Import the required libraries
import rospy 
from std_msgs.msg import String
import cv2
import os 
import sys
import select
import time
import numpy as np
import Camera  #Library defines the CSI camera parameters

#Define a node named 'LF_Publisher'
rospy.init_node('LF_Publisher', anonymous=True)
#The topic published by the node is a string named 'LF_Command'
pub = rospy.Publisher('LF_Command', String, queue_size=10)
#Define the publish rate
rate = rospy.Rate(30) # 30hz

#Function used to calibrate the camera before line following
#Determine the way of binary conversion based on average pixel intensity
def camera_calibration(cap):

    global dark  #global variable used by the function 'Line_follow'
    ret, frame = cap.read()
    height, width = frame.shape[:2]  #Obtain the size of the frame
    roi = frame[int(2*height/3):height, int(width/6):int(width*5/6)]  #crop the frame, leave the region of interests
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)  #Convert the ROI to grayscale
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)  #Convert the grayscale frame into binary based on threshold
    path_avg = cv2.mean(binary)[0]  # calculate the average pixel value of the path
    if path_avg < 100:  # if the average value is less than 100, path is considered dark
        status = "Path is dark"
        pub.publish(status)  #Publish the status to topic 'LF_Command'
        dark = True  #Set the global variable to True
    else:
        status = "Path is bright"  #Otherwise the path is considered bright
        pub.publish(status)  #Publish the status to topic 'LF_Command'
        dark = False  #Set the global variable to False
    rospy.loginfo(status)
    return dark  

#Function used to generate the turning command for the line following
def Line_follow():
    #Capture the video with the CSI camera
    cap = cv2.VideoCapture(Camera.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    #Run the calibration function and obtain the variable 'dark'
    camera_calibration(cap)
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        height, width = frame.shape[:2]
        roi = frame[int(2*height/3):height, int(width/6):int(width*5/6)]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        if dark == True:  # if the average value is less than 100, path is considered dark
            _, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
            binary = cv2.bitwise_not(binary) #Bitwise conversion to display the path as white
        else:
            _, binary = cv2.threshold(gray,145, 255, cv2.THRESH_BINARY)  #Normal binary conversion
            
        kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
        mask = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernal, iterations = 1)  #morphology close operation
        contours, _ = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)  #Draw contours
        if len(contours) > 0 :
            max_contour = max(contours, key=cv2.contourArea)  #Find maximum contour
            M = cv2.moments(max_contour)
            if M["m00"] !=0 :  #Obtain the centre coordinate of the contour
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if cx <= 320-100:  #If the center is on the left of the ROI
                    status = "Turn Left"
                if cx < 320+100 and cx > 320-100 :  #If the center is in the middle
                    status = "On Track!"
                if cx >= 320+100:  #If the center is on the right of the ROI
                    status = "Turn Right"
                #cv2.circle(roi, (cx,cy), 5, (255,255,255), -1)  #Draw the centre point
                #cv2.drawContours(roi, max_contour, -1, (0,255,0), 1)  #Draw the contour
                pub.publish(status)  #Publish the turning command to the topic 'LF_Command'
        #cv2.imshow('Line_Follower_View', roi)  #Show the image window
        if cv2.waitKey(1)&0xFF ==ord('q'):
            break
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        Line_follow()
    except rospy.ROSInterruptException:
        pass
