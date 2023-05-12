#Import the required libraries
import cv2
import os 
import sys
import select
import time
from robot import Robot
import numpy as np
import Camera  #Library defines the CSI camera parameters
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE

robot = Robot()
#Define parameters of the TOF sensor
vl53 = vl53l5cx.VL53L5CX()
vl53.set_resolution(8 * 8)
vl53.enable_motion_indicator(8 * 8)
vl53.set_motion_distance(400, 1400)
vl53.start_ranging()


#Function used to calibrate the camera before line following
#Determine the way of binary conversion based on average pixel intensity
def camera_calibration(cap):
    global dark  #global variable used by main function
    ret, frame = cap.read()
    height, width = frame.shape[:2]  #Obtain the size of the frame
    roi = frame[int(2*height/3):height, int(width/6):int(width*5/6)]  #crop the frame, leave the region of interests
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)  #Convert the ROI to grayscale
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)  #Convert the grayscale frame into binary based on threshold
    path_avg = cv2.mean(binary)[0]  # calculate the average pixel value of the path 
    if path_avg < 100:  # if the average value is less than 100, path is considered dark
        print("Path is dark")
        dark = True  #Set the global variable to True
    else:  #Otherwise the path is considered bright
        print("Path is bright")
        dark = False  #Set the global variable to False
    return dark

#Use the TOF sensor for obstacle detection
def Object_detection():
    global stop
    if vl53.data_ready():
        data = vl53.get_data()
        d = np.flipud(np.array(data.distance_mm).reshape((8, 8)))  #Obtain the distance matrix
        #Average the center 4x4 matrix
        avg_distance = (d[2,2]+d[2,3]+d[2,4]+d[2,5]+d[3,2]+d[3,3]+d[3,4]+d[3,5]+d[4,2]+d[4,3]+d[4,4]+d[4,5]+d[5,2]+d[5,3]+d[5,4]+d[5,5])/16 
        if avg_distance <= 200:  #If an object is detected within 20 cm in the front
            print('Object detected, stop!')
            robot.stop()
            stop = True
        else:
            stop = False

    return stop

if __name__ == '__main__':
    
    #Capture the video with the CSI camera
    cap = cv2.VideoCapture(Camera.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    #Run the calibration function and obtain the variable 'dark'
    camera_calibration(cap)
    
    while True:
        Object_detection()
        ret, frame = cap.read()
        height, width = frame.shape[:2]
        roi = frame[int(2*height/3):height, int(width/6):int(width*5/6)]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        if dark == True:  # if the average value is less than 100, path is considered dark
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)  #Bitwise conversion to display the path as white
            binary = cv2.bitwise_not(binary)
        else:
            _, binary = cv2.threshold(gray,145, 255, cv2.THRESH_BINARY)  #Normal binary conversion

        kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
        mask = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernal, iterations = 1)  #morphology close operation
        contours, _ = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)  #Draw contours
        if stop == False: #If no obstacle is detected, do line follwoing
            if len(contours) > 0:
                max_contour = max(contours, key=cv2.contourArea)  #Find maximum contour
                M = cv2.moments(max_contour)
                if M["m00"] !=0:  #Obtain the centre coordinate of the contour
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    #Give different speed to the motors
                    if cx <= 320-100 :  #If the center is on the left of the ROI
                        print("Turn Left")
                        robot.left_motor.value = -0.35
                        robot.right_motor.value = 0.4
                    elif cx >=320+100 :
                        print("Turn Right")  #If the center is on the right of the ROI
                        robot.left_motor.value = 0.35
                        robot.right_motor.value = -0.4
                    else:  #If the center is in the middle
                        print("On Track!")
                        robot.left_motor.value = 0.35
                        robot.right_motor.value = 0.4
                    
                    cv2.circle(roi, (cx,cy), 5, (255,255,255), -1)  #Draw the centre point
                    cv2.drawContours(roi, max_contour, -1, (0,255,0), 1)  #Draw the contour
        cv2.imshow("ROI",roi)  #Show the original image window
        cv2.imshow("Original",frame)  #Show the ROI window

        if cv2.waitKey(1)&0xFF ==ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
