#!/usr/bin/env python3

#Import the required libraries
import time
import numpy
import rospy
from std_msgs.msg import String
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE

def TOF_publisher():
    rospy.init_node('TOF_Publisher', anonymous=True)  #Define a node named 'TOF_Publisher'
    pub = rospy.Publisher('TOF', String, queue_size=10)  #The topic published by the node is a string named 'TOF'
    rate = rospy.Rate(10) # 10 Hz
    print("Uploading firmware, please wait...")
    vl53 = vl53l5cx.VL53L5CX()
    rospy.loginfo("TOF is ready")  #Inform the user that the sensor is ready
    #Define parameters of the sensor
    vl53.set_resolution(8 * 8)
    vl53.enable_motion_indicator(8 * 8)
    vl53.enable_motion_indicator(8 * 8)
    vl53.set_motion_distance(400, 1400)
    vl53.start_ranging()

    while not rospy.is_shutdown():
        if vl53.data_ready():
            data = vl53.get_data()
            d = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))  #Obtain the distance matrix
            #Average the center 4x4 matrix
            avg_distance = (d[2,2]+d[2,3]+d[2,4]+d[2,5]+d[3,2]+d[3,3]+d[3,4]+d[3,5]+d[4,2]+d[4,3]+d[4,4]+d[4,5]+d[5,2]+d[5,3]+d[5,4]+d[5,5])/16 
            rospy.loginfo("Average distance: %f", avg_distance)  #Display the Average distance
            if avg_distance <= 200:  #If an object is detected within 20 cm in the front
                status = "Obstacle detected, stop!"
            else:
                status = "No Obstacles"
            rospy.loginfo(status)  
            pub.publish(status)  #Publish the detection result
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        TOF_publisher()
    except rospy.ROSInterruptException:
        pass



