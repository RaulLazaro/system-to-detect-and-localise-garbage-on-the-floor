#!/usr/bin/env python
"""
"""

#-------------------------------------------------------------------------------
#--- IMPORT MODULES 
#-------------------------------------------------------------------------------
import argparse                     #Read command line arguments
import numpy as np                  #Arrays and opencv images

import rospy                        #ros python module
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan


#-------------------------------------------------------------------------------
#--- DEFINITIONS 
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#--- HEADER 
#-------------------------------------------------------------------------------
__author__ = "Miguel Riem de Oliveira"
__date__ = "December 2015"
__copyright__ = "Copyright 2015, V3V"
__credits__ = ["Miguel Riem de Oliveira"]
__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Miguel Oliveira"
__email__ = "m.riem.oliveira@gmail.com"
__status__ = "Development"

#-------------------------------------------------------------------------------
#--- GLOBAL VARIABLES
#-------------------------------------------------------------------------------

bridge = CvBridge()
cv_image = []
laser_scan = []
image_stamp = []

#-------------------------------------------------------------------------------
#--- FUNCTION DEFINITION
#-------------------------------------------------------------------------------

def ImageReceivedCallback(data):
    global cv_image
    global bridge
    global image_stamp

    #print("Received image")

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        image_header = data.header
        image_stamp = data.header.stamp

    except CvBridgeError as e:
        print(e)

def LaserReceivedCallback(data):
    global laser_scan

    print("Received laser")
    laser_scan = data


    
#-------------------------------------------------------------------------------
#--- MAIN
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    global cv_image
    global image_stamp

    #---------------------------------------
    #--- Argument parser
    #---------------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--capture_path", help = "path to the capture folder", default = ".")
    args = vars(ap.parse_args())
   
    #---------------------------------------
    #--- Intitialization
    #---------------------------------------
    rospy.init_node('collect_data_node') #ros node init
    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    h = open('laser.txt','w')
    hi = open('image_stamps.txt','w')

    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, ImageReceivedCallback)
    laser_sub = rospy.Subscriber("/scan", LaserScan, LaserReceivedCallback)

    rospy.sleep(rospy.Duration(0.1)) #time for the tf listener to receive some transforms
   
    rate = rospy.Rate(100) # 10hz
    count = 1

    while not rospy.is_shutdown():

        #print("One iteration complete")

        #cv2.imshow("Camera", cv_image)

        key = (cv2.waitKey(20) & 255)
        #print("key = " + str(key))

        #<timestamp> StartAngleRads AngleIncrementRads EndAngleRads RangeUnitType NoAngles [Ranges]
        range_unit_type = 3 #for meters
        ss_ranges = " ".join(["%.8f" % i for i in laser_scan.ranges])

        ss_time = str(laser_scan.header.stamp.secs) + "." + str(laser_scan.header.stamp.nsecs)
        ss = ss_time + " " + str(laser_scan.angle_min) + " " + str(laser_scan.angle_increment) + " " + str(laser_scan.angle_max) + " " + str(range_unit_type) + " " + str(len(laser_scan.ranges)) + " " + ss_ranges + "\n"

        h.write(ss)

        if key == 113: #q for quit
            print("Quit")
            break
        elif key == 115: #s for save
            print("Saving image and laser scan number " + str(count))
            cv2.imwrite("image_" + str(count) + ".bmp", cv_image)

            ss = str(image_stamp.secs) + "." + str(image_stamp.nsecs) + " " + str(image_stamp.secs) + "." + str(image_stamp.nsecs) + "\n"
            hi.write(ss)

            count += 1

        rate.sleep()

    h.close()
    hi.close()