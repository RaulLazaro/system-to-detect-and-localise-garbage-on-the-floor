#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print msg.ranges[270]

rospy.init_node('sub_node')
sub = rospy.Subscriber("/scan", LaserScan, callback)

rospy.spin()