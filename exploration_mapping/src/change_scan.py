#! /usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan

def scan_cb(msg):
    global new_scan 
    global pub
    new_scan = msg
    new_scan.range_max = 1.0
    new_scan.ranges = list(msg.ranges)
    for i in range(len(msg.ranges)):
        if msg.ranges[i] > new_scan.range_max:
            new_scan.ranges[i] == float('nan')
        else:
            new_scan.ranges[i] = msg.ranges[i]
    pub.publish(new_scan)

rospy.init_node("a_node")
sub = rospy.Subscriber("/scan", LaserScan, scan_cb)
pub = rospy.Publisher("/scan_filtered", LaserScan, queue_size=1)
rospy.spin()