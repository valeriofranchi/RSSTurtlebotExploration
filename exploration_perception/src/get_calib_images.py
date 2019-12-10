#! /usr/bin/env python 

import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError

i = 0
middle = 0.0

def laser_cb(msg):
	global middle
	middle = msg.ranges[len(msg.ranges) // 2]

def image_sub(data):
	global i 		
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		#returns and stores last key pressed on keyboard by user
		cv2.imshow("image", cv_image)
		key = cv2.waitKey(2000) & 0xFF
		#check if key corresponds to esc (quit), if yes quit the video capture
		if key == ord('q'):
			cv2.putText(cv_image, "laser value: %1.2f" %middle, (50, 50), 
				cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, 0)
			cv2.imwrite("/home/craig_ros/Pictures/test/image{}.png".format(i), cv_image)
			i += 1			
	except CvBridgeError as e:
		print(e)
	
rospy.init_node("a_node")
bridge = CvBridge()
subscriber = rospy.Subscriber("/camera/rgb/image_rect_color", Image, image_sub) 
laser_subscriber = rospy.Subscriber("/scan", LaserScan, laser_cb)
rospy.spin()
