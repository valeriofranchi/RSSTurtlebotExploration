#! /usr/bin/env python 
# Subscribes to raw image topic, displays image using OpenCV, 'Q' used to save to file
# Note: uses a hard-coded directory for the image path

import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError

i = 0
middle = 0.0

"""
Obtains middle laser value in scan, used during testing
"""
def laser_cb(msg):
	global middle
	middle = msg.ranges[len(msg.ranges) // 2]

"""
Image subscriber. Displays image using OpenCV, image can then be saved to file.
"""
def image_sub(data):
	global i 		
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		# Returns and stores last key pressed on keyboard by user
		cv2.imshow("image", cv_image)
		key = cv2.waitKey(2000) & 0xFF
		# Check if key corresponds to q (take picture), if yes save image to file
		if key == ord('q'):
			cv2.putText(cv_image, "laser value: %1.2f" %middle, (50, 50), 
				cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, 0)
			# Hard-coded directory, would need to be changed on a different machine
			cv2.imwrite("/home/craig_ros/Pictures/test/image{}.png".format(i), cv_image)
			i += 1			
	except CvBridgeError as e:
		print(e)
	
rospy.init_node("image_capture")
bridge = CvBridge()
subscriber = rospy.Subscriber("/camera/rgb/image_rect_color", Image, image_sub) 
laser_subscriber = rospy.Subscriber("/scan", LaserScan, laser_cb)
rospy.spin()
