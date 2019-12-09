#! /usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
from cv_bridge  import CvBridge, CvBridgeError
import roslaunch
import os

def image_callback(msg):
	try:
		image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		simplified_image = simplify(image)
		imagedecomp_pub.publish(bridge.cv2_to_imgmsg(simplified_image))
		
	except CvBridgeError as e:
		print(e)

def simplify(image):
	#resize image 
	ratio = 500 / float(image.shape[0])
	resized = cv2.resize(image, (int(ratio * image.shape[1]), 500))
	#convert to hsv 
	hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
	#create bounds, apply inRange function 
	cardboard_lower, cardboard_upper = (13, 97, 49), (18, 154, 199)
	cardboard_mask = cv2.inRange(hsv, cardboard_lower, cardboard_upper)
	#apply opening operation to remove blobs and bitwise AND it with resized image 
	opened = cv2.morphologyEx(cardboard_mask, cv2.MORPH_OPEN, (7, 7), iterations=4)
	output = cv2.bitwise_and(resized, resized, mask=opened)
	#find non zero pixels and then positions in matrix of these pixels 
	non_zero_y = np.nonzero(output)[0]
	#find max y value of these positions 
	maxy = np.min(non_zero_y)
	"""
	#convert to all values above maxy to black in image 
	#for resized one (for displaying and debugging purposes)
	b = np.tile(resized[maxy, :, 0], (maxy, 1))
	g = np.tile(resized[maxy, :, 1], (maxy, 1))
	r = np.tile(resized[maxy, :, 2], (maxy, 1))
	overlay = cv2.merge([b, g, r])
	zeros = np.zeros((image[:maxy_converted, :].shape), np.uint8)
	resized[:maxy, :, :] = overlay
	zeros = np.zeros((resized[:maxy, :].shape), np.uint8)
	resized[:maxy, :] = zeros
	"""
	#convert maxy for original size image and make every pixel above it black
	maxy_converted = int((maxy / float(resized.shape[0])) * image.shape[0]) 
	overlay = np.array(image[maxy_converted, :] * maxy_converted)
	#zeros = np.zeros((image[:maxy_converted, :].shape), np.uint8)
	b = np.tile(image[maxy_converted, :, 0], (maxy_converted, 1))
	g = np.tile(image[maxy_converted, :, 1], (maxy_converted, 1))
	r = np.tile(image[maxy_converted, :, 2], (maxy_converted, 1))
	overlay = cv2.merge([b, g, r])
	image[:maxy_converted, :] = overlay
	#image[:maxy_converted, :] = zeros
	return image

rospy.init_node("simplify_image_node")
image_sub = rospy.Subscriber("/camera/rgb/image_rect_color/compressed", CompressedImage, image_callback)
imagedecomp_pub = rospy.Publisher("/simplified_image/decompressed", Image, queue_size=1)
bridge = CvBridge()

#simplify("/home/valeriofranchi/catkin_ws/src/exploration_perception/test/c6.JPG")

"""
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
configure and execute the launch file 
current_dir = os.getcwd()
launch_file_name = "republish_image.launch"
launch_path = os.path.join(current_dir, "../launch", launch_file_name)
launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
launch.start()
"""

rospy.spin()

