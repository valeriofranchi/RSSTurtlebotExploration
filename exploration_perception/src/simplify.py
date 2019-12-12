#! /usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
from cv_bridge  import CvBridge, CvBridgeError
import roslaunch
import os

def simplified_callback(msg):
	pass
	#rospy.loginfo("SIMPLIFIED\nHeader:{}, Height:{}, Width:{}, Encoding:{}, isBigEndian:{}, Step:{}, Length of Data:{}".format(msg.header,
	#	msg.height, msg.width, msg.encoding, msg.is_bigendian, msg.step, len(msg.data)))

def depth_callback(msg):
	pass
	#rospy.loginfo("DEPTH\nHeader:{}, Height:{}, Width:{}, Encoding:{}, isBigEndian:{}, Step:{}, Length of Data:{}".format(msg.header,
	#	msg.height, msg.width, msg.encoding, msg.is_bigendian, msg.step, len(msg.data)))

def original_callback(msg):
	pass
	#rospy.loginfo("RAW\nHeader:{}, Height:{}, Width:{}, Encoding:{}, isBigEndian:{}, Step:{}, Length of Data:{}".format(msg.header,
	#	msg.height, msg.width, msg.encoding, msg.is_bigendian, msg.step, len(msg.data)))

def image_callback(msg):
	global bridge
	global imagedecomp_pub
	try:
		image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		#simplified_image = simplify(image)
		image_message = bridge.cv2_to_imgmsg(image, "bgr8")
		image_message.header.stamp = rospy.Time.now()
		image_message.header.frame_id = "camera_rgb_optical_frame"
		imagedecomp_pub.publish(image_message)
	except CvBridgeError as e:
		print(e)

def simplify(image):
	#image = cv2.imread(image) 
	#resize image 
	ratio = 500 / float(image.shape[0])
	resized = cv2.resize(image, (int(ratio * image.shape[1]), 500))
	cutoff_point = resized.shape[0] // 8
	cutoff = np.zeros((cutoff_point, resized.shape[1], 3), np.uint8)
	resized[:cutoff_point, :] = cutoff
	#convert to hsv 
	hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
	h, s, v = cv2.split(hsv)
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
	s = clahe.apply(s)
	v = clahe.apply(v)
	hsv = cv2.merge([h, s, v])
	#create bounds, apply inRange function 
	cardboard_lower, cardboard_upper = (0, 0, 0), (179, 116, 133)
	cardboard_mask = cv2.inRange(hsv, cardboard_lower, cardboard_upper)
	#apply opening operation to remove blobs and bitwise AND it with resized image 
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
	opened = cv2.morphologyEx(cardboard_mask, cv2.MORPH_OPEN, kernel, iterations=6)
	output = cv2.bitwise_and(resized, resized, mask=opened)
	#cv2.imshow("output", output)
	#cv2.waitKey(0)
	#cv2.destroyAllWindows()
	#find non zero pixels and then positions in matrix of these pixels 
	non_zero_y = np.nonzero(output)[0]
	#find max y value of these positions 
	if len(non_zero_y) > 0:
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
		if maxy_converted - 30 > 0:
			overlay = np.array(image[maxy_converted-30, :] * maxy_converted-30)
			#zeros = np.zeros((image[:maxy_converted, :].shape), np.uint8)
			b = np.tile(image[maxy_converted-30, :, 0], (maxy_converted-30, 1))
			g = np.tile(image[maxy_converted-30, :, 1], (maxy_converted-30, 1))
			r = np.tile(image[maxy_converted-5, :, 2], (maxy_converted-30, 1))
			overlay = cv2.merge([b, g, r])
			image[:maxy_converted - 30, :] = overlay
			#image[:maxy_converted, :] = zeros"""
	return image

rospy.init_node("simplify_image_node")
bridge = CvBridge()
imagedecomp_pub = rospy.Publisher("/simplified_image/decompressed", Image, queue_size=1)
image_sub = rospy.Subscriber("/camera/rgb/image_rect_color/compressed", CompressedImage, image_callback)
depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, depth_callback)
simplified_sub = rospy.Subscriber("/camera/rgb/simplified", Image, simplified_callback)
original_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, original_callback)
#thresh_pub = rospy.Publisher("/thresholded_image",Image, queue_size=1)

#simplify("/home/valeriofranchi/catkin_ws/src/exploration_perception/test/image0.png")

rospy.spin()

