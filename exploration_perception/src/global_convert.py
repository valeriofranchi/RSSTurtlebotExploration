#! /usr/bin/env python
# Imports
import rospy
import tf
import os
import cv2
import numpy as np
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
# Messages
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from exploration_perception.msg import DangerSign
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

# Change from sign number to name
sign_to_name = {
	0: "biohazard",
	1: "danger",
	2: "risk of fire",
	3: "radioactive",
	4: "no smoking",
	5: "toxic",
	6: "worker",
	7: "alive worker",
	8: "dead worker"
}
# Change from object number to sign number
object_to_sign = {
	1: 3,
	2: 3,
	3: 3,
	4: 3,
	5: 4,
	6: 4,
	7: 4,
	8: 4,
	9: 4,
	10: 4,
	11: 0,
	12: 0,
	13: 0,
	14: 0,
	15: 2,
	16: 2,
	17: 2,
	18: 2,
	19: 2,
	20: 6,
	21: 6,
	22: 6,
	23: 6,
	24: 2,
	25: 3,
	26: 5,
	27: 1,
	28: 1,
	29: 1,
	30: 5,
	31: 3,
	32: 3,
	33: 4,
	34: 1,
	35: 5,
	36: 5,
	37: 4,
	38: 5,
	39: 3,
	40: 3,
	41: 4,
	42: 4,
	43: 6,
	44: 6,
	45: 2,
	46: 2,
	47: 6,
	48: 6,
	49: 6,
	50: 0,
	51: 2,
	52: 6,
	53: 6,
	54: 6,
	55: 6,
	56: 6,
	57: 6
}

"""
Determines whether a worker is dead or alive.
Return: "dead" if dead, "alive" if alive.
"""
def determine_worker(image_message, width, height, homography_matrix):
	# Convert image message to OpenCV image
	bridge = CvBridge()
	cv_img = bridge.imgmsg_to_cv2(image_message, "bgr8")
	# Crop image to the relevant sign section
	# Build homography matrix
	h = np.matrix([[homography_matrix[0], homography_matrix[1], homography_matrix[2]],
				   [homography_matrix[3], homography_matrix[4], homography_matrix[5]],
				   [homography_matrix[6], homography_matrix[7], homography_matrix[8]]]).T
	# Get bounding box corners in image frame
	in_points = np.array([[[0, 0], [width, 0], [0, height], [width, height]]])
	out_points = cv2.perspectiveTransform(in_points, h)[0]	# Only top-left corner is used, approximations used for width and height
	# Unpack x, y for top-left corner and convert to integer
	x, y = list(out_points[0])
	x, y = int(x), int(y)
	# Crop the image
	cv_sign_img = cv_img[y:y+int(height), x:x+int(width)]
	# Convert image to HSV - histogram to be made from Hue channel
	cv_sign_img = cv2.cvtColor(cv_sign_img, cv2.COLOR_BGR2HSV)
	# Create histogram - initially all channels were used for histograms, but hue was found to be most useful
	# Colour space ranges
	colour_space_ranges = [180, 256, 256]	# H, S, V
	bin_divider = 10
	# (range)//(bin_divider) bins for each channel
	bin_counts = [num // bin_divider for num in colour_space_ranges]
	ch1, ch2, ch3 = cv2.split(cv_sign_img)  # Separate channels from image
	# Histogram of "hue" channel
	hist1 = cv2.calcHist([ch1], [0], None, [bin_counts[0]], [0, colour_space_ranges[0]])
	# Normalise the histogram so a more meaningful comparison to threshold can be made
	cv2.normalize(hist1, hist1)
	# Bins 5, 6, 7, 8 were found to have the greatest difference between "alive" and "dead" worker
	green_sum = sum(hist1[5:9])
	# Compare the sum to the threshold value. If there's more "green" than threshold then the worker is alive, otherwise it's dead
	result = "alive" if green_sum > 0.05 else "dead"
	return result

"""
Listener to the "/object" topic where find_object_2d publishes information on detected objects.
Determine what sign corresponds to the detected object, publish the sign name and map pose to "/danger_signs".
"""
def object_listener(msg):
	global seq		# Used when creating the message header
	global pub		# "/danger_signs" publisher
	if len(msg.data) == 0:
		return

	# Message index 0 is the object number
	object_index = int(msg.data[0])
	object_frame = "object_" + str(object_index)
	
	# Try/catch required for transform_listener.lookupTransform
	try:
		# Lookup transform of object to map (global cooridnates)
		(trans,rot) = transform_listener.lookupTransform('map', object_frame, rospy.Time(0))

		# Instantiate message
		object_Pose = DangerSign()

		# Add pose to message
		object_Pose.sign_pose.pose.position.x = trans[0]
		object_Pose.sign_pose.pose.position.y = trans[1]
		object_Pose.sign_pose.pose.position.z = trans[2]
		object_Pose.sign_pose.pose.orientation.x = rot[0]
		object_Pose.sign_pose.pose.orientation.y = rot[1]
		object_Pose.sign_pose.pose.orientation.z = rot[2]
		object_Pose.sign_pose.pose.orientation.w = rot[3]

		# Ensure object number is in object-sign dictionary
		if object_index in object_to_sign:
			# Convert object number to sign ID
			sign_id = object_to_sign[object_index]
			# If a worker was detected more checks need to be made
			if sign_id == 6:
				# determine_worker requires the decompressed Image message as well as the Float32MultiArray message
				global image_msg
				if image_msg is None:
					# Will fire if this block is entered before the Image subscriber receives the first image
					# Shouldn't happen, but also shouldn't break anything
					rospy.logerr("Worker detected but image not stored yet")
				else:
					# Width and Height of detected object are in index 1 and 2 of the Float32MultiArray
					width = msg.data[1]
					height = msg.data[2]
					# Indices 3 through 11 make up the homography matrix of the detected object
					homography_matrix = msg.data[3:12]
					# determine_worker returns either "alive" or "dead"
					worker_type = determine_worker(image_msg, width, height, homography_matrix)
					# Set the sign_id based on return string
					# Alive worker - 7, dead worker - 8
					sign_id = 7 if worker_type=="alive" else 8
			# Look up the sign name from the sign_to_name dictionary
			object_Pose.sign_name.data = sign_to_name[sign_id]
		else:
			# Object number wasn't in object-sign dictionary, use a default name
			object_Pose.sign_name.data = "Unknown sign found"

		# Use sequence in header, increment sequence
		object_Pose.sign_pose.header.seq = seq
		seq += 1
		#Publish message
		pub.publish(object_Pose)

	except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
		# Exception occurred when trying to lookup object transform. Provide warning and log error.
		rospy.logwarn("Object not in frame" + object_frame)
		rospy.logerr(e.message)
		# If sign type determined say which it is
		if object_index in object_to_sign:
			rospy.logwarn("Sign: " + sign_to_name[object_to_sign[object_index]])

"""
Listener to the "/simplified_image/decompressed" topic, used to store the most recent decompressed image
Required for the determine_worker method
"""
def image_storage_listener(msg):
	global image_msg
	image_msg = msg


rospy.init_node("dangerSignsNode")			# Initialise node
transform_listener = tf.TransformListener()	# Create transform listener
seq = 0										# Used in creating object_Pose.sign_pose.header
image_msg = None							# Need to keep image message for worker determination
sub = rospy.Subscriber("/objects", Float32MultiArray, object_listener)							# Object subscriber
image_sub = rospy.Subscriber("/simplified_image/decompressed", Image, image_storage_listener)	# Image subscriber
pub = rospy.Publisher("/danger_signs", DangerSign, queue_size=1)								# Sign publisher
rospy.spin()

