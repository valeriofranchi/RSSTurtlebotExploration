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

# Change from sign int to name
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
# Change from object int to sign int
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


# Determine whether a worker is dead or alive
def determine_worker(image_message, width, height, homography_matrix):
	# Get cropped cv image
	print("Checking worker! {},{}, len(mat):{}".format(width, height, len(homography_matrix)))
	bridge = CvBridge()
	cv_img = bridge.imgmsg_to_cv2(image_message, "bgr8")
	h = np.matrix([[homography_matrix[0], homography_matrix[1], homography_matrix[2]],
				   [homography_matrix[3], homography_matrix[4], homography_matrix[5]],
				   [homography_matrix[6], homography_matrix[7], homography_matrix[8]]]).T
	in_points = np.array([[[0, 0], [width, 0], [0, height], [width, height]]])
	out_points = cv2.perspectiveTransform(in_points, h)[0]
	rospy.logwarn("POINTS\n"+str(out_points))
	rospy.logwarn("MATRIX\n"+str(h))
	rospy.logwarn("OUT SHAPE: "+str(out_points.shape))
	x, y = list(out_points[0])
	x, y = int(x), int(y)
	cv_sign_img = cv_img[y:y+int(height), x:x+int(width)]
	# CLAHE
	cv_sign_hsv = cv2.cvtColor(cv_sign_img, cv2.COLOR_BGR2HSV)
	h, s, v = cv2.split(cv_sign_hsv)
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
	v = clahe.apply(v)
	cv_sign_hsv = cv2.merge([h, s, v])
	cv_sign_img = cv_sign_hsv#cv2.cvtColor(cv_sign_hsv, cv2.COLOR_HSV2BGR)
	# Create histograms
	# Colour space ranges
	colour_space_ranges = [180, 256, 256]# [180, 256, 256]   # HSV
	bin_divider = 10
	bin_counts = [num // bin_divider for num in colour_space_ranges]
	ch1, ch2, ch3 = cv2.split(cv_sign_img)  # Separate channels
	hist1 = cv2.calcHist([ch1], [0], None, [bin_counts[0]], [0, colour_space_ranges[0]])
	hist2 = cv2.calcHist([ch2], [0], None, [bin_counts[1]], [0, colour_space_ranges[1]])
	hist3 = cv2.calcHist([ch3], [0], None, [bin_counts[2]], [0, colour_space_ranges[2]])
	cv2.normalize(hist1, hist1)
	cv2.normalize(hist2, hist2)
	cv2.normalize(hist3, hist3)
	#cv2.imshow("sign", cv_sign_img)
	#cv2.waitKey(0)
	#g_sum = sum([i * hist2[i] for i in range(len(hist2))])	# how green it is
	#r_sum = sum([i * hist3[i] for i in range(len(hist3))])	# how red it is
	#result = "green" if g_sum > r_sum else "red"
	green_sum = sum(hist1[5:9])
	result = "green" if green_sum > 0.15 else "red"
	result += str(green_sum)
	#display_hists([hist1, hist2, hist3], result)
	#cv2.destroyAllWindows()
	return "alive" if green_sum > 0.15 else "dead"		# is it more green than red?

def display_hists(histograms, title):
	fig, ax = plt.subplots(len(histograms))
	fig.canvas.set_window_title(title)
	for i, hist in enumerate(histograms):
		ax[i].plot(np.arange(len(hist)), hist.ravel())
	plt.show()

#Define Interrupt method for when data is recieved from topic
def object_listener(msg):
	global seq
	global pub
	rospy.logdebug("Entered callback")
	if len(msg.data) == 0:
		return

	#Data array element zero and generate object name
	object_index = int(msg.data[0])
	object_name = "object_"
	object_frame = object_name + str(object_index)
	rospy.logdebug("About to enter try/catch")
	
	#Attempt to lookup tranform 
	try:
		rospy.logdebug("Entered try")

		#Lookup transform of object to map (global cooridnates)
		(trans,rot) = listener.lookupTransform('map',object_frame, rospy.Time(0))
		rospy.logdebug("After lookup")

		#Create empty message to send data
		object_Pose = DangerSign()
		rospy.logdebug("Created Message")

		#Add position info to message
		object_Pose.sign_pose.pose.position.x = trans[0]
		object_Pose.sign_pose.pose.position.y = trans[1]
		object_Pose.sign_pose.pose.position.z = trans[2]
		rospy.logdebug("Assigned linear positions: ")

		#Add rotation info to message
		object_Pose.sign_pose.pose.orientation.x = rot[0]
		object_Pose.sign_pose.pose.orientation.y = rot[1]
		object_Pose.sign_pose.pose.orientation.z = rot[2]
		object_Pose.sign_pose.pose.orientation.w = rot[3]
		rospy.logdebug("Assigned angular positions: ")

		if object_index in object_to_sign:  #Lookup marker ID based from dictionary
			rospy.logdebug("Found Object in dictionary")
			# Convert object num to sign ID
			sign_id = object_to_sign[object_index]
			# Check for worker
			if sign_id == 6:
				global image_msg
				if image_msg is None:
					rospy.logerr("Worker detected but image not stored yet")
				else:
					width = msg.data[1]
					height = msg.data[2]
					homography_matrix = msg.data[3:12]
					# "alive" or "dead"
					worker_type = determine_worker(image_msg, width, height, homography_matrix)
					# Alive worker - 7, dead worker - 8
					sign_id = 7 if worker_type=="alive" else 8
			# Set pose ID to determined sign ID
			object_Pose.sign_name.data = sign_to_name[sign_id]
			rospy.logwarn("The detected sign is {}".format(object_Pose.sign_name.data))


		else:							   #If not in dictionary set it to default value
			rospy.logdebug("Object not in dictionary") 
			object_Pose.sign_name.data = "No object found"

		object_Pose.sign_pose.header.seq = seq
		seq += 1

		#Publish message
		rospy.logdebug("Publishing Message")
		pub.publish(object_Pose)

	except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
		rospy.logwarn("Object not in frame" + object_frame)
		rospy.logerr(e.message)

		if object_index in object_to_sign:  #Lookup marker ID based from dictionary
			rospy.logwarn("Sign: " + sign_to_name[object_to_sign[object_index]])

def image_storage_listener(msg):
	global image_msg
	image_msg = msg


rospy.init_node("dangerSignsNode")
listener = tf.TransformListener()	# Create listener
seq = 0					# Used in creating object_Pose.sign_pose.header
image_msg = None			# Need to keep image message for worker determination
sub = rospy.Subscriber("/objects", Float32MultiArray, object_listener)	# Object subscriber
image_sub = rospy.Subscriber("/simplified_image/decompressed", Image, image_storage_listener)	# Image subscriber
pub = rospy.Publisher("/danger_signs", DangerSign, queue_size=1)	# Sign publisher
rospy.spin()

