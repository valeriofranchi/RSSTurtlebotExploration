#! /usr/bin/env python

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

#Needed imports and message formats
import rospy
import tf
import os
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from exploration_perception.msg import DangerSign
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

#Initialise Transform listener
#listener = None
rospy.init_node("dangerSignsNode")
listener = tf.TransformListener()
seq = 0
image_msg = None
# Determine whether a workeris dead or alive
def determine_worker(image_message, width, height, homography_matrix):
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
	cv_img_cropped = cv_img[y:y+int(height), x:x+int(width)]
	cv2.imshow("sign", cv_img_cropped)
	cv2.waitKey(0)

#Define Interrupt method for when data is recieved from topic
def object_listener(msg):

    global seq
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
        (trans,rot) = listener.lookupTransform('odom',object_frame, rospy.Time(0))
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
            # Check for worker
            if object_to_sign[object_index] == 6:
                global image_msg
                if image_msg is None:
                    rospy.logerr("Worker detected but image not stored yet")
                else:
                    width = msg.data[1]
                    height = msg.data[2]
                    homography_matrix = msg.data[3:12]
                    determine_worker(image_msg, width, height, homography_matrix)
            object_Pose.sign_id = object_to_sign[object_index]


        else:                               #If not in dictionary set it to default value
            rospy.logdebug("Object not in dictionary") 
            object_Pose.sign_id = 99    

        object_Pose.sign_pose.header.seq = seq
        seq = seq + 1

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

sub = rospy.Subscriber("/objects", Float32MultiArray, object_listener)
image_sub = rospy.Subscriber("/simplified_image/decompressed", Image, image_storage_listener)
pub = rospy.Publisher("/danger_signs", DangerSign, queue_size=1)
rospy.spin()


#First Method for getting TFs from message
# def tf_listener(data):
# 	global transform
# 	for transform in data.transforms:
# 		print(transform)
# 		if transform.child_frame_id[:6] == "object":
# 			publisher.publish(transform)



# subscriber = rospy.Subscriber("/tf", TFMessage, tf_listener)
# publisher = rospy.Publisher("/tf_signs", TransformStamped, queue_size=1)
# rospy.spin()

#---------------------------------------------------------------------------
#Second Method		
# if __name__ == '__main__':

#     #Setup node, publisher and listener
# 	rospy.init_node("sign_node")
# 	publisher = rospy.Publisher("/danger_signs", DangerSign, queue_size=1)
# 	listener = tf.TransformListener()

# 	print "Starting loop"
	
#     #Infinite loop
# 	while not rospy.is_shutdown():
#         #Set path to where object are stored
# 		path = '/home/craig_ros/catkin_ws/src/rssp/exploration_perception/src/vision_data/objects' #This path should be relative not absolute

#         #Get number of object that currenly exist
# 		object_list_size = len(os.listdir(path))
# 		object_name = "/object_"        

#         #Check every object that currently exists
# 		for i in range(object_list_size):
#             object_frame = object_name + str(i)
#             # #Attempt to lookup tranform 
#             try:
#                 #Lookup transform of object to map (global cooridnates)
#                 (trans,rot) = listener.lookupTransform('/map',object_frame, rospy.Time(0))

#                 #Create empty message to send data
#                 object_Pose = DangerSign()

#                 #Add position info to message
#                 object_Pose.sign_pose.pose.position.x = trans[0]
#                 object_Pose.sign_pose.pose.position.y = trans[1]
#                 object_Pose.sign_pose.pose.position.z = trans[2]

#                 #Add rotation info to message
#                 object_Pose.sign_pose.pose.orientation.x = rot[0]
#                 object_Pose.sign_pose.pose.orientation.y = rot[1]
#                 object_Pose.sign_pose.pose.orientation.z = rot[2]
#                 object_Pose.sign_pose.pose.orientation.w = rot[3]

#                 if i in object_to_sign:  #Lookup marker ID based from dictionary
#                     object_Pose.sign_id = object_to_sign[i]
#                 else:                               #If not in dictionary set it to default value 
#                     object_Pose.sign_id = 99    

#                 #Publish message
#                 pub.publish(object_Pose)

#             except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                 # rospy.logwarn("Object not in frame")
#                 continue