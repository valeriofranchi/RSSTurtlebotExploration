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

#Initialise Transform listener
#listener = None
rospy.init_node("dangerSignsNode")
listener = tf.TransformListener()

#Define Interrupt method for when data is recieved from topic
def object_listener(msg):
    rospy.loginfo("Entered callback")
    if len(msg.data) == 0:
        return

    #Data array element zero and generate object name
    object_index = int(msg.data[0])
    object_name = "object_"
    object_frame = object_name + str(object_index)
    rospy.loginfo("About to enter try/catch")
    
    #Attempt to lookup tranform 
    try:
        rospy.loginfo("Entered try")

        #Lookup transform of object to map (global cooridnates)
        (trans,rot) = listener.lookupTransform('odom',object_frame, rospy.Time(0))
        rospy.loginfo("After lookup")

        #Create empty message to send data
        object_Pose = DangerSign()
        rospy.loginfo("Created Message")

        #Add position info to message
        object_Pose.sign_pose.pose.position.x = trans[0]
        object_Pose.sign_pose.pose.position.y = trans[1]
        object_Pose.sign_pose.pose.position.z = trans[2]
        rospy.loginfo("Assigned linear positions")

        #Add rotation info to message
        object_Pose.sign_pose.pose.orientation.x = rot[0]
        object_Pose.sign_pose.pose.orientation.y = rot[1]
        object_Pose.sign_pose.pose.orientation.z = rot[2]
        object_Pose.sign_pose.pose.orientation.w = rot[3]
        rospy.loginfo("Assigned angular positions")

        if object_index in object_to_sign:  #Lookup marker ID based from dictionary
            rospy.loginfo("Found Object in dictionary")
            object_Pose.sign_id = object_to_sign[object_index]

        else:                               #If not in dictionary set it to default value
            rospy.loginfo("Object not in dictionary") 
            object_Pose.sign_id = 99    

        #Publish message
        rospy.loginfo("Publishing Message")
        pub.publish(object_Pose)

    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("Object not in frame" + object_frame)
        rospy.logerr(e.message)

        if object_index in object_to_sign:  #Lookup marker ID based from dictionary
            rospy.logwarn("Sign: " + sign_to_name[object_to_sign[object_index]])

sub = rospy.Subscriber("/objects", Float32MultiArray, object_listener)
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