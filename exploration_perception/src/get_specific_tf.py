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

import rospy
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from exploration_perception.msg import DangerSign

listener = None

def tf_listener(data):
	global transform
	for transform in data.transforms:
		print(transform)
		if transform.child_frame_id[:6] == "object":
			publisher.publish(transform)

# rospy.init_node("a_node")
# subscriber = rospy.Subscriber("/tf", TFMessage, tf_listener)
# publisher = rospy.Publisher("/tf_signs", TransformStamped, queue_size=1)
# rospy.spin()
		
if __name__ == '__main__':
	rospy.init_node("sign_node")
	publisher = rospy.Publisher("/danger_signs", DangerSign, queue_size=5)

	listener = tf.TransformListener()

	

	print "Starting loop"
	
	while not rospy.is_shutdown():
		path = os.getcwd() + '/exploration_perception/src/vision_data/objects'

		object_List_size = len(os.listdir(path))
		object_Name = "/object_"

		for i in range(object_List_size):
			
			object_Frame = object_Name + str(i)
			# print object_Frame

			try:
				#print "Trying to transform"
				(trans,rot) = listener.lookupTransform('/map',object_Frame, rospy.Time(0))

				print "1"
				object_Pose = DangerSign()
				print "Message Created"
				object_Pose.sign_pose.pose.position.x = trans[0]
				object_Pose.sign_pose.pose.position.y = trans[1]
				object_Pose.sign_pose.pose.position.z = trans[2]
				object_Pose.sign_pose.pose.orientation.x = rot[0]
				object_Pose.sign_pose.pose.orientation.y = rot[1]
				object_Pose.sign_pose.pose.orientation.z = rot[2]
				object_Pose.sign_pose.pose.orientation.w = rot[3]
				print "Transform data added to message"
				print(object_Pose)
				
				if i in object_to_sign:
					object_Pose.sign_id = object_to_sign[i]
				else:
					object_Pose.sign_id = -1

				publisher.publish(object_Pose)
				print object_Frame

			except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				# rospy.logwarn("Object not in frame")
				continue
	
