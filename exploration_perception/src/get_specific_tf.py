#! /usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

def tf_listener(data):
	global transform
	for transform in data.transforms:
		print(transform)
		if transform.child_frame_id[:6] == "object":
			publisher.publish(transform)

rospy.init_node("a_node")
subscriber = rospy.Subscriber("/tf", TFMessage, tf_listener)
publisher = rospy.Publisher("/tf_signs", TransformStamped, queue_size=1)
rospy.spin()
		

