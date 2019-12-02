#! /usr/bin/env python

import rospy
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from exploration_perception.msg import DangerSign

# def tf_listener(data):
# 	global transform
# 	for transform in data.transforms:
# 		print(transform)
# 		if transform.child_frame_id[:6] == "object":
# 			publisher.publish(transform)

# rospy.init_node("a_node")
# subscriber = rospy.Subscriber("/tf", TFMessage, tf_listener)
# publisher = rospy.Publisher("/tf_signs", TransformStamped, queue_size=1)
# rospy.spin()
		
if __name__ == '__main__':
	rospy.init_node("a_node")
	publisher = rospy.Publisher("/danger_signs", DangerSign, queue_size=5)

	listener = tf.TransformListener()
	
	while not rospy.is_shutdown():
		object_List_size = 55
		object_Name = "/object_"

		for i in range(object_List_size):
			
			object_Frame = object_Name + str(i)
			print object_Frame

			try:
				(trans,rot) = listener.lookupTransform(object_Frame,'/map',rospy.Time(0))
				#Convert i to object type
				object_Pose = exploration_perception.msg.DangerSign()
				object_Pose.sign_pose.pose.position = trans
				object_pose.sign_pose.pose.orientation = rot
				publisher.publish(object_Pose)


			except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				rospy.logwarn("Object not in frame")
				continue