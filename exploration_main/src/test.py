#! /usr/bin/env python
import rospy
import tf

rospy.init_node("test")
listener = tf.TransformListener()

try:
    (trans, rot) = listener.lookupTransform("map", "base_footprint", rospy.Time(0))
except:
    rospy.logerr("problem with transform lookup")