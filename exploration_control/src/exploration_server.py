#! /usr/bin/env python

import rospy
import actionlib
import exploration_control.msg
import control

class ExplorationServer(object):
    # create messages that are used to publish feedback/result
    _feedback = exploration_control.msg.ExplorationFeedback()
    _result = exploration_control.msg.ExplorationResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, exploration_control.msg.ExplorationAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def execute_cb(self, goal):
        #execute exploration
        control.main()
        #success
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('exploration_server')
    server = ExplorationServer(rospy.get_name())
    rospy.spin()