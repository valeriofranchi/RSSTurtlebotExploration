#! /usr/bin/env python
import rospy
import actionlib
import exploration_control.msg
import control

class ExplorationServer(object):
    # Create messages that are used to publish feedback/result
    _feedback = exploration_control.msg.ExplorationFeedback()
    _result = exploration_control.msg.ExplorationResult()

    def __init__(self, name):
        self._action_name = name
        # Create SimpleActionServer passing it a name, an action type and an execute callback
        self._as = actionlib.SimpleActionServer(self._action_name, exploration_control.msg.ExplorationAction, execute_cb=self.execute_cb, auto_start = False)
        # Start the action server
        self._as.start()
    
    def execute_cb(self, goal):
        # Execute exploration
        control.main()
        # Set goal as succeeded and publish it on the log 
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    # Main method that creates the action server and spinds the node 
    rospy.init_node('exploration_server')
    server = ExplorationServer(rospy.get_name())
    rospy.spin()