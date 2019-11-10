#! /usr/bin/env python

import rospy
import actionlib
import geometry_msgs
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header, ColorRGBA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, fabs
from visualization_msgs.msg import Marker
from frontier_exploration.srv import GetNextFrontier, GetNextFrontierRequest

class TurtlebotExploration:

    def __init__(self):
        #Initializes node, publishers and subscribers 
        rospy.init_node('map_navigation')
        self.movement_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.marker_pub = rospy.Publisher('rviz_marker', Marker, queue_size=1)
        self.rate = rospy.Rate(20)

        #variable describing if there is a new frontier to explore or not
        self.next_frontier = True

        #current robot pose coordinates updated by feedback callback function 
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0

        #initialize move_base action server 
        self.move_base_client = None 
        self.initialize_action_server()

        #initialize frontier_exploration service server 
        self.frontier_service = None
        self.initialize_service_server()        

        #start exploration
        self.perform_exploration()
    
    """
    This function initializes the move_base action server and creates an action client object
    """
    def initialize_action_server(self):
        # Creates the SimpleActionClient
        action_server_name = '/move_base'
        self.move_base_client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        # Waits until the action server has started up 
        rospy.loginfo('Waiting for action Server ' + action_server_name)
        self.move_base_client.wait_for_server()
        rospy.loginfo('Action Server Found...' + action_server_name)
    
    """
    This function initializes the frontier_exploration service server and creates a service client object
    """
    def initialize_service_server(self):
        service_name = '/get_next_frontier'
        #wait for service to be running
        rospy.loginfo('Waiting for service ' + service_name)
        rospy.wait_for_service("/get_next_frontier")
        rospy.loginfo('Service Found...' + service_name)
        #create a connection to the service
        self.frontier_service = rospy.ServiceProxy("/get_next_frontier", GetNextFrontier)

    """
    This function will be called when feedback is received from the action server.
    It prints the current location of the robot based on the x,y and theta coordinates.
    """
    def feedback_callback(self, feedback):
        self.curr_x = feedback.base_position.pose.position.x
        self.curr_y = feedback.base_position.pose.position.y
        c_or = feedback.base_position.pose.orientation
        (_, _, self.curr_yaw) = euler_from_quaternion([c_or.x, c_or.y, c_or.z, c_or.w])
        rospy.loginfo('Feedback received => Current Robot Position: x = %f, y = %f, theta = %f', self.curr_x, self.curr_y, self.curr_yaw)

    """
    This function waits for at least for one subscriber to be active before publishing.
    It is useful when only one publish call is performed (usually) outside a while loop 
    """
    def publish_once(self, rotate):
        while not rospy.is_shutdown():
            connections = self.movement_pub.get_num_connections()
            if connections > 0:
                self.movement_pub.publish(rotate)
                break
            self.rate.sleep()
    """
    This function performs the turtlebot exploration of the unknown environment until no next frontier to explore is found.
    """
    def perform_exploration(self):
		#while a new frontier to explore exists, continue with the navigation goals
        while self.next_frontier:
            #request to service a new frontier for exploration 
            frontier_request = GetNextFrontierRequest()
            result = self.frontier_service(frontier_request)
		
            #retrieve x y and yaw coordinates for next exploration goal 
            new_x, new_y = result.pose.position.x, result.pose.position.y
            new_orient = result.pose.orientation
            (_, _, new_yaw) = euler_from_quaternion([new_orient.x, new_orient.y, new_orient.z, new_orient.w])

            #send new pose to move_base action server 
            self.send_goal(new_x, new_y, new_yaw)
            rospy.loginfo("Sending goal to main algorithm -- x:%f, y:%f, theta:%f", new_x ,new_y, new_yaw)

        rospy.loginfo("Turtlebot exploration has finished...")
        #display results of exploration 

    def send_goal(self, x, y, yaw):
        # Creates a goal to send to the action server.
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.orientation = geometry_msgs.msg.Quaternion(*q)
        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        #Creates an rviz marker for the goal
        robot_marker = Marker(type=Marker.ARROW, action=Marker.ADD, scale=Vector3(0.9, 0.05, 0.1),
            header=Header(frame_id='map'), color=ColorRGBA(1.0, 0.0, 0.0, 1.0))
        robot_marker.pose = pose

        # Sends the goal to the action server.
        rospy.loginfo('Sending goal to action server: %s', goal)
        self.move_base_client.send_goal(goal, feedback_cb=self.feedback_callback)

        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = self.move_base_client.get_state()

        rospy.loginfo("state_result: "+str(state_result))

        # We create some constants with the corresponing vaules from the SimpleGoalState class
        PENDING = 0
        ACTIVE = 1
        DONE = 2
        WARN = 3
        ERROR = 4

        rospy.loginfo("[Result] State: "+str(state_result))
        if state_result == PENDING:
            rospy.loginfo("Action goal is pending...")
        if state_result == ACTIVE:
            rospy.loginfo("Action goal is being performed...")

        while state_result < DONE:
            rospy.loginfo("Checking for danger signs while performing exploration....")

            #self.move_base_client.cancel_goal()
            #print "Cancelling goal..."
            #rospy.loginfo("Sending goal to main algorithm -- x:%f, y:%f, theta:%f",x,y,yaw)

            #publish rviz marker for goal
            self.marker_pub.publish(robot_marker)

            #update state value and display it on the log 
            state_result = self.move_base_client.get_state()
            rospy.loginfo("state_result: "+str(state_result))
            self.rate.sleep()

        rospy.loginfo("[Result] State: "+str(state_result))
        if state_result == ERROR:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == WARN:
            rospy.logwarn("There is a warning in the Server Side")
        else:
          # Waits for the server to finish performing the action.
          print 'Waiting for result...'
          self.move_base_client.wait_for_result()
        rospy.loginfo("Frontier Reached! Success!")
		
        #make the robot make a slow 360 degree sweep to update the map 
        move = Twist()
        initial_yaw = self.curr_yaw
        move.angular.z = 0.1

        #check when the 360 sweep has been finished 
        while True:
            self.movement_pub.publish(move)
            if (initial_yaw > 0.0 and self.curr_yaw < 0.0) or (initial_yaw < 0.0 and self.curr_yaw < initial_yaw):
                self.curr_yaw += 2*pi
            difference = fabs(initial_yaw - self.curr_yaw)
            if difference > 2*pi:
                break

        #stop the robot rotation
        move.angular.z = 0.0
        self.publish_once(move)

        return self.move_base_client.get_result()


turtlebot_exploration = TurtlebotExploration()
