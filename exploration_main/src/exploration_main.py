# ! /usr/bin/env python
import os
import time
import random
from math import fabs, pi, sqrt, pow
import tf
import actionlib
import roslaunch 
import rospy
from exploration_control.msg import ExplorationAction, ExplorationGoal
from geometry_msgs.msg import PoseStamped, Twist, Pose, Vector3, PointStamped, Point32, Point, PolygonStamped, Transform
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, MapMetaData, OccupancyGrid
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import subprocess
from exploration_perception.msg import DangerSign

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

class TurtlebotExploration:

    def __init__(self):
        # Initializes node, publishers and subscribers 
        rospy.init_node('map_navigation')
        self.sign_sub = rospy.Subscriber('/danger_signs', DangerSign, self.sign_callback) 
        self.rate = rospy.Rate(20)

        # Current robot pose coordinates updated by feedback callback function 
        self.pose = PoseStamped()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Danger sign message and boolean describing if detection happened
        self.danger_sign = DangerSign()
        self.sign_detected = False
    
    """
    This function initializes the explore_server action server and creates an action client object
    """
    def initialize_action_server(self, action_server_name, action):
        # Creates the SimpleActionClient
        client = actionlib.SimpleActionClient(action_server_name, action)
        # Waits until the action server has started up 
        rospy.loginfo('Waiting for action Server ' + action_server_name)
        client.wait_for_server()
        rospy.loginfo('Action Server Found...' + action_server_name)
        # Return SimpleActionClient
        return client
    
    """
    Subscriber to the '/danger_signs' topic; when a sign is detected it stores the DangerSign object and assigns the 
    boolean sign_detected a True value.
    """
    def sign_callback(self, msg):
        self.danger_sign = msg
        self.sign_detected = True
        rospy.logwarn("SIGN CALLBACK: sign detected")

    """
    This function calculates the euclidean distance between two poses
    """
    def euclidean_distance(self, pose1, pose2):
        return sqrt(pow(pose2.position.x - pose1.position.x, 2) + pow(pose2.position.y - pose1.position.y, 2))

    """
    This function performs the turtlebot exploration of the unknown environment until no next frontier to explore is found.
    """
    def perform_exploration(self):
        rospy.loginfo("Waiting for find_object_2d node to load...")
        rospy.sleep(5)

        # Initialise action server and SimpleActionClient 
        rospy.loginfo("Calling exploration server and initialising action client...")
        exploration_server = '/exploration_server'
        exploration_client = self.initialize_action_server(exploration_server, ExplorationAction)  

        # Perform the exploration
        rospy.loginfo("Setting the goal for the frontier exploration...")
        goal = ExplorationGoal()

        # Sends the goal to the action server.
        rospy.loginfo('Sending goal to action server: %s', goal)
        exploration_client.send_goal(goal)

        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = exploration_client.get_state()
        rospy.loginfo("state_result: "+str(state_result))

        # Initialise MarkerArray publisher and MarkerArray object
        marker_array_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
        signs = MarkerArray()
        
        # Performs object detection while exploration is underway
        while state_result < DONE:
            rospy.loginfo("Checking for workers and danger signs while performing exploration...")

            # Check if sign has been detected
            if self.sign_detected == True:
                # Check if detected sign is a new one or the same as one of the already detected and recognized ones 
                if len(signs.markers) == 0 or not any([self.euclidean_distance(i.pose, self.danger_sign.sign_pose.pose) < 0.20 for i in signs.markers]):
                    # Create the rviz marker for the signs and append it to MarkerArray object
                    signs.markers.append(Marker(type=Marker.TEXT_VIEW_FACING, scale=Vector3(1.0, 1.0, 0.2), text=self.danger_sign.sign_name.data,
                        header=Header(frame_id='map'), color=ColorRGBA(1.0, 0.0, 0.0, 1.0), pose=self.danger_sign.sign_pose.pose))
                self.sign_detected = False

            # Assign ID's to every marker 
            id = 0
            for sign in signs.markers:
                sign.header.stamp = rospy.Time.now()
                sign.id = id
                id += 1 
            
            # Publish MarkerArray and print to log the number of unique markers detected up until now 
            marker_array_pub.publish(signs)
            rospy.logwarn("Number of markers: {}".format(len(signs.markers)))
                
            # Update state value and display it on the log 
            state_result = exploration_client.get_state()
            rospy.loginfo("state_result: "+str(state_result))
            self.rate.sleep()

        rospy.loginfo("[Result] State: "+str(state_result))
        if state_result == ERROR:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == WARN:
            rospy.logwarn("There is a warning in the Server Side")
        
        rospy.loginfo("Area Explored! Success!")
    

# Create TurtlebotExploration object that initialises everything and start exploration
turtlebot_exploration = TurtlebotExploration()
turtlebot_exploration.perform_exploration()

# Launch the map saver node 
file_path = rospy.get_param("~bash_file")
subprocess.call(file_path)
