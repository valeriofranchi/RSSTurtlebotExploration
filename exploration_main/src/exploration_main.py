#! /usr/bin/env python
import os
import time
import random
from math import fabs, pi
import tf
import actionlib
import roslaunch 
import rospy
from exploration_control.msg import ExplorationAction, ExplorationGoal
from geometry_msgs.msg import PoseStamped, Twist, Pose, Vector3, PointStamped, Point32, Point, PolygonStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, MapMetaData, OccupancyGrid
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import subprocess
#from exploration_perception.msg import DangerSign

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

class TurtlebotExploration:

    def __init__(self):
        #Initializes node, publishers and subscribers 
        rospy.init_node('map_navigation')
        #self.sign_sub = rospy.Subscriber('/danger_signs', DangerSign, self.sign_callback) 
        self.marker_array_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
        self.rate = rospy.Rate(20)

        #current robot pose coordinates updated by feedback callback function 
        self.pose = PoseStamped()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        #danger sign message and boolean describing if detection happened
        #self.danger_sign = DangerSign()
        #self.sign_detected = False
        #self.signs = MarkerArray()
    
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
        #return SimpleActionClient
        return client
    
    """
    Subscriber to the '/danger_signs' topic; when a sign is detected it stores the DangerSign object and assigns the 
    boolean sign_detected a True value.
    """
    #def sign_callback(self, msg):
    #    self.danger_sign = msg
    #    self.sign_detected = True

    """
    This function performs the turtlebot exploration of the unknown environment until no next frontier to explore is found.
    """
    def perform_exploration(self):
        #rospy.loginfo("Waiting for find_object_2d node to load...")
        #rospy.sleep(20)

        #initialise action server and SimpleActionClient 
        rospy.loginfo("Calling exploration server and initialising action client...")
        exploration_server = '/exploration_server'
        exploration_client = self.initialize_action_server(exploration_server, ExplorationAction)  

        rospy.loginfo("Setting the goal for the frontier exploration...")
        #perform exploration
        goal = ExplorationGoal()

        #create the rviz marker for the signs 
        #sign_marker = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, scale=Vector3(0.75, 0.75, 0.05),
        #    header=Header(frame_id='map'), color=ColorRGBA(1.0, 0.0, 0.0, 1.0))

        # Sends the goal to the action server.
        rospy.loginfo('Sending goal to action server: %s', goal)
        exploration_client.send_goal(goal)

        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = exploration_client.get_state()

        rospy.loginfo("state_result: "+str(state_result))

        #listener = tf.TransformListener()
        #object_pose = Pose()

        while state_result < DONE:
            rospy.loginfo("Checking for DEAD PEOPLE while performing exploration....")

            #if self.sign_detected:
            #    sign_marker.pose = self.danger_sign.sign_pose.pose
            #    sign_marker.text = self.danger_sign.sign_name 
            #    self.signs.markers.append(sign_marker)
            #    self.sign_detected = False

            #you should be able to get the transform from map to the object if tf is publishing
            #with synchronized clocks else i need to concatenate the two transforms using matrices 

            """try:
                (trans, rot) = listener.lookupTransform("map", "camera_rgb_optical_frame", rospy.Time(0))
                object_pose.position = trans
                object_pose.orientation = rot
            except:
                (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)
                continue"""
            
            #publish rviz markers for boundary polygon, frontier, signs 
            #self.marker_array_pub.publish(self.signs)
                
            #update state value and display it on the log 
            state_result = exploration_client.get_state()
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
          exploration_client.wait_for_result()
        rospy.loginfo("Area Explored! Success!")
    

#create TurtlebotExploration object that initialises everything and start exploration
turtlebot_exploration = TurtlebotExploration()
turtlebot_exploration.perform_exploration()

#launch the map saver node 
file_path = rospy.get_param("~bash_file")
subprocess.call(file_path)
