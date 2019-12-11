#! /usr/bin/env python
import os
import time
import random
from math import fabs, pi
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
    This function will be called when feedback is received from the move base action server.
    It prints the current location of the robot based in the form of its x,y and yaw coordinates.
    """
    def movebase_feedback_cb(self, feedback):
        self.x = feedback.base_position.pose.position.x
        self.y = feedback.base_position.pose.position.y
        c_or = feedback.base_position.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([c_or.x, c_or.y, c_or.z, c_or.w])
        rospy.loginfo("Feedback received\nRobot Position: x: {}, y: {}, yaw: {}".format(self.x, self.y, self.yaw))
    
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
        rospy.loginfo("Calling exploration server and initialising action client...")
        #initialise action server and SimpleActionClient 
        exploration_server = '/exploration_server'
        exploration_client = self.initialize_action_server(exploration_server, ExplorationAction)  

        rospy.loginfo("Setting the goal for the frontier exploration...")
        #perform exploration
        goal = ExplorationGoal()

        #create the rviz marker for the signs 
        #sign_marker = Marker(type=Marker.CUBE, action=Marker.ADD, scale=Vector3(0.75, 0.75, 0.05),
        #    header=Header(frame_id='map'), color=ColorRGBA(1.0, 0.0, 0.0, 1.0))

        # Sends the goal to the action server.
        rospy.loginfo('Sending goal to action server: %s', goal)
        exploration_client.send_goal(goal)

        marker_array = MarkerArray()

        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = exploration_client.get_state()

        rospy.loginfo("state_result: "+str(state_result))

        while state_result < DONE:
            #rospy.loginfo("Checking for danger signs while performing exploration....")

            #if self.sign_detected:
            #    sign_marker.pose = self.danger_sign.sign_pose.pose
            #    self.signs.markers.append(sign_marker)
            #    marker_array.markers.append(sign_marker)
            #    self.sign_detected = False
            
            #publish rviz markers for boundary polygon, frontier, signs 
            self.marker_array_pub.publish(marker_array)
                
            #update state value and display it on the log 
            state_result = exploration_client.get_state()
            #rospy.loginfo("state_result: "+str(state_result))
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
    
    """
    put description, locations are PoseStamped messages
    """
    def check_locations(self, locations):
         #initialise action server and SimpleActionClient 
        movebase_server = '/move_base'
        movebase_client = self.initialize_action_server(movebase_server, MoveBaseAction)  

        #create marker array and add all signs to it 
        marker_array = MarkerArray()
        #marker_array.markers.extend(self.signs.markers)        

        for location in locations:
            # Creates a goal to send to the action server.
            goal = MoveBaseGoal()
            goal.target_pose.pose = location.pose
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            #create goal marker 
            goal_marker = Marker(type=Marker.CUBE, action=Marker.ADD, scale=Vector3(0.35, 0.35, 0.35),
            header=Header(frame_id='map'), color=ColorRGBA(0.0, 1.0, 0.0, 1.0))
            goal_marker.pose = location.pose

            #add goal marker to marker array
            marker_array.markers.append(goal_marker)

            #create the rviz marker for the signs 
            #sign_marker = Marker(type=Marker.CUBE, action=Marker.ADD, scale=Vector3(0.75, 0.75, 0.05),
            #    header=Header(frame_id='map'), color=ColorRGBA(1.0, 0.0, 0.0, 1.0))

            # Sends the goal to the action server.
            rospy.loginfo('Sending goal to action server: %s', goal)
            movebase_client.send_goal(goal, feedback_cb=self.movebase_feedback_cb)

            #perform navigation
            state_result = movebase_client.get_state()

            rospy.loginfo("state_result: "+str(state_result))
            if state_result == PENDING:
                rospy.loginfo("Action goal is pending...")
            if state_result == ACTIVE:
                rospy.loginfo("Action goal is being performed...")
                
            while state_result < DONE:
                rospy.loginfo("Checking for danger signs while performing navigation....")

                #if self.sign_detected:
                #    sign_marker.pose = self.danger_sign.sign_pose.pose
                #    marker_array.markers.append(sign_marker)
                #    self.sign_detected = False

                #publish rviz marker for goal
                self.marker_array_pub.publish(marker_array)

                #update state value and display it on the log 
                state_result = movebase_client.get_state()
                rospy.loginfo("state_result: "+str(state_result))
                self.rate.sleep()
            
            #wait for the server to finish performing the action
            movebase_client.wait_for_result()
            rospy.loginfo("Location Reached! Success!")

#create TurtlebotExploration object that initialises everything and start exploration
turtlebot_exploration = TurtlebotExploration()
turtlebot_exploration.perform_exploration()
"""
#save the map into a file
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
#configure and execute the launch file 
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/valeriofranchi/catkin_ws/src/exploration_main/launch/map_saver.launch"])
launch.start()

#define the map_saver generated file directories  
map_name = rospy.get_param("~map_name")
yaml_file_path = map_name + ".yaml"
pgm_file_path = map_name + ".pgm"
yaml_dir = os.path.join("home/valeriofranchi/catkin_ws/src/exploration_main", yaml_file_path)
pgm_dir = os.path.join("home/valeriofranchi/catkin_ws/src/exploration_main", pgm_file_path)

#if both files exist, wait for 3 seconds then stop the launch file 
if os.path.exists(yaml_dir) and os.path.exists(pgm_dir):
    time.sleep(3.0) 
    launch.shutdown()
"""