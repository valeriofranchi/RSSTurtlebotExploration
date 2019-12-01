#! /usr/bin/env python

from math import pi, fabs
import rospy
import actionlib
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from geometry_msgs.msg import PolygonStamped, PointStamped, Point32, Point
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header, ColorRGBA
from exploration_perception.msg import DangerSign
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

class TurtlebotExploration:

    def __init__(self):
        #Initializes node, publishers and subscribers 
        rospy.init_node('map_navigation')
        self.movement_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.marker_pub = rospy.Publisher('/rviz_marker', Marker, queue_size=1)
        self.sign_sub = rospy.Subscriber('/danger_signs', DangerSign, self.sign_callback) #modify with correct type 
        self.rate = rospy.Rate(20)

        #current robot pose coordinates updated by feedback callback function 
        self.pose = PoseStamped()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        #current target frontier to explore 
        self.frontier = PoseStamped()    

        #danger sign message and boolean describing if detection happened
        self.danger_sign = DangerSign()
        self.sign_detected = False

        #start exploration
        self.perform_exploration()
    
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
    This function will be called when feedback is received from the explore_server action server.
    It prints the current location of the robot based in the form of its x,y and yaw coordinates 
    and stores the location of the current frontier the robot is exploring.
    """
    def exploration_feedback_cb(self, feedback):
        self.pose = feedback.base_position
        self.x = feedback.base_position.pose.position.x
        self.y = feedback.base_position.pose.position.y
        c_or = feedback.base_position.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([c_or.x, c_or.y, c_or.z, c_or.w])
        self.frontier = feedback.next_frontier.pose
        rospy.loginfo("Feedback received")
        rospy.loginfo("Robot Position: x: {}, y: {}, yaw: {}".format(self.x, self.y, self.yaw))
        rospy.loginfo("Target Frontier: x: {}, y: {}".format(self.frontier.position.x, self.frontier.position.y))
    
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
    def sign_callback(self, data):
        self.danger_sign = data
        self.sign_detected = True

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
    Rotate robot 360 degrees on the spot. 
    """
    def rotate360(self):
        move = Twist()
        initial_yaw = self.yaw
        move.angular.z = 0.1

        #check when the 360 sweep has been finished 
        while True:
            self.movement_pub.publish(move)
            if (initial_yaw > 0.0 and self.yaw < 0.0) or (initial_yaw < 0.0 and self.yaw < initial_yaw):
                self.yaw += 2*pi
            difference = fabs(initial_yaw - self.yaw)
            if difference > 2*pi:
                break

        #stop the robot rotation
        move.angular.z = 0.0
        self.publish_once(move)

    """
    This function performs the turtlebot exploration of the unknown environment until no next frontier to explore is found.
    """
    def perform_exploration(self):
        #before the exploration starts, make robot perform an initial 360 sweep of the area 
        self.rotate360()
        
        #initialise action server and SimpleActionClient 
        exploration_server = '/explore_server'
        exploration_client = self.initialize_action_server(exploration_server, ExploreTaskAction)  

        #perform exploration
        goal = ExploreTaskGoal()

        #define boundaries of exploration 
        polygon_boundary = PolygonStamped()
        points = [Point32(x=-5.0, y=-5.0, z=0.0), Point32(x=5.0, y=-5.0, z=0.0), 
            Point32(x=5.0, y=5.0, z=0.0), Point32(x=-5.0, y=5.0, z=0.0), Point32(x=-5.0, y=-5.0, z=0.0)]
        polygon_boundary.polygon.points = points
        polygon_boundary.header.frame_id = 'map'
        polygon_boundary.header.stamp = rospy.Time.now()
        
        #centre point for frontier exploration, inside explore_boundary 
        centre_point = PointStamped()
        point = Point(x=0.0, y=0.0, z=0.0)
        centre_point.point = point
        centre_point.header.frame_id = 'map'
        centre_point.header.stamp = rospy.Time.now()

        goal.explore_boundary = polygon_boundary
        goal.explore_center = centre_point

        #Creates an rviz marker for the goal
        frontier_marker = Marker(type=Marker.CUBE, action=Marker.ADD, scale=Vector3(0.35, 0.35, 0.35),
            header=Header(frame_id='map'), color=ColorRGBA(1.0, 0.0, 0.0, 1.0))
        frontier_marker.pose = self.frontier

        # Sends the goal to the action server.
        rospy.loginfo('Sending goal to action server: %s', goal)
        exploration_client.send_goal(goal, feedback_cb=self.exploration_feedback_cb)

        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = exploration_client.get_state()

        rospy.loginfo("state_result: "+str(state_result))

        while state_result < DONE:
            rospy.loginfo("Checking for danger signs while performing exploration....")

            #self.exploration_client.cancel_goal()
            #print("Cancelling goal...")

            #publish rviz marker for frontier 
            self.marker_pub.publish(frontier_marker)

            if self.sign_detected:
                sign_marker = Marker(type=Marker.CUBE, action=Marker.ADD, scale=Vector3(0.75, 0.75, 0.05),
                    header=Header(frame_id='map'), color=ColorRGBA(1.0, 0.0, 0.0, 1.0))
                """PROBLEM ONE IS A POSE OTHER IS A TRANSFORM MESSAGE """
                sign_marker.pose = self.danger_sign.sign_pose.transform
                self.sign_detected = False
                
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
        rospy.loginfo("Frontier Reached! Success!")
    
    """
    put description, locations are PoseStamped messages
    """
    def check_locations(self, locations):
         #initialise action server and SimpleActionClient 
        movebase_server = '/move_base'
        movebase_client = self.initialize_action_server(movebase_server, MoveBaseAction)  

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

                #self.exploration_client.cancel_goal()
                #print("Cancelling goal...")

                #publish rviz marker for goal
                self.marker_pub.publish(goal_marker)

                if self.sign_detected:
                    sign_marker = Marker(type=Marker.CUBE, action=Marker.ADD, scale=Vector3(0.75, 0.75, 0.05),
                        header=Header(frame_id='map'), color=ColorRGBA(1.0, 0.0, 0.0, 1.0))
                    """PROBLEM ONE IS A POSE OTHER IS A TRANSFORM MESSAGE """
                    sign_marker.pose = self.danger_sign.sign_pose.transform
                    self.sign_detected = False
                
                #update state value and display it on the log 
                state_result = movebase_client.get_state()
                rospy.loginfo("state_result: "+str(state_result))
                self.rate.sleep()
            
            #wait for the server to finish performing the action
            movebase_client.wait_for_result()
            rospy.loginfo("Location Reached! Success!")


turtlebot_exploration = TurtlebotExploration()
