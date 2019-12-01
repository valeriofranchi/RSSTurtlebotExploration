#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion
from math import pow, pi, cos, sin, sqrt, atan2, fabs
import numpy as np

class PotentialField():
    def __init__(self):
        #initialize node
        rospy.init_node("obstacle_avoidance_turtlebot")

        #initialize subscribers and publishers
        self.laser_subscriber = rospy.Subscriber("/kobuki/laser/scan", LaserScan, self.laser_cb)
        self.odom_subscriber = rospy.Subscriber("odom", Odometry, self.odom_cb)
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.goal_subscriber = rospy.Subscriber("/potential_field_goal", PoseStamped, self.goal_cb)
        self.marker_array_publisher = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)

        #initialize robot odometry variables 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_pose = geometry_msgs.msg.Pose()
        self.odom_counter = 0

        #initialize robot laser variables 
        self.laser_readings = []

        #initialize goal pose variables 
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_pose = geometry_msgs.msg.Pose()
        self.goal_published = False

        #loop rate in Hz
        self.rate = rospy.Rate(20)

        #initialize variable used for escaping local minima 
        self.distances_to_goal = []
        
    #subscriber to topic in which rviz 2D Nav Goal /potential_field_goal publishes into 
    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_published = True
        rospy.loginfo('Goal has been published => x: %f, y: %f' %(self.goal_x, self.goal_y))

    #Hokuyo laser range finder subscriber 
    def laser_cb(self, msg):
        self.laser_readings = msg.ranges

    #robot odometry subscriber; in order to not have spammy odometry messages, only the first is printed 
    #to make the code user aware that the robot started publishing the odometry 
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y  = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([orientation.x, orientation.y,
            orientation.z, orientation.w])
        self.odom_pose = msg.pose.pose
        self.odom_counter += 1
        if self.odom_counter == 1:
            rospy.loginfo('Initial robot pose => x: %f, y: %f, yaw: %f' %(self.x, self.y, self.theta))
            rospy.loginfo('Odometry is being published...')
    
    #the function waits for at least for one subscriber to be active to publish a message before publishing 
    #it is useful when only one publish call is done usually outside a while loop 
    def publish_once(self, rotate):
        while not rospy.is_shutdown():
            connections = self.publisher.get_num_connections()
            if connections > 0:
                self.publisher.publish(rotate)
                break
            self.rate.sleep()
    
    #returns the potential field vector (magnitude, direction and potential field in x and y directions)
    def potential_field(self, xi , eta, distance_of_influence):
        attractive_potential = -xi * np.asarray([self.x - self.goal_x, self.y - self.goal_y])
        repulsive_potential = 0.0
        
        for i, reading in enumerate(self.laser_readings):
            if reading <= 30.0:
                obstacle_angle = (i - 360) * pi / 720
                q_obstacle_x = self.x + reading * cos(self.theta + obstacle_angle)
                q_obstacle_y = self.y + reading * sin(self.theta + obstacle_angle)
                if reading <= distance_of_influence:
                    repulsive_potential += eta * (1 / reading - 1 / distance_of_influence) * (1 / pow(reading, 3)) * np.asarray([self.x - q_obstacle_x, self.y - q_obstacle_y])
                elif reading > distance_of_influence:
                    repulsive_potential += 0

        potential_field = attractive_potential + repulsive_potential
        magnitude = sqrt(pow(potential_field[0], 2) + pow(potential_field[1], 2))
        direction = atan2(potential_field[1], potential_field[0])
        #rospy.loginfo("The potential field in the x direction is %f, while in the y direction is %f" %(potential_field[0], potential_field[1]))
        #rospy.loginfo("The magnitude of the potential field vector is %s while the direction in degrees (0-360) is %s" % (magnitude, (direction + 2*pi) * 180 / pi if direction < 0 else direction * 180 / pi))
        return (potential_field[0], potential_field[1], magnitude, direction)

    #euclidean distance from current position to a certain (x,y) position
    def euclidean_distance(self, x, y):
        return sqrt(pow((x - self.x), 2) + pow((y - self.y), 2))

    #PID-controlled angular velocity
    def angular_velocity(self, theta, constant=2):
        angular_vel = fabs(theta - self.theta)
        if self.theta > theta:
            angular_vel = -angular_vel
        return constant * angular_vel

    #Proportional Integral Differential Controller to get to a certain pose 
    def PID_controller(self, x, y, theta, distance_tolerance):
        vel = Twist()

        #create an rviz marker for the goal position and set the pose 
        step_goal_marker = Marker(type=Marker.CUBE, action=Marker.ADD, scale=Vector3(0.2, 0.2, 0.2),
            header=Header(frame_id='odom'), color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
        step_goal_marker.pose.position.x = x
        step_goal_marker.pose.position.y = y
        #rospy.loginfo("step x is %f and step y is %f" %(x, y))

        #create an rviz marker for the potential field vector and set the direction
        vector_marker = Marker(type=Marker.ARROW, action=Marker.ADD, scale=Vector3(0.75, 0.2, 0.2),
            header=Header(frame_id='odom'), color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self.odom_pose.orientation.z = theta
        vector_marker.pose = self.odom_pose

        #create an rviz marker for the final goal position 
        final_goal_marker = Marker(type=Marker.SPHERE, action=Marker.ADD, scale=Vector3(0.2, 0.2, 0.2),
            header=Header(frame_id='odom'), color=ColorRGBA(0.0, 0.0, 1.0, 0.8))
        final_goal_marker.pose.position.x = self.goal_x
        final_goal_marker.pose.position.y = self.goal_y
                
        #create rviz marker array and append the three rviz markers showing potential field vector, step and final goal position
        marker_array = MarkerArray()
        marker_array.markers.append(step_goal_marker)
        marker_array.markers.append(vector_marker)
        marker_array.markers.append(final_goal_marker)

        #number the markers
        id = 0
        for m in marker_array.markers:
            m.id = id
            id +=1 
        
        count = 0
        initial_distance = self.euclidean_distance(x, y)
        while (self.euclidean_distance(x, y) >= distance_tolerance) and (self.euclidean_distance(x, y) <= initial_distance):
            self.distances_to_goal.append(self.euclidean_distance(self.goal_x, self.goal_y))

            #publish the marker array 
            self.marker_array_publisher.publish(marker_array)

            #rospy.loginfo("entering pid controller loop...")
            #rospy.loginfo("next goal in x: %f and y: %f" %(x, y))
            #rospy.loginfo("current x: %f, current y: %f" % (self.x, self.y))
            #rospy.loginfo("distance to next goal point is %f and distance tolerance is %f" % (self.euclidean_distance(x, y), distance_tolerance))
            #linear velocity in x axis
            vel.linear.x = 0.3
            vel.linear.y = 0
            vel.linear.z = 0

            #angular velocitu in z axis
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = self.angular_velocity(theta)

            #publish velocity command
            self.publisher.publish(vel)

            #increase count
            count += 1

            #analyze the last 10 
            if len(self.distances_to_goal) > 20:
                distance_travelled = fabs(self.distances_to_goal[-1] - self.distances_to_goal[-21])
                if distance_travelled < 0.05 and self.euclidean_distance(self.goal_x, self.goal_y) > 1.0:
                    self.escape_local_minima()

            self.rate.sleep()

    #potential field gradient descent algorithm with PID-controlled speeds
    def gradient_descent_with_PID(self, step=0.0075, xi=20.0, eta=0.9, distance_of_influence=0.85):
        vel = Twist()
        i = 0       
        while not rospy.is_shutdown():
            if self.goal_published == True:
                #generate potential field 
                (f_x, f_y, magnitude, direction) = self.potential_field(xi, eta, distance_of_influence)
                self.odom_pose.orientation.z = direction

                #if it is the first iteration, set the next goal as robot starting position
                if i == 0:
                    q_x = self.x
                    q_y = self.y

                q_theta = 0.0
                distance_to_goal = sqrt(pow((self.goal_x - q_x), 2) + pow((self.goal_y - q_y), 2))

                #rospy.loginfo("distance from current location to final goal is %f" % distance_to_goal)
                if distance_to_goal > 0.25: #add some tolerance
                    q_x += step * (f_x / magnitude)
                    q_y += step * (f_y / magnitude)
                    q_theta = direction
                else:
                    self.goal_published = False
                    continue
                i += 1

                #Proportional Integral Derivative (PID) controller to drive the robot through the
                #generated path plan
                #rospy.loginfo("new step goal point generated...")
                self.PID_controller(q_x, q_y, q_theta, 0.175)

                if self.euclidean_distance(q_x, q_y) > 0.4:
                    i = 0

            else:
                #stop robot once goal is reached                 
                vel.linear.x = 0
                vel.angular.z = 0
                self.publisher.publish(vel)
                self.distances_to_goal = []

        #when Ctrl-C, the node gets killed, else a new goal can be published and the process can re-begin
        rospy.spin()

    #simple function to escape local minima by moving in the direction of no obstacle and then giving back 
    #control to the PID controller
    def escape_local_minima(self):
        #rotate until no obstacle is detected in the middle value of the laser range finder array then move for 4 seconds straight ahead
        vel = Twist()
        while self.laser_readings[len(self.laser_readings) / 2] <= 10.0:
            vel.angular.z = 5.0
            self.publisher.publish(vel)
            self.distances_to_goal = []

        vel.angular.z = 0.0
        self.publish_once(vel)

        start = rospy.Time().now()
        while rospy.Time().now() < (start + rospy.Duration(4)):
            vel.linear.x = 0.5
            self.publisher.publish(vel)
        vel.angular.z = 0.0
        self.publish_once(vel)   
        #once the function has finished executing, the control is given back to the PID and the potential field algorithm 

            
if __name__ == "__main__":
    pf = PotentialField()
    pf.gradient_descent_with_PID(xi=0.15, eta=0.0095, distance_of_influence=4.0)
