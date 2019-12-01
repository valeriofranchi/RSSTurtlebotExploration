#ifndef POTENTIAL_FIELD_ROS_HEADER
#define POTENTIAL_FIELD_ROS_HEADER

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <vector>

class PotentialField {
private:
    //NodeHandle, Publishers and Subscriber Nodes 
    ros::NodeHandle nh;
    ros::Publisher velocityPublisher;
    ros::Publisher markerArrayPublisher;
    ros::Subscriber odomSubscriber;
    ros::Subscriber laserSubscriber;
    ros::Subscriber goalSubscriber;

    //real-time x,y and theta coordinates for the robot odometry 
    double x, y, theta;

    //variable used to find out when robot odometry has started being published
    int odomCounter = 0;

    //odometry pose of the robot in real time
    geometry_msgs::Pose odomPose;

    //array where robot laser range finder values are to be stored
    std::vector<float> laserReadings;

    //goal pose coordinates and boolean specifying if goal was published
    double goalX, goalY;
    bool goalPublished;
    geometry_msgs::Pose goalPose;

    //array where distances from goal are stored along the path 
    std::vector<double> distancesToGoal;

public:
    //constructor for the PotentialField class with nodehandle variable 
    //whose only job is initializing the subscribers and publishers 
    PotentialField(ros::NodeHandle nh_): nh(nh_) {
        initializePublishers();
        initializeSubscribers();
    } 

    //initializers for publishers and subscribers and subscriber callbacks 
    void initializePublishers();
    void initializeSubscribers();
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr&);
    void odomCB(const nav_msgs::Odometry::ConstPtr&);
    void laserCB(const sensor_msgs::LaserScan::ConstPtr&);

    //function to publish only once, usually when publish is called outside while loop
    void publishOnce(geometry_msgs::Twist);

    //potential field function returning a vector of values used for path planning
    std::vector<double> potentialField(double, double, double);

    //euclidean distance between robot current position and a point in space
    double euclideanDistance(double, double);

    //PID-controlled angular velocity of the robot 
    double angularVelocity(double, double);

    //PID controller for the generated path plan from the potential field 
    void PIDcontroller(double, double, double, double);

    //gradient descent algorithm for the path planning 
    void gradientDescentWithPID(double, double, double, double);

    //function to get robot out of local minimas of the potential field 
    void escapeLocalMinima();  

};

#endif