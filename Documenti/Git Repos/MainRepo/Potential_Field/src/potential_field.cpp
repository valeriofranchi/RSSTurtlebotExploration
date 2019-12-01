#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <vector>

#include "potential_field.hpp"


void PotentialField::initializePublishers() 
{
    velocityPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    markerArrayPublisher = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
}

void PotentialField::initializeSubscribers() 
{
    odomSubscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 1, this);
    laserSubscriber = nh.subscribe<sensor_msgs::LaserScan>("/kobuki/laser/scan", 1, this);
    goalSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/visualization_marker_array", 1, this);
}

void PotentialField::goalCB(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    goalX = msg->pose.position.x;
    goalY = msg->pose.position.y;
    goalPublished = true;
    ROS_INFO("Goal has been published => x: %f, y: %f", goalX, goalY);
}

void PotentialField::laserCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserReadings = msg->ranges;
    ROS_INFO("Laser Range Finder has started publishing...");
}

void PotentialField::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    //update x, y and yaw robot odometry values 
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta);
   
   //check if odometry has started publishing and print appropriate messages 
    if (odomCounter == 0)
        ROS_INFO("Odometry has started publishing...");
        ROS_INFO("Initial robot pose => x: %f, y: %f, yaw: %f", x, y, theta);
     ++odomCounter;
}

void PotentialField::publishOnce(geometry_msgs::Twist vel)
{
    ros::Rate loop_rate(20);
    while (ros::ok()) 
    {
        int connections = velocityPublisher.getNumSubscribers();
        if (connections > 0) {
            velocityPublisher.publish(vel);
            break;
        }
        loop_rate.sleep();
    }
}

std::vector<double> PotentialField::potentialField(double xi, double eta, double distanceOfInfluence)
{
    double attractivePotentialX = xi * (x - goalX);
    double attractivePotentialY =  xi * (y - goalY);
    double repulsivePotentialX(0.0);
    double repulsivePotentialY(0.0);

    for (std::vector<float>::const_iterator it = laserReadings.begin() ; it != laserReadings.end(); ++it) {
        if (*it <= 30.0) {
            double obstacleAngle = ((it - laserReadings.begin()) - 360) * M_PI / 720;
            double distanceToObstacleX = x + *it * cos(theta + obstacleAngle);
            double distanceToObstacleY = y + *it * sin(theta + obstacleAngle);

            repulsivePotentialX += eta * (1 / *it - 1 / distanceOfInfluence) * (1 / std::pow(*it, 3)) * (x - distanceToObstacleX);
            repulsivePotentialY += eta * (1 / *it - 1 / distanceOfInfluence) * (1 / std::pow(*it, 3)) * (y - distanceToObstacleY);
        }
    }

    double potentialFieldX = attractivePotentialX + repulsivePotentialX;
    double potentialFieldY = attractivePotentialY + repulsivePotentialY;
    double magnitude = std::sqrt(std::pow(potentialFieldX, 2) + std::pow(potentialFieldY, 2));
    double direction = std::atan2(potentialFieldX, potentialFieldY);

    std::vector<double> result = {potentialFieldX, potentialFieldY, magnitude, direction};
    return result;  
}

double PotentialField::euclideanDistance(double targetX, double targetY) 
{
    return std::sqrt(std::pow(targetX - x, 2) + std::pow(targetY -y, 2));
}

double PotentialField::angularVelocity(double targetTheta, double constant = 2.0)
{
    double angularVelocity = std::abs(targetTheta - theta);
    if (theta > targetTheta)
        angularVelocity = -1 * angularVelocity;
    return constant * angularVelocity;
}

void PotentialField::PIDcontroller(double targetX, double targetY, double targetTheta, double distanceTolerance)
{
    geometry_msgs::Twist vel;
    ros::Rate loop_rate(20);

    visualization_msgs::Marker stepGoalMarker;
    stepGoalMarker.type = visualization_msgs::Marker::CUBE;
    stepGoalMarker.action = visualization_msgs::Marker::ADD;
    stepGoalMarker.scale.x = 0.2;
    stepGoalMarker.scale.y = 0.2;
    stepGoalMarker.scale.z = 0.2;
    stepGoalMarker.header.frame_id = "odom";
    stepGoalMarker.color.r = 1.0;
    stepGoalMarker.color.g = 0.0;
    stepGoalMarker.color.b = 0.0;
    stepGoalMarker.color.a = 0.8;
    stepGoalMarker.pose.position.x = x;
    stepGoalMarker.pose.position.y = y;

    visualization_msgs::Marker vectorMarker;
    stepGoalMarker.type = visualization_msgs::Marker::ARROW;
    stepGoalMarker.action = visualization_msgs::Marker::ADD;
    stepGoalMarker.scale.x = 0.75;
    stepGoalMarker.scale.y = 0.2;
    stepGoalMarker.scale.z = 0.2;
    stepGoalMarker.header.frame_id = "odom";
    stepGoalMarker.color.r = 0.0;
    stepGoalMarker.color.g = 1.0;
    stepGoalMarker.color.b = 0.0;
    stepGoalMarker.color.a = 0.8;
    stepGoalMarker.pose = odomPose;
    stepGoalMarker.pose.orientation.z = theta;

    visualization_msgs::Marker finalGoalMarker;
    stepGoalMarker.type = visualization_msgs::Marker::SPHERE;
    stepGoalMarker.action = visualization_msgs::Marker::ADD;
    stepGoalMarker.scale.x = 0.2;
    stepGoalMarker.scale.y = 0.2;
    stepGoalMarker.scale.z = 0.2;
    stepGoalMarker.header.frame_id = "odom";
    stepGoalMarker.color.r = 0.0;
    stepGoalMarker.color.g = 0.0;
    stepGoalMarker.color.b = 1.0;
    stepGoalMarker.color.a = 0.8;
    stepGoalMarker.pose.position.x = goalX;
    stepGoalMarker.pose.position.y = goalY;

    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.push_back(stepGoalMarker);
    markerArray.markers.push_back(vectorMarker);
    markerArray.markers.push_back(finalGoalMarker);

    int id = 0;
    for (int i = 0; i != markerArray.markers.size; ++i) {
        markerArray.markers.at(i).id = id;
        ++i;
    }

    int count = 0;
    double initialDistance = euclideanDistance(targetX, targetY);
    while ((euclideanDistance(targetX, targetY) >= distanceTolerance) && 
        (euclideanDistance(targetX, targetY) <= initialDistance))
    {
        distancesToGoal.push_back(euclideanDistance(goalX, goalY));
        markerArrayPublisher.publish(markerArray);

        vel.linear.x = 0.3;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;

        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = angularVelocity(targetTheta);

        velocityPublisher.publish(vel);

        ++count;

        if (distancesToGoal.size() > 20) {
            double distanceTravelled = std::abs(distancesToGoal.at(distancesToGoal.size() - 1)
                - distancesToGoal.at(distancesToGoal.size() - 21));
            if ((distanceTravelled < 0.05) && (euclideanDistance(goalX, goalY) > 1.0)) {
                escapeLocalMinima();
            }
        loop_rate.sleep();
        }
    }
}

void PotentialField::gradientDescentWithPID(double step=0.0075, double xi=20.0, 
    double eta=0.9, double distanceOfInfluence=75.0)
{
    geometry_msgs::Twist vel;
    int i = 0;
    double stepGoalX;
    double stepGoalY;

    while (ros::ok()) 
    {
        if (goalPublished == true) {
            std::vector<double> potentialFieldVector(potentialField(xi, eta, distanceOfInfluence));
            odomPose.orientation.z = potentialFieldVector.at(3);

            double stepGoalTheta = 0.0;

            if (i == 0) {
                stepGoalX = x;
                stepGoalY = y;
            }

            double distanceToGoal = std::sqrt(std::pow(goalX - stepGoalX, 2) + std::pow(goalY - stepGoalY, 2));

            if (distanceToGoal > 0.25) {
                stepGoalX += step * (potentialFieldVector.at(0) / potentialFieldVector.at(2));
                stepGoalY += step * (potentialFieldVector.at(1) / potentialFieldVector.at(2));
                stepGoalTheta = potentialFieldVector.at(3);
            } else {
                goalPublished = false;
                continue;
            }
            ++i;

            PIDcontroller(stepGoalX, stepGoalY, stepGoalTheta, 0.175);

            if (euclideanDistance(stepGoalX, stepGoalY) > 0.4) 
                i = 0;

        }  else {
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            velocityPublisher.publish(vel);
            distancesToGoal.clear();
        }
    }
    ros::spin();
}

void PotentialField::escapeLocalMinima() 
{
    geometry_msgs::Twist vel;
    while (laserReadings.at(laserReadings.size() / 2) <= 30.0) 
    {
        vel.angular.z = 5.0;
        velocityPublisher.publish(vel);
        distancesToGoal.clear();
    }

    vel.angular.z = 0.0;
    publishOnce(vel);

    ros::Time start = ros::Time::now();
    while (ros::Time::now() < (start + ros::Duration(4))) 
    {
        vel.linear.x = 0.5;
        velocityPublisher.publish(vel);
    }
    vel.angular.z = 0.0;
    publishOnce(vel);
}

int main(int argc, char** argv) 
{
    ros::NodeHandle nodeHandle;
    ros::init(argc, argv, "potential_field_node");
    PotentialField potentialField(nodeHandle);
    potentialField.gradientDescentWithPID(0.0075, 20.0, 0.9, 75.0);
    return 0;
}