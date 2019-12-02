#ifndef EXAMPLE_ROS_CLASS_H_
#define EXAMPLE_ROS_CLASS_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <vector>
#include <frontier_exploration/ExploreTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <exploration_perception/DangerSign.h>

class TurtlebotExploration {
private:
    //NodeHandle, Publishers and Subscriber Nodes 
    ros::NodeHandle nh;
    ros::Publisher velocityPublisher;
    ros::Publisher markerPublisher;
    ros::Subscriber signSubscriber;

    //real-time x,y and theta coordinates and pose 
    double x, y, yaw;
    geometry_msgs::PoseStamped pose;

    //current target frontier to explore
    geometry_msgs::Pose frontier;

    //DangerSign message and boolean indicating whether sign has beend detected
    exploration_perception::DangerSign dangerSign;
    bool signDetected;

public:
    //constructor for the PotentialField class with nodehandle variable 
    //whose only job is initializing the subscribers and publishers 
    TurtlebotExploration(ros::NodeHandle* nh_): nh(*nh_) {
        initialisePublishers();
        initialiseSubscribers();
        signDetected = false;
    } 

    //initializers for publishers, subscribers, subscriber callbacks and action clients
    void initialisePublishers();
    void initialiseSubscribers();
    void signCB(const exploration_perception::DangerSignConstPtr&);

    //function to publish only once, usually when publish is called outside while loop
    void publishOnce(const geometry_msgs::Twist&);

    //function to rotate robot on its z axis 360 degrees
    void rotate360();

    //function to perform exploration
    void performExploration();
    
    //function to explore tagged locations during exploration
    void checkLocations(const std::vector<geometry_msgs::PoseStamped>&);

    //function to initialise the action server and action client functions
    void moveBaseFeedbackCB(const geometry_msgs::PoseStampedConstPtr&);
    void explorationFeedbackCB(const geometry_msgs::PoseStampedConstPtr&, const geometry_msgs::PoseStampedConstPtr&);
    void doneCB(const actionlib::SimpleClientGoalState&);    
    void activeCB();
};

#endif 