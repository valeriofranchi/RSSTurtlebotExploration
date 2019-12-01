#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/ExploreTaskGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <exploration_perception/DangerSign.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include "exploration.hpp"
#include "map_saver.cpp"

// function initialises the publishers for the TurtlebotExploration Class
void TurtlebotExploration::initialisePublishers() 
{
    velocityPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    markerPublisher = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
}

// function initialises the subscribers for the TurtlebotExploration Class
void TurtlebotExploration::initialiseSubscribers() 
{
    signSubscriber = nh.subscribe<exploration_perception::DangerSign>("/danger_signs", 1, this);
}

// definition of done callback. it is called once when the goal completes
void TurtlebotExploration::doneCB(const actionlib::SimpleClientGoalState &state) 
{
  ROS_INFO("[State Result]: %s", state.toString().c_str());
  ROS_INFO("The action has been completed");
  ros::shutdown();
}

// definition of the active callback. it is called once when the goal becomes
// active
void TurtlebotExploration::activeCB() { ROS_INFO("goal just went active"); }

// definition of the feedback callback. it will be called when the feedback is
// received from the action server. it just prints a message indicating a new
// message has been received
void TurtlebotExploration::moveBaseFeedbackCB(const geometry_msgs::PoseStamped::ConstPtr& feedback) 
{
  x = feedback->pose.position.x;
  y = feedback->pose.position.y;
  tf::Quaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y, 
    feedback->pose.orientation.z, feedback->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("[Feedback] Robot Position: x: %f, y: %f, yaw: %f", x, y, yaw);
}

// definition of the feedback callback. it will be clled when the feedback is
// received from the action server. it just prints a message indicating a new
// message has been received
void TurtlebotExploration::explorationFeedbackCB(const geometry_msgs::PoseStamped::ConstPtr& nextFrontier,
  const geometry_msgs::PoseStamped::ConstPtr& basePosition) 
{
  x = basePosition->pose.position.x;
  y = basePosition->pose.position.y;
  tf::Quaternion q(basePosition->pose.orientation.x, basePosition->pose.orientation.y, 
    basePosition->pose.orientation.z, basePosition->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("[Feedback] Robot Position: x: %f, y: %f, yaw: %f", x, y, yaw);
  frontier = nextFrontier->pose;
  ROS_INFO("[Feedback] Target Frontier: x: %f, y: %f", frontier.position.x, frontier.position.y);
}

void TurtlebotExploration::signCB(const exploration_perception::DangerSign::ConstPtr& msg)
{
  dangerSign = *msg;
  signDetected = true;
}

void TurtlebotExploration::publishOnce(const geometry_msgs::Twist& vel) 
{
  ros::Rate rate(20);
  while (ros::ok()) {
    int numConnections = velocityPublisher.getNumSubscribers();
    if (numConnections > 0) {
      velocityPublisher.publish(vel);
      break;
    } else {
      rate.sleep();
    }
  }
}

void TurtlebotExploration::rotate360()
{
  geometry_msgs::Twist move;
  double init_yaw = yaw;
  move.angular.z = 0.1;

  //check when 360 sweep has finished
  while (1)
  {
    velocityPublisher.publish(move);
    if ((init_yaw > 0.0 && yaw < 0.0) || (init_yaw < 0.0 && yaw < init_yaw))
      yaw += 2 * M_PI;
    double difference = std::abs(init_yaw - yaw);
    if (difference > 2 * M_PI) 
      break;
  }

  //stop robot
  move.angular.z = 0.0;
  velocityPublisher.publish(move);
}

void publish_once_twist(const ros::Publisher& publisher, const geometry_msgs::Twist& vel) 
{
  ros::Rate rate(20);
  while (ros::ok()) {
    int numConnections = publisher.getNumSubscribers();
    if (numConnections > 0) {
      publisher.publish(vel);
      break;
    } else {
      rate.sleep();
    }
  }
}

void TurtlebotExploration::performExploration() 
{
  //before exploration starts, make robot perform an initial 360 sweep of the area
  rotate360();

  //initialize action server 
  std::string serverName = "/explore_server";
  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> client(serverName, true);
  client.waitForServer();
  ROS_INFO("Action Server Found... %s", serverName);

  //create goal to send to the action server
  frontier_exploration::ExploreTaskGoal goal;

  //define boundaries of exploration
  geometry_msgs::PolygonStamped polygonBoundary;
  geometry_msgs::Point32 point1, point2, point3, point4;
  point1.x, point1.y, point1.z = -5.0, -5.0, 0.0;
  point2.x, point2.y, point2.z = 5.0, -5.0, 0.0;
  point3.x, point3.y, point3.z = 5.0, 5.0, 0.0;
  point4.x, point4.y, point4.z = -5.0, 5.0, 0.0;
  std::vector<geometry_msgs::Point32> points{point1, point2, point3, point4, point1};
  polygonBoundary.polygon.points = points;
  polygonBoundary.header.frame_id = "map";
  polygonBoundary.header.stamp = ros::Time::now();
  
  //centre point for exploration
  geometry_msgs::PointStamped centrePoint;
  geometry_msgs::Point point;
  point.x, point.y, point.z = 0.0, 0.0, 0.0;
  centrePoint.point = point;
  centrePoint.header.frame_id = "map";
  centrePoint.header.stamp = ros::Time::now();

  goal.explore_boundary = polygonBoundary;
  goal.explore_center = centrePoint;

  //create goal marker
  visualization_msgs::Marker frontierMarker;
  frontierMarker.type = visualization_msgs::Marker::CUBE;
  frontierMarker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Vector3 scale;
  scale.x, scale.y, scale.z = 0.35, 0.35, 0.35;
  frontierMarker.scale = scale;
  std_msgs::Header header;
  header.frame_id = "map";
  frontierMarker.header = header;
  std_msgs::ColorRGBA color;
  color.r, color.g, color.b, color.a = 1.0, 0.0, 0.0, 1.0;
  frontierMarker.color = color;
  frontierMarker.pose = frontier;

  //create sign marker
  visualization_msgs::Marker signMarker;
  signMarker.type = visualization_msgs::Marker::CUBE;
  signMarker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Vector3 scale;
  scale.x, scale.y, scale.z = 0.75, 0.75, 0.05;
  signMarker.scale = scale;
  std_msgs::Header header;
  header.frame_id = "map";
  signMarker.header = header;
  std_msgs::ColorRGBA color;
  color.r, color.g, color.b, color.a = 1.0, 0.0, 0.0, 1.0;
  signMarker.color = color;

  //send goal to action server 
  ROS_INFO("Sending goal to action server: %s", goal);
  client.sendGoal(goal, &doneCB, &activeCB, &explorationFeedbackCB);

  //perform navigation
  ros::Rate rate(20);
  actionlib::SimpleClientGoalState stateResult = client.getState();
  ROS_INFO("[State Result]: %s", stateResult.toString().c_str());

  while ( stateResult == actionlib::SimpleClientGoalState::ACTIVE || 
    stateResult == actionlib::SimpleClientGoalState::PENDING )
  {
    ROS_INFO("Checking for danger signs while performing navigation...");
    
    //ROS_INFO("Canceling goal...");
    //client.cancelGoal();
    //ROS_INFO("Goal canceled...");

    //publish marker
    markerPublisher.publish(frontierMarker);

    if (signDetected)
    {
      //PROBLEM
      //signMarker.pose = dangerSign.sign_pose.transform;
      signDetected = false;
    }
    
    //update state value and display it on log
    stateResult = client.getState();
    ROS_INFO("[State Result]: %s", stateResult.toString().c_str());
    rate.sleep();
  }

  //wait for the server to finish performing the action
  client.waitForResult();
  ROS_INFO("Unknown Environment Explored! Success!");
}

void TurtlebotExploration::checkLocations(const std::vector<geometry_msgs::PoseStamped>& locations)
{
  //initialize action server 
  std::string serverName = "/move_base";
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client(serverName, true);
  client.waitForServer();
  ROS_INFO("Action Server Found... %s", serverName);

  std::vector<geometry_msgs::PoseStamped>::const_iterator constIter;
  for(constIter = locations.begin(); constIter != locations.end(); ++constIter)
  { 
    //create goal to send to the action server
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose = (*constIter).pose;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    //create goal marker
    visualization_msgs::Marker goalMarker;
    goalMarker.type = visualization_msgs::Marker::CUBE;
    goalMarker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 scale;
    scale.x, scale.y, scale.z = 0.35, 0.35, 0.35;
    goalMarker.scale = scale;
    std_msgs::Header header;
    header.frame_id = "map";
    goalMarker.header = header;
    std_msgs::ColorRGBA color;
    color.r, color.g, color.b, color.a = 0.0, 1.0, 0.0, 1.0;
    goalMarker.color = color;
    goalMarker.pose = (*constIter).pose;

    //create sign marker
    visualization_msgs::Marker signMarker;
    signMarker.type = visualization_msgs::Marker::CUBE;
    signMarker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 scale;
    scale.x, scale.y, scale.z = 0.75, 0.75, 0.05;
    signMarker.scale = scale;
    std_msgs::Header header;
    header.frame_id = "map";
    signMarker.header = header;
    std_msgs::ColorRGBA color;
    color.r, color.g, color.b, color.a = 1.0, 0.0, 0.0, 1.0;
    signMarker.color = color;

    //send goal to action server 
    ROS_INFO("Sending goal to action server: %s", goal);
    client.sendGoal(goal, &doneCB, &activeCB, &moveBaseFeedbackCB);

    //perform navigation
    ros::Rate rate(20);
    actionlib::SimpleClientGoalState stateResult = client.getState();
    ROS_INFO("[State Result]: %s", stateResult.toString().c_str());
      
    while ( stateResult == actionlib::SimpleClientGoalState::ACTIVE || 
      stateResult == actionlib::SimpleClientGoalState::PENDING )
    {
      ROS_INFO("Checking for danger signs while performing navigation...");
      
      //ROS_INFO("Canceling goal...");
      //client.cancelGoal();
      //ROS_INFO("Goal canceled...");

      //publish marker
      markerPublisher.publish(goalMarker);

      if (signDetected)
      {
        //PROBLEM
        //signMarker.pose = dangerSign.sign_pose.transform;
        signDetected = false;
      }
      
      //update state value and display it on log
      stateResult = client.getState();
      ROS_INFO("[State Result]: %s", stateResult.toString().c_str());
      rate.sleep();
    }

    //wait for the server to finish performing the action
    client.waitForResult();
    ROS_INFO("Location Reached! Success!");
  }
}

int main(int argc, char **argv) 
{
  // initialize node and node handler
  ros::init(argc, argv, "drone_action_client");
  ros::NodeHandle nh;

  //create TurtlebotExploration object and perform exploration 
  TurtlebotExploration::TurtlebotExploration turtlebotExploration(nh);
  turtlebotExploration.performExploration();



  return 0;
}