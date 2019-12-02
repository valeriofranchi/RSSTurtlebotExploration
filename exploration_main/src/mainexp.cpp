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
#include <frontier_exploration/ExploreTaskFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <exploration_perception/DangerSign.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include "exp.hpp"
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
  signSubscriber = nh.subscribe<exploration_perception::DangerSign>("/danger_signs", 1, &TurtlebotExploration::signCB, this);
}

// definition of done callback. it is called once when the goal completes
void TurtlebotExploration::doneCB(const actionlib::SimpleClientGoalState& state) 
{
  ROS_INFO("[State Result]: %s", state.toString().c_str());
  ROS_INFO("The action has been completed");
  ros::shutdown();
}

// definition of the feedback callback. it will be called when the feedback is
// received from the action server. it just prints a message indicating a new
// message has been received

void TurtlebotExploration::moveBaseFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) 
{
  x = feedback->base_position.pose.position.x;
  y = feedback->base_position.pose.position.y;
  tf::Quaternion q(feedback->base_position.pose.orientation.x, feedback->base_position.pose.orientation.y, 
    feedback->base_position.pose.orientation.z, feedback->base_position.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("[Feedback] Robot Position: x: %f, y: %f, yaw: %f", x, y, yaw);
}

// definition of the feedback callback. it will be clled when the feedback is
// received from the action server. it just prints a message indicating a new
// message has been received
void TurtlebotExploration::explorationFeedbackCB(const frontier_exploration::ExploreTaskFeedbackConstPtr& feedback) 
{
  x = feedback->base_position.pose.position.x;
  y = feedback->base_position.pose.position.y;
  tf::Quaternion q(feedback->base_position.pose.orientation.x, feedback->base_position.pose.orientation.y, 
    feedback->base_position.pose.orientation.z, feedback->base_position.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("[Feedback] Robot Position: x: %f, y: %f, yaw: %f", x, y, yaw);
  frontier = feedback->next_frontier.pose;
  ROS_INFO("[Feedback] Target Frontier: x: %f, y: %f", frontier.position.x, frontier.position.y);
}

void TurtlebotExploration::signCB(const exploration_perception::DangerSignConstPtr& msg)
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

void TurtlebotExploration::performExploration() 
{
  //before exploration starts, make robot perform an initial 360 sweep of the area
  TurtlebotExploration::rotate360();

  //initialize action server 
  typedef actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> ExplorationClient;
  ExplorationClient explorationClient("/explore_server", true);

  //wait for the action server to come up
  while(!explorationClient.waitForServer())
  {
    ROS_INFO("Waiting for the explore_server action server to come up");            
  }      
    ROS_INFO("explore_server action server is up");

  //create goal to send to the action server
  frontier_exploration::ExploreTaskGoal goal;

  //define boundaries of exploration
  geometry_msgs::PolygonStamped polygonBoundary;
  geometry_msgs::Point32 point1, point2, point3, point4;
  point1.x, point1.y, point1.z = -5.0, -5.0, 0.0;
  point2.x, point2.y, point2.z = 5.0, -5.0, 0.0;
  point3.x, point3.y, point3.z = 5.0, 5.0, 0.0;
  point4.x, point4.y, point4.z = -5.0, 5.0, 0.0;

  std::vector<geometry_msgs::Point32> points;
  points.reserve(5);
  points.push_back(point1);
  points.push_back(point2);
  points.push_back(point3);
  points.push_back(point4);
  points.push_back(point1);

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
  geometry_msgs::Vector3 fscale;
  fscale.x, fscale.y, fscale.z = 0.35, 0.35, 0.35;
  frontierMarker.scale = fscale;
  std_msgs::Header fHeader;
  fHeader.frame_id = "map";
  frontierMarker.header = fHeader;
  std_msgs::ColorRGBA fcolour;
  fcolour.r, fcolour.g, fcolour.b, fcolour.a = 1.0, 0.0, 0.0, 1.0;
  frontierMarker.color = fcolour;
  frontierMarker.pose = frontier;

  //create sign marker
  visualization_msgs::Marker signMarker;
  signMarker.type = visualization_msgs::Marker::CUBE;
  signMarker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Vector3 sScale;
  sScale.x, sScale.y, sScale.z = 0.75, 0.75, 0.05;
  signMarker.scale = sScale;
  std_msgs::Header sHeader;
  sHeader.frame_id = "map";
  signMarker.header = sHeader;
  std_msgs::ColorRGBA sColour;
  sColour.r, sColour.g, sColour.b, sColour.a = 1.0, 0.0, 0.0, 1.0;
  signMarker.color = sColour;
  
  //send goal to action server 
  ROS_INFO("Sending goal to action server...");
  explorationClient.sendGoal(goal, boost::bind(&TurtlebotExploration::doneCB, this, _1), ExplorationClient::SimpleActiveCallback(),
    boost::bind(&TurtlebotExploration::explorationFeedbackCB, this, _1));
  
  //perform navigation
  ros::Rate rate(20);
  actionlib::SimpleClientGoalState stateResult = explorationClient.getState();
  ROS_INFO("[State Result]: %s", stateResult.toString().c_str());

  while ( stateResult == actionlib::SimpleClientGoalState::ACTIVE || 
    stateResult == actionlib::SimpleClientGoalState::PENDING )
  {
    ROS_INFO("Checking for danger signs while performing navigation...");

    //publish marker
    markerPublisher.publish(frontierMarker);

    if (signDetected)
    {
      signMarker.pose = dangerSign.sign_pose.pose;
      signDetected = false;
    }
    
    //update state value and display it on log
    stateResult = explorationClient.getState();
    ROS_INFO("[State Result]: %s", stateResult.toString().c_str());
    rate.sleep();
  }

  //wait for the server to finish performing the action
  explorationClient.waitForResult();
  ROS_INFO("Unknown Environment Explored! Success!");
}

void TurtlebotExploration::checkLocations(const std::vector<geometry_msgs::PoseStamped>& locations)
{
  //initialize action server 
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient mbClient("move_base", true);

  //wait for the action server to come up
  while(!mbClient.waitForServer())
  {
    ROS_INFO("Waiting for the move_base action server to come up");            
  }      
    ROS_INFO("move_base action server is up");

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
    geometry_msgs::Vector3 gScale;
    gScale.x, gScale.y, gScale.z = 0.35, 0.35, 0.35;
    goalMarker.scale = gScale;
    std_msgs::Header gHeader;
    gHeader.frame_id = "map";
    goalMarker.header = gHeader;
    std_msgs::ColorRGBA gColour;
    gColour.r, gColour.g, gColour.b, gColour.a = 0.0, 1.0, 0.0, 1.0;
    goalMarker.color = gColour;
    goalMarker.pose = (*constIter).pose;

    //create sign marker
    visualization_msgs::Marker signMarker;
    signMarker.type = visualization_msgs::Marker::CUBE;
    signMarker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 sScale;
    sScale.x, sScale.y, sScale.z = 0.75, 0.75, 0.05;
    signMarker.scale = sScale;
    std_msgs::Header sHeader;
    sHeader.frame_id = "map";
    signMarker.header = sHeader;
    std_msgs::ColorRGBA sColour;
    sColour.r, sColour.g, sColour.b, sColour.a = 1.0, 0.0, 0.0, 1.0;
    signMarker.color = sColour;

    //send goal to action server 
    ROS_INFO("Sending goal to action server...");
    mbClient.sendGoal(goal, boost::bind(&TurtlebotExploration::doneCB, this, _1), MoveBaseClient::SimpleActiveCallback(),
      boost::bind(&TurtlebotExploration::moveBaseFeedbackCB, this, _1));

    //perform navigation
    ros::Rate rate(20);
    actionlib::SimpleClientGoalState stateResult = mbClient.getState();
    ROS_INFO("[State Result]: %s", stateResult.toString().c_str());
      
    while ( stateResult == actionlib::SimpleClientGoalState::ACTIVE || 
      stateResult == actionlib::SimpleClientGoalState::PENDING )
    {
      ROS_INFO("Checking for danger signs while performing navigation...");

      //publish marker
      markerPublisher.publish(goalMarker);

      if (signDetected)
      {
        signMarker.pose = dangerSign.sign_pose.pose;
        signDetected = false;
      }
      
      //update state value and display it on log
      stateResult = mbClient.getState();
      ROS_INFO("[State Result]: %s", stateResult.toString().c_str());
      rate.sleep();
    }

    //wait for the server to finish performing the action
    mbClient.waitForResult();
    ROS_INFO("Location Reached! Success!");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "turtlebot_exploration_node");
  ros::NodeHandle nodeHandle;

  //create TurtlebotExploration object and perform exploration 
  TurtlebotExploration turtlebotExp(&nodeHandle);
  //turtlebotExp.performExploration();

  //save map
  std::string mapname = "map";
  int threshold_occupied = 65;
  int threshold_free = 25;

  nodeHandle.getParam("~map_name", mapname);

  MapGenerator mg(mapname, threshold_occupied, threshold_free);

  while(!mg.saved_map_ && ros::ok())
    ros::spinOnce();
  
  return 0;
}