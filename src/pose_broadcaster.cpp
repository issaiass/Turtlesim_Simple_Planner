/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : pose_broadcaster.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : It will broadcast the current turtle to the /tf topic
*  ----------------------------------------------------------------------------
*  HISTORY
*  MM DD YY
*  9 26 20 Created.
*  9 28 20 Added comments
*  9 30 20 Documented
*******************************************************************************
*/


/*
*******************************************************************************
*                     NECCESARY HEADER FILES FOR THE APPLICATION
*******************************************************************************
*/

#include <ros/ros.h>                           // ros
#include <tf2/LinearMath/Quaternion.h>         // quaternion math
#include <tf2_ros/transform_broadcaster.h>     // transform broadcaster
#include <geometry_msgs/TransformStamped.h>    // transform messages
#include <turtlesim/Pose.h>                    // pose

/*
*******************************************************************************
*                             GLOBAL VARIABLES
*******************************************************************************
*/

std::string turtle_name;                       // current turtle name


/*
*******************************************************************************
*                            FUNCTION PROTOTYPE
*******************************************************************************
*/

void poseCallback(const turtlesim::PoseConstPtr& msg);

/*
*******************************************************************************
*                                 MAIN PROGRAM
*******************************************************************************
*/

int main(int argc, char** argv){
  ros::init(argc, argv, "main");

  ros::NodeHandle private_node("~");
  if (! private_node.hasParam("sdturtle"))
  {
    if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    turtle_name = argv[1];
  }
  else
  {
    private_node.getParam("sdturtle", turtle_name); // get the name
  }

  // subscribe to the pose and broadcast by the callback function
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  ros::spin();
  return 0;
};

/*
*******************************************************************************
*                            FUNCTION DEFINITION
*******************************************************************************
*/

/*
*******************************************************************************
*
*          CALLBACK FUNCTION TO PUBLISH THE TRANSFORM TO /TF TOPIC
*
*
*  Description : It will get the pose of the current turtle and publish to /tf
*  Func. Name  : poseCallback
*  Arguments   : turtlesim::PoseConstPtr::  &mg
*                    a pointer that has the pose of the current turtle
*  Returns     : None
*  Notes       : Inside, the pose is published to the /tf topic
*******************************************************************************
*/

void poseCallback(const turtlesim::PoseConstPtr& msg){
  // we need a stamped transform and its broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  // a turtle with a name is published by its timestamp at x,y and quaternion values
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = turtle_name;
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // publish to /tf
  br.sendTransform(transformStamped);
}