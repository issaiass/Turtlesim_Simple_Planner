/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : tf2_static.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : TF2SB Class (Transform Broadcaster Class Static).
*  ----------------------------------------------------------------------------
*  HISTORY
*  MM DD YY
*  9 26 20 Created.
*  9 28 20 Added comments
*  9 31 20 Documented
*******************************************************************************
*/

/*
*******************************************************************************
*                     NECCESARY HEADER FILES FOR THE APPLICATION
*******************************************************************************
*/     

#include <ros/ros.h>                                // ros
#include <tf2_ros/static_transform_broadcaster.h>   // the static broadcast
#include <geometry_msgs/TransformStamped.h>         // stamped transform
#include <tf2/LinearMath/Quaternion.h>              // quaternion compute
#include <geometry_msgs/Vector3.h>                  // for vector3 manipulation


/*
*******************************************************************************
*                                NAMESPACES
*******************************************************************************
*/

using namespace std;

/*
*******************************************************************************
*                               TF2SB CLASS
*******************************************************************************
*/

class TF2SB {
  public:
    // constructor by parend and child nodes
    TF2SB(string parent, string child);     
    // set the frame (same as constructor for changing the frame in real time) 
    void setFrame(string parent, string child);
    // message broadcast
    void broadcast(geometry_msgs::Vector3 tr, geometry_msgs::Vector3 rot);
    // conversion of arguments vectors
    geometry_msgs::Vector3 tr2Vector3(char **argv);
    geometry_msgs::Vector3 rot2Vector3(char **argv);
  private:
    // static broadcaster
    tf2_ros::StaticTransformBroadcaster _static_broadcaster;
    // stamped message
    geometry_msgs::TransformStamped _static_transformStamped;
    // parent frame
    string _parent;
    // child frame
    string _child;
};