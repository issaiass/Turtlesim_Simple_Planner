/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : tf2_frame.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : TF2FB Class (Transform Broadcaster Class).
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

#include <ros/ros.h>                          // ros
#include <tf2_ros/transform_listener.h>       // the listener of transform
#include <geometry_msgs/TransformStamped.h>   // the message of transform
#include <tf2/LinearMath/Quaternion.h>        // quaternion computation
#include <geometry_msgs/Vector3.h>            // Vector3 format


/*
*******************************************************************************
*                                NAMESPACES
*******************************************************************************
*/

using namespace std;

/*
*******************************************************************************
*                             TF2FRAME CLASS
*******************************************************************************
*/

class TF2Frame {
  public:
    // constructs the frame
    TF2Frame();   
    // virtual function to elimnate the object
    virtual ~TF2Frame(); 
    // look over the transform between source (frame2) and destination (frame1)
    geometry_msgs::TransformStamped  lookup(string frame1, string frame2);
  private:
    tf2_ros::Buffer tfBuffer;                // buffer for the listener
    geometry_msgs::TransformStamped ts;      // timestamped transform
    tf2_ros::TransformListener tfListener;   // listener for the transform
};