#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Vector3.h>

using namespace std;  

class TF2SB {
  public:
    TF2SB(string parent, string child);      
    void setFrame(string parent, string child);
    void broadcast(geometry_msgs::Vector3 tr, geometry_msgs::Vector3 rot);
    geometry_msgs::Vector3 tr2Vector3(char **argv);
    geometry_msgs::Vector3 rot2Vector3(char **argv);
  private:
    tf2_ros::StaticTransformBroadcaster _static_broadcaster;
    geometry_msgs::TransformStamped _static_transformStamped;
    string _parent;
    string _child;
};