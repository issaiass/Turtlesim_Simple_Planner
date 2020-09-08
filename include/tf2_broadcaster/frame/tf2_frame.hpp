#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Vector3.h>

using namespace std;  

class TF2Frame {
  public:
    TF2Frame();   
    virtual ~TF2Frame(); 
    geometry_msgs::TransformStamped  lookup(string frame1, string frame2);
  private:
    tf2_ros::Buffer tfBuffer;
    geometry_msgs::TransformStamped ts;
    tf2_ros::TransformListener tfListener;
};