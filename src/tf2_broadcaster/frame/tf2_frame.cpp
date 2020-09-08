#include <tf2_broadcaster/frame/tf2_frame.hpp>

TF2Frame::TF2Frame() : tfListener(tfBuffer)
{ 

}

TF2Frame::~TF2Frame() { }

geometry_msgs::TransformStamped  TF2Frame::lookup(string frame1, string frame2) {
   try {
      ts = tfBuffer.lookupTransform(frame2, frame1, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    } 
    return ts;
}