/*
*******************************************************************************
*                     NECCESARY HEADER FILES FOR THE APPLICATION
*******************************************************************************
*/

#include <tf2_broadcaster/static/tf2_static.hpp>


TF2SB::TF2SB(string parent, string child) :
  _parent(parent), 
  _child(child) 
{ 

}


void TF2SB::setFrame(string parent, string child)
{
  _parent = parent;
  _child = child;
}


void TF2SB::broadcast(geometry_msgs::Vector3 tr, geometry_msgs::Vector3 rot) {
  _static_transformStamped.header.stamp = ros::Time::now();
  _static_transformStamped.header.frame_id = _parent;
  _static_transformStamped.child_frame_id = _child;
  _static_transformStamped.transform.translation.x = tr.x;
  _static_transformStamped.transform.translation.y = tr.y;
  _static_transformStamped.transform.translation.z = tr.z;
  tf2::Quaternion q;
  q.setRPY(rot.x, rot.y, rot.z);
  _static_transformStamped.transform.rotation.x = q.x();
  _static_transformStamped.transform.rotation.y = q.y();
  _static_transformStamped.transform.rotation.z = q.z();
  _static_transformStamped.transform.rotation.w = q.w();
  _static_broadcaster.sendTransform(_static_transformStamped);
}

geometry_msgs::Vector3 TF2SB::tr2Vector3(char **argv) {
  geometry_msgs::Vector3 tr;


  tr.x = atof(argv[2]);
  tr.y = atof(argv[3]);
  tr.z = atof(argv[4]);
  return tr;
}

geometry_msgs::Vector3 TF2SB::rot2Vector3(char **argv) {
  geometry_msgs::Vector3 rot;


  rot.x = atof(argv[5]);
  rot.y = atof(argv[6]);
  rot.z = atof(argv[7]);
  return rot;
}