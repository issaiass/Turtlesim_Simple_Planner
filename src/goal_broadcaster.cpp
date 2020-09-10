/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : goal_broadcaster.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : It will broadcast the current turtle to the /tf_static topic
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


#include <tf2_broadcaster/static/tf2_static.hpp>

/*
*******************************************************************************
*                                 MAIN PROGRAM
*******************************************************************************
*/

int main(int argc, char **argv)
{
  std::string static_goal_name;

  ros::init(argc,argv, "goal_broadcaster_node");
  if(argc != 8) {
    ROS_ERROR("Invalid number of parameters\nusage: goal_broadcaster child_frame_name x y z roll pitch yaw");
    return -1;
  }
  if(strcmp(argv[1],"world")==0) {
    ROS_ERROR("Your static turtle name cannot be 'world'");
    return -1;
  }
  static_goal_name = argv[1];                        // the static frame name
  TF2SB sb("world", static_goal_name);               // initialize the staic broadcaster
  geometry_msgs::Vector3 tr = sb.tr2Vector3(argv);   // string to float
  geometry_msgs::Vector3 rot = sb.rot2Vector3(argv); // string to float
  sb.broadcast(tr, rot);                             // broadcast the transform
  ROS_INFO("Spinning until killed publishing %s to world", static_goal_name.c_str());
  ros::spin();
  return 0;
};