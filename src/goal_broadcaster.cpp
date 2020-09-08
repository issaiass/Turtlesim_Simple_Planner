#include <tf2_broadcaster/static/tf2_static.hpp>

std::string static_goal_name;

int main(int argc, char **argv)
{
  ros::init(argc,argv, "goal_broadcaster_node");
  if(argc != 8) {
    ROS_ERROR("Invalid number of parameters\nusage: goal_broadcaster child_frame_name x y z roll pitch yaw");
    return -1;
  }
  if(strcmp(argv[1],"world")==0) {
    ROS_ERROR("Your static turtle name cannot be 'world'");
    return -1;
  }
  static_goal_name = argv[1];
  TF2SB sb("world", static_goal_name);
  geometry_msgs::Vector3 tr = sb.tr2Vector3(argv);
  geometry_msgs::Vector3 rot = sb.rot2Vector3(argv);
  sb.broadcast(tr, rot);
  ROS_INFO("Spinning until killed publishing %s to world", static_goal_name.c_str());
  ros::spin();
  return 0;
};