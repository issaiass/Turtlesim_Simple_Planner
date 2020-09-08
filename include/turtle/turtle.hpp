#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/Kill.h>

using namespace std;            

class Turtle {
  public:
    Turtle(ros::NodeHandle &nh);      
    void spawn(turtlesim::Pose leader_pose, string name);
    void penOff(string name, bool off); 
    void kill(void);                  
    void kill(string name);           

  private:
    ros::NodeHandle &n;               
    ros::ServiceClient spawn_client;  
    ros::ServiceClient kill_client;   
    string turtle_name;               
};