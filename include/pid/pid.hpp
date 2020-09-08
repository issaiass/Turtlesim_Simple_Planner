#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class pid {
  public:
    pid(std::vector<double> &Kx, std::vector<double> &Kz);
    geometry_msgs::Twist update(geometry_msgs::TransformStamped ts, double dt);
    vector<double> getError(void);
  private:
    vector<double> _Kx;
    vector<double> _Kz; 
    vector<double> _error;
};