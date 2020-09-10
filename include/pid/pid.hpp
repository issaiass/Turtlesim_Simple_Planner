/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : pid.hpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : PID Class
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
#include <geometry_msgs/TransformStamped.h>   // the transform message
#include <geometry_msgs/Twist.h>              // twist for cmd_vel message

/*
*******************************************************************************
*                                NAMESPACES
*******************************************************************************
*/

using namespace std;

/*
*******************************************************************************
*                                PID CLASS
*******************************************************************************
*/

class pid {
  public:
    // constructs the pid object
    pid(std::vector<double> &Kx, std::vector<double> &Kz);
    // compute the controller output
    geometry_msgs::Twist update(geometry_msgs::TransformStamped ts, double dt);
    // gets the error
    vector<double> getError(void);  
  private:
    vector<double> _Kx;     // Kp, Ki, Kd for distance controller
    vector<double> _Kz;     // Kp, Ki, Kd for angle controller
    vector<double> _error;  // error of distance and angle
};