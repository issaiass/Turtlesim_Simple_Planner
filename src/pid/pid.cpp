/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : pid.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : A simple PID controller class function.
*  ----------------------------------------------------------------------------
*  HISTORY
*  MM DD YY
*  09 26 20 Created.
*  09 28 20 Added comments
*  10 30 20 Documented
*******************************************************************************
*/


/*
*******************************************************************************
*                     NECCESARY HEADER FILES FOR THE APPLICATION
*******************************************************************************
*/

#include <pid/pid.hpp>

/*
*******************************************************************************
*                                   FUNCTIONS
*******************************************************************************
*/

/*
*******************************************************************************
*
*                  PID CONSTRUCTOR BY GIVING THE PID CONSTANTS
*
*
*  Description : Initializes the PID with Kp, Ki, Kd for angle and distance
*  Func. Name  : pid
*  Arguments   : std::vector<double> &Kx
*                    set the value of distances Kp, Ki, Kd constants
*                std::vector<double> &Kz
*                    set the value of angles Kp, Ki, Kd constants
*  Returns     : None
*  Notes       : Initialization lisf of the error as 0
*******************************************************************************
*/

pid::pid(std::vector<double> &Kx, std::vector<double> &Kz) :
_Kx(Kx),
_Kz(Kz),
_error({0,0})
{

}

/*
*******************************************************************************
*
*                  A FUNCTION TO UPDATE THE PID CONTROLLER
*
*
*  Description : Update controller values by giving the transform and delta time
*  Func. Name  : update
*  Arguments   : geometry_msgs::TransformStamped ts
*                    the transform between the current object and the goal
*                double dt
*                    value in seconds of the time elapsed to apply the PID
*  Returns     : geometry_msgs::Twist
*                    pose format to easy pass to the cmd_vel publisher
*  Notes       : None
*******************************************************************************
*/

geometry_msgs::Twist pid::update(geometry_msgs::TransformStamped ts, double dt) {
   double Kx_p = _Kx.at(0);
   double Kx_i = _Kx.at(1);
   double Kx_d = _Kx.at(2);
   
   double Kz_p = _Kz.at(0);
   double Kz_i = _Kz.at(1);
   double Kz_d = _Kz.at(2);

   double error_d = sqrt(pow(ts.transform.translation.x, 2) + pow(ts.transform.translation.y, 2));
   double error_a = atan2(ts.transform.translation.y, ts.transform.translation.x);

   static double prev_error_d;
   static double prev_error_a;
   double ierror_d, ierror_a;
   ierror_d += prev_error_d + error_d;
   ierror_a += prev_error_a + error_a;

   double derror_d = error_d - prev_error_d;
   double derror_a = error_d - prev_error_a;


   geometry_msgs::Twist controller;
   controller.linear.x =  Kx_p*error_d + Kx_i*error_d*dt + Kx_d*derror_d/dt;
   controller.angular.z = Kz_p*error_a + Kz_i*error_a*dt + Kz_d*derror_a/dt;   
   prev_error_d = error_d;
   prev_error_a = error_a;

   _error[0] = error_d;
   _error[1] = error_a;

   return controller;
}

/*
*******************************************************************************
*
*                  GET ERRORS FOR COMPARE END OF MOTION
*
*
*  Description : get back the error of distance and angle
*  Func. Name  : getError
*  Arguments   : None
*  Returns     : std::vector<double>
*                    val[0] = error in distance
*                    val[1] = error in angle
*  Notes       : None
*******************************************************************************
*/

std::vector<double> pid::getError(void) {
   return _error;
}