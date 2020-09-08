#include <pid/pid.hpp>


pid::pid(std::vector<double> &Kx, std::vector<double> &Kz) :
_Kx(Kx),
_Kz(Kz),
_error({0,0})
{

}

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

std::vector<double> pid::getError(void) {
   return _error;
}