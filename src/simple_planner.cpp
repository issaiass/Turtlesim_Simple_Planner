#include <turtle/turtle.hpp>
#include <tf2_broadcaster/frame/tf2_frame.hpp>
#include <pid/pid.hpp>
#include <geometry_msgs/Twist.h>

// Minimal obstacle distance and factor
#define SP_OBS_MIN_D 2.5
#define SP_MULT_D 0.6

// PID Distance
#define KDP 0.4 
#define KDI 0.38
#define KDD 0.056

// PID Angle
#define KAA 3.5
#define KAI 0.25
#define KAD 0.01

// Rate and Buffer
#define SP_NODE_RATE 10
#define SP_CMD_VEL_BFR 10

double distance(geometry_msgs::TransformStamped ts);

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_planner"); 
  ros::NodeHandle n;
  vector<double> K_d{KDP, KDI, KDD}; // pid distance  
  vector<double> K_a{KAA, KAI, KAD}; // pid angle
  geometry_msgs::TransformStamped ts;   
  geometry_msgs::Twist vel_msg;
  ros::Publisher turtle_vel;

  if(argc != 5) {
    ROS_ERROR("Invalid number of parameters\nusage: rosrun turtlesim_simple_planner simple_planer \"child_frame_name x y yaw\"");
    return -1;
  }

  std::string sdturtle = argv[1];
  turtlesim::Pose turtle_pose;               
  turtle_pose.x = stof(argv[2]);      
  turtle_pose.y = stof(argv[3]);
  turtle_pose.theta = stof(argv[4]);

  Turtle turtle(n);
  turtle.spawn(turtle_pose, sdturtle);
  turtle.penOff(sdturtle, true);
  ros::Rate rate(SP_NODE_RATE);
                       

  TF2Frame fb; 
  pid controller(K_d, K_a);

  turtle_vel = n.advertise<geometry_msgs::Twist>(sdturtle+"/cmd_vel", SP_CMD_VEL_BFR);


  double dt;
  ros::Time start = ros::Time::now();
  ros::Time end = ros::Time::now() + ros::Duration(0.1);

  while(ros::ok()) {
    
    // move to goal
    ts = fb.lookup(sdturtle + "_goal", sdturtle);

    end = ros::Time::now();
    dt = end.toSec() - start.toSec();
    dt = dt < (1.0/SP_NODE_RATE)*(0.05)?1.0/SP_NODE_RATE:dt; // if the dt is 95% below 0.1 clip to 0.1

    vel_msg = controller.update(ts, dt);
    
    start = ros::Time::now();

    vector<double> error = controller.getError();
    if (error[0] < 0.1 && error[2] < 0.1) { // if distance error and angle error 
      vel_msg.angular.z = 0;
      vel_msg.linear.x = 0;
    }

    // avoid obstacle
    vector<string> turtles{"turtle_lt", "turtle_rt", "turtle_up", "turtle_dn", "turtle1_fixed", "turtle_x", "turtle_y", "turtle_z", "turtle_w"};
    for(int i = 0; i < turtles.size(); i++) {     
      string curr_turtle = turtles.operator[](i);
      if (curr_turtle != sdturtle) {
        ts = fb.lookup(curr_turtle, sdturtle);
        double d = distance(ts); 
        if (d < SP_OBS_MIN_D) {
          ts = fb.lookup(sdturtle, curr_turtle);
          vel_msg.angular.z += M_PI/d;
          vel_msg.linear.x -= SP_MULT_D*d;
          cout << sdturtle << " near of " << curr_turtle << " at " << d << " meters." << endl;
          cout << "Changing diretion to " << vel_msg.angular.z << " rads." << endl;
          cout << "Changing speed to " << vel_msg.linear.x << " m/s." << endl;          
        }
      }
    }

    turtle_vel.publish(vel_msg);    
    ros::spinOnce();
    rate.sleep();

  }
  return 0;                                
}


double distance(geometry_msgs::TransformStamped ts) {
   double d;

   d = sqrt(pow(ts.transform.translation.x,2) + pow(ts.transform.translation.y,2));
   return d;
}
