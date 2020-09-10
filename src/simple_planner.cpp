/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : simple_planner.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : A simple controller to navigate under the simulator.
*                  Simple collision avoidance algorithm.
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

#include <turtle/turtle.hpp>                   // turtlesim library
#include <tf2_broadcaster/frame/tf2_frame.hpp> // the pose lookup
#include <pid/pid.hpp>                         // the PID library

/*
*******************************************************************************
*                              CONSTANT DEFINITIONS
*******************************************************************************
*/

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

/*
*******************************************************************************
*                            FUNCTION PROTOTYPES
*******************************************************************************
*/

double distance(geometry_msgs::TransformStamped ts);

/*
*******************************************************************************
*                                NAMESPACES
*******************************************************************************
*/

using namespace std;

/*
*******************************************************************************
*                                 MAIN PROGRAM
*******************************************************************************
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_planner"); 
  ros::NodeHandle n;                        // node handler
  vector<double> K_d{KDP, KDI, KDD};        // pid distance  
  vector<double> K_a{KAA, KAI, KAD};        // pid angle
  geometry_msgs::TransformStamped ts;       // for frame transformations
  geometry_msgs::Twist vel_msg;             // command to the turtlesim (msg)
  ros::Publisher turtle_vel;                // the publisher to turtlesim
  turtlesim::Pose turtle_pose;              // pose of this turtle
  TF2Frame fb;                              // frame broadcaster  
  double dt;                                // time delta for PID controller
  vector<double> error;                     // this is to know when to stop
  double d;                                 // variable to the eucledian distance

  if(argc != 5) {
    ROS_ERROR("Invalid number of parameters\nusage: rosrun turtlesim_simple_planner simple_planer \"child_frame_name x y yaw\"");
    return -1;
  }

  // get the name and the pose of the turtle
  std::string sdturtle = argv[1];
  turtle_pose.x = stof(argv[2]);             
  turtle_pose.y = stof(argv[3]);
  turtle_pose.theta = stof(argv[4]);

  // node handler to the turtle, spawns and set pen off
  Turtle turtle(n);
  turtle.spawn(turtle_pose, sdturtle);
  turtle.penOff(sdturtle, true);

  // Rate of loop as declared
  ros::Rate rate(SP_NODE_RATE);
                       
  // PID control for distance and angle
  pid controller(K_d, K_a);

  // subscribe this turtle to the cmd_vel publisher
  turtle_vel = n.advertise<geometry_msgs::Twist>(sdturtle+"/cmd_vel", SP_CMD_VEL_BFR);

  // get the end and start time (initialization)
  ros::Time start = ros::Time::now();
  ros::Time end = ros::Time::now() + ros::Duration(0.1);

  while(ros::ok()) {
    // move to goal
    ts = fb.lookup(sdturtle + "_goal", sdturtle);  // lookup the transform between turtle and goal
    end = ros::Time::now();                        // get finish time
    dt = end.toSec() - start.toSec();              // calculate delta
    dt = dt < (1.0/SP_NODE_RATE)*(0.05)?1.0/SP_NODE_RATE:dt; // if the dt is 95% below 0.1 clip it to 0.1
    vel_msg = controller.update(ts, dt);           // update the controller 'x' and 'theta'
    start = ros::Time::now();                      // start to contabilize time again
    error = controller.getError();                 // distance and angle errors 
    if (error[0] < 0.1 && error[2] < 0.1) {        // if threshold is below, then stop 
      vel_msg.angular.z = 0;
      vel_msg.linear.x = 0;
    }

    // avoid obstacle (list all turtles)
    vector<string> turtles{"turtle_lt", "turtle_rt", "turtle_up", "turtle_dn", "turtle1_fixed", "turtle_x", "turtle_y", "turtle_z", "turtle_w"};
    for(int i = 0; i < turtles.size(); i++) {     // for all turtles (including itself)
      string curr_turtle = turtles.operator[](i); // get the current turtle
      if (curr_turtle != sdturtle) {              // if i am not in the loop then proceed
        ts = fb.lookup(curr_turtle, sdturtle);    // look the transform between the other turtle
        d = distance(ts);                         // get the distance
        if (d < SP_OBS_MIN_D) {                   // if is below this minimal distance
          ts = fb.lookup(sdturtle, curr_turtle);  // get the transform
          vel_msg.angular.z += M_PI/d;            // add PI depending of the distance
          vel_msg.linear.x -= SP_MULT_D*d;        // move backwards a little to let pass the turtle
          cout << sdturtle << " near of " << curr_turtle << " at " << d << " meters." << endl;
          cout << "Changing diretion to " << vel_msg.angular.z << " rads." << endl;
          cout << "Changing speed to " << vel_msg.linear.x << " m/s." << endl;          
        }
      }
    }

    turtle_vel.publish(vel_msg);                  // publish the command
    ros::spinOnce();                              // loop until complete the rate
    rate.sleep();                                 // sleep

  }
  return 0;                                
}

/*
*******************************************************************************
*                                   FUNCTIONS
*******************************************************************************
*/

/*
*******************************************************************************
*
*                  A FUNCTION TO GET THE EUCLEDIAN DISTANCE
*
*
*  Description : Function that measure the eucledian distance between tf
*  Func. Name  : distance
*  Arguments   : geometry_msgs::TransformStamped:: ts
*                    a transform between two frames
*  Returns     : double
*                    the magnitude of the eucledian distance
*  Notes       : None
*******************************************************************************
*/

double distance(geometry_msgs::TransformStamped ts) {
   double d;

   d = sqrt(pow(ts.transform.translation.x,2) + pow(ts.transform.translation.y,2));
   return d;
}