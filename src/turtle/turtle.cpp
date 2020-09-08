/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : turtle.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : Simple functions of turtlesime resumed
*  ----------------------------------------------------------------------------
*  HISTORY
*  MM DD YY
*  09 06 20 Created.
*******************************************************************************
*/

/*
*******************************************************************************
*                     NECCESARY HEADER FILES FOR THE APPLICATION
*******************************************************************************
*/

#include <turtle/turtle.hpp>

/*
*******************************************************************************
*                              SUPPORT FUNCTIONS
*******************************************************************************
*/

/*
*******************************************************************************
*
*              CLASS CONSTRUCTOR TO SPAWN A TURTLE AND DISABLE PEN
*
*  Description : 
*  Func. Name  : Turtle (Constructor)
*  Arguments   : ros::NodeHandle &n
*                  it handles the services calls
*  Returns     : None
*  Notes       : Set pen off
*******************************************************************************
*/

Turtle::Turtle(ros::NodeHandle &nh) : n(nh) { 
  // initialization list nh with n
  spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
}

/*
*******************************************************************************
*
*                       SPAWN A NEW TURTLE ON THE FIELD
*
*  Description : Function to spawn turtles on the field given th pose and name 
*  Func. Name  : spawn
*  Arguments   : turtlesim::Pose pose
*                  for setting the pose of the new spawned turtle
*                string name
*                  set the name of the new turtle
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void Turtle::spawn(turtlesim::Pose pose, string name) {
  turtlesim::Spawn::Request spawn_req;        // need a req. and resp. for spawn
  turtlesim::Spawn::Response spawn_resp; 


  spawn_req.x = pose.x;
  spawn_req.y = pose.y;
  spawn_req.theta = pose.theta;
  spawn_req.name = name;
  turtle_name = name; // private variable
  ros::service::waitForService("/spawn", ros::Duration(10));
  bool success = spawn_client.call(spawn_req, spawn_resp);
  std:: cout << "the service is " << success << std::endl;
  if(success)
    ROS_INFO_STREAM("Spawned turtle " << spawn_resp.name);
  else
    ROS_ERROR_STREAM("Failed to spawn turtle " << spawn_resp.name);
}

/*
*******************************************************************************
*
*                           DISABLE/ENABLE THE PEN
*
*  Description : Disable or enable the pen of the turtle 
*  Func. Name  : penOff
*  Arguments   : bool off
*                  true/false = pen is disabled/enabled 
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void Turtle::penOff(string name, bool off) {
    turtlesim::SetPen set_pen;
    set_pen.request.off = off;
    ros::service::call("/" + name + "/set_pen", set_pen);
}

/*
*******************************************************************************
*
*          KILL A TURTLE AT GIVEN A POSE AS AN ARGUMENT
*
*
*  Description : Kill a turtle by remembering its name
*  Func. Name  : killTurtle
*  Arguments   : None 
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void Turtle::kill(void) {
  turtlesim::Kill::Request kill_req;        // need a req. and resp. for kill
  turtlesim::Kill::Response kill_resp; 
   
   kill_req.name = turtle_name;
   ros::service::waitForService("/kill", ros::Duration(5));
   bool success = kill_client.call(kill_req, kill_resp);
   if(success){
       ROS_INFO_STREAM("Killed turtle" << kill_req.name);
   }else{
       ROS_ERROR_STREAM("Failed to kill turtle " << kill_req.name);
   } 
}

/*
*******************************************************************************
*
*          KILL A TURTLE AT GIVEN A POSE AS AN ARGUMENT
*
*
*  Description : Kill a turtle by naming it
*  Func. Name  : killTurtle
*  Arguments   : string name
*                  the name of a turtle to kill 
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void Turtle::kill(string name) {
  turtlesim::Kill::Request kill_req;        // need a req. and resp. for kill
  turtlesim::Kill::Response kill_resp; 
   
   kill_req.name = name;
   ros::service::waitForService("/kill", ros::Duration(5));
   bool success = kill_client.call(kill_req, kill_resp);
   if(success){
       ROS_INFO_STREAM("Killed turtle" << kill_req.name);
   }else{
       ROS_ERROR_STREAM("Failed to kill turtle " << kill_req.name);
   } 
}
