/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : turtle.hpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : A turtle class for easy manipulation.
*  ----------------------------------------------------------------------------
*  HISTORY
*  MM DD YY
*  9 26 20 Created.
*  9 28 20 Added comments
*  9 31 20 Documented
*******************************************************************************
*/

/*
*******************************************************************************
*                     NECCESARY HEADER FILES FOR THE APPLICATION
*******************************************************************************
*/     

#include <ros/ros.h>            // ros
#include <turtlesim/Pose.h>     // pose to set
#include <turtlesim/Spawn.h>    // for position a turtle in the field
#include <turtlesim/SetPen.h>   // enable/disable turtle traceability
#include <turtlesim/Kill.h>     // kill the turtle

/*
*******************************************************************************
*                                NAMESPACES
*******************************************************************************
*/

using namespace std;

/*
*******************************************************************************
*                               TURTLE CLASS
*******************************************************************************
*/

class Turtle {
  public:
    // the constructor by giving the node handler
    Turtle(ros::NodeHandle &nh);      
    // spawns a new turtle by giving the pose and anme
    void spawn(turtlesim::Pose pose, string name);
    // disable the pen of a named turtle
    void penOff(string name, bool off); 
    // kills the current turtle
    void kill(void);           
    // kills another turtle by name       
    void kill(string name);           

  private:
    ros::NodeHandle &n;               // node handler for operations  
    ros::ServiceClient spawn_client;  // spawn client
    ros::ServiceClient kill_client;   // kill client
    string turtle_name;               // the turtle name
};