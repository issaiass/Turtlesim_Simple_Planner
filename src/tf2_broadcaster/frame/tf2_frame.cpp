/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : tf2_frame.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : A simple frame broadcaster to /tf function definitions.
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

#include <tf2_broadcaster/frame/tf2_frame.hpp>

/*
*******************************************************************************
*                                   FUNCTIONS
*******************************************************************************
*/

/*
*******************************************************************************
*
*                         TF2 DYNAMIC FRAME CONSTRUCTOR
*
*
*  Description : Frame constructor
*  Func. Name  : TF2Frame
*  Arguments   : None
*  Returns     : None
*  Notes       : Initialization listener with the buffer
*******************************************************************************
*/

TF2Frame::TF2Frame() : tfListener(tfBuffer)
{ 

}

/*
*******************************************************************************
*
*                           THE FUNCTION DESTRUCTOR
*
*
*  Description : Destructs this object
*  Func. Name  : ~TF2Frame
*  Arguments   : None
*  Returns     : None
*  Notes       : virtual function
*******************************************************************************
*/

TF2Frame::~TF2Frame() { }

/*
*******************************************************************************
*
*         LOOKUP FOR THE TRANSFORM BETWEEN SOURCE AND DESTINATION FRAMES
*
*
*  Description : looks the transform of two frames
*  Func. Name  : lookup
*  Arguments   : string frame1
*                    the target frame 
*                string frame2
*                    the source frame
*  Returns     : geometry_msgs::TransformStamped
*                    the transform between the two frames
*  Notes       : None
*******************************************************************************
*/

geometry_msgs::TransformStamped  TF2Frame::lookup(string frame1, string frame2) {
   try {
      ts = tfBuffer.lookupTransform(frame2, frame1, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    } 
    return ts;
}