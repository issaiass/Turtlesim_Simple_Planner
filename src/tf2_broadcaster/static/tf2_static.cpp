/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : tf2_static.cpp
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   : A simple static broadcaster class functions definitions.
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

#include <tf2_broadcaster/static/tf2_static.hpp>

/*
*******************************************************************************
*                                   FUNCTIONS
*******************************************************************************
*/

/*
*******************************************************************************
*
*                  CONSTRUCTOR OF THE TF2 STATIC BROADCASTER
*
*
*  Description : Initialize the broadcaster with teh parend and child frames
*  Func. Name  : TF2SB
*  Arguments   : string parent
*                    the parent frame name
*                string child
*                    the child frame name
*  Returns     : None
*  Notes       : Initialization of parend and child private variables
*******************************************************************************
*/

TF2SB::TF2SB(string parent, string child) :
  _parent(parent), 
  _child(child) 
{ 

}

/*
*******************************************************************************
*
*                                 SET THE FRAME 
*
*
*  Description : Same function as the initializer to change in any moment the
*                frame to do the transformation
*  Func. Name  : setFrame
*  Arguments   : string parent
*                    the parent frame name
*                string child
*                    the child frame name
*  Returns     : None
*  Notes       : Initialization of parend and child private variables
*******************************************************************************
*/

void TF2SB::setFrame(string parent, string child)
{
  _parent = parent;
  _child = child;
}

/*
*******************************************************************************
*
*                  CONSTRUCTOR OF THE TF2 STATIC BROADCASTER
*
*
*  Description : Initialize the broadcaster with teh parend and child frames
*  Func. Name  : broadcast
*  Arguments   : geometry_msgs::Vector3 tr
*                    translation vector3 message to pass to TransformStamped
*                geomery_msgs:Vector3 rot
*                    rotation vector3 message to pass to TransformStamped
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void TF2SB::broadcast(geometry_msgs::Vector3 tr, geometry_msgs::Vector3 rot) {
  _static_transformStamped.header.stamp = ros::Time::now();
  _static_transformStamped.header.frame_id = _parent;
  _static_transformStamped.child_frame_id = _child;
  _static_transformStamped.transform.translation.x = tr.x;
  _static_transformStamped.transform.translation.y = tr.y;
  _static_transformStamped.transform.translation.z = tr.z;
  tf2::Quaternion q;
  q.setRPY(rot.x, rot.y, rot.z);
  _static_transformStamped.transform.rotation.x = q.x();
  _static_transformStamped.transform.rotation.y = q.y();
  _static_transformStamped.transform.rotation.z = q.z();
  _static_transformStamped.transform.rotation.w = q.w();
  _static_broadcaster.sendTransform(_static_transformStamped);
}

/*
*******************************************************************************
*
*                  VECTOR3 STRING TO VECTOR3 FLOAT CONVERSION
*
*
*  Description : Convert the pointer of pointer vector3 string to floats 
*                for translation
*  Func. Name  : broadcast
*  Arguments   : char **argv
*                    pointer of pointers
*                    val[2] = .x
*                    val[3] = .y
*                    val[4] = .z
*  Returns     : geometry_msgs::Vector3
*                    the message in format to pass to the TransformStamped
*                    broadcater
*  Notes       : None
*******************************************************************************
*/

geometry_msgs::Vector3 TF2SB::tr2Vector3(char **argv) {
  geometry_msgs::Vector3 tr;


  tr.x = atof(argv[2]);
  tr.y = atof(argv[3]);
  tr.z = atof(argv[4]);
  return tr;
}

/*
*******************************************************************************
*
*                  VECTOR3 STRING TO VECTOR3 FLOAT CONVERSION
*
*
*  Description : Convert the pointer of pointer vector3 string to floats
*                for rotation 
*  Func. Name  : broadcast
*  Arguments   : char **argv
*                    pointer of pointers
*                    val[5] = roll
*                    val[6] = pitch
*                    val[7] = yaw
*  Returns     : geometry_msgs::Vector3
*                    the message in format to pass to the TransformStamped
*                    broadcater
*  Notes       : None
*******************************************************************************
*/

geometry_msgs::Vector3 TF2SB::rot2Vector3(char **argv) {
  geometry_msgs::Vector3 rot;


  rot.x = atof(argv[5]);
  rot.y = atof(argv[6]);
  rot.z = atof(argv[7]);
  return rot;
}