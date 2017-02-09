/*
 * 	robotinoNavigator.h
 * 		Basic robotion controlls common to most navigator classes.  
 * 		this class is designed to be inherited
 *
 * 	Developed by: Kristopher Krasnosky
 * 	Organization: DASL
 * 	Created on:   11/5/2013
 * 
 */


#ifndef robotinoNavigator_H
#define robotinoNavigator_H

#include <ros/ros.h>
#include <cmath>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "potentialField.h"


#include <robotino_msgs/ResetOdometry.h>


class robotinoNavigator{
public:
  robotinoNavigator();
  void spin(){return;};						// designed to be inherited
  void setRobotinoVelocity(float vx, float vy, float omega);	//  sends a velocity message to robotino
  void setRobotinoGlobalVel(float vx,float vy, float omega);	// sets the velocity of the robotion in global reference frame
  
  float magnitude(std::vector<float> vect);			// find the magnitued of a vector
  std::vector<float> normalize(std::vector<float> vect);	// normalize a vector
  
  struct pose{
    float array[6];
    bool gripperClosed;
  } BHAPose;
  
  void setBHAPose(pose myPose);   	// set the bhapose using a pose object 
  void setBHAPose(float x0,float x1,float x2,float x3,float x4,float x5, bool gripper); // set bha pose using primitives
  void sendPoseMsg();			// send the pose message.
  
  float targetPhi(float angle);		// returns a velocity to hold orientation at angle
  float targetX(float x);
  float targetY(float y);
  
  bool moveToPos(float x,float y,float phi, float tolerance);
  
  bool setOdom(float x, float y, float phi);

  
  // adjustable parameters
  float targetPhiGain;			// gain for proportional controll on in targetPhi()
  float linearGain;
  float minimumLinearVelocity;		// x or y velocity below this setting will be set to 0
  float minimumAngularVelocity;	// anguluar velocity below this setting will be set to 0
  float maximumLinearVelocity;
  
  potentialField::point getRobotinoPoint();
  


protected:
  /*
   * 	data
   */
  double x_,y_,phi_;			// the position of the robotino
  
  /*
   *	callback functions
   */
   void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  
  
  /*
   *	publishers/subscribers
   */
  ros::NodeHandle n;
  ros::Publisher vel_pub;
  ros::Publisher bha_pub;
  ros::Publisher gripper_pub;
  ros::Subscriber odom_sub;
  
  /*
   * 	services
   */
  
  ros::ServiceClient odom_reset;

  
};


#endif
