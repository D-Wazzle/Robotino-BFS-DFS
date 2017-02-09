/*
 * 	approach_grab.h
 * 		A program designed to approach a colored object in a robotino camera vield of view
 * 		and then grab it.
 *
 * 	Developed by: Kristopher Krasnosky
 * 	Organization: DASL
 * 	Created on:   10/28/2013
 * 
 */


#ifndef APPROACH_GRAB_H
#define APPROACH_GRAB_H

#include <ros/ros.h>
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


class approach_grab{
public:
  approach_grab();
  void spin();				// main operational loop of the class
  void spinOnce();			// only dones one spin loop
  void updateDistToTarget();		//  recalculate the distToTarget
  void setRobotinoVelocity(float vx, float vy, float omega);	//  sends a velocity message to robotino
  bool moveTowardTarget();		// takes one step toward target
  
  struct pose{
    float array[6];
    bool gripperClosed;
  } BHAPose;
  
  void setBHAPose(pose myPose);   	// set the bhapose using a pose object 
  void setBHAPose(float x0,float x1,float x2,float x3,float x4,float x5, bool gripper); // set bha pose using primitives
  void sendPoseMsg();			// send the pose message.

private:
  
  /*
   *	callback functions
   */
  
  // called when the target object's position is recieved
  void targetPosCallback(const std_msgs::Int32MultiArray::ConstPtr& array);   
  
  
  /*
   *	publishers/subscribers
   */
  ros::NodeHandle n;
  ros::Subscriber obj_sub;
  ros::Publisher vel_pub;
  ros::Publisher bha_pub;
  ros::Publisher gripper_pub;
  
  /*
   * 	data
   */
  
  // the folowing are all in units of pixels
  int targetPos[2];  		// the position of the target object
  int desiredPos[2]; 		// the position desired for pickup
  int desiredPosError[2];	// the maximum error allowed for pickup
  int distToTarget[2];		// the distance to target pos
  int turnLeftTol;		// how far left  the ball is able to go on the screen before we take action
  int turnRightTol;		// how far right the ball is able to go on the screen before we take action
  // scaling quantities 
  float posGain;		// multiplier for proportinoal control

};


#endif
