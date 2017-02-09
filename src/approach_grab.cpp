#include "approach_grab.h"


/*
 * ==================
 * public Functions
 * ==================
 */

// constructor:  initialized the subscribers and the listeners
approach_grab::approach_grab(){
  ros::NodeHandle n;
  obj_sub = n.subscribe("/obj_pos", 1, &approach_grab::targetPosCallback, this);
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  bha_pub = n.advertise<std_msgs::Float32MultiArray>("/bha_pose", 1, true);
  gripper_pub = n.advertise<std_msgs::Bool>("/bha_gripper", 1, true);
  
  // set default valuse for data
  
  targetPos[0]=-1;   	// object not found yet
  targetPos[1]=-1;	// object not found yet
  
  
  desiredPos[0]=115;	// default location
  desiredPos[1]=165;	//
  
  desiredPosError[0]=2;	// minimum acceptable error is 2 pixels
  desiredPosError[1]=2;	//
  
  distToTarget[0]=0;
  distToTarget[1]=0;
  
  turnLeftTol=75;	//px
  turnRightTol=265;	//px
  
  posGain=.001;

  setBHAPose(0.591398, 0.522972, 0.532747, 0.796676, 0.821114, 0.904203,false);
}

void approach_grab::spin(){
  
  ros::Rate loop_rate( 10 );
  
  while(ros::ok()){
    spinOnce();
    // hold it up for 10 secconds
    ros::Duration(10).sleep();
    
    ros::spinOnce();
    loop_rate.sleep();
    
  }// end while
}

void approach_grab::spinOnce(){
      //setRobotinoVelocity(0,0,0);
    ros::Rate loop_rate( 10 );
    setBHAPose(0.591398, 0.522972, 0.532747, 0.796676, 0.821114, 0.904203,false);
    
    ROS_INFO("dist to target %i,%i",distToTarget[0],distToTarget[1]);
    float omega;
    if(distToTarget[1]<=0)
      omega=.5;
    else
      omega=-.5;
    //  wait for the claw to open and for the ball to roll away.
    ros::Duration(5).sleep();
    // reset the dist to target for a new loop iteration
    distToTarget[0]=0;
    distToTarget[1]=0;
    ROS_INFO("dist to target %i,%i",distToTarget[0],distToTarget[1]);
    ROS_INFO("Looking For Target Object");
    
    // if target is not found rotate to look for it.
    while(distToTarget[0]==0||distToTarget[1]==0){
      setRobotinoVelocity(0,0,omega);      //rotate to look for ball
      sendPoseMsg();
      ros::spinOnce();
      loop_rate.sleep();
    }
    // move toward target until it is within tolerances
    ROS_INFO("moving toward target");
    while(moveTowardTarget()){
      //ROS_INFO("dist to target %i,%i",distToTarget[0],distToTarget[1]);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("Target Approached. Error in pixels %i,%i",distToTarget[0],distToTarget[1]);
    ROS_INFO("Starting Pickup.");   
    
    //
    setBHAPose(0.803812,0.459433,0.498534, 0.982405, 0.694037,0.855327, false);
    
    ros::spinOnce();
    loop_rate.sleep();
    
    ros::Duration(5).sleep();
    ROS_INFO("Object Grabbed");
    
    BHAPose.gripperClosed=true;
    sendPoseMsg();
    
    ros::Duration(2).sleep();
   
    //lift up object
    ROS_INFO("lifting object"); 
    setBHAPose(0.591398, 0.522972, 0.532747, 0.796676, 0.821114, 0.904203,true);
    
    
}

void approach_grab::updateDistToTarget(){
  
  distToTarget[0] = desiredPos[0]-targetPos[0];	//update x coord
  distToTarget[1] = desiredPos[1]-targetPos[1];	//update y coord
  //ROS_INFO("dist to target %i,%i",distToTarget[0],distToTarget[1]);
  return;
}

void approach_grab::setRobotinoVelocity(float vx, float vy, float omega){
  geometry_msgs::Twist cmd_vel_msg;
    
  cmd_vel_msg.linear.x 		= vx;
  cmd_vel_msg.linear.y  	= vy;
  cmd_vel_msg.angular.z 	= omega;
  
  vel_pub.publish(cmd_vel_msg);
}


bool approach_grab::moveTowardTarget(){
  float vx=0;
  float vy=0;
  float omega=0;
  
  // if we see the target in between the left and right tolerances we apprach the object.
  if(targetPos[0]>turnLeftTol && targetPos[0]<turnRightTol){
    // use a simple proportinoal controller to set the velocites 
    if(abs(distToTarget[1])>desiredPosError[1])  vx=distToTarget[1]*posGain;
    if(abs(distToTarget[0])>desiredPosError[0])  vy=distToTarget[0]*posGain;
  }
  
  // if the object is to the left of our tolerance zone turn left to look for it
  if(targetPos[0]<turnLeftTol){
    omega=.5;
  }
  // if the object is to the right of our tolerance zone turn right to look for it
  if(targetPos[0]>turnRightTol){
    omega=-.5;
  }
  
  // set the velocity of the robotino
  setRobotinoVelocity(vx,vy,omega);
  
  // we want to return ture while we are still approaching the ball.
  return (abs(distToTarget[0])>desiredPosError[0] || (distToTarget[1])>desiredPosError[1]);  // return true while we still need to move
}

void approach_grab::setBHAPose(pose myPose){
  BHAPose=myPose;
  sendPoseMsg();
}


void approach_grab::setBHAPose(float x0,float x1,float x2,float x3,float x4,float x5, bool gripper){
  pose p = { { x0,x1,x2,x3,x4,x5}, gripper};
  setBHAPose(p);
}

void approach_grab::sendPoseMsg(){
  
  // declare our messages
  std_msgs::Float32MultiArray msg;
  std_msgs::Bool gripmsg;
  
  // prepare our pose message by setting the pose data from our pose object.
  for (int i = 0; i < 6; i++){
	msg.data.push_back(BHAPose.array[i]);
  }
  
  // prepare a message for the gripper state
  gripmsg.data=BHAPose.gripperClosed;
  
  // publish our messages
  gripper_pub.publish(gripmsg);
  bha_pub.publish(msg);
}


/*
 * ==================
 * callback functions
 * ==================
 */

void approach_grab::targetPosCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	int i = 0;
	// print all the remaining numbers
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		targetPos[i] = *it;
		i++;
	}
	
	updateDistToTarget();
	//ROS_INFO("Pos recieved %i,%i",targetPos[0],targetPos[1]);   // debuging
	return;
}
