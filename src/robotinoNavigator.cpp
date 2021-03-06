#include "robotinoNavigator.h"


/*
 * ==================
 * public Functions
 * ==================
 */

// constructor:  initialized the subscribers and the listeners
robotinoNavigator::robotinoNavigator(){
  ros::NodeHandle n;
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  bha_pub = n.advertise<std_msgs::Float32MultiArray>("/bha_pose", 1, true);
  gripper_pub = n.advertise<std_msgs::Bool>("/bha_gripper", 1, true);
  
  odom_sub = n.subscribe("/odom", 1, &robotinoNavigator::odomCallback, this);
  
  odom_reset = n.serviceClient<robotino_msgs::ResetOdometry>("reset_odometry");
  
  // set default valuse for data
  setBHAPose(0.591398, 0.522972, 0.532747, 0.796676, 0.821114, 0.904203,false);
  
  targetPhiGain=2;
  linearGain=1;
  minimumLinearVelocity=.05;
  maximumLinearVelocity=.2;
}

void robotinoNavigator::setRobotinoVelocity(float vx, float vy, float omega){
  geometry_msgs::Twist cmd_vel_msg;
  
  if(vx<minimumLinearVelocity		&&	vx>-minimumLinearVelocity	) vx=0;
  if(vy<minimumLinearVelocity 		&&	vy>-minimumLinearVelocity	) vy=0;
  if(omega<minimumAngularVelocity	&& 	omega>-minimumAngularVelocity	) omega=0;
    
  cmd_vel_msg.linear.x 		= vx;
  cmd_vel_msg.linear.y  	= vy;
  cmd_vel_msg.angular.z 	= omega;
  
  vel_pub.publish(cmd_vel_msg);
  
  return;
}


float robotinoNavigator::magnitude(std::vector<float> vect){
  unsigned int i;
  float output=0;
  for (i=0; i<=vect.size(); i++){
    output=output+vect[i]*vect[i];
  }
  output=std::sqrt(output);
  return output;
}

std::vector<float> robotinoNavigator::normalize(std::vector<float> vect){
  unsigned int i;
  std::vector<float> output(vect.size(),0);
  float mag=magnitude(vect);
  for (i=0; i<=vect.size(); i++){
    output[i]=vect[i]/mag;
  }
  return output;
}

void robotinoNavigator::setBHAPose(pose myPose){
  BHAPose=myPose;
  sendPoseMsg();
}


void robotinoNavigator::setBHAPose(float x0,float x1,float x2,float x3,float x4,float x5, bool gripper){
  pose p = { { x0,x1,x2,x3,x4,x5}, gripper};
  setBHAPose(p);
}

void robotinoNavigator::sendPoseMsg(){
  
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

void robotinoNavigator::setRobotinoGlobalVel(float vx,float vy, float omega){
  float x,y;
  x=vx*cos(-phi_)-vy*sin(-phi_);
  y=vy*cos(-phi_)+vx*sin(-phi_);
  setRobotinoVelocity(x,y,omega);
}

float robotinoNavigator::targetPhi(float angle){
  float omega = targetPhiGain*(angle-phi_);
  return omega;
}

float robotinoNavigator::targetX(float x){
  float vx = linearGain*(x-x_);
  return vx;
}

float robotinoNavigator::targetY(float y){
  float vy = linearGain*(y-y_);
  return vy;
}

bool robotinoNavigator::moveToPos(float x,float y,float phi, float tolerance){
  std::vector<float> v(2,0);
  v[0]=targetX(x);
  v[1]=targetY(y);
    v=normalize(v);
    v[0]=maximumLinearVelocity*v[0];
    v[1]=maximumLinearVelocity*v[1];
  
  setRobotinoVelocity(v[0],v[1],targetPhi(phi));
  //std::cout<<((x-x_)*(x-x_) + (y-y_)*(y-y_))<<","<<tolerance*tolerance<<std::endl;
  bool output =((x-x_)*(x-x_) + (y-y_)*(y-y_))  >  tolerance*tolerance;
  return  output;
}

bool robotinoNavigator::setOdom(float x, float y, float phi){
  robotino_msgs::ResetOdometry serv;
  
  serv.request.x=x;
  serv.request.y=y;
  serv.request.phi=phi;
  
  
  if(odom_reset.call(serv))  	return true;
  else 				return false;
  
  
  
}

potentialField::point robotinoNavigator::getRobotinoPoint(){
    potentialField::point output {x_,y_};
    return output;
    
    
}



/*
 *	Callback funcitons 
 */


void robotinoNavigator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  x_=msg->pose.pose.position.x;
  y_=msg->pose.pose.position.y;
  phi_=tf::getYaw(msg->pose.pose.orientation);
  
  //std::cout<< phi_ << std::endl;
  
  return;
}

