#include "PFNav.h"

PFNav::PFNav(){
  movementGain=.2;
  potentialField::attractivePoint p {0,0,0};
  navField.addPoint(p);
}

void PFNav::spin(){
  
  
  ros::Rate loop_rate( 10 );
  while(ros::ok()){
    moveTowardTarget();		// continuously move toward target.
    if(navField.distFromAttractivePoint(0,getRobotinoPoint())<.05) break;	// when we are near the targget break!
    ros::spinOnce();
    loop_rate.sleep();
  }
  return;
  
}

void PFNav::tileGridObstacle(float x,float y, float w, float d){
  float scale=0.2794;
  x=x*scale;
  y=y*scale;
  d=d*scale;
  potentialField::repulsivePoint p {x,y,w,d};
  
  navField.addPoint(p);
  return;
}

void PFNav::tileGridGoal(float x,float y, float w){
  float scale=0.2794;
  x=x*scale;
  y=y*scale;
  potentialField::attractivePoint p {x,y,w};
  
  navField.setPoint(0,p);
  return;
}


void PFNav::moveTowardTarget(){
  std::vector<float> grad;
  grad=navField.netGradNormalized(getRobotinoPoint());		// find the normalized vector of the grad
  setRobotinoGlobalVel(movementGain * grad[0],movementGain * grad[1],targetPhi(0));	// move in the direction of the grad
  // a printout so we can see what is going on
  ROS_INFO("\n\n     curent direction: %f,%f \n     position: %f,%f\n     Target Dist: %f\n",grad[0],grad[1],getRobotinoPoint().x,getRobotinoPoint().y,navField.distFromAttractivePoint(0,getRobotinoPoint()));
  return;
}


void PFNav::setPotentialField(potentialField field){
  navField=field;
}