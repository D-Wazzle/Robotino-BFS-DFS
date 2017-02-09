#include "BFNavigator.h"


void BFNavigator::navigatePath(){
  pathfinder.start(x_,y_);  // start from our current point
  ros::Rate loop_rate( 10 );
  float nextX,nextY;
  for (int i=0; i<pathfinder.path.size(); i++) {
    ROS_INFO("move to x: %d y: %d",pathfinder.path[i][0],pathfinder.path[i][1]);
    nextX = pathfinder.path[i][0]*pathfinder.getMapScale();
    nextY = pathfinder.path[i][1]*pathfinder.getMapScale();
    
    while(moveToPos(nextX,nextY,0 , 0.1)){
	//ROS_INFO("vx: %f",targetX(1));
	ros::spinOnce();
	loop_rate.sleep();
     } // endwhile
  }

}
