/*
 * 	PFNavNode.cpp
 * 		Implemetnation and demostration of PFNav
*
 * 	Developed by: Kristopher Krasnosky
 * 	Organization: DASL
 * 	Created on:   11/5/2013
 * 
 */





//#include <iostream>
#include "PFNav.h"
#include "approach_grab.h"


int main(int argc, char **argv)
{
   ros::init(argc, argv, "PFNavNode");

    PFNav navigator;
    approach_grab mygrab;
    
    // preapare some points and add them to the potential field.
    
    navigator.tileGridGoal(10,2,10);
    navigator.tileGridObstacle(4,2,20,2);
    navigator.tileGridObstacle(6,-2,20,2);
    navigator.tileGridObstacle(8,3,20,2);
    navigator.navField.printPotentialField();

    
    navigator.setOdom(0,0,0);				// set the odometer to 0
    navigator.spin();					// start running!
    mygrab.spinOnce();
    navigator.tileGridGoal(0,0,10);
    navigator.spin();					// Run again!
    
    
    return 0;
}